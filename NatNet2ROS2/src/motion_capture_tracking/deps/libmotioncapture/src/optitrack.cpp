#include "libmotioncapture/optitrack.h"
#include "libmotioncapture/natnet_clock_sync.h"

#include <algorithm>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/time.h>

using boost::asio::ip::udp;

// Source - https://stackoverflow.com/a/3312896
// Posted by Steph, modified by community. See post 'Timeline' for change history
// Retrieved 2026-02-21, License - CC BY-SA 4.0

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif


namespace libmotioncapture {

  constexpr int MAX_PACKETSIZE = 65503; // max size of packet (actual packet size is dynamic)
  constexpr int MAX_NAMELENGTH = 256;

  constexpr int NAT_CONNECT           = 0;
  constexpr int NAT_SERVERINFO        = 1;
  constexpr int NAT_REQUEST_MODELDEF  = 4;
  constexpr int NAT_MODELDEF          = 5;
  constexpr int NAT_KEEPALIVE         = 10;
  constexpr int NAT_ECHOREQUEST       = 12;
  constexpr int NAT_ECHORESPONSE      = 13;

  // HOPE patch (2026-07-30; verified against Motive 3.1.0.4 / NatNet 4.1 at the
  // venue): Motive silently DROPS a payload-less NAT_REQUEST_MODELDEF -- 6/6
  // requests went unanswered. It replies only when a 4-byte descriptor-type
  // bitmask follows the 4-byte header. Masks with undefined bits set (0x7f,
  // 0xff, ~0) are dropped too, so request exactly the two descriptor types
  // parseModelDef() consumes. Measured: 0x1 -> 3 datasets, 0x2 -> 2 datasets
  // (Ball, P1), 0x3 -> 5 datasets / 1083 B.
  constexpr int32_t MODELDEF_TYPES    = 0x3;  // bit0 MarkerSet | bit1 RigidBody

  // Upper bound on each connect-time handshake receive. Before this patch both
  // waits were unbounded blocking receives, so the unanswered MODELDEF above
  // deadlocked the constructor: the node never reached create_publisher(), and
  // every HOPE topic stayed silent with no error logged anywhere.
  constexpr int HANDSHAKE_TIMEOUT_S   = 5;
  constexpr int HANDSHAKE_ATTEMPTS    = 3;

  // NatNet's echo protocol is the clock-domain bridge used by the official
  // NatNetClient::SecondsSinceHostTimestamp API.  The request carries an
  // opaque client token; Motive echoes it and appends its QPC tick at request
  // reception.  Cristian's midpoint estimate then maps Motive QPC ticks into
  // this client's steady-clock domain without involving either host's wall
  // clock.
  constexpr int CLOCK_SYNC_INITIAL_SAMPLES = 20;
  constexpr int CLOCK_SYNC_MIN_SAMPLES = 5;
  constexpr int CLOCK_SYNC_MAX_ATTEMPTS = 60;
  constexpr int CLOCK_SYNC_REPLY_TIMEOUT_MS = 100;
  constexpr double CLOCK_SYNC_ACCEPT_SLOP_S = 0.00025;  // 0.25 ms over min RTT
  constexpr double CLOCK_SYNC_DRIFT_BOUND = 100e-6;     // conservative 100 ppm
  constexpr double CLOCK_SYNC_UPDATE_ALPHA = 0.2;
  // Runtime echoes arrive every 500 ms. Ten consecutive valid echoes above
  // the old floor allow recovery in about five seconds after a permanent
  // route/link RTT change, while transient congestion remains rejected.
  constexpr std::size_t CLOCK_SYNC_RTT_REGIME_RECOVERY_REJECTIONS = 10;

  PACK(struct sModelDefRequest {
    uint16_t iMessage;
    uint16_t nDataBytes;
    int32_t types;
  });

  PACK(struct sEchoRequest {
    uint16_t iMessage;
    uint16_t nDataBytes;
    uint64_t requestTimestamp;
  });

  /**
   * \brief Unpack number of bytes of data for a given data type. 
   * Useful if you want to skip this type of data. 
   * \param ptr - input data stream pointer
   * \param major - NatNet major version
   * \param minor - NatNet minor version
   * \return - pointer after decoded object
  */
  char* UnpackDataSize(char* ptr, int major, int minor, int& nBytes, bool skip = false )
  {
      nBytes = 0;

      // size of all data for this data type (in bytes);
      if (((major == 4) && (minor > 0)) || (major > 4))
      {
          memcpy(&nBytes, ptr, 4); ptr += 4;
          // printf("Byte Count: %d\n", nBytes);
          if (skip)
          {
              ptr += nBytes;
          }
      }
      return ptr;
  }

  uint64_t ticksToMicroseconds(uint64_t ticks, uint64_t frequency)
  {
    if (frequency == 0) {
      return 0;
    }
    // Divide before multiplying the whole-second term.  The old
    // ticks * 1e6 / frequency expression overflowed uint64_t after roughly
    // three weeks of uptime for a 10 MHz QPC.
    return (ticks / frequency) * 1000000ULL +
           ((ticks % frequency) * 1000000ULL) / frequency;
  }

  class MotionCaptureOptitrackImpl{
  public:
    MotionCaptureOptitrackImpl()
      : version()
      , versionMajor(0)
      , versionMinor(0)
      , io_context()
      , socket(io_context)
      , sender_endpoint()
      , data(MAX_PACKETSIZE)
      , incoming_data(MAX_PACKETSIZE)
      , clock_mapping(
          CLOCK_SYNC_MIN_SAMPLES,
          CLOCK_SYNC_ACCEPT_SLOP_S,
          CLOCK_SYNC_DRIFT_BOUND,
          CLOCK_SYNC_UPDATE_ALPHA,
          CLOCK_SYNC_RTT_REGIME_RECOVERY_REJECTIONS)
      , clock_sync_enabled(false)
      , pending_echo(false)
      , pending_echo_token(0)
      , cameraMidExposureTimestamp(0)
      , transmitTimestamp(0)
    {
    }

    static double steadySeconds(std::chrono::steady_clock::time_point value)
    {
      return std::chrono::duration<double>(value.time_since_epoch()).count();
    }

    static uint64_t steadyNanoseconds(std::chrono::steady_clock::time_point value)
    {
      return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          value.time_since_epoch()).count());
    }

    bool decodeEchoResponse(
      const char* packet,
      size_t length,
      uint64_t expected_token,
      uint64_t& server_receive_ticks) const
    {
      if (length < 20) {
        return false;
      }
      uint16_t message_id = 0;
      uint16_t payload_size = 0;
      uint64_t echoed_token = 0;
      memcpy(&message_id, packet, sizeof(message_id));
      memcpy(&payload_size, packet + 2, sizeof(payload_size));
      memcpy(&echoed_token, packet + 4, sizeof(echoed_token));
      memcpy(&server_receive_ticks, packet + 12, sizeof(server_receive_ticks));
      return message_id == NAT_ECHORESPONSE && payload_size >= 16 &&
             echoed_token == expected_token;
    }

    bool receiveInitialClockSample(detail::NatNetClockSample& sample)
    {
      const auto send_time = std::chrono::steady_clock::now();
      const uint64_t token = steadyNanoseconds(send_time);
      const sEchoRequest request = {
        NAT_ECHOREQUEST, sizeof(uint64_t), token
      };
      boost::system::error_code send_ec;
      socket.send_to(
        boost::asio::buffer(&request, sizeof(request)), cmd_endpoint, 0, send_ec);
      if (send_ec) {
        return false;
      }

      const auto deadline =
        send_time + std::chrono::milliseconds(CLOCK_SYNC_REPLY_TIMEOUT_MS);
      while (true) {
        boost::system::error_code receive_ec;
        const size_t length = socket.receive_from(
          boost::asio::buffer(incoming_data.data(), incoming_data.size()),
          sender_endpoint,
          0, receive_ec);
        const auto receive_time = std::chrono::steady_clock::now();
        if (receive_ec) {
          return false;
        }
        if (receive_time >= deadline) {
          return false;
        }
        uint64_t server_receive_ticks = 0;
        if (decodeEchoResponse(
              incoming_data.data(), length, token, server_receive_ticks)) {
          sample.local_send_seconds = steadySeconds(send_time);
          sample.local_receive_seconds = steadySeconds(receive_time);
          sample.server_receive_seconds =
            server_receive_ticks / static_cast<double>(clockFrequency);
          return true;
        }
        // A unicast Motive also streams frames to this data socket.
        // Bound the echo wait by elapsed time, not merely SO_RCVTIMEO, or a
        // continuous frame stream could prevent the socket timeout forever.
      }
    }

    bool initializeClockSync()
    {
      if (!clock_sync_enabled) {
        return false;
      }
      if (clockFrequency == 0) {
        throw std::runtime_error(
          "NatNet reported HighResClockFrequency=0; cannot synchronize clocks");
      }

      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = CLOCK_SYNC_REPLY_TIMEOUT_MS * 1000;
      if (::setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                       &timeout, sizeof(timeout)) != 0) {
        throw std::runtime_error(
          "Failed to set NatNet clock-sync receive timeout");
      }

      std::vector<detail::NatNetClockSample> samples;
      samples.reserve(CLOCK_SYNC_INITIAL_SAMPLES);
      for (int attempt = 0;
           attempt < CLOCK_SYNC_MAX_ATTEMPTS &&
             static_cast<int>(samples.size()) < CLOCK_SYNC_INITIAL_SAMPLES;
           ++attempt) {
        detail::NatNetClockSample sample;
        if (receiveInitialClockSample(sample) && sample.rtt() >= 0.0) {
          samples.push_back(sample);
        }
      }
      if (!clock_mapping.initialize(samples)) {
        return false;
      }
      last_echo_sent = std::chrono::steady_clock::now();

      // Restore blocking frame reception after the bounded startup exchange.
      timeout.tv_sec = 0;
      timeout.tv_usec = 0;
      if (::setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                       &timeout, sizeof(timeout)) != 0) {
        throw std::runtime_error(
          "Failed to restore blocking NatNet frame reception");
      }
      return true;
    }

    void updateClockMapping(
      std::chrono::steady_clock::time_point send_time,
      std::chrono::steady_clock::time_point receive_time,
      uint64_t server_receive_ticks)
    {
      detail::NatNetClockSample sample{
        steadySeconds(send_time),
        steadySeconds(receive_time),
        server_receive_ticks / static_cast<double>(clockFrequency)
      };
      if (clock_mapping.update(sample) &&
          clock_mapping.lastUpdateRebasedRttFloor()) {
        std::cerr
          << "[optitrack] NatNet clock sync recovered after a persistent RTT "
          << "regime change: new floor "
          << clock_mapping.minimumRttSeconds() * 1e3
          << " ms, uncertainty "
          << clock_mapping.baseUncertaintySeconds() * 1e3 << " ms"
          << std::endl;
      }
    }

    void sendRuntimeEcho(std::chrono::steady_clock::time_point now)
    {
      pending_echo_token = steadyNanoseconds(now);
      const sEchoRequest request = {
        NAT_ECHOREQUEST, sizeof(uint64_t), pending_echo_token
      };
      boost::system::error_code ec;
      socket.send_to(
        boost::asio::buffer(&request, sizeof(request)), cmd_endpoint, 0, ec);
      last_echo_sent = now;
      pending_echo_sent = now;
      pending_echo = !ec;
    }

    void pollClockSync()
    {
      if (!clock_sync_enabled || !clock_mapping.valid()) {
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      if (pending_echo &&
          now - pending_echo_sent >
            std::chrono::milliseconds(CLOCK_SYNC_REPLY_TIMEOUT_MS)) {
        pending_echo = false;
      }
      if (!pending_echo &&
          now - last_echo_sent > std::chrono::milliseconds(500)) {
        sendRuntimeEcho(now);
      }
    }

    double timestampAge() const
    {
      return clock_mapping.timestampAgeSeconds(
        cameraMidExposureTimestamp, clockFrequency,
        steadySeconds(std::chrono::steady_clock::now()));
    }

    double timestampAgeUncertainty() const
    {
      return clock_mapping.uncertaintySeconds(
        steadySeconds(std::chrono::steady_clock::now()));
    }
    // void getObjectByRigidbody(
    //   const RigidBody& rb,
    //   Object& result) const
    //   {
    //     std::stringstream sstr;
    //     sstr << rb.id();
    //     const std::string name_number = sstr.str();
    //     std::string name_cf = "cf";
    //     const std::string name = name_cf + name_number;

    //     auto const translation = rb.location();
    //     auto const quaternion = rb.orientation();

    //     if(rb.trackingValid()) {
    //         Eigen::Vector3f position(
    //           -translation.y,     
    //           translation.x,
    //           translation.z);

    //         Eigen::Quaternionf rotation(
    //           quaternion.qw,
    //           -quaternion.qy,
    //           quaternion.qx,
    //           quaternion.qz
    //           );

    //         result = Object(name, position, rotation);

    //     } else {
    //         result = Object(name);
    //     }
    //   } 

    void parseModelDef(const char* data)
    {
      const char *ptr = data;
      int major = versionMajor;
      int minor = versionMinor;

      // First 2 Bytes is message ID
      int MessageID = 0;
      memcpy(&MessageID, ptr, 2); ptr += 2;
      // printf("Message ID : %d\n", MessageID);

      // Second 2 Bytes is the size of the packet
      int nBytes = 0;
      memcpy(&nBytes, ptr, 2); ptr += 2;
      // printf("Byte count : %d\n", nBytes);

      if(MessageID == NAT_MODELDEF) // Data Descriptions
      {
        // number of datasets
        int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
        // printf("Dataset Count : %d\n", nDatasets);

        for(int i=0; i < nDatasets; i++)
        {
          // printf("Dataset %d\n", i);

          int type = 0; memcpy(&type, ptr, 4); ptr += 4;
          int description_size = 0;
          // printf("Type : %d\n", i, type);

          if ((major == 4 && minor >= 1) || major > 4)
          {
            // If the NatNet version is 4.1 or greater, next four bytes represent
            // the number of bytes in the dataset. Just skip them.
            memcpy(&description_size, ptr, 4); ptr += 4;
          }

          if(type == 0)   // markerset
          {
            ptr += strlen(ptr) + 1; // name

            // marker data
            int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
            // printf("Marker Count : %d\n", nMarkers);

            for(int j=0; j < nMarkers; j++)
            {
              ptr += strlen(ptr) + 1;
            }
          }
          else if(type ==1)   // rigid body
          {
            char szName[MAX_NAMELENGTH];
            if(major >= 2)
            {
              // name
              strcpy(szName, ptr);
              ptr += strlen(ptr) + 1;
              // printf("Name: %s\n", szName);
            }

            int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
            // printf("ID : %d\n", ID);

            rigidBodyDefinitions[ID].name = szName;
            rigidBodyDefinitions[ID].ID = ID;
         
            memcpy(&rigidBodyDefinitions[ID].parentID, ptr, 4); ptr +=4;
            memcpy(&rigidBodyDefinitions[ID].xoffset, ptr, 4); ptr +=4;
            memcpy(&rigidBodyDefinitions[ID].yoffset, ptr, 4); ptr +=4;
            memcpy(&rigidBodyDefinitions[ID].zoffset, ptr, 4); ptr +=4;

            // Per-marker data (NatNet 3.0 and later)
            if ( major >= 3 )
            {
              int nMarkers = 0; memcpy( &nMarkers, ptr, 4 ); ptr += 4;
              // Marker positions
              nBytes = nMarkers * 3 * sizeof(float);
              ptr += nBytes;
              // Marker required active labels
              nBytes = nMarkers * sizeof(int);
              ptr += nBytes;
              // Marker Name
              if (major >= 4) {
                for (int markerIdx = 0; markerIdx < nMarkers; ++markerIdx) {
                  ptr += strlen(ptr) + 1;
                }
              }
            }
          }
          else if ((major == 4 && minor >= 1) || major > 4)
          {
            // We got a description_size for > 4.1, which is simpler to discard
            // for unsuported datatypes
            ptr += description_size;
          }
          else if(type ==2)   // skeleton
          {
            // char szName[MAX_NAMELENGTH];
            // strcpy(szName, ptr);
            ptr += strlen(ptr) + 1;
            // printf("Name: %s\n", szName);

            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr +=4;
            // printf("ID : %d\n", ID);

            int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
            // printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

            for(int i=0; i< nRigidBodies; i++)
            {
                if(major >= 2)
                {
                    // RB name
                    // char szName[MAX_NAMELENGTH];
                    // strcpy(szName, ptr);
                    ptr += strlen(ptr) + 1;
                    // printf("Rigid Body Name: %s\n", szName);
                }

                // int ID = 0; memcpy(&ID, ptr, 4);
                ptr +=4;
                // printf("RigidBody ID : %d\n", ID);

                // int parentID = 0; memcpy(&parentID, ptr, 4);
                ptr +=4;
                // printf("Parent ID : %d\n", parentID);

                // float xoffset = 0; memcpy(&xoffset, ptr, 4);
                ptr +=4;
                // printf("X Offset : %3.2f\n", xoffset);

                // float yoffset = 0; memcpy(&yoffset, ptr, 4);
                ptr +=4;
                // printf("Y Offset : %3.2f\n", yoffset);

                // float zoffset = 0; memcpy(&zoffset, ptr, 4);
                ptr +=4;
                // printf("Z Offset : %3.2f\n", zoffset);
            }
          }
        }   // next dataset

       // printf("End Packet\n-------------\n");

      }
    }

  public:
    // NatNetClient client;
    std::string version;
    int versionMajor;
    int versionMinor;
    uint64_t clockFrequency; // ticks/second for timestamps

    boost::asio::io_context io_context;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::vector<char> data;
    std::vector<char> incoming_data;
    // HOPE patch (see PIN.md): Motive command endpoint + keep-alive pacing for
    // unicast streaming, where Motive sends frames to the NAT_CONNECT source
    // socket and drops clients that stay silent for a few seconds.
    boost::asio::ip::udp::endpoint cmd_endpoint;
    std::chrono::steady_clock::time_point last_keepalive;
    std::chrono::steady_clock::time_point last_modeldef_request;

    // NatNet QPC -> adapter steady-clock mapping.  Wall-clock/Unix conversion
    // deliberately happens in the ROS node so it uses RCL_SYSTEM_TIME, which
    // is disciplined by Chrony on the deployed Linux adapter host.
    detail::NatNetClockMapping clock_mapping;
    bool clock_sync_enabled;
    std::chrono::steady_clock::time_point last_echo_sent;
    bool pending_echo;
    uint64_t pending_echo_token;
    std::chrono::steady_clock::time_point pending_echo_sent;
    uint64_t cameraMidExposureTimestamp;
    uint64_t transmitTimestamp;

    struct rigidBody {
      int ID;
      float x;
      float y;
      float z;
      float qx;
      float qy;
      float qz;
      float qw;
      float fError; // mean marker error
      bool bTrackingValid;
    };
    std::vector<rigidBody> rigidBodies;

    struct marker {
      float x;
      float y;
      float z;
    };
    std::vector<marker> markers;

    struct rigidBodyDefinition {
      std::string name;
      int ID;
      int parentID;
      float xoffset;
      float yoffset;
      float zoffset;
    };
    std::map<int, rigidBodyDefinition> rigidBodyDefinitions;
  };

  MotionCaptureOptitrack::MotionCaptureOptitrack(
    const std::string &hostname,
    const std::string& interface_ip,
    int port_command,
    bool enable_clock_sync)
  {
    pImpl = new MotionCaptureOptitrackImpl;
    pImpl->clock_sync_enabled = enable_clock_sync;

    // Connect to command port to query version
    boost::asio::io_context io_context_cmd;
    udp::socket socket_cmd(io_context_cmd, udp::endpoint(udp::v4(), 0));
    udp::endpoint endpoint_cmd(boost::asio::ip::make_address(hostname), port_command);
    pImpl->cmd_endpoint = endpoint_cmd;

    typedef struct
    {
      unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
      unsigned short nDataBytes;              // Num bytes in payload
    } sRequest;

    typedef struct
    {
      unsigned short iMessage;
      unsigned short nDataBytes;
      char szName[MAX_NAMELENGTH];      // host app's name
      unsigned char Version[4];         // host app's version [major.minor.build.revision]
      unsigned char NatNetVersion[4];   // host app's NatNet version [major.minor.build.revision]
      uint8_t HighResClockFrequency[8];   // host's high resolution clock frequency (ticks per second)
      uint16_t DataPort;
      bool IsMulticast;
      uint8_t MulticastGroupAddress[4];
    } sResponse;

    // HOPE patch (see PIN.md): the NAT_CONNECT below registers this command
    // socket as a unicast client, so Motive starts streaming FRAMEOFDATA to
    // it immediately — the replies awaited here can interleave with (larger)
    // frame packets. Receive into a full-size buffer and discard until the
    // expected message id arrives; receiving a frame into the small response
    // struct would throw boost::asio message_size and kill the node.
    udp::endpoint sender_endpoint;
    std::vector<char> reply(MAX_PACKETSIZE);
    size_t reply_length = 0;
    uint16_t reply_id = 0;

    // HOPE patch (2026-07-30): bound the wait so an unresponsive Motive fails
    // loudly instead of hanging the node forever (see HANDSHAKE_TIMEOUT_S).
    {
      struct timeval tv;
      tv.tv_sec = HANDSHAKE_TIMEOUT_S;
      tv.tv_usec = 0;
      ::setsockopt(socket_cmd.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                   &tv, sizeof(tv));
    }

    auto await_reply = [&](uint16_t expected, const void *request,
                           size_t request_len, const char *what) {
      for (int attempt = 0; attempt < HANDSHAKE_ATTEMPTS; ++attempt) {
        socket_cmd.send_to(boost::asio::buffer(request, request_len),
                           endpoint_cmd);
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(HANDSHAKE_TIMEOUT_S);
        while (std::chrono::steady_clock::now() < deadline) {
          boost::system::error_code ec;
          reply_length = socket_cmd.receive_from(
              boost::asio::buffer(reply.data(), reply.size()),
              sender_endpoint, 0, ec);
          if (ec) {
            break;  // receive timeout -> re-send the request
          }
          if (reply_length >= 2) {
            memcpy(&reply_id, reply.data(), 2);
            if (reply_id == expected) {
              return;
            }
          }
        }
        std::cerr << "[optitrack] no " << what << " from " << hostname
                  << " (attempt " << attempt + 1 << "/" << HANDSHAKE_ATTEMPTS
                  << "), retrying" << std::endl;
      }
      throw std::runtime_error(
          std::string("NatNet handshake failed: no ") + what + " from " +
          hostname + " after " + std::to_string(HANDSHAKE_ATTEMPTS) +
          " attempts");
    };

    sRequest connectCmd = {NAT_CONNECT, 0};
    await_reply(NAT_SERVERINFO, &connectCmd, sizeof(connectCmd),
                "NAT_SERVERINFO");

    sResponse response;
    memcpy(&response, reply.data(),
           std::min(reply_length, sizeof(response)));

    if (response.iMessage != NAT_SERVERINFO) {
      throw std::runtime_error("Could not query NatNet version!");
    }

    std::ostringstream stringStream;
    stringStream << (int)response.NatNetVersion[0] << "."
                 << (int)response.NatNetVersion[1] << "."
                 << (int)response.NatNetVersion[2] << "."
                 << (int)response.NatNetVersion[3];
    pImpl->version = stringStream.str();

    pImpl->versionMajor = response.NatNetVersion[0];
    pImpl->versionMinor = response.NatNetVersion[1];
    memcpy(&pImpl->clockFrequency, response.HighResClockFrequency, sizeof(uint64_t));

    uint16_t port_data = response.DataPort;

    // query model def. The 4-byte MODELDEF_TYPES mask is MANDATORY on Motive
    // 3.1 / NatNet 4.1 -- without it the request is silently dropped. The
    // message-id filter inside await_reply() also guards the interleave hazard
    // described above: without it we could "parse" a frame packet, which fails
    // silently and leaves EVERY streamed rigid body with an empty name.
    sModelDefRequest modelDefCmd = {NAT_REQUEST_MODELDEF, sizeof(int32_t),
                                    MODELDEF_TYPES};
    await_reply(NAT_MODELDEF, &modelDefCmd, sizeof(modelDefCmd), "NAT_MODELDEF");
    std::vector<char> modelDef(reply.begin(), reply.begin() + reply_length);
    pImpl->parseModelDef(modelDef.data());

    // The transient handshake registration must not remain as a second
    // unicast frame destination once the persistent data socket registers.
    // Echo synchronization below deliberately uses that data socket too.
    socket_cmd.close();

    // connect to data port to receive mocap data
    auto listen_address_boost = boost::asio::ip::make_address_v4(interface_ip);

    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        boost::asio::ip::address_v4::any(), port_data);
    pImpl->socket.open(listen_endpoint.protocol());
    pImpl->socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    pImpl->socket.bind(listen_endpoint);

    if (response.IsMulticast) {
      std::stringstream sstr;
      sstr << (int)response.MulticastGroupAddress[0] << "."
           << (int)response.MulticastGroupAddress[1] << "."
           << (int)response.MulticastGroupAddress[2] << "."
           << (int)response.MulticastGroupAddress[3];
      std::string multicast_address = sstr.str();
      auto multicast_address_boost = boost::asio::ip::make_address_v4(multicast_address);
      // Join the multicast group on a specific interface
      pImpl->socket.set_option(boost::asio::ip::multicast::join_group(multicast_address_boost, listen_address_boost));
    } else {
      // log some server info
      std::ostringstream ustr;
      ustr << "Using unicast from server " << hostname << ":" << port_data;
      std::cout << ustr.str() << std::endl;
    }

    // HOPE patch (see PIN.md): register the DATA socket with Motive. In
    // unicast mode Motive (verified against Motive 3.1 / NatNet 4.1) streams
    // FRAMEOFDATA back to the SOURCE endpoint of the NAT_CONNECT packet — not
    // to the advertised data port — so a registration sent from the transient
    // command socket above leaves this data socket permanently silent.
    // Harmless in multicast mode (the extra NAT_SERVERINFO reply is dropped by
    // the MessageID==7 filter in waitForNextFrame).
    const uint16_t connect_from_data[2] = {NAT_CONNECT, 0};
    pImpl->socket.send_to(
        boost::asio::buffer(connect_from_data, sizeof(connect_from_data)),
        pImpl->cmd_endpoint);
    pImpl->last_keepalive = std::chrono::steady_clock::now();

    if (pImpl->clock_sync_enabled) {
      if (!pImpl->initializeClockSync()) {
        throw std::runtime_error(
          "NatNet clock synchronization failed: Motive did not answer "
          "NAT_ECHOREQUEST. camera_utc timestamps are unavailable. If and "
          "only if acquisition-time alignment is not required, explicitly "
          "relaunch with header_time:=ros; receipt-time stamps are not valid "
          "for moving cross-sensor calibration");
      }
      std::cout << "[optitrack] NatNet clock sync ready: min RTT "
                << pImpl->clock_mapping.minimumRttSeconds() * 1e3
                << " ms, initial uncertainty "
                << pImpl->clock_mapping.baseUncertaintySeconds() * 1e3 << " ms"
                << std::endl;
    }
  }

  const std::string & MotionCaptureOptitrack::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureOptitrack::waitForNextFrame()
  {
    // Schedule a periodic NatNet echo refresh (or expire a lost request).
    // Responses are consumed below alongside frame packets on this same data
    // socket, so the refresh never creates a second unicast stream or blocks
    // the camera-rate path.
    pImpl->pollClockSync();

    // HOPE patch (see PIN.md): ~1 Hz keep-alive from the data socket so Motive
    // does not expire this unicast client (it drops receivers that stay silent
    // for a few seconds; official SDK clients do the same). Sent before the
    // blocking receive; if Motive restarts mid-run the stream stays down until
    // the node is restarted — same behavior as the VRPN path.
    auto ka_now = std::chrono::steady_clock::now();
    if (ka_now - pImpl->last_keepalive > std::chrono::seconds(1)) {
      const uint16_t keepalive[2] = {NAT_KEEPALIVE, 0};
      boost::system::error_code ka_ec;
      pImpl->socket.send_to(boost::asio::buffer(keepalive, sizeof(keepalive)),
                            pImpl->cmd_endpoint, 0, ka_ec);
      pImpl->last_keepalive = ka_now;
    }

    // Use a loop to retain the newest frame while consuming interleaved
    // command replies.  Echo requests originate from this same registered
    // data socket so Motive does not see a second unicast streaming client.
    bool have_frame = false;
    do {
      const size_t length = pImpl->socket.receive_from(
        boost::asio::buffer(
          pImpl->incoming_data.data(), pImpl->incoming_data.size()),
        pImpl->sender_endpoint);
      const auto receive_time = std::chrono::steady_clock::now();
      if (length < 2) {
        continue;
      }

      uint16_t drain_id = 0;
      memcpy(&drain_id, pImpl->incoming_data.data(), 2);
      if (drain_id == 7) {  // NAT_FRAMEOFDATA
        pImpl->data.assign(
          pImpl->incoming_data.begin(), pImpl->incoming_data.begin() + length);
        have_frame = true;
      } else if (drain_id == NAT_MODELDEF) {
        // Model-definition replies requested below arrive interleaved with
        // frames; consume them so the latest-only drain cannot discard one.
        pImpl->parseModelDef(pImpl->incoming_data.data());
      } else if (drain_id == NAT_ECHORESPONSE && pImpl->pending_echo) {
        uint64_t server_receive_ticks = 0;
        if (pImpl->decodeEchoResponse(
              pImpl->incoming_data.data(), length,
              pImpl->pending_echo_token, server_receive_ticks)) {
          pImpl->updateClockMapping(
            pImpl->pending_echo_sent, receive_time, server_receive_ticks);
          pImpl->pending_echo = false;
        }
      }
    } while (!have_frame || pImpl->socket.available() > 0);

    if (pImpl->data.size() > 4) {
      char *ptr = pImpl->data.data();
      int major = pImpl->versionMajor;
      int minor = pImpl->versionMinor;

      // First 2 Bytes is message ID
      int MessageID = 0;
      memcpy(&MessageID, ptr, 2); ptr += 2;
      // printf("Message ID : %d\n", MessageID);

      // Second 2 Bytes is the size of the packet
      int nBytes = 0;
      memcpy(&nBytes, ptr, 2); ptr += 2;
      // printf("Byte count : %d\n", nBytes);

      if(MessageID == 7)      // FRAME OF MOCAP DATA packet
      {
        // Next 4 Bytes is the frame number
        int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
        // printf("Frame # : %d\n", frameNumber);
      
        // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
        int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
        // printf("Marker Set Count : %d\n", nMarkerSets);

        int nBytes=0;
        ptr = UnpackDataSize(ptr, major, minor,nBytes);

        // Loop through number of marker sets and get name and data
        for (int i=0; i < nMarkerSets; i++)
        {
          ptr += strlen(ptr) + 1;
          int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
          ptr += nMarkers * 12;
        }

        // Loop through unlabeled markers
        // OtherMarker list is Deprecated
        int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
        ptr = UnpackDataSize(ptr, major, minor,nBytes);
        pImpl->markers.resize(nOtherMarkers);
        for (int j = 0; j < nOtherMarkers; j++)
        {
          memcpy(&pImpl->markers[j].x, ptr, 4); ptr += 4;
          memcpy(&pImpl->markers[j].y, ptr, 4); ptr += 4;
          memcpy(&pImpl->markers[j].z, ptr, 4); ptr += 4;
        }

        // Loop through rigidbodies
        int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        ptr = UnpackDataSize(ptr, major, minor,nBytes);
        pImpl->rigidBodies.resize(nRigidBodies);
        // printf("Rigid Body Count : %d\n", nRigidBodies);
        for (int j=0; j < nRigidBodies; j++)
        {
          // Rigid body position and orientation 
          memcpy(&pImpl->rigidBodies[j].ID, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].x, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].y, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].z, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qx, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qy, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qz, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qw, ptr, 4); ptr += 4;

          // NatNet version 2.0 and later
          if(major >= 2)
          {
            // Mean marker error
            memcpy(&pImpl->rigidBodies[j].fError, ptr, 4); ptr += 4;
          }

          // NatNet version 2.6 and later
          if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
          {
            // params
            short params = 0; memcpy(&params, ptr, 2); ptr += 2;
            pImpl->rigidBodies[j].bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
          }
        } // Go to next rigid body

        // Skeletons (NatNet version 2.1 and later)
        // (we do not support skeletons)
        if( ((major == 2)&&(minor>0)) || (major>2))
        {
          int nSkeletons = 0; memcpy(&nSkeletons, ptr, 4); ptr += 4;
          // printf("Skeleton Count : %d\n", nSkeletons);
          ptr = UnpackDataSize(ptr, major, minor,nBytes);

          // Loop through skeletons
          for (int j=0; j < nSkeletons; j++)
          {
            // skeleton id
            // int skeletonID = 0;
            // memcpy(&skeletonID, ptr, 4);
            ptr += 4;

            // Number of rigid bodies (bones) in skeleton
            int nRigidBodies = 0;
            memcpy(&nRigidBodies, ptr, 4); ptr += 4;
            // printf("Rigid Body Count : %d\n", nRigidBodies);

            // Loop through rigid bodies (bones) in skeleton
            for (int j=0; j < nRigidBodies; j++)
            {
              // Rigid body position and orientation
              ptr += 8*4;

              // Mean marker error (NatNet version 2.0 and later)
              if(major >= 2)
              {
                ptr += 4;
              }

              // Tracking flags (NatNet version 2.6 and later)
              if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
              {
                ptr += 2;
              }
            } // next rigid body
          } // next skeleton
        }

        // Assets ( Motive 3.1 / NatNet 4.1 and greater)
        if (((major == 4) && (minor > 0)) || (major > 4))
        {
            int nAssets = 0;
            memcpy(&nAssets, ptr, 4); ptr += 4;
            // printf("Asset Count : %d\n", nAssets);

            int nBytes=0;
            ptr = UnpackDataSize(ptr, major, minor,nBytes);
            ptr += nBytes;
        }
        
        // labeled markers (NatNet version 2.3 and later)
        // labeled markers - this includes all markers: Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
        if( ((major == 2)&&(minor>=3)) || (major>2))
        {
          int nLabeledMarkers = 0;
          memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
          ptr = UnpackDataSize(ptr, major, minor,nBytes);
          pImpl->markers.resize(nOtherMarkers + nLabeledMarkers);
          // printf("Labeled Marker Count : %d\n", nLabeledMarkers);

          // Loop through labeled markers
          for (int j=0; j < nLabeledMarkers; j++)
          {
            // id
            // Marker ID Scheme:
            // Active Markers:
            //   ID = ActiveID, correlates to RB ActiveLabels list
            // Passive Markers: 
            //   If Asset with Legacy Labels
            //      AssetID   (Hi Word)
            //      MemberID  (Lo Word)
            //   Else
            //      PointCloud ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // int modelID, markerID;
            // DecodeMarkerID(ID, &modelID, &markerID);

            memcpy(&pImpl->markers[nOtherMarkers + j].x, ptr, 4); ptr += 4;
            memcpy(&pImpl->markers[nOtherMarkers + j].y, ptr, 4); ptr += 4;
            memcpy(&pImpl->markers[nOtherMarkers + j].z, ptr, 4); ptr += 4;
            // size
            //float size = 0.0f; memcpy(&size, ptr, 4);
            ptr += 4;

            // NatNet version 2.6 and later
            if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
            {
              // marker params
              // short params = 0; memcpy(&params, ptr, 2);
              ptr += 2;
              // bool bOccluded = (params & 0x01) != 0;     // marker was not visible (occluded) in this frame
              // bool bPCSolved = (params & 0x02) != 0;     // position provided by point cloud solve
              // bool bModelSolved = (params & 0x04) != 0;  // position provided by model solve
              // if ((major >= 3) || (major == 0))
              // {
              //   bool bHasModel = (params & 0x08) != 0;     // marker has an associated asset in the data stream
              //   bool bUnlabeled = (params & 0x10) != 0;    // marker is 'unlabeled', but has a point cloud ID
              //   bool bActiveMarker = (params & 0x20) != 0; // marker is an actively labeled LED marker
              // }
            }

            // NatNet version 3.0 and later
            // float residual = 0.0f;
            if ((major >= 3) || (major == 0))
            {
              // Marker residual
              // memcpy(&residual, ptr, 4);
              ptr += 4;
            }
          }
        }

        // Force Plate data (NatNet version 2.9 and later)
        if (((major == 2) && (minor >= 9)) || (major > 2))
        {
          int nForcePlates;
          memcpy(&nForcePlates, ptr, 4); ptr += 4;
          ptr = UnpackDataSize(ptr, major, minor,nBytes);
          for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
          {
            // ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // printf("Force Plate : %d\n", ID);

            // Channel Count
            int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

            // Channel Data
            for (int i = 0; i < nChannels; i++)
            {
              // printf(" Channel %d : ", i);
              int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
              for (int j = 0; j < nFrames; j++)
              {
                  // float val = 0.0f;  memcpy(&val, ptr, 4);
                  ptr += 4;
                  // printf("%3.2f   ", val);
              }
              // printf("\n");
            }
          }
        }

        // Device data (NatNet version 3.0 and later)
        if (((major == 2) && (minor >= 11)) || (major > 2))
        {
          int nDevices;
          memcpy(&nDevices, ptr, 4); ptr += 4;
          ptr = UnpackDataSize(ptr, major, minor,nBytes);
          for (int iDevice = 0; iDevice < nDevices; iDevice++)
          {
            // ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // printf("Device : %d\n", ID);

            // Channel Count
            int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

            // Channel Data
            for (int i = 0; i < nChannels; i++)
            {
              // printf(" Channel %d : ", i);
              int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
              for (int j = 0; j < nFrames; j++)
              {
                  // float val = 0.0f;  memcpy(&val, ptr, 4); 
                  ptr += 4;
                  // printf("%3.2f   ", val);
              }
              // printf("\n");
            }
          }
        }
    
        // software latency (removed in version 3.0)
        if ( major < 3 )
        {
          // float softwareLatency = 0.0f; memcpy(&softwareLatency, ptr, 4);
          ptr += 4;
          // printf("software latency : %3.3f\n", softwareLatency);
        }

        // timecode
        // unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);
        ptr += 4;
        // unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4);
        ptr += 4;
        // char szTimecode[128] = "";
        // TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

        // timestamp
        // double timestamp = 0.0f;

        // NatNet version 2.7 and later - increased from single to double precision
        if( ((major == 2)&&(minor>=7)) || (major>2))
        {
          // memcpy(&timestamp, ptr, 8);
          ptr += 8;
        }
        else
        {
          // float fTemp = 0.0f;
          // memcpy(&fTemp, ptr, 4);
          ptr += 4;
          // timestamp = (double)fTemp;
        }
        // printf("Timestamp : %3.3f\n", timestamp);

        // high res timestamps (version 3.0 and later)
        latencies_.clear();
        if ( (major >= 3) || (major == 0) )
        {
          uint64_t cameraMidExposureTimestamp = 0;
          memcpy( &cameraMidExposureTimestamp, ptr, 8 );
          ptr += 8;
          pImpl->cameraMidExposureTimestamp = cameraMidExposureTimestamp;

          uint64_t cameraDataReceivedTimestamp = 0;
          memcpy( &cameraDataReceivedTimestamp, ptr, 8 );
          ptr += 8;

          uint64_t transmitTimestamp = 0;
          memcpy( &transmitTimestamp, ptr, 8 );
          ptr += 8;
          pImpl->transmitTimestamp = transmitTimestamp;

          const bool timing_valid = pImpl->clockFrequency != 0 &&
            cameraMidExposureTimestamp != 0 &&
            cameraDataReceivedTimestamp >= cameraMidExposureTimestamp &&
            transmitTimestamp >= cameraDataReceivedTimestamp;
          if (timing_valid) {
            const uint64_t cameraLatencyTicks =
              cameraDataReceivedTimestamp - cameraMidExposureTimestamp;
            const double cameraLatencySeconds =
              cameraLatencyTicks / static_cast<double>(pImpl->clockFrequency);
            latencies_.emplace_back(
              LatencyInfo("Camera", cameraLatencySeconds));

            const uint64_t swLatencyTicks =
              transmitTimestamp - cameraDataReceivedTimestamp;
            const double swLatencySeconds =
              swLatencyTicks / static_cast<double>(pImpl->clockFrequency);
            latencies_.emplace_back(LatencyInfo("Motive", swLatencySeconds));

            // Preserve the legacy vendor-relative microsecond field for V2.
            timestamp_ = ticksToMicroseconds(
              cameraMidExposureTimestamp, pImpl->clockFrequency);
          } else {
            pImpl->cameraMidExposureTimestamp = 0;
            timestamp_ = 0;
          }
        }

        // frame params
        short params = 0;  memcpy(&params, ptr, 2);
        ptr += 2;
        // bool bIsRecording = (params & 0x01) != 0;                  // 0x01 Motive is recording
        bool bTrackedModelsChanged = (params & 0x02) != 0;         // 0x02 Actively tracked model list has changed

        // end of data tag
        // int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
        // printf("End Packet\n-------------\n");
      }
      else if (MessageID == NAT_MODELDEF)
      {
          // already consumed by the receive drain above (HOPE patch)
      }
      else
      {
          printf("Unrecognized Packet Type.\n");
      }
    }

    // HOPE patch (see PIN.md): self-heal stale model definitions. Rigid-body
    // NAMES come from the connect-time model definition; assets created or
    // renamed in Motive afterwards stream under an unknown ID and would show
    // an empty name forever. When that happens, re-request the model
    // definition (throttled to 1 Hz) from the data socket; the reply is
    // parsed by the drain hook above within a cycle or two.
    bool unnamed_body = false;
    for (const auto& rb : pImpl->rigidBodies) {
      auto def_it = pImpl->rigidBodyDefinitions.find(rb.ID);
      if (def_it == pImpl->rigidBodyDefinitions.end() || def_it->second.name.empty()) {
        unnamed_body = true;
        break;
      }
    }
    if (unnamed_body &&
        ka_now - pImpl->last_modeldef_request > std::chrono::seconds(1)) {
      // Same mandatory 4-byte type mask as the connect-time request.
      sModelDefRequest request_modeldef = {NAT_REQUEST_MODELDEF,
                                           sizeof(int32_t), MODELDEF_TYPES};
      boost::system::error_code md_ec;
      pImpl->socket.send_to(
          boost::asio::buffer(&request_modeldef, sizeof(request_modeldef)),
          pImpl->cmd_endpoint, 0, md_ec);
      pImpl->last_modeldef_request = ka_now;
    }
  }

  const std::map<std::string, RigidBody>& MotionCaptureOptitrack::rigidBodies() const
  {
    // TODO: avoid copies here...
    rigidBodies_.clear();
    for (const auto& rb : pImpl->rigidBodies) {
      if (rb.bTrackingValid) {
        const auto& def = pImpl->rigidBodyDefinitions[rb.ID];

        Eigen::Vector3f position(
          rb.x + def.xoffset,
          rb.y + def.yoffset,
          rb.z + def.zoffset);

        Eigen::Quaternionf rotation(
          rb.qw, // w
          rb.qx, // x
          rb.qy, // y
          rb.qz  // z
          );
        rigidBodies_.emplace(def.name, RigidBody(def.name, position, rotation));
      }
    }
    return rigidBodies_;
  }

  const PointCloud& MotionCaptureOptitrack::pointCloud() const
  {
    // TODO: avoid copies here...
    pointcloud_.resize(pImpl->markers.size(), Eigen::NoChange);
    for (size_t r = 0; r < pImpl->markers.size(); ++r) {
      const auto& marker = pImpl->markers[r];
      pointcloud_.row(r) << marker.x, marker.y, marker.z;
    }
    return pointcloud_;
  }

  const std::vector<LatencyInfo> &MotionCaptureOptitrack::latency() const
  {
    return latencies_;
  }

  uint64_t MotionCaptureOptitrack::timeStamp() const
  {
    return timestamp_;
  }

  double MotionCaptureOptitrack::timeStampAge() const
  {
    return pImpl->timestampAge();
  }

  double MotionCaptureOptitrack::timeStampAgeUncertainty() const
  {
    return pImpl->timestampAgeUncertainty();
  }

  bool MotionCaptureOptitrack::supportsTimeStampAge() const
  {
    return pImpl->clock_mapping.valid() &&
           pImpl->cameraMidExposureTimestamp != 0;
  }

  MotionCaptureOptitrack::~MotionCaptureOptitrack()
  {
    delete pImpl;
  }

}
