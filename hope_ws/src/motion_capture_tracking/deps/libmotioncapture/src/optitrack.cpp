#include "libmotioncapture/optitrack.h"

#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
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

  PACK(struct sModelDefRequest {
    uint16_t iMessage;
    uint16_t nDataBytes;
    int32_t types;
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
    {
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
    // HOPE patch (see PIN.md): Motive command endpoint + keep-alive pacing for
    // unicast streaming, where Motive sends frames to the NAT_CONNECT source
    // socket and drops clients that stay silent for a few seconds.
    boost::asio::ip::udp::endpoint cmd_endpoint;
    std::chrono::steady_clock::time_point last_keepalive;
    std::chrono::steady_clock::time_point last_modeldef_request;

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
    int port_command)
  {
    pImpl = new MotionCaptureOptitrackImpl;

    // Connect to command port to query version
    boost::asio::io_context io_context_cmd;
    udp::socket socket_cmd(io_context_cmd, udp::endpoint(udp::v4(), 0));
    udp::resolver resolver_cmd(io_context_cmd);
    udp::endpoint endpoint_cmd(boost::asio::ip::make_address(hostname), port_command);

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
    pImpl->cmd_endpoint = endpoint_cmd;
    const uint16_t connect_from_data[2] = {NAT_CONNECT, 0};
    pImpl->socket.send_to(
        boost::asio::buffer(connect_from_data, sizeof(connect_from_data)),
        pImpl->cmd_endpoint);
    pImpl->last_keepalive = std::chrono::steady_clock::now();
  }

  const std::string & MotionCaptureOptitrack::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureOptitrack::waitForNextFrame()
  {
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

    // use a loop to get latest data
    do {
      pImpl->data.resize(MAX_PACKETSIZE);
      size_t length = pImpl->socket.receive_from(boost::asio::buffer(pImpl->data.data(), pImpl->data.size()), pImpl->sender_endpoint);
      pImpl->data.resize(length);
      // HOPE patch (see PIN.md): model-definition replies (requested below
      // when a streamed body has no name) arrive interleaved with frames on
      // this socket — parse them here so the latest-only drain cannot
      // discard one.
      if (pImpl->data.size() >= 4) {
        uint16_t drain_id = 0;
        memcpy(&drain_id, pImpl->data.data(), 2);
        if (drain_id == NAT_MODELDEF) {
          pImpl->parseModelDef(pImpl->data.data());
        }
      }
    } while (pImpl->socket.available() > 0);

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

          uint64_t cameraDataReceivedTimestamp = 0;
          memcpy( &cameraDataReceivedTimestamp, ptr, 8 );
          ptr += 8;

          uint64_t transmitTimestamp = 0;
          memcpy( &transmitTimestamp, ptr, 8 );
          ptr += 8;

          const uint64_t cameraLatencyTicks = cameraDataReceivedTimestamp - cameraMidExposureTimestamp;
          const double cameraLatencySeconds = cameraLatencyTicks / (double)pImpl->clockFrequency;
          latencies_.emplace_back(LatencyInfo("Camera", cameraLatencySeconds));

          const uint64_t swLatencyTicks = transmitTimestamp - cameraDataReceivedTimestamp;
          const double swLatencySeconds = swLatencyTicks / (double)pImpl->clockFrequency;
          latencies_.emplace_back(LatencyInfo("Motive", swLatencySeconds));

          // convert actual shutter timestamp to microseconds
          timestamp_ = cameraMidExposureTimestamp * 1e6 / pImpl->clockFrequency;
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

  MotionCaptureOptitrack::~MotionCaptureOptitrack()
  {
    delete pImpl;
  }

}

