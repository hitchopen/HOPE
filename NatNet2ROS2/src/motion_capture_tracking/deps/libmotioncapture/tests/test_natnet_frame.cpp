#include "libmotioncapture/natnet_frame.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {

template<typename T>
void append(std::vector<char>& packet, const T& value)
{
  const char* bytes = reinterpret_cast<const char*>(&value);
  packet.insert(packet.end(), bytes, bytes + sizeof(T));
}

void appendBytes(std::vector<char>& packet, std::size_t count, char value)
{
  packet.insert(packet.end(), count, value);
}

template<typename Callback>
void appendSection(std::vector<char>& packet, int32_t count, Callback callback)
{
  append<int32_t>(packet, count);
  const std::size_t size_offset = packet.size();
  append<int32_t>(packet, 0);
  const std::size_t data_offset = packet.size();
  callback();
  const int32_t size = static_cast<int32_t>(packet.size() - data_offset);
  std::memcpy(packet.data() + size_offset, &size, sizeof(size));
}

std::vector<char> makeNatNet45Frame(bool include_extensions)
{
  std::vector<char> packet;
  append<uint16_t>(
    packet, libmotioncapture::detail::NAT_FRAMEOFDATA_MESSAGE_ID);
  append<uint16_t>(packet, 0);
  append<int32_t>(packet, 4242);

  // The bridge skips marker-set payloads by their authoritative section size;
  // this deliberately is not a valid null-terminated marker-set string.
  appendSection(packet, 1, [&]() { appendBytes(packet, 7, 'M'); });

  appendSection(packet, 1, [&]() {
    append<float>(packet, 1.0f);
    append<float>(packet, 2.0f);
    append<float>(packet, 3.0f);
  });

  appendSection(packet, 1, [&]() {
    append<int32_t>(packet, 55);
    append<float>(packet, 0.1f);
    append<float>(packet, 0.2f);
    append<float>(packet, 0.3f);
    append<float>(packet, 0.0f);
    append<float>(packet, 0.0f);
    append<float>(packet, 0.0f);
    append<float>(packet, 1.0f);
    append<float>(packet, 0.004f);
    append<uint16_t>(packet, 0x01);
  });

  appendSection(packet, 0, []() {});  // skeletons
  appendSection(packet, 0, []() {});  // assets

  appendSection(packet, 1, [&]() {
    append<uint32_t>(packet, (55U << 16) | 1U);
    append<float>(packet, 4.0f);
    append<float>(packet, 5.0f);
    append<float>(packet, 6.0f);
    append<float>(packet, 0.012f);
    append<uint16_t>(packet, 0x04);
    append<float>(packet, 0.001f);
  });

  appendSection(packet, 0, []() {});  // force plates
  appendSection(packet, 0, []() {});  // devices

  if (include_extensions) {
    // NatNet 4.5 IMU and GPIO payload contents are irrelevant to this bridge.
    // Their section boundaries must not shift the timestamp suffix.
    appendSection(packet, 1, [&]() { appendBytes(packet, 38, 'I'); });
    appendSection(packet, 1, [&]() { appendBytes(packet, 13, 'G'); });
  }

  append<uint32_t>(packet, 0);       // timecode
  append<uint32_t>(packet, 0);       // timecode subframe
  append<double>(packet, 12.5);      // software timestamp
  append<uint64_t>(packet, 1000);    // camera mid-exposure
  append<uint64_t>(packet, 1100);    // camera data received
  append<uint64_t>(packet, 1200);    // transmit
  append<uint32_t>(packet, 123);     // precision timestamp seconds
  append<uint32_t>(packet, 456);     // precision fractional seconds
  append<uint16_t>(packet, 0x02);    // tracked models changed
  append<int32_t>(packet, 0);        // EOD

  const uint16_t payload_size = static_cast<uint16_t>(packet.size() - 4);
  std::memcpy(packet.data() + 2, &payload_size, sizeof(payload_size));
  return packet;
}

bool near(float lhs, float rhs, float tolerance = 1e-6f)
{
  return std::abs(lhs - rhs) <= tolerance;
}

template<typename Callback>
bool throwsRuntimeError(Callback callback)
{
  try {
    callback();
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

}  // namespace

int main()
{
  using libmotioncapture::detail::parseNatNetSizedFrame;

  const std::vector<char> natnet_45 = makeNatNet45Frame(true);
  const auto frame =
    parseNatNetSizedFrame(natnet_45.data(), natnet_45.size(), 4, 5);
  assert(frame.frame_number == 4242);
  assert(frame.skipped_extension_sections == 2);
  assert(frame.rigid_bodies.size() == 1);
  assert(frame.rigid_bodies[0].id == 55);
  assert(frame.rigid_bodies[0].tracking_valid);
  assert(near(frame.rigid_bodies[0].mean_marker_error, 0.004f));
  assert(frame.labeled_markers.size() == 1);
  assert(frame.labeled_markers[0].modelId == 55);
  assert(frame.labeled_markers[0].memberId == 1);
  assert(near(frame.labeled_markers[0].position.z(), 6.0f));
  assert(frame.markers.size() == 2);
  assert(frame.camera_mid_exposure_timestamp == 1000);
  assert(frame.camera_data_received_timestamp == 1100);
  assert(frame.transmit_timestamp == 1200);
  assert(frame.precision_timestamp_seconds == 123);
  assert(frame.precision_timestamp_fractional_seconds == 456);
  assert(frame.frame_params == 0x02);

  const std::vector<char> natnet_42 = makeNatNet45Frame(false);
  const auto frame_42 =
    parseNatNetSizedFrame(natnet_42.data(), natnet_42.size(), 4, 2);
  assert(frame_42.skipped_extension_sections == 0);
  assert(frame_42.transmit_timestamp == 1200);

  std::vector<char> truncated = natnet_45;
  truncated.pop_back();
  assert(throwsRuntimeError([&]() {
    parseNatNetSizedFrame(truncated.data(), truncated.size(), 4, 5);
  }));

  std::vector<char> bad_eod = natnet_45;
  bad_eod[bad_eod.size() - 1] = 1;
  assert(throwsRuntimeError([&]() {
    parseNatNetSizedFrame(bad_eod.data(), bad_eod.size(), 4, 5);
  }));

  return 0;
}
