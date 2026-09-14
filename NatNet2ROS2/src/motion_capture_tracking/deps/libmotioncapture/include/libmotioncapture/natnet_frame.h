#pragma once

#include "libmotioncapture/motioncapture.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace libmotioncapture {
namespace detail {

constexpr uint16_t NAT_FRAMEOFDATA_MESSAGE_ID = 7;
constexpr int32_t NATNET_MAX_FRAME_ITEMS = 10000;

// NatNet 4.1 added an authoritative byte count to every FRAMEOFDATA section.
// The suffix contains timecode (8), timestamp (8), three QPC timestamps (24),
// two precision/PTP timestamp words (8), frame params (2), and EOD (4).
constexpr std::size_t NATNET_41_FRAME_SUFFIX_BYTES = 54;

class NatNetFrameCursor
{
public:
  NatNetFrameCursor(const char* begin, const char* end)
    : current_(begin), end_(end)
  {
    if (begin == nullptr || end == nullptr || begin > end) {
      throw std::runtime_error("NatNet FRAMEOFDATA packet bounds are invalid");
    }
  }

  template<typename T>
  T read(const char* field)
  {
    require(sizeof(T), field);
    T value{};
    std::memcpy(&value, current_, sizeof(T));
    current_ += sizeof(T);
    return value;
  }

  void skip(std::size_t bytes, const char* field)
  {
    require(bytes, field);
    current_ += bytes;
  }

  std::size_t remaining() const
  {
    return static_cast<std::size_t>(end_ - current_);
  }

  const char* current() const
  {
    return current_;
  }

  const char* end() const
  {
    return end_;
  }

private:
  void require(std::size_t bytes, const char* field) const
  {
    if (bytes > remaining()) {
      throw std::runtime_error(
        std::string("NatNet FRAMEOFDATA truncated while reading ") + field);
    }
  }

  const char* current_;
  const char* end_;
};

struct NatNetFrameRigidBody
{
  int32_t id;
  float x;
  float y;
  float z;
  float qx;
  float qy;
  float qz;
  float qw;
  float mean_marker_error;
  bool tracking_valid;
};

struct NatNetFrameData
{
  int32_t frame_number = 0;
  std::vector<Eigen::Vector3f> markers;
  std::vector<NatNetFrameRigidBody> rigid_bodies;
  std::vector<LabeledMarker> labeled_markers;
  uint64_t camera_mid_exposure_timestamp = 0;
  uint64_t camera_data_received_timestamp = 0;
  uint64_t transmit_timestamp = 0;
  uint32_t precision_timestamp_seconds = 0;
  uint32_t precision_timestamp_fractional_seconds = 0;
  uint16_t frame_params = 0;
  std::size_t skipped_extension_sections = 0;
};

inline int32_t readFrameItemCount(NatNetFrameCursor& packet, const char* field)
{
  const int32_t count = packet.read<int32_t>(field);
  if (count < 0 || count > NATNET_MAX_FRAME_ITEMS) {
    throw std::runtime_error(
      std::string("NatNet FRAMEOFDATA invalid item count: ") + field);
  }
  return count;
}

inline NatNetFrameCursor readSizedFrameSection(
  NatNetFrameCursor& packet, int32_t& count, const char* field)
{
  count = readFrameItemCount(packet, field);
  const int32_t byte_count = packet.read<int32_t>("section byte count");
  if (byte_count < 0 ||
      static_cast<std::size_t>(byte_count) > packet.remaining()) {
    throw std::runtime_error(
      std::string("NatNet FRAMEOFDATA invalid section size: ") + field);
  }
  const char* begin = packet.current();
  packet.skip(static_cast<std::size_t>(byte_count), field);
  return NatNetFrameCursor(begin, begin + byte_count);
}

inline NatNetFrameData parseNatNetSizedFrame(
  const char* data, std::size_t length, int major, int minor)
{
  if (major < 4 || (major == 4 && minor < 1)) {
    throw std::runtime_error(
      "Sized FRAMEOFDATA parsing requires NatNet 4.1 or newer");
  }
  if (data == nullptr || length < 8) {
    throw std::runtime_error(
      "NatNet FRAMEOFDATA packet is shorter than its header");
  }

  NatNetFrameCursor packet(data, data + length);
  const uint16_t message_id = packet.read<uint16_t>("message id");
  const uint16_t payload_size = packet.read<uint16_t>("payload size");
  if (message_id != NAT_FRAMEOFDATA_MESSAGE_ID) {
    throw std::runtime_error("NatNet packet is not a FRAMEOFDATA message");
  }
  if (static_cast<std::size_t>(payload_size) > length - 4) {
    throw std::runtime_error(
      "NatNet FRAMEOFDATA payload exceeds the UDP datagram");
  }

  NatNetFrameCursor payload(packet.current(), data + 4 + payload_size);
  NatNetFrameData frame;
  frame.frame_number = payload.read<int32_t>("frame number");

  int32_t count = 0;

  // Marker-set strings are not used by the bridge. The section byte count is
  // authoritative, so no strlen walk is needed.
  (void)readSizedFrameSection(payload, count, "marker sets");

  NatNetFrameCursor other_markers =
    readSizedFrameSection(payload, count, "legacy other markers");
  frame.markers.reserve(static_cast<std::size_t>(count));
  for (int32_t i = 0; i < count; ++i) {
    const float x = other_markers.read<float>("other-marker x");
    const float y = other_markers.read<float>("other-marker y");
    const float z = other_markers.read<float>("other-marker z");
    frame.markers.emplace_back(x, y, z);
  }

  NatNetFrameCursor rigid_bodies =
    readSizedFrameSection(payload, count, "rigid bodies");
  frame.rigid_bodies.reserve(static_cast<std::size_t>(count));
  for (int32_t i = 0; i < count; ++i) {
    NatNetFrameRigidBody rigid_body{};
    rigid_body.id = rigid_bodies.read<int32_t>("rigid-body id");
    rigid_body.x = rigid_bodies.read<float>("rigid-body x");
    rigid_body.y = rigid_bodies.read<float>("rigid-body y");
    rigid_body.z = rigid_bodies.read<float>("rigid-body z");
    rigid_body.qx = rigid_bodies.read<float>("rigid-body qx");
    rigid_body.qy = rigid_bodies.read<float>("rigid-body qy");
    rigid_body.qz = rigid_bodies.read<float>("rigid-body qz");
    rigid_body.qw = rigid_bodies.read<float>("rigid-body qw");
    rigid_body.mean_marker_error =
      rigid_bodies.read<float>("rigid-body mean marker error");
    const uint16_t params =
      rigid_bodies.read<uint16_t>("rigid-body params");
    rigid_body.tracking_valid = (params & 0x01U) != 0;
    frame.rigid_bodies.push_back(rigid_body);
  }

  // Skeletons and assets are not published by this bridge.
  (void)readSizedFrameSection(payload, count, "skeletons");
  (void)readSizedFrameSection(payload, count, "assets");

  NatNetFrameCursor labeled_markers =
    readSizedFrameSection(payload, count, "labeled markers");
  frame.labeled_markers.reserve(static_cast<std::size_t>(count));
  frame.markers.reserve(
    frame.markers.size() + static_cast<std::size_t>(count));
  for (int32_t i = 0; i < count; ++i) {
    LabeledMarker marker{};
    marker.id = labeled_markers.read<uint32_t>("labeled-marker id");
    marker.modelId = (marker.id >> 16) & 0xffffU;
    marker.memberId = marker.id & 0xffffU;
    const float x = labeled_markers.read<float>("labeled-marker x");
    const float y = labeled_markers.read<float>("labeled-marker y");
    const float z = labeled_markers.read<float>("labeled-marker z");
    marker.position = Eigen::Vector3f(x, y, z);
    marker.size = labeled_markers.read<float>("labeled-marker size");
    marker.params = labeled_markers.read<uint16_t>("labeled-marker params");
    marker.residual = labeled_markers.read<float>("labeled-marker residual");
    frame.markers.push_back(marker.position);
    frame.labeled_markers.push_back(marker);
  }

  // Force plates and legacy devices precede the frame suffix in NatNet 4.1.
  (void)readSizedFrameSection(payload, count, "force plates");
  (void)readSizedFrameSection(payload, count, "devices");

  // Motive 3.5 / NatNet 4.5 appends IMU and GPIO frame sections. Anchor
  // markers add model metadata but no data needed by this bridge. Consume any
  // appended sized sections generically until the fixed 4.1+ suffix remains;
  // this also keeps future additive stream types from shifting timestamps.
  while (payload.remaining() > NATNET_41_FRAME_SUFFIX_BYTES) {
    if (payload.remaining() < NATNET_41_FRAME_SUFFIX_BYTES + 8) {
      throw std::runtime_error(
        "NatNet FRAMEOFDATA extension section overlaps the frame suffix");
    }
    (void)readSizedFrameSection(payload, count, "4.5 extension data");
    ++frame.skipped_extension_sections;
  }
  if (payload.remaining() != NATNET_41_FRAME_SUFFIX_BYTES) {
    throw std::runtime_error("NatNet FRAMEOFDATA suffix size is invalid");
  }

  payload.skip(4, "timecode");
  payload.skip(4, "timecode subframe");
  payload.skip(8, "software timestamp");
  frame.camera_mid_exposure_timestamp =
    payload.read<uint64_t>("camera mid-exposure timestamp");
  frame.camera_data_received_timestamp =
    payload.read<uint64_t>("camera data-received timestamp");
  frame.transmit_timestamp = payload.read<uint64_t>("transmit timestamp");
  frame.precision_timestamp_seconds =
    payload.read<uint32_t>("precision timestamp seconds");
  frame.precision_timestamp_fractional_seconds =
    payload.read<uint32_t>("precision timestamp fractional seconds");
  frame.frame_params = payload.read<uint16_t>("frame params");
  const int32_t end_of_data = payload.read<int32_t>("end-of-data tag");
  if (end_of_data != 0 || payload.remaining() != 0) {
    throw std::runtime_error("NatNet FRAMEOFDATA end-of-data tag is invalid");
  }

  return frame;
}

}  // namespace detail
}  // namespace libmotioncapture
