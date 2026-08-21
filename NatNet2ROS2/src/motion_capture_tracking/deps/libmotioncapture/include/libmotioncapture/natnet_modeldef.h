#pragma once

#include "libmotioncapture/motioncapture.h"

#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace libmotioncapture {
namespace detail {

constexpr uint16_t NAT_MODELDEF_MESSAGE_ID = 5;
constexpr int32_t NATNET_MAX_MODELDEF_DATASETS = 10000;
constexpr int32_t NATNET_MAX_RIGID_BODY_MARKERS = 10000;

class NatNetPacketCursor
{
public:
  NatNetPacketCursor(const char* begin, const char* end)
    : current_(begin), end_(end)
  {
    if (begin == nullptr || end == nullptr || begin > end) {
      throw std::runtime_error("NatNet MODELDEF packet bounds are invalid");
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

  std::string readString(const char* field)
  {
    const void* terminator = std::memchr(
      current_, '\0', static_cast<std::size_t>(end_ - current_));
    if (terminator == nullptr) {
      throw std::runtime_error(
        std::string("NatNet MODELDEF unterminated string: ") + field);
    }
    const char* string_end = static_cast<const char*>(terminator);
    std::string value(current_, string_end);
    current_ = string_end + 1;
    return value;
  }

  void skip(std::size_t bytes, const char* field)
  {
    require(bytes, field);
    current_ += bytes;
  }

  const char* current() const
  {
    return current_;
  }

  const char* end() const
  {
    return end_;
  }

  void advanceTo(const char* target, const char* field)
  {
    if (target < current_ || target > end_) {
      throw std::runtime_error(
        std::string("NatNet MODELDEF invalid dataset boundary: ") + field);
    }
    current_ = target;
  }

private:
  void require(std::size_t bytes, const char* field) const
  {
    if (bytes > static_cast<std::size_t>(end_ - current_)) {
      throw std::runtime_error(
        std::string("NatNet MODELDEF truncated while reading ") + field);
    }
  }

  const char* current_;
  const char* end_;
};

inline bool natNetVersionAtLeast(
  int major, int minor, int required_major, int required_minor)
{
  return major > required_major ||
         (major == required_major && minor >= required_minor);
}

inline std::map<int32_t, RigidBodyDefinition> parseNatNetModelDef(
  const char* data, std::size_t length, int major, int minor)
{
  if (data == nullptr || length < 8) {
    throw std::runtime_error("NatNet MODELDEF packet is shorter than its header");
  }

  NatNetPacketCursor packet(data, data + length);
  const uint16_t message_id = packet.read<uint16_t>("message id");
  const uint16_t payload_size = packet.read<uint16_t>("payload size");
  if (message_id != NAT_MODELDEF_MESSAGE_ID) {
    throw std::runtime_error("NatNet packet is not a MODELDEF message");
  }
  if (static_cast<std::size_t>(payload_size) > length - 4) {
    throw std::runtime_error("NatNet MODELDEF payload exceeds the UDP datagram");
  }

  NatNetPacketCursor payload(packet.current(), data + 4 + payload_size);
  const int32_t dataset_count = payload.read<int32_t>("dataset count");
  if (dataset_count < 0 || dataset_count > NATNET_MAX_MODELDEF_DATASETS) {
    throw std::runtime_error("NatNet MODELDEF dataset count is invalid");
  }

  const bool has_dataset_sizes = natNetVersionAtLeast(major, minor, 4, 1);
  const bool has_rotation_offset = natNetVersionAtLeast(major, minor, 4, 2);
  std::map<int32_t, RigidBodyDefinition> definitions;

  for (int32_t dataset_index = 0;
       dataset_index < dataset_count; ++dataset_index) {
    const int32_t type = payload.read<int32_t>("dataset type");
    const char* dataset_end = payload.end();
    if (has_dataset_sizes) {
      const int32_t description_size =
        payload.read<int32_t>("dataset description size");
      if (description_size < 0 ||
          static_cast<std::size_t>(description_size) >
            static_cast<std::size_t>(payload.end() - payload.current())) {
        throw std::runtime_error(
          "NatNet MODELDEF dataset description size is invalid");
      }
      dataset_end = payload.current() + description_size;
    }

    NatNetPacketCursor dataset(payload.current(), dataset_end);
    if (type == 0) {
      dataset.readString("marker-set name");
      const int32_t marker_count = dataset.read<int32_t>("marker-set count");
      if (marker_count < 0 || marker_count > NATNET_MAX_RIGID_BODY_MARKERS) {
        throw std::runtime_error("NatNet marker-set marker count is invalid");
      }
      for (int32_t marker_index = 0;
           marker_index < marker_count; ++marker_index) {
        dataset.readString("marker-set marker name");
      }
    } else if (type == 1) {
      RigidBodyDefinition definition{};
      definition.name = major >= 2
        ? dataset.readString("rigid-body name") : std::string();
      definition.id = dataset.read<int32_t>("rigid-body id");
      definition.parentId = dataset.read<int32_t>("rigid-body parent id");
      const float x_offset = dataset.read<float>("rigid-body x offset");
      const float y_offset = dataset.read<float>("rigid-body y offset");
      const float z_offset = dataset.read<float>("rigid-body z offset");
      definition.parentOffset =
        Eigen::Vector3f(x_offset, y_offset, z_offset);
      definition.parentRotationOffset = Eigen::Quaternionf::Identity();

      // NatNet 4.2 added this quaternion to sRigidBodyDescription. Multicast
      // always uses the Motive-installed bitstream, so it cannot be omitted by
      // negotiating an older version.
      if (has_rotation_offset) {
        const float qx = dataset.read<float>("rigid-body rotation offset qx");
        const float qy = dataset.read<float>("rigid-body rotation offset qy");
        const float qz = dataset.read<float>("rigid-body rotation offset qz");
        const float qw = dataset.read<float>("rigid-body rotation offset qw");
        definition.parentRotationOffset = Eigen::Quaternionf(qw, qx, qy, qz);
      }

      if (major >= 3) {
        const int32_t marker_count =
          dataset.read<int32_t>("rigid-body marker count");
        if (marker_count < 0 ||
            marker_count > NATNET_MAX_RIGID_BODY_MARKERS) {
          throw std::runtime_error(
            "NatNet rigid-body marker count is invalid");
        }
        definition.markers.resize(static_cast<std::size_t>(marker_count));
        for (int32_t marker_index = 0;
             marker_index < marker_count; ++marker_index) {
          const float x = dataset.read<float>("rigid-body marker x");
          const float y = dataset.read<float>("rigid-body marker y");
          const float z = dataset.read<float>("rigid-body marker z");
          auto& marker = definition.markers[static_cast<std::size_t>(marker_index)];
          // FRAMEOFDATA encodes the marker's 1-based member ID in the low
          // word of the labeled-marker ID. MODELDEF supplies only the marker
          // array order, so synthesize the matching wire-level member ID.
          marker.memberId = static_cast<uint32_t>(marker_index + 1);
          marker.position = Eigen::Vector3f(x, y, z);
          marker.requiredActiveLabel = -1;
        }
        for (int32_t marker_index = 0;
             marker_index < marker_count; ++marker_index) {
          definition.markers[static_cast<std::size_t>(marker_index)]
            .requiredActiveLabel =
              dataset.read<int32_t>("rigid-body marker active label");
        }
        if (major >= 4) {
          for (int32_t marker_index = 0;
               marker_index < marker_count; ++marker_index) {
            definition.markers[static_cast<std::size_t>(marker_index)].name =
              dataset.readString("rigid-body marker name");
          }
        }
      }
      definitions[definition.id] = std::move(definition);
    } else if (!has_dataset_sizes) {
      throw std::runtime_error(
        "NatNet MODELDEF contains an unsupported unsized dataset type");
    }

    // For NatNet >= 4.1, description_size is authoritative. Skipping any
    // unconsumed suffix preserves alignment when OptiTrack extends a known
    // dataset in a future bitstream.
    if (has_dataset_sizes) {
      payload.advanceTo(dataset_end, "dataset");
    } else {
      payload.advanceTo(dataset.current(), "legacy dataset");
    }
  }

  return definitions;
}

}  // namespace detail
}  // namespace libmotioncapture
