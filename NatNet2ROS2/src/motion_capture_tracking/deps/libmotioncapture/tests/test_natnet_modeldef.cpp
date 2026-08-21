#include "libmotioncapture/natnet_modeldef.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

template<typename T>
void append(std::vector<char>& packet, const T& value)
{
  const char* bytes = reinterpret_cast<const char*>(&value);
  packet.insert(packet.end(), bytes, bytes + sizeof(T));
}

void appendString(std::vector<char>& packet, const std::string& value)
{
  packet.insert(packet.end(), value.begin(), value.end());
  packet.push_back('\0');
}

void patchInt32(std::vector<char>& packet, std::size_t offset, int32_t value)
{
  assert(offset + sizeof(value) <= packet.size());
  std::memcpy(packet.data() + offset, &value, sizeof(value));
}

std::size_t beginPacket(std::vector<char>& packet, int32_t dataset_count)
{
  append<uint16_t>(packet, libmotioncapture::detail::NAT_MODELDEF_MESSAGE_ID);
  append<uint16_t>(packet, 0);
  append<int32_t>(packet, dataset_count);
  return packet.size();
}

std::size_t beginDataset(std::vector<char>& packet, int32_t type)
{
  append<int32_t>(packet, type);
  const std::size_t size_offset = packet.size();
  append<int32_t>(packet, 0);
  return size_offset;
}

void endDataset(std::vector<char>& packet, std::size_t size_offset)
{
  const std::size_t description_begin = size_offset + sizeof(int32_t);
  patchInt32(
    packet, size_offset,
    static_cast<int32_t>(packet.size() - description_begin));
}

void endPacket(std::vector<char>& packet)
{
  const uint16_t payload_size = static_cast<uint16_t>(packet.size() - 4);
  std::memcpy(packet.data() + 2, &payload_size, sizeof(payload_size));
}

void appendRigidBody(
  std::vector<char>& packet,
  const std::string& name,
  int32_t id,
  bool include_rotation_offset)
{
  appendString(packet, name);
  append<int32_t>(packet, id);
  append<int32_t>(packet, -1);
  append<float>(packet, 0.1f);
  append<float>(packet, -0.2f);
  append<float>(packet, 0.3f);
  if (include_rotation_offset) {
    append<float>(packet, 0.1f);
    append<float>(packet, 0.2f);
    append<float>(packet, 0.3f);
    append<float>(packet, 0.9f);
  }

  append<int32_t>(packet, 2);
  append<float>(packet, 1.0f);
  append<float>(packet, 2.0f);
  append<float>(packet, 3.0f);
  append<float>(packet, 4.0f);
  append<float>(packet, 5.0f);
  append<float>(packet, 6.0f);
  append<int32_t>(packet, 1001);
  append<int32_t>(packet, 1002);
  appendString(packet, "marker_0");
  appendString(packet, "marker_1");
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
  using libmotioncapture::detail::parseNatNetModelDef;

  std::vector<char> natnet_41;
  beginPacket(natnet_41, 1);
  const std::size_t ball_size = beginDataset(natnet_41, 1);
  appendRigidBody(natnet_41, "Ball", 101, false);
  endDataset(natnet_41, ball_size);
  endPacket(natnet_41);

  const auto definitions_41 =
    parseNatNetModelDef(natnet_41.data(), natnet_41.size(), 4, 1);
  assert(definitions_41.size() == 1);
  const auto& ball = definitions_41.at(101);
  assert(ball.name == "Ball");
  assert(ball.parentId == -1);
  assert(near(ball.parentOffset.x(), 0.1f));
  assert(near(ball.parentRotationOffset.w(), 1.0f));
  assert(ball.markers.size() == 2);
  assert(ball.markers[0].name == "marker_0");
  assert(ball.markers[1].requiredActiveLabel == 1002);
  assert(near(ball.markers[1].position.z(), 6.0f));

  // The version gate is load-bearing: treating a 4.1 description as 4.2
  // consumes the marker count and first marker coordinates as a quaternion,
  // so the packet must fail rather than appear to decode successfully.
  assert(throwsRuntimeError([&]() {
    parseNatNetModelDef(natnet_41.data(), natnet_41.size(), 4, 2);
  }));

  // Include an unknown sized dataset first. A 4.2 decoder must skip it using
  // description_size and still consume the rigid-body rotation quaternion.
  std::vector<char> natnet_42;
  beginPacket(natnet_42, 2);
  const std::size_t unknown_size = beginDataset(natnet_42, 99);
  append<int32_t>(natnet_42, 0x12345678);
  endDataset(natnet_42, unknown_size);
  const std::size_t p1_size = beginDataset(natnet_42, 1);
  appendRigidBody(natnet_42, "P1", 202, true);
  endDataset(natnet_42, p1_size);
  endPacket(natnet_42);

  const auto definitions_42 =
    parseNatNetModelDef(natnet_42.data(), natnet_42.size(), 4, 2);
  assert(definitions_42.size() == 1);
  const auto& p1 = definitions_42.at(202);
  assert(p1.name == "P1");
  assert(near(p1.parentRotationOffset.x(), 0.1f));
  assert(near(p1.parentRotationOffset.y(), 0.2f));
  assert(near(p1.parentRotationOffset.z(), 0.3f));
  assert(near(p1.parentRotationOffset.w(), 0.9f));
  assert(p1.markers.size() == 2);
  assert(p1.markers[0].requiredActiveLabel == 1001);

  std::vector<char> truncated = natnet_42;
  truncated.pop_back();
  assert(throwsRuntimeError([&]() {
    parseNatNetModelDef(truncated.data(), truncated.size(), 4, 2);
  }));

  std::vector<char> invalid_description = natnet_42;
  patchInt32(invalid_description, 12, 0x7fffffff);
  assert(throwsRuntimeError([&]() {
    parseNatNetModelDef(
      invalid_description.data(), invalid_description.size(), 4, 2);
  }));

  return 0;
}
