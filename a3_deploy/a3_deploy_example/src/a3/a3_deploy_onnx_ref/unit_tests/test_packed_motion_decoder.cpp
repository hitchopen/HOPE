#include "input_interface/packed_motion_decoder.hpp"
#include "input_interface/zmq_endpoint_interface.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

using BufferView = ZMQPackedMessageSubscriber::BufferView;
using DecodedHeader = ZMQPackedMessageSubscriber::DecodedHeader;
using FieldInfo = ZMQPackedMessageSubscriber::FieldInfo;

template <typename T>
std::vector<std::uint8_t> BytesOf(const std::vector<T>& values) {
  std::vector<std::uint8_t> bytes(values.size() * sizeof(T));
  std::memcpy(bytes.data(), values.data(), bytes.size());
  return bytes;
}

FieldInfo Field(std::string name,
                std::string dtype,
                std::vector<std::size_t> shape) {
  FieldInfo f;
  f.name = std::move(name);
  f.dtype = std::move(dtype);
  f.shape = std::move(shape);
  return f;
}

std::vector<BufferView> Views(const std::vector<std::vector<std::uint8_t>>& b) {
  std::vector<BufferView> out;
  out.reserve(b.size());
  for (const auto& bytes : b) {
    out.push_back(BufferView{bytes.data(), bytes.size()});
  }
  return out;
}

}  // namespace

TEST(PackedMotionDecoder, DecodesV3MotionProtocol) {
  DecodedHeader header;
  header.version = 3;
  header.endian = "le";
  header.fields = {
      Field("frame_index", "i64", {2}),
      Field("body_quat_w", "f32", {2, 1, 4}),
      Field("joint_pos", "f64", {2, 29}),
      Field("joint_vel", "f32", {2, 29}),
      Field("smpl_joints", "f32", {2, 24, 3}),
      Field("smpl_pose", "f64", {2, 24, 3}),
      Field("catch_up", "i32", {1}),
  };

  std::vector<float> body_quat = {
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f,
  };
  std::vector<double> joint_pos(2 * 29);
  std::vector<float> joint_vel(2 * 29);
  std::vector<float> smpl_joints(2 * 24 * 3);
  std::vector<double> smpl_pose(2 * 24 * 3);
  for (std::size_t i = 0; i < joint_pos.size(); ++i) {
    joint_pos[i] = 10.0 + static_cast<double>(i);
    joint_vel[i] = 20.0f + static_cast<float>(i);
  }
  for (std::size_t i = 0; i < smpl_joints.size(); ++i) {
    smpl_joints[i] = 30.0f + static_cast<float>(i);
    smpl_pose[i] = 40.0 + static_cast<double>(i);
  }

  const std::vector<std::vector<std::uint8_t>> storage = {
      BytesOf<std::int64_t>({100, 102}),
      BytesOf(body_quat),
      BytesOf(joint_pos),
      BytesOf(joint_vel),
      BytesOf(smpl_joints),
      BytesOf(smpl_pose),
      BytesOf<std::int32_t>({0}),
  };

  input_interface::PackedMotionDecodeResult decoded;
  ASSERT_TRUE(input_interface::DecodePackedMotionMessage(
      header, Views(storage), decoded)) << decoded.error;

  const auto& data = decoded.data;
  EXPECT_EQ(data.protocol_version, 3);
  ASSERT_EQ(data.frame_indices.size(), 2u);
  EXPECT_EQ(data.frame_indices[0], 100);
  EXPECT_EQ(data.frame_indices[1], 102);
  EXPECT_EQ(data.num_frames, 2);
  EXPECT_EQ(data.num_quat_bodies, 1);
  EXPECT_EQ(data.num_joints, 29);
  EXPECT_EQ(data.num_smpl_joints, 24);
  EXPECT_EQ(data.num_smpl_poses, 24);
  EXPECT_FALSE(data.catch_up_enabled);
  EXPECT_DOUBLE_EQ(data.body_quat[1][0][1], 1.0);
  EXPECT_DOUBLE_EQ(data.joint_pos[1][0], 39.0);
  EXPECT_DOUBLE_EQ(data.joint_vel[1][0], 49.0);
  EXPECT_DOUBLE_EQ(data.smpl_joints[1][0][0], 102.0);
  EXPECT_DOUBLE_EQ(data.smpl_pose[1][0][0], 112.0);
}

TEST(PackedMotionDecoder, AcceptsA3LegacySingleItemShapesAndFrameAlias) {
  DecodedHeader header;
  header.version = 2;
  header.endian = "le";
  header.fields = {
      Field("last_smpl_global_frames", "i32", {1}),
      Field("body_quat", "f64", {1, 4}),
      Field("smpl_joints", "f64", {1, 3}),
      Field("smpl_pose", "f32", {1, 3}),
  };

  const std::vector<std::vector<std::uint8_t>> storage = {
      BytesOf<std::int32_t>({7}),
      BytesOf<double>({0.5, 0.5, 0.5, 0.5}),
      BytesOf<double>({1.0, 2.0, 3.0}),
      BytesOf<float>({4.0f, 5.0f, 6.0f}),
  };

  input_interface::PackedMotionDecodeResult decoded;
  ASSERT_TRUE(input_interface::DecodePackedMotionMessage(
      header, Views(storage), decoded)) << decoded.error;

  const auto& data = decoded.data;
  EXPECT_EQ(data.protocol_version, 2);
  ASSERT_EQ(data.frame_indices.size(), 1u);
  EXPECT_EQ(data.frame_indices[0], 7);
  EXPECT_EQ(data.num_frames, 1);
  EXPECT_EQ(data.num_quat_bodies, 1);
  EXPECT_EQ(data.num_smpl_joints, 1);
  EXPECT_EQ(data.num_smpl_poses, 1);
  EXPECT_DOUBLE_EQ(data.body_quat[0][0][3], 0.5);
  EXPECT_DOUBLE_EQ(data.smpl_joints[0][0][2], 3.0);
  EXPECT_DOUBLE_EQ(data.smpl_pose[0][0][1], 5.0);
}
