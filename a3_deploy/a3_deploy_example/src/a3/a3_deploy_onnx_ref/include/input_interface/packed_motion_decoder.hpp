// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Shared pure decoder for the ZMQ packed motion protocol used by A3 teleop.
#pragma once

#include "motion_data_reader.hpp"
#include "input_interface/streamed_motion_merger.hpp"
#include "input_interface/zmq_packed_message_subscriber.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

namespace input_interface {

struct PackedMotionDecodeResult {
  StreamedMotionMerger::IncomingData data;
  std::string error;
};

namespace detail {

inline int FindField(
    const ZMQPackedMessageSubscriber::DecodedHeader& header,
    std::initializer_list<const char*> names) {
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    for (const char* name : names) {
      if (header.fields[i].name == name) return static_cast<int>(i);
    }
  }
  return -1;
}

template <typename T>
inline bool ReadScalar(const std::uint8_t* ptr, bool swap, T& out) {
  std::memcpy(&out, ptr, sizeof(T));
  if (swap) out = byte_swap(out);
  return true;
}

inline bool DecodeFrameIndex(
    const ZMQPackedMessageSubscriber::FieldInfo& field,
    const ZMQPackedMessageSubscriber::BufferView& buf,
    bool swap,
    std::vector<std::int64_t>& out,
    std::string& error) {
  if (field.shape.empty()) {
    error = "frame_index shape must not be empty";
    return false;
  }
  const std::size_t n = field.shape[0];
  out.resize(n);
  const auto* bytes = static_cast<const std::uint8_t*>(buf.data);
  if (field.dtype == "i64") {
    for (std::size_t i = 0; i < n; ++i) {
      std::int64_t v{};
      ReadScalar(bytes + i * sizeof(std::int64_t), swap, v);
      out[i] = v;
    }
  } else if (field.dtype == "i32") {
    for (std::size_t i = 0; i < n; ++i) {
      std::int32_t v{};
      ReadScalar(bytes + i * sizeof(std::int32_t), swap, v);
      out[i] = static_cast<std::int64_t>(v);
    }
  } else {
    error = "frame_index dtype must be i64 or i32";
    return false;
  }
  return true;
}

inline bool DecodeDoubleMatrix(
    const ZMQPackedMessageSubscriber::FieldInfo& field,
    const ZMQPackedMessageSubscriber::BufferView& buf,
    bool swap,
    std::vector<std::vector<double>>& out,
    int& rows,
    int& cols,
    std::string& error) {
  if (field.shape.size() != 2) {
    error = field.name + " shape must be [N,D]";
    return false;
  }
  rows = static_cast<int>(field.shape[0]);
  cols = static_cast<int>(field.shape[1]);
  if (rows <= 0 || cols <= 0) {
    error = field.name + " shape must be positive";
    return false;
  }
  out.assign(rows, std::vector<double>(cols, 0.0));
  const auto* bytes = static_cast<const std::uint8_t*>(buf.data);
  if (field.dtype == "f32") {
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        float v{};
        ReadScalar(bytes + (static_cast<std::size_t>(r) * cols + c) *
                               sizeof(float),
                   swap, v);
        out[r][c] = static_cast<double>(v);
      }
    }
  } else if (field.dtype == "f64") {
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        double v{};
        ReadScalar(bytes + (static_cast<std::size_t>(r) * cols + c) *
                               sizeof(double),
                   swap, v);
        out[r][c] = v;
      }
    }
  } else {
    error = field.name + " dtype must be f32 or f64";
    return false;
  }
  return true;
}

template <std::size_t K>
inline bool DecodeArray3D(
    const ZMQPackedMessageSubscriber::FieldInfo& field,
    const ZMQPackedMessageSubscriber::BufferView& buf,
    bool swap,
    std::vector<std::vector<std::array<double, K>>>& out,
    int& rows,
    int& items,
    std::string& error) {
  if (field.shape.size() == 3 && field.shape[2] == K) {
    rows = static_cast<int>(field.shape[0]);
    items = static_cast<int>(field.shape[1]);
  } else if (field.shape.size() == 2 && field.shape[1] == K) {
    rows = static_cast<int>(field.shape[0]);
    items = 1;
  } else {
    error = field.name + " shape must be [N,D," + std::to_string(K) +
            "] or [N," + std::to_string(K) + "]";
    return false;
  }
  if (rows <= 0 || items <= 0) {
    error = field.name + " shape must be positive";
    return false;
  }
  out.assign(rows, std::vector<std::array<double, K>>(items));
  const auto* bytes = static_cast<const std::uint8_t*>(buf.data);
  const std::size_t stride = static_cast<std::size_t>(items) * K;
  if (field.dtype == "f32") {
    for (int r = 0; r < rows; ++r) {
      for (int item = 0; item < items; ++item) {
        for (std::size_t k = 0; k < K; ++k) {
          float v{};
          ReadScalar(bytes + (static_cast<std::size_t>(r) * stride +
                              static_cast<std::size_t>(item) * K + k) *
                                 sizeof(float),
                     swap, v);
          out[r][item][k] = static_cast<double>(v);
        }
      }
    }
  } else if (field.dtype == "f64") {
    for (int r = 0; r < rows; ++r) {
      for (int item = 0; item < items; ++item) {
        for (std::size_t k = 0; k < K; ++k) {
          double v{};
          ReadScalar(bytes + (static_cast<std::size_t>(r) * stride +
                              static_cast<std::size_t>(item) * K + k) *
                                 sizeof(double),
                     swap, v);
          out[r][item][k] = v;
        }
      }
    }
  } else {
    error = field.name + " dtype must be f32 or f64";
    return false;
  }
  return true;
}

inline bool DecodeBoolField(
    const ZMQPackedMessageSubscriber::FieldInfo& field,
    const ZMQPackedMessageSubscriber::BufferView& buf,
    bool swap,
    bool& out) {
  if (field.shape.empty()) return false;
  const auto* bytes = static_cast<const std::uint8_t*>(buf.data);
  if (field.dtype == "bool" || field.dtype == "u8" || field.dtype == "i8") {
    out = bytes[0] != 0;
    return true;
  }
  if (field.dtype == "i32") {
    std::int32_t v{};
    if (buf.size < sizeof(v)) return false;
    ReadScalar(bytes, swap, v);
    out = v != 0;
    return true;
  }
  if (field.dtype == "i64") {
    std::int64_t v{};
    if (buf.size < sizeof(v)) return false;
    ReadScalar(bytes, swap, v);
    out = v != 0;
    return true;
  }
  return false;
}

}  // namespace detail

inline bool DecodePackedMotionMessage(
    const ZMQPackedMessageSubscriber::DecodedHeader& header,
    const std::vector<ZMQPackedMessageSubscriber::BufferView>& buffers,
    PackedMotionDecodeResult& result) {
  result = {};
  if (header.fields.size() != buffers.size()) {
    result.error = "header field count does not match buffer count";
    return false;
  }
  if (header.version < 1 || header.version > 3) {
    result.error = "unsupported motion protocol version " +
                   std::to_string(header.version);
    return false;
  }

  const bool swap = header.NeedsByteSwap();
  auto& data = result.data;
  data.protocol_version = header.version;

  const int frame_idx = detail::FindField(
      header, {"frame_index", "frame_idx", "last_smpl_global_frames"});
  const int body_quat_idx =
      detail::FindField(header, {"body_quat_w", "body_quat"});
  const int joint_pos_idx = detail::FindField(header, {"joint_pos"});
  const int joint_vel_idx = detail::FindField(header, {"joint_vel"});
  const int smpl_joints_idx = detail::FindField(header, {"smpl_joints"});
  const int smpl_pose_idx = detail::FindField(header, {"smpl_pose"});
  const int catch_up_idx = detail::FindField(header, {"catch_up"});

  if (frame_idx < 0 || body_quat_idx < 0) {
    result.error = "missing required frame_index or body_quat field";
    return false;
  }

  if (!detail::DecodeFrameIndex(header.fields[frame_idx], buffers[frame_idx],
                                swap, data.frame_indices, result.error)) {
    return false;
  }
  data.num_frames = static_cast<int>(data.frame_indices.size());
  if (data.num_frames <= 0) {
    result.error = "frame_index is empty";
    return false;
  }

  int quat_rows = 0;
  if (!detail::DecodeArray3D<4>(header.fields[body_quat_idx],
                                buffers[body_quat_idx], swap, data.body_quat,
                                quat_rows, data.num_quat_bodies,
                                result.error)) {
    return false;
  }
  if (quat_rows != data.num_frames) {
    result.error = "body_quat frame count does not match frame_index";
    return false;
  }

  if (joint_pos_idx >= 0) {
    int rows = 0;
    if (!detail::DecodeDoubleMatrix(header.fields[joint_pos_idx],
                                    buffers[joint_pos_idx], swap,
                                    data.joint_pos, rows, data.num_joints,
                                    result.error)) {
      return false;
    }
    if (rows != data.num_frames) {
      result.error = "joint_pos frame count does not match frame_index";
      return false;
    }
  }
  if (joint_vel_idx >= 0) {
    int rows = 0;
    int vel_cols = 0;
    if (!detail::DecodeDoubleMatrix(header.fields[joint_vel_idx],
                                    buffers[joint_vel_idx], swap,
                                    data.joint_vel, rows, vel_cols,
                                    result.error)) {
      return false;
    }
    if (rows != data.num_frames) {
      result.error = "joint_vel frame count does not match frame_index";
      return false;
    }
    if (data.num_joints == 0) data.num_joints = vel_cols;
  }
  if (data.joint_pos.empty() && !data.joint_vel.empty()) {
    data.joint_pos.assign(data.num_frames,
                          std::vector<double>(data.num_joints, 0.0));
  }
  if (!data.joint_pos.empty() && data.joint_vel.empty()) {
    data.joint_vel.assign(data.num_frames,
                          std::vector<double>(data.num_joints, 0.0));
  }

  if (smpl_joints_idx >= 0) {
    int rows = 0;
    if (!detail::DecodeArray3D<3>(header.fields[smpl_joints_idx],
                                  buffers[smpl_joints_idx], swap,
                                  data.smpl_joints, rows,
                                  data.num_smpl_joints, result.error)) {
      return false;
    }
    if (rows != data.num_frames) {
      result.error = "smpl_joints frame count does not match frame_index";
      return false;
    }
  }
  if (smpl_pose_idx >= 0) {
    int rows = 0;
    if (!detail::DecodeArray3D<3>(header.fields[smpl_pose_idx],
                                  buffers[smpl_pose_idx], swap,
                                  data.smpl_pose, rows, data.num_smpl_poses,
                                  result.error)) {
      return false;
    }
    if (rows != data.num_frames) {
      result.error = "smpl_pose frame count does not match frame_index";
      return false;
    }
  }
  if (catch_up_idx >= 0) {
    bool catch_up = true;
    if (detail::DecodeBoolField(header.fields[catch_up_idx],
                                buffers[catch_up_idx], swap, catch_up)) {
      data.catch_up_enabled = catch_up;
    }
  }
  return true;
}

inline bool DecodePackedMotionMessage(
    const ZMQPackedMessageSubscriber::DecodedHeader& header,
    const std::vector<std::vector<std::uint8_t>>& buffers,
    PackedMotionDecodeResult& result) {
  std::vector<ZMQPackedMessageSubscriber::BufferView> views;
  views.reserve(buffers.size());
  for (const auto& buffer : buffers) {
    views.push_back(ZMQPackedMessageSubscriber::BufferView{
        buffer.data(), buffer.size()});
  }
  return DecodePackedMotionMessage(header, views, result);
}

}  // namespace input_interface
