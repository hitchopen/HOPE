// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/common/xmodel_reader.h"
#include <cstddef>

namespace aimrt_mujoco_sim::mujoco_sim_module::common {
std::optional<int32_t> GetJointSensorIdByJointName(const mjModel* m, std::string_view joint_name, mjtSensor sensor_type) {
  int32_t jointId = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (jointId < 0) {
    AIMRT_ERROR_THROW("Invalid joint name: {}, cannot find a joint sensor.", joint_name);
    return std::nullopt;
  }

  for (int32_t i = 0; i < m->nsensor; i++) {
    if (m->sensor_type[i] == sensor_type && m->sensor_objid[i] == jointId) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<int32_t> GetJointposIdByJointName(const mjModel* m, std::string_view joint_name) {
  auto sensor_id = GetJointSensorIdByJointName(m, joint_name, mjSENS_JOINTPOS);
  if (!sensor_id.has_value()) {
    return std::nullopt;
  }
  return m->sensor_adr[sensor_id.value()];
}

std::optional<int32_t> GetJointvelIdByJointName(const mjModel* m, std::string_view joint_name) {
  auto sensor_id = GetJointSensorIdByJointName(m, joint_name, mjSENS_JOINTVEL);
  if (!sensor_id.has_value()) {
    return std::nullopt;
  }
  return m->sensor_adr[sensor_id.value()];
}

std::optional<int32_t> GetJointactfrcIdByJointName(const mjModel* m, std::string_view joint_name) {
  auto sensor_id = GetJointSensorIdByJointName(m, joint_name, mjSENS_JOINTACTFRC);
  if (!sensor_id.has_value()) {
    return std::nullopt;
  }
  return m->sensor_adr[sensor_id.value()];
}

std::optional<std::string> GetJointSensorNameByJointName(const mjModel* m, std::string_view joint_name, mjtSensor sensor_type) {
  if (auto sensor_id = GetJointSensorIdByJointName(m, joint_name, sensor_type)) {
    return mj_id2name(m, mjOBJ_SENSOR, sensor_id.value_or(-1));
  }
  return std::nullopt;
}

std::optional<std::string> GetJointvelNameByJointName(const mjModel* m, std::string_view joint_name) {
  return GetJointSensorNameByJointName(m, joint_name, mjSENS_JOINTVEL);
}

std::optional<std::string> GetJointposNameByJointName(const mjModel* m, std::string_view joint_name) {
  return GetJointSensorNameByJointName(m, joint_name, mjSENS_JOINTPOS);
}

std::optional<std::string> GetJointactfrcNameByJointName(const mjModel* m, std::string_view joint_name) {
  return GetJointSensorNameByJointName(m, joint_name, mjSENS_JOINTACTFRC);
}

std::optional<int32_t> GetJointActIdByJointName(const mjModel* m, std::string_view joint_name) {
  int32_t joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name.data());
  if (joint_id < 0) {
    AIMRT_ERROR_THROW("Invalid joint name: {}, cannot find a joint actuator.", joint_name);
    return std::nullopt;
  }

  for (int i = 0; i < m->nu; i++) {
    if (m->actuator_trntype[i] == mjTRN_JOINT && m->actuator_trnid[static_cast<ptrdiff_t>(i * 2)] == joint_id) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<std::string> GetJointActNameByJointName(const mjModel* m, std::string_view joint_name) {
  if (auto actuator_id = GetJointActIdByJointName(m, joint_name.data())) {
    return mj_id2name(m, mjOBJ_ACTUATOR, actuator_id.value_or(-1));
  }
  return std::nullopt;
}

std::optional<std::string> GetJointActTypeByJointName(const mjModel* m, std::string_view joint_name) {
  if (!m || !joint_name.data()) return std::nullopt;
  for (int32_t i = 0; i < m->nu; i++) {
    int32_t targetId = m->actuator_trnid[static_cast<ptrdiff_t>(i * 2)];
    if (m->actuator_trntype[i] == mjTRN_JOINT) {
      const char* targetName = mj_id2name(m, mjOBJ_JOINT, targetId);
      if (targetName && strcmp(targetName, joint_name.data()) == 0) {
        // position
        if (m->actuator_gaintype[i] == mjGAIN_FIXED &&
            m->actuator_biastype[i] == mjBIAS_AFFINE &&
            (m->actuator_dyntype[i] == mjDYN_NONE ||
             m->actuator_dyntype[i] == mjDYN_FILTEREXACT)) {
          if (m->actuator_biasprm[i * mjNBIAS + 1] == -m->actuator_gainprm[static_cast<ptrdiff_t>(i * mjNGAIN)]) {
            return "position";
          }
        }
        // velocity
        if (m->actuator_gaintype[i] == mjGAIN_FIXED &&
            m->actuator_biastype[i] == mjBIAS_AFFINE &&
            m->actuator_dyntype[i] == mjDYN_NONE) {
          if (m->actuator_biasprm[i * mjNBIAS + 2] == -m->actuator_gainprm[static_cast<ptrdiff_t>(i * mjNGAIN)]) {
            return "velocity";
          }
        }
        // motor
        if (m->actuator_dyntype[i] == mjDYN_NONE &&
            m->actuator_gaintype[i] == mjGAIN_FIXED &&
            m->actuator_biastype[i] == mjBIAS_NONE) {
          if (m->actuator_gainprm[static_cast<ptrdiff_t>(i * mjNGAIN)] == 1) {
            return "motor";
          }
        }
        // other (todo:(hj) adapt to other types of actuators)
      }
    }
  }
  // cannot find the joint name in the actuators
  AIMRT_ERROR_THROW("The actuator type bound to joint {} is unsupported, only support position/velocity/motor.", joint_name);
  return std::nullopt;
}

std::optional<int32_t> GetSensorIdBySensorName(const mjModel* m, std::string_view sensor_name) {
  if (sensor_name.empty()) return std::nullopt;

  int32_t sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_name.data());
  if (sensor_id < 0) {
    AIMRT_ERROR_THROW("Invalid sensor name: {}.", sensor_name);
    return std::nullopt;
  }

  return m->sensor_adr[sensor_id];
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common