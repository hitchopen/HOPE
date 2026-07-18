// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstring>
#include "mujoco/mjmodel.h"
#include "mujoco_sim_module/global.h"
#include "simulate.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::common {
// joint sensor
std::optional<int32_t> GetJointSensorIdByJointName(const mjModel* m, std::string_view joint_name, mjtSensor sensor_type);
std::optional<int32_t> GetJointvelIdByJointName(const mjModel* m, std::string_view joint_name);
std::optional<int32_t> GetJointposIdByJointName(const mjModel* m, std::string_view joint_name);
std::optional<int32_t> GetJointactfrcIdByJointName(const mjModel* m, std::string_view joint_name);

std::optional<std::string> GetJointSensorNameByJointName(const mjModel* m, std::string_view joint_name, mjtSensor sensor_type);
std::optional<std::string> GetJointvelNameByJointName(const mjModel* m, std::string_view joint_name);
std::optional<std::string> GetJointposNameByJointName(const mjModel* m, std::string_view joint_name);
std::optional<std::string> GetJointactfrcNameByJointName(const mjModel* m, std::string_view joint_name);

// joint actuator
std::optional<int32_t> GetJointActIdByJointName(const mjModel* m, std::string_view joint_name);
std::optional<std::string> GetJointActNameByJointName(const mjModel* m, std::string_view joint_name);
std::optional<std::string> GetJointActTypeByJointName(const mjModel* m, std::string_view joint_name);

// normal sensor
std::optional<int32_t> GetSensorIdBySensorName(const mjModel* m, std::string_view sensor_name);
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common
