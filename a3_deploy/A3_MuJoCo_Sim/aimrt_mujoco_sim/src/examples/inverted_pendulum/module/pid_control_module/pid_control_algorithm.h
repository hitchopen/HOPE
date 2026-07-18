// Copyright (c) 2023, AgiBot Inc.
// All rights reserved

#pragma once

#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

struct PIDParm {
  double Kp = 0, Ki = 0, Kd = 0;
};

class PIDController {
 public:
  void SetPIDParm(const PIDParm& parm) { parm_ = parm; }
  PIDParm GetPIDParm() const { return parm_; }

  double Compute(double setpoint, double current_value) {
    // calculate error term
    double error = setpoint - current_value;

    // calculate integral term
    integral += error * dt;

    // calculate derivative term
    double derivative = (error - prev_error) / dt;

    double output = parm_.Kp * error + parm_.Ki * integral + parm_.Kd * derivative;

    prev_error = error;

    return output;
  }

 private:
  PIDParm parm_;
  double dt = 0.001;
  double integral = 0;
  double prev_error = 0;
};

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module