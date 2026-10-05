#pragma once
#include "a3_pingpong/pp_command_transition.hpp"
#include "a3_pingpong/pp_humanlike_policy.hpp"

namespace a3_pingpong {
// Driver-thread-only handoff. Feet must be balanced by the walking controller
// while the arms return from Serve. A long whole-body blend retains stiff
// Stand gains while pulling both loaded feet toward the walking stance.
class PpTeleopEntry {
 public:
  static constexpr double kSupportSeconds = .5;
  static constexpr double kSeconds = 2.;
  void Begin(const robot_io::RobotCommand& source,
             const robot_io::RobotCommand& /*stand*/) {
    source_ = source;
    started_ = false;
    active_ = true;
  }
  bool active() const { return active_; }
  robot_io::RobotCommand Step(double elapsed, const robot_io::RobotState& state,
                             PpHumanLikePolicy& actor, const Eigen::Vector3d& velocity) {
    if (!started_) {
      actor.Reset(state, source_);
      auto target = actor.Step(state, Eigen::Vector3d::Zero());
      support_.Begin(source_, target, kSupportSeconds, .15);
      arms_.Begin(source_, target, kSeconds, .25);
      started_ = true;
      start_s_ = elapsed;
      return source_;
    }
    auto command = actor.Step(state, active_ ? Eigen::Vector3d::Zero() : velocity);
    auto arms = command;
    support_.Apply(elapsed - start_s_, command);
    arms_.Apply(elapsed - start_s_, arms);
    command.q_des.segment(5,14) = arms.q_des.segment(5,14);
    command.dq_des.segment(5,14) = arms.dq_des.segment(5,14);
    command.tau_ff.segment(5,14) = arms.tau_ff.segment(5,14);
    command.kp.segment(5,14) = arms.kp.segment(5,14);
    command.kd.segment(5,14) = arms.kd.segment(5,14);
    active_ = support_.active() || arms_.active();
    return command;
  }
 private:
  PpCommandTransition support_, arms_;
  robot_io::RobotCommand source_;
  double start_s_{0};
  bool active_{false}, started_{false};
};
} // namespace a3_pingpong
