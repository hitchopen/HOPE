// Local simulation ABI. Links the production controller; has no HAL or transport.
#include "a3_pingpong/pp_serve_controller.hpp"
#include "a3_pingpong/pp_humanlike_policy.hpp"
#include "a3_pingpong/pp_policy.hpp"
#include "a3_pingpong/pp_hybrid_lower_policy.hpp"
#include "a3_pingpong/pp_runner_control.hpp"
#include <memory>
#include <string>

namespace {
using robot_io::RobotCommand;

struct ReceiveProbe {
  bool kernel = false;
  bool has_torso = false;
  Eigen::Vector4d torso = Eigen::Vector4d(1,0,0,0);
  std::uint64_t base_sequence = 0;
    std::unique_ptr<a3_pingpong::PpPolicy> policy;
    std::shared_ptr<a3_pingpong::PpBasePoseInput> base = std::make_shared<a3_pingpong::PpBasePoseInput>();
};

struct Probe {
    std::unique_ptr<a3_pingpong::PpServeController> serve;
    a3_pingpong::PpCommandTransition transition;
    a3_pingpong::PpRunnerControl control{a3_pingpong::RunnerMode::kPdStand, 1, "simulation"};
    bool managed = false;
};

RobotCommand Read(const double* p) {
  RobotCommand c;
  c.q_des = Eigen::Map<const Eigen::VectorXd>(p, 31);
  c.dq_des = Eigen::Map<const Eigen::VectorXd>(p + 31, 31);
  c.tau_ff = Eigen::Map<const Eigen::VectorXd>(p + 62, 31);
  c.kp = Eigen::Map<const Eigen::VectorXd>(p + 93, 31);
  c.kd = Eigen::Map<const Eigen::VectorXd>(p + 124, 31);
  return c;
}

void Write(const RobotCommand& c, double* p) {
  Eigen::Map<Eigen::VectorXd>(p, 31) = c.q_des;
  Eigen::Map<Eigen::VectorXd>(p + 31, 31) = c.dq_des;
  Eigen::Map<Eigen::VectorXd>(p + 62, 31) = c.tau_ff;
  Eigen::Map<Eigen::VectorXd>(p + 93, 31) = c.kp;
  Eigen::Map<Eigen::VectorXd>(p + 124, 31) = c.kd;
}
} // namespace

extern "C" {
// Actual receive policy in planner waiting mode. There is deliberately no
// incoming ball in these handoff tests; swing performance is a separate test.
void* receive_create_mode(const char* onnx, const char* config, int kernel) {
  try {
    a3_pingpong::PpPolicyConfig cfg;
    cfg.planner_mode = true;
    cfg.policy_native = true;
    cfg.single_swing = true;
    cfg.level = 0;
    cfg.swing_rest_s = .5;
    cfg.command_timeout_s = .5;
    cfg.kernel_mode = kernel != 0;
    cfg.loc_mode = kernel ? a3_pingpong::LocMode::kKernelImuLocal : a3_pingpong::LocMode::kExternalBase;
    cfg.deploy_cfg_path = config;
    auto probe = std::make_unique<ReceiveProbe>();
    probe->kernel = kernel != 0;
    probe->policy = std::make_unique<a3_pingpong::PpPolicy>(onnx, cfg);
    probe->policy->SetBasePoseInput(probe->base);
    probe->policy->SetRacketInput(std::make_shared<a3_pingpong::PpRacketTargetInput>());
    return probe.release();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return nullptr;
  }
}

void* receive_create(const char* onnx, const char* config) { return receive_create_mode(onnx, config, 0); }
void* receive_create_kernel(const char* onnx, const char* config) { return receive_create_mode(onnx, config, 1); }
void receive_set_torso(void* p, const double* q) {
  auto& probe = *static_cast<ReceiveProbe*>(p);
  probe.has_torso = true;
  probe.torso = Eigen::Map<const Eigen::Vector4d>(q);
}
void receive_destroy(void* p) { delete static_cast<ReceiveProbe*>(p); }
int probe_return_seconds(void* p, double seconds) {
  try { static_cast<Probe*>(p)->serve->SetPolicyReturnSeconds(seconds); return 0; }
  catch (...) { return -1; }
}
int receive_observation_size(void* p) {
  return static_cast<ReceiveProbe*>(p)->policy->last_obs_unsafe().size();
}
int receive_localization_fresh(void* p) {
  return static_cast<ReceiveProbe*>(p)->policy->planner_trace_snapshot(0).localization_fresh;
}
void receive_stand_command(void* p, double* out) {
  const auto& policy = *static_cast<ReceiveProbe*>(p)->policy;
  RobotCommand command;
  command.q_des = policy.official_stand_q();
  command.dq_des = Eigen::VectorXd::Zero(31);
  command.tau_ff = Eigen::VectorXd::Zero(31);
  command.kp = policy.official_stand_kp();
  command.kd = policy.official_stand_kd();
  Write(command, out);
}

void receive_rearm(void* p, int from_serve) {
  auto& policy = *static_cast<ReceiveProbe*>(p)->policy;
  (void)from_serve;  // Native receive uses the same reset after measured stand recovery.
  policy.rearm_yaw_align();
}
void receive_delivered(void* p, const double* command, int sent) {
  static_cast<ReceiveProbe*>(p)->policy->RecordCommandDelivery(Read(command), sent != 0);
}


void* humanlike_create(const char* directory) {
  try {
    return new a3_pingpong::PpHumanLikePolicy(directory);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return nullptr;
  }
}

void humanlike_destroy(void* p) { delete static_cast<a3_pingpong::PpHumanLikePolicy*>(p); }

robot_io::RobotState State(const double* p) {
  robot_io::RobotState s;
  s.q = Eigen::Map<const Eigen::VectorXd>(p, 31);
  s.dq = Eigen::Map<const Eigen::VectorXd>(p + 31, 31);
  s.imu_quat_wxyz = Eigen::Map<const Eigen::Vector4d>(p + 62);
  s.imu_gyro = Eigen::Map<const Eigen::Vector3d>(p + 66);
  return s;
}

int receive_step(void* p, unsigned tick, const double* state, int handoff, double* out) {
  try {
    auto& probe = *static_cast<ReceiveProbe*>(p);
    auto& policy = *probe.policy;
    const double now = a3_pingpong::PpNowWallSec();
    const double sec = std::floor(now);
    if (!probe.kernel) probe.base->SetFromFlat({2, 1, static_cast<double>(++probe.base_sequence), sec, std::floor((now - sec) * 1e9), state[69],
                             state[70], state[71], state[62], state[63], state[64], state[65], 1,
                             static_cast<double>(a3_pingpong::kV17RequiredBaseFlags), 1, 1});
    policy.set_mode_handoff_hold(handoff != 0);
    RobotCommand command;
    auto measured = State(state);
    measured.has_secondary_imu = probe.has_torso;
    measured.sec_imu_quat_wxyz = probe.torso;
    if (!policy.ComputeCommand(tick, measured, command)) return -1;
    Write(command, out);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return -1;
  }
}

void* lower_create(const char* path) {
  try {
    a3_pingpong::PpHybridLowerConfig cfg;
    cfg.model_path = path;
    cfg.owner = a3_pingpong::PpHybridLowerOwner::kWaistRollPitchAndLegs;
    auto lower = std::make_unique<a3_pingpong::PpHybridLowerPolicy>(cfg);
    if (!lower->Initialize()) return nullptr;
    return lower.release();
  } catch (...) { return nullptr; }
}
void lower_destroy(void* p) { delete static_cast<a3_pingpong::PpHybridLowerPolicy*>(p); }
int lower_step(void* p, const double* state, double* command) {
  auto c = Read(command);
  if (!static_cast<a3_pingpong::PpHybridLowerPolicy*>(p)->Apply(State(state), c)) return -1;
  Write(c, command);
  return 0;
}
int humanlike_reset(void* p, const double* state, const double* entry) {
  try {
    static_cast<a3_pingpong::PpHumanLikePolicy*>(p)->Reset(State(state), Read(entry));
    return 0;
  } catch (...) { return -1; }
}

int humanlike_step(void* p, const double* state, const double* velocity, double* out) {
  try {
    auto& policy = *static_cast<a3_pingpong::PpHumanLikePolicy*>(p);
    Write(policy.Step(State(state), Eigen::Map<const Eigen::Vector3d>(velocity)), out);
    return policy.settled() ? 1 : 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return -1;
  }
}

void* probe_create(const char* csv, const double* stand) {
  try {
    a3_pingpong::PpServe025FullbodyTimeline timeline;
    std::string error;
    if (!timeline.LoadCsv(csv, error)) return nullptr;
    auto p = std::make_unique<Probe>();
    auto c = Read(stand);
    p->serve = std::make_unique<a3_pingpong::PpServeController>(std::move(timeline), c.q_des, c.kp, c.kd, nullptr);
    return p.release();
  } catch (...) { return nullptr; }
}

int probe_handoff_frame(void* v, int frame) {
  try { static_cast<Probe*>(v)->serve->SetPolicyHandoffFrame(frame); return 0; }
  catch (...) { return -1; }
}
void probe_destroy(void* v) { delete static_cast<Probe*>(v); }

// Exercise the real action admission and repeated serve lifecycle, including
// the pending prepare consumed on the command thread (no gripper transport).
int probe_control_action(void* v, int action) {
  auto& p = *static_cast<Probe*>(v);
  p.managed = true;
  if (!p.control.EnqueueLocalAction(static_cast<a3_pingpong::RunnerAction>(action))) return -1;
  auto d = p.control.ProcessPending(false, p.serve->active(), true,
                                    static_cast<int>(p.serve->state()), true, -1, false, true, true);
  if (d.size() != 1) return -1;
  if (d[0].request_ready_to_serve) p.serve->TriggerReadyToServe();
  if (d[0].request_serve_abort) p.serve->RequestAbort();
  return static_cast<int>(d[0].result);
}
int probe_mode(void* v) { return static_cast<int>(static_cast<Probe*>(v)->control.mode()); }
void probe_runner_state(void* v, double* out) {
  auto& p = *static_cast<Probe*>(v);
  p.control.ObserveExternalState(true, true, false, true,
                                static_cast<int>(p.serve->state()), -1, false);
  const auto state = p.control.EncodeState();
  std::copy(state.begin(), state.end(), out);
}
int probe_stop_requested(void* v) {
  return static_cast<Probe*>(v)->control.TeleopStopRequested();
}
void probe_finish_teleop_stop(void* v) {
  static_cast<Probe*>(v)->control.SetRuntimeMode(a3_pingpong::RunnerMode::kPdStand);
}

int probe_action(void* v, int action, const double* entry) {
  try {
    auto& c = *static_cast<Probe*>(v)->serve;
    if (action == 0) {
      c.SetEntryCommand(Read(entry));
      c.Start();
    } else if (action == 1) c.TriggerReadyToServe();
    else if (action == 2) c.RequestAbort();
    else return -1;
    return 0;
  } catch (...) { return -1; }
}

int probe_serve_with_gyro(void* v, const double* q, const double* dq, const double* quat,
                          const double* gyro, double* out) {
  try {
    robot_io::RobotState s;
    s.q = Eigen::Map<const Eigen::VectorXd>(q, 31);
    s.dq = Eigen::Map<const Eigen::VectorXd>(dq, 31);
    s.imu_quat_wxyz = Eigen::Map<const Eigen::Vector4d>(quat);
    s.imu_gyro = Eigen::Map<const Eigen::Vector3d>(gyro);
    RobotCommand c;
    auto& p = *static_cast<Probe*>(v);
    auto& serve = *p.serve;
    if (p.managed && p.control.ConsumeServePrepare()) {
      serve.SetEntryCommand(Read(out));
      serve.Start();
    }
    if (!serve.ComputeCommand(0, s, c)) return -1;
    Write(c, out);
    if (p.managed && serve.state() == a3_pingpong::ServeControllerState::kComplete)
      p.control.CompleteServe();
    return static_cast<int>(serve.state());
  } catch (...) { return -1; }
}

int probe_serve(void* v, const double* q, const double* dq, const double* quat, double* out) {
  const double gyro[3] = {0, 0, 0};
  return probe_serve_with_gyro(v, q, dq, quat, gyro, out);
}

int probe_begin(void* v, const double* source, const double* target, double duration) {
  try {
    static_cast<Probe*>(v)->transition.Begin(Read(source), Read(target), duration);
    return 0;
  } catch (...) { return -1; }
}

int probe_apply(void* v, double elapsed, double* target) {
  try {
    auto c = Read(target);
    static_cast<Probe*>(v)->transition.Apply(elapsed, c);
    Write(c, target);
    return 0;
  } catch (...) { return -1; }
}
}
