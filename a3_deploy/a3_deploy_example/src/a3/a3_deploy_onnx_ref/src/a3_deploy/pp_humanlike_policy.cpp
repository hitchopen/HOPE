#include "a3_pingpong/pp_humanlike_policy.hpp"
#include "a3_pingpong/pp_humanlike_contract.hpp"
#include "a3_pingpong/pp_command_transition.hpp"
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <vector>

namespace a3_pingpong {
namespace {
Eigen::Vector3d Vector3(const YAML::Node& n) {
  if (!n.IsSequence() || n.size() != 3) throw std::runtime_error("HumanLike requires a 3-vector");
  Eigen::Vector3d v;
  for (int i = 0; i < 3; ++i) v[i] = n[i].as<double>();
  if (!v.allFinite()) throw std::runtime_error("nonfinite HumanLike config");
  return v;
}

Eigen::VectorXd JointVector(const YAML::Node& n) {
  if (!n.IsSequence() || n.size() != 6) throw std::runtime_error("HumanLike requires six joint groups");
  static const std::array<std::vector<int>, 5> groups = {
      std::vector<int> {19, 20, 21, 22, 23, 24}, std::vector<int> {25, 26, 27, 28, 29, 30}, std::vector<int> {0, 1, 2},
      std::vector<int> {5, 6, 7, 8, 9, 10, 11}, std::vector<int> {12, 13, 14, 15, 16, 17, 18}};
  Eigen::VectorXd v = Eigen::VectorXd::Zero(31);
  for (int g = 0; g < 5; ++g) {
    if (n[g].size() != groups[g].size()) throw std::runtime_error("HumanLike joint group size mismatch");
    for (std::size_t j = 0; j < groups[g].size(); ++j) v[groups[g][j]] = n[g][j].as<double>();
  }
  if (!v.allFinite()) throw std::runtime_error("nonfinite HumanLike joints");
  return v;
}

class Network {
  public:
    Network(Ort::Env& env, const std::filesystem::path& path, const char* input, const char* output, int ni, int no)
      : input_(input),
        output_(output),
        in_(ni),
        out_(no),
        shape_ {1, ni},
        oshape_ {1, no} {
      Ort::SessionOptions options;
      options.SetIntraOpNumThreads(1);
      options.SetInterOpNumThreads(1);
      session_ = std::make_unique<Ort::Session>(env, path.c_str(), options);
      Ort::AllocatorWithDefaultOptions alloc;
      if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1 ||
          std::string(session_->GetInputNameAllocated(0, alloc).get()) != input_ ||
          std::string(session_->GetOutputNameAllocated(0, alloc).get()) != output_)
        throw std::runtime_error("HumanLike ONNX tensor names mismatch");
      auto it = session_->GetInputTypeInfo(0), ot = session_->GetOutputTypeInfo(0);
      auto i = it.GetTensorTypeAndShapeInfo(), o = ot.GetTensorTypeAndShapeInfo();
      if (i.GetShape() != std::vector<int64_t> {1, ni} || o.GetShape() != std::vector<int64_t> {1, no} ||
          i.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
          o.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error("HumanLike ONNX tensor shape/type mismatch");
    }

    const std::vector<float>& Run(const float* p) {
      std::copy(p, p + in_.size(), in_.begin());
      auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      auto in = Ort::Value::CreateTensor<float>(memory, in_.data(), in_.size(), shape_.data(), 2);
      auto out = Ort::Value::CreateTensor<float>(memory, out_.data(), out_.size(), oshape_.data(), 2);
      const char* ins[] = {input_.c_str()};
      const char* outs[] = {output_.c_str()};
      session_->Run(Ort::RunOptions {nullptr}, ins, &in, 1, outs, &out, 1);
      for (float x : out_)
        if (!std::isfinite(x)) throw std::runtime_error("nonfinite HumanLike network output");
      return out_;
    }
  private:
    std::string input_, output_;
    std::vector<float> in_, out_;
    std::array<int64_t, 2> shape_, oshape_;
    std::unique_ptr<Ort::Session> session_;
};
} // namespace

class PpHumanLikePolicy::Impl {
  public:
    explicit Impl(const std::string& path)
      : env(ORT_LOGGING_LEVEL_WARNING, "humanlike25") {
      const auto root = std::filesystem::path(path);
      const auto y = YAML::LoadFile((root / "humanlike.yaml").string());
      const auto c = y["command"], o = y["obs"], a = y["actions"];
      if (o["num_obs"].as<int>() != 89 || o["num_obs_history"].as<int>() != 16 || a["num_actions"].as<int>() != 25 ||
          y["common"]["decimation"].as<int>() != 10 ||
          y["common"]["use_imu_list"].as<std::vector<int>>() != std::vector<int> {1})
        throw std::runtime_error("unsupported HumanLike layout or rate");
      params.default_q = JointVector(a["default_joint_angles"]);
      params.action_scale = JointVector(a["action_scale"]);
      params.lower = JointVector(a["action_clip_lower"]);
      params.upper = JointVector(a["action_clip_upper"]);
      params.pos_scale = JointVector(o["dof_pos_scale"]);
      params.vel_scale = JointVector(o["dof_vel_scale"]);
      params.kp = JointVector(c["kp_limb"]);
      params.kd = JointVector(c["kd_limb"]);
      params.torque = JointVector(c["torques_limit"]);
      params.gyro_scale = Vector3(o["ang_vel_scale"]);
      params.gravity_scale = Vector3(o["quat_scale"]);
      params.command_scale = Vector3(o["cmd_vel_scale"]);
      for (int j = 0; j < 31; ++j)
        if (j != 3 && j != 4) {
          if (params.kp[j] <= 0 || params.kd[j] < 0 || params.torque[j] <= 0 || params.action_scale[j] <= 0 ||
              params.lower[j] > params.upper[j])
            throw std::runtime_error("invalid HumanLike joint limits/gains");
        }
      alpha = Vector3(c["alpha_cmd_vel"]);
      lo = Vector3(c["cmd_vel_limit"][0]);
      hi = Vector3(c["cmd_vel_limit"][1]);
      delta = Vector3(c["cmd_transition_guard_max_delta"]);
      if ((alpha.array() <= 0).any() || (alpha.array() > 1).any() || (delta.array() <= 0).any() ||
          (lo.array() > hi.array()).any())
        throw std::runtime_error("invalid HumanLike velocity parameters");
      HumanLikeGaitParameters gp;
#define READ(member, key)                                                                                              \
  gp.member = c[key].as<double>();                                                                                     \
  if (!std::isfinite(gp.member)) throw std::runtime_error("nonfinite HumanLike gait config")
      READ(dt, "gait_dt");
      READ(initial_frequency, "gait_frequency");
      READ(base, "gait_freq_base");
      READ(vx_coeff, "gait_freq_cmd_coeff");
      READ(lateral_floor, "gait_freq_lateral_floor");
      READ(lateral_vx, "gait_freq_lateral_vx_thresh");
      READ(lateral_vy, "gait_freq_lateral_vy_thresh");
      READ(low_floor, "gait_freq_low_speed_floor");
      READ(fade_lo, "gait_freq_low_speed_fade_lo");
      READ(fade_hi, "gait_freq_low_speed_fade_hi");
      READ(backward_floor, "gait_freq_backward_floor");
      READ(backward_vx, "gait_freq_backward_vx_thresh");
      READ(turning_floor, "gait_freq_turning_floor");
      READ(turning_yaw, "gait_freq_turning_yaw_thresh");
      READ(turning_vx, "gait_freq_turning_vx_thresh");
      READ(turning_vy, "gait_freq_turning_vy_thresh");
      READ(recovery_floor, "gait_freq_recovery_floor");
      READ(min_frequency, "gait_freq_min");
      READ(max_frequency, "gait_freq_max");
      READ(offset, "gait_offset");
      READ(height, "gait_height");
      READ(roll_limit, "roll_limits");
      READ(pitch_limit, "pitch_limits");
      READ(pitch_offset, "pitch_offset");
      READ(foot_dx, "foot_delta_x_thres");
      READ(foot_y_min, "foot_delta_y_min");
      READ(foot_y_max, "foot_delta_y_max");
#undef READ
      gp.zero_count = c["stand_zero_cmd_count"].as<int>();
      if (std::abs(gp.dt - .02) > 1e-9 || gp.fade_hi <= gp.fade_lo || gp.min_frequency <= 0 ||
          gp.max_frequency < gp.min_frequency)
        throw std::runtime_error("unsupported HumanLike gait parameters");
      // These additions are disabled in the inspected vendor config. Reject a
      // changed config instead of silently ignoring a newly enabled transform.
      for (const auto* key : {"yaw_from_y_gain", "yaw_from_x_gain", "y_from_x_gain", "x_from_abs_y_gain",
                              "gait_freq_vy_coeff", "gait_freq_low_speed_vy_coeff"})
        if (c[key] && c[key].as<double>() != 0)
          throw std::runtime_error("unsupported nonzero HumanLike command transform");
      if (Vector3(c["cmd_vel_offset"]).norm() != 0) throw std::runtime_error("unsupported HumanLike command offset");
      gp.finish_step_on_stop = true;
      gait = HumanLikeGait(gp);
      smooth_count = y["common"]["smooth_count"].as<int>();
      if (smooth_count < 0) throw std::runtime_error("invalid HumanLike smooth_count");
      encoder = std::make_unique<Network>(env, root / "lin_vel_encoder.onnx", "obs_history", "latent", 1424, 3);
      actor = std::make_unique<Network>(env, root / "policy.onnx", "actor_input", "actions", 1427, 25);
    }

    void Reset(const robot_io::RobotState& s, const robot_io::RobotCommand& entry) {
      if (!PpCommandTransition::Valid(entry) || entry.q_des.size() != 31)
        throw std::invalid_argument("invalid HumanLike entry command");
      gait.Reset();
      tick = 0;
      velocity.setZero();
      guarded.setZero();
      raw.fill(0);
      head = entry;
      // Preserve a position hold on the two head slots; no head output in the
      // vendor 25-action model. Runner owns these slots throughout the mode.
      head.dq_des.segment<2>(3).setZero();
      head.tau_ff.segment<2>(3).setZero();
      HumanLike25Frame(s, params, velocity, raw, gait.features()); // state validation
      for (int i = 0; i < 25; ++i) {
        int j = kHumanLike25ToSdk[i];
        previous[i] = (s.q[j] - params.default_q[j]) / params.action_scale[j];
      }
      const auto frame = HumanLike25Frame(s, params, velocity, previous, gait.features());
      for (int h = 0; h < 16; ++h) std::copy(frame.begin(), frame.end(), history.begin() + h * 89);
      initialized = true;
    }

    robot_io::RobotCommand Step(const robot_io::RobotState& s, const Eigen::Vector3d& input) {
      if (!initialized || !input.allFinite()) throw std::runtime_error("HumanLike not reset or invalid velocity");
      if (s.q.size() != 31 || s.dq.size() != 31 || !s.q.allFinite() || !s.dq.allFinite() ||
          !s.imu_quat_wxyz.allFinite() || !s.imu_gyro.allFinite() || std::abs(s.imu_quat_wxyz.norm() - 1) > .02)
        throw std::runtime_error("invalid HumanLike robot state");
      // Runner uses the vendor configured maximum delta continuously. This is
      // intentionally simpler than the original conditional reversal guard.
      for (int i = 0; i < 3; ++i) {
        double t = std::clamp(input[i], lo[i], hi[i]);
        guarded[i] += std::clamp(t - guarded[i], -delta[i], delta[i]);
        velocity[i] += alpha[i] * (guarded[i] - velocity[i]);
      }
      if (tick % 10 == 0) {
        gait.Update(guarded, s.imu_quat_wxyz, HumanLikeAnkleInPelvis(s, true), HumanLikeAnkleInPelvis(s, false));
        const auto frame = HumanLike25Frame(s, params, velocity, previous, gait.features());
        std::move(history.begin() + 89, history.end(), history.begin());
        std::copy(frame.begin(), frame.end(), history.end() - 89);
        const auto& latent = encoder->Run(history.data());
        std::copy(latent.begin(), latent.end(), actor_input.begin());
        std::copy(history.begin(), history.end(), actor_input.begin() + 3);
        const auto& action = actor->Run(actor_input.data());
        std::copy(action.begin(), action.end(), raw.begin());
      }
      auto processed = raw;
      if (tick < static_cast<unsigned>(smooth_count))
        for (int i = 0; i < 25; ++i) {
          double a = static_cast<double>(tick) / smooth_count;
          processed[i] = a * raw[i] + (1 - a) * previous[i];
        }
      // Original processing writes the entry-smoothed raw-space action back to
      // the policy action vector before base_run copies it to previous_action.
      raw = processed;
      auto out = HumanLike25Command(s, params, raw, head);
      previous = raw;
      ++tick;
      if (!PpCommandTransition::Valid(out)) throw std::runtime_error("invalid HumanLike command");
      return out;
    }

    HumanLike25Parameters params;
    HumanLikeGait gait;
    Ort::Env env;
    std::unique_ptr<Network> encoder, actor;
    Eigen::Vector3d velocity {0, 0, 0}, guarded {0, 0, 0}, alpha, lo, hi, delta;
    std::array<double, 25> raw {}, previous {};
    std::array<float, 1424> history {};
    std::array<float, 1427> actor_input {};
    robot_io::RobotCommand head;
    unsigned tick = 0;
    int smooth_count = 50;
    bool initialized = false;
};

PpHumanLikePolicy::PpHumanLikePolicy(const std::string& directory)
  : impl_(std::make_unique<Impl>(directory)) {}

PpHumanLikePolicy::~PpHumanLikePolicy() = default;

void PpHumanLikePolicy::Reset(const robot_io::RobotState& s, const robot_io::RobotCommand& c) { impl_->Reset(s, c); }

robot_io::RobotCommand PpHumanLikePolicy::Step(const robot_io::RobotState& s, const Eigen::Vector3d& v) {
  return impl_->Step(s, v);
}

bool PpHumanLikePolicy::settled() const {
  return impl_->initialized && impl_->gait.settled() && impl_->velocity.norm() < .02;
}

Eigen::Vector3d PpHumanLikePolicy::filtered_velocity() const { return impl_->velocity; }
} // namespace a3_pingpong
