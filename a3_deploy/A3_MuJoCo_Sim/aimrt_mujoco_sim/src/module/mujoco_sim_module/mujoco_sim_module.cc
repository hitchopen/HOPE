// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/mujoco_sim_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/common/gate3_ball_contact_model.h"
#include "mujoco_sim_module/common/gate3_ball_layout.h"
#include "mujoco_sim_module/publisher/gate3_ball_state_ros2_publisher.h"
#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"
#include "mujoco_sim_module/publisher/joint_sensor_publisher.h"
#include "mujoco_sim_module/publisher/touch_sensor_publisher.h"
#include "mujoco_sim_module/subscriber/gate3_ball_command_ros2_subscriber.h"
#include "mujoco_sim_module/subscriber/joint_actuator_subscriber.h"
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include "mujoco_sim_module/publisher/pose_twist_ros2_publisher.h"
  #include "mujoco_sim_module/subscriber/sim_reset_ros2_subscriber.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <thread>

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::MujocoSimModule::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::MujocoSimModule::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["simulation_model_path"] = rhs.simulation_model_path;
    node["sim_executor"] = rhs.sim_executor;
    node["gui_executor"] = rhs.gui_executor;
    node["default_free_camera_focus_body"] = rhs.default_free_camera_focus_body;
    node["default_tracking_camera_body"] = rhs.default_tracking_camera_body;
    node["default_camera_distance"] = rhs.default_camera_distance;

    node["subscriber_options"] = YAML::Node();
    for (const auto& subscriber_option : rhs.subscriber_options) {
      Node subscriber_option_node;
      subscriber_option_node["topic"] = subscriber_option.topic;
      subscriber_option_node["type"] = subscriber_option.type;
      subscriber_option_node["options"] = subscriber_option.options;
      node["subscriber_options"].push_back(subscriber_option_node);
    }

    node["publisher_options"] = YAML::Node();
    for (const auto& publisher_option : rhs.publisher_options) {
      Node publisher_option_node;
      publisher_option_node["topic"] = publisher_option.topic;
      publisher_option_node["frequency"] = publisher_option.frequency;
      publisher_option_node["executor"] = publisher_option.executor;
      publisher_option_node["type"] = publisher_option.type;
      publisher_option_node["options"] = publisher_option.options;
      node["publisher_options"].push_back(publisher_option_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    rhs.simulation_model_path = node["simulation_model_path"].as<std::string>();
    rhs.sim_executor = node["sim_executor"].as<std::string>();
    rhs.gui_executor = node["gui_executor"].as<std::string>();
    if (node["default_free_camera_focus_body"]) {
      rhs.default_free_camera_focus_body = node["default_free_camera_focus_body"].as<std::string>();
    }
    if (node["default_tracking_camera_body"]) {
      rhs.default_tracking_camera_body = node["default_tracking_camera_body"].as<std::string>();
    }
    if (node["default_camera_distance"]) {
      rhs.default_camera_distance = node["default_camera_distance"].as<double>();
    }

    if (node["subscriber_options"] && node["subscriber_options"].IsSequence()) {
      for (const auto& subscriber_option_node : node["subscriber_options"]) {
        auto subscriber_options = Options::SubscriberOption{
            .topic = subscriber_option_node["topic"].as<std::string>(),
            .type = subscriber_option_node["type"].as<std::string>()};

        if (subscriber_option_node["options"])
          subscriber_options.options = subscriber_option_node["options"];
        else
          subscriber_options.options = YAML::Node(YAML::NodeType::Null);

        rhs.subscriber_options.emplace_back(std::move(subscriber_options));
      }
    }

    if (node["publisher_options"] && node["publisher_options"].IsSequence()) {
      for (const auto& publisher_option_node : node["publisher_options"]) {
        auto publisher_options = Options::PublisherOption{
            .topic = publisher_option_node["topic"].as<std::string>(),
            .frequency = publisher_option_node["frequency"].as<uint32_t>(),
            .executor = publisher_option_node["executor"].as<std::string>(),
            .type = publisher_option_node["type"].as<std::string>()};

        if (publisher_option_node["options"])
          publisher_options.options = publisher_option_node["options"];
        else
          publisher_options.options = YAML::Node(YAML::NodeType::Null);

        rhs.publisher_options.emplace_back(std::move(publisher_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module {
namespace gate3 = common::gate3_ball;

bool MujocoSimModule::Initialize(aimrt::CoreRef core) {
  core_ = core;

  SetLogger(core_.GetLogger());

  // Read cfg
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  auto yaml_node = YAML::LoadFile(std::string(file_path));
  options_ = yaml_node.as<Options>();

  // Get executor handle
  gui_executor_ = core_.GetExecutorManager().GetExecutor(options_.gui_executor);
  AIMRT_CHECK_ERROR_THROW(gui_executor_, "Get executor '{}' failed.", options_.gui_executor);

  sim_executor_ = core_.GetExecutorManager().GetExecutor(options_.sim_executor);
  AIMRT_CHECK_ERROR_THROW(sim_executor_, "Get executor '{}' failed.", options_.sim_executor);
  AIMRT_CHECK_ERROR_THROW(sim_executor_.SupportTimerSchedule(),
                          "Sim executor '{}' do not support time schedule.", options_.sim_executor);

  // load model
  m_ = mj_loadXML(options_.simulation_model_path.c_str(), nullptr, nullptr, 0);
  AIMRT_CHECK_ERROR_THROW(m_ != nullptr, "Load model failed, model path: '{}'.", options_.simulation_model_path);

  if (const char* pd_mode = std::getenv("A3_MUJOCO_PD_MODE")) debug_pd_mode_ = pd_mode;
  AIMRT_CHECK_ERROR_THROW(debug_pd_mode_ == "explicit" || debug_pd_mode_ == "implicit",
                          "Invalid A3_MUJOCO_PD_MODE='{}' (expected explicit or implicit).",
                          debug_pd_mode_);
  if (debug_pd_mode_ == "implicit") m_->opt.integrator = mjINT_IMPLICITFAST;
  AIMRT_INFO("MuJoCo body-drive PD mode='{}', integrator={}.{}",
             debug_pd_mode_, static_cast<int>(m_->opt.integrator),
             debug_pd_mode_ == "implicit"
                 ? " Diagnostic Isaac-faithful A/B only; message kd is folded into passive damping."
                 : " AGI default: message kd is explicit motor torque.");

  d_ = mj_makeData(m_);
  AIMRT_CHECK_ERROR_THROW(d_ != nullptr, "Make data failed.");

  InitializeGate3Ball();
  InitializeDebugCsv();

  // register subscriber gen func
  RegisterSubscriberGenFunc();

  // create subscriber
  for (auto& item : options_.subscriber_options) {
    auto finditr = subscriber_gen_func_map_.find(item.type);
    AIMRT_CHECK_ERROR_THROW(finditr != subscriber_gen_func_map_.end(),
                            "Invalid type '{}' for subscriber.", item.type);

    auto ptr = finditr->second();

    ptr->SetMj(m_, d_);
    ptr->SetSubscriberHandle(core_.GetChannelHandle().GetSubscriber(item.topic));

    ptr->Initialize(item.options);

    subscriber_map_.emplace(item.topic, std::move(ptr));
  }

  // register publisher gen func
  RegisterPublisherGenFunc();

  // create publisher
  for (auto& item : options_.publisher_options) {
    auto finditr = publisher_gen_func_map_.find(item.type);
    AIMRT_CHECK_ERROR_THROW(finditr != publisher_gen_func_map_.end(),
                            "Invalid type '{}' for publisher.", item.type);

    auto executor = core_.GetExecutorManager().GetExecutor(item.executor);
    AIMRT_CHECK_ERROR_THROW(executor, "Can not get executor '{}' for publisher topic '{}'.",
                            item.executor, item.topic);

    auto ptr = finditr->second();

    ptr->SetMj(m_, d_);
    ptr->SetPublisherHandle(core_.GetChannelHandle().GetPublisher(item.topic));
    ptr->SetExecutor(executor);
    ptr->SetFreq(item.frequency);

    ptr->Initialize(item.options);

    publisher_map_.emplace(item.topic, std::move(ptr));
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool MujocoSimModule::Start() {
  AIMRT_INFO("Start succeeded.");

  run_flag_ = true;
  sim_loop_exited_ = false;

  scope_.spawn(aimrt::co::On(aimrt::co::InlineScheduler(), GuiLoop()));
  scope_.spawn(aimrt::co::On(aimrt::co::InlineScheduler(), SimLoop()));

  for (auto& itr : subscriber_map_) {
    itr.second->Start();
  }

  for (auto& itr : publisher_map_) {
    itr.second->Start();
  }

  return true;
}

void MujocoSimModule::Shutdown() {
  run_flag_ = false;

  for (auto& itr : publisher_map_) {
    itr.second->Shutdown();
  }

  for (auto& itr : subscriber_map_) {
    itr.second->Shutdown();
  }

  mujoco::Simulate* sim = nullptr;
  {
    const std::lock_guard<std::mutex> lock(sim_lifecycle_mutex_);
    sim = sim_.get();
  }
  if (sim) {
    sim->exitrequest.store(1);
  }

  aimrt::co::SyncWait(scope_.complete());

  default_camera_follow_body_id_ = -1;

  if (debug_csv_.is_open()) {
    debug_csv_.flush();
    debug_csv_.close();
  }

  if (d_) {
    mj_deleteData(d_);
    d_ = nullptr;
  }
  if (m_) {
    mj_deleteModel(m_);
    m_ = nullptr;
  }

  AIMRT_INFO("Shutdown succeeded.");
}

void MujocoSimModule::RegisterSubscriberGenFunc() {
  auto generator = [this]<typename T>(std::string_view name) {
    subscriber_gen_func_map_.emplace(
        name,
        []() -> std::unique_ptr<subscriber::SubscriberBase> {
          return std::make_unique<T>();
        });
  };

  generator.template operator()<subscriber::JointActuatorSubscriber>("joint_actuator");

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  generator.template operator()<subscriber::JointActuatorRos2Subscriber>("joint_actuator_ros2");
  generator.template operator()<subscriber::BodyDriveJointActuatorSubscriber>("body_drive_joint_actuator");
  generator.template operator()<subscriber::Gate3BallCommandRos2Subscriber>("gate3_ball_command_ros2");
  generator.template operator()<subscriber::SimResetRos2Subscriber>("sim_reset_ros2");
#endif
}

void MujocoSimModule::RegisterPublisherGenFunc() {
  auto generator = [this]<typename T>(std::string_view name) {
    publisher_gen_func_map_.emplace(
        name,
        []() -> std::unique_ptr<publisher::PublisherBase> {
          return std::make_unique<T>();
        });
  };

  generator.template operator()<publisher::JointSensorPublisher>("joint_sensor");
  generator.template operator()<publisher::ImuSensorPublisher>("imu_sensor");
  generator.template operator()<publisher::TouchSensorPublisher>("touch_sensor");
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  generator.template operator()<publisher::ImuSensorRos2Publisher>("imu_sensor_ros2");
  generator.template operator()<publisher::TouchSensorRos2Publisher>("touch_sensor_ros2");
  generator.template operator()<publisher::JointSensorRos2Publisher>("joint_sensor_ros2");
  generator.template operator()<publisher::BodyDriveJointSensorPublisher>("body_drive_joint_sensor");
  generator.template operator()<publisher::Gate3BallStateRos2Publisher>("gate3_ball_state_ros2");
  generator.template operator()<publisher::PoseSensorRos2Publisher>("pose_sensor_ros2");
  generator.template operator()<publisher::TwistSensorRos2Publisher>("twist_sensor_ros2");
  generator.template operator()<publisher::OdometryRos2Publisher>("odometry_ros2");
#endif
}

void MujocoSimModule::ApplyDefaultCameraFocus() {
  if (options_.default_tracking_camera_body.empty() && options_.default_free_camera_focus_body.empty()) return;

  const bool tracking = !options_.default_tracking_camera_body.empty();
  const auto& focus_body = tracking ? options_.default_tracking_camera_body : options_.default_free_camera_focus_body;

  const int body_id = mj_name2id(m_, mjOBJ_BODY, focus_body.c_str());
  if (body_id < 0) {
    AIMRT_WARN("Default camera focus body '{}' not found.", focus_body);
    return;
  }

  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  mj_forward(m_, d_);
  sim_->cam.fixedcamid = -1;
  if (options_.default_camera_distance > 0.0) {
    sim_->cam.distance = options_.default_camera_distance;
  }

  const mjtNum* focus_pos = d_->subtree_com + 3 * body_id;
  if (tracking) {
    default_camera_follow_body_id_ = body_id;
    sim_->cam.type = mjCAMERA_FREE;
    sim_->cam.trackbodyid = -1;
    sim_->camera = 0;
    mju_copy3(sim_->cam.lookat, focus_pos);
    mju_copy3(default_camera_follow_last_pos_, focus_pos);
  } else {
    default_camera_follow_body_id_ = -1;
    sim_->cam.type = mjCAMERA_FREE;
    sim_->cam.trackbodyid = -1;
    sim_->camera = 0;
    mju_copy3(sim_->cam.lookat, focus_pos);
  }

  AIMRT_INFO("Default {} camera focus set to body '{}' at [{:.3f}, {:.3f}, {:.3f}], distance {:.3f}.",
             tracking ? "following free" : "free",
             focus_body,
             sim_->cam.lookat[0],
             sim_->cam.lookat[1],
             sim_->cam.lookat[2],
             sim_->cam.distance);
}

void MujocoSimModule::UpdateDefaultCameraFollowLocked() {
  if (default_camera_follow_body_id_ < 0) return;

  const mjtNum* body_pos = d_->subtree_com + 3 * default_camera_follow_body_id_;
  if (sim_->cam.type == mjCAMERA_FREE && sim_->camera == 0) {
    mjtNum delta[3];
    mju_sub3(delta, body_pos, default_camera_follow_last_pos_);
    mju_addTo3(sim_->cam.lookat, delta);
  }
  mju_copy3(default_camera_follow_last_pos_, body_pos);
}

void MujocoSimModule::InitializeGate3Ball() {
  gate3_ball_body_id_ = mj_name2id(m_, mjOBJ_BODY, gate3::kBodyName);
  gate3_ball_joint_id_ = mj_name2id(m_, mjOBJ_JOINT, gate3::kJointName);
  gate3_ball_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, gate3::kGeomName);
  gate3_racket_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, gate3::kRacketGeomName);
  gate3_racket_site_id_ = mj_name2id(m_, mjOBJ_SITE, "right_racket");
  gate3_table_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, gate3::kTableGeomName);
  gate3_net_geom_id_ = mj_name2id(m_, mjOBJ_GEOM, gate3::kNetGeomName);
  AIMRT_CHECK_ERROR_THROW(
      gate3_ball_body_id_ >= 0 && gate3_ball_joint_id_ >= 0 &&
          gate3_ball_geom_id_ >= 0 && gate3_racket_geom_id_ >= 0 &&
          gate3_racket_site_id_ >= 0 &&
          gate3_table_geom_id_ >= 0 && gate3_net_geom_id_ >= 0,
      "Gate3 physical ball/table/net/racket geometry is incomplete.");
  AIMRT_CHECK_ERROR_THROW(
      m_->nuserdata >= static_cast<int>(gate3::kRequiredUserData),
      "Gate3 physical telemetry needs {} userdata values, model has {}.",
      gate3::kRequiredUserData, m_->nuserdata);
  gate3_ball_qpos_addr_ = m_->jnt_qposadr[gate3_ball_joint_id_];
  gate3_ball_dof_addr_ = m_->jnt_dofadr[gate3_ball_joint_id_];
  const auto read_nonnegative_env = [](const char* name, double fallback,
                                       double upper) {
    const char* raw = std::getenv(name);
    if (!raw) return fallback;
    char* end = nullptr;
    const double parsed = std::strtod(raw, &end);
    AIMRT_CHECK_ERROR_THROW(
        end != raw && *end == '\0' && std::isfinite(parsed) &&
            parsed >= 0.0 && parsed <= upper,
        "Invalid {}='{}'.", name, raw);
    return parsed;
  };
  gate3_ball_drag_k_ =
      read_nonnegative_env("A3_GATE3_BALL_DRAG_K", gate3_ball_drag_k_, 10.0);
  gate3_ball_restitution_h_ = read_nonnegative_env(
      "A3_GATE3_BALL_RESTITUTION_H", gate3_ball_restitution_h_, 1.0);
  gate3_ball_restitution_v_ = read_nonnegative_env(
      "A3_GATE3_BALL_RESTITUTION_V", gate3_ball_restitution_v_, 1.0);
  d_->userdata[gate3::kShotId] = 0.0;
  d_->userdata[gate3::kActive] = 0.0;
  d_->userdata[gate3::kContactBits] = 0.0;
  d_->userdata[gate3::kRacketContactCount] = 0.0;
  d_->userdata[gate3::kTableContactCount] = 0.0;
  d_->userdata[gate3::kNetContactCount] = 0.0;
  d_->userdata[gate3::kRacketNormalForce] = 0.0;
  AIMRT_INFO(
      "Gate3 physical ball initialized: drag_k={:.4f}, restitution_h/v="
      "{:.4f}/{:.4f}, ball='{}', "
      "racket='{}', table='{}', net='{}'.",
      gate3_ball_drag_k_, gate3_ball_restitution_h_,
      gate3_ball_restitution_v_, gate3::kGeomName,
      gate3::kRacketGeomName, gate3::kTableGeomName,
      gate3::kNetGeomName);
}

void MujocoSimModule::ApplyGate3BallDrag() {
  if (gate3_ball_body_id_ < 0 || gate3_ball_dof_addr_ < 0) return;
  mjtNum* wrench = d_->xfrc_applied + 6 * gate3_ball_body_id_;
  wrench[0] = wrench[1] = wrench[2] = 0.0;
  if (d_->userdata[gate3::kActive] <= 0.5) return;

  const mjtNum* velocity = d_->qvel + gate3_ball_dof_addr_;
  gate3_ball_pre_step_velocity_ = {
      static_cast<double>(velocity[0]),
      static_cast<double>(velocity[1]),
      static_cast<double>(velocity[2]),
      static_cast<double>(velocity[3]),
      static_cast<double>(velocity[4]),
      static_cast<double>(velocity[5])};
  const gate3::Vec3 linear_velocity{
      gate3_ball_pre_step_velocity_[0],
      gate3_ball_pre_step_velocity_[1],
      gate3_ball_pre_step_velocity_[2]};
  const gate3::Vec3 angular_velocity{
      gate3_ball_pre_step_velocity_[3],
      gate3_ball_pre_step_velocity_[4],
      gate3_ball_pre_step_velocity_[5]};
  const auto acceleration = gate3::FlightAcceleration(
      linear_velocity, angular_velocity, gate3_ball_drag_k_);
  const double mass = m_->body_mass[gate3_ball_body_id_];
  wrench[0] = mass * acceleration[0];
  wrench[1] = mass * acceleration[1];
  wrench[2] = mass * acceleration[2];
}

void MujocoSimModule::UpdateGate3BallContacts() {
  if (gate3_ball_geom_id_ < 0) return;

  const auto shot_id = static_cast<std::uint64_t>(
      std::max<mjtNum>(0.0, d_->userdata[gate3::kShotId]));
  if (shot_id != gate3_last_shot_id_) {
    gate3_last_shot_id_ = shot_id;
    gate3_racket_contact_prev_ = false;
    gate3_table_contact_prev_ = false;
    gate3_net_contact_prev_ = false;
  }

  bool racket = false;
  bool table = false;
  bool net = false;
  double racket_normal_force = 0.0;
  gate3::Vec3 racket_contact_position{};
  bool racket_contact_position_valid = false;
  for (int contact_id = 0; contact_id < d_->ncon; ++contact_id) {
    const auto& contact = d_->contact[contact_id];
    const int other =
        contact.geom1 == gate3_ball_geom_id_
            ? contact.geom2
            : contact.geom2 == gate3_ball_geom_id_ ? contact.geom1 : -1;
    if (other < 0) continue;
    if (other == gate3_racket_geom_id_) {
      racket = true;
      if (!racket_contact_position_valid) {
        racket_contact_position = {
            static_cast<double>(contact.pos[0]),
            static_cast<double>(contact.pos[1]),
            static_cast<double>(contact.pos[2])};
        racket_contact_position_valid = true;
      }
      std::array<mjtNum, 6> wrench{};
      mj_contactForce(m_, d_, contact_id, wrench.data());
      racket_normal_force += std::abs(static_cast<double>(wrench[0]));
    } else if (other == gate3_table_geom_id_) {
      table = true;
    } else if (other == gate3_net_geom_id_) {
      net = true;
    }
  }

  unsigned char bits = 0;
  if (racket) bits |= gate3::kContactRacket;
  if (table) bits |= gate3::kContactTable;
  if (net) bits |= gate3::kContactNet;
  d_->userdata[gate3::kContactBits] = static_cast<mjtNum>(bits);
  // State is published at 250 Hz, while contacts are sampled at 1 kHz.
  // Latch the per-shot peak so diagnostic force evidence cannot disappear
  // between publisher ticks. ResetTelemetry clears it at the next launch.
  d_->userdata[gate3::kRacketNormalForce] = std::max<mjtNum>(
      d_->userdata[gate3::kRacketNormalForce], racket_normal_force);
  if (racket && !gate3_racket_contact_prev_)
    d_->userdata[gate3::kRacketContactCount] += 1.0;
  if (table && !gate3_table_contact_prev_)
    d_->userdata[gate3::kTableContactCount] += 1.0;
  if (net && !gate3_net_contact_prev_)
    d_->userdata[gate3::kNetContactCount] += 1.0;

  // The venue fit and training virtual ball use an impulse map; MuJoCo's
  // underdamped soft-contact parameter is not that map and previously injected
  // up to 40 m/s into a ball hit by a roughly 2 m/s racket. Keep collision
  // detection physical, but on exactly one rising edge replace the solver's
  // outgoing ball state with the fitted planner/training law. The racket
  // surface velocity includes omega x r at the actual contact point.
  if (racket && !gate3_racket_contact_prev_ &&
      racket_contact_position_valid) {
    std::array<mjtNum, 6> racket_spatial_velocity{};
    mj_objectVelocity(
        m_, d_, mjOBJ_GEOM, gate3_racket_geom_id_,
        racket_spatial_velocity.data(), 0);
    const gate3::Vec3 racket_angular_velocity{
        static_cast<double>(racket_spatial_velocity[0]),
        static_cast<double>(racket_spatial_velocity[1]),
        static_cast<double>(racket_spatial_velocity[2])};
    const gate3::Vec3 racket_origin_velocity{
        static_cast<double>(racket_spatial_velocity[3]),
        static_cast<double>(racket_spatial_velocity[4]),
        static_cast<double>(racket_spatial_velocity[5])};
    const mjtNum* racket_position =
        d_->geom_xpos + 3 * gate3_racket_geom_id_;
    const gate3::Vec3 contact_offset{
        racket_contact_position[0] - racket_position[0],
        racket_contact_position[1] - racket_position[1],
        racket_contact_position[2] - racket_position[2]};
    const gate3::Vec3 racket_contact_velocity = gate3::Add(
        racket_origin_velocity,
        gate3::Cross(racket_angular_velocity, contact_offset));

    // The deploy/training observation contract defines the racket face normal
    // as the right_racket SITE's local +Y.  Do not use the mesh geom frame:
    // MuJoCo applies mesh_quat principal-axis alignment, so geom local +Y is
    // not the wrist/racket-frame +Y used by Isaac and the planner.
    const mjtNum* racket_rotation =
        d_->site_xmat + 9 * gate3_racket_site_id_;
    const gate3::Vec3 racket_face_normal{
        static_cast<double>(racket_rotation[1]),
        static_cast<double>(racket_rotation[4]),
        static_cast<double>(racket_rotation[7])};
    const gate3::Vec3 incoming_velocity{
        gate3_ball_pre_step_velocity_[0],
        gate3_ball_pre_step_velocity_[1],
        gate3_ball_pre_step_velocity_[2]};
    const gate3::Vec3 incoming_spin{
        gate3_ball_pre_step_velocity_[3],
        gate3_ball_pre_step_velocity_[4],
        gate3_ball_pre_step_velocity_[5]};
    const auto outgoing = gate3::PredictPaddleContact(
        incoming_velocity, racket_contact_velocity, racket_face_normal,
        incoming_spin, gate3::kBallRadius);
    for (int axis = 0; axis < 3; ++axis) {
      d_->qvel[gate3_ball_dof_addr_ + axis] =
          outgoing.linear_velocity[axis];
      d_->qvel[gate3_ball_dof_addr_ + 3 + axis] =
          outgoing.angular_velocity[axis];
      d_->qpos[gate3_ball_qpos_addr_ + axis] =
          racket_contact_position[axis] +
          (gate3::kBallRadius + 1.0e-4) *
              outgoing.oriented_normal[axis];
    }
  }

  // MuJoCo's soft-contact damping ratio is not a coefficient of restitution.
  // Gate3 must exercise the exact venue/planner ball model, so on the first
  // physical table-contact step apply the same discrete bounce law used by
  // hope_planner:
  //   v_xy+ = e_h v_xy-,  v_z+ = -e_v v_z-.
  // Moving the ball center just clear of the table prevents the stable
  // critically-damped contact from applying a second impulse next step.
  if (table && !gate3_table_contact_prev_ &&
      gate3_ball_pre_step_velocity_[2] < 0.0) {
    d_->qvel[gate3_ball_dof_addr_ + 0] =
        gate3_ball_restitution_h_ * gate3_ball_pre_step_velocity_[0];
    d_->qvel[gate3_ball_dof_addr_ + 1] =
        gate3_ball_restitution_h_ * gate3_ball_pre_step_velocity_[1];
    d_->qvel[gate3_ball_dof_addr_ + 2] =
        -gate3_ball_restitution_v_ * gate3_ball_pre_step_velocity_[2];
    d_->qpos[gate3_ball_qpos_addr_ + 2] = std::max<mjtNum>(
        d_->qpos[gate3_ball_qpos_addr_ + 2],
        gate3::kTableSurfaceZ + gate3::kBallRadius + 1.0e-4);
  }
  gate3_racket_contact_prev_ = racket;
  gate3_table_contact_prev_ = table;
  gate3_net_contact_prev_ = net;
}

void MujocoSimModule::InitializeDebugCsv() {
  const char* path = std::getenv("A3_MUJOCO_DEBUG_CSV");
  if (!path || !*path) return;

  if (const char* stride = std::getenv("A3_MUJOCO_DEBUG_STRIDE")) {
    char* end = nullptr;
    const long parsed = std::strtol(stride, &end, 10);
    if (end != stride && parsed > 0 && parsed <= 10000) debug_stride_ = static_cast<int>(parsed);
  }

  debug_csv_.open(path, std::ios::out | std::ios::trunc);
  if (!debug_csv_) {
    AIMRT_WARN("Cannot open MuJoCo debug CSV '{}'.", path);
    return;
  }

  debug_pelvis_body_id_ = mj_name2id(m_, mjOBJ_BODY, "pelvis_link");
  debug_torso_body_id_ = mj_name2id(m_, mjOBJ_BODY, "torso_Link");
  debug_left_foot_body_id_ = mj_name2id(m_, mjOBJ_BODY, "left_ankle_roll_Link");
  debug_right_foot_body_id_ = mj_name2id(m_, mjOBJ_BODY, "right_ankle_roll_Link");
  debug_racket_site_id_ = mj_name2id(m_, mjOBJ_SITE, "right_racket");

  debug_actuators_.reserve(m_->nu);
  for (int actuator_id = 0; actuator_id < m_->nu; ++actuator_id) {
    const int joint_id = m_->actuator_trnid[2 * actuator_id];
    const char* actuator_name = mj_id2name(m_, mjOBJ_ACTUATOR, actuator_id);
    const char* joint_name =
        joint_id >= 0 ? mj_id2name(m_, mjOBJ_JOINT, joint_id) : nullptr;
    debug_actuators_.push_back(DebugActuator{
        .name = joint_name ? joint_name : (actuator_name ? actuator_name : "unnamed"),
        .qpos_addr = joint_id >= 0 ? m_->jnt_qposadr[joint_id] : -1,
        .dof_addr = joint_id >= 0 ? m_->jnt_dofadr[joint_id] : -1,
    });
  }

  auto& o = debug_csv_;
  o << "sim_step,reset_seq,sim_time,wall_time_ns,pd_mode,integrator,ncon,ctrl_sat_count,max_ctrl_ratio"
    << ",base_x,base_y,base_z,base_qw,base_qx,base_qy,base_qz"
    << ",base_wx,base_wy,base_wz,base_vx,base_vy,base_vz"
    << ",torso_x,torso_y,torso_z,torso_qw,torso_qx,torso_qy,torso_qz"
    << ",torso_wx,torso_wy,torso_wz,torso_vx,torso_vy,torso_vz"
    << ",racket_x,racket_y,racket_z,racket_wx,racket_wy,racket_wz"
    << ",racket_vx,racket_vy,racket_vz"
    << ",left_foot_x,left_foot_y,left_foot_z,left_foot_vx,left_foot_vy,left_foot_vz"
    << ",right_foot_x,right_foot_y,right_foot_z,right_foot_vx,right_foot_vy,right_foot_vz"
    << ",left_foot_normal_force,right_foot_normal_force";
  for (const auto& a : debug_actuators_) o << ",q_" << a.name;
  for (const auto& a : debug_actuators_) o << ",qd_" << a.name;
  for (const auto& a : debug_actuators_) o << ",ctrl_" << a.name;
  for (const auto& a : debug_actuators_) o << ",force_" << a.name;
  for (const auto& a : debug_actuators_) o << ",ctrl_ratio_" << a.name;
  o << '\n';
  o << std::setprecision(10);
  AIMRT_INFO("MuJoCo plant debug CSV -> '{}' (stride={}, physics_hz={:.1f}, output_hz={:.1f}).",
             path, debug_stride_, 1.0 / m_->opt.timestep,
             1.0 / (m_->opt.timestep * debug_stride_));
}

void MujocoSimModule::WriteDebugCsv(std::uint64_t wall_time_ns) {
  if (!debug_csv_.is_open()) return;

  ++debug_sim_step_;
  if (debug_last_sim_time_ >= 0.0 && d_->time + 1e-12 < debug_last_sim_time_) ++debug_reset_seq_;
  debug_last_sim_time_ = d_->time;
  if (debug_sim_step_ % static_cast<std::uint64_t>(debug_stride_) != 0) return;

  auto body_pos = [this](int id, int axis) -> double {
    return id >= 0 ? d_->xpos[3 * id + axis] : 0.0;
  };
  auto body_quat = [this](int id, int axis) -> double {
    return id >= 0 ? d_->xquat[4 * id + axis] : (axis == 0 ? 1.0 : 0.0);
  };
  auto body_vel = [this](int id) {
    std::array<mjtNum, 6> v{};
    if (id >= 0) mj_objectVelocity(m_, d_, mjOBJ_BODY, id, v.data(), 0);
    return v;  // angular xyz, then linear xyz, in world coordinates
  };
  const auto pelvis_vel = body_vel(debug_pelvis_body_id_);
  const auto torso_vel = body_vel(debug_torso_body_id_);
  const auto left_foot_vel = body_vel(debug_left_foot_body_id_);
  const auto right_foot_vel = body_vel(debug_right_foot_body_id_);
  std::array<mjtNum, 6> racket_vel{};
  if (debug_racket_site_id_ >= 0)
    mj_objectVelocity(m_, d_, mjOBJ_SITE, debug_racket_site_id_, racket_vel.data(), 0);

  double left_foot_force = 0.0;
  double right_foot_force = 0.0;
  for (int contact_id = 0; contact_id < d_->ncon; ++contact_id) {
    const auto& contact = d_->contact[contact_id];
    const int body1 = m_->geom_bodyid[contact.geom1];
    const int body2 = m_->geom_bodyid[contact.geom2];
    std::array<mjtNum, 6> wrench{};
    mj_contactForce(m_, d_, contact_id, wrench.data());
    const double normal = std::abs(static_cast<double>(wrench[0]));
    if (body1 == debug_left_foot_body_id_ || body2 == debug_left_foot_body_id_)
      left_foot_force += normal;
    if (body1 == debug_right_foot_body_id_ || body2 == debug_right_foot_body_id_)
      right_foot_force += normal;
  }

  int ctrl_sat_count = 0;
  double max_ctrl_ratio = 0.0;
  std::vector<double> ctrl_ratios(m_->nu, 0.0);
  for (int i = 0; i < m_->nu; ++i) {
    if (!m_->actuator_ctrllimited[i]) continue;
    const double lo = m_->actuator_ctrlrange[2 * i];
    const double hi = m_->actuator_ctrlrange[2 * i + 1];
    const double limit = std::max(std::abs(lo), std::abs(hi));
    if (limit <= 0.0) continue;
    ctrl_ratios[i] = std::abs(d_->ctrl[i]) / limit;
    max_ctrl_ratio = std::max(max_ctrl_ratio, ctrl_ratios[i]);
    if (d_->ctrl[i] <= lo + 1e-6 || d_->ctrl[i] >= hi - 1e-6) ++ctrl_sat_count;
  }

  auto& o = debug_csv_;
  o << debug_sim_step_ << ',' << debug_reset_seq_ << ',' << d_->time << ',' << wall_time_ns
    << ',' << debug_pd_mode_ << ',' << static_cast<int>(m_->opt.integrator)
    << ',' << d_->ncon << ',' << ctrl_sat_count << ',' << max_ctrl_ratio;
  for (int i = 0; i < 3; ++i) o << ',' << body_pos(debug_pelvis_body_id_, i);
  for (int i = 0; i < 4; ++i) o << ',' << body_quat(debug_pelvis_body_id_, i);
  for (double v : pelvis_vel) o << ',' << v;
  for (int i = 0; i < 3; ++i) o << ',' << body_pos(debug_torso_body_id_, i);
  for (int i = 0; i < 4; ++i) o << ',' << body_quat(debug_torso_body_id_, i);
  for (double v : torso_vel) o << ',' << v;
  for (int i = 0; i < 3; ++i)
    o << ',' << (debug_racket_site_id_ >= 0 ? d_->site_xpos[3 * debug_racket_site_id_ + i] : 0.0);
  for (double v : racket_vel) o << ',' << v;
  for (int i = 0; i < 3; ++i) o << ',' << body_pos(debug_left_foot_body_id_, i);
  for (int i = 3; i < 6; ++i) o << ',' << left_foot_vel[i];
  for (int i = 0; i < 3; ++i) o << ',' << body_pos(debug_right_foot_body_id_, i);
  for (int i = 3; i < 6; ++i) o << ',' << right_foot_vel[i];
  o << ',' << left_foot_force << ',' << right_foot_force;
  for (const auto& a : debug_actuators_) o << ',' << (a.qpos_addr >= 0 ? d_->qpos[a.qpos_addr] : 0.0);
  for (const auto& a : debug_actuators_) o << ',' << (a.dof_addr >= 0 ? d_->qvel[a.dof_addr] : 0.0);
  for (int i = 0; i < m_->nu; ++i) o << ',' << d_->ctrl[i];
  for (int i = 0; i < m_->nu; ++i) o << ',' << d_->actuator_force[i];
  for (double ratio : ctrl_ratios) o << ',' << ratio;
  o << '\n';
  if (++debug_rows_ % 200 == 0) o.flush();
}

aimrt::co::Task<void> MujocoSimModule::GuiLoop() {
  auto gui_scheduler = aimrt::co::AimRTScheduler(gui_executor_);
  co_await aimrt::co::Schedule(gui_scheduler);

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_shared<mujoco::Simulate>(
      std::make_unique<mujoco::GlfwAdapter>(),
      &cam, &opt, &pert, false);
  {
    const std::lock_guard<std::mutex> lock(sim_lifecycle_mutex_);
    sim_ = sim;
  }

  if (!run_flag_) {
    sim->exitrequest.store(1);
  }

  sim->RenderLoop();

  while (!sim_loop_exited_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  {
    const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
    sim->mnew_ = nullptr;
    sim->dnew_ = nullptr;
    sim->m_ = nullptr;
    sim->d_ = nullptr;
  }
  {
    const std::lock_guard<std::mutex> lock(sim_lifecycle_mutex_);
    if (sim_ == sim) {
      sim_.reset();
    }
  }
  sim.reset();

  AIMRT_INFO("GuiLoop exit.");

  co_return;
}

aimrt::co::Task<void> MujocoSimModule::SimLoop() {
  auto sim_scheduler = aimrt::co::AimRTScheduler(sim_executor_);

  mujoco::Simulate* sim = nullptr;
  while (!sim && run_flag_) {
    {
      const std::lock_guard<std::mutex> lock(sim_lifecycle_mutex_);
      sim = sim_.get();
    }
    co_await aimrt::co::ScheduleAfter(sim_scheduler, std::chrono::milliseconds(500));
  }
  if (!sim) {
    sim_loop_exited_ = true;
    co_return;
  }
  co_await aimrt::co::ScheduleAfter(sim_scheduler, std::chrono::milliseconds(100));

  sim->Load(m_, d_, options_.simulation_model_path.c_str());
  ApplyDefaultCameraFocus();

  // loop
  auto next_sche_tp = sim_executor_.Now();
  std::chrono::nanoseconds dt(static_cast<uint64_t>(m_->opt.timestep * 1e9));

  while (!sim->exitrequest.load()) {
    next_sche_tp += dt;

    co_await aimrt::co::ScheduleAt(sim_scheduler, next_sche_tp);

    {
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      // apply reset/control requests before normal actuator commands
      for (auto& itr : subscriber_map_) {
        if (itr.second->Type() == "sim_reset_ros2") {
          itr.second->ApplyCtrlData();
        }
      }

      // apply ctrl data
      for (auto& itr : subscriber_map_) {
        if (itr.second->Type() != "sim_reset_ros2") {
          itr.second->ApplyCtrlData();
        }
      }

      // read sensor data
      const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count();
      const auto publish_context = publisher::PublishContext{
          .sequence = publish_sequence_++,
          .timestamp_ns = static_cast<uint64_t>(timestamp_ns)};
      for (auto& itr : publisher_map_) {
        itr.second->SetPublishContext(publish_context);
        itr.second->PublishSensorData();
      }

      // step
      if (sim->run) {
        ApplyGate3BallDrag();
        mj_step(m_, d_);
        UpdateGate3BallContacts();
        sim->AddToHistory();
        const auto debug_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        WriteDebugCsv(static_cast<std::uint64_t>(debug_wall_ns));
      } else {
        mj_forward(m_, d_);
      }

      UpdateDefaultCameraFollowLocked();
    }
  }

  AIMRT_INFO("SimLoop exit.");
  sim_loop_exited_ = true;

  co_return;
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module
