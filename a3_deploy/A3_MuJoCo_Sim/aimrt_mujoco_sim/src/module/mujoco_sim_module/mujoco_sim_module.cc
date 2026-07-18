// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/mujoco_sim_module.h"
#include "aimrt_module_cpp_interface/co/aimrt_context.h"
#include "aimrt_module_cpp_interface/co/inline_scheduler.h"
#include "aimrt_module_cpp_interface/co/on.h"
#include "aimrt_module_cpp_interface/co/schedule.h"
#include "aimrt_module_cpp_interface/co/sync_wait.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"
#include "mujoco_sim_module/publisher/joint_sensor_publisher.h"
#include "mujoco_sim_module/publisher/touch_sensor_publisher.h"
#include "mujoco_sim_module/subscriber/joint_actuator_subscriber.h"
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include "mujoco_sim_module/publisher/pose_twist_ros2_publisher.h"
  #include "mujoco_sim_module/subscriber/sim_reset_ros2_subscriber.h"
#endif

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

  d_ = mj_makeData(m_);
  AIMRT_CHECK_ERROR_THROW(d_ != nullptr, "Make data failed.");

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
        mj_step(m_, d_);
        sim->AddToHistory();
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
