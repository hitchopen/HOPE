// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "pid_control_module/pid_control_module.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "pid_control_module/global.h"
#include "yaml-cpp/yaml.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

bool PidControlModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  SetLogger(core_.GetLogger());

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    AIMRT_CHECK_ERROR_THROW(!file_path.empty(), "Can not get cfg file.");

    YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));

    auto topic_name_sub = cfg_node["topic_name_sub"].as<std::string>();
    auto topic_name_pub = cfg_node["topic_name_pub"].as<std::string>();

    // Register joint state subscriber
    auto subscriber = core_.GetChannelHandle().GetSubscriber(topic_name_sub);
    AIMRT_CHECK_ERROR_THROW(subscriber, "Get subscriber for topic '{}' failed.", topic_name_sub);
    bool ret = aimrt::channel::Subscribe<aimrt::protocols::sensor::JointStateArray>(
        subscriber,
        std::bind(&PidControlModule::EventHandle, this, std::placeholders::_1));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

    // Register joint command publisher
    auto publisher = core_.GetChannelHandle().GetPublisher(topic_name_pub);
    AIMRT_CHECK_ERROR_THROW(publisher, "Get publisher for topic '{}' failed.", topic_name_pub);
    ret = aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::JointCommandArray>(publisher);
    AIMRT_CHECK_ERROR_THROW(ret, "Register publishType failed.");
    publisher_proxy_ = std::make_unique<aimrt::channel::PublisherProxy<aimrt::protocols::sensor::JointCommandArray>>(publisher);

    // Register service
    service_ptr_ = std::make_unique<PidControlServiceImpl>(controller_);
    ret = core_.GetRpcHandle().RegisterService(service_ptr_.get());
    AIMRT_CHECK_ERROR_THROW(ret, "Register service failed.");

    // Init pid controller
    controller_.SetPIDParm(PIDParm{
        .Kp = cfg_node["init_kp"].as<double>(),
        .Ki = cfg_node["init_ki"].as<double>(),
        .Kd = cfg_node["init_kd"].as<double>()});

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool PidControlModule::Start() { return true; }

void PidControlModule::Shutdown() {}

void PidControlModule::EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointStateArray>& data) {
  // adjust to [-π, π] range
  double normalized = fmod(data->joints()[1].position(), 2 * M_PI);
  if (normalized > M_PI) {
    normalized -= 2 * M_PI;
  } else if (normalized < -M_PI) {
    normalized += 2 * M_PI;
  }

  double target = data->joints()[0].velocity() * 0.015;

  double control_output = controller_.Compute(target, normalized);

  aimrt::protocols::sensor::JointCommandArray msg;
  auto* joint_cmd = msg.add_joints();
  joint_cmd->set_name("center_joint");
  joint_cmd->set_effort(control_output);

  publisher_proxy_->Publish(msg);
}

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module
