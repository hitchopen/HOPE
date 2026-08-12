#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>
#include <motion_capture_tracking_interfaces/msg/named_pose_array_v2.hpp>
#include <motion_capture_tracking_interfaces/msg/rigid_body_marker_array.hpp>

// Motion Capture
#include <libmotioncapture/motioncapture.h>
#include <motion_capture_tracking/competition_rigid_body_filter.h>
#include <motion_capture_tracking/output_rate_limiter.h>

std::set<std::string> extract_names(
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
  const std::string& pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

std::vector<double> get_vec(const rclcpp::ParameterValue& param_value)
{
  if (param_value.get_type() == rclcpp::PARAMETER_INTEGER_ARRAY) {
    const auto int_vec = param_value.get<std::vector<int64_t>>();
    std::vector<double> result;
    for (int v : int_vec) {
      result.push_back(v);
    }
    return result;
  }
  return param_value.get<std::vector<double>>();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_capture_tracking_node");
  node->declare_parameter<std::string>("type", "vicon");
  node->declare_parameter<std::string>("hostname", "localhost");
  node->declare_parameter<std::string>("topics.frame_id", "world");
  node->declare_parameter<std::string>("topics.header_time", "ros");
  node->declare_parameter<double>("topics.output_rate_hz", 200.0);
  node->declare_parameter<double>("topics.network_latency_ms", 0.0);
  node->declare_parameter<double>("topics.max_clock_sync_uncertainty_ms", 2.0);
  node->declare_parameter<double>("topics.max_capture_age_ms", 100.0);
  node->declare_parameter<uint8_t>("topics.poses.version", 1);
  node->declare_parameter<std::string>("topics.poses.qos.mode", "none");
  node->declare_parameter<double>("topics.poses.qos.deadline", 100.0);
  node->declare_parameter<bool>("topics.rigid_body_markers.enabled", false);
  node->declare_parameter<std::string>(
      "topics.rigid_body_markers.asset_name", "P1");
  node->declare_parameter<double>(
      "topics.rigid_body_markers.geometric_match_max_distance_m", 0.005);
  node->declare_parameter<bool>(
      "topics.rigid_body_markers.modeldef_y_up_to_z_up", false);

  std::string motionCaptureType = node->get_parameter("type").as_string();
  std::string motionCaptureHostname = node->get_parameter("hostname").as_string();
  std::string frame_id = node->get_parameter("topics.frame_id").as_string();
  std::string header_time = node->get_parameter("topics.header_time").as_string();
  double output_rate_hz = node->get_parameter("topics.output_rate_hz").as_double();
  double network_latency_ms = node->get_parameter("topics.network_latency_ms").as_double();
  double max_clock_sync_uncertainty_ms =
    node->get_parameter("topics.max_clock_sync_uncertainty_ms").as_double();
  double max_capture_age_ms =
    node->get_parameter("topics.max_capture_age_ms").as_double();
  if (!std::isfinite(output_rate_hz) || output_rate_hz < 0.0) {
    throw std::runtime_error("topics.output_rate_hz must be finite and non-negative");
  }
  if (network_latency_ms < 0.0) {
    throw std::runtime_error("topics.network_latency_ms must be non-negative");
  }
  if (max_clock_sync_uncertainty_ms <= 0.0) {
    throw std::runtime_error(
      "topics.max_clock_sync_uncertainty_ms must be positive");
  }
  if (max_capture_age_ms <= 0.0) {
    throw std::runtime_error("topics.max_capture_age_ms must be positive");
  }
  if (header_time != "ros" && header_time != "camera_utc" &&
      header_time != "ros_latency_compensated" && header_time != "camera") {
    throw std::runtime_error(
      "Unknown topics.header_time '" + header_time +
      "' (expected ros, camera_utc, ros_latency_compensated, or camera)");
  }
  if (header_time == "camera_utc" && motionCaptureType != "optitrack") {
    throw std::runtime_error(
      "topics.header_time=camera_utc currently requires type=optitrack");
  }
  if (header_time == "camera" && motionCaptureType == "optitrack") {
    throw std::runtime_error(
      "topics.header_time=camera is not an absolute ROS timestamp for "
      "OptiTrack; Motive reports QPC-relative ticks. Use camera_utc");
  }
  if (
    motionCaptureType == "optitrack" &&
    header_time == "ros_latency_compensated" &&
    network_latency_ms == 0.0)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "topics.network_latency_ms is 0.0; measure the deployed one-way "
      "NatNet network/host latency before moving cross-sensor calibration");
  }
  uint8_t poses_version = node->get_parameter("topics.poses.version").as_int();
  std::string poses_qos = node->get_parameter("topics.poses.qos.mode").as_string();
  double poses_deadline = node->get_parameter("topics.poses.qos.deadline").as_double();
  if (poses_version != 1 && poses_version != 2) {
    throw std::runtime_error(
      "topics.poses.version must be 1 (NamedPoseArray) or 2 "
      "(NamedPoseArrayV2)");
  }
  bool rigid_body_markers_enabled =
      node->get_parameter("topics.rigid_body_markers.enabled").as_bool();
  std::string rigid_body_markers_asset_name =
      node->get_parameter("topics.rigid_body_markers.asset_name").as_string();
  double rigid_body_markers_geometric_match_max_distance =
      node->get_parameter(
          "topics.rigid_body_markers.geometric_match_max_distance_m")
          .as_double();
  if (!std::isfinite(rigid_body_markers_geometric_match_max_distance) ||
      rigid_body_markers_geometric_match_max_distance <= 0.0) {
    throw std::runtime_error(
        "topics.rigid_body_markers.geometric_match_max_distance_m "
        "must be finite and positive");
  }
  bool rigid_body_markers_modeldef_y_up_to_z_up =
      node->get_parameter(
          "topics.rigid_body_markers.modeldef_y_up_to_z_up")
          .as_bool();
  if (poses_qos == "sensor" &&
    (!std::isfinite(poses_deadline) || poses_deadline <= 0.0))
  {
    throw std::runtime_error(
      "topics.poses.qos.deadline must be finite and positive in sensor mode");
  }
  if (poses_qos == "sensor" && output_rate_hz > 0.0 &&
    output_rate_hz <= poses_deadline)
  {
    RCLCPP_WARN(
      node->get_logger(),
      "topics.output_rate_hz (%.3f Hz) is not above the publisher deadline "
      "rate (%.3f Hz); lower topics.poses.qos.deadline or use QoS mode none "
      "to avoid expected deadline misses",
      output_rate_hz, poses_deadline);
  }

  motion_capture_tracking::detail::OutputRateLimiter output_rate_limiter(
    output_rate_hz);
  if (output_rate_hz == 0.0) {
    RCLCPP_INFO(
      node->get_logger(),
      "ROS 2 output downsampling disabled; publishing every valid source frame");
  } else {
    RCLCPP_INFO(
      node->get_logger(), "ROS 2 output rate capped at %.3f Hz", output_rate_hz);
  }

  auto node_parameters_iface = node->get_node_parameters_interface();
  const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
      node_parameters_iface->get_parameter_overrides();

  // Make a new client
  std::map<std::string, std::string> cfg;
  cfg["hostname"] = motionCaptureHostname;
  cfg["enable_clock_sync"] = header_time == "camera_utc" ? "true" : "false";

  // if the mock type is selected, add the defined rigid bodies
  if (motionCaptureType == "mock") {
    auto rigid_body_names = extract_names(parameter_overrides, "rigid_bodies");
    for (const auto &name : rigid_body_names)
    {
      const auto pos = get_vec(parameter_overrides.at("rigid_bodies." + name + ".initial_position"));
      cfg["rigid_bodies"] += name + "(" + std::to_string(pos[0]) + "," + std::to_string(pos[1]) + "," + std::to_string(pos[2]) +",1,0,0,0);";
    }
  }

  libmotioncapture::MotionCapture *mocap = libmotioncapture::MotionCapture::connect(motionCaptureType, cfg);

  // prepare pose array publisher
  rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr pubPoses;
  rclcpp::Publisher<motion_capture_tracking_interfaces::msg::NamedPoseArrayV2>::SharedPtr pubPosesV2;

  if (poses_qos == "none") {
    if (poses_version == 1) {
      pubPoses = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("poses", 1);
    } else if (poses_version == 2) {
      pubPosesV2 = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArrayV2>("poses", 1);
    }
  } else if (poses_qos == "sensor") {
    rclcpp::SensorDataQoS sensor_data_qos;
    sensor_data_qos.keep_last(1);
    sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, (int)1e9/poses_deadline /*ns*/));
    if (poses_version == 1) {
      pubPoses = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArray>("poses", sensor_data_qos);
    } else if (poses_version == 2) {
      pubPosesV2 = node->create_publisher<motion_capture_tracking_interfaces::msg::NamedPoseArrayV2>("poses", sensor_data_qos);
    }
  } else {
    throw std::runtime_error("Unknown QoS mode! " + poses_qos);
  }

  motion_capture_tracking_interfaces::msg::NamedPoseArray msgPoses;
  msgPoses.header.frame_id = frame_id;

  motion_capture_tracking_interfaces::msg::NamedPoseArrayV2 msgPosesV2;
  msgPosesV2.header.frame_id = frame_id;

  std::vector<motion_capture_tracking_interfaces::msg::NamedPose> output_poses;

  // Atomic marker output used by the laptop's per-run calibration service.
  // The standalone adapter default remains disabled unless explicitly enabled.
  rclcpp::Publisher<
      motion_capture_tracking_interfaces::msg::RigidBodyMarkerArray>::SharedPtr
      pubRigidBodyMarkers;
  if (rigid_body_markers_enabled) {
    rclcpp::SensorDataQoS marker_qos;
    marker_qos.keep_last(1);
    pubRigidBodyMarkers = node->create_publisher<
        motion_capture_tracking_interfaces::msg::RigidBodyMarkerArray>(
        "rigid_body_markers", marker_qos);
  }
  // RCL_SYSTEM_TIME is Unix epoch time supplied by CLOCK_REALTIME on Linux.
  // Chrony disciplines that host clock; do not use node->now() for camera_utc
  // because /use_sim_time could put it in an unrelated ROS simulation epoch.
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

  while (rclcpp::ok()) {

    // Get a frame
    mocap->waitForNextFrame();
    rclcpp::Time time;
    if (header_time == "ros") {
      time = node->now();
    } else if (header_time == "camera_utc") {
      if (!mocap->supportsTimeStampAge()) {
        RCLCPP_ERROR_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "camera_utc requested, but the NatNet clock mapping or camera "
          "mid-exposure timestamp is unavailable; dropping frame");
        continue;
      }

      // Bracket the monotonic-age query with two system-clock reads.  The
      // midpoint removes almost all call-order bias; half the bracket is
      // included in the uncertainty budget below.
      const auto system_before = system_clock.now();
      const double capture_age_seconds = mocap->timeStampAge();
      const auto system_after = system_clock.now();
      const double system_clock_bracket_seconds =
        (system_after - system_before).seconds();
      const double clock_uncertainty_seconds =
        mocap->timeStampAgeUncertainty() +
        std::abs(system_clock_bracket_seconds) * 0.5;

      const bool invalid_age = !std::isfinite(capture_age_seconds) ||
        capture_age_seconds < -clock_uncertainty_seconds ||
        capture_age_seconds * 1e3 > max_capture_age_ms;
      const bool invalid_uncertainty =
        !std::isfinite(clock_uncertainty_seconds) ||
        clock_uncertainty_seconds * 1e3 > max_clock_sync_uncertainty_ms;
      const bool system_clock_stepped_back =
        system_clock_bracket_seconds < 0.0;
      if (invalid_age || invalid_uncertainty || system_clock_stepped_back) {
        RCLCPP_ERROR_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "NatNet camera_utc timestamp rejected: age=%.3f ms, mapping "
          "uncertainty=%.3f ms, system bracket=%.3f ms (limits %.3f/%.3f "
          "ms); dropping frame",
          capture_age_seconds * 1e3, clock_uncertainty_seconds * 1e3,
          system_clock_bracket_seconds * 1e3,
          max_capture_age_ms, max_clock_sync_uncertainty_ms);
        continue;
      }

      const int64_t system_mid_ns =
        system_before.nanoseconds() +
        (system_after.nanoseconds() - system_before.nanoseconds()) / 2;
      time = rclcpp::Time(system_mid_ns, RCL_SYSTEM_TIME) -
        rclcpp::Duration::from_seconds(capture_age_seconds);
    } else if (header_time == "ros_latency_compensated") {
      // Put the exposure estimate in the local ROS clock epoch without using
      // Motive's unrelated high-resolution-clock epoch. NatNet >= 3 reports
      // camera and Motive processing latency for every frame. The remaining
      // one-way network/host receive latency is a measured deployment
      // parameter (zero by default), not something inferable from one UDP
      // packet.
      double latency_seconds = network_latency_ms * 1e-3;
      const auto& reported_latencies = mocap->latency();
      if (reported_latencies.empty() && motionCaptureType == "optitrack") {
        throw std::runtime_error(
          "ros_latency_compensated requires NatNet per-frame latency data "
          "(NatNet 3 or newer)");
      }
      for (const auto& latency : reported_latencies) {
        latency_seconds += latency.value();
      }
      time = node->now() - rclcpp::Duration::from_seconds(latency_seconds);
    } else if (header_time == "camera") {
      time = rclcpp::Time(mocap->timeStamp() * 1000);
    } else {
      throw std::logic_error("validated topics.header_time became unreachable");
    }

    // Use a monotonic publication schedule, but keep the selected source
    // frame's acquisition timestamp unchanged. All source frames have already
    // reached the NatNet backend and timestamp checks; only ROS output is
    // downsampled.
    const double output_gate_now_seconds =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool publish_this_frame =
      output_rate_limiter.shouldPublish(output_gate_now_seconds);

    if (publish_this_frame) {
      output_poses.clear();
      const auto &rigid_bodies = mocap->rigidBodies();

      // Per-run calibration input. Publishing this stream does not itself
      // recalculate anything; the laptop service samples it once after each
      // settled PREPARE and atomically replaces its local JSON.
      if (pubRigidBodyMarkers) {
        const auto& definitions = mocap->rigidBodyDefinitions();
        const auto& labeled_markers = mocap->labeledMarkers();
        for (const auto& definition_entry : definitions) {
          const auto& definition = definition_entry.second;
          if (!rigid_body_markers_asset_name.empty() &&
              definition.name != rigid_body_markers_asset_name) {
            continue;
          }
          const auto rigid_body_iter = rigid_bodies.find(definition.name);
          if (rigid_body_iter == rigid_bodies.end()) {
            continue;
          }
          const auto& rigid_body = rigid_body_iter->second;

          motion_capture_tracking_interfaces::msg::RigidBodyMarkerArray output;
          output.header.stamp = time;
          output.header.frame_id = frame_id;
          output.timestamp = mocap->timeStamp();
          output.rigid_body_id = definition.id;
          output.rigid_body_name = definition.name;
          output.rigid_body_pose.position.x = rigid_body.position().x();
          output.rigid_body_pose.position.y = rigid_body.position().y();
          output.rigid_body_pose.position.z = rigid_body.position().z();
          output.rigid_body_pose.orientation.x = rigid_body.rotation().x();
          output.rigid_body_pose.orientation.y = rigid_body.rotation().y();
          output.rigid_body_pose.orientation.z = rigid_body.rotation().z();
          output.rigid_body_pose.orientation.w = rigid_body.rotation().w();
          output.mean_marker_error_m = rigid_body.meanMarkerError();
          output.markers.reserve(definition.markers.size());

          auto model_position_in_stream_axes =
              [&](const libmotioncapture::RigidBodyMarkerDefinition& marker) {
                const auto& position = marker.position;
                if (rigid_body_markers_modeldef_y_up_to_z_up) {
                  return Eigen::Vector3f(
                      position.x(), -position.z(), position.y());
                }
                return position;
              };
          const uint32_t model_id =
              static_cast<uint32_t>(definition.id) & 0xffffU;
          std::vector<const libmotioncapture::LabeledMarker*> samples(
              definition.markers.size(), nullptr);
          std::set<size_t> used_sample_indices;

          for (size_t marker_index = 0;
               marker_index < definition.markers.size(); ++marker_index) {
            const auto& marker = definition.markers[marker_index];
            for (size_t sample_index = 0;
                 sample_index < labeled_markers.size(); ++sample_index) {
              const auto& sample = labeled_markers[sample_index];
              if (sample.modelId == model_id &&
                  sample.memberId == marker.memberId) {
                samples[marker_index] = &sample;
                used_sample_indices.insert(sample_index);
                break;
              }
            }
          }

          struct Candidate {
            float distance;
            size_t marker_index;
            size_t sample_index;
          };
          std::vector<Candidate> candidates;
          for (size_t marker_index = 0;
               marker_index < definition.markers.size(); ++marker_index) {
            if (samples[marker_index] != nullptr) {
              continue;
            }
            const Eigen::Vector3f expected_world =
                rigid_body.position() + rigid_body.rotation() *
                model_position_in_stream_axes(definition.markers[marker_index]);
            for (size_t sample_index = 0;
                 sample_index < labeled_markers.size(); ++sample_index) {
              if (used_sample_indices.count(sample_index) != 0U) {
                continue;
              }
              const auto& sample = labeled_markers[sample_index];
              const bool visible_point_cloud_sample =
                  (sample.params & 0x01U) == 0U &&
                  (sample.params & 0x02U) != 0U;
              if (!visible_point_cloud_sample) {
                continue;
              }
              const float distance =
                  (sample.position - expected_world).norm();
              if (distance <=
                  rigid_body_markers_geometric_match_max_distance) {
                candidates.push_back(
                    {distance, marker_index, sample_index});
              }
            }
          }
          std::sort(
              candidates.begin(), candidates.end(),
              [](const Candidate& left, const Candidate& right) {
                return left.distance < right.distance;
              });
          bool used_geometric_fallback = false;
          for (const auto& candidate : candidates) {
            if (samples[candidate.marker_index] != nullptr ||
                used_sample_indices.count(candidate.sample_index) != 0U) {
              continue;
            }
            samples[candidate.marker_index] =
                &labeled_markers[candidate.sample_index];
            used_sample_indices.insert(candidate.sample_index);
            used_geometric_fallback = true;
          }
          if (used_geometric_fallback) {
            RCLCPP_WARN_ONCE(
                node->get_logger(),
                "P1 labeled-marker IDs require geometric association "
                "(max %.1f mm)",
                rigid_body_markers_geometric_match_max_distance * 1000.0);
          }

          for (size_t marker_index = 0;
               marker_index < definition.markers.size(); ++marker_index) {
            const auto& definition_marker = definition.markers[marker_index];
            output.markers.emplace_back();
            auto& message_marker = output.markers.back();
            message_marker.member_id = definition_marker.memberId;
            message_marker.name = definition_marker.name;
            const Eigen::Vector3f model_position =
                model_position_in_stream_axes(definition_marker);
            message_marker.model_position.x = model_position.x();
            message_marker.model_position.y = model_position.y();
            message_marker.model_position.z = model_position.z();
            message_marker.required_active_label =
                definition_marker.requiredActiveLabel;
            message_marker.has_live_sample = false;
            message_marker.position.x =
                std::numeric_limits<double>::quiet_NaN();
            message_marker.position.y =
                std::numeric_limits<double>::quiet_NaN();
            message_marker.position.z =
                std::numeric_limits<double>::quiet_NaN();
            message_marker.size_m =
                std::numeric_limits<float>::quiet_NaN();
            message_marker.residual_m =
                std::numeric_limits<float>::quiet_NaN();
            const auto* sample = samples[marker_index];
            if (sample != nullptr) {
              message_marker.has_live_sample = true;
              message_marker.position.x = sample->position.x();
              message_marker.position.y = sample->position.y();
              message_marker.position.z = sample->position.z();
              message_marker.size_m = sample->size;
              message_marker.params = sample->params;
              message_marker.residual_m = sample->residual;
            }
          }
          pubRigidBodyMarkers->publish(output);
        }
      }

      // The standalone adapter has one deliberately narrow ROS contract:
      // publish only competition rigid bodies, in canonical order. Motive may
      // stream setup assets, skeletons, marker sets, or other rigid bodies;
      // none of them are allowed onto ROS 2. A body missing from the current
      // frame is simply omitted, with no placeholder. If none are present,
      // publish an empty array heartbeat. That keeps transport/adapter
      // liveness distinguishable from competition-body tracking loss while
      // exposing no non-allowlisted data.
      for (const auto allowed_name :
        motion_capture_tracking::detail::competitionRigidBodyNames())
      {
        for (const auto &entry : rigid_bodies) {
          const auto &rigid_body = entry.second;
          if (rigid_body.name() != allowed_name) {
            continue;
          }

          motion_capture_tracking_interfaces::msg::NamedPose named_pose;
          named_pose.name = std::string(allowed_name);
          named_pose.pose.position.x = rigid_body.position().x();
          named_pose.pose.position.y = rigid_body.position().y();
          named_pose.pose.position.z = rigid_body.position().z();
          named_pose.pose.orientation.x = rigid_body.rotation().x();
          named_pose.pose.orientation.y = rigid_body.rotation().y();
          named_pose.pose.orientation.z = rigid_body.rotation().z();
          named_pose.pose.orientation.w = rigid_body.rotation().w();
          output_poses.push_back(std::move(named_pose));
          break;
        }
      }

      if (output_poses.empty()) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "valid NatNet frames contain none of the exact-name competition "
          "bodies Ball, P1, or P2; publishing an empty heartbeat");
      }

      if (poses_version == 1) {
        msgPoses.header.stamp = time;
        msgPoses.poses = output_poses;
        pubPoses->publish(msgPoses);
      } else if (poses_version == 2) {
        msgPosesV2.header.stamp = time;
        msgPosesV2.timestamp = mocap->timeStamp();
        const auto& latencies = mocap->latency();
        msgPosesV2.latencies.resize(latencies.size());
        for (size_t i = 0; i < latencies.size(); ++i) {
          msgPosesV2.latencies[i].source = latencies[i].name();
          msgPosesV2.latencies[i].latency = latencies[i].value() * 1e6;
        }
        msgPosesV2.poses = output_poses;
        pubPosesV2->publish(msgPosesV2);
      }
    }
    rclcpp::spin_some(node);
  }

  return 0;
  }
