#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "a3_pingpong/pp_sha256.hpp"

namespace a3_pingpong {

// Unitree-style policy-directory contract.  The common fields intentionally
// mirror unitree_rl_lab's params/deploy.yaml.  HOPE adds only the observation
// dimensions and q_des hard-limit contract needed by the ping-pong plugin.
struct PpDeployConfig {
  static constexpr int kJointCount = 31;

  std::string path;
  std::string task_plugin;
  std::string policy_sha256;
  std::string actor_observation_contract;
  std::string action_contract;
  std::string runtime_contract;
  std::string training_recipe;
  std::string training_recipe_version;
  double step_dt = 0.0;
  double actual_q_hard_tolerance_rad = 0.0;
  int observation_dim = 0;
  int action_dim = 0;
  std::array<int, kJointCount> joint_ids_map{};
  std::vector<std::string> joint_sdk_names;
  std::vector<std::string> joint_names;
  std::vector<std::string> observation_names;
  std::vector<int> observation_dims;
  Eigen::VectorXd default_q;
  Eigen::VectorXd action_scale;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd safe_lower;
  Eigen::VectorXd safe_upper;
  Eigen::VectorXd hard_lower;
  Eigen::VectorXd hard_upper;

  static PpDeployConfig Load(const std::string& deploy_path,
                             const std::string& policy_path) {
    const YAML::Node root = YAML::LoadFile(deploy_path);
    PpDeployConfig out;
    out.path = deploy_path;
    RequireScalar_(root, "format");
    if (root["format"].as<std::string>() != "unitree_rl_lab.deploy" ||
        !root["format_version"] || root["format_version"].as<int>() != 1) {
      throw std::runtime_error(
          "deploy.yaml must use format=unitree_rl_lab.deploy version=1");
    }
    out.task_plugin = Required_<std::string>(root, "task_plugin");
    out.step_dt = Required_<double>(root, "step_dt");
    if (!std::isfinite(out.step_dt) || out.step_dt <= 0.0) {
      throw std::runtime_error("deploy.yaml step_dt must be finite and positive");
    }

    out.joint_sdk_names = Vector_<std::string>(root, "joint_sdk_names");
    out.joint_names = Vector_<std::string>(
        root["actions"]["JointPositionAction"], "joint_names");
    const auto ids = Vector_<int>(root, "joint_ids_map");
    if (out.joint_sdk_names.size() != kJointCount ||
        out.joint_names.size() != kJointCount || ids.size() != kJointCount) {
      throw std::runtime_error(
          "deploy.yaml A3 joint_sdk_names/joint_names/joint_ids_map must have 31 entries");
    }
    std::array<int, kJointCount> hits{};
    for (int index = 0; index < kJointCount; ++index) {
      const int sdk = ids[index];
      if (sdk < 0 || sdk >= kJointCount || ++hits[sdk] != 1) {
        throw std::runtime_error("deploy.yaml joint_ids_map is not a 31-slot bijection");
      }
      out.joint_ids_map[index] = sdk;
      if (out.joint_names[index] != out.joint_sdk_names[sdk]) {
        throw std::runtime_error(
            "deploy.yaml joint_ids_map does not map policy joint names to SDK names");
      }
    }

    const YAML::Node action = root["actions"]["JointPositionAction"];
    out.default_q = EigenVector_(root, "default_joint_pos", kJointCount);
    out.action_scale = EigenVector_(action, "scale", kJointCount);
    const Eigen::VectorXd offset = EigenVector_(action, "offset", kJointCount);
    RequireClose_(offset, out.default_q, "action offset/default_joint_pos");

    const Eigen::VectorXd stiffness_sdk =
        EigenVector_(root, "stiffness", kJointCount);
    const Eigen::VectorXd damping_sdk =
        EigenVector_(root, "damping", kJointCount);
    out.kp.resize(kJointCount);
    out.kd.resize(kJointCount);
    for (int index = 0; index < kJointCount; ++index) {
      out.kp[index] = stiffness_sdk[out.joint_ids_map[index]];
      out.kd[index] = damping_sdk[out.joint_ids_map[index]];
    }

    const YAML::Node clip = action["clip"];
    if (!clip || !clip.IsSequence() ||
        static_cast<int>(clip.size()) != kJointCount) {
      throw std::runtime_error(
          "deploy.yaml JointPositionAction.clip must contain 31 [lo,hi] pairs");
    }
    out.safe_lower.resize(kJointCount);
    out.safe_upper.resize(kJointCount);
    for (int index = 0; index < kJointCount; ++index) {
      if (!clip[index].IsSequence() || clip[index].size() != 2) {
        throw std::runtime_error("deploy.yaml action clip entry is not [lo,hi]");
      }
      out.safe_lower[index] = clip[index][0].as<double>();
      out.safe_upper[index] = clip[index][1].as<double>();
    }

    const YAML::Node action_contract = root["contracts"]["action"];
    out.action_contract = Required_<std::string>(action_contract, "name");
    out.action_dim = Required_<int>(action_contract, "total_dim");
    out.hard_lower = EigenVector_(action_contract, "hard_lower_rad", kJointCount);
    out.hard_upper = EigenVector_(action_contract, "hard_upper_rad", kJointCount);
    out.actual_q_hard_tolerance_rad =
        Required_<double>(action_contract, "actual_q_hard_tolerance_rad");
    if (!std::isfinite(out.actual_q_hard_tolerance_rad) ||
        out.actual_q_hard_tolerance_rad < 0.0) {
      throw std::runtime_error(
          "deploy.yaml actual_q_hard_tolerance_rad is invalid");
    }
    for (int index = 0; index < kJointCount; ++index) {
      if (!(out.hard_lower[index] <= out.safe_lower[index] &&
            out.safe_lower[index] < out.default_q[index] &&
            out.default_q[index] < out.safe_upper[index] &&
            out.safe_upper[index] <= out.hard_upper[index])) {
        throw std::runtime_error(
            "deploy.yaml contains an invalid hard/safe/default q_des interval");
      }
    }

    const YAML::Node obs_contract = root["contracts"]["actor_observation"];
    out.actor_observation_contract =
        Required_<std::string>(obs_contract, "name");
    out.observation_dim = Required_<int>(obs_contract, "total_dim");
    const YAML::Node observations = root["observations"]["policy"];
    if (!observations || !observations.IsMap()) {
      throw std::runtime_error("deploy.yaml observations.policy must be a map");
    }
    int observed_total = 0;
    for (const auto& term : observations) {
      const std::string name = term.first.as<std::string>();
      const int dim = Required_<int>(term.second, "dim");
      const int history = Required_<int>(term.second, "history_length");
      if (dim <= 0 || history <= 0) {
        throw std::runtime_error("deploy.yaml observation dim/history must be positive");
      }
      out.observation_names.push_back(name);
      out.observation_dims.push_back(dim * history);
      observed_total += dim * history;
    }
    if (observed_total != out.observation_dim) {
      throw std::runtime_error(
          "deploy.yaml observation terms do not sum to contracts.actor_observation.total_dim");
    }

    out.runtime_contract = root["contracts"]["runtime"]
                               ? root["contracts"]["runtime"].as<std::string>()
                               : std::string{};
    const YAML::Node recipe = root["contracts"]["training_recipe"];
    out.training_recipe = recipe && recipe["name"]
                              ? recipe["name"].as<std::string>()
                              : std::string{};
    out.training_recipe_version = recipe && recipe["version"]
                                      ? recipe["version"].as<std::string>()
                                      : std::string{};

    out.policy_sha256 =
        Required_<std::string>(root["provenance"], "policy_sha256");
    if (!IsSha256_(out.policy_sha256)) {
      throw std::runtime_error("deploy.yaml policy_sha256 is malformed");
    }
    const std::string actual_policy_sha = PpSha256::File(policy_path);
    if (actual_policy_sha != out.policy_sha256) {
      throw std::runtime_error(
          "deploy.yaml policy_sha256 does not match exported/policy.onnx");
    }
    return out;
  }

 private:
  template <typename T>
  static T Required_(const YAML::Node& node, const char* key) {
    if (!node || !node[key]) {
      throw std::runtime_error(std::string("deploy.yaml missing ") + key);
    }
    return node[key].as<T>();
  }

  static void RequireScalar_(const YAML::Node& node, const char* key) {
    if (!node || !node[key] || !node[key].IsScalar()) {
      throw std::runtime_error(
          std::string("deploy.yaml missing scalar ") + key);
    }
  }

  template <typename T>
  static std::vector<T> Vector_(const YAML::Node& node, const char* key) {
    if (!node || !node[key] || !node[key].IsSequence()) {
      throw std::runtime_error(
          std::string("deploy.yaml missing vector ") + key);
    }
    return node[key].as<std::vector<T>>();
  }

  static Eigen::VectorXd EigenVector_(const YAML::Node& node,
                                      const char* key, int expected) {
    const std::vector<double> values = Vector_<double>(node, key);
    if (static_cast<int>(values.size()) != expected) {
      throw std::runtime_error(
          std::string("deploy.yaml vector length mismatch: ") + key);
    }
    Eigen::VectorXd out(expected);
    for (int index = 0; index < expected; ++index) {
      if (!std::isfinite(values[index])) {
        throw std::runtime_error(
            std::string("deploy.yaml non-finite value in ") + key);
      }
      out[index] = values[index];
    }
    return out;
  }

  static void RequireClose_(const Eigen::VectorXd& lhs,
                            const Eigen::VectorXd& rhs,
                            const char* label,
                            double tolerance = 1.0e-7) {
    if (lhs.size() != rhs.size() ||
        (lhs - rhs).cwiseAbs().maxCoeff() > tolerance) {
      throw std::runtime_error(
          std::string("deploy.yaml mismatch: ") + label);
    }
  }

  static bool IsSha256_(const std::string& value) {
    return value.size() == 64 &&
           std::all_of(value.begin(), value.end(), [](unsigned char ch) {
             return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
           });
  }
};

}  // namespace a3_pingpong
