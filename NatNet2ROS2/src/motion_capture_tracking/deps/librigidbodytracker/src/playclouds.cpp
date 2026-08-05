#include "librigidbodytracker/rigid_body_tracker.h"
#include "librigidbodytracker/cloudlog.hpp"
#include "yaml-cpp/yaml.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

using namespace librigidbodytracker;

static void log_stderr(std::string s)
{
  std::cout << s << "\n";
}

static pcl::PointXYZ eig2pcl(Eigen::Vector3f v)
{
  return pcl::PointXYZ(v.x(), v.y(), v.z());
}


Eigen::Vector3f asVec(YAML::Node const &node)
{
  assert(node.IsSequence());
  assert(node.size() == 3);
  return Eigen::Vector3f(
    node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
}

static void readConfig(
  const std::string& cfgfile,
  std::vector<DynamicsConfiguration>& dynamicsConfigurations,
  std::vector<MarkerConfiguration>& markerConfigurations,
  std::vector<RigidBody>& rigidBodies)
{
  YAML::Node cfg = YAML::LoadFile(cfgfile);

  // read dynamics
  auto dynRoot = cfg["dynamics_configurations"];
  assert(dynRoot.IsMap());

  std::map<std::string, size_t> dynamics_name_to_index;
  size_t i = 0;
  dynamicsConfigurations.clear();
  for (auto &&dyn : dynRoot) {
    auto val = dyn.second; // first is key
    assert(val.IsMap());
    dynamicsConfigurations.push_back(librigidbodytracker::DynamicsConfiguration());
    auto &conf = dynamicsConfigurations.back();
    Eigen::Vector3f max_vel = asVec(val["max_velocity"]);
    conf.maxXVelocity = max_vel(0);
    conf.maxYVelocity = max_vel(1);
    conf.maxZVelocity = max_vel(2);
    Eigen::Vector3f max_angular_vel = asVec(val["max_angular_velocity"]);
    conf.maxPitchRate = max_angular_vel(0);
    conf.maxRollRate = max_angular_vel(1);
    conf.maxYawRate = max_angular_vel(2);
    conf.maxRoll = val["max_roll"].as<float>();
    conf.maxPitch = val["max_pitch"].as<float>();
    conf.maxFitnessScore = val["max_fitness_score"].as<float>();

    dynamics_name_to_index[dyn.first.as<std::string>()] = i;
    ++i;
  }

  // read marker config

  auto markerRoot = cfg["marker_configurations"];
  assert(markerRoot.IsMap());

  std::map<std::string, size_t> marker_name_to_index;
  i = 0;
  markerConfigurations.clear();
  for (auto &&config : markerRoot) {    
    auto val = config.second; // first is key
    assert(val.IsMap());
    auto offset = asVec(val["offset"]);
    markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>));
    for (auto &&point : val["points"]) {
      auto pt = asVec(point.second) + offset;
      markerConfigurations.back()->push_back(eig2pcl(pt));
    }

    marker_name_to_index[config.first.as<std::string>()] = i;
    ++i;
  }

  // read rigid bodies

  auto rbs = cfg["rigid_bodies"];
  assert(rbs.IsMap());

  for (auto &&rb : rbs) {
    auto val = rb.second; // first is key
    auto initPos = val["initial_position"];
    Eigen::Affine3f xf(Eigen::Translation3f(asVec(initPos)));
    rigidBodies.emplace_back(
      marker_name_to_index.at(val["marker"].as<std::string>()),
      dynamics_name_to_index.at(val["dynamics"].as<std::string>()),
      xf,     // Eigen::Affine3f xf(Eigen::Translation3f(asVec(initPos)));
      rb.first.as<std::string>());
  }
}
  
int main(int argc, char **argv)
{
  using namespace librigidbodytracker;

  if (argc < 3) {
    std::cerr << "use arguments: <cfg> <recording> [<debuglog>]\n";
    return -1;
  }

  std::vector<DynamicsConfiguration> dynamicsConfigurations;
  std::vector<MarkerConfiguration> markerConfigurations;
  std::vector<RigidBody> rigidBodies;

  readConfig(argv[1], dynamicsConfigurations, markerConfigurations, rigidBodies);

  std::cout << dynamicsConfigurations.size() << " dynamics configurations, "
            << markerConfigurations.size() << " marker configurations, "
            << rigidBodies.size() << " rigid bodies.\n";

  librigidbodytracker::RigidBodyTracker tracker(
      dynamicsConfigurations,
      markerConfigurations,
      rigidBodies);

  tracker.setLogWarningCallback(&log_stderr);
  if (argc < 4) {
    PointCloudPlayer player;
    player.load(argv[2]);
    player.play(tracker);
  }
  else {
    PointCloudDebugger debugger(argv[3]);
    debugger.load(argv[2]);
    debugger.convert(tracker,markerConfigurations);
  }
}
