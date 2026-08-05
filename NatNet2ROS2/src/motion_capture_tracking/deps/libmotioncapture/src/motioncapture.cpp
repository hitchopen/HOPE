#include "libmotioncapture/motioncapture.h"
#include "libmotioncapture/mock.h"
#ifdef ENABLE_VICON
#include "libmotioncapture/vicon.h"
#endif
#ifdef ENABLE_OPTITRACK
#include "libmotioncapture/optitrack.h"
#endif
#ifdef ENABLE_OPTITRACK_CLOSED_SOURCE
#include "libmotioncapture/optitrack_closed_source.h"
#endif
#ifdef ENABLE_QUALISYS
#include "libmotioncapture/qualisys.h"
#endif
#ifdef ENABLE_NOKOV
#include "libmotioncapture/nokov.h"
#endif
#ifdef ENABLE_VRPN
#include "libmotioncapture/vrpn.h"
#endif
#ifdef ENABLE_MOTIONANALYSIS
#include "libmotioncapture/motionanalysis.h"
#endif
#ifdef ENABLE_FZMOTION
#include "libmotioncapture/fzmotion.h"
#endif

namespace libmotioncapture {

  const char *version_string =
    #include "../version"
    ;

  const char *version()
  {
    return version_string;
  }

  RigidBody MotionCapture::rigidBodyByName(
      const std::string& name) const
  {
    const auto& obj = rigidBodies();
    const auto iter = obj.find(name);
    if (iter != obj.end()) {
      return iter->second;
    }
    throw std::runtime_error("Rigid body not found!");
  }

  std::string getString(
    const std::map<std::string, std::string> &cfg,
    const std::string& key,
    const std::string& default_value)
  {
    const auto iter = cfg.find(key);
    if (iter != cfg.end()) {
      return iter->second;
    }
    return default_value;
  }

  bool getBool(
    const std::map<std::string, std::string> &cfg,
    const std::string& key,
    bool default_value)
  {
    const auto iter = cfg.find(key);
    if (iter != cfg.end()) {
      if (iter->second == "1" || iter->second == "true") {
        return true;
      }
      return false;
    }
    return default_value;
  }

  int getInt(
      const std::map<std::string, std::string> &cfg,
      const std::string &key,
      int default_value)
  {
    const auto iter = cfg.find(key);
    if (iter != cfg.end())
    {
      return std::stoi(iter->second);
    }
    return default_value;
  }

  MotionCapture *MotionCapture::connect(
      const std::string &type,
      const std::map<std::string, std::string> &cfg)
  {
    MotionCapture* mocap = nullptr;

    if (false)
    {
    }
    else if (type == "mock")
    {

      // read rigid bodies from string
      // e.g., "rb1(x,y,z,qw,qx,qy,qz);rb2(x,y,z,qw,qx,qy,qz)"

      std::vector<libmotioncapture::RigidBody> rigidBodies;
      auto rbstring = getString(cfg, "rigid_bodies", "");
      size_t pos1 = 0, pos2 = 0;
      while (pos1 <= rbstring.size()) {
        pos2 = rbstring.find(';', pos1);
        if (pos2 == std::string::npos) {
          pos2 = rbstring.size();
        }
        auto rbstr = rbstring.substr(pos1, pos2-pos1);
        float x, y, z, qw, qx, qy, qz;
        char name[100];
        int scanned = std::sscanf(rbstr.c_str(), "%[^(](%f,%f,%f,%f,%f,%f,%f)", name, &x, &y, &z, &qw, &qx, &qy, &qz);
        if (scanned == 8) {
          Eigen::Vector3f pos(x,y,z);
          Eigen::Quaternionf rot(qw, qx, qy, qz);
          rigidBodies.emplace_back(libmotioncapture::RigidBody(std::string(name), pos, rot));
        } else {
          break;
        }
        pos1 = pos2 + 1;
      }

      // read pointcloud from string
      // e.g., "x,y,z;x,y,z"
      
      PointCloud pc;
      auto pcstring = getString(cfg, "pointcloud", "");
      pos1 = 0, pos2 = 0;
      while (pos1 <= pcstring.size()) {
        pos2 = pcstring.find(';', pos1);
        if (pos2 == std::string::npos) {
          pos2 = pcstring.size();
        }
        auto pcstr = pcstring.substr(pos1, pos2-pos1);
        float x, y, z;
        int scanned = std::sscanf(pcstr.c_str(), "%f,%f,%f", &x, &y, &z);
        if (scanned == 3) {
          pc.conservativeResize(pc.rows()+1, Eigen::NoChange);
          pc.row(pc.rows()-1) << x, y, z;
        } else {
          break;
        }
        pos1 = pos2 + 1;
      }
      // pc.resize(4, Eigen::NoChange);
      // pc.row(0) << 0, 0, 0;
      // pc.row(1) << 0, 0.5, 0;
      // pc.row(2) << 0, -0.5, 0;
      // pc.row(3) << 0.5, 0, 0;

      mocap = new libmotioncapture::MotionCaptureMock(
        1.0f / getInt(cfg, "frequency", 100),
        rigidBodies, pc);
    }
#ifdef ENABLE_VICON
    else if (type == "vicon")
    {
      mocap = new libmotioncapture::MotionCaptureVicon(
        getString(cfg, "hostname", "localhost"),
        getBool(cfg, "enable_objects", true),
        getBool(cfg, "enable_pointclout", true));
    }
#endif
#ifdef ENABLE_OPTITRACK
    else if (type == "optitrack")
    {
      mocap = new libmotioncapture::MotionCaptureOptitrack(
        getString(cfg, "hostname", "localhost"),
        getString(cfg, "interface_ip", "0.0.0.0"),
        getInt(cfg, "port_command", 1510),
        getBool(cfg, "enable_clock_sync", false));
    }
#endif
#ifdef ENABLE_OPTITRACK_CLOSED_SOURCE
    else if (type == "optitrack_closed_source")
    {
      mocap = new libmotioncapture::MotionCaptureOptitrackClosedSource(
          getString(cfg, "hostname", "localhost"),
          getInt(cfg, "port_command", 1510));
    }
#endif
#ifdef ENABLE_QUALISYS
    else if (type == "qualisys")
    {
      mocap = new libmotioncapture::MotionCaptureQualisys(
        getString(cfg, "hostname", "localhost"),
        getInt(cfg, "port", 22222),
        getBool(cfg, "enable_objects", true),
        getBool(cfg, "enable_pointcloud", true));
    }
#endif
#ifdef ENABLE_NOKOV
	else if (type == "nokov")
	{
        mocap = new libmotioncapture::MotionCaptureNokov(
            getString(cfg, "hostname", "localhost"),
            getBool(cfg, "enableFrequency", false),
            getInt(cfg, "updateFrequency", 100));
  }
#endif
#ifdef ENABLE_VRPN
    else if (type == "vrpn")
    {
      mocap = new libmotioncapture::MotionCaptureVrpn(
        getString(cfg, "hostname", "localhost"));
    }
#endif
#ifdef ENABLE_MOTIONANALYSIS
    else if (type == "motionanalysis")
    {
      mocap = new libmotioncapture::MotionCaptureMotionAnalysis(
        getString(cfg, "hostname", "localhost"));
    }
#endif
#ifdef ENABLE_FZMOTION
    else if (type == "fzmotion")
    {
      mocap = libmotioncapture::MotionCaptureFZMotion::getInstance(
        getString(cfg, "local_IP", "0.0.0.0"),
        getInt(cfg, "local_port", 9762),
        getString(cfg, "hostname", "fzmotion"),
        getInt(cfg, "remote_port", 9761));
    }
#endif
    else
    {
      throw std::runtime_error("Unknown motion capture type!");
    }

    return mocap;
  }

}
