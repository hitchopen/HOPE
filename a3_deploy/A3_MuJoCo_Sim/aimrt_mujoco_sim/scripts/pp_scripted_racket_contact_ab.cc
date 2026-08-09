// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.
//
// Isolated Gate3 diagnostic: compare MuJoCo's raw soft-contact response with
// the explicit planner/training paddle map under identical scripted impacts.

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "common/gate3_ball_contact_model.h"

namespace {

namespace gate3 =
    aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball;

constexpr double kBallMassKg = 0.0034;
constexpr double kBallRadiusM = 0.020;
constexpr double kPaddleHalfThicknessM = 0.003;
constexpr double kInitialBallXM = 0.150;
constexpr double kSpeedCeilingMps = 8.0;
constexpr double kMapToleranceMps = 1.0e-6;

constexpr char kModelXml[] = R"xml(
<mujoco model="pp_scripted_racket_contact_ab">
  <compiler angle="radian"/>
  <option timestep="0.001" gravity="0 0 0"
          integrator="Euler" noslip_iterations="3"
          noslip_tolerance="1e-6"/>
  <worldbody>
    <body name="paddle" pos="0 0 0">
      <joint name="paddle_slide" type="slide" axis="1 0 0"
             limited="false" damping="0"/>
      <inertial pos="0 0 0" mass="20"
                diaginertia="0.50 0.50 0.50"/>
      <geom name="paddle_collision" type="box"
            size="0.003 0.11 0.20"
            contype="1" conaffinity="2"
            friction="0.20 0.001 0.0001"
            solref="0.002 1"/>
    </body>
    <body name="ball" pos="0.15 0 0">
      <freejoint name="ball_free_joint"/>
      <inertial pos="0 0 0" mass="0.0034"
                diaginertia="0.000000906667 0.000000906667 0.000000906667"/>
      <geom name="ball_collision" type="sphere" size="0.020"
            mass="0.0034" contype="2" conaffinity="1"
            friction="0.20 0.001 0.0001"
            solref="0.002 1"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="ball_collision" geom2="paddle_collision"
          condim="3" friction="0.20 0.001 0.0001"
          solref="0.002 1"/>
  </contact>
</mujoco>
)xml";

struct ModelDeleter {
  void operator()(mjModel* model) const {
    if (model != nullptr) mj_deleteModel(model);
  }
};

struct DataDeleter {
  void operator()(mjData* data) const {
    if (data != nullptr) mj_deleteData(data);
  }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

enum class ContactMode { kRaw, kExplicit };

struct ImpactCase {
  std::string name;
  gate3::Vec3 incoming_velocity;
  gate3::Vec3 incoming_spin;
  double racket_speed_x = 0.0;
};

struct ImpactResult {
  std::string name;
  std::string mode;
  gate3::Vec3 requested_incoming{};
  gate3::Vec3 measured_pre{};
  gate3::Vec3 outgoing{};
  gate3::Vec3 expected_outgoing{};
  double racket_speed_x = 0.0;
  double contact_time_s = std::numeric_limits<double>::quiet_NaN();
  int contact_rising_edges = 0;
  double peak_raw_normal_force_n = 0.0;
  double incoming_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double outgoing_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double incoming_ke_j = std::numeric_limits<double>::quiet_NaN();
  double outgoing_ke_j = std::numeric_limits<double>::quiet_NaN();
  double ball_ke_ratio = std::numeric_limits<double>::quiet_NaN();
  double effective_restitution = std::numeric_limits<double>::quiet_NaN();
  double measured_relative_restitution =
      std::numeric_limits<double>::quiet_NaN();
  double map_max_abs_error_mps = std::numeric_limits<double>::quiet_NaN();
  bool contacted = false;
  bool separated = false;
  bool finite = false;
  bool speed_reasonable = false;
  bool map_match = false;
};

double KineticEnergy(double speed_mps) {
  return 0.5 * kBallMassKg * speed_mps * speed_mps;
}

bool FiniteVec(const gate3::Vec3& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

double MaxAbsError(const gate3::Vec3& a, const gate3::Vec3& b) {
  return std::max(
      {std::abs(a[0] - b[0]), std::abs(a[1] - b[1]),
       std::abs(a[2] - b[2])});
}

bool BallPaddleContact(const mjModel* model, const mjData* data,
                       int ball_geom_id, int paddle_geom_id,
                       double* normal_force_n,
                       gate3::Vec3* contact_position) {
  bool found = false;
  *normal_force_n = 0.0;
  for (int contact_id = 0; contact_id < data->ncon; ++contact_id) {
    const mjContact& contact = data->contact[contact_id];
    const bool matches =
        (contact.geom1 == ball_geom_id &&
         contact.geom2 == paddle_geom_id) ||
        (contact.geom1 == paddle_geom_id &&
         contact.geom2 == ball_geom_id);
    if (!matches) continue;

    if (!found) {
      *contact_position = {
          static_cast<double>(contact.pos[0]),
          static_cast<double>(contact.pos[1]),
          static_cast<double>(contact.pos[2])};
    }
    std::array<mjtNum, 6> wrench{};
    mj_contactForce(model, data, contact_id, wrench.data());
    *normal_force_n += std::abs(static_cast<double>(wrench[0]));
    found = true;
  }
  return found;
}

ModelPtr LoadModel() {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  const int add_result = mj_addBufferVFS(
      &vfs, "pp_scripted_racket_contact_ab.xml", kModelXml,
      static_cast<int>(sizeof(kModelXml) - 1));
  if (add_result != 0) {
    mj_deleteVFS(&vfs);
    throw std::runtime_error("mj_addBufferVFS failed");
  }

  std::array<char, 4096> error{};
  ModelPtr model(mj_loadXML(
      "pp_scripted_racket_contact_ab.xml", &vfs, error.data(),
      static_cast<int>(error.size())));
  mj_deleteVFS(&vfs);
  if (!model) {
    throw std::runtime_error(
        std::string("mj_loadXML failed: ") + error.data());
  }
  return model;
}

ImpactResult RunImpact(const ImpactCase& impact, ContactMode mode) {
  ModelPtr model = LoadModel();
  DataPtr data(mj_makeData(model.get()));
  if (!data) throw std::runtime_error("mj_makeData failed");

  const int paddle_joint_id =
      mj_name2id(model.get(), mjOBJ_JOINT, "paddle_slide");
  const int ball_joint_id =
      mj_name2id(model.get(), mjOBJ_JOINT, "ball_free_joint");
  const int paddle_geom_id =
      mj_name2id(model.get(), mjOBJ_GEOM, "paddle_collision");
  const int ball_geom_id =
      mj_name2id(model.get(), mjOBJ_GEOM, "ball_collision");
  if (paddle_joint_id < 0 || ball_joint_id < 0 ||
      paddle_geom_id < 0 || ball_geom_id < 0) {
    throw std::runtime_error("diagnostic model IDs missing");
  }

  const int paddle_qpos = model->jnt_qposadr[paddle_joint_id];
  const int paddle_dof = model->jnt_dofadr[paddle_joint_id];
  const int ball_qpos = model->jnt_qposadr[ball_joint_id];
  const int ball_dof = model->jnt_dofadr[ball_joint_id];
  const double closing_speed =
      impact.racket_speed_x - impact.incoming_velocity[0];
  if (!(closing_speed > 0.0)) {
    throw std::runtime_error("impact case does not close on paddle");
  }

  const double nominal_contact_time =
      (kInitialBallXM - kPaddleHalfThicknessM - kBallRadiusM) /
      closing_speed;
  data->qpos[paddle_qpos] = 0.0;
  data->qvel[paddle_dof] = impact.racket_speed_x;
  data->qpos[ball_qpos + 0] = kInitialBallXM;
  data->qpos[ball_qpos + 1] = 0.0;
  data->qpos[ball_qpos + 2] =
      -impact.incoming_velocity[2] * nominal_contact_time;
  data->qpos[ball_qpos + 3] = 1.0;
  data->qpos[ball_qpos + 4] = 0.0;
  data->qpos[ball_qpos + 5] = 0.0;
  data->qpos[ball_qpos + 6] = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    data->qvel[ball_dof + axis] = impact.incoming_velocity[axis];
    data->qvel[ball_dof + 3 + axis] = impact.incoming_spin[axis];
  }
  mj_forward(model.get(), data.get());

  ImpactResult result;
  result.name = impact.name;
  result.mode = mode == ContactMode::kRaw ? "raw_mujoco" : "explicit_map";
  result.requested_incoming = impact.incoming_velocity;
  result.racket_speed_x = impact.racket_speed_x;

  bool contact_previous = false;
  bool seen_contact = false;
  gate3::Vec3 pre_linear{};
  gate3::Vec3 pre_spin{};
  gate3::PaddleContactResult expected{};

  constexpr int kMaxSteps = 400;
  for (int step = 0; step < kMaxSteps; ++step) {
    // Make the paddle trajectory exactly scripted. The solver can perturb its
    // slide coordinate within a step, but every next step starts from the
    // requested position and velocity.
    data->qpos[paddle_qpos] = impact.racket_speed_x * data->time;
    data->qvel[paddle_dof] = impact.racket_speed_x;
    for (int axis = 0; axis < 3; ++axis) {
      pre_linear[axis] = data->qvel[ball_dof + axis];
      pre_spin[axis] = data->qvel[ball_dof + 3 + axis];
    }

    mj_step(model.get(), data.get());

    double normal_force_n = 0.0;
    gate3::Vec3 contact_position{};
    const bool contact = BallPaddleContact(
        model.get(), data.get(), ball_geom_id, paddle_geom_id,
        &normal_force_n, &contact_position);
    result.peak_raw_normal_force_n =
        std::max(result.peak_raw_normal_force_n, normal_force_n);

    if (contact && !contact_previous) {
      ++result.contact_rising_edges;
      if (!seen_contact) {
        seen_contact = true;
        result.contacted = true;
        result.contact_time_s = data->time;
        result.measured_pre = pre_linear;
        expected = gate3::PredictPaddleContact(
            pre_linear, {impact.racket_speed_x, 0.0, 0.0},
            {1.0, 0.0, 0.0}, pre_spin, kBallRadiusM);
        result.expected_outgoing = expected.linear_velocity;
        result.effective_restitution = expected.effective_restitution;

        if (mode == ContactMode::kExplicit) {
          for (int axis = 0; axis < 3; ++axis) {
            data->qvel[ball_dof + axis] =
                expected.linear_velocity[axis];
            data->qvel[ball_dof + 3 + axis] =
                expected.angular_velocity[axis];
            data->qpos[ball_qpos + axis] =
                contact_position[axis] +
                (kBallRadiusM + 1.0e-4) *
                    expected.oriented_normal[axis];
          }
        }
      }
    }

    if (seen_contact && !contact && contact_previous) {
      result.separated = true;
      for (int axis = 0; axis < 3; ++axis) {
        result.outgoing[axis] = data->qvel[ball_dof + axis];
      }
      break;
    }
    contact_previous = contact;
  }

  if (!result.separated && seen_contact) {
    for (int axis = 0; axis < 3; ++axis) {
      result.outgoing[axis] = data->qvel[ball_dof + axis];
    }
  }

  result.incoming_speed_mps = gate3::Norm(result.measured_pre);
  result.outgoing_speed_mps = gate3::Norm(result.outgoing);
  result.incoming_ke_j = KineticEnergy(result.incoming_speed_mps);
  result.outgoing_ke_j = KineticEnergy(result.outgoing_speed_mps);
  result.ball_ke_ratio =
      result.incoming_ke_j > 0.0
          ? result.outgoing_ke_j / result.incoming_ke_j
          : std::numeric_limits<double>::quiet_NaN();
  result.map_max_abs_error_mps =
      MaxAbsError(result.outgoing, result.expected_outgoing);
  const double pre_relative_normal =
      result.measured_pre[0] - impact.racket_speed_x;
  const double post_relative_normal =
      result.outgoing[0] - impact.racket_speed_x;
  if (pre_relative_normal < -1.0e-9) {
    result.measured_relative_restitution =
        post_relative_normal / -pre_relative_normal;
  }
  result.finite =
      result.contacted && result.separated &&
      FiniteVec(result.measured_pre) && FiniteVec(result.outgoing) &&
      std::isfinite(result.outgoing_speed_mps);
  result.speed_reasonable =
      result.finite && result.outgoing_speed_mps <= kSpeedCeilingMps;
  result.map_match =
      result.finite &&
      result.map_max_abs_error_mps <= kMapToleranceMps;
  return result;
}

void PrintVec(const gate3::Vec3& value) {
  std::cout << '[' << value[0] << ',' << value[1] << ',' << value[2]
            << ']';
}

void PrintResult(const ImpactResult& result) {
  std::cout << "    {\n"
            << "      \"case\": \"" << result.name << "\",\n"
            << "      \"mode\": \"" << result.mode << "\",\n"
            << "      \"requested_incoming_mps\": ";
  PrintVec(result.requested_incoming);
  std::cout << ",\n      \"measured_pre_contact_mps\": ";
  PrintVec(result.measured_pre);
  std::cout << ",\n      \"outgoing_mps\": ";
  PrintVec(result.outgoing);
  std::cout << ",\n      \"map_expected_outgoing_mps\": ";
  PrintVec(result.expected_outgoing);
  std::cout << ",\n"
            << "      \"racket_speed_x_mps\": "
            << result.racket_speed_x << ",\n"
            << "      \"contact_time_s\": "
            << result.contact_time_s << ",\n"
            << "      \"contact_rising_edges\": "
            << result.contact_rising_edges << ",\n"
            << "      \"peak_raw_normal_force_n\": "
            << result.peak_raw_normal_force_n << ",\n"
            << "      \"incoming_speed_mps\": "
            << result.incoming_speed_mps << ",\n"
            << "      \"outgoing_speed_mps\": "
            << result.outgoing_speed_mps << ",\n"
            << "      \"incoming_ball_ke_j\": "
            << result.incoming_ke_j << ",\n"
            << "      \"outgoing_ball_ke_j\": "
            << result.outgoing_ke_j << ",\n"
            << "      \"ball_ke_ratio\": "
            << result.ball_ke_ratio << ",\n"
            << "      \"effective_restitution\": "
            << result.effective_restitution << ",\n"
            << "      \"measured_relative_restitution\": "
            << result.measured_relative_restitution << ",\n"
            << "      \"map_max_abs_error_mps\": "
            << result.map_max_abs_error_mps << ",\n"
            << "      \"contacted\": "
            << (result.contacted ? "true" : "false") << ",\n"
            << "      \"separated\": "
            << (result.separated ? "true" : "false") << ",\n"
            << "      \"finite\": "
            << (result.finite ? "true" : "false") << ",\n"
            << "      \"speed_reasonable\": "
            << (result.speed_reasonable ? "true" : "false") << ",\n"
            << "      \"map_match\": "
            << (result.map_match ? "true" : "false") << "\n"
            << "    }";
}

}  // namespace

int main() {
  try {
    const std::vector<ImpactCase> cases{
        {
            .name = "normal_stationary_racket",
            .incoming_velocity = {-1.5, 0.0, 0.0},
            .incoming_spin = {0.0, 0.0, 0.0},
            .racket_speed_x = 0.0,
        },
        {
            .name = "normal_1p5mps_racket",
            .incoming_velocity = {-1.5, 0.0, 0.0},
            .incoming_spin = {0.0, 0.0, 0.0},
            .racket_speed_x = 1.5,
        },
        {
            .name = "venue_like_oblique",
            .incoming_velocity = {-1.5, 0.0, 3.2},
            .incoming_spin = {0.0, 0.0, 0.0},
            .racket_speed_x = 1.5,
        },
        {
            .name = "launch_like_oblique",
            .incoming_velocity = {-3.0, 0.0, 2.2},
            .incoming_spin = {0.0, 0.0, 0.0},
            .racket_speed_x = 2.0,
        },
    };

    std::vector<ImpactResult> results;
    for (const auto& impact : cases) {
      results.push_back(RunImpact(impact, ContactMode::kRaw));
      results.push_back(RunImpact(impact, ContactMode::kExplicit));
    }

    bool explicit_pass = true;
    double explicit_max_speed_mps = 0.0;
    double explicit_max_map_error_mps = 0.0;
    for (const auto& result : results) {
      if (result.mode != "explicit_map") continue;
      explicit_pass =
          explicit_pass && result.contact_rising_edges == 1 &&
          result.speed_reasonable && result.map_match;
      explicit_max_speed_mps =
          std::max(explicit_max_speed_mps, result.outgoing_speed_mps);
      explicit_max_map_error_mps =
          std::max(explicit_max_map_error_mps,
                   result.map_max_abs_error_mps);
    }

    std::cout << std::fixed << std::setprecision(9)
              << "{\n"
              << "  \"schema\": \"pp-scripted-racket-contact-ab-v1\",\n"
              << "  \"timestep_s\": 0.001000000,\n"
              << "  \"ball_mass_kg\": " << kBallMassKg << ",\n"
              << "  \"ball_radius_m\": " << kBallRadiusM << ",\n"
              << "  \"contact_solref\": [0.002000000,1.000000000],\n"
              << "  \"explicit_speed_ceiling_mps\": "
              << kSpeedCeilingMps << ",\n"
              << "  \"explicit_map_tolerance_mps\": "
              << kMapToleranceMps << ",\n"
              << "  \"results\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
      PrintResult(results[i]);
      std::cout << (i + 1 == results.size() ? "\n" : ",\n");
    }
    std::cout << "  ],\n"
              << "  \"explicit_max_outgoing_speed_mps\": "
              << explicit_max_speed_mps << ",\n"
              << "  \"explicit_max_map_error_mps\": "
              << explicit_max_map_error_mps << ",\n"
              << "  \"explicit_contact_energy_pass\": "
              << (explicit_pass ? "true" : "false") << "\n"
              << "}\n";
    return explicit_pass ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& error) {
    std::cerr << "pp_scripted_racket_contact_ab: " << error.what()
              << '\n';
    return EXIT_FAILURE;
  }
}
