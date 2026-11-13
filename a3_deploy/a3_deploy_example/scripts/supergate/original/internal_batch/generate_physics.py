from pathlib import Path
W=Path(__file__).resolve().parent
R=Path('/home/user/Desktop/workspace/nohope/agi/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/module/mujoco_sim_module')
s=(R/'mujoco_sim_module.cc').read_text();methods=s[s.index('void MujocoSimModule::ApplyGate3BallDrag'):s.index('void MujocoSimModule::InitializeDebugCsv')].replace('MujocoSimModule::','BallPhysicsRuntime::')
head='''#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include "mujoco/mujoco.h"
#include "mujoco_sim_module/common/gate3_ball_layout.h"
#include "mujoco_sim_module/common/gate3_ball_contact_model.h"
namespace gate3 = aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball;
struct BallPhysicsRuntime {
 const mjModel* m_; mjData* d_;
 int gate3_racket_geom_id_, gate3_table_geom_id_, gate3_net_geom_id_, gate3_racket_site_id_;
 double gate3_ball_drag_k_=.12453915600992894,gate3_ball_magnus_k_=.00399279130564425;
 double gate3_ball_restitution_v_=.9373319241133231,gate3_ball_tangential_gain_=.41642144985036184,gate3_ball_friction_cap_mu_=.218520906499864;
 std::array<std::array<double,6>,gate3::kFlightSlots> flight_velocities_{};
 std::array<std::uint64_t,gate3::kFlightSlots> flight_shot_ids_{};
 std::array<bool,gate3::kFlightSlots> flight_racket_prev_{},flight_table_prev_{},flight_net_prev_{};
 std::array<gate3::TableRestTracker,gate3::kFlightSlots> flight_rest_{};
 BallPhysicsRuntime(const mjModel* m,mjData* d):m_(m),d_(d) {
  gate3_racket_geom_id_=mj_name2id(m,mjOBJ_GEOM,gate3::kRacketGeomName);
  gate3_table_geom_id_=mj_name2id(m,mjOBJ_GEOM,gate3::kTableGeomName);
  gate3_net_geom_id_=mj_name2id(m,mjOBJ_GEOM,gate3::kNetGeomName);
  gate3_racket_site_id_=mj_name2id(m,mjOBJ_SITE,"right_racket");
 }
 void ApplyGate3BallDrag(int slot);
 void UpdateGate3BallContacts(int slot);
};
'''
(W/'physics.hpp').write_text(head+methods)
p=Path('/home/user/Documents/Codex/2026-11-04/new-chat-6/work/include/a3_pingpong/pp_planner_input.hpp');s=p.read_text().replace('inline double simulation_time_s=0.;','inline thread_local double simulation_time_s=0.;');(W/'include/a3_pingpong/pp_planner_input.hpp').write_text(s)
print('Copied production pre-step/post-step ball physics without numerical edits; made offline simulation clock thread-local.')
