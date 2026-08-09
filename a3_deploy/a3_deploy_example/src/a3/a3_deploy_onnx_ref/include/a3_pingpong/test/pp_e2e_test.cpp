// C++ front-end e2e: load real policy.onnx, fetch refs (side-outputs), build the
// 180-D obs from a fixed deterministic state/target, run the action, decode
// target_q. Dumps everything so a numpy-only checker can re-derive obs (from the
// dumped refs) and decode (from the dumped action) and confirm parity -- no
// python-onnxruntime needed. Also self-checks finiteness/dims.
//   ./pp_e2e_test <policy.onnx> <dump_out.txt>
#include <cmath>
#include <cstdio>
#include <fstream>

#include "a3_pingpong/pp_obs_builder.hpp"
#include "a3_pingpong/pp_onnx_policy.hpp"

using namespace a3_pingpong;

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: %s policy.onnx dump_out.txt\n", argv[0]); return 2; }
  PpOnnxPolicy pol(argv[1]);
  const int ts = 20;

  // fixed deterministic state/target (reproducible; checker reads them back)
  PpRobotState st;
  st.base_pos_w = Vec3(0.0, 0.0, 0.95);
  st.base_quat_w = Vec4(0.9975, 0.0, 0.0706, 0.0);  // ~8 deg pitch, normalized-ish
  st.base_quat_w /= st.base_quat_w.norm();
  st.torso_pos_w = Vec3(0.02, 0.0, 1.20);
  st.torso_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
  st.base_ang_vel_b = Vec3(0.01, -0.02, 0.03);
  st.q = Eigen::VectorXd(31);
  st.qd = Eigen::VectorXd(31);
  for (int i = 0; i < 31; ++i) { st.q[i] = 0.01 * i - 0.1; st.qd[i] = 0.005 * i; }

  PpRacketTarget tg;
  tg.pos_w = Vec3(0.40, -0.22, 0.82);
  tg.vel_w = Vec3(1.0, 0.0, 0.0);
  tg.swing_sign = 1.0;
  tg.time_to_strike = 0.37;
  tg.base_target_xy = Vec2(0.0, 0.0);

  Eigen::VectorXd last_action = Eigen::VectorXd::Zero(31);

  PpRefs refs = pol.refs(ts);
  Eigen::VectorXd obs = build_obs_180(refs, st, tg, last_action, pol.default_q());
  Eigen::VectorXd action = pol.mean_action(obs, ts);
  Eigen::VectorXd tq = pol.target_q(action);

  // self-checks
  bool finite = obs.allFinite() && action.allFinite() && tq.allFinite();
  std::printf("[self] obs dim=%ld action dim=%ld finite=%d\n", (long)obs.size(),
              (long)action.size(), finite ? 1 : 0);
  std::printf("[self] action[:4]=%.5f %.5f %.5f %.5f  |action|=%.4f  max|tq|=%.4f\n",
              action[0], action[1], action[2], action[3], action.norm(),
              tq.cwiseAbs().maxCoeff());

  std::ofstream f(argv[2]);
  f.precision(17);
  auto put = [&](const Eigen::VectorXd& v) { for (int i = 0; i < v.size(); ++i) f << v[i] << " "; };
  auto put3 = [&](const Vec3& v) { f << v[0] << " " << v[1] << " " << v[2] << " "; };
  auto put4 = [&](const Vec4& v) { f << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " "; };
  f << ts << " ";
  put(refs.joint_pos); put(refs.joint_vel); put3(refs.anchor_pos_w); put4(refs.anchor_quat_w);
  put3(st.base_pos_w); put4(st.base_quat_w); put3(st.torso_pos_w); put4(st.torso_quat_w);
  put3(st.base_ang_vel_b); put(st.q); put(st.qd);
  put3(tg.pos_w); put3(tg.vel_w); f << tg.swing_sign << " " << tg.time_to_strike << " ";
  f << tg.base_target_xy[0] << " " << tg.base_target_xy[1] << " ";
  put(last_action); put(pol.default_q()); put(pol.action_scale());
  put(obs); put(action); put(tq);
  f.close();
  std::printf("[dump] wrote %s\n", argv[2]);
  return finite ? 0 : 1;
}
