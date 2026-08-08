// Verify the Isaac(ONNX joint_names) -> backend(MuJoCo slot) permutation built
// from the REAL policy.onnx metadata is a valid bijection, and that scattering
// default_q lands sensible values in the right slots.
//   ./pp_jointmap_test <policy.onnx>
#include <cstdio>

#include "a3_pingpong/pp_joint_map.hpp"
#include "a3_pingpong/pp_onnx_policy.hpp"

using namespace a3_pingpong;

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: %s policy.onnx\n", argv[0]); return 2; }
  PpOnnxPolicy pol(argv[1]);
  const auto& src = pol.joint_names();  // Isaac order

  std::array<int, 31> map{};
  bool ok = build_src_to_sdk(src, map);
  std::printf("[map] bijection valid = %d\n", ok ? 1 : 0);
  if (!ok) { std::printf("JOINTMAP FAIL\n"); return 1; }

  // scatter default_q (Isaac) -> backend slots, print to eyeball correctness
  Eigen::VectorXd dq_sdk = to_sdk_order(pol.default_q(), map);
  const auto& B = backend_joint_order();
  std::printf("[map] slot  name                         default_q\n");
  for (int s = 0; s < 31; ++s)
    std::printf("       %2d  %-28s % .3f\n", s, B[s].c_str(), dq_sdk[s]);

  // sanity: waist (slots 0..2) and neck (3..4) defaults should be ~0; knees > 0
  bool sane = std::abs(dq_sdk[0]) < 1e-6 && std::abs(dq_sdk[3]) < 1e-6 &&
              dq_sdk[22] > 0.1 && dq_sdk[28] > 0.1;  // left/right knee bent
  std::printf("[map] sanity (waist/neck~0, knees>0.1) = %d\n", sane ? 1 : 0);
  std::printf("%s\n", sane ? "JOINTMAP PASS" : "JOINTMAP FAIL");
  return sane ? 0 : 1;
}
