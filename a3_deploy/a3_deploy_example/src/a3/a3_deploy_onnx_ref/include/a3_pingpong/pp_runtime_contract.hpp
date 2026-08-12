#pragma once

#include <stdexcept>
#include <string>

namespace a3_pingpong {

enum class HitterPureRuntimeContract {
  kLegacy,
  kRallyFinalV1,
  kRallyFinalV2,
  kRallyV15,
  kRallyV17FixedStationBallClockV1,
};

// Parse the capability contract and enforce the bidirectional component-velocity recipe pairing
// before any recipe-specific metadata is consumed. RallyV10--V14 and RallyV17 share the v2
// wire and core-or-planner velocity gate; their recipe-specific fields are validated by
// PpOnnxPolicy.
inline HitterPureRuntimeContract validate_hitter_pure_runtime_contract(
    const std::string& runtime_contract, const std::string& training_recipe) {
  HitterPureRuntimeContract contract = HitterPureRuntimeContract::kLegacy;
  if (runtime_contract.empty()) {
    contract = HitterPureRuntimeContract::kLegacy;
  } else if (runtime_contract == "rally_final_v1") {
    contract = HitterPureRuntimeContract::kRallyFinalV1;
  } else if (runtime_contract == "rally_final_v2") {
    contract = HitterPureRuntimeContract::kRallyFinalV2;
  } else if (runtime_contract == "rally_v15") {
    contract = HitterPureRuntimeContract::kRallyV15;
  } else if (runtime_contract == "rally_v17_fixed_station_ball_clock_v1") {
    contract = HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1;
  } else {
    throw std::runtime_error(
        "ONNX has unsupported hitter_pure_runtime_contract='" + runtime_contract + "'");
  }

  const bool runtime_v2 = contract == HitterPureRuntimeContract::kRallyFinalV2;
  const bool legacy_component_recipe =
      training_recipe == "rally_v10" || training_recipe == "rally_v11" ||
      training_recipe == "rally_v12" || training_recipe == "rally_v13" ||
      training_recipe == "rally_v14";
  const bool v17_recipe = training_recipe == "rally_v17";
  const bool runtime_v17_fixed =
      contract == HitterPureRuntimeContract::kRallyV17FixedStationBallClockV1;
  if ((runtime_v2 && !(legacy_component_recipe || v17_recipe)) ||
      (legacy_component_recipe && !runtime_v2) ||
      (v17_recipe && !(runtime_v2 || runtime_v17_fixed)) ||
      (runtime_v17_fixed && !v17_recipe)) {
    throw std::runtime_error(
        "ONNX runtime/recipe pairing is invalid: RallyV10--V14 require "
        "rally_final_v2; legacy RallyV17 accepts rally_final_v2; fixed-station "
        "RallyV17 requires rally_v17_fixed_station_ball_clock_v1");
  }
  const bool runtime_v15 = contract == HitterPureRuntimeContract::kRallyV15;
  if (runtime_v15 != (training_recipe == "rally_v15")) {
    throw std::runtime_error(
        "ONNX runtime_contract=rally_v15 and training_recipe=rally_v15 must be declared "
        "together; the V15 executed-q_des/finite-gait contract cannot fall back to V14 wiring");
  }
  return contract;
}

}  // namespace a3_pingpong
