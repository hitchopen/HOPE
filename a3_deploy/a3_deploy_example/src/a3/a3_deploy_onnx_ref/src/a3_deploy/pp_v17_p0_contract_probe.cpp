#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "a3_pingpong/pp_onnx_policy.hpp"
#include "a3_pingpong/pp_sha256.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " /abs/path/policy.onnx\n";
    return 64;
  }
  try {
    bool production_rejected = false;
    try {
      a3_pingpong::PpOnnxPolicy production_policy(
          argv[1], a3_pingpong::PpOnnxLoadProfile::kProductionStrict);
    } catch (const std::exception& error) {
      production_rejected =
          std::string(error.what()).find("P0 contract-only") !=
          std::string::npos;
    }
    if (!production_rejected) {
      throw std::runtime_error(
          "production loader did not reject this P0 artifact at its "
          "hardware-authorization boundary");
    }

    a3_pingpong::PpOnnxPolicy policy(
        argv[1],
        a3_pingpong::PpOnnxLoadProfile::kV17R6P0ContractAudit);
    if (!policy.is_v17_r6_p0_contract_audit()) {
      std::cerr << "V17-r6 P0 audit profile was not bound\n";
      return 2;
    }

    std::filesystem::path parity_path(argv[1]);
    parity_path.replace_extension(".qdes_parity.csv");
    const std::string parity_sha =
        a3_pingpong::PpSha256::File(parity_path.string());
    if (parity_sha != policy.qdes_parity_csv_sha256()) {
      throw std::runtime_error(
          "q_des parity sidecar SHA256 does not match ONNX metadata");
    }
    std::ifstream parity_stream(parity_path);
    if (!parity_stream) {
      throw std::runtime_error(
          "failed to open q_des parity sidecar: " + parity_path.string());
    }
    std::size_t parity_rows = 0;
    double max_abs_qdes_error = 0.0;
    for (std::string line; std::getline(parity_stream, line);) {
      if (line.empty()) continue;
      std::vector<double> values;
      std::stringstream row(line);
      for (std::string field; std::getline(row, field, ',');) {
        values.push_back(std::stod(field));
      }
      if (values.size() != 62) {
        throw std::runtime_error(
            "q_des parity row must contain 31 actions and 31 expected targets");
      }
      Eigen::VectorXd action(31);
      for (int index = 0; index < 31; ++index) {
        action[index] = values[static_cast<std::size_t>(index)];
      }
      const Eigen::VectorXd actual_qdes = policy.target_q(action);
      if (actual_qdes.size() != 31) {
        throw std::runtime_error("C++ q_des decoder did not return 31 joints");
      }
      for (int index = 0; index < 31; ++index) {
        const double expected =
            values[31 + static_cast<std::size_t>(index)];
        max_abs_qdes_error =
            std::max(max_abs_qdes_error,
                     std::abs(actual_qdes[index] - expected));
      }
      ++parity_rows;
    }
    if (parity_rows == 0 || max_abs_qdes_error > 1.0e-6) {
      throw std::runtime_error(
          "Python/C++ final q_des parity failed: rows=" +
          std::to_string(parity_rows) +
          " max_abs_error=" + std::to_string(max_abs_qdes_error));
    }
    std::cout
        << "V17-r6 P0 CONTRACT PASS\n"
        << "  model=" << argv[1] << "\n"
        << "  status=" << policy.deploy_manifest_status() << "\n"
        << "  manifest_sha256=" << policy.deploy_manifest_sha256() << "\n"
        << "  contract_fingerprint_sha256="
        << policy.deploy_contract_fingerprint_sha256() << "\n"
        << "  qdes_parity_rows=" << parity_rows << "\n"
        << "  qdes_max_abs_error_rad=" << max_abs_qdes_error << "\n"
        << "  production_loader=p0_rejected\n"
        << "  qualification_status=not_qualified\n"
        << "  hardware_authorized=false\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "V17-r6 P0 CONTRACT FAIL: " << error.what() << "\n";
    return 2;
  }
}
