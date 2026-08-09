#pragma once

#include <cstddef>
#include <memory>
#include <string>

namespace a3_deploy {

struct A3PolicyRuntimeOptions {
  std::string backend{"ort_cpu"};
  bool use_fp16{false};
  int intra_op_num_threads{1};
  int inter_op_num_threads{1};
  std::string rknn_core_mask{"auto"};
};

class A3PolicyRuntime {
 public:
  virtual ~A3PolicyRuntime() = default;

  virtual bool Initialize(const std::string& model_path,
                          const A3PolicyRuntimeOptions& options) = 0;
  virtual bool Infer() = 0;
  virtual bool CaptureGraph() { return true; }

  virtual float* MutableInputData() = 0;
  virtual const float* ActionData() const = 0;

  virtual std::size_t GetInputDimension() const = 0;
  virtual std::size_t GetActionDimension() const = 0;
  virtual const std::string& BackendName() const = 0;
};

std::unique_ptr<A3PolicyRuntime> CreateA3PolicyRuntime(
    const A3PolicyRuntimeOptions& options);

}  // namespace a3_deploy
