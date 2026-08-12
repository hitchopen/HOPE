#include "a3_deploy/a3_policy_runtime.hpp"

#include "a3_deploy/numeric_safety.hpp"

#include <onnxruntime_cxx_api.h>

#ifdef ENABLE_A3_TRT_POLICY_RUNTIME
#include "control_policy.hpp"
#endif

#ifdef ENABLE_A3_RKNN_POLICY_RUNTIME
#include <rknn_api.h>
#endif

#include <algorithm>
#include <cstdint>
#include <cctype>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace a3_deploy {
namespace {

std::string NormalizeBackend(std::string backend) {
  std::transform(backend.begin(), backend.end(), backend.begin(),
                 [](unsigned char c) {
                   if (c == '-') return static_cast<char>('_');
                   return static_cast<char>(std::tolower(c));
                 });
  if (backend.empty() || backend == "cpu" || backend == "ort" ||
      backend == "onnxruntime" || backend == "onnxruntime_cpu") {
    return "ort_cpu";
  }
  if (backend == "tensorrt") return "trt";
  if (backend == "rk_npu" || backend == "rockchip_npu") return "rknn";
  return backend;
}

bool GetTensorElementCount(std::vector<int64_t>& shape,
                           const std::string& tensor_name,
                           std::size_t& count_out) {
  if (shape.empty()) {
    std::cerr << "policy tensor '" << tensor_name
              << "' has scalar shape; expected a vector/tensor\n";
    return false;
  }

  count_out = 1;
  for (auto& dim : shape) {
    if (dim <= 0) {
      // The exported policy normally has a dynamic batch dim (-1). Runtime
      // deployment always runs batch=1.
      dim = 1;
    }
    count_out *= static_cast<std::size_t>(dim);
  }
  return true;
}

void ConfigureOrtCpuSessionOptions(Ort::SessionOptions& session_options,
                                   int intra_op_num_threads,
                                   int inter_op_num_threads) {
  if (intra_op_num_threads > 0) {
    session_options.SetIntraOpNumThreads(intra_op_num_threads);
  }
  if (inter_op_num_threads > 0) {
    session_options.SetInterOpNumThreads(inter_op_num_threads);
  }
  session_options.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_options.AddConfigEntry("session.intra_op.allow_spinning", "0");
  session_options.AddConfigEntry("session.inter_op.allow_spinning", "0");
}

class OrtCpuPolicyRuntime final : public A3PolicyRuntime {
 public:
  bool Initialize(const std::string& model_path,
                  const A3PolicyRuntimeOptions& options) override {
    if (model_path.empty()) {
      std::cerr << "A3 ORT policy init failed: empty model path\n";
      return false;
    }

    try {
      Ort::SessionOptions session_options;
      ConfigureOrtCpuSessionOptions(session_options,
                                    options.intra_op_num_threads,
                                    options.inter_op_num_threads);

      session_ =
          std::make_unique<Ort::Session>(env_, model_path.c_str(),
                                         session_options);

      if (session_->GetInputCount() != 1) {
        std::cerr << "A3 policy must have exactly 1 input, got "
                  << session_->GetInputCount() << "\n";
        return false;
      }
      if (session_->GetOutputCount() != 1) {
        std::cerr << "A3 policy must have exactly 1 output, got "
                  << session_->GetOutputCount() << "\n";
        return false;
      }

      auto input_name =
          session_->GetInputNameAllocated(0, allocator_);
      input_name_ = input_name.get();
      if (input_name_ != "obs_dict") {
        std::cerr << "A3 policy input tensor must be 'obs_dict', got '"
                  << input_name_ << "'\n";
        return false;
      }
      auto output_name =
          session_->GetOutputNameAllocated(0, allocator_);
      output_name_ = output_name.get();
      if (output_name_ != "action") {
        std::cerr << "A3 policy output tensor must be 'action', got '"
                  << output_name_ << "'\n";
        return false;
      }

      const auto input_type_info = session_->GetInputTypeInfo(0);
      const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
      if (input_info.GetElementType() !=
          ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        std::cerr << "A3 policy input 'obs_dict' must be float32\n";
        return false;
      }
      input_shape_ = input_info.GetShape();
      if (!GetTensorElementCount(input_shape_, input_name_, input_dim_)) {
        return false;
      }

      const auto output_type_info = session_->GetOutputTypeInfo(0);
      const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
      if (output_info.GetElementType() !=
          ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        std::cerr << "A3 policy output 'action' must be float32\n";
        return false;
      }
      output_shape_ = output_info.GetShape();
      if (!GetTensorElementCount(output_shape_, output_name_, action_dim_)) {
        return false;
      }

      input_buffer_.assign(input_dim_, 0.0f);
      action_buffer_.assign(action_dim_, 0.0f);
      memory_info_ = std::make_unique<Ort::MemoryInfo>(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
      input_tensor_ = Ort::Value::CreateTensor<float>(
          *memory_info_, input_buffer_.data(), input_buffer_.size(),
          input_shape_.data(), input_shape_.size());
      output_tensor_ = Ort::Value::CreateTensor<float>(
          *memory_info_, action_buffer_.data(), action_buffer_.size(),
          output_shape_.data(), output_shape_.size());

      std::cout << "A3 ORT CPU policy initialised\n"
                << "  Model: " << model_path << "\n"
                << "  Input: " << input_name_ << " dim=" << input_dim_
                << "\n"
                << "  Output: " << output_name_ << " dim=" << action_dim_
                << "\n"
                << "  Threads: intra=" << options.intra_op_num_threads
                << " inter=" << options.inter_op_num_threads << "\n";
      if (options.use_fp16) {
        std::cout << "  Note: onnx.fp16 is ignored by ORT CPU backend\n";
      }
      return true;
    } catch (const Ort::Exception& e) {
      std::cerr << "A3 ORT policy init failed: " << e.what() << "\n";
      session_.reset();
      return false;
    } catch (const std::exception& e) {
      std::cerr << "A3 ORT policy init failed: " << e.what() << "\n";
      session_.reset();
      return false;
    }
  }

  bool Infer() override {
    if (!session_) {
      std::cerr << "A3 ORT policy Infer called before Initialize\n";
      return false;
    }

    try {
      const char* input_names[] = {input_name_.c_str()};
      const char* output_names[] = {output_name_.c_str()};
      Ort::RunOptions run_options;
      session_->Run(run_options, input_names, &input_tensor_, 1, output_names,
                    &output_tensor_, 1);

      for (float& value : action_buffer_) {
        if (!numeric_safety::IsFinite(value)) value = 0.0f;
      }
      return true;
    } catch (const Ort::Exception& e) {
      std::cerr << "A3 ORT policy Infer failed: " << e.what() << "\n";
      return false;
    } catch (const std::exception& e) {
      std::cerr << "A3 ORT policy Infer failed: " << e.what() << "\n";
      return false;
    }
  }

  float* MutableInputData() override { return input_buffer_.data(); }
  const float* ActionData() const override { return action_buffer_.data(); }
  std::size_t GetInputDimension() const override { return input_dim_; }
  std::size_t GetActionDimension() const override { return action_dim_; }
  const std::string& BackendName() const override { return backend_name_; }

 private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "a3_ort_cpu_policy"};
  Ort::AllocatorWithDefaultOptions allocator_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
  std::vector<float> input_buffer_;
  std::vector<float> action_buffer_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
  std::size_t input_dim_{0};
  std::size_t action_dim_{0};
  std::string backend_name_{"ort_cpu"};
};

#ifdef ENABLE_A3_RKNN_POLICY_RUNTIME
std::vector<std::uint8_t> ReadBinaryFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("failed to open model file: " + path);
  }
  const std::streamsize size = file.tellg();
  if (size <= 0) {
    throw std::runtime_error("empty model file: " + path);
  }
  if (static_cast<unsigned long long>(size) >
      static_cast<unsigned long long>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::runtime_error("RKNN model is larger than uint32_t API limit: " +
                             path);
  }
  std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
  file.seekg(0, std::ios::beg);
  if (!file.read(reinterpret_cast<char*>(data.data()), size)) {
    throw std::runtime_error("failed to read model file: " + path);
  }
  return data;
}

std::string FormatRknnDims(const rknn_tensor_attr& attr) {
  std::string out = "[";
  for (uint32_t i = 0; i < attr.n_dims; ++i) {
    if (i > 0) out += ", ";
    out += std::to_string(attr.dims[i]);
  }
  out += "]";
  return out;
}

bool ParseRknnCoreMask(std::string value, rknn_core_mask& mask_out) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   if (c == '-' || c == ',' || c == '+' || c == '|') {
                     return static_cast<char>('_');
                   }
                   return static_cast<char>(std::tolower(c));
                 });
  if (value.empty() || value == "auto") {
    mask_out = RKNN_NPU_CORE_AUTO;
    return true;
  }
  if (value == "all") {
    mask_out = RKNN_NPU_CORE_ALL;
    return true;
  }
  if (value == "0" || value == "core0") {
    mask_out = RKNN_NPU_CORE_0;
    return true;
  }
  if (value == "1" || value == "core1") {
    mask_out = RKNN_NPU_CORE_1;
    return true;
  }
  if (value == "2" || value == "core2") {
    mask_out = RKNN_NPU_CORE_2;
    return true;
  }
  if (value == "0_1" || value == "01" || value == "core0_1") {
    mask_out = RKNN_NPU_CORE_0_1;
    return true;
  }
  if (value == "0_1_2" || value == "012" || value == "core0_1_2") {
    mask_out = RKNN_NPU_CORE_0_1_2;
    return true;
  }
  return false;
}

const char* RknnCoreMaskName(rknn_core_mask mask) {
  switch (mask) {
    case RKNN_NPU_CORE_AUTO:
      return "auto";
    case RKNN_NPU_CORE_0:
      return "0";
    case RKNN_NPU_CORE_1:
      return "1";
    case RKNN_NPU_CORE_2:
      return "2";
    case RKNN_NPU_CORE_0_1:
      return "0_1";
    case RKNN_NPU_CORE_0_1_2:
      return "0_1_2";
    case RKNN_NPU_CORE_ALL:
      return "all";
    default:
      return "unknown";
  }
}

rknn_tensor_format HostInputFormatFor(const rknn_tensor_attr& attr) {
  if (attr.fmt == RKNN_TENSOR_UNDEFINED) {
    return RKNN_TENSOR_NCHW;
  }
  return attr.fmt;
}

class RknnPolicyRuntime final : public A3PolicyRuntime {
 public:
  ~RknnPolicyRuntime() override {
    if (ctx_ != 0) {
      rknn_destroy(ctx_);
      ctx_ = 0;
    }
  }

  bool Initialize(const std::string& model_path,
                  const A3PolicyRuntimeOptions& options) override {
    if (model_path.empty()) {
      std::cerr << "A3 RKNN policy init failed: empty model path\n";
      return false;
    }

    try {
      auto model_data = ReadBinaryFile(model_path);
      const int init_ret =
          rknn_init(&ctx_, model_data.data(),
                    static_cast<std::uint32_t>(model_data.size()), 0, nullptr);
      if (init_ret != RKNN_SUCC) {
        std::cerr << "A3 RKNN policy init failed: rknn_init ret="
                  << init_ret << " model=" << model_path << "\n";
        ctx_ = 0;
        return false;
      }

      rknn_core_mask core_mask = RKNN_NPU_CORE_AUTO;
      if (!ParseRknnCoreMask(options.rknn_core_mask, core_mask)) {
        std::cerr << "A3 RKNN policy init failed: invalid rknn_core_mask='"
                  << options.rknn_core_mask
                  << "'; expected auto/all/0/1/2/0_1/0_1_2\n";
        return false;
      }
      if (core_mask != RKNN_NPU_CORE_AUTO) {
        const int core_ret = rknn_set_core_mask(ctx_, core_mask);
        if (core_ret != RKNN_SUCC) {
          std::cerr << "A3 RKNN policy init failed: rknn_set_core_mask("
                    << RknnCoreMaskName(core_mask) << ") ret="
                    << core_ret << "\n";
          return false;
        }
      }

      rknn_input_output_num io_num{};
      int ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num,
                           sizeof(io_num));
      if (ret != RKNN_SUCC) {
        std::cerr << "A3 RKNN policy init failed: query io num ret="
                  << ret << "\n";
        return false;
      }
      if (io_num.n_input != 1) {
        std::cerr << "A3 policy must have exactly 1 input, got "
                  << io_num.n_input << "\n";
        return false;
      }
      if (io_num.n_output != 1) {
        std::cerr << "A3 policy must have exactly 1 output, got "
                  << io_num.n_output << "\n";
        return false;
      }

      std::memset(&input_attr_, 0, sizeof(input_attr_));
      input_attr_.index = 0;
      ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr_,
                       sizeof(input_attr_));
      if (ret != RKNN_SUCC) {
        std::cerr << "A3 RKNN policy init failed: query input attr ret="
                  << ret << "\n";
        return false;
      }
      std::memset(&output_attr_, 0, sizeof(output_attr_));
      output_attr_.index = 0;
      ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attr_,
                       sizeof(output_attr_));
      if (ret != RKNN_SUCC) {
        std::cerr << "A3 RKNN policy init failed: query output attr ret="
                  << ret << "\n";
        return false;
      }

      input_name_ = input_attr_.name;
      output_name_ = output_attr_.name;
      if (input_name_ != "obs_dict") {
        std::cerr << "A3 RKNN policy input tensor name is '" << input_name_
                  << "'; expected exported ONNX name 'obs_dict'\n";
      }
      if (output_name_ != "action") {
        std::cerr << "A3 RKNN policy output tensor name is '" << output_name_
                  << "'; expected exported ONNX name 'action'\n";
      }
      input_dim_ = input_attr_.n_elems;
      action_dim_ = output_attr_.n_elems;
      if (input_dim_ == 0 || action_dim_ == 0) {
        std::cerr << "A3 RKNN policy init failed: invalid tensor element "
                     "counts input="
                  << input_dim_ << " output=" << action_dim_ << "\n";
        return false;
      }

      input_buffer_.assign(input_dim_, 0.0f);
      action_buffer_.assign(action_dim_, 0.0f);
      input_format_ = HostInputFormatFor(input_attr_);

      rknn_sdk_version version{};
      if (rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &version,
                     sizeof(version)) == RKNN_SUCC) {
        api_version_ = version.api_version;
        driver_version_ = version.drv_version;
      }

      std::cout << "A3 RKNN policy initialised\n"
                << "  Model: " << model_path << "\n"
                << "  Input: " << input_name_ << " dims="
                << FormatRknnDims(input_attr_) << " elems=" << input_dim_
                << " type=" << get_type_string(input_attr_.type)
                << " fmt=" << get_format_string(input_attr_.fmt) << "\n"
                << "  Output: " << output_name_ << " dims="
                << FormatRknnDims(output_attr_) << " elems=" << action_dim_
                << " type=" << get_type_string(output_attr_.type)
                << " fmt=" << get_format_string(output_attr_.fmt) << "\n"
                << "  Core mask: " << RknnCoreMaskName(core_mask) << "\n";
      if (!api_version_.empty() || !driver_version_.empty()) {
        std::cout << "  RKNN API: " << api_version_
                  << " driver: " << driver_version_ << "\n";
      }
      if (options.use_fp16) {
        std::cout << "  Note: onnx.fp16 is ignored by RKNN runtime; use the "
                     "converted .rknn precision instead\n";
      }
      return true;
    } catch (const std::exception& e) {
      std::cerr << "A3 RKNN policy init failed: " << e.what() << "\n";
      return false;
    }
  }

  bool Infer() override {
    if (ctx_ == 0) {
      std::cerr << "A3 RKNN policy Infer called before Initialize\n";
      return false;
    }

    rknn_input input{};
    input.index = 0;
    input.buf = input_buffer_.data();
    input.size =
        static_cast<std::uint32_t>(input_buffer_.size() * sizeof(float));
    input.pass_through = 0;
    input.type = RKNN_TENSOR_FLOAT32;
    input.fmt = input_format_;

    int ret = rknn_inputs_set(ctx_, 1, &input);
    if (ret != RKNN_SUCC) {
      std::cerr << "A3 RKNN policy Infer failed: rknn_inputs_set ret="
                << ret << "\n";
      return false;
    }
    ret = rknn_run(ctx_, nullptr);
    if (ret != RKNN_SUCC) {
      std::cerr << "A3 RKNN policy Infer failed: rknn_run ret=" << ret
                << "\n";
      return false;
    }

    rknn_output output{};
    output.want_float = 1;
    output.is_prealloc = 1;
    output.index = 0;
    output.buf = action_buffer_.data();
    output.size =
        static_cast<std::uint32_t>(action_buffer_.size() * sizeof(float));
    ret = rknn_outputs_get(ctx_, 1, &output, nullptr);
    if (ret != RKNN_SUCC) {
      std::cerr << "A3 RKNN policy Infer failed: rknn_outputs_get ret="
                << ret << "\n";
      return false;
    }
    const int release_ret = rknn_outputs_release(ctx_, 1, &output);
    if (release_ret != RKNN_SUCC) {
      std::cerr << "A3 RKNN policy Infer failed: rknn_outputs_release ret="
                << release_ret << "\n";
      return false;
    }

    for (float& value : action_buffer_) {
      if (!numeric_safety::IsFinite(value)) value = 0.0f;
    }
    return true;
  }

  float* MutableInputData() override { return input_buffer_.data(); }
  const float* ActionData() const override { return action_buffer_.data(); }
  std::size_t GetInputDimension() const override { return input_dim_; }
  std::size_t GetActionDimension() const override { return action_dim_; }
  const std::string& BackendName() const override { return backend_name_; }

 private:
  rknn_context ctx_{0};
  rknn_tensor_attr input_attr_{};
  rknn_tensor_attr output_attr_{};
  rknn_tensor_format input_format_{RKNN_TENSOR_NCHW};
  std::string input_name_;
  std::string output_name_;
  std::string api_version_;
  std::string driver_version_;
  std::vector<float> input_buffer_;
  std::vector<float> action_buffer_;
  std::size_t input_dim_{0};
  std::size_t action_dim_{0};
  std::string backend_name_{"rknn"};
};
#endif

#ifdef ENABLE_A3_TRT_POLICY_RUNTIME
class TrtPolicyRuntime final : public A3PolicyRuntime {
 public:
  bool Initialize(const std::string& model_path,
                  const A3PolicyRuntimeOptions& options) override {
    return engine_.Initialize(model_path, options.use_fp16);
  }

  bool Infer() override { return engine_.Infer(); }
  bool CaptureGraph() override { return engine_.CaptureGraph(); }
  float* MutableInputData() override { return engine_.GetInputBuffer().data(); }
  const float* ActionData() const override {
    return engine_.GetActionBuffer().data();
  }
  std::size_t GetInputDimension() const override {
    return engine_.GetInputDimension();
  }
  std::size_t GetActionDimension() const override {
    return engine_.GetActionDimension();
  }
  const std::string& BackendName() const override { return backend_name_; }

 private:
  mutable PolicyEngine engine_;
  std::string backend_name_{"trt"};
};
#endif

}  // namespace

std::unique_ptr<A3PolicyRuntime> CreateA3PolicyRuntime(
    const A3PolicyRuntimeOptions& options) {
  const std::string backend = NormalizeBackend(options.backend);
  if (backend == "ort_cpu") {
    return std::make_unique<OrtCpuPolicyRuntime>();
  }
  if (backend == "trt") {
#ifdef ENABLE_A3_TRT_POLICY_RUNTIME
    return std::make_unique<TrtPolicyRuntime>();
#else
    std::cerr << "A3 TensorRT backend requested, but this build was "
                 "configured without ENABLE_TRT_INFERENCE=ON\n";
    return nullptr;
#endif
  }
  if (backend == "rknn") {
#ifdef ENABLE_A3_RKNN_POLICY_RUNTIME
    return std::make_unique<RknnPolicyRuntime>();
#else
    std::cerr << "A3 RKNN backend requested, but this build was "
                 "configured without ENABLE_RKNN_INFERENCE=ON\n";
    return nullptr;
#endif
  }

  std::cerr << "unknown A3 policy backend '" << options.backend
            << "'; expected ort_cpu, trt, or rknn\n";
  return nullptr;
}

}  // namespace a3_deploy
