// Copyright (c) 2026, AgiBot Inc. All rights reserved.

#include "a3_deploy/a3_encoder_decoder_runtime.hpp"

#include "a3_deploy/numeric_safety.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
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
  return backend;
}

bool GetTensorElementCount(std::vector<int64_t>& shape,
                           const std::string& tensor_name,
                           std::size_t& count_out) {
  if (shape.empty()) {
    std::cerr << "tensor '" << tensor_name
              << "' has scalar shape; expected a vector/tensor\n";
    return false;
  }
  count_out = 1;
  for (auto& dim : shape) {
    if (dim <= 0) dim = 1;
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

class OrtFloatModel {
 public:
  bool Initialize(Ort::Env& env,
                  Ort::AllocatorWithDefaultOptions& allocator,
                  const std::string& model_path,
                  const std::string& expected_input_name,
                  const std::string& expected_output_name,
                  const A3EncoderDecoderRuntimeOptions& options) {
    if (model_path.empty()) {
      std::cerr << "ORT model init failed: empty model path\n";
      return false;
    }
    try {
      Ort::SessionOptions session_options;
      ConfigureOrtCpuSessionOptions(session_options,
                                    options.intra_op_num_threads,
                                    options.inter_op_num_threads);

      session_ = std::make_unique<Ort::Session>(env, model_path.c_str(),
                                                session_options);
      if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
        std::cerr << "ORT model must have exactly 1 input and 1 output: "
                  << model_path << "\n";
        return false;
      }

      auto input_name = session_->GetInputNameAllocated(0, allocator);
      input_name_ = input_name.get();
      if (input_name_ != expected_input_name) {
        std::cerr << "ORT model input must be '" << expected_input_name
                  << "', got '" << input_name_ << "'\n";
        return false;
      }
      auto output_name = session_->GetOutputNameAllocated(0, allocator);
      output_name_ = output_name.get();
      if (output_name_ != expected_output_name) {
        std::cerr << "ORT model output must be '" << expected_output_name
                  << "', got '" << output_name_ << "'\n";
        return false;
      }

      const auto input_type_info = session_->GetInputTypeInfo(0);
      const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
      if (input_info.GetElementType() !=
          ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        std::cerr << "ORT model input '" << input_name_
                  << "' must be float32\n";
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
        std::cerr << "ORT model output '" << output_name_
                  << "' must be float32\n";
        return false;
      }
      output_shape_ = output_info.GetShape();
      if (!GetTensorElementCount(output_shape_, output_name_, output_dim_)) {
        return false;
      }

      input_buffer_.assign(input_dim_, 0.0f);
      output_buffer_.assign(output_dim_, 0.0f);
      memory_info_ = std::make_unique<Ort::MemoryInfo>(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
      input_tensor_ = Ort::Value::CreateTensor<float>(
          *memory_info_, input_buffer_.data(), input_buffer_.size(),
          input_shape_.data(), input_shape_.size());
      output_tensor_ = Ort::Value::CreateTensor<float>(
          *memory_info_, output_buffer_.data(), output_buffer_.size(),
          output_shape_.data(), output_shape_.size());

      std::cout << "ORT model initialised: " << model_path << "\n"
                << "  Input: " << input_name_ << " dim=" << input_dim_
                << "\n"
                << "  Output: " << output_name_ << " dim=" << output_dim_
                << "\n";
      return true;
    } catch (const Ort::Exception& e) {
      std::cerr << "ORT model init failed: " << e.what() << "\n";
      session_.reset();
      return false;
    } catch (const std::exception& e) {
      std::cerr << "ORT model init failed: " << e.what() << "\n";
      session_.reset();
      return false;
    }
  }

  bool Run() {
    if (!session_) {
      std::cerr << "ORT Run called before Initialize\n";
      return false;
    }
    try {
      const char* input_names[] = {input_name_.c_str()};
      const char* output_names[] = {output_name_.c_str()};
      Ort::RunOptions run_options;
      session_->Run(run_options, input_names, &input_tensor_, 1, output_names,
                    &output_tensor_, 1);
      for (float& value : output_buffer_) {
        if (!numeric_safety::IsFinite(value)) value = 0.0f;
      }
      return true;
    } catch (const Ort::Exception& e) {
      std::cerr << "ORT Run failed: " << e.what() << "\n";
      return false;
    } catch (const std::exception& e) {
      std::cerr << "ORT Run failed: " << e.what() << "\n";
      return false;
    }
  }

  float* MutableInputData() noexcept { return input_buffer_.data(); }
  const float* OutputData() const noexcept { return output_buffer_.data(); }
  std::size_t InputDim() const noexcept { return input_dim_; }
  std::size_t OutputDim() const noexcept { return output_dim_; }

 private:
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
  std::size_t input_dim_{0};
  std::size_t output_dim_{0};
};

class OrtCpuEncoderDecoderRuntime final : public A3EncoderDecoderRuntime {
 public:
  bool Initialize(const std::string& encoder_model_path,
                  const std::string& decoder_model_path,
                  const A3EncoderDecoderRuntimeOptions& options) override {
    if (!encoder_.Initialize(env_, allocator_, encoder_model_path, "obs_dict",
                             "encoded_tokens", options)) {
      return false;
    }
    if (!decoder_.Initialize(env_, allocator_, decoder_model_path, "obs_dict",
                             "action", options)) {
      return false;
    }
    std::cout << "A3 ORT CPU encoder+decoder initialised\n"
              << "  Threads: intra=" << options.intra_op_num_threads
              << " inter=" << options.inter_op_num_threads << "\n";
    return true;
  }

  bool Encode() override { return encoder_.Run(); }
  bool Decode() override { return decoder_.Run(); }

  float* MutableEncoderInputData() override {
    return encoder_.MutableInputData();
  }
  float* MutableDecoderInputData() override {
    return decoder_.MutableInputData();
  }
  const float* EncodedTokenData() const override {
    return encoder_.OutputData();
  }
  const float* ActionData() const override { return decoder_.OutputData(); }

  std::size_t GetEncoderInputDimension() const override {
    return encoder_.InputDim();
  }
  std::size_t GetTokenDimension() const override { return encoder_.OutputDim(); }
  std::size_t GetDecoderInputDimension() const override {
    return decoder_.InputDim();
  }
  std::size_t GetActionDimension() const override {
    return decoder_.OutputDim();
  }
  const std::string& BackendName() const override { return backend_name_; }

 private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "a3_ort_cpu_encoder_decoder"};
  Ort::AllocatorWithDefaultOptions allocator_;
  OrtFloatModel encoder_;
  OrtFloatModel decoder_;
  std::string backend_name_{"ort_cpu"};
};

}  // namespace

std::unique_ptr<A3EncoderDecoderRuntime> CreateA3EncoderDecoderRuntime(
    const A3EncoderDecoderRuntimeOptions& options) {
  const std::string backend = NormalizeBackend(options.backend);
  if (backend == "ort_cpu") {
    return std::make_unique<OrtCpuEncoderDecoderRuntime>();
  }
  std::cerr << "A3 encoder+decoder backend '" << options.backend
            << "' is not supported; use ort_cpu\n";
  return nullptr;
}

}  // namespace a3_deploy
