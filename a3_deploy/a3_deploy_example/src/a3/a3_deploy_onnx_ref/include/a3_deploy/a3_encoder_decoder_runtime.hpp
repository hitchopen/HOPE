// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// ORT/CPU runtime for UniversalToken encoder.onnx + decoder.onnx deployment.
#pragma once

#include <cstddef>
#include <memory>
#include <string>

namespace a3_deploy {

struct A3EncoderDecoderRuntimeOptions {
  std::string backend{"ort_cpu"};
  int intra_op_num_threads{1};
  int inter_op_num_threads{1};
};

class A3EncoderDecoderRuntime {
 public:
  virtual ~A3EncoderDecoderRuntime() = default;

  virtual bool Initialize(const std::string& encoder_model_path,
                          const std::string& decoder_model_path,
                          const A3EncoderDecoderRuntimeOptions& options) = 0;
  virtual bool Encode() = 0;
  virtual bool Decode() = 0;

  virtual float* MutableEncoderInputData() = 0;
  virtual float* MutableDecoderInputData() = 0;
  virtual const float* EncodedTokenData() const = 0;
  virtual const float* ActionData() const = 0;

  virtual std::size_t GetEncoderInputDimension() const = 0;
  virtual std::size_t GetTokenDimension() const = 0;
  virtual std::size_t GetDecoderInputDimension() const = 0;
  virtual std::size_t GetActionDimension() const = 0;
  virtual const std::string& BackendName() const = 0;
};

std::unique_ptr<A3EncoderDecoderRuntime> CreateA3EncoderDecoderRuntime(
    const A3EncoderDecoderRuntimeOptions& options);

}  // namespace a3_deploy
