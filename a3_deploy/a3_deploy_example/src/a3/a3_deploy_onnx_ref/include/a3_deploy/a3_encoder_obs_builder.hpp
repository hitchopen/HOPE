// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Fixed flat layouts for the A3 encoder+decoder ONNX path.
#pragma once

#include "a3_deploy/a3_obs_builder.hpp"

#include <array>
#include <cstddef>
#include <cstring>

namespace a3_deploy {

inline constexpr std::size_t kA3TokenDim = 64;
inline constexpr std::size_t kA3EncoderInputTotalFloats =
    1 + kA3TokenizerTotalFloats + kA3SmplTokenizerTotalFloats;  // 1481
inline constexpr std::size_t kA3DecoderInputTotalFloats =
    kA3TokenDim + kA3ProprioTotalFloats;  // 994

inline constexpr std::size_t kA3SmplJointsFloats = 10 * 24 * 3;  // 720
inline constexpr std::size_t kA3SmplRootOriFloats = 10 * 6;      // 60
inline constexpr std::size_t kA3SmplWristJointFloats = 10 * 6;   // 60

enum class A3EncoderSource : int {
  kA3 = 0,
  kSmpl = 1,
  kA3Fast = 2,
};

inline const char* A3EncoderSourceName(A3EncoderSource source) noexcept {
  switch (source) {
    case A3EncoderSource::kA3: return "a3";
    case A3EncoderSource::kSmpl: return "smpl";
    case A3EncoderSource::kA3Fast: return "a3_fast";
  }
  return "unknown";
}

inline void BuildA3EncoderInput(
    int encoder_index,
    const float* a3_tokenizer_640,
    const float* smpl_tokenizer_840,
    std::array<float, kA3EncoderInputTotalFloats>& out) noexcept {
  out.fill(0.0f);
  out[0] = static_cast<float>(encoder_index);
  if (a3_tokenizer_640 != nullptr) {
    std::memcpy(out.data() + 1, a3_tokenizer_640,
                kA3TokenizerTotalFloats * sizeof(float));
  }
  if (smpl_tokenizer_840 != nullptr) {
    std::memcpy(out.data() + 1 + kA3TokenizerTotalFloats, smpl_tokenizer_840,
                kA3SmplTokenizerTotalFloats * sizeof(float));
  }
}

inline void BuildA3DecoderInput(
    const float* encoded_tokens_64,
    const std::array<float, kA3ProprioTotalFloats>& proprio_930,
    std::array<float, kA3DecoderInputTotalFloats>& out) noexcept {
  if (encoded_tokens_64 != nullptr) {
    std::memcpy(out.data(), encoded_tokens_64, kA3TokenDim * sizeof(float));
  } else {
    std::memset(out.data(), 0, kA3TokenDim * sizeof(float));
  }
  std::memcpy(out.data() + kA3TokenDim, proprio_930.data(),
              proprio_930.size() * sizeof(float));
}

}  // namespace a3_deploy
