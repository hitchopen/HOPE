#include "a3_deploy/a3_encoder_obs_builder.hpp"

#include <gtest/gtest.h>

#include <array>

namespace {

template <std::size_t N>
std::array<float, N> Sequence(float base) {
  std::array<float, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = base + static_cast<float>(i);
  }
  return out;
}

}  // namespace

TEST(A3EncoderObsBuilder, SplitOnnxDimensionsMatchExportContract) {
  EXPECT_EQ(a3_deploy::kA3TokenDim, 64u);
  EXPECT_EQ(a3_deploy::kA3SmplTokenizerTotalFloats, 840u);
  EXPECT_EQ(a3_deploy::kA3EncoderInputTotalFloats, 1481u);
  EXPECT_EQ(a3_deploy::kA3DecoderInputTotalFloats, 994u);
  EXPECT_EQ(a3_deploy::kA3SmplJointsFloats, 720u);
  EXPECT_EQ(a3_deploy::kA3SmplRootOriFloats, 60u);
  EXPECT_EQ(a3_deploy::kA3SmplWristJointFloats, 60u);
}

TEST(A3EncoderObsBuilder, EncoderInputPacksIndexA3AndSmplSlices) {
  const auto a3 = Sequence<a3_deploy::kA3TokenizerTotalFloats>(1000.0f);
  const auto smpl = Sequence<a3_deploy::kA3SmplTokenizerTotalFloats>(2000.0f);

  std::array<float, a3_deploy::kA3EncoderInputTotalFloats> out{};
  a3_deploy::BuildA3EncoderInput(1, a3.data(), smpl.data(), out);

  EXPECT_FLOAT_EQ(out[0], 1.0f);
  EXPECT_FLOAT_EQ(out[1], 1000.0f);
  EXPECT_FLOAT_EQ(out[a3_deploy::kA3TokenizerTotalFloats], 1639.0f);
  EXPECT_FLOAT_EQ(out[1 + a3_deploy::kA3TokenizerTotalFloats], 2000.0f);
  EXPECT_FLOAT_EQ(out.back(), 2839.0f);
}

TEST(A3EncoderObsBuilder, EncoderInputZerosInactiveSource) {
  const auto smpl = Sequence<a3_deploy::kA3SmplTokenizerTotalFloats>(10.0f);

  std::array<float, a3_deploy::kA3EncoderInputTotalFloats> out{};
  a3_deploy::BuildA3EncoderInput(3, nullptr, smpl.data(), out);

  EXPECT_FLOAT_EQ(out[0], 3.0f);
  for (std::size_t i = 1; i <= a3_deploy::kA3TokenizerTotalFloats; ++i) {
    EXPECT_FLOAT_EQ(out[i], 0.0f) << "a3 slot " << i;
  }
  EXPECT_FLOAT_EQ(out[1 + a3_deploy::kA3TokenizerTotalFloats], 10.0f);
}

TEST(A3EncoderObsBuilder, DecoderInputPacksTokenAndProprio) {
  const auto token = Sequence<a3_deploy::kA3TokenDim>(5.0f);
  const auto proprio = Sequence<a3_deploy::kA3ProprioTotalFloats>(100.0f);

  std::array<float, a3_deploy::kA3DecoderInputTotalFloats> out{};
  a3_deploy::BuildA3DecoderInput(token.data(), proprio, out);

  EXPECT_FLOAT_EQ(out[0], 5.0f);
  EXPECT_FLOAT_EQ(out[a3_deploy::kA3TokenDim - 1], 68.0f);
  EXPECT_FLOAT_EQ(out[a3_deploy::kA3TokenDim], 100.0f);
  EXPECT_FLOAT_EQ(out.back(), 1029.0f);
}
