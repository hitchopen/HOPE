// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §5 (Tests — test_a3_tokenizer_replay.cpp)
//
// Verifies A3TokenizerReplay loading, At() indexing, and the three on-end
// policies (kHoldLast, kWrap, kStop) against a synthetic 10-tick binary.
#include <gtest/gtest.h>

#include "a3_deploy/a3_tokenizer_replay.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using a3_deploy::A3TokenizerReplay;
using a3_deploy::kA3TokenizerBytesPerTick;
using a3_deploy::kA3TokenizerFloatsPerTick;
using a3_deploy::OnEndPolicy;

namespace {

// Builds a synthetic replay dataset: tick t float f = static_cast<float>(t * 1000 + f).
// This encoding makes miscounted indexing / shape errors easy to spot in test
// failures.
std::vector<float> BuildSyntheticData(std::size_t num_ticks) {
  std::vector<float> out(num_ticks * kA3TokenizerFloatsPerTick);
  for (std::size_t t = 0; t < num_ticks; ++t) {
    for (std::size_t f = 0; f < kA3TokenizerFloatsPerTick; ++f) {
      out[t * kA3TokenizerFloatsPerTick + f] =
          static_cast<float>(t * 1000 + f);
    }
  }
  return out;
}

class TokenizerReplayFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("a3_tok_replay_test_" + std::to_string(::getpid()) + "_" +
                std::to_string(
                    ::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
    bin_path_  = (tmp_dir_ / "tokenizer.bin").string();
    meta_path_ = (tmp_dir_ / "meta.json").string();
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  void WriteBinary(const std::vector<float>& floats) {
    std::ofstream f(bin_path_, std::ios::binary);
    ASSERT_TRUE(f.is_open());
    f.write(reinterpret_cast<const char*>(floats.data()),
            static_cast<std::streamsize>(floats.size() * sizeof(float)));
    ASSERT_TRUE(f.good());
  }

  void WriteMeta(std::size_t num_ticks) {
    std::ofstream f(meta_path_);
    ASSERT_TRUE(f.is_open());
    // JSON-as-YAML; yaml-cpp parses both.
    f << "{\n"
      << "  \"num_ticks\": " << num_ticks << ",\n"
      << "  \"dt_ns\": 20000000,\n"
      << "  \"pkl_source\": \"/tmp/fake.pkl\",\n"
      << "  \"clip_name\": \"fake_clip\",\n"
      << "  \"checkpoint_path\": \"/tmp/fake.pt\",\n"
      << "  \"generation_timestamp\": \"2026-04-27T00:00:00Z\",\n"
      << "  \"git_commit\": \"deadbeef\"\n"
      << "}\n";
    ASSERT_TRUE(f.good());
  }

  std::filesystem::path tmp_dir_;
  std::string bin_path_;
  std::string meta_path_;
};

}  // namespace

TEST_F(TokenizerReplayFixture, LoadsSyntheticTenTickBin) {
  const std::size_t N = 10;
  auto floats = BuildSyntheticData(N);
  WriteBinary(floats);
  WriteMeta(N);

  A3TokenizerReplay replay;
  ASSERT_TRUE(replay.Load(bin_path_, meta_path_));
  EXPECT_EQ(replay.NumTicks(), N);
  EXPECT_EQ(replay.Meta().dt_ns, 20'000'000);
  EXPECT_EQ(replay.Meta().clip_name, "fake_clip");
  EXPECT_EQ(replay.Meta().pkl_source, "/tmp/fake.pkl");
  EXPECT_EQ(replay.Meta().git_commit, "deadbeef");

  // Spot-check a few ticks byte-exact.
  const float* t0 = replay.At(0);
  ASSERT_NE(t0, nullptr);
  EXPECT_FLOAT_EQ(t0[0],   0.0f);
  EXPECT_FLOAT_EQ(t0[639], 639.0f);

  const float* t5 = replay.At(5);
  ASSERT_NE(t5, nullptr);
  EXPECT_FLOAT_EQ(t5[0],   5000.0f);
  EXPECT_FLOAT_EQ(t5[100], 5100.0f);
  EXPECT_FLOAT_EQ(t5[639], 5639.0f);

  const float* t9 = replay.At(9);
  ASSERT_NE(t9, nullptr);
  EXPECT_FLOAT_EQ(t9[0],   9000.0f);
  EXPECT_FLOAT_EQ(t9[639], 9639.0f);
}

TEST_F(TokenizerReplayFixture, HoldLastReturnsLastSliceBeyondEnd) {
  const std::size_t N = 10;
  WriteBinary(BuildSyntheticData(N));
  WriteMeta(N);

  A3TokenizerReplay replay;
  ASSERT_TRUE(replay.Load(bin_path_, meta_path_, OnEndPolicy::kHoldLast));

  const float* t9  = replay.At(9);
  const float* t10 = replay.At(10);
  const float* t100 = replay.At(100);
  ASSERT_NE(t9, nullptr);
  ASSERT_NE(t10, nullptr);
  ASSERT_NE(t100, nullptr);
  EXPECT_EQ(t9, t10);
  EXPECT_EQ(t9, t100);
}

TEST_F(TokenizerReplayFixture, WrapLoopsBackToStart) {
  const std::size_t N = 10;
  WriteBinary(BuildSyntheticData(N));
  WriteMeta(N);

  A3TokenizerReplay replay;
  ASSERT_TRUE(replay.Load(bin_path_, meta_path_, OnEndPolicy::kWrap));

  EXPECT_EQ(replay.At(10), replay.At(0));
  EXPECT_EQ(replay.At(13), replay.At(3));
  EXPECT_EQ(replay.At(25), replay.At(5));
}

TEST_F(TokenizerReplayFixture, StopReturnsNullptrBeyondEnd) {
  const std::size_t N = 10;
  WriteBinary(BuildSyntheticData(N));
  WriteMeta(N);

  A3TokenizerReplay replay;
  ASSERT_TRUE(replay.Load(bin_path_, meta_path_, OnEndPolicy::kStop));

  EXPECT_NE(replay.At(0), nullptr);
  EXPECT_NE(replay.At(9), nullptr);
  EXPECT_EQ(replay.At(10), nullptr);
  EXPECT_EQ(replay.At(1000), nullptr);
}

TEST_F(TokenizerReplayFixture, LoadFailsOnMissingBin) {
  WriteMeta(10);
  A3TokenizerReplay replay;
  EXPECT_FALSE(replay.Load((tmp_dir_ / "no_such.bin").string(), meta_path_));
  EXPECT_EQ(replay.NumTicks(), 0u);
  EXPECT_EQ(replay.At(0), nullptr);
}

TEST_F(TokenizerReplayFixture, LoadFailsOnMissingMeta) {
  WriteBinary(BuildSyntheticData(10));
  A3TokenizerReplay replay;
  EXPECT_FALSE(replay.Load(bin_path_, (tmp_dir_ / "no_such.json").string()));
  EXPECT_EQ(replay.NumTicks(), 0u);
}

TEST_F(TokenizerReplayFixture, LoadFailsOnShapeMismatch) {
  // Write 9 ticks but claim 10 in meta — should fail.
  WriteBinary(BuildSyntheticData(9));
  WriteMeta(10);

  A3TokenizerReplay replay;
  EXPECT_FALSE(replay.Load(bin_path_, meta_path_));
  EXPECT_EQ(replay.NumTicks(), 0u);
}

TEST_F(TokenizerReplayFixture, LoadFailsOnBinSizeNotMultipleOf640) {
  // 640*4 - 1 bytes: not a multiple of the slice size.
  std::vector<char> garbage(kA3TokenizerBytesPerTick - 1, 'x');
  {
    std::ofstream f(bin_path_, std::ios::binary);
    ASSERT_TRUE(f.is_open());
    f.write(garbage.data(), static_cast<std::streamsize>(garbage.size()));
  }
  WriteMeta(1);

  A3TokenizerReplay replay;
  EXPECT_FALSE(replay.Load(bin_path_, meta_path_));
}

TEST_F(TokenizerReplayFixture, LoadFailsOnZeroNumTicks) {
  WriteBinary({});  // empty bin
  WriteMeta(0);

  A3TokenizerReplay replay;
  EXPECT_FALSE(replay.Load(bin_path_, meta_path_));
}

TEST(A3TokenizerReplay, UnloadedReplayReturnsNullptr) {
  A3TokenizerReplay replay;
  EXPECT_EQ(replay.At(0), nullptr);
  EXPECT_EQ(replay.NumTicks(), 0u);
}
