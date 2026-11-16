#pragma once
// Shared by the production Runner and the bounded native parity executable.
#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace a3_pingpong {
class PpExplicitGru {
 public:
  static bool IsRecurrent(Ort::Session& session) {
    Ort::AllocatorWithDefaultOptions alloc;
    for (size_t i = 0; i < session.GetInputCount(); ++i)
      if (std::string(session.GetInputNameAllocated(i, alloc).get()) == "h_in") return true;
    for (size_t i = 0; i < session.GetOutputCount(); ++i)
      if (std::string(session.GetOutputNameAllocated(i, alloc).get()) == "h_out") return true;
    return false;
  }
  explicit PpExplicitGru(Ort::Session& session) : session_(session),
      memory_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::AllocatorWithDefaultOptions alloc;
    std::map<std::string, std::vector<int64_t>> inputs, outputs;
    for (size_t i = 0; i < session.GetInputCount(); ++i) {
      const auto type = session.GetInputTypeInfo(i);
      const auto info = type.GetTensorTypeAndShapeInfo();
      if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error("GRU inputs must be float32");
      inputs.emplace(session.GetInputNameAllocated(i, alloc).get(), info.GetShape());
    }
    for (size_t i = 0; i < session.GetOutputCount(); ++i) {
      const auto type = session.GetOutputTypeInfo(i);
      const auto info = type.GetTensorTypeAndShapeInfo();
      if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error("GRU outputs must be float32");
      outputs.emplace(session.GetOutputNameAllocated(i, alloc).get(), info.GetShape());
    }
    if (inputs["obs"] != std::vector<int64_t>{1,110} ||
        inputs["h_in"] != std::vector<int64_t>{1,1,110} ||
        outputs["actions"] != std::vector<int64_t>{1,31} ||
        outputs["h_out"] != std::vector<int64_t>{1,1,110})
      throw std::runtime_error("Unsupported explicit GRU obs/action/state shapes");
    has_clock_ = inputs.count("time_step") != 0;
    if ((has_clock_ && inputs["time_step"] != std::vector<int64_t>{1,1}) ||
        inputs.size() != (has_clock_ ? 3u : 2u))
      throw std::runtime_error("Unsupported explicit GRU inputs/clock");
    has_refs_ = outputs.count("joint_pos") && outputs.count("joint_vel") &&
                outputs.count("body_pos_w") && outputs.count("body_quat_w") &&
                outputs.count("body_lin_vel_w") && outputs.count("body_ang_vel_w");
    if (outputs.size() != (has_refs_ ? 8u : 2u))
      throw std::runtime_error("Unsupported explicit GRU outputs");
    if (has_refs_ && (!has_clock_ || outputs["joint_pos"] != std::vector<int64_t>{1,31} ||
        outputs["joint_vel"] != std::vector<int64_t>{1,31} ||
        outputs["body_pos_w"] != std::vector<int64_t>{1,14,3} ||
        outputs["body_quat_w"] != std::vector<int64_t>{1,14,4} ||
        outputs["body_lin_vel_w"] != std::vector<int64_t>{1,14,3} ||
        outputs["body_ang_vel_w"] != std::vector<int64_t>{1,14,3}))
      throw std::runtime_error("GRU genuine reference output shapes/clock invalid");
    ResetSession();
  }
  bool has_references() const { return has_refs_; }
  const std::array<float,110>& hidden() const { return hidden_; }
  uint64_t advances() const { return advances_; }
  void ResetSession() {
    hidden_.fill(0); cached_action_.fill(0); have_tick_ = cached_ = false; advances_ = 0;
  }
  // Bounded parity tool uses arbitrary nonzero state; production never restores training memory.
  void SetVerificationState(const std::array<float,110>& state) {
    for (float v : state) if (!std::isfinite(v)) throw std::runtime_error("Nonfinite GRU state");
    ResetSession(); hidden_ = state;
  }
  void BeginTick(uint64_t tick) {
    if (have_tick_ && tick < tick_) throw std::runtime_error("GRU control tick regressed");
    if (!have_tick_ || tick != tick_) { tick_ = tick; have_tick_ = true; cached_ = false; }
  }
  std::vector<Ort::Value> Run(const std::vector<float>& obs, int clock,
                              const std::vector<const char*>& requested) {
    if (obs.size()!=110) throw std::runtime_error("GRU observation dimension !=110");
    for (float v : obs) if (!std::isfinite(v)) throw std::runtime_error("Nonfinite GRU observation");
    const bool actor = std::any_of(requested.begin(),requested.end(),
                                  [](const char* s){return std::string(s)=="actions";});
    if (actor && (requested.size()!=1 || !have_tick_))
      throw std::runtime_error("GRU action requires one actions output and BeginTick");
    if (actor && cached_) {
      if (obs != cached_obs_ || clock != cached_clock_)
        throw std::runtime_error("Different duplicate GRU observation/clock within one tick");
      std::vector<Ort::Value> result;
      std::array<int64_t,2> shape{1,31};
      result.emplace_back(Ort::Value::CreateTensor<float>(memory_,cached_action_.data(),31,shape.data(),2));
      return result;
    }
    if (!actor) {
      for (const char* s : requested) {
        const std::string name(s);
        if (name=="h_out" || !has_refs_ ||
            (name!="joint_pos" && name!="joint_vel" && name!="body_pos_w" &&
             name!="body_quat_w" && name!="body_lin_vel_w" && name!="body_ang_vel_w"))
          throw std::runtime_error("Unsupported read-only GRU query");
      }
    }
    std::array<int64_t,2> obs_shape{1,110}, clock_shape{1,1};
    std::array<int64_t,3> h_shape{1,1,110};
    std::vector<float> input(obs); auto h_input=hidden_; float ts=static_cast<float>(clock);
    std::vector<Ort::Value> inputs;
    inputs.emplace_back(Ort::Value::CreateTensor<float>(memory_,input.data(),110,obs_shape.data(),2));
    inputs.emplace_back(Ort::Value::CreateTensor<float>(memory_,h_input.data(),110,h_shape.data(),3));
    std::vector<const char*> names{"obs","h_in"};
    if (has_clock_) {
      names.push_back("time_step");
      inputs.emplace_back(Ort::Value::CreateTensor<float>(memory_,&ts,1,clock_shape.data(),2));
    }
    std::vector<const char*> outputs(requested);
    if (actor) outputs.push_back("h_out");
    auto result=session_.Run(Ort::RunOptions{nullptr},names.data(),inputs.data(),inputs.size(),outputs.data(),outputs.size());
    for (const auto& value : result) {
      const auto info=value.GetTensorTypeAndShapeInfo();
      if (info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error("GRU result dtype changed");
      const float* v=value.GetTensorData<float>();
      for(size_t i=0;i<info.GetElementCount();++i)
        if(!std::isfinite(v[i])) throw std::runtime_error("Nonfinite GRU/reference output");
    }
    if (actor) {
      if(result[0].GetTensorTypeAndShapeInfo().GetShape()!=std::vector<int64_t>{1,31} ||
         result[1].GetTensorTypeAndShapeInfo().GetShape()!=std::vector<int64_t>{1,1,110})
        throw std::runtime_error("GRU result shape changed");
      std::copy_n(result[0].GetTensorData<float>(),31,cached_action_.begin());
      std::copy_n(result[1].GetTensorData<float>(),110,hidden_.begin());
      ++advances_; cached_obs_=obs; cached_clock_=clock; cached_=true;
      result.pop_back();
    }
    return result;
  }
 private:
  Ort::Session& session_; Ort::MemoryInfo memory_;
  std::array<float,110> hidden_{}; std::array<float,31> cached_action_{};
  std::vector<float> cached_obs_; uint64_t tick_=0,advances_=0;
  int cached_clock_=0; bool have_tick_=false,cached_=false,has_clock_=false,has_refs_=false;
};
} // namespace a3_pingpong
