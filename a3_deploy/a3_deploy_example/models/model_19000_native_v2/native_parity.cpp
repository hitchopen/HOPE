// Uses the exact explicit-state class called by production PpOnnxPolicy::Run.
#include "a3_pingpong/pp_explicit_gru.hpp"
#include "a3_pingpong/pp_reference_clock.hpp"
#ifdef HOPE_VERIFY_FULL_POLICY
#include "a3_pingpong/pp_onnx_policy.hpp"
#endif
#include <fstream>
#include <iostream>
#include <cstring>
#include <limits>
#include <sstream>

static std::vector<float> Read(const std::string& path) {
  std::ifstream f(path,std::ios::binary|std::ios::ate);
  if(!f) throw std::runtime_error("Cannot read "+path);
  const auto size=f.tellg(); if(size<0 || size%4) throw std::runtime_error("Invalid float file");
  std::vector<float> x(static_cast<size_t>(size)/4);f.seekg(0);f.read(reinterpret_cast<char*>(x.data()),size);
  if(!f) throw std::runtime_error("Short read");return x;
}
static void Check(float a,float b,float& maximum) {
  if(!std::isfinite(a)||!std::isfinite(b)) throw std::runtime_error("Nonfinite parity value");
  maximum=std::max(maximum,std::abs(a-b));
  if(std::abs(a-b)>2e-5f+2e-4f*std::abs(b)) throw std::runtime_error("Native parity tolerance exceeded");
}
int main(int argc,char** argv) {
 try {
  if(argc!=5 && argc!=6)throw std::runtime_error("usage: native_parity original.onnx candidate.onnx evidence_dir report.json");
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"native_gru_parity");Ort::SessionOptions opts;opts.SetIntraOpNumThreads(1);
  Ort::Session original(env,argv[1],opts),wrapped(env,argv[2],opts);
  a3_pingpong::PpExplicitGru a(original),b(wrapped);
  a3_pingpong::ClipLayout clip;
  Ort::AllocatorWithDefaultOptions allocator;
  auto metadata=wrapped.GetModelMetadata();
  const auto lengths=metadata.LookupCustomMetadataMapAllocated("clip_seg_lengths",allocator);
  const auto phases=metadata.LookupCustomMetadataMapAllocated("clip_strike_phases",allocator);
  if(!lengths||!phases)throw std::runtime_error("Missing frozen clip clock metadata");
  std::string length_text=lengths.get(),phase_text=phases.get();
  std::replace(length_text.begin(),length_text.end(),',',' ');std::replace(phase_text.begin(),phase_text.end(),',',' ');
  std::istringstream length_stream(length_text),phase_stream(phase_text);
  length_stream>>clip.seg_len[0]>>clip.seg_len[1];phase_stream>>clip.strike_phase[0]>>clip.strike_phase[1];
  if(clip.seg_len[0]!=107||clip.seg_len[1]!=109||clip.strike_frame(0)!=41||clip.strike_frame(1)!=155)
    throw std::runtime_error("Frozen motion clock identity mismatch");
  for(int c=0;c<2;++c) {
    for(int j=0;j<clip.seg_len[c];++j) {
      const int frame=clip.seg_start(c)+j;
      const double tts=(clip.strike_frame(c)-frame)*clip.step_dt;
      if(clip.time_step_for(c,tts)!=frame)throw std::runtime_error("Native clock/sequence frame mismatch");
    }
    if(clip.time_step_for(c,10)!=clip.seg_start(c)||clip.time_step_for(c,-10)!=clip.seg_start(c)+clip.seg_len[c]-1)
      throw std::runtime_error("Native clock bounds mismatch");
  }
  const std::string dir=argv[3];auto obs=Read(dir+"/obs.f32"),expected=Read(dir+"/actions.f32"),hidden=Read(dir+"/hidden.f32"),initial=Read(dir+"/initial_h.f32"),reference=Read(dir+"/refs.f32");
  if(obs.size()%110 || initial.size()!=110 || expected.size()!=obs.size()/110*31 || hidden.size()!=obs.size())throw std::runtime_error("Fixture dimensions");
  std::array<float,110> state;std::copy(initial.begin(),initial.end(),state.begin());a.SetVerificationState(state);b.SetVerificationState(state);
  float action_error=0,hidden_error=0,ref_error=0;const size_t count=obs.size()/110;
#ifdef HOPE_VERIFY_FULL_POLICY
  const std::string candidate_path=argv[2];const auto slash=candidate_path.find_last_of('/');
  const std::string yaml=candidate_path.substr(0,slash)+"/../params/deploy.yaml";
  a3_pingpong::PpExplicitGru full_reference(original);
  a3_pingpong::PpOnnxPolicy full(candidate_path,a3_pingpong::PpOnnxLoadProfile::kProductionStrict,yaml);
  if(!full.is_recurrent() || full.recurrent_advances()!=0)throw std::runtime_error("Full loader did not initialize GRU once");
#endif
  for(size_t i=0;i<count;++i) {
   if(i==32||i==192){a.ResetSession();b.ResetSession();}
   a.BeginTick(i);b.BeginTick(i);std::vector<float> x(obs.begin()+i*110,obs.begin()+(i+1)*110);
   const auto before=b.hidden();const auto n=b.advances();
   auto refs=b.Run(x,static_cast<int>(i),{"joint_pos","joint_vel","body_pos_w","body_quat_w"});
   size_t cursor=i*160;
   for(auto& value:refs) {
    const auto info=value.GetTensorTypeAndShapeInfo();const auto* values=value.GetTensorData<float>();
    for(size_t j=0;j<info.GetElementCount();++j)Check(values[j],reference.at(cursor++),ref_error);
   }
   if(b.hidden()!=before || b.advances()!=n)throw std::runtime_error("Reference query advanced hidden state");
   auto left=a.Run(x,static_cast<int>(i),{"actions"});auto right=b.Run(x,static_cast<int>(i),{"actions"});
   for(size_t j=0;j<31;++j){Check(right[0].GetTensorData<float>()[j],left[0].GetTensorData<float>()[j],action_error);Check(right[0].GetTensorData<float>()[j],expected[i*31+j],action_error);}
   for(size_t j=0;j<110;++j){Check(b.hidden()[j],a.hidden()[j],hidden_error);Check(b.hidden()[j],hidden[i*110+j],hidden_error);}
   const auto memory=b.hidden();const auto advances=b.advances();
   b.BeginTick(i);auto duplicate=b.Run(x,static_cast<int>(i),{"actions"});
   if(b.hidden()!=memory || b.advances()!=advances)throw std::runtime_error("Duplicate advanced GRU");
   for(size_t j=0;j<31;++j)Check(duplicate[0].GetTensorData<float>()[j],right[0].GetTensorData<float>()[j],action_error);
#ifdef HOPE_VERIFY_FULL_POLICY
   // Actual full loader/Run path: zero start, then carry; same native state engine.
   if(i==0 || i==32 || i==192) {full.ResetRecurrentSession();full_reference.ResetSession();}
   full_reference.BeginTick(i);auto expected_full=full_reference.Run(x,static_cast<int>(i),{"actions"});
   full.BeginPolicyTick(i);Eigen::VectorXd input(110);for(int j=0;j<110;++j)input[j]=x[j];
   const auto ref=full.refs(static_cast<int>(i));const auto old_count=full.recurrent_advances();
   const auto output=full.mean_action(input,static_cast<int>(i));
   const auto repeat=full.mean_action(input,static_cast<int>(i));
   if(full.recurrent_advances()!=old_count+1 || (output-repeat).cwiseAbs().maxCoeff()!=0)throw std::runtime_error("Full runtime tick count/cache");
   for(int j=0;j<31;++j)Check(static_cast<float>(output[j]),expected_full[0].GetTensorData<float>()[j],action_error);
   for(int j=0;j<110;++j)Check(full.recurrent_state()[j],full_reference.hidden()[j],hidden_error);
#endif
  }
  b.ResetSession();b.BeginTick(0);std::vector<float> first(obs.begin(),obs.begin()+110);auto reset=b.Run(first,0,{"actions"});
  std::array<float,31> saved_action;std::copy_n(reset[0].GetTensorData<float>(),31,saved_action.begin());const auto saved_h=b.hidden();
  for(int i=1;i<9;++i){b.BeginTick(i);b.Run(first,i,{"actions"});}
  b.ResetSession();b.BeginTick(0);auto repeated=b.Run(first,0,{"actions"});
  if(b.hidden()!=saved_h || !std::equal(saved_action.begin(),saved_action.end(),repeated[0].GetTensorData<float>()))throw std::runtime_error("Reset repeat");
  bool rejected=false;try{first[0]+=1;b.Run(first,0,{"actions"});}catch(const std::runtime_error&){rejected=true;}
  if(!rejected)throw std::runtime_error("Changed duplicate observation accepted");
  // Reference clock HOLD/recovery/clip handoffs must never own recurrent reset.
  a.ResetSession(); b.ResetSession();
  const std::array<int,7> clocks{0,0,53,107,107,170,0};
  for(size_t i=0;i<clocks.size();++i) {
    a.BeginTick(i);b.BeginTick(i);
    auto left=a.Run(first,clocks[i],{"actions"});
    auto right=b.Run(first,clocks[i],{"actions"});
    for(int j=0;j<31;++j)Check(right[0].GetTensorData<float>()[j],left[0].GetTensorData<float>()[j],action_error);
    for(int j=0;j<110;++j)Check(a.hidden()[j],b.hidden()[j],hidden_error);
    if(b.advances()!=i+1)throw std::runtime_error("Reference clock handoff reset GRU");
  }
  const auto safe_h=b.hidden();const auto safe_count=b.advances();
  bool bad_tick=false,bad_query=false,bad_finite=false;
  try{b.BeginTick(0);}catch(const std::runtime_error&){bad_tick=true;}
  try{b.Run(first,0,{"h_out"});}catch(const std::runtime_error&){bad_query=true;}
  auto bad=first;bad[0]=std::numeric_limits<float>::quiet_NaN();
  try{b.Run(bad,0,{"actions"});}catch(const std::runtime_error&){bad_finite=true;}
  if(!bad_tick||!bad_query||!bad_finite||safe_h!=b.hidden()||safe_count!=b.advances())throw std::runtime_error("Invalid call did not fail closed");
#ifdef HOPE_VERIFY_FULL_POLICY
  if(argc!=6) throw std::runtime_error("Full policy parity requires genuine feed-forward fixture path");
  const std::string ff_path=argv[5];const auto pos=ff_path.find_last_of('/');
  a3_pingpong::PpOnnxPolicy ff(ff_path,a3_pingpong::PpOnnxLoadProfile::kProductionStrict,ff_path.substr(0,pos)+"/../params/deploy.yaml");
  Ort::Session ff_session(env,argv[5],opts);if(ff.is_recurrent())throw std::runtime_error("Feed-forward fixture was relabelled");
  auto mem=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
  for(int i=0;i<8;++i) {
    Eigen::VectorXd input=Eigen::VectorXd::Zero(ff.obs_dim());std::vector<float> x(ff.obs_dim(),0);
    std::array<int64_t,2> shape{1,ff.obs_dim()},ts_shape{1,1};float ts=static_cast<float>(i);
    std::array<Ort::Value,2> in{Ort::Value::CreateTensor<float>(mem,x.data(),x.size(),shape.data(),2),Ort::Value::CreateTensor<float>(mem,&ts,1,ts_shape.data(),2)};
    const char* names[]={"obs","time_step"};const char* outputs[]={"actions"};auto expected_ff=ff_session.Run(Ort::RunOptions{nullptr},names,in.data(),2,outputs,1);
    ff.BeginPolicyTick(i);auto actual=ff.mean_action(input,i);ff.ResetRecurrentSession();
    for(int j=0;j<31;++j)Check(static_cast<float>(actual[j]),expected_ff[0].GetTensorData<float>()[j],action_error);
    if(ff.recurrent_advances()!=0)throw std::runtime_error("Feed-forward state behavior changed");
  }
#endif
  std::ofstream report(argv[4]);report<<"{\"status\":\"PASS\",\"steps\":"<<count<<",\"max_action_error\":"<<action_error<<",\"max_hidden_error\":"<<hidden_error<<",\"max_reference_error\":"<<ref_error<<",\"duplicate_no_advance\":true,\"reference_no_advance\":true,\"reset_repeat\":true,\"nonzero_initial_state\":true,\"reference_clock_handoff_carry\":true,\"invalid_calls_fail_closed\":true,\"reference_clock_all_216_frames\":true,\"full_policy_loader_test\":"
#ifdef HOPE_VERIFY_FULL_POLICY
  <<"true,\"feedforward_compatibility_pass\":true"
#else
  <<"false"
#endif
  <<"}\n";
  std::cout<<"NATIVE GRU PARITY PASS "<<count<<" action="<<action_error<<" hidden="<<hidden_error<<" reference="<<ref_error<<"\n";return 0;
 } catch(const std::exception& e){std::cerr<<"NATIVE_GRU_PARITY_FAILED: "<<e.what()<<"\n";return 2;}
}
