#pragma once
// Pure, deterministic twin of hope_observer_core / hope_runner_control_core.
// Keep validation order, receipt semantics and output values paired with the
// Python reference. Differential replay tests cover every emitted scalar.
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fmt/format.h>

namespace hope_operations {
using J = nlohmann::json;
constexpr std::int64_t max_integer = 1LL << 52;
inline double nan() { return std::numeric_limits<double>::quiet_NaN(); }
inline std::string repr(double v) { auto s = fmt::format("{}", v); if (s.find_first_of(".eE") == std::string::npos) s += ".0"; return s; }
inline std::int64_t integer(double v, const std::string &label, std::int64_t lo=0, std::int64_t hi=max_integer) {
  if (!std::isfinite(v)) throw std::runtime_error(label + " is not finite");
  if (v < lo || v > hi || std::trunc(v) != v)
    throw std::runtime_error(fmt::format("{} must be an exact integer in [{}, {}]", label, lo, hi));
  return static_cast<std::int64_t>(v);
}
inline void validate(const std::vector<double> &v, std::size_t size, const std::string &label, int schema=2, bool flag=true) {
  if (v.size()!=size) throw std::runtime_error(fmt::format("{} has {} values; expected {}",label,v.size(),size));
  for (auto x:v) if (!std::isfinite(x)) throw std::runtime_error(label+" contains a non-finite value");
  if(v[0]!=schema) throw std::runtime_error(fmt::format("{} schema is {}; expected {}",label,repr(v[0]),schema));
  if(flag && v[1]!=0 && v[1]!=1) throw std::runtime_error(label+" validity field is not 0 or 1");
}
inline J stamp(double sec, double ns, const std::string &label) {
  auto s=integer(sec,label+" seconds",1), n=integer(ns,label+" nanoseconds",0,999999999);
  // Python permits timestamps beyond int64. Keep that domain without UB;
  // production timestamps use int64, oversized synthetic stamps use decimal.
  if(s <= (INT64_MAX-n)/1000000000LL) return s*1000000000LL+n;
  return std::to_string(s)+fmt::format("{:09}",n);
}
inline double numeric(const J &v) { return v.is_string()?std::stod(v.get<std::string>()):v.get<double>(); }
inline J decode_base(const std::vector<double> &v) {
  validate(v,16,"base schema-2 packet");
  auto seq=integer(v[2],"base sequence");
  if(v[1]==0) return {{"valid",false},{"sequence",seq},{"source_time_ns",0},
    {"position_xyz",{0.,0.,0.}},{"quaternion_wxyz",{1.,0.,0.,0.}},{"tracking_quality",0.},
    {"flags",integer(v[13],"base flags",0,1<<20)},{"calibration_id",0},{"world_frame_id",0},
    {"reason","producer marked base packet invalid"}};
  auto time=stamp(v[3],v[4],"base source stamp");
  double norm=std::sqrt(v[8]*v[8]+v[9]*v[9]+v[10]*v[10]+v[11]*v[11]);
  if(norm<.5 || norm>1.5) throw std::runtime_error("base quaternion norm is outside [0.5, 1.5]");
  if(v[12]<0 || v[12]>1) throw std::runtime_error("base tracking quality is outside [0, 1]");
  auto flags=integer(v[13],"base flags",0,1<<20);
  if((flags&47)!=47) throw std::runtime_error("base packet is missing required base flags");
  auto calibration=integer(v[14],"base calibration id",1), world=integer(v[15],"base world-frame id",1);
  return {{"valid",true},{"sequence",seq},{"source_time_ns",time},
    {"position_xyz",{v[5],v[6],v[7]}},{"quaternion_wxyz",{v[8],v[9],v[10],v[11]}},
    {"tracking_quality",v[12]},{"flags",flags},{"calibration_id",calibration},
    {"world_frame_id",world},{"reason","valid"}};
}
inline J decode_racket(const std::vector<double> &v) {
  int schema=v.size()==20?4:v.size()==22?5:2;
  validate(v,schema==2?19:schema==4?20:22,"racket packet",schema);
  auto home=schema==2?0:integer(v[19],"HOME epoch",static_cast<int>(v[1]));
  if(schema!=2 && v[11]!=0) throw std::runtime_error("shared-HOME target must use world frame");
  if(schema==5 && v[21]!=1 && v[21]!=2) throw std::runtime_error("dynamic plane mode must be current or posture-stable");
  auto seq=integer(v[14],"command sequence");
  J out={{"valid",v[1]==1},{"swing_sign",0.},{"position_xyz",{0.,0.,0.}},
    {"velocity_xyz",{0.,0.,0.}},{"published_time_to_strike_s",0.},{"strike_deadline_wall_s",0.},
    {"producer_time_ns",0},{"command_sequence",seq},{"flight_id",0},{"revision_id",0},
    {"estimator_sample_count",0},{"estimator_span_s",0.},{"home_epoch",home},
    {"reason",v[1]==1?"valid":"producer marked racket packet invalid"}};
  if(v[1]==0) return out;
  if(v[9]<=0) throw std::runtime_error("published time-to-strike is not positive");
  if(v[10]<=0) throw std::runtime_error("strike deadline is not positive");
  out["producer_time_ns"]=stamp(v[12],v[13],"command producer stamp");
  out["flight_id"]=integer(v[15],"flight id",1);
  out["revision_id"]=integer(v[16],"revision id",1);
  out["estimator_sample_count"]=integer(v[17],"estimator sample count");
  if(v[18]<0) throw std::runtime_error("estimator span is negative");
  out["swing_sign"]=v[2]; out["position_xyz"]={v[3],v[4],v[5]}; out["velocity_xyz"]={v[6],v[7],v[8]};
  out["published_time_to_strike_s"]=v[9]; out["strike_deadline_wall_s"]=v[10]; out["estimator_span_s"]=v[18];
  return out;
}
using Names=std::map<int,std::string>;
inline const Names modes={{0,"PASSIVE"},{1,"PD_STAND"},{2,"SHADOW"},{3,"MOTION"},{4,"REFERENCE_PLAYBACK"},{5,"SERVE"},{6,"TELEOP"}};
inline const Names roles={{0,"UNASSIGNED"},{1,"SERVER"},{2,"RECEIVER"}};
inline const Names actions={{0,"NONE"},{1,"SET_SERVER"},{2,"SET_RECEIVER"},{3,"ENTER_PD_STAND"},{4,"ENTER_MOTION"},{5,"EMERGENCY_PASSIVE"},{6,"ENTER_SHADOW"},{7,"PREPARE_SERVE"},{8,"CONFIRM_BALL_LOADED"},{9,"READY_TO_SERVE"},{10,"OPEN_GRIPPER"},{12,"CONFIRM_GRIP_SECURE"},{13,"ENTER_TELEOP"}};
inline const Names results={{0,"NONE"},{1,"APPLIED"},{2,"ALREADY_SET"},{3,"ACCEPTED_PENDING"},{4,"REJECTED_WRONG_MODE"},{5,"REJECTED_RUNNER_FAULT"},{6,"REJECTED_SERVE_ACTIVE"},{7,"INVALID_REQUEST"},{8,"QUEUE_FULL"},{9,"REJECTED_SERVE_UNAVAILABLE"},{10,"REJECTED_SERVE_NOT_READY"},{11,"REJECTED_GAIN_SCALE"},{12,"REJECTED_WRONG_ROLE"},{13,"REJECTED_CLEANUP_REQUIRED"},{14,"REJECTED_GRIPPER_STATE"},{15,"REJECTED_TELEOP"}};
inline const Names reasons={{0,"NONE"},{1,"ROLE_CHANGED"},{2,"ROLE_UNCHANGED"},{3,"MODE_CHANGED"},{4,"MODE_UNCHANGED"},{5,"SERVE_ABORT_REQUESTED"},{6,"ROLE_CHANGE_REQUIRES_PASSIVE_OR_PD_STAND"},{7,"RUNNER_COMMAND_FAULT_LATCHED"},{8,"SERVE_OWNS_COMMAND"},{9,"MALFORMED_REQUEST"},{10,"ACTION_QUEUE_FULL"},{11,"SERVE_PREPARE_REQUESTED"},{12,"BALL_LOADED_CONFIRM_REQUESTED"},{13,"SERVE_CONTROLLER_UNAVAILABLE"},{14,"SERVE_PHASE_MISMATCH"},{15,"SERVE_GAIN_SCALES_MUST_BE_ONE"},{16,"SERVE_FAULT_LATCHED"},{17,"SERVER_ROLE_REQUIRED"},{18,"RECEIVER_ROLE_REQUIRED"},{19,"GRIPPER_CLEANUP_REQUIRED"},{20,"READY_TO_SERVE_PLAY_REQUESTED"},{21,"GRIPPER_OPEN_REQUESTED"},{22,"LOADING_ZONE_CLEAR_REQUESTED"},{23,"PD_STAND_REQUIRED"},{24,"GRIPPER_MUST_BE_GRABBED"},{25,"GRIP_SECURE_CONFIRM_REQUESTED"},{26,"BALL_LOAD_MERGED_INTO_PREPARE"},{27,"LOADING_ZONE_CLEAR_REMOVED"},{28,"PURE_SERVE_ONLY"},{29,"TELEOP_UNAVAILABLE"},{30,"TELEOP_INPUT_NOT_READY"},{31,"TELEOP_STOP_REQUESTED"},{34,"SERVE_RECOVERY_PENDING"},{35,"MOCAP_UNAVAILABLE"},{36,"OPERATOR_UI_UNAVAILABLE"},{37,"OPERATOR_COMPUTER_UNAVAILABLE"}};
inline const Names serves={{-1,"UNAVAILABLE"},{0,"IDLE"},{1,"PREPARING_STAND"},{2,"TRANSITION_TO_LOAD"},{3,"WAIT_BALL_LOAD"},{4,"GRIP_CLOSING"},{5,"WAIT_READY_TO_SERVE"},{6,"PLAYING_PRE_RELEASE"},{7,"RELEASE_PENDING"},{8,"STRIKE"},{9,"FOLLOW_THROUGH"},{10,"RECOVERY"},{11,"HANDOFF_READY"},{12,"COMPLETE"},{13,"ABORT_WAIT_HUMAN_CLEAR"},{14,"ABORT_RETURN"},{15,"ABORTED"},{16,"FAULT"},{17,"CLEANUP_OPENING"},{18,"WAIT_GRIP_SECURE"}};
inline const Names grippers={{-1,"UNAVAILABLE"},{0,"UNKNOWN"},{1,"OPENING"},{2,"OPEN"},{3,"CLOSING"},{4,"GRABBED"},{5,"RELEASING"},{6,"RELEASED"},{7,"FAULT"}};
inline std::string name(double value,const Names &names,const std::string &label) {
  auto code=integer(value,label,names.begin()->first,names.rbegin()->first);
  auto it=names.find(code);
  if(it==names.end()) throw std::runtime_error(fmt::format("{} has unknown code {}",label,code));
  return it->second;
}
inline bool flag(double v,const std::string &label) { return integer(v,label,0,1)!=0; }
inline J decode_runner(const std::vector<double> &v) {
  validate(v,21,"Runner state",2,false);
  bool capability=flag(v[12],"serve capability");
  auto serve=name(v[13],serves,"serve state"), gripper=name(v[14],grippers,"gripper state");
  bool cleanup=flag(v[15],"serve cleanup required");
  if(!capability && serve!="UNAVAILABLE") throw std::runtime_error("serve state must be UNAVAILABLE when serve capability is unavailable");
  if(capability && serve=="UNAVAILABLE") throw std::runtime_error("serve state cannot be UNAVAILABLE when serve capability is available");
  if(!capability && gripper!="UNAVAILABLE") throw std::runtime_error("gripper state must be UNAVAILABLE when serve capability is unavailable");
  if(!capability && cleanup) throw std::runtime_error("serve cleanup cannot be required when serve capability is unavailable");
  return {{"boot_id",integer(v[1],"Runner boot id",1)},{"state_sequence",integer(v[2],"Runner state sequence",1)},
    {"run_mode",name(v[3],modes,"Runner mode")},{"command_publishing",flag(v[4],"command publishing")},
    {"policy_native",flag(v[5],"policy native")},{"command_fault_latched",flag(v[6],"command fault")},
    {"local_role",name(v[7],roles,"local role")},{"role_epoch",integer(v[8],"role epoch")},
    {"role_change_allowed",flag(v[9],"role change allowed")},{"role_last_result",name(v[10],results,"role last result")},
    {"role_last_reason",name(v[11],reasons,"role last reason")},{"serve_capability",capability?"AVAILABLE":"UNAVAILABLE"},
    {"serve_state",serve},{"gripper_state",gripper},{"serve_cleanup_required",cleanup},
    {"last_action_id",integer(v[16],"last action id")},{"last_action",name(v[17],actions,"last action")},
    {"last_action_result",name(v[18],results,"last action result")},{"last_action_reason",name(v[19],reasons,"last action reason")},
    {"session_fingerprint",integer(v[20],"session fingerprint",1)}};
}
inline std::uint64_t fingerprint(const std::string &s) {
  std::uint64_t v=1469598103934665603ULL;
  for(unsigned char c:s) { v^=c; v*=1099511628211ULL; }
  v&=max_integer-1; return v?v:1;
}
struct Wire {
  J value;
  std::int64_t received=0;
  std::uint64_t generation=0;
  std::string error,decode_error;
  explicit Wire(const std::string &label):error("NO "+label) {}
  template<class Decoder> void update(const std::vector<double>& data,std::int64_t now,const std::string& label,Decoder decoder,bool retain=true) {
    ++generation;
    try { value=decoder(data); received=now; decode_error.clear(); error=retain&&value.value("valid",true)?"":value.at("reason").get<std::string>(); }
    catch(const std::runtime_error& e) {
      error=decode_error="MALFORMED "+label+": "+e.what();
      if(!retain) { value=nullptr; received=now; }
    }
  }
  J snapshot() const { return {{"value",value},{"received_ns",received},{"error",error},{"decode_error",decode_error},{"generation",generation}}; }
};
inline double age(std::int64_t now,std::int64_t received) { return received>0?(now-received)*1e-6:nan(); }

struct Observer {
  Wire base{"BASE PACKET"}, runner{"RUNNER STATE"}, command{"RACKET PACKET"};
  std::deque<std::int64_t> balls;
  double base_stale=.5,ball_stale=.1,runner_stale=1.;
  void ball(std::int64_t now) { balls.push_back(now); if(balls.size()>512) balls.pop_front(); }
  J view(std::int64_t now,std::int64_t ros,const J &runtime) {
    J o=J::object();
    auto put=[&](const char* k,J v){o[k]=std::move(v);};
    bool fresh=runtime.value("fresh",false);
    std::string session=runtime.value("session_id",std::string{}), attempt=runtime.value("attempt",std::string{});
    std::string runtime_status=fresh?"READY":"UNKNOWN: "+runtime.value("error",std::string("RUNTIME SNAPSHOT PENDING"));
    bool planner=fresh && runtime.value("planner_alive",false);
    J xhit=fresh?runtime.value("x_hit",J()):J();
    put("runtime_known",fresh); put("session_active",fresh&&!session.empty()); put("session_id",session);
    put("session_text",fmt::format("session={} planner_attempt={} runtime={}",session.empty()?(fresh?"NONE":"UNKNOWN"):session,attempt.empty()?"NONE":attempt,runtime_status));
    put("planner_alive",planner); put("planner_status",fresh?(planner?"ALIVE":"NOT_RUNNING"):runtime_status); put("planner_attempt",attempt);
    const J b=base.decode_error.empty()?base.value:J();
    double ba=age(now,base.received),bs=nan();
    bool bv=!b.is_null()&&b.at("valid").get<bool>(), bf=bv&&std::isfinite(ba)&&ba<=base_stale*1000;
    put("base_seen",base.received>0);put("base_valid",bv);put("base_fresh",bf);put("base_receipt_age_ms",ba);
    std::string summary=base.error; J xyz={nan(),nan(),nan()};
    if(!b.is_null()) {
      const auto &source=b.at("source_time_ns");
      if(source.is_string()) {
        // stamp() bounds this decimal integer to < 2^83. Subtract exactly,
        // then round once to double, like Python's int -> float conversion.
        // long double has different precision on Linux and Apple ARM64.
        __int128_t source_ns=0;
        for(char digit:source.get<std::string>()) source_ns=source_ns*10+(digit-'0');
        bs=static_cast<double>(static_cast<__int128_t>(ros)-source_ns)*1e-6;
      }
      else if(source.get<std::int64_t>()>0) bs=(ros-source.get<std::int64_t>())*1e-6;
      if(bv) xyz=b.at("position_xyz");
      summary=fmt::format("valid={} fresh={} seq={} receipt_age_ms={:.3f} source_age_ms={:.3f} reason={}",int(bv),int(bf),b.at("sequence").get<std::int64_t>(),ba,bs,b.at("reason").get<std::string>());
    }
    put("base_source_age_ms",bs);put("base_x",xyz[0]);put("base_y",xyz[1]);put("base_z",xyz[2]);put("base_summary",summary);
    while(balls.size()>1&&balls.front()<now-1000000000LL) balls.pop_front();
    double ball_age=age(now,balls.empty()?0:balls.back()),rate=nan();
    if(balls.size()>1&&balls.back()>balls.front()) rate=(balls.size()-1)/((balls.back()-balls.front())*1e-9);
    put("ball_live",std::isfinite(ball_age)&&ball_age<=ball_stale*1000);put("ball_receipt_age_ms",ball_age);put("ball_rate_hz",rate);
    const J &c=command.value; double ca=age(now,command.received),countdown=nan();
    bool cv=!c.is_null()&&c.at("valid").get<bool>();
    std::int64_t seq=0,flight=0,rev=0; summary=command.error;
    if(!c.is_null()) {
      seq=c.at("command_sequence");flight=c.at("flight_id");rev=c.at("revision_id");
      double deadline=c.at("strike_deadline_wall_s");if(deadline>0) countdown=deadline-ros*1e-9;
      summary=fmt::format("valid={} seq={} flight={} revision={} receipt_age_ms={:.3f} hdu_wall_countdown_s={:.4f} reason={}",int(cv),seq,flight,rev,ca,countdown,c.at("reason").get<std::string>());
    }
    put("command_seen",command.received>0);put("command_valid",cv);put("command_receipt_age_ms",ca);put("command_countdown_s",countdown);
    put("command_sequence",seq);put("command_flight",flight);put("command_revision",rev);put("command_summary",summary);
    put("x_hit_available",!xhit.is_null());put("x_hit_success",!xhit.is_null()&&xhit.at("success").get<bool>());
    put("x_hit_value",!xhit.is_null()&&!xhit.at("x_hit_m").is_null()?xhit.at("x_hit_m"):J(nan()));
    put("x_hit_status",fresh?runtime.value("x_hit_text",std::string("NO X_HIT STATUS")):runtime_status);
    const J r=runner.decode_error.empty()?runner.value:J();
    auto rv=[&](const char* key,J fallback){return r.is_null()?fallback:r.at(key);};
    double ra=age(now,runner.received);bool alive=!r.is_null()&&std::isfinite(ra)&&ra<=runner_stale*1000;
    bool match=fresh&&!r.is_null()&&!session.empty()&&r.at("session_fingerprint").get<std::uint64_t>()==fingerprint(session);
    std::string mode=rv("run_mode","UNKNOWN"), role=rv("local_role","UNASSIGNED"),serve=rv("serve_state","UNAVAILABLE");
    std::string expected=alive?(role=="SERVER"?"RECEIVER":role=="RECEIVER"?"SERVER":"UNKNOWN"):"UNKNOWN";
    put("runner_alive",alive);put("mdu_active",alive);put("mdu_text",alive?"MDU Runner state fresh":"MDU Runner state absent or stale");
    put("runner_session_id",match?session:"");put("runner_session_matches",match);put("runner_mode",mode);
    for(const auto* key:{"boot_id","state_sequence","role_epoch","last_action_id"}) put((std::string("runner_")+key).c_str(),rv(key,0));
    for(const auto* key:{"command_publishing","policy_native","role_change_allowed","serve_cleanup_required"}) put((std::string("runner_")+key).c_str(),rv(key,false));
    put("runner_command_fault",rv("command_fault_latched",false));put("runner_local_role",role);
    for(const auto* key:{"role_last_result","last_action","last_action_result","last_action_reason"}) put((std::string("runner_")+key).c_str(),rv(key,"NONE"));
    for(const auto* key:{"serve_capability","serve_state","gripper_state"}) put((std::string("runner_")+key).c_str(),rv(key,"UNAVAILABLE"));
    put("runner_standing",alive&&mode=="PD_STAND");put("runner_ready",alive&&mode=="MOTION");
    put("runner_ready_to_serve",alive&&mode=="SERVE"&&serve=="WAIT_READY_TO_SERVE");
    put("runner_serving",alive&&mode=="SERVE"&&(serve=="PLAYING_PRE_RELEASE"||serve=="RELEASE_PENDING"||serve=="STRIKE"||serve=="FOLLOW_THROUGH"||serve=="RECOVERY"));
    summary=runner.error;
    if(!r.is_null()) summary=fmt::format("alive={} receipt_age_ms={:.3f} boot={} seq={} session_matches={} mode={} publishing={} fault={} local_role={} source=RUNNER_CONFIRMED role_epoch={} serve={}/{} gripper={} cleanup_required={} last_action={} result={} reason={}",int(alive),ra,r.at("boot_id").get<std::int64_t>(),r.at("state_sequence").get<std::int64_t>(),int(match),mode,int(r.at("command_publishing").get<bool>()),int(r.at("command_fault_latched").get<bool>()),role,r.at("role_epoch").get<std::int64_t>(),r.at("serve_capability").get<std::string>(),serve,r.at("gripper_state").get<std::string>(),int(r.at("serve_cleanup_required").get<bool>()),r.at("last_action").get<std::string>(),r.at("last_action_result").get<std::string>(),r.at("last_action_reason").get<std::string>());
    put("runner_summary",summary);put("opponent_expected_role",expected);put("opponent_role_source","INFERRED_FROM_LOCAL_ROLE");put("opponent_role_confirmed",false);
    put("opponent_summary","expected_role="+expected+" source=INFERRED_FROM_LOCAL_ROLE confirmed=0");put("observer_alive",true);
    return o;
  }
};
// Same replay/invalid-sample semantics as hope_player_positions.PlayerPositions.
struct Players {
  struct Sample {double x,received;std::int64_t stamp;};
  std::map<std::string,Sample> samples;
  void update(const std::string &name,const std::vector<double>& p,const std::vector<double>& q,
              std::int64_t stamp,const std::string& frame,double now) {
    if(name!="UCB_P1"&&name!="UCB_P2")return;
    bool valid=frame=="hope_table_global"&&stamp>0&&p.size()==3&&q.size()==4&&std::isfinite(now);
    double norm=0;for(auto x:p)valid=valid&&std::isfinite(x);
    for(auto x:q){valid=valid&&std::isfinite(x);norm+=x*x;}
    norm=std::sqrt(norm);valid=valid&&norm>=.5&&norm<=1.5;
    if(!valid){samples.erase(name);return;}
    auto it=samples.find(name);if(it!=samples.end()&&stamp<=it->second.stamp)return;
    samples[name]={p[0],now,stamp};
  }
  J status(double now)const {
    J live=J::object();bool p1=false,p2=false;
    for(const auto &[name,s]:samples)if(now-s.received>=0&&now-s.received<=.25){live[name]=s.x;p1=p1||s.x<0;p2=p2||s.x>2.74;}
    return {{"frame","hope_table_global"},{"table_length_m",2.74},{"max_age_s",.25},{"players_x_m",live},{"P1",p1},{"P2",p2}};
  }
};
} // namespace hope_operations
