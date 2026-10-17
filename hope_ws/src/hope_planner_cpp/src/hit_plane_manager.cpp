#include "hope_planner_cpp/hit_plane_manager.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace hope_planner_cpp {
void HitPlaneSettings::validate() const {
  for(double v:{offset_m,window_s,max_age_s,max_span_m,min_x_m,max_x_m})
    if(!std::isfinite(v)) throw std::invalid_argument("nonfinite plane settings");
  if(offset_m<=0 || offset_m>2 || window_s<=0 || window_s>5 || max_age_s<=0 ||
     max_age_s>window_s || max_span_m<=0 || max_span_m>.1 || min_samples<2 ||
     min_samples>4096 || min_x_m>=max_x_m || min_x_m<-.3 || max_x_m>.3)
    throw std::invalid_argument("unsupported plane settings");
}
HitPlaneManager::HitPlaneManager(HitPlaneSettings s,std::string frame,
    std::string registration,std::uint64_t world):settings_(s),world_id_(world) {
  s.validate(); if(frame.empty()) throw std::invalid_argument("plane frame required");
  frozen_.frame_id=std::move(frame); frozen_.registration_id=std::move(registration);
  frozen_.offset_m=s.offset_m;
}
bool HitPlaneManager::observe(const std::vector<double>& w,std::int64_t receipt,
    std::int64_t now,bool clock) {
  auto fail=[&](const char* why){reason_=why;samples_.clear();return false;};
  if(!clock) return fail("base_clock_unregistered");
  if(frozen_.registration_id.empty() || !world_id_) return fail("base_frame_unregistered");
  if(w.size()!=16 || !std::all_of(w.begin(),w.end(),[](double v){return std::isfinite(v);}) ||
     w[0]!=2 || w[1]!=1) return fail("invalid_base_schema");
  auto integer=[](double v,double max){return v>=0 && v<=max && std::floor(v)==v;};
  constexpr double exact=4503599627370496.;
  if(!integer(w[2],exact) || !integer(w[3],2147483647.) || w[3]<=0 ||
     !integer(w[4],999999999.) || !integer(w[13],1048576.) ||
     !integer(w[14],exact) || w[14]<=0 || !integer(w[15],exact) || w[15]<=0)
    return fail("invalid_base_identity");
  const auto flags=static_cast<unsigned>(w[13]);
  if((flags&47u)!=47u || w[12]<0 || w[12]>1) return fail("invalid_base_flags");
  if(static_cast<std::uint64_t>(w[15])!=world_id_) return fail("base_world_frame_mismatch");
  const double q=std::sqrt(w[8]*w[8]+w[9]*w[9]+w[10]*w[10]+w[11]*w[11]);
  if(!std::isfinite(q) || q<.5 || q>1.5) return fail("invalid_base_quaternion");
  const auto source=static_cast<std::int64_t>(w[3])*1000000000LL+static_cast<std::int64_t>(w[4]);
  const long double age=(static_cast<long double>(now)-source)*1e-9L;
  if(receipt<=0 || age<-.001 || age>settings_.max_age_s) return fail("stale_or_future_base");
  const auto seq=static_cast<std::uint64_t>(w[2]);
  if(last_source_ && (source<=last_source_ || seq<=last_sequence_)) return fail("reordered_base");
  last_source_=source;last_sequence_=seq;
  const auto calibration=static_cast<std::uint64_t>(w[14]);
  if(!samples_.empty() && samples_.back().calibration!=calibration) samples_.clear();
  samples_.push_back({w[5],receipt,source,calibration});
  while(!samples_.empty() && (receipt-samples_.front().receipt>settings_.window_s*1e9 || samples_.size()>4096)) samples_.pop_front();
  reason_="base_samples_available";return true;
}
bool HitPlaneManager::freeze(std::int64_t now,std::int64_t source,
    std::uint64_t rally,bool safe) {
  auto fail=[&](const char* why){reason_=why;return false;};
  if(!safe) return fail("refresh_forbidden_active_rally");
  while(!samples_.empty() && now-samples_.front().receipt>settings_.window_s*1e9) samples_.pop_front();
  if(samples_.size()<settings_.min_samples) return fail("insufficient_base_samples");
  if(now<samples_.back().receipt || now-samples_.back().receipt>settings_.max_age_s*1e9 ||
      static_cast<long double>(source)-samples_.back().source>settings_.max_age_s*1e9L ||
      static_cast<long double>(source)-samples_.back().source < -1000000.L)
    return fail("stale_base_samples");
  std::vector<double> xs;for(const auto&s:samples_) xs.push_back(s.x);
  std::sort(xs.begin(),xs.end());
  if(xs.back()-xs.front()>settings_.max_span_m) return fail("moving_base");
  const auto n=xs.size();const double base=(xs[(n-1)/2]+xs[n/2])*.5;
  const double x=base+settings_.offset_m;
  if(!std::isfinite(x) || x<settings_.min_x_m || x>settings_.max_x_m) return fail("plane_out_of_range");
  frozen_.base_x_m=base;frozen_.x_hit_m=x;frozen_.calibrated=true;frozen_.test_plane=false;
  frozen_.source_ns=samples_.back().source;frozen_.receipt_ns=samples_.back().receipt;
  frozen_.calibration_id=samples_.back().calibration;frozen_.world_frame_id=world_id_;
  frozen_.activated_source_ns=source;frozen_.rally_id=rally;++frozen_.generation;
  frozen_.reason=reason_="calibrated";return true;
}
void HitPlaneManager::use_test_plane(double x,std::int64_t source) {
  if(!std::isfinite(x) || x<settings_.min_x_m || x>settings_.max_x_m)
    throw std::invalid_argument("test plane out of range");
  frozen_.x_hit_m=x;frozen_.base_x_m=x-settings_.offset_m;frozen_.test_plane=true;
  frozen_.calibrated=true;frozen_.activated_source_ns=source;++frozen_.generation;
  frozen_.reason=reason_="explicit_known_test_plane";
}
void HitPlaneManager::revoke(const std::string& reason) {
  samples_.clear();last_source_=0;last_sequence_=0;frozen_.calibrated=false;
  frozen_.reason=reason_=reason;
}
} // namespace hope_planner_cpp
