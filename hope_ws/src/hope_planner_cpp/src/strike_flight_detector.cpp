#include "hope_planner_cpp/strike_flight_detector.hpp"
#include <cmath>
#include <stdexcept>
namespace hope_planner_cpp {
StrikeFlightDetector::StrikeFlightDetector(StrikeFlightDetectorConfig c):config_(c) {
  if(c.net_x!=1.37 || !std::isfinite(c.commit_delay_s) || c.commit_delay_s<0 || c.commit_delay_s>.2)
    throw std::invalid_argument("invalid fixed net/timing registration");
}
void StrikeFlightDetector::reset_phase() {
  points_.clear();last_=start_=net_=0;negative_=positive_=0;active_=crossed_=false;
}
StrikeFlightUpdate StrikeFlightDetector::observe(const BallSample& sample) {
  StrikeFlightUpdate out;
  const auto ns=sample.source_time_ns;const double x=sample.position.x();
  if(ns<=0 || !std::isfinite(x))return out;
  if(last_ && ns<=last_) {out.source_epoch_reset=ns<last_-1000000;return out;}
  if(last_ && ns-last_>250000000) {out.source_epoch_reset=true;return out;}
  // Preserve straddle before direction confirmation. Integer endpoints never
  // pass through epoch-valued double seconds; long double computes only offset.
  if(!points_.empty() && !net_ && points_.back().x>config_.net_x && x<=config_.net_x) {
    const auto& a=points_.back();
    const long double f=(a.x-config_.net_x)/(a.x-x);
    net_=a.ns+static_cast<std::int64_t>(std::llround(f*(ns-a.ns)));
  }
  points_.push_back({x,ns});last_=ns;
  while(points_.size()>64 || (points_.size()>2 && ns-points_.front().ns>45000000))points_.pop_front();
  if(points_.size()<2)return out;
  const auto& a=points_.front();
  const double speed=(x-a.x)/(static_cast<double>(ns-a.ns)*1e-9);
  negative_=speed<-.25 ? negative_+1 : 0;
  positive_=speed>.25 && x>config_.net_x+.03 ? positive_+1 : 0;
  if(active_ && positive_>=2) {
    active_=crossed_=false;net_=0;negative_=0;
    // Keep the current point only: an old incoming direction cannot rearm.
    points_.clear();points_.push_back({x,ns});return out;
  }
  if(!active_ && negative_>=2) {
    active_=true;crossed_=false;start_=a.ns;
    out.incoming_started=true;out.segment_start_ns=start_;
    if(!net_ && x<=config_.net_x) {
      const long double offset=(config_.net_x-x)/speed*1e9L;
      if(std::isfinite(offset) && offset>=-static_cast<long double>(ns)) {
        net_=ns+static_cast<std::int64_t>(std::llround(offset));out.late_acquisition=true;
      }
    }
  }
  if(active_ && !crossed_ && net_>0) {
    crossed_=true;out.net_crossed=true;out.net_cross_ns=net_;
  }
  return out;
}
} // namespace hope_planner_cpp
