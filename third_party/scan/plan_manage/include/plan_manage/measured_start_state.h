#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace scan_planner {
// Timestamp-based measured world velocity. Never inherit unexecuted spline
// derivatives after a turn/stop, and never change the measured start position.
class MeasuredStartState {
public:
  double max_speed{.12};
  double time_constant{.4};
  void update(const Eigen::Vector3d &velocity, int64_t stamp, bool frozen) {
    if (!velocity.allFinite() || stamp <= stamp_) return;
    Eigen::Vector3d observed = velocity;
    if (observed.norm() > max_speed) observed *= max_speed/observed.norm();
    const double dt = stamp_ < 0 ? 0. : (stamp-stamp_)*1e-9;
    stamp_ = stamp;
    if (frozen) { filtered_.setZero(); ready_=true; return; }
    if (!ready_) filtered_=observed;
    else filtered_ += (-std::expm1(-dt/time_constant))*(observed-filtered_);
    ready_=true;
  }
  Eigen::Vector3d velocity(bool frozen) const {
    return frozen ? Eigen::Vector3d::Zero() : filtered_;
  }
  void stop() { filtered_.setZero(); }
private:
  Eigen::Vector3d filtered_{Eigen::Vector3d::Zero()};
  int64_t stamp_{-1};
  bool ready_{false};
};
} // namespace scan_planner
