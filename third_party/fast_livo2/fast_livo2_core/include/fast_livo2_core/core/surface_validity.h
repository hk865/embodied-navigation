#pragma once
// V32 candidate: no simulator, floor label, pose reset or measurement-gate change.
// The normalized fit statistic is an engineering gate, not calibrated chi-square.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace fastlivo_surface {
inline bool enabled() {
  static const bool value = [] {
    const char *p = std::getenv("FASTLIVO_SURFACE_VALIDITY");
    if (!p || std::strcmp(p,"0")==0) return false;
    if (std::strcmp(p,"1")==0) return true;
    throw std::invalid_argument("FASTLIVO_SURFACE_VALIDITY must be exactly 0 or 1");
  }();
  return value;
}
constexpr size_t minimum_points = 6;
constexpr double minimum_mid_max_ratio = 0.05;
constexpr double maximum_normalized_rss_per_dof = 4.0;
constexpr double normal_variance_floor_m2 = 1e-8;
struct Quality {
  // 0 accepted, 1 insufficient support, 2 nonfinite/invalid covariance,
  // 3 one-dimensional support, 4 excess normalized residual energy.
  int reason = 1;
  bool accepted = false;
  double normalized_rss_per_dof = std::numeric_limits<double>::quiet_NaN();
  double mid_max_ratio = std::numeric_limits<double>::quiet_NaN();
  double normal_variance_mean = std::numeric_limits<double>::quiet_NaN();
  double residual_rms = std::numeric_limits<double>::quiet_NaN();
};
template<class Points>
Quality assess(const Points &points, const Eigen::Vector3d &center,
               const Eigen::Vector3d &normal, double mid, double maximum) {
  Quality q;
  if (points.size()<minimum_points) return q;
  q.reason=2;
  if (!center.allFinite() || !normal.allFinite() ||
      std::abs(normal.norm()-1.0)>1e-6 || !std::isfinite(mid) ||
      !std::isfinite(maximum) || mid<0.0 || maximum<=0.0) return q;
  q.mid_max_ratio=mid/maximum;
  double normalized=0.0, squared=0.0, variance_sum=0.0;
  for (const auto &p:points) {
    if (!p.point_w.allFinite() || !p.var.allFinite()) return q;
    const double tolerance=1e-10*std::max(1.0,p.var.norm());
    if ((p.var-p.var.transpose()).cwiseAbs().maxCoeff()>tolerance) return q;
    const double variance=normal.dot(p.var*normal);
    if (!std::isfinite(variance) || variance<0.0) return q;
    const double residual=normal.dot(p.point_w-center);
    normalized+=residual*residual/std::max(variance,normal_variance_floor_m2);
    squared+=residual*residual;
    variance_sum+=variance;
  }
  q.normalized_rss_per_dof=normalized/double(points.size()-3);
  q.residual_rms=std::sqrt(squared/double(points.size()));
  q.normal_variance_mean=variance_sum/double(points.size());
  if (!std::isfinite(q.normalized_rss_per_dof)) return q;
  if (q.mid_max_ratio<minimum_mid_max_ratio) {q.reason=3;return q;}
  if (q.normalized_rss_per_dof>maximum_normalized_rss_per_dof) {q.reason=4;return q;}
  q.accepted=true;q.reason=0;return q;
}
} // namespace fastlivo_surface
