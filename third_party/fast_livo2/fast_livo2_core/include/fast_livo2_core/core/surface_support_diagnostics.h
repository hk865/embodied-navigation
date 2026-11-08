#pragma once
// Support snapshots travel through the existing bounded asynchronous logger.
// ROI/time/budget conditions affect recording only; never association or fitting.
#include "diagnostics.h"
#include "surface_validity.h"
#include <array>
#include <sstream>

namespace fastlivo_surface_diag {
struct Config {
  std::vector<std::array<double,2>> windows;
  std::array<std::array<double,2>,3> bounds;
  uint64_t maximum_bytes=64ULL*1024*1024;
};
inline std::vector<std::array<double,2>> pairs(const char *text,size_t maximum) {
  if (!text || !*text || text[std::strlen(text)-1]==',')
    throw std::invalid_argument("Invalid surface diagnostic interval text");
  std::vector<std::array<double,2>> result;
  std::stringstream stream(text);std::string item;
  while(std::getline(stream,item,',')) {
    auto split=item.find(':');
    if(split==std::string::npos) throw std::invalid_argument("Surface diagnostic requires lo:hi pairs");
    size_t a=0,b=0;double lo=std::stod(item.substr(0,split),&a),hi=std::stod(item.substr(split+1),&b);
    if(a!=split || b!=item.size()-split-1 || !std::isfinite(lo) || !std::isfinite(hi) || lo>hi)
      throw std::invalid_argument("Invalid surface diagnostic interval");
    result.push_back({lo,hi});
  }
  if(result.empty() || result.size()>maximum)throw std::invalid_argument("Invalid surface diagnostic interval count");
  return result;
}
inline const Config &config() {
  static const Config cfg=[] {
    Config c;
    const char *windows=std::getenv("FASTLIVO_SURFACE_DIAG_WINDOWS");
    if(windows && *windows) {
      c.windows=pairs(windows,8);
      for(size_t i=0;i<c.windows.size();++i)
        if(c.windows[i][0]<0 || (i && c.windows[i][0]<=c.windows[i-1][1]))
          throw std::invalid_argument("Surface diagnostic windows must be ordered, disjoint and nonnegative");
      const char *bounds=std::getenv("FASTLIVO_SURFACE_DIAG_BOUNDS");
      if(!bounds || !*bounds)throw std::invalid_argument("Surface support diagnostics require explicit XYZ bounds");
      auto v=pairs(bounds,3);
      if(v.size()!=3)throw std::invalid_argument("Surface diagnostic requires three XYZ bounds");
      for(size_t i=0;i<3;++i)c.bounds[i]=v[i];
    }
    if(const char *cap=std::getenv("FASTLIVO_SURFACE_DIAG_MAX_BYTES")) {
      size_t used=0;c.maximum_bytes=std::stoull(cap,&used);
      if(used!=std::strlen(cap) || c.maximum_bytes<1024 || c.maximum_bytes>256ULL*1024*1024)
        throw std::invalid_argument("Surface support diagnostic budget outside 1 KiB..256 MiB");
    }
    return c;
  }();
  return cfg;
}
inline bool in_window(uint64_t ns) {
  const double t=ns*1e-9;
  for(const auto &w:config().windows)if(t>=w[0]&&t<=w[1])return true;
  return false;
}
inline bool selected(const VoxelOctoTree &tree) {
  auto &log=fastlivo_diag::Logger::instance();
  if(!log.enabled() || !in_window(log.context().stamp_ns))return false;
  for(int i=0;i<3;++i)
    if(tree.voxel_center_[i]<config().bounds[i][0] || tree.voxel_center_[i]>config().bounds[i][1])return false;
  return true;
}
constexpr size_t maximum_points_per_snapshot=128;
constexpr size_t maximum_snapshots_per_phase=256;
struct Counters {
  uint64_t bytes=0,attempted=0,submitted=0,point_cap_omitted=0,phase_cap_omitted=0,budget_omitted=0;
  uint64_t phase_sequence=0,phase_count=0;
};
inline Counters &counters() {static thread_local Counters c;return c;}
inline void snapshot(const VoxelOctoTree &tree,const std::vector<pointWithVar> &points,
                     const Eigen::Vector3d &fit_normal,const Eigen::Vector3d &eigenvalues,
                     bool legacy_planar,const fastlivo_surface::Quality &quality) {
  if(!selected(tree))return;
  auto &log=fastlivo_diag::Logger::instance();auto context=log.context();auto &c=counters();
  ++c.attempted;
  if(c.phase_sequence!=context.sequence){c.phase_sequence=context.sequence;c.phase_count=0;}
  if(points.size()>maximum_points_per_snapshot){++c.point_cap_omitted;return;}
  if(c.phase_count>=maximum_snapshots_per_phase){++c.phase_cap_omitted;return;}
  const auto &p=*tree.plane_ptr_;
  std::vector<double> v{1,0,double(fastlivo_surface::enabled()),double(points.size()),14,
    double(tree.layer_),tree.voxel_center_[0],tree.voxel_center_[1],tree.voxel_center_[2],tree.quater_length_,
    double(p.id_),double(p.is_init_),double(p.is_plane_),double(legacy_planar),double(quality.accepted),double(quality.reason),
    double(p.diag_birth_sequence_),double(p.diag_birth_stamp_ns_),double(p.diag_update_sequence_),double(p.diag_update_stamp_ns_),
    double(p.diag_update_count_),double(tree.update_enable_),quality.normalized_rss_per_dof,quality.mid_max_ratio,
    quality.normal_variance_mean,quality.residual_rms,fastlivo_surface::maximum_normalized_rss_per_dof,
    fastlivo_surface::minimum_mid_max_ratio,fastlivo_surface::normal_variance_floor_m2};
  fastlivo_diag::append(v,p.center_);fastlivo_diag::append(v,p.covariance_);
  fastlivo_diag::append(v,fit_normal);fastlivo_diag::append(v,eigenvalues);
  fastlivo_diag::append(v,p.plane_var_);
  // Prefix length 83, rows are exact inserted XYZ+covariance and source identity.
  if(v.size()!=83)throw std::logic_error("Surface support prefix differs from frozen schema");
  const uint64_t bytes=7*8+(v.size()+points.size()*14)*8;
  if(c.bytes+bytes>config().maximum_bytes){++c.budget_omitted;return;}
  v.reserve(v.size()+points.size()*14);
  for(const auto &point:points) {
    v.push_back(double(point.surface_diag_sequence));v.push_back(double(point.surface_diag_stamp_ns));
    fastlivo_diag::append(v,point.point_w);fastlivo_diag::append(v,point.var);
  }
  ++c.submitted;++c.phase_count;c.bytes+=bytes;
  log.emit(104,-1,int(tree.layer_),std::move(v));
}
inline void summary(int operation) {
  auto &log=fastlivo_diag::Logger::instance();
  if(!log.enabled() || !in_window(log.context().stamp_ns))return;
  const auto &c=counters();
  log.emit(105,-1,-1,{1,double(operation),double(fastlivo_surface::enabled()),double(c.attempted),double(c.submitted),
    double(c.point_cap_omitted),double(c.phase_cap_omitted),double(c.budget_omitted),double(c.bytes),double(config().maximum_bytes),
    double(maximum_points_per_snapshot),double(maximum_snapshots_per_phase)});
}
} // namespace fastlivo_surface_diag
