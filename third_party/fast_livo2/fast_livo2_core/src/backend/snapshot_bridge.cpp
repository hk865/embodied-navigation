#include "fast_livo2_core/backend/snapshot_bridge.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace fastlivo_backend {
const char* stage_name(FinalStage stage) noexcept {
  switch(stage) { case FinalStage::Livo:return "livo";
    case FinalStage::LioVioSkipped:return "lio_vio_skipped"; default:return "lio"; }
}
SnapshotBridge::SnapshotBridge(BridgeOptions options):options_(std::move(options)) {
  if(options_.spool_directory.empty() || !options_.queue_capacity ||
     !options_.max_points || !options_.max_ready_directories || !options_.epoch ||
     options_.queue_capacity>64 || options_.max_points>65536 ||
     options_.max_ready_directories>256 || !std::isfinite(options_.min_translation_m) ||
     !std::isfinite(options_.min_rotation_rad) ||
     options_.min_translation_m<0 || options_.min_rotation_rad<0 ||
     options_.max_interval_ns<options_.min_interval_ns)
    throw std::invalid_argument("invalid snapshot bridge options");
  std::filesystem::create_directories(options_.spool_directory);
  pool_size_=options_.queue_capacity+2;  // queued, one worker, one paired LIO
  pool_=std::make_unique<Slot[]>(pool_size_);
  for(size_t i=0;i<pool_size_;++i) pool_[i].snapshot.points_lidar.resize(options_.max_points);
  queue_.resize(options_.queue_capacity+1,nullptr);
  worker_=std::thread(&SnapshotBridge::work,this);
}
SnapshotBridge::~SnapshotBridge() { close(); }
bool SnapshotBridge::selected(uint64_t stamp_ns,const Transform& pose) const noexcept {
  if(!last_selected_ns_)return true;
  if(stamp_ns-last_selected_ns_<options_.min_interval_ns)return false;
  if(stamp_ns-last_selected_ns_>=options_.max_interval_ns)return true;
  double translation2=0,trace=0;
  for(size_t r=0;r<3;++r) {
    const double d=pose[4*r+3]-last_selected_pose_[4*r+3];translation2+=d*d;
    for(size_t c=0;c<3;++c)trace+=pose[4*r+c]*last_selected_pose_[4*r+c];
  }
  const double cosine=std::max(-1.0,std::min(1.0,(trace-1)*0.5));
  return translation2>=options_.min_translation_m*options_.min_translation_m ||
    std::acos(cosine)>=options_.min_rotation_rad;
}
bool SnapshotBridge::queue_full() const noexcept {
  const size_t next=(head_.load(std::memory_order_relaxed)+1)%queue_.size();
  return next==tail_.load(std::memory_order_acquire);
}
SnapshotBridge::Slot* SnapshotBridge::acquire_slot() noexcept {
  for(size_t i=0;i<pool_size_;++i) {
    bool free=false;
    if(pool_[i].owned.compare_exchange_strong(free,true,std::memory_order_acquire,
                                           std::memory_order_relaxed)) return &pool_[i];
  }
  return nullptr;
}
void SnapshotBridge::finish_pending(FinalStage stage) noexcept {
  if(!pending_)return;
  pending_->snapshot.final_stage=stage;
  const size_t head=head_.load(std::memory_order_relaxed);
  const size_t next=(head+1)%queue_.size();
  if(next==tail_.load(std::memory_order_acquire)) {
    queue_dropped_.fetch_add(1,std::memory_order_relaxed);
    pending_->owned.store(false,std::memory_order_release);
  } else {
    queue_[head]=pending_;head_.store(next,std::memory_order_release);
    submitted_.fetch_add(1,std::memory_order_relaxed);
  }
  pending_=nullptr;
}
bool SnapshotBridge::on_vio(uint64_t stamp_ns,bool updated,const Transform& pose) {
  if(stopped_.load(std::memory_order_relaxed))return false;
  if(!pending_ || pending_->snapshot.stamp_ns!=stamp_ns) {
    unmatched_vio_.fetch_add(1,std::memory_order_relaxed);return false;
  }
  if(updated){pending_->snapshot.T_odom_body=pose;last_selected_pose_=pose;}
  finish_pending(updated?FinalStage::Livo:FinalStage::LioVioSkipped);
  return true;
}
void SnapshotBridge::close() {
  if(stopped_.load(std::memory_order_relaxed)) {if(worker_.joinable())worker_.join();return;}
  finish_pending(FinalStage::LioVioSkipped);
  stopped_.store(true,std::memory_order_release);
  if(worker_.joinable())worker_.join();
}
void SnapshotBridge::work() {
  auto next_status=std::chrono::steady_clock::now();
  for(;;) {
    const auto now=std::chrono::steady_clock::now();
    if(now>=next_status){write_status();next_status=now+std::chrono::seconds(1);}
    const size_t tail=tail_.load(std::memory_order_relaxed);
    if(tail==head_.load(std::memory_order_acquire)) {
      if(stopped_.load(std::memory_order_acquire))break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;
    }
    Slot* slot=queue_[tail];
    tail_.store((tail+1)%queue_.size(),std::memory_order_release);
    try {if(write_snapshot(slot->snapshot))written_.fetch_add(1,std::memory_order_relaxed);}
    catch(...) {io_failed_.fetch_add(1,std::memory_order_relaxed);}
    slot->owned.store(false,std::memory_order_release);
  }
  write_status();
}
namespace {
void matrix(std::ostream& out,const Transform& t) {
  out<<'[';
  for(size_t r=0;r<4;++r){if(r)out<<',';out<<'[';
    for(size_t c=0;c<4;++c){if(c)out<<',';out<<t[4*r+c];}out<<']';}
  out<<']';
}
}
bool SnapshotBridge::write_snapshot(const Snapshot& snapshot) {
  size_t ready=0;
  for(const auto& entry:std::filesystem::directory_iterator(options_.spool_directory)) {
    // A consumer may rename a ready directory while this worker enumerates.
    std::error_code error;
    if(entry.path().filename().string().rfind("kf_",0)==0 && entry.is_directory(error) && !error)++ready;
  }
  if(ready>=options_.max_ready_directories) {
    spool_dropped_.fetch_add(1,std::memory_order_relaxed);return false;
  }
  const std::string suffix=std::to_string(snapshot.epoch)+"_"+std::to_string(snapshot.sequence);
  const auto temporary=options_.spool_directory/("tmp_"+suffix);
  const auto ready_path=options_.spool_directory/("kf_"+suffix);
  if(std::filesystem::exists(temporary) || std::filesystem::exists(ready_path))
    throw std::runtime_error("snapshot output already exists; use a new session directory");
  std::filesystem::create_directory(temporary);
  try {
    std::ofstream cloud(temporary/"points_lidar.pcd");
    cloud.exceptions(std::ios::failbit|std::ios::badbit);
    cloud<<"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
      <<snapshot.point_count<<"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
      <<snapshot.point_count<<"\nDATA ascii\n"<<std::setprecision(9);
    for(size_t i=0;i<snapshot.point_count;++i) {
      const auto& p=snapshot.points_lidar[i];cloud<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
    }
    cloud.close();
    std::ofstream metadata(temporary/"metadata.json");
    metadata.exceptions(std::ios::failbit|std::ios::badbit);
    metadata<<std::setprecision(17)<<"{\"schema\":\"livo_keyframe/v1\",\"epoch\":"<<snapshot.epoch
      <<",\"sequence\":"<<snapshot.sequence<<",\"stamp_ns\":"<<snapshot.stamp_ns
      <<",\"T_odom_body\":";matrix(metadata,snapshot.T_odom_body);
    metadata<<",\"T_body_lidar\":";matrix(metadata,snapshot.T_body_lidar);
    metadata<<",\"final_stage\":\""<<stage_name(snapshot.final_stage)<<"\",\"point_count\":"
      <<snapshot.point_count<<",\"source_point_count\":"<<snapshot.source_point_count
      <<",\"copy_ns\":"<<snapshot.copy_ns<<",\"cloud_frame\":\"lidar\"}\n";
    metadata.close();
    std::filesystem::rename(temporary,ready_path);
  } catch(...) {
    std::error_code ignored;std::filesystem::remove_all(temporary,ignored);throw;
  }
  return true;
}
void SnapshotBridge::write_status() noexcept {
  try {
    const auto snapshot=stats();
    const auto temporary=options_.spool_directory/"bridge_status.tmp";
    std::ofstream out(temporary);out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<"{\"schema\":\"livo_snapshot_bridge_status/v1\",\"epoch\":"<<options_.epoch
      <<",\"owner_stopped\":"<<(stopped_.load()?"true":"false")
      <<",\"candidates\":"<<snapshot.candidates<<",\"submitted\":"<<snapshot.submitted
      <<",\"written\":"<<snapshot.written<<",\"queue_dropped\":"<<snapshot.queue_dropped
      <<",\"pool_dropped\":"<<snapshot.pool_dropped<<",\"spool_dropped\":"<<snapshot.spool_dropped
      <<",\"io_failed\":"<<snapshot.io_failed<<",\"duplicates\":"<<snapshot.duplicates
      <<",\"unmatched_vio\":"<<snapshot.unmatched_vio<<",\"copied_points\":"<<snapshot.copied_points
      <<",\"copy_ns\":"<<snapshot.copy_ns<<",\"max_copy_ns\":"<<snapshot.max_copy_ns
      <<",\"last_measurement_ns\":"<<snapshot.last_measurement_ns
      <<",\"status_failed\":"<<snapshot.status_failed<<"}\n";
    out.close();std::filesystem::rename(temporary,options_.spool_directory/"bridge_status.json");
  } catch(...) {status_failed_.fetch_add(1,std::memory_order_relaxed);}
}
BridgeStats SnapshotBridge::stats() const noexcept {
  return {candidates_.load(),submitted_.load(),written_.load(),queue_dropped_.load(),
    pool_dropped_.load(),spool_dropped_.load(),io_failed_.load(),duplicates_.load(),
    unmatched_vio_.load(),copied_points_.load(),copy_ns_.load(),max_copy_ns_.load(),
    last_measurement_ns_.load(),status_failed_.load()};
}
}  // namespace fastlivo_backend
