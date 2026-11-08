#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>

namespace fastlivo_backend {
static_assert(std::atomic<size_t>::is_always_lock_free &&
              std::atomic<uint64_t>::is_always_lock_free &&
              std::atomic<bool>::is_always_lock_free,
              "snapshot bridge requires lock-free scalar atomics");

// Row-major T_AB maps coordinates in B into A. Body is the estimator's IMU.
using Transform = std::array<double, 16>;
inline Transform identity_transform() {
  return {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
}
struct PointXYZ { float x=0, y=0, z=0; };
enum class FinalStage { Lio, Livo, LioVioSkipped };
const char* stage_name(FinalStage stage) noexcept;

struct Snapshot {
  uint64_t epoch=0, sequence=0, stamp_ns=0;
  Transform T_odom_body=identity_transform(), T_body_lidar=identity_transform();
  FinalStage final_stage=FinalStage::Lio;
  size_t point_count=0, source_point_count=0;
  uint64_t copy_ns=0;
  // Sized once at startup. Only [0, point_count) contains this keyframe's data.
  std::vector<PointXYZ> points_lidar;
};
struct BridgeOptions {
  std::filesystem::path spool_directory;
  uint64_t epoch=1;
  size_t queue_capacity=4, max_points=4096, max_ready_directories=16;
  uint64_t min_interval_ns=1'000'000'000ULL, max_interval_ns=2'000'000'000ULL;
  double min_translation_m=0.8, min_rotation_rad=0.2;
};
struct BridgeStats {
  uint64_t candidates=0, submitted=0, written=0, queue_dropped=0,
    pool_dropped=0, spool_dropped=0, io_failed=0, duplicates=0,
    unmatched_vio=0, copied_points=0, copy_ns=0, max_copy_ns=0,
    last_measurement_ns=0, status_failed=0;
};

// One estimator owner calls on_lio/on_vio; exactly one worker consumes.
// Startup allocates all slots/cloud storage. No locks, waits, I/O, JSON, full-
// cloud checks, or worker work occur in these owner calls. Selected clouds are
// uniformly capped for the BACKEND ONLY; frontend clouds/maps are untouched.
// A successful LIO in LIVO mode retains one owned slot until the matching VIO
// returns. A skipped VIO publishes the LIO pose once, never a second graph node.
class SnapshotBridge {
 public:
  explicit SnapshotBridge(BridgeOptions options);
  ~SnapshotBridge();
  SnapshotBridge(const SnapshotBridge&)=delete;
  SnapshotBridge& operator=(const SnapshotBridge&)=delete;

  template<class PointReader>
  bool on_lio(uint64_t stamp_ns, const Transform& T_odom_body,
              const Transform& T_body_lidar, bool expect_vio,
              size_t source_point_count, PointReader&& reader) {
    if (stopped_.load(std::memory_order_relaxed)) return false;
    if (!stamp_ns || stamp_ns<=last_seen_ns_) {
      duplicates_.fetch_add(1,std::memory_order_relaxed); return false;
    }
    if (pending_) finish_pending(FinalStage::LioVioSkipped);
    last_seen_ns_=stamp_ns;
    last_measurement_ns_.store(stamp_ns,std::memory_order_relaxed);
    if (!source_point_count || !selected(stamp_ns,T_odom_body)) return false;
    candidates_.fetch_add(1,std::memory_order_relaxed);
    // Admission is checked before touching point payloads.
    if (queue_full()) { queue_dropped_.fetch_add(1,std::memory_order_relaxed); return false; }
    Slot* slot=acquire_slot();
    if (!slot) { pool_dropped_.fetch_add(1,std::memory_order_relaxed); return false; }
    auto& snapshot=slot->snapshot;
    snapshot.epoch=options_.epoch; snapshot.sequence=++sequence_;
    snapshot.stamp_ns=stamp_ns; snapshot.T_odom_body=T_odom_body;
    snapshot.T_body_lidar=T_body_lidar; snapshot.final_stage=FinalStage::Lio;
    snapshot.source_point_count=source_point_count;
    snapshot.point_count=source_point_count<options_.max_points?source_point_count:options_.max_points;
    const auto start=std::chrono::steady_clock::now();
    try {
      for (size_t i=0;i<snapshot.point_count;++i)
        snapshot.points_lidar[i]=reader(i*source_point_count/snapshot.point_count);
    } catch(...) {
      slot->owned.store(false,std::memory_order_release);throw;
    }
    snapshot.copy_ns=uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-start).count());
    copied_points_.fetch_add(snapshot.point_count,std::memory_order_relaxed);
    copy_ns_.fetch_add(snapshot.copy_ns,std::memory_order_relaxed);
    // Only the owner writes this maximum.
    if (snapshot.copy_ns>max_copy_ns_.load(std::memory_order_relaxed))
      max_copy_ns_.store(snapshot.copy_ns,std::memory_order_relaxed);
    pending_=slot; last_selected_ns_=stamp_ns; last_selected_pose_=T_odom_body;
    if (!expect_vio) finish_pending(FinalStage::Lio);
    return true;
  }

  // updated must mean an actually retained, numerically valid EKF update,
  // not merely that processFrame returned. False keeps the successful LIO pose.
  bool on_vio(uint64_t stamp_ns, bool updated, const Transform& T_odom_body);
  // Called after the owner loop stops, not on the real-time processing path.
  void close();
  BridgeStats stats() const noexcept;

 private:
  struct Slot { std::atomic<bool> owned{false}; Snapshot snapshot; };
  bool selected(uint64_t stamp_ns,const Transform& pose) const noexcept;
  bool queue_full() const noexcept;
  Slot* acquire_slot() noexcept;
  void finish_pending(FinalStage stage) noexcept;
  void work();
  bool write_snapshot(const Snapshot& snapshot);
  void write_status() noexcept;
  BridgeOptions options_;
  std::unique_ptr<Slot[]> pool_;
  size_t pool_size_=0;
  std::vector<Slot*> queue_;
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
  Slot* pending_=nullptr;
  uint64_t sequence_=0,last_seen_ns_=0,last_selected_ns_=0;
  Transform last_selected_pose_=identity_transform();
  std::atomic<bool> stopped_{false};
  std::thread worker_;
  std::atomic<uint64_t> candidates_{0},submitted_{0},written_{0},queue_dropped_{0},
    pool_dropped_{0},spool_dropped_{0},io_failed_{0},duplicates_{0},unmatched_vio_{0},
    copied_points_{0},copy_ns_{0},max_copy_ns_{0},last_measurement_ns_{0},status_failed_{0};
};
}  // namespace fastlivo_backend
