#pragma once
// Observation-only owner-thread ledger. No estimator/control code reads it.
// Requires FASTLIVO_SYNC_OBSERVE=1 and FASTLIVO_BOUNDARY_TIMING_FILE. Reuses
// the existing exclusive-create, no-fsync, shared 8 MiB ScalarWriter cap.
#include "boundary_timing.h"
#include <array>
#include <cstring>

namespace fastlivo_sync {
enum Reason : size_t {
    RawEmpty, ImageEmpty, ImuEmpty, SparseDrainEmpty, WaitScanImu,
    OnlyLioSelected, RawTailEmpty, OldImageDropped, WaitPointCoverage,
    WaitImageImu, LivoLioSelected, LivoVioSelected, InvalidStage,
    OnlyLoEmpty, OnlyLoSelected, InvalidMode, PairIdentityChanged, PairSkipped,
    PairLioUnfinished, PairVioUnfinished, ReasonCount
};
inline bool enabled() noexcept {
    static const bool selected = [] {
        const char *s = std::getenv("FASTLIVO_SYNC_OBSERVE");
        return s && std::strcmp(s, "1") == 0;
    }();
    return selected && fastlivo_timing::scalar_writer().fd >= 0;
}
inline uint64_t now() noexcept {
    bool valid = true;
    return fastlivo_timing::read_clock(CLOCK_MONOTONIC, valid);
}
struct View {
    int stage = 0, mode = 0;
    // Valid bits: image_target=1, last_lio=2, raw_latest_point=4,
    // imu_latest=8, last_committed_imu=16. Missing values are zero, never stale.
    unsigned valid = 0;
    std::array<double, 5> times{};
    // raw clouds, raw headers, images, image stamps, IMUs, current/pending points.
    std::array<size_t, 7> sizes{};
    bool imu_initializing = false, complete_imu_required = false;
    // Source-time windows, O(1); no inferred span from count or sample rate.
    std::array<size_t, 3> cache_counts{}; // primary IMU, images, propagation IMU
    std::array<double, 6> cache_endpoints{}; // oldest/newest for each queue
    unsigned cache_valid = 0;
    bool complete_scan_enabled = false, complete_scan_valid = false;
    std::array<uint64_t, 3> complete_scan_interval{}; // first S, contiguous E, sequence
};
inline View snapshot(const LIVMapper &mapper, const LidarMeasureGroup &m) noexcept {
    View out;
    out.complete_scan_enabled = mapper.complete_scan_watermark;
    out.complete_scan_valid = mapper.scan_watermark.valid;
    out.complete_scan_interval = {mapper.scan_watermark.start_ns, mapper.scan_watermark.end_ns,
                                 mapper.scan_watermark.sequence};
    out.stage = m.lio_vio_flg;
    out.mode = mapper.slam_mode_;
    out.sizes = {mapper.lid_raw_data_buffer.size(), mapper.lid_header_time_buffer.size(),
                 mapper.img_buffer.size(), mapper.img_time_buffer.size(), mapper.imu_buffer.size(),
                 m.pcl_proc_cur ? m.pcl_proc_cur->size() : 0,
                 m.pcl_proc_next ? m.pcl_proc_next->size() : 0};
    const auto set_time = [&](size_t i, double value) {
        if (std::isfinite(value) && value >= 0) {
            out.times[i] = value;
            out.valid |= 1u << i;
        }
    };
    if (!mapper.img_time_buffer.empty() && out.sizes[2] == out.sizes[3])
        set_time(0, mapper.img_time_buffer.front() + mapper.exposure_time_init);
    set_time(1, m.last_lio_update_time);
    if (!mapper.lid_raw_data_buffer.empty() && out.sizes[0] == out.sizes[1]) {
        const auto &cloud = mapper.lid_raw_data_buffer.back().cloud;
        if (cloud && !cloud->empty())
            set_time(2, mapper.lid_header_time_buffer.back() + cloud->back().curvature / 1000.);
    }
    if (!mapper.imu_buffer.empty())
        set_time(3, stamp2Sec(mapper.imu_buffer.back()->header.stamp));
    set_time(4, mapper.last_timestamp_imu);
    out.imu_initializing = mapper.p_imu && mapper.p_imu->imu_need_init;
    out.complete_imu_required = mapper.require_complete_imu;
    out.cache_counts = {mapper.imu_buffer.size(), mapper.img_time_buffer.size(),
                        mapper.prop_imu_buffer.size()};
    const auto window = [&](size_t i, double first, double last) {
        if (std::isfinite(first) && std::isfinite(last)) {
            out.cache_valid |= 1u << i;
            out.cache_endpoints[2 * i] = first;
            out.cache_endpoints[2 * i + 1] = last;
        }
    };
    if (!mapper.imu_buffer.empty())
        window(0, stamp2Sec(mapper.imu_buffer.front()->header.stamp),
                  stamp2Sec(mapper.imu_buffer.back()->header.stamp));
    if (!mapper.img_time_buffer.empty())
        window(1, mapper.img_time_buffer.front(), mapper.img_time_buffer.back());
    if (!mapper.prop_imu_buffer.empty())
        window(2, stamp2Sec(mapper.prop_imu_buffer.front().header.stamp),
                  stamp2Sec(mapper.prop_imu_buffer.back().header.stamp));
    return out;
}
inline nlohmann::json json_pair(const LIVMapper &mapper) {
    const auto &p = mapper.pending_pair;
    return {{"enabled", mapper.same_pair_vio_handoff}, {"sequence", p.sequence}, {"phase", int(p.phase)}, {"outcome", int(p.outcome)},
            {"source_stamp_ns", p.source_stamp_ns}, {"source_stamp_valid", p.source_stamp_valid},
            {"image_time", p.image_time}, {"target_time", p.target_time},
            {"retired", p.retired}, {"skipped", p.skipped}};
}
inline nlohmann::json json_view(const View &v) {
    return {{"stage", v.stage}, {"mode", v.mode}, {"valid", v.valid},
            {"times", v.times}, {"sizes", v.sizes},
            {"imu_initializing", v.imu_initializing},
            {"complete_imu_required", v.complete_imu_required},
            {"cache_counts", v.cache_counts}, {"cache_endpoints", v.cache_endpoints},
            {"cache_valid", v.cache_valid},
            {"complete_scan_enabled", v.complete_scan_enabled},
            {"complete_scan_valid", v.complete_scan_valid},
            {"complete_scan_interval_ns", v.complete_scan_interval}};
}
struct Ledger {
    uint64_t rows = 0, attempts = 0, pair = 0, last_summary = 0;
    // Complete cumulative counts, indexed by reason then entry stage (WAIT,VIO,LIO,LO).
    std::array<std::array<uint64_t, 4>, ReasonCount> counts{};
    double selected_target = 0;
    bool selected_target_valid = false, lio_completed = false, lio_effective = false;
};
inline Ledger &ledger() noexcept { static thread_local Ledger value; return value; }
inline void emit(nlohmann::json row) noexcept {
    try {
        auto &l = ledger();
        row["schema"] = "native_sync_observation/v1";
        row["row"] = ++l.rows; // attempts, not a delivery receipt; gaps must be reported.
        row["wall_ns"] = now();
        row["tid"] = ::syscall(SYS_gettid);
        row["pair"] = l.pair;
        row["observation_only"] = true;
        fastlivo_timing::scalar_writer().emit_raw(row.dump() + "\n");
    } catch (...) {} // Never replace primary estimator results/exceptions.
}
inline bool same_target(const View &v, bool image) noexcept {
    const auto &l = ledger();
    const size_t index = image ? 0 : 1;
    return l.selected_target_valid && (v.valid & (1u << index)) &&
           fastlivo_diag::nanoseconds(l.selected_target) == fastlivo_diag::nanoseconds(v.times[index]);
}
class Attempt {
  public:
    Attempt(const LIVMapper &mapper, const LidarMeasureGroup &m) noexcept
        : active_(enabled()), mapper_(mapper), m_(m) {
        if (active_) before_ = snapshot(mapper, m);
    }
    bool result(Reason reason, bool result) noexcept {
        if (!active_) return result;
        try {
            auto &l = ledger();
            const View after = snapshot(mapper_, m_);
            ++l.attempts;
            if (before_.stage >= 0 && before_.stage < 4) ++l.counts[reason][before_.stage];
            if (reason == LivoLioSelected || reason == OnlyLioSelected) {
                ++l.pair;
                l.selected_target_valid = !m_.measures.empty() &&
                    std::isfinite(m_.measures.back().lio_time) && m_.measures.back().lio_time >= 0;
                l.selected_target = l.selected_target_valid ? m_.measures.back().lio_time : 0;
                l.lio_completed = l.lio_effective = false;
            }
            const auto wall = now();
            const bool summary = !l.last_summary || wall - l.last_summary >= 1000000000ULL;
            if (result || summary || reason == PairSkipped || reason == PairIdentityChanged) {
                nlohmann::json row = {{"event", "sync_result"}, {"reason", reason},
                    {"result", result}, {"attempt", l.attempts}, {"before", json_view(before_)},
                    {"after", json_view(after)}, {"pending_pair", json_pair(mapper_)},
                    {"selected_target", l.selected_target},
                    {"selected_target_valid", l.selected_target_valid},
                    {"pair_target_matches", same_target(before_, true)},
                    {"lio_completed", l.lio_completed}, {"lio_effective", l.lio_effective}};
                if (summary) {
                    row["cumulative_reason_entry_stage_counts"] = l.counts;
                    l.last_summary = wall;
                }
                emit(std::move(row));
            }
        } catch (...) {}
        return result;
    }
  private:
    bool active_;
    const LIVMapper &mapper_;
    const LidarMeasureGroup &m_;
    View before_;
};
inline void stage(const LIVMapper &mapper, const char *event, bool completed,
                  bool ekf_updated, bool initializing_before = false) noexcept {
    if (!enabled()) return;
    try {
        const auto view = snapshot(mapper, mapper.LidarMeasures);
        auto &l = ledger();
        const bool match = same_target(view, false);
        // Effective means the normal LIO handler completed with measured residuals,
        // initialized IMU and finite pose. It is not navigation readiness or accuracy.
        const bool lio = std::strcmp(event, "lio_completed") == 0;
        const size_t residuals = lio ? mapper.voxelmap_manager->ptpl_list_.size() : 0;
        const bool finite_pose = mapper._state.pos_end.allFinite() && mapper._state.rot_end.allFinite();
        const bool finite_state = finite_pose && mapper._state.vel_end.allFinite() &&
            mapper._state.bias_g.allFinite() && mapper._state.bias_a.allFinite() &&
            mapper._state.gravity.allFinite() && mapper._state.cov.allFinite() &&
            std::isfinite(mapper._state.inv_expo_time);
        if (lio && match) {
            l.lio_completed = completed;
            l.lio_effective = completed && !view.imu_initializing && residuals > 0 && finite_pose;
        }
        emit({{"event", event}, {"view", json_view(view)}, {"pending_pair", json_pair(mapper)},
              {"completed", completed},
              {"ekf_updated", ekf_updated}, {"initializing_before", initializing_before},
              {"pair_target_matches", match}, {"lio_completed", l.lio_completed},
              {"lio_effective", l.lio_effective}, {"effective_residual_count", residuals},
              {"handler_completed", completed}, {"initialized", !view.imu_initializing},
              {"has_output_points", mapper.pcl_w_wait_pub && !mapper.pcl_w_wait_pub->empty()},
              {"finite_state", finite_state},
              {"finite_pose", finite_pose},
              {"undistorted_points", mapper.feats_undistort ? mapper.feats_undistort->size() : 0}});
    } catch (...) {}
}
inline void flush(const LIVMapper &mapper) noexcept {
    if (!enabled()) return;
    try {
        emit({{"event", "sync_summary"}, {"final", true},
              {"attempt", ledger().attempts}, {"view", json_view(snapshot(mapper, mapper.LidarMeasures))},
              {"pending_pair", json_pair(mapper)},
              {"cumulative_reason_entry_stage_counts", ledger().counts}});
    } catch (...) {}
}

}
