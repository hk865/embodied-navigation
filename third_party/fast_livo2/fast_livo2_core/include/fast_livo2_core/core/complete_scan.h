#pragma once
#include <dynanav_sensor_msgs/msg/complete_lidar_scan.hpp>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace fastlivo_scan {
using Certificate = dynanav_sensor_msgs::msg::CompleteLidarScan;
inline constexpr uint64_t period_ns = 100000000ULL;
inline constexpr const char *config_id = "OS1_REV6_32ch10hz1024res:sha256:ec5b732ef18521efe33ccdfc4e1242e9c3ce066e9482a91d29b3cea91fc7000b";
inline double seconds(uint64_t ns) {
    return double(ns / 1000000000ULL) + double(ns % 1000000000ULL) * 1e-9;
}
inline void validate(const Certificate &c, const sensor_msgs::msg::PointCloud2 *bound) {
    // Pointer identity: the decoder owns an alias into this exact envelope.
    // A separate latest metadata topic, header-only match, or replay cannot bind.
    if (bound != &c.cloud || c.contract_version != 1 || !c.accumulate_outputs ||
        !c.scan_complete || c.session_id.empty() || c.session_id.size() > 64 ||
        c.sequence == 0 || c.sensor_id != "go2_rtx_os1" || c.config_id != config_id ||
        c.source_start_ns > uint64_t(INT64_MAX) - period_ns ||
        c.source_end_ns != c.source_start_ns + period_ns ||
        c.coordinate_frame_start_ns != c.source_start_ns + 15000000ULL ||
        c.coordinate_frame_end_ns != c.source_end_ns)
        throw std::runtime_error("complete scan certificate identity/config/interval failure");
    const auto &m = c.cloud;
    if (m.header.stamp.sec < 0 || m.header.stamp.nanosec >= 1000000000U ||
        uint64_t(m.header.stamp.sec) * 1000000000ULL + m.header.stamp.nanosec != c.source_start_ns ||
        m.header.frame_id != "go2_imu" || m.height != 1 || m.width > 65536 ||
        m.is_bigendian || m.point_step != 32 || m.row_step != uint64_t(m.width) * 32 ||
        m.data.size() != m.row_step)
        throw std::runtime_error("complete scan cloud binding/layout failure");
    const char *names[] = {"x","y","z","intensity","t","reflectivity","ring","ambient","range"};
    const uint32_t offsets[] = {0,4,8,12,16,20,22,24,28};
    const uint8_t types[] = {7,7,7,7,6,4,2,4,6};
    if (m.fields.size() != 9) throw std::runtime_error("complete scan field count failure");
    for (size_t f = 0; f < 9; ++f)
        if (m.fields[f].name != names[f] || m.fields[f].offset != offsets[f] ||
            m.fields[f].datatype != types[f] || m.fields[f].count != 1)
            throw std::runtime_error("complete scan field schema failure");
    uint32_t previous = 0;
    for (size_t i = 0; i < m.width; ++i) {
        const auto *p = m.data.data() + i * 32;
        uint32_t t; std::memcpy(&t, p + 16, 4);
        if (t >= period_ns || (i && t < previous) || p[22] >= 32)
            throw std::runtime_error("complete scan point outside certified interval/order/ring");
        for (size_t f = 0; f < 4; ++f) {
            float value; std::memcpy(&value, p + f * 4, 4);
            if (!std::isfinite(value)) throw std::runtime_error("complete scan nonfinite retained point");
        }
        previous = t;
    }
}
// Only the estimator owner commits this state. It survives raw->pending moves.
// Never reset/reseed within a run, never bridge a missing source interval.
struct Watermark {
    bool valid = false, failed = false;
    uint64_t start_ns = 0, end_ns = 0, sequence = 0, frame_id = 0;
    std::string session_id;
    void commit(const Certificate &c, const sensor_msgs::msg::PointCloud2 *bound) {
        if (failed) throw std::runtime_error("complete scan session already failed: new owned run required");
        try {
        validate(c, bound);
        if (valid && (c.session_id != session_id || c.sequence != sequence + 1 ||
                      c.native_frame_id <= frame_id || c.source_start_ns != end_ns))
            throw std::runtime_error("complete scan discontinuity/restart/duplicate: new owned run required");
        if (!valid) { start_ns = c.source_start_ns; session_id = c.session_id; }
        end_ns = c.source_end_ns; sequence = c.sequence; frame_id = c.native_frame_id; valid = true;
        } catch (...) { failed = true; throw; }
    }
    bool covers(double previous, double target) const {
        return valid && !failed && std::isfinite(previous) && std::isfinite(target) &&
            previous >= seconds(start_ns) && target > previous && target <= seconds(end_ns);
    }
};
} // namespace fastlivo_scan
