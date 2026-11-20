#ifndef DEMO_STATIONARY_IMU_INITIALIZATION_H
#define DEMO_STATIONARY_IMU_INITIALIZATION_H

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

// Acceleration and angular velocity only. No attitude, map, or truth input.
// This gate is used only before the first gravity alignment; it never constrains
// the normal IMU propagation during exploration or navigation.
class StationaryImuInitialization {
  public:
    // Gate limits. gyro_norm is the max |omega| in rad/s, acceleration_norm_error
    // is the allowed deviation of |a| from 9.81 m/s^2, acceleration_axis_std is the
    // per-axis std limit in m/s^2, max_sample_gap is the largest allowed spacing
    // between samples and min_span is the required window duration in seconds.
    struct Limits {
        double gyro_norm = .03;
        double acceleration_norm_error = 1.2;
        double acceleration_axis_std = .40;
        double max_sample_gap = .03;
        double min_span = 2.9;
    } limits;

    void reset() {
        clear_statistics();
        last_seen_stamp_ = -1.;
        reset_count = 0;
        last_reset_reason = "none";
    }

    // Feed one IMU sample. Rejection clears the statistics window and records
    // its reason. Readiness requires the configured sample count and time span
    // of measurements continuously satisfying the inertial-statistics limits.
    bool add(double stamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro) {
        if (!std::isfinite(stamp) || !acc.allFinite() || !gyro.allFinite()) {
            reject("nonfinite");
            last_seen_stamp_ = -1.;
            return false;
        }
        if (last_seen_stamp_ >= 0. && stamp <= last_seen_stamp_) {
            reject("nonincreasing_stamp");
            return false;
        }
        if (last_seen_stamp_ >= 0. && stamp - last_seen_stamp_ > limits.max_sample_gap)
            reject("sample_gap");
        last_seen_stamp_ = stamp;
        if (gyro.norm() > limits.gyro_norm) {
            reject("angular_motion");
            return false;
        }
        if (std::abs(acc.norm() - 9.81) > limits.acceleration_norm_error) {
            reject("acceleration_norm");
            return false;
        }

        if (count == 0)
            first_stamp = stamp;
        ++count;
        last_stamp = stamp;
        const Eigen::Vector3d delta = acc - mean_acc;
        mean_acc += delta / static_cast<double>(count);
        acc_m2 += delta.cwiseProduct(acc - mean_acc);
        mean_gyro += (gyro - mean_gyro) / static_cast<double>(count);
        max_gyro_norm = std::max(max_gyro_norm, gyro.norm());
        max_acc_norm_error = std::max(max_acc_norm_error, std::abs(acc.norm() - 9.81));
        // Allow enough observations for a useful variance estimate, then discard
        // the entire window on vibration or a changing acceleration direction.
        if (count >= 20 && acceleration_std().maxCoeff() > limits.acceleration_axis_std) {
            reject("acceleration_variation");
            return false;
        }
        return true;
    }

    // Per-axis population standard deviation: sqrt(M2/count), using running M2
    // (Welford); negative rounding noise is clamped to zero before the square root.
    Eigen::Vector3d acceleration_std() const {
        if (count == 0)
            return Eigen::Vector3d::Zero();
        return (acc_m2 / static_cast<double>(count)).cwiseMax(0.).cwiseSqrt();
    }
    double span() const { return count > 0 ? last_stamp - first_stamp : 0.; }
    bool ready(std::size_t required_samples) const {
        return count >= required_samples && span() >= limits.min_span;
    }

    std::size_t count = 0, reset_count = 0;
    double first_stamp = -1., last_stamp = -1.;
    double max_gyro_norm = 0., max_acc_norm_error = 0.;
    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
    std::string last_reset_reason = "none";

  private:
    void clear_statistics() {
        count = 0;
        first_stamp = last_stamp = -1.;
        max_gyro_norm = max_acc_norm_error = 0.;
        mean_acc.setZero();
        mean_gyro.setZero();
        acc_m2.setZero();
    }
    void reject(const char *reason) {
        clear_statistics();
        ++reset_count;
        last_reset_reason = reason;
    }
    double last_seen_stamp_ = -1.;
    Eigen::Vector3d acc_m2 = Eigen::Vector3d::Zero();
};

#endif
