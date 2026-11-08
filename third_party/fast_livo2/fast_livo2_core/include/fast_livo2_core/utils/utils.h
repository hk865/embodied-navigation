#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <cstdint> // for int64_t
#include <cmath>
#include <limits>  // for std::numeric_limits
#include <stdexcept> // for std::out_of_range
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

std::vector<int> convertToIntVectorSafe(const std::vector<int64_t>& int64_vector);

inline double stamp2Sec(const builtin_interfaces::msg::Time& stamp)
{
    return rclcpp::Time(stamp).seconds();
}

inline rclcpp::Time sec2Stamp(double timestamp)
{
  // Estimator seconds originate from integer ROS headers. Truncating their
  // floating remainder loses a nanosecond (9.54 becomes 9539999999), breaking
  // exact joins and strict observation-gap checks. Round the complete value;
  // this changes encoding only, never acquisition time or a freshness limit.
  if (!std::isfinite(timestamp) || timestamp < 0.0 ||
      timestamp >= static_cast<double>(std::numeric_limits<int32_t>::max()) + 1.0)
    throw std::out_of_range("Timestamp cannot be encoded as a ROS header");
  return rclcpp::Time(static_cast<int64_t>(std::llround(timestamp * 1e9)));
}

namespace tf
{

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

inline tf2::Quaternion createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

inline tf2::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}
}

inline geometry_msgs::msg::TransformStamped createTransformStamped(
    const tf2::Transform &transform,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    const std::string &child_frame_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform = tf2::toMsg(transform);
    return transform_stamped;
}

#endif // UTILS_H
