#pragma once

#include <memory>

#include <rclcpp/node.hpp>

namespace fast_livo2_core
{

// Keep the concrete LIVMapper type inside the core shared library.  The class
// contains Eigen/PCL members whose layout depends on compiler architecture
// flags, so constructing it in a separately compiled ROS executable creates
// an ABI hazard.
void request_mapping_stop() noexcept;
bool mapping_stop_requested() noexcept;
void run_mapping(const std::shared_ptr<rclcpp::Node> &node);

}  // namespace fast_livo2_core
