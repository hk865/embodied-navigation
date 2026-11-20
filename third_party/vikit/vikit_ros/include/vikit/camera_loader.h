/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 *  Update on: Feb 01, 2025
 *      Author: StrangeFly
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>

namespace vk {
namespace camera_loader {

// Build a camera model from ROS parameters. The single-camera overload reads
// `cam_model` (Ocam / Pinhole / EquidistantCamera / PolynomialCamera / ATAN) plus
// the matching intrinsics, preferring the remote (parameter_blackboard)
// namespace `ns` and falling back to the node's own parameters; an unknown model
// falls back to a hardcoded 1280x1024 PinholeCamera. The list overload reads
// `ns/cam_num` cameras from `ns/cam_i` namespaces (model names include
// FishPoly). The returned AbstractCamera* is heap allocated, owned by the caller.
/// Load from ROS Namespace
bool loadFromRosNs(const rclcpp::Node::SharedPtr &nh, const std::string &ns,
                   vk::AbstractCamera *&cam);
bool loadFromRosNs(const rclcpp::Node::SharedPtr &nh, const std::string &ns,
                   std::vector<vk::AbstractCamera *> &cam_list);

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
