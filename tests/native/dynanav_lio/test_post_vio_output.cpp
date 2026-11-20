// Opt-in output contract: no simulator, camera service or physical sensor input.
#include <fast_livo2_core/core/LIVMapper.h>
#include <image_transport/image_transport.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
void require(bool value, const char *reason) {
    if (!value)
        throw std::runtime_error(reason);
}
rclcpp::NodeOptions options() {
    rclcpp::NodeOptions value;
    value.parameter_overrides(std::vector<rclcpp::Parameter>{
        {"common.img_en", 0},
        {"common.allow_lio_only", true},
        {"common.lidar_en", 1},
        {"preprocess.lidar_type", 0},
        {"extrin_calib.extrinsic_T", std::vector<double>{0., 0., 0.}},
        {"extrin_calib.extrinsic_R", std::vector<double>{1., 0., 0., 0., 1., 0., 0., 0., 1.}},
        {"publish.odom_frame", "estimated_map"},
        {"publish.child_frame", "measured_imu"},
    });
    return value;
}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<rclcpp::Node>("post_vio_output_contract", options());
        LIVMapper mapper(node, node->get_name());
        require(!mapper.post_vio_odometry_en, "Final-pose output enabled without opt-in");
        image_transport::ImageTransport images(node);
        mapper.initializeSubscribersAndPublishers(node, images);
        require(!mapper.pubPostVioOdometry && !mapper.pubPostVioOdometryStatus,
                "Disabled final-pose output created publishers");
        require(!mapper.sub_gps && !mapper.sub_img,
                "Output fixture unexpectedly enabled camera or GT input");

        // Isolate publication from the estimator: constructor remains camera-free,
        // then this test injects already-final states and a VIO-stage timestamp.
        mapper.post_vio_odometry_en = true;
        mapper.pubPostVioOdometry = node->create_publisher<nav_msgs::msg::Odometry>(
            "/test_post_vio_pose", rclcpp::QoS(10));
        mapper.pubPostVioOdometryStatus = node->create_publisher<std_msgs::msg::String>(
            "/test_post_vio_status", rclcpp::QoS(10));
        std::vector<nav_msgs::msg::Odometry> poses;
        std::vector<nlohmann::json> statuses;
        auto pose_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
            "/test_post_vio_pose", rclcpp::QoS(10),
            [&](nav_msgs::msg::Odometry::ConstSharedPtr message) { poses.push_back(*message); });
        auto status_subscription = node->create_subscription<std_msgs::msg::String>(
            "/test_post_vio_status", rclcpp::QoS(10),
            [&](std_msgs::msg::String::ConstSharedPtr message) {
                statuses.push_back(nlohmann::json::parse(message->data));
            });
        const auto wait_for = [&](auto condition, const char *reason) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (!condition() && std::chrono::steady_clock::now() < deadline) {
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            require(condition(), reason);
        };
        wait_for([&] { return mapper.pubPostVioOdometry->get_subscription_count() == 1 &&
                              mapper.pubPostVioOdometryStatus->get_subscription_count() == 1; },
                 "Output test subscriptions did not match");

        mapper.LidarMeasures.lio_vio_flg = VIO;
        MeasureGroup measured{};
        measured.vio_time = 9.54; // Exercises the integer-ns rounding contract.
        mapper.LidarMeasures.measures.push_back(measured);
        mapper._state.pos_end = V3D(1., 2., 3.);
        const StatesGroup before = mapper._state;
        mapper.publish_post_vio_odometry(false); // A valid no-op still has final pose.
        mapper._state.pos_end = V3D(4., 5., 6.);
        mapper.publish_post_vio_odometry(true); // Same image cannot publish twice.
        mapper.LidarMeasures.measures.back().vio_time = 9.53;
        mapper.publish_post_vio_odometry(true); // Backward image cannot be admitted.
        mapper.LidarMeasures.measures.back().vio_time = 9.64;
        mapper.publish_post_vio_odometry(true);
        mapper.LidarMeasures.measures.back().vio_time = 9.74;
        mapper._state.pos_end.x() = std::numeric_limits<double>::quiet_NaN();
        mapper.publish_post_vio_odometry(true);
        mapper._state.pos_end = before.pos_end;
        mapper.LidarMeasures.lio_vio_flg = LIO;
        mapper.publish_post_vio_odometry(true); // This is not a pre-VIO publisher.
        wait_for([&] { return poses.size() >= 2 && statuses.size() >= 2; },
                 "Expected final-pose/status messages were not delivered");
        for (int i = 0; i < 5; ++i)
            rclcpp::spin_some(node);
        require(poses.size() == 2 && statuses.size() == 2,
                "Duplicate, backward, invalid or LIO-stage sample published");
        for (size_t i = 0; i < poses.size(); ++i) {
            const auto &pose = poses[i];
            const auto &status = statuses[i];
            const int64_t stamp_ns = rclcpp::Time(pose.header.stamp).nanoseconds();
            require(stamp_ns == (i == 0 ? 9540000000LL : 9640000000LL),
                    "Final pose did not retain the measured image stamp");
            require(pose.header.frame_id == "estimated_map" && pose.child_frame_id == "measured_imu",
                    "Configured output frames were not retained");
            require(pose.pose.pose.position.x == (i == 0 ? 1. : 4.),
                    "Output used another stage's state");
            require(pose.pose.pose.orientation.w == 1., "State rotation encoding changed");
            for (size_t axis = 0; axis < 6; ++axis)
                require(pose.pose.covariance[axis * 7] == 1e12 &&
                            pose.twist.covariance[axis * 7] == 1e12,
                        "Unavailable pose/twist covariance looked certain");
            require(status.at("image_stamp_ns") == stamp_ns &&
                        status.at("pose_source") == "livo_post_vio" &&
                        status.at("state_stage") == "post_vio" &&
                        status.at("visual_update_applied") == (i == 1) &&
                        status.at("twist_available") == false &&
                        status.at("pose_covariance_available") == false,
                    "Same-stamp final-stage/update/availability provenance was lost");
        }
        require(mapper._state.rot_end == before.rot_end &&
                    mapper._state.vel_end == before.vel_end &&
                    mapper._state.cov == before.cov,
                "Publication mutated estimator arithmetic");
        rclcpp::shutdown();
        std::cout << "PASS opt-in post-VIO pose, no-op status, frames, timestamps, unavailable covariance/twist, duplicate/stale/invalid rejection\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
