// Native adapter checks only: no simulator, camera service or sensor publisher.
#include <fast_livo2_core/core/LIVMapper.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <future>
#include <cmath>
#include <stdexcept>
#include <cstring>
#include <new>

namespace {
void require(bool value, const char *reason) {
    if (!value)
        throw std::runtime_error(reason);
}
rclcpp::NodeOptions options(bool opt_in, int init_frames = 30) {
    rclcpp::NodeOptions value;
    std::vector<rclcpp::Parameter> parameters{
        {"common.img_en", 0},
        {"common.lidar_en", 1},
        {"preprocess.lidar_type", 0},
        {"imu.imu_int_frame", init_frames},
        {"extrin_calib.extrinsic_T", std::vector<double>{0., 0., 0.}},
        {"extrin_calib.extrinsic_R", std::vector<double>{1., 0., 0., 0., 1., 0., 0., 0., 1.}},
    };
    // Deliberately omit the parameter in the default-rejection case.
    if (opt_in)
        parameters.emplace_back("common.allow_lio_only", true);
    value.parameter_overrides(parameters);
    return value;
}
sensor_msgs::msg::Imu::SharedPtr imu(double time, double ax = 0., double wz = 0.) {
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(time * 1e9)));
    message->linear_acceleration.x = ax;
    message->linear_acceleration.z = 9.81;
    message->angular_velocity.z = wz;
    return message;
}
PointCloudXYZI::Ptr cloud(size_t count) {
    auto points = std::make_shared<PointCloudXYZI>();
    points->resize(count);
    for (size_t i = 0; i < count; ++i) {
        (*points)[i].x = 1. + i;
        (*points)[i].y = (*points)[i].z = (*points)[i].curvature = 0.;
        (*points)[i].intensity = 100. + i;
    }
    return points;
}
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        // Deterministic regression for camera-free construction: allocator-zeroed
        // memory used to hide visual_submap's indeterminate pointer. Placement
        // construction must establish empty ownership even in nonzero storage.
        alignas(VIOManager) unsigned char storage[sizeof(VIOManager)];
        std::memset(storage, 0xA5, sizeof(storage));
        auto *uninitialized_visual = new (storage) VIOManager();
        require(uninitialized_visual->visual_submap == nullptr,
                "VIOManager did not initialize visual_submap ownership");
        uninitialized_visual->~VIOManager();
        std::cout << "PASS visual manager lifecycle without camera initialization\n";
        bool rejected = false;
        auto default_node = std::make_shared<rclcpp::Node>("lio_default_rejection", options(false));
        try {
            LIVMapper mapper(default_node, default_node->get_name());
        } catch (const std::runtime_error &error) {
            rejected = std::string(error.what()).find("common.allow_lio_only") != std::string::npos;
        }
        require(rejected, "Default mode accepted image-free LIO without explicit opt-in");
        {
            auto node = std::make_shared<rclcpp::Node>("lio_native_fixture", options(true));
            LIVMapper mapper(node, node->get_name());
            require(mapper.slam_mode_ == ONLY_LIO, "Opt-in did not select ONLY_LIO");
            require(mapper.vio_manager->visual_submap == nullptr,
                    "Camera-free ONLY_LIO unexpectedly owns an initialized visual submap");
            image_transport::ImageTransport images(node);
            mapper.initializeSubscribersAndPublishers(node, images);
            require(!mapper.sub_img, "LIO-only created an image subscription");
            require(!mapper.sub_gps, "LIO-only created a GT/GPS subscription");
            require(bool(mapper.sub_pcl) && bool(mapper.sub_imu), "Missing LIO sensor subscription");
            mapper.lid_raw_data_buffer.push_back({cloud(0)});
            mapper.lid_header_time_buffer.push_back(0.1);
            mapper.lid_raw_data_buffer.push_back({cloud(1)});
            mapper.lid_header_time_buffer.push_back(0.2);
            auto original = cloud(3);
            mapper.lid_raw_data_buffer.push_back({original});
            mapper.lid_header_time_buffer.push_back(1.0);
            for (double time : {0.9, 1.0, 1.1})
                mapper.imu_buffer.push_back(imu(time));
            mapper.last_timestamp_imu = 1.1;
            LidarMeasureGroup measures;
            // The sparse pop must obey the same buffer mutex as ingress and
            // the normal whole-frame pop. Hold it while the consumer enters.
            mapper.mtx_buffer.lock();
            auto synced = std::async(std::launch::async, [&] { return mapper.sync_packages(measures); });
            const bool waited = synced.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout;
            const bool untouched = mapper.lid_raw_data_buffer.size() == 3 && mapper.lid_header_time_buffer.size() == 3;
            mapper.mtx_buffer.unlock();
            require(waited && untouched, "Sparse-cloud consumer did not respect buffer ownership");
            require(synced.get(), "Sparse clouds blocked the following valid frame");
            require(mapper.lio_sparse_cloud_drops == 2, "Sparse-frame drop count is incorrect");
            require(mapper.lid_raw_data_buffer.empty() && mapper.lid_header_time_buffer.empty(),
                    "Cloud/header queues diverged or did not drain");
            require(measures.lidar == original && measures.lidar->size() == 3,
                    "Valid geometry was copied, filled or replaced");
            require(measures.last_lio_update_time == 1.0,
                    "Dropped frame initialized the LIO reference time");
            require(measures.measures.size() == 1 && measures.measures.front().imu.size() == 2,
                    "IMU evidence changed when dropping sparse clouds");
        }
        {
            auto node = std::make_shared<rclcpp::Node>("lio_instantaneous_deskew_fixture", options(true, 2));
            LIVMapper mapper(node, node->get_name());
            auto &measures = mapper.LidarMeasures;
            mapper.lid_raw_data_buffer.push_back({cloud(3)});
            mapper.lid_header_time_buffer.push_back(1.03);
            for (double time : {1.01, 1.02, 1.03})
                mapper.imu_buffer.push_back(imu(time));
            mapper.last_timestamp_imu = 1.03;
            require(mapper.sync_packages(measures), "Initialization frame failed to synchronize");
            mapper.handleFirstFrame();
            mapper.processImu();
            require(!mapper.p_imu->imu_need_init, "Synthetic stationary IMU initialization failed");
            require(std::abs(measures.last_lio_update_time - 1.03) < 1e-9,
                    "Initialization did not advance the propagation origin to its current frame");
            int previous_tick = 103;
            for (int end_tick : {113, 133}) {
                const double stamp = end_tick / 100.;
                auto raw = cloud(3);
                const auto original = *raw;
                mapper.lid_raw_data_buffer.push_back({raw});
                mapper.lid_header_time_buffer.push_back(stamp);
                for (int tick = previous_tick + 1; tick <= end_tick; ++tick)
                    mapper.imu_buffer.push_back(imu(tick / 100., 2., 1.));
                mapper.last_timestamp_imu = stamp;
                require(mapper.sync_packages(measures), "Moving instantaneous frame failed to synchronize");
                mapper.processImu();
                require(mapper._state.vel_end.norm() > 0.1 &&
                            std::abs(std::atan2(mapper._state.rot_end(1, 0), mapper._state.rot_end(0, 0))) > 0.05,
                        "Motion fixture did not actually propagate nonzero IMU motion");
                require(measures.lidar == raw && mapper.feats_undistort->size() == raw->size(),
                        "Raw point ownership or deskew point count changed");
                double max_error = 0.;
                for (size_t i = 0; i < raw->size(); ++i) {
                    const auto &expected = original[i];
                    const auto &input = (*raw)[i];
                    const auto &output = (*mapper.feats_undistort)[i];
                    require(input.x == expected.x && input.y == expected.y && input.z == expected.z &&
                                input.intensity == expected.intensity && input.curvature == 0.,
                            "Raw instantaneous point or its source offset was mutated");
                    require(output.intensity == expected.intensity, "Point identity/order was lost by deskew");
                    max_error = std::max(max_error, double((output.getVector3fMap() - expected.getVector3fMap()).norm()));
                }
                std::cout << "instantaneous frame=" << stamp << " current_end_point_error=" << max_error << '\n';
                require(max_error < 1e-5,
                        "Current-end instantaneous points were incorrectly deskewed at the previous frame");
                require(std::abs(measures.last_lio_update_time - stamp) < 1e-9,
                        "Moving frame did not advance the propagation origin");
                previous_tick = end_tick;
            }
        }
        {
            auto node = std::make_shared<rclcpp::Node>("lio_timed_cloud_fixture", options(true));
            LIVMapper mapper(node, node->get_name());
            mapper.p_pre->lidar_type = VELO16;
            auto raw = cloud(3);
            (*raw)[1].curvature = 5.;
            (*raw)[2].curvature = 11.;
            mapper.LidarMeasures.last_lio_update_time = 0.9;
            mapper.lid_raw_data_buffer.push_back({raw});
            mapper.lid_header_time_buffer.push_back(1.0);
            mapper.imu_buffer.push_back(imu(1.02));
            mapper.last_timestamp_imu = 1.02;
            require(mapper.sync_packages(mapper.LidarMeasures), "Timed frame failed to synchronize");
            require(mapper.LidarMeasures.pcl_proc_cur == raw,
                    "Non-generic cloud ownership was changed");
            require((*raw)[0].curvature == 0. && (*raw)[1].curvature == 5. && (*raw)[2].curvature == 11.,
                    "Non-generic point offsets were rebased");
            require(std::abs(mapper.LidarMeasures.lidar_frame_beg_time - 1.0) < 1e-9 &&
                        std::abs(mapper.LidarMeasures.lidar_frame_end_time - 1.011) < 1e-9,
                    "Non-generic frame time semantics changed");
        }
        rclcpp::shutdown();
        std::cout << "PASS default opt-in guard, no-camera/no-image/no-GT LIO, paired sparse-cloud drop, unchanged valid points/IMU\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
