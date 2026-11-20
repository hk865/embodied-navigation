// Real native Ouster adapter, synchronization and IMU deskew contracts.
// Synthetic sensor inputs only: no Isaac, external camera service or truth poses.
#include <fast_livo2_core/core/LIVMapper.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value, const char *reason) {
    if (!value) throw std::runtime_error(reason);
}
using Parameters = std::vector<rclcpp::Parameter>;
rclcpp::NodeOptions options(Parameters extra = {}, bool rgb = false) {
    Parameters values{
        {"common.img_en", rgb ? 1 : 0}, {"common.allow_lio_only", true},
        {"common.allow_ouster_lidar", true}, {"common.require_complete_imu", true},
        {"preprocess.lidar_type", int(OUST64)}, {"preprocess.scan_line", 32},
        {"preprocess.point_filter_num", 1}, {"imu.imu_int_frame", 2},
        {"extrin_calib.extrinsic_T", std::vector<double>{0., 0., 0.}},
        {"extrin_calib.extrinsic_R", std::vector<double>{1., 0., 0., 0., 1., 0., 0., 0., 1.}},
        {"extrin_calib.Pcl", std::vector<double>{0., 0., 0.}},
        {"extrin_calib.Rcl", std::vector<double>{1., 0., 0., 0., 1., 0., 0., 0., 1.}},
    };
    for (const auto &p : extra) {
        values.erase(std::remove_if(values.begin(), values.end(), [&](const auto &v) {
            return v.get_name() == p.get_name();
        }), values.end());
        values.push_back(p);
    }
    rclcpp::NodeOptions result;
    result.parameter_overrides(values);
    return result;
}
rclcpp::Node::SharedPtr node(const std::string &name, Parameters extra = {}, bool rgb = false) {
    auto n = std::make_shared<rclcpp::Node>(name, options(extra, rgb));
    if (rgb) {
        n->declare_parameter("cam_model", "Pinhole");
        n->declare_parameter("cam_width", 320);
        n->declare_parameter("cam_height", 240);
        for (const auto &p : std::vector<std::pair<std::string, double>>{
                 {"scale", 1.}, {"cam_fx", 200.}, {"cam_fy", 200.},
                 {"cam_cx", 160.}, {"cam_cy", 120.}, {"cam_d0", 0.},
                 {"cam_d1", 0.}, {"cam_d2", 0.}, {"cam_d3", 0.}})
            n->declare_parameter(p.first, p.second);
    }
    return n;
}
sensor_msgs::msg::Imu::SharedPtr imu(double time, double wz = 0.) {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->header.stamp = rclcpp::Time(int64_t(std::llround(time * 1e9)));
    msg->linear_acceleration.z = 9.81;
    msg->angular_velocity.z = wz;
    return msg;
}
sensor_msgs::msg::PointCloud2::SharedPtr cloud(double time, std::vector<uint32_t> ns) {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.stamp = rclcpp::Time(int64_t(std::llround(time * 1e9)));
    msg->header.frame_id = "go2_imu";
    msg->height = 1; msg->width = ns.size(); msg->point_step = 24;
    msg->row_step = msg->width * msg->point_step;
    msg->is_dense = false;
    const std::vector<std::string> names{"x", "y", "z", "intensity", "t", "ring"};
    for (size_t i = 0; i < names.size(); ++i) {
        sensor_msgs::msg::PointField f;
        f.name = names[i]; f.offset = i * 4; f.count = 1;
        f.datatype = i < 4 ? f.FLOAT32 : i == 4 ? f.UINT32 : f.UINT8;
        msg->fields.push_back(f);
    }
    msg->data.resize(msg->row_step);
    for (size_t i = 0; i < ns.size(); ++i) {
        const float data[]{2., 0., 0., float(100 + i)};
        auto *p = msg->data.data() + i * msg->point_step;
        std::memcpy(p, data, sizeof(data));
        std::memcpy(p + 16, &ns[i], sizeof(ns[i]));
        p[20] = i % 32;
    }
    return msg;
}
void check_guards() {
    const std::vector<Parameters> rejected{
        {{"common.allow_ouster_lidar", false}}, {{"common.require_complete_imu", false}},
        {{"common.allow_lio_only", false}}, {{"imu.imu_en", false}},
        {{"common.use_gps", true}}, {{"preprocess.hilti_en", true}},
        {{"common.lidar_en", 0}}, {{"preprocess.lidar_type", 2}},
        {{"preprocess.lidar_type", 99}}, {{"preprocess.feature_extract_enabled", true}},
        {{"preprocess.scan_line", 0}}, {{"preprocess.scan_line", 129}},
        {{"preprocess.point_filter_num", 0}},
    };
    for (size_t i = 0; i < rejected.size(); ++i) {
        auto n = node("ouster_reject_" + std::to_string(i), rejected[i]);
        bool failed = false;
        try { LIVMapper mapper(n, n->get_name()); }
        catch (const std::runtime_error &e) {
            failed = std::string(e.what()).find("V19 requires") != std::string::npos ||
                     std::string(e.what()).find("Ouster requires") != std::string::npos;
        }
        require(failed, "An Ouster safety guard was bypassed");
    }
    // Omit the opt-in entirely, not merely override it to false.
    auto opts = options();
    auto p = opts.parameter_overrides();
    p.erase(std::remove_if(p.begin(), p.end(), [](const auto &v) {
        return v.get_name() == "common.allow_ouster_lidar";
    }), p.end());
    opts.parameter_overrides(p);
    auto n = std::make_shared<rclcpp::Node>("ouster_default_reject", opts);
    bool failed = false;
    try { LIVMapper mapper(n, n->get_name()); }
    catch (const std::runtime_error &e) {
        failed = std::string(e.what()).find("common.allow_ouster_lidar") != std::string::npos;
    }
    require(failed, "Ouster enabled by default");
    std::cout << "PASS explicit Ouster opt-in and all existing safety gates\n";
}
void check_preprocess() {
    Preprocess pre;
    pre.set(false, OUST64, .1, 1); pre.N_SCANS = 32;
    auto input = cloud(4., {99900000, 0, 25000000});
    auto original = input->data;
    PointCloudXYZI::Ptr out(new PointCloudXYZI());
    pre.process(input, out);
    require(out->size() == 3, "Ouster minimal field schema lost points");
    require((*out)[0].intensity == 101 && (*out)[1].intensity == 102 && (*out)[2].intensity == 100,
            "Ouster point identity lost during time ordering");
    require((*out)[0].curvature == 0 && (*out)[1].curvature == 25 &&
                std::abs((*out)[2].curvature - 99.9) < 1e-5,
            "Ouster ns timing not preserved as ms");
    require(input->data == original, "Preprocess mutated raw Ouster bytes");
    for (int mode = 0; mode < 5; ++mode) {
        auto bad = std::make_shared<sensor_msgs::msg::PointCloud2>(*input);
        if (mode == 0) bad->fields.erase(bad->fields.begin() + 4);
        if (mode == 1) bad->fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        if (mode == 2) bad->fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
        if (mode == 3) bad->data.resize(bad->data.size() - 1);
        if (mode == 4) bad->is_bigendian = true;
        bool failed = false;
        try { pre.process(bad, out); } catch (const std::runtime_error &) { failed = true; }
        require(failed, "Malformed Ouster timing/layout accepted");
    }
    auto invalid = cloud(5., {0, 1, 2, 3, 4});
    float nan = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(invalid->data.data(), &nan, 4);
    invalid->data[24 + 20] = 32;
    std::memcpy(invalid->data.data() + 48 + 12, &nan, 4);
    float zero = 0.; std::memcpy(invalid->data.data() + 72, &zero, 4);
    pre.process(invalid, out);
    require(out->size() == 1 && out->front().intensity == 104,
            "Invalid Ouster XYZ/intensity/ring or blind return entered estimator");
    std::cout << "PASS minimal Ouster schema, timing, ordering and invalid-point rejection\n";
}
void check_lio_deskew() {
    auto n = node("ouster_lio_deskew");
    LIVMapper mapper(n, n->get_name());
    require(mapper.slam_mode_ == ONLY_LIO && mapper.p_imu->lidar_type == OUST64,
            "Ouster not connected to native IMU path");
    image_transport::ImageTransport images(n);
    mapper.initializeSubscribersAndPublishers(n, images);
    require(mapper.sub_pcl && mapper.sub_imu && !mapper.sub_img && !mapper.sub_gps,
            "Ouster ONLY_LIO subscription contract changed");
    auto &m = mapper.LidarMeasures;
    mapper.receive_cloud(cloud(.9, {0, 40000000, 100000000}));
    for (int tick = 90; tick <= 100; ++tick) mapper.receive_imu(imu(tick / 100., 1.));
    require(mapper.sync_packages(m), "Ouster initialization scan failed to synchronize");
    mapper.handleFirstFrame(); mapper.processImu();
    require(!mapper.p_imu->imu_need_init && std::abs(m.last_lio_update_time - 1.) < 1e-9,
            "Ouster initialization propagation origin was not advanced");
    int previous = 100;
    for (int start : {100, 113}) {
        const int end = start + 10;
        auto input = cloud(start / 100., {0, 40000000, 100000000});
        mapper.receive_cloud(input);
        auto raw = mapper.lid_raw_data_buffer.back().cloud;
        auto before = *raw;
        for (int tick = previous + 1; tick <= end - 1; ++tick)
            mapper.receive_imu(imu(tick / 100., 1.));
        require(!mapper.sync_packages(m), "Ouster scan passed incomplete IMU gate");
        mapper.receive_imu(imu(end / 100., 1.));
        require(mapper.sync_packages(m), "Complete Ouster scan did not synchronize");
        require(m.lidar == raw && m.pcl_proc_cur != raw,
                "Ouster working offsets mutated raw cloud ownership");
        mapper.processImu();
        require(mapper.feats_undistort->size() == 3, "Ouster deskew lost points");
        double max_error = 0.;
        for (size_t i = 0; i < 3; ++i) {
            const auto &out = (*mapper.feats_undistort)[i];
            double remain = (.1 - before[i].curvature / 1000.);
            Eigen::Vector3f expected(2 * std::cos(remain), -2 * std::sin(remain), 0.);
            max_error = std::max(max_error, double((out.getVector3fMap() - expected).norm()));
            require((*raw)[i].curvature == before[i].curvature && (*raw)[i].x == before[i].x &&
                        out.intensity == before[i].intensity,
                    "Ouster deskew altered raw timing/geometry or point identity");
        }
        std::cout << "Ouster angular deskew start=" << start / 100. << " max_error=" << max_error << '\n';
        require(max_error < 2e-5, "Ouster timed deskew incorrect at scan boundary or gap");
        previous = end;
    }
    std::cout << "PASS native receive, initialization, complete IMU and timed angular deskew\n";
}
void check_livo_sync() {
    auto n = node("ouster_livo_sync", {}, true);
    LIVMapper mapper(n, n->get_name());
    require(mapper.slam_mode_ == LIVO && mapper.vio_manager->visual_submap,
            "Opt-in Ouster+RGB did not initialize LIVO");
    image_transport::ImageTransport images(n);
    mapper.initializeSubscribersAndPublishers(n, images);
    require(mapper.sub_pcl && mapper.sub_imu && mapper.sub_img && !mapper.sub_gps,
            "Ouster LIVO input subscriptions are incorrect");
    mapper.receive_cloud(cloud(10., {100000000, 0, 50000000, 25000000, 75000000}));
    auto raw = mapper.lid_raw_data_buffer.back().cloud;
    auto &m = mapper.LidarMeasures;
    m.last_lio_update_time = 10.;
    mapper.img_buffer.push_back(cv::Mat::zeros(240, 320, CV_8UC1));
    mapper.img_time_buffer.push_back(10.05);
    for (double t : {10.01, 10.02, 10.04}) mapper.receive_imu(imu(t));
    require(!mapper.sync_packages(m), "Ouster LIVO allowed incomplete IMU coverage");
    require(mapper.lid_raw_data_buffer.size() == 1 && mapper.imu_buffer.size() == 3,
            "Incomplete LIVO sync consumed evidence");
    mapper.receive_imu(imu(10.05)); mapper.receive_imu(imu(10.06));
    require(mapper.sync_packages(m), "Ouster LIVO cut failed despite complete IMU");
    require(m.lio_vio_flg == LIO && m.pcl_proc_cur->size() == 2 && m.pcl_proc_next->size() == 3,
            "Ouster LIVO did not split scan at image time");
    require(std::abs(m.pcl_proc_cur->back().curvature - 25.) < 1e-4 &&
                std::abs(m.pcl_proc_next->front().curvature) < 1e-4,
            "Ouster LIVO slice offsets use the wrong origin");
    require(m.measures.back().imu.size() == 4 && raw->size() == 5 && raw->front().curvature == 0,
            "Ouster LIVO changed raw cloud or IMU evidence");
    std::cout << "PASS Ouster+RGB constructor/subscriptions and complete-IMU image-time slicing\n";
}
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        check_guards(); check_preprocess(); check_lio_deskew(); check_livo_sync();
        rclcpp::shutdown(); return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL " << e.what() << '\n';
        rclcpp::shutdown(); return 1;
    }
}
