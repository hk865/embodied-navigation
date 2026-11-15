// Real mapper pipeline with synthetic messages. No Isaac, GPU, or external ROS run.
#define main original_ouster_fixture_main
#include "test_ouster_mode.cpp"
#undef main
#include <fast_livo2_core/core/sync_observation.h>

namespace {
void image(LIVMapper &mapper, double t) {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.stamp = rclcpp::Time(int64_t(std::llround(t * 1e9)));
    msg->encoding = "bgr8"; msg->width = 320; msg->height = 240; msg->step = 960;
    msg->data.resize(msg->height * msg->step, 96);
    mapper.receive_image(msg);
}
void initial_pair(LIVMapper &mapper) {
    auto &m = mapper.LidarMeasures;
    mapper.receive_cloud(cloud(10., {0, 25000000, 50000000, 75000000, 100000000}));
    image(mapper, 10.05);
    for (int tick = 1000; tick <= 1005; ++tick) mapper.receive_imu(imu(tick / 100.));
    require(mapper.sync_packages(m), "Initial pair not selected");
    require(mapper.pending_pair.source_stamp_valid && mapper.pending_pair.source_stamp_ns == 10050000000ULL,
            "Original image stamp not retained");
    // Deliberately seed old output. The init return must never promote it.
    PointType old{}; old.x = 5.; mapper.feats_undistort->push_back(old);
    mapper.pcl_w_wait_pub->push_back(old);
    mapper.handleFirstFrame(); mapper.processImu(); mapper.handleLIO();
    require(!mapper.p_imu->imu_need_init && mapper.feats_undistort->empty(),
            "Initialization completion reused an old deskew output");
    require(mapper.pending_pair.phase == PendingLivoPair::Skip &&
                mapper.pending_pair.outcome == PendingLivoPair::Initializing,
            "Initialization pair falsely became LIO ready");
    require(!mapper.sync_packages(m) && mapper.img_buffer.empty() && m.lio_vio_flg == WAIT,
            "Initialization skip depended on next raw/IMU or retained its image");
    require(mapper.pending_pair.retired == 1 && mapper.pending_pair.skipped == 1 &&
                mapper.pcl_w_wait_pub->empty(), "Initialization skip not consumed exactly once");
    require(!mapper.sync_packages(m) && mapper.pending_pair.retired == 1,
            "Repeated sync consumed skipped pair twice");
}
void select_mature(LIVMapper &mapper, double start = 10.1) {
    mapper.receive_cloud(cloud(start, {0, 25000000, 50000000, 75000000, 100000000}));
    image(mapper, start + .05);
    for (int i = 1; i <= 10; ++i) mapper.receive_imu(imu(start - .05 + i * .01));
    require(mapper.sync_packages(mapper.LidarMeasures), "Mature LIO not selected");
}
void equal_state(const StatesGroup &a, const StatesGroup &b, const char *why) {
    require((a.rot_end-b.rot_end).norm() == 0 && (a.pos_end-b.pos_end).norm() == 0 &&
        (a.vel_end-b.vel_end).norm() == 0 && (a.bias_g-b.bias_g).norm() == 0 &&
        (a.bias_a-b.bias_a).norm() == 0 && (a.gravity-b.gravity).norm() == 0 &&
        (a.cov-b.cov).norm() == 0 && a.inv_expo_time == b.inv_expo_time, why);
}
void check_success() {
    auto n = node("pair_success", {{"common.same_pair_vio_handoff", true}}, true); LIVMapper mapper(n, n->get_name());
    image_transport::ImageTransport images(n); mapper.initializeSubscribersAndPublishers(n, images);
    initial_pair(mapper); select_mature(mapper);
    auto &m = mapper.LidarMeasures;
    require(mapper.lid_raw_data_buffer.empty() && mapper.imu_buffer.empty(),
            "Fixture did not exhaust new raw/IMU queues");
    const auto input = *m.pcl_proc_cur;
    const auto future = *m.pcl_proc_next;
    require(!mapper.sync_packages(m), "Selected flag alone admitted VIO");
    mapper.processImu();
    require(!mapper.sync_packages(m), "IMU completion alone admitted VIO");
    mapper.handleLIO();
    require(mapper.pending_pair.phase == PendingLivoPair::LioReady,
            "Normal finite LIO completion not marked ready");
    const StatesGroup posterior = mapper._state;
    const auto imu_tail = mapper.p_imu->last_imu_sample();
    const auto target = mapper.p_imu->propagation_end_time();
    const auto pub = *mapper.pcl_w_wait_pub;
    const auto pv_size = mapper._pv_list.size();
    const auto map_size = mapper.voxelmap_manager->voxel_map_.size();
    require(mapper.sync_packages(m) && m.lio_vio_flg == VIO, "Same pair waited for unrelated new raw/IMU");
    require(mapper.img_buffer.empty() && mapper.img_source_stamp_buffer.empty() &&
            m.measures.size() == 1 && m.measures.back().imu.empty() &&
            m.measures.back().lio_time == target && m.measures.back().vio_time == target,
            "VIO handoff changed target or consumed extra IMU/image");
    require(!mapper.sync_packages(m), "Next LIO admitted before VIO finished");
    mapper.processImu();
    equal_state(mapper._state, posterior, "Zero-duration VIO Process2 changed posterior/covariance");
    equal_state(mapper.state_propagat, posterior, "VIO prior is not current LIO posterior");
    require(mapper.p_imu->last_imu_sample() == imu_tail && mapper.p_imu->propagation_end_time() == target &&
            m.last_lio_update_time == target && mapper.feats_undistort->empty(),
            "VIO repeated integration/deskew or advanced IMU target");
    require(mapper.pcl_w_wait_pub->size() == pub.size() && mapper._pv_list.size() == pv_size &&
            mapper.voxelmap_manager->voxel_map_.size() == map_size &&
            m.pcl_proc_cur->size() == input.size() && m.pcl_proc_next->size() == future.size(),
            "VIO prepare damaged cached LIO output or next-slice ownership");
    for (size_t i = 0; i < input.size(); ++i)
        require((*m.pcl_proc_cur)[i].x == input[i].x && (*m.pcl_proc_cur)[i].curvature == input[i].curvature,
                "VIO handoff changed source point geometry/timing");
    mapper.handleVIO();
    require(mapper.pending_pair.retired == 2 && mapper.pending_pair.skipped == 1 &&
                mapper.pending_pair.outcome == PendingLivoPair::VioCompleted,
            "Successful VIO did not retire its pair exactly once");
    require(!mapper.sync_packages(m) && mapper.pending_pair.retired == 2,
            "Retired same pair was consumed twice");
    fastlivo_sync::flush(mapper);
    std::cout << "EQUIVALENCE " << m.lio_vio_flg << ' ' << m.last_lio_update_time << ' '
              << mapper.pending_pair.sequence << ' ' << mapper.pending_pair.retired << ' '
              << mapper.pending_pair.skipped << ' ' << mapper._state.cov.trace() << ' '
              << mapper._pv_list.size() << ' ' << m.pcl_proc_next->size() << '\n';
    std::cout << "PASS real LIO completion, empty new queues, same-time VIO, unchanged prior/cov/IMU/point ownership\n";
}
void check_rejection() {
    for (int mode = 0; mode < 7; ++mode) {
        auto n = node("pair_reject_" + std::to_string(mode), {{"common.same_pair_vio_handoff", true}}, true);
        LIVMapper mapper(n, n->get_name()); image_transport::ImageTransport images(n);
        mapper.initializeSubscribersAndPublishers(n, images); initial_pair(mapper); select_mature(mapper);
        auto &m = mapper.LidarMeasures;
        if (mode == 0) {
            // Empty current slice, with deliberately retained previous output.
            m.pcl_proc_cur->clear();
            mapper.processImu(); mapper.handleLIO();
            require(mapper.pending_pair.phase == PendingLivoPair::Skip, "Empty LIO became ready");
        } else {
            mapper.processImu(); mapper.handleLIO();
            require(mapper.pending_pair.phase == PendingLivoPair::LioReady, "Rejection fixture not ready");
            if (mode == 1) mapper.img_time_buffer.front() += .01;
            if (mode == 2) mapper.img_source_stamp_buffer.front() += 1;
            if (mode == 3) {
                cv::Mat replacement(240, 320, CV_8UC3, cv::Scalar(0));
                require(replacement.data != mapper.img_buffer.front().data, "Replacement fixture reused image storage");
                mapper.img_buffer.front() = replacement;
            }
            if (mode == 6) mapper.img_source_stamp_buffer.clear();
            if (mode == 4) mapper.stop_ingress(false);
            if (mode == 5) {
                bool failed = false;
                try { mapper.handleLIO(); } catch (const std::logic_error &) { failed = true; }
                require(failed && mapper.pending_pair.outcome == PendingLivoPair::Aborted,
                        "Exception left a reusable pair token");
            }
        }
        require(!mapper.sync_packages(m), "Invalid/skipped/stopped pair admitted VIO");
        require(mapper.pending_pair.retired == 2 && mapper.pending_pair.skipped == 2,
                "Invalid pair did not retire once");
        require(!mapper.sync_packages(m) && mapper.pending_pair.retired == 2,
                "Invalid pair retired more than once");
    }
    std::cout << "PASS empty/mismatched image time/source stamp/object, stop, and exception pair rejection\n";
}
StatesGroup run_toggle(bool enabled) {
    auto n = node(enabled ? "pair_flag_on" : "pair_flag_off",
                  {{"common.same_pair_vio_handoff", enabled}}, true);
    LIVMapper mapper(n, n->get_name()); image_transport::ImageTransport images(n);
    mapper.initializeSubscribersAndPublishers(n, images);
    auto &m = mapper.LidarMeasures;
    if (enabled) { initial_pair(mapper); select_mature(mapper); }
    else {
        mapper.receive_cloud(cloud(10., {0, 25000000, 50000000, 75000000, 100000000}));
        image(mapper, 10.05);
        for (int tick = 1000; tick <= 1005; ++tick) mapper.receive_imu(imu(tick / 100.));
        require(mapper.sync_packages(m), "Legacy initial LIO not selected");
        mapper.handleFirstFrame(); mapper.processImu(); mapper.handleLIO();
        require(!mapper.sync_packages(m) && m.lio_vio_flg == LIO && mapper.img_buffer.size() == 1,
                "Disabled handoff changed legacy initialization raw gate/skip timing");
        mapper.receive_cloud(cloud(10.1, {0, 25000000, 50000000, 75000000, 100000000}));
        require(!mapper.sync_packages(m), "Disabled handoff bypassed legacy empty IMU gate");
        mapper.receive_imu(imu(10.06));
        require(mapper.sync_packages(m) && m.lio_vio_flg == VIO, "Legacy initialization VIO not selected");
        mapper.processImu(); mapper.handleVIO();
        image(mapper, 10.15);
        for (int tick = 1007; tick <= 1015; ++tick) mapper.receive_imu(imu(tick / 100.));
        require(mapper.sync_packages(m), "Legacy mature LIO not selected");
    }
    mapper.processImu(); mapper.handleLIO();
    const auto posterior = mapper._state;
    const auto target = mapper.p_imu->propagation_end_time();
    if (!enabled) {
        require(!mapper.sync_packages(m) && mapper.img_buffer.size() == 1,
                "Disabled handoff bypassed legacy successful LIO raw gate");
        mapper.receive_cloud(cloud(10.2, {0, 100000000}));
        require(!mapper.sync_packages(m), "Disabled handoff bypassed legacy successful LIO IMU gate");
        mapper.receive_imu(imu(10.16));
    }
    require(mapper.sync_packages(m) && m.lio_vio_flg == VIO, "Toggle VIO not selected");
    mapper.processImu();
    equal_state(mapper._state, posterior, "Toggle changed zero-duration VIO propagation");
    equal_state(mapper.state_propagat, posterior, "Toggle changed VIO posterior-to-prior handoff");
    require(mapper.p_imu->propagation_end_time() == target, "Toggle changed propagation target");
    mapper.handleVIO();
    return mapper._state;
}
void check_toggle() {
    const auto baseline = run_toggle(false), fixed = run_toggle(true);
    // Different arrival schedules have the same physical sensor measurements.
    // Floating subtraction paths in the synthetic IMU generator are avoided.
    equal_state(baseline, fixed, "Flag on/off changed same-pair state/covariance");
    std::cout << "PASS same binary flag OFF legacy gates/init timing and ON identical state/cov/prior\n";
}

void check_capacity() {
    for (int mode = 0; mode < 3; ++mode) {
        Parameters p{{"common.max_buffered_images", 2}, {"common.max_buffered_imu", 2},
                     {"common.max_buffered_prop_imu", 2}, {"uav.imu_rate_odom", mode == 2}};
        auto n = node("pair_capacity_" + std::to_string(mode), p, true);
        LIVMapper mapper(n, n->get_name());
        mapper.receive_cloud(cloud(20., {0, 100000000}));
        if (mode == 0) { image(mapper, 20.01); image(mapper, 20.04); }
        else {
            if (mode == 2) mapper.p_imu->imu_need_init = false;
            mapper.receive_imu(imu(20.01)); mapper.receive_imu(imu(20.02));
            if (mode == 2) mapper.imu_buffer.clear();
        }
        const auto images = mapper.img_buffer.size(), times = mapper.img_time_buffer.size();
        const auto sources = mapper.img_source_stamp_buffer.size(), imus = mapper.imu_buffer.size();
        const auto props = mapper.prop_imu_buffer.size();
        const auto last_img = mapper.last_timestamp_img, last_imu = mapper.last_timestamp_imu;
        bool rejected = false;
        try { if (mode == 0) image(mapper, 20.08); else mapper.receive_imu(imu(20.03)); }
        catch (const std::runtime_error &e) {
            const std::string expected = mode == 0 ? "img_buffer" : mode == 1 ? "imu_buffer" : "prop_imu_buffer";
            rejected = std::string(e.what()).find("sensor cache capacity: " + expected) != std::string::npos;
        }
        require(rejected && mapper.img_buffer.size() == images && mapper.img_time_buffer.size() == times &&
            mapper.img_source_stamp_buffer.size() == sources && mapper.imu_buffer.size() == imus &&
            mapper.prop_imu_buffer.size() == props && mapper.last_timestamp_img == last_img &&
            mapper.last_timestamp_imu == last_imu, "Cache overflow silently evicted or partly committed input");
        require(mapper.mtx_buffer.try_lock(), "Cache overflow leaked sensor mutex"); mapper.mtx_buffer.unlock();
    }
    auto n = node("pair_capacity_default"); LIVMapper mapper(n, n->get_name());
    require(!mapper.max_buffered_images && !mapper.max_buffered_imu && !mapper.max_buffered_prop_imu,
            "Explicit cache guard changed legacy defaults");
    std::cout << "PASS explicit image/IMU/prop-IMU hard caps, no eviction/partial commit, legacy defaults\n";
}

}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try { check_success(); check_rejection(); check_toggle(); check_capacity(); rclcpp::shutdown(); return 0; }
    catch (const std::exception &e) { std::cerr << "FAIL " << e.what() << '\n'; rclcpp::shutdown(); return 1; }
}
