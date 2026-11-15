// Reuse the original native synthetic input helpers without altering its three tests.
#define main original_ouster_fixture_main
#include "test_ouster_mode.cpp"
#undef main
#include <fast_livo2_core/core/sync_observation.h>
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto n = node("sync_observation_boundary", {{"common.same_pair_vio_handoff", true}}, true);
        LIVMapper mapper(n, n->get_name());
        image_transport::ImageTransport images(n);
        mapper.initializeSubscribersAndPublishers(n, images);
        auto &m = mapper.LidarMeasures;
        require(!mapper.sync_packages(m), "Empty raw gate changed");
        mapper.receive_cloud(cloud(10., {100000000, 0, 50000000, 25000000, 75000000}));
        require(!mapper.sync_packages(m), "Empty image gate changed");
        m.last_lio_update_time = 10.;
        mapper.img_buffer.push_back(cv::Mat::zeros(240, 320, CV_8UC1));
        mapper.img_time_buffer.push_back(10.05);
        mapper.img_source_stamp_buffer.push_back(10050000000ULL);
        require(!mapper.sync_packages(m), "Empty IMU gate changed");
        for (double t : {10.01, 10.02, 10.04}) mapper.receive_imu(imu(t));
        require(!mapper.sync_packages(m), "Incomplete IMU was accepted");
        require(mapper.lid_raw_data_buffer.size() == 1 && mapper.imu_buffer.size() == 3,
                "Incomplete sync consumed input evidence");
        mapper.receive_imu(imu(10.05)); mapper.receive_imu(imu(10.06));
        require(mapper.sync_packages(m), "Complete pair selection changed");
        require(m.lio_vio_flg == LIO && m.pcl_proc_cur->size() == 2 && m.pcl_proc_next->size() == 3,
                "Point slicing changed");
        mapper.handleFirstFrame(); mapper.processImu(); mapper.handleLIO();
        require(std::abs(m.last_lio_update_time - 10.05) < 1e-9, "Initialization origin changed");
        require(!mapper.sync_packages(m), "Initializing LIO was admitted to VIO");
        require(m.lio_vio_flg == WAIT && mapper.img_buffer.empty() &&
                    mapper.pending_pair.retired == 1 && mapper.pending_pair.skipped == 1,
                "Initialization did not retire its own pair exactly once");
        mapper.receive_cloud(cloud(10.1, {0, 10000000}));
        mapper.img_buffer.push_back(cv::Mat::zeros(240, 320, CV_8UC1));
        mapper.img_time_buffer.push_back(10.15);
        mapper.img_source_stamp_buffer.push_back(10150000000ULL);
        require(!mapper.sync_packages(m), "True point-coverage gate was bypassed");
        mapper.receive_cloud(cloud(10.14, {0, 100000000}));
        require(!mapper.sync_packages(m), "Full image IMU coverage gate was bypassed");
        mapper.receive_imu(imu(10.16));
        require(mapper.sync_packages(m) && m.lio_vio_flg == LIO,
                "Complete second image pair changed");
        fastlivo_sync::flush(mapper);
        std::cout << "EQUIVALENCE " << m.lio_vio_flg << ' ' << m.last_lio_update_time << ' '
                  << m.pcl_proc_cur->size() << ' ' << m.pcl_proc_next->size() << ' '
                  << mapper.lid_raw_data_buffer.size() << ' ' << mapper.img_buffer.size() << ' '
                  << mapper.imu_buffer.size() << ' ' << mapper.p_imu->imu_need_init << '\n';
        std::cout << "PASS first-stage raw/image/IMU gates, true coverage, initialization skip, and point ownership\n";
        rclcpp::shutdown(); return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL " << e.what() << '\n';
        rclcpp::shutdown(); return 1;
    }
}
