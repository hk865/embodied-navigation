#include <optional>
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <unordered_set>
#include <cstdio>
#include <exception>
#include <nlohmann/json.hpp>
#include <map>
#include <set>
#include "diagnostics.h"
// BOUNDARY_TIMING_BEGIN include
#include "boundary_timing.h"
// BOUNDARY_TIMING_END include
#include <chrono>
#include <filesystem>
#include <thread>
#include <pthread.h>
#include <iomanip>
#include <fast_livo2_core/runner.hpp>
#include <limits>
#include <sys/syscall.h>
#include <unistd.h>
#include <vikit/camera_loader.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace Sophus;
namespace {
uint64_t diag_stamp(const builtin_interfaces::msg::Time& stamp) {
  return uint64_t(stamp.sec)*1000000000ULL+uint64_t(stamp.nanosec);
}
uint64_t ingress_thread_cpu() { timespec t{}; if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t)!=0) throw std::runtime_error("thread CPU clock failed"); return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec; }
double diag_receipt() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void diag_stage(uint64_t kind,double flag,const StatesGroup& state) {
  auto& d=fastlivo_diag::Logger::instance();
  if(!d.enabled()) return;
  std::vector<double> values{flag,diag_receipt()};
  fastlivo_diag::state(values,state); fastlivo_diag::append(values,state.cov);
  d.emit(kind,-1,-1,std::move(values));
}
}
LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name)
    : node(node ? node : std::make_shared<rclcpp::Node>(node_name)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  (void)fastlivo_diag::Logger::instance();
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  const char* option=std::getenv("FASTLIVO_PIPELINE_MODE");
  pipeline_mode=option?option:"serial";
  if(pipeline_mode!="serial"&&pipeline_mode!="rx_decode"&&pipeline_mode!="staged")throw std::runtime_error("FASTLIVO_PIPELINE_MODE must be serial, rx_decode or staged");
  const char* copy=std::getenv("FASTLIVO_IMAGE_COPY_OPT");
  if(copy&&std::string(copy)!="0"&&std::string(copy)!="1")throw std::runtime_error("FASTLIVO_IMAGE_COPY_OPT must be 0 or 1");
  image_copy_opt=copy&&std::string(copy)=="1";
  const char* imu_priority=std::getenv("FASTLIVO_SERIAL_IMU_PRIORITY");
  if(imu_priority&&std::string(imu_priority)!="0"&&std::string(imu_priority)!="1")
    throw std::runtime_error("FASTLIVO_SERIAL_IMU_PRIORITY must be 0 or 1");
  serial_imu_priority=imu_priority&&std::string(imu_priority)=="1";
  if(serial_imu_priority&&pipeline_mode!="serial")
    throw std::runtime_error("FASTLIVO_SERIAL_IMU_PRIORITY requires serial mode");
  if(p_pre->lidar_type!=0||use_gps||hilti_en||this->node->get_parameter("preprocess.hilti_en").as_bool()||!lidar_en||!img_en||!imu_en)
    throw std::runtime_error("V19 requires generic cloud, IMU+RGB, GPS off, HILTI off");
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  initializeBackendSnapshots();
  path.header.stamp = this->node->now();
  path.header.frame_id = odom_frame;
}

LIVMapper::~LIVMapper() {try{stop_ingress(false);}catch(...) {}}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // declare parameters
  this->node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->node->declare_parameter<bool>("common.require_complete_imu", false);
  this->node->declare_parameter<bool>("debug.sensor_sync_diagnostics", false);
  this->node->declare_parameter<int>("common.img_en", 1);
  this->node->declare_parameter<int>("common.lidar_en", 1);
  this->node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  this->node->declare_parameter<bool>("vio.normal_en", true);
  this->node->declare_parameter<bool>("vio.inverse_composition_en", false);
  this->node->declare_parameter<int>("vio.max_iterations", 5);
  this->node->declare_parameter<int>("vio.img_point_cov", 100);
  this->node->declare_parameter<bool>("vio.raycast_en", false);
  this->node->declare_parameter<bool>("vio.exposure_estimate_en", true);
  this->node->declare_parameter<double>("vio.inv_expo_cov", 0.1);
  this->node->declare_parameter<int>("vio.grid_size", 5);
  this->node->declare_parameter<int>("vio.grid_n_height", 17);
  this->node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
  this->node->declare_parameter<int>("vio.patch_size", 8);
  this->node->declare_parameter<int>("vio.outlier_threshold", 100);
  this->node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  this->node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->node->declare_parameter<bool>("uav.imu_rate_odom", false);
  this->node->declare_parameter<bool>("uav.gravity_align_en", false);

  this->node->declare_parameter<std::string>("evo.seq_name", "01");
  this->node->declare_parameter<bool>("evo.pose_output_en", false);
  this->node->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->node->declare_parameter<double>("imu.acc_cov", 1.0);
  this->node->declare_parameter<int>("imu.imu_int_frame", 30);
  this->node->declare_parameter<bool>("imu.stationary_initialization_en", false);
  this->node->declare_parameter<double>("imu.init_max_gyro_norm", .03);
  this->node->declare_parameter<double>("imu.init_max_acc_norm_error", 1.2);
  this->node->declare_parameter<double>("imu.init_max_acc_axis_std", .40);
  this->node->declare_parameter<double>("imu.init_max_sample_gap", .03);
  this->node->declare_parameter<double>("imu.init_min_span", 2.9);
  this->node->declare_parameter<bool>("imu.imu_en", true);
  this->node->declare_parameter<bool>("imu.gravity_est_en", true);
  this->node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->node->declare_parameter<double>("preprocess.blind", 0.01);
    this->node->declare_parameter<bool>("preprocess.hilti_en", false);
  this->node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->node->declare_parameter<int>("preprocess.scan_line",6);
  this->node->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->node->declare_parameter<int>("pcd_save.interval", -1);
  this->node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->node->declare_parameter<bool>("image_save.img_save_en", false);
  this->node->declare_parameter<int>("image_save.interval", 1);

  this->node->declare_parameter<int>("pcd_save.type", 0);
  this->node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>{});
  this->node->declare_parameter<double>("debug.plot_time", -10);
  this->node->declare_parameter<int>("debug.frame_cnt", 6);

  this->node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  this->node->declare_parameter<int>("publish.pub_scan_num", 1);
  this->node->declare_parameter<bool>("publish.pub_effect_point_en", false);
  this->node->declare_parameter<bool>("publish.dense_map_en", false);
  this->node->declare_parameter<bool>("publish.publish_base_tf", false);
  this->node->declare_parameter<std::string>("publish.base_frame", "base_footprint");
  this->node->declare_parameter<std::string>("publish.odom_frame", "camera_init");
  this->node->declare_parameter<std::string>("publish.child_frame", "aft_mapped");

  // ---- GPS / Ground Truth Fusion ----
  this->node->declare_parameter<bool>("common.use_gps", false);
  this->node->declare_parameter<double>("common.gps_cov", 0.01);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.require_complete_imu", require_complete_imu);
  this->node->get_parameter("debug.sensor_sync_diagnostics", sensor_sync_diagnostics);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->node->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.type", pcd_save_type);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("image_save.img_save_en", img_save_en);
  this->node->get_parameter("image_save.interval", img_save_interval);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
  this->node->get_parameter("publish.publish_base_tf", publish_base_tf);
  this->node->get_parameter("publish.base_frame", base_frame);
  this->node->get_parameter("publish.odom_frame", odom_frame);
  this->node->get_parameter("publish.child_frame", raw_body_frame);

  // GPS / ground truth fusion
  this->node->get_parameter("common.use_gps", use_gps);
  this->node->get_parameter("common.gps_cov", gps_cov);
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  
  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs(this->node, "parameter_blackboard", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->lidar_type = p_pre->lidar_type;
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  StationaryImuInitialization::Limits initialization_limits;
  bool stationary_initialization_en = false;
  node->get_parameter("imu.stationary_initialization_en", stationary_initialization_en);
  node->get_parameter("imu.init_max_gyro_norm", initialization_limits.gyro_norm);
  node->get_parameter("imu.init_max_acc_norm_error", initialization_limits.acceleration_norm_error);
  node->get_parameter("imu.init_max_acc_axis_std", initialization_limits.acceleration_axis_std);
  node->get_parameter("imu.init_max_sample_gap", initialization_limits.max_sample_gap);
  node->get_parameter("imu.init_min_span", initialization_limits.min_span);
  p_imu->configure_stationary_initialization(stationary_initialization_en, initialization_limits);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  const std::filesystem::path &log_dir = log_directory;
  std::error_code create_error;
  std::filesystem::create_directories(log_dir, create_error);
  if (create_error)
  {
    RCLCPP_WARN(
        this->node->get_logger(),
        "Cannot create log directory '%s': %s. File logging is disabled.",
        log_dir.c_str(), create_error.message().c_str());
    pcd_save_en = false;
    img_save_en = false;
    pose_output_en = false;
    colmap_output_en = false;
    return;
  }

  if(std::getenv("FASTLIVO_DISABLE_STATE_LOG") && std::string(std::getenv("FASTLIVO_DISABLE_STATE_LOG"))=="1")return;

  auto ensure_output_directory = [this](const std::filesystem::path &directory,
                                        const char *output_name) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error)
    {
      RCLCPP_ERROR(
          this->node->get_logger(),
          "Cannot create %s directory '%s': %s. This output is disabled.",
          output_name, directory.c_str(), error.message().c_str());
      return false;
    }
    return true;
  };

  if (pcd_save_en &&
      !ensure_output_directory(log_dir / "pcd", "PCD"))
    pcd_save_en = false;
  if (img_save_en &&
      !ensure_output_directory(log_dir / "image", "image"))
    img_save_en = false;
  if (pose_output_en)
    ensure_output_directory(log_dir / "result", "pose");

  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open((log_dir / "mat_pre.txt").string(), std::ios::out);
  fout_out.open((log_dir / "mat_out.txt").string(), std::ios::out);
  RCLCPP_INFO(node->get_logger(), "[DEMO_LOG] directory=%s", log_dir.c_str());
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it_)
{
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = this->node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, rclcpp::SensorDataQoS().keep_last(4), std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS().keep_last(4), std::bind(&LIVMapper::receive_cloud, this, std::placeholders::_1));
  }
  if(serial_imu_priority) {
    imu_priority_group=this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group=imu_priority_group;
    sub_imu=this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic,rclcpp::SensorDataQoS().keep_last(512),std::bind(&LIVMapper::receive_imu,this,std::placeholders::_1),imu_options);
  } else {
    sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(512), std::bind(&LIVMapper::receive_imu, this, std::placeholders::_1));
  }
  sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(img_topic, rclcpp::SensorDataQoS().keep_last(4), std::bind(&LIVMapper::receive_image, this, std::placeholders::_1));
  if (use_gps) {
    sub_gps = this->node->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth/odometry", 10, std::bind(&LIVMapper::gps_cbk, this, std::placeholders::_1));
    RCLCPP_INFO(this->node->get_logger(), "[ GPS ] GPS/ground-truth fusion ENABLED");
  }
  
  pubNavigationCloud = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_full", 10);
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = this->node->create_publisher<sensor_msgs::msg::Image>("/rgb_img", 1);
  RCLCPP_INFO(this->node->get_logger(), "[ INIT ] rgb_img publisher created, node=%s serial_imu_priority=%d", this->node->get_name(),int(bool(imu_priority_group)));
  pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::receive_tick, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  auto& diag=fastlivo_diag::Logger::instance();
  if(diag.enabled()) {
    const auto& m=LidarMeasures.measures.back();
    const double target=LidarMeasures.lio_vio_flg==LIO?m.lio_time:m.vio_time;
    diag.set_context(LidarMeasures.lio_vio_flg,fastlivo_diag::nanoseconds(target));
    std::vector<double> values{diag_receipt(),LidarMeasures.lidar_frame_beg_time,
      LidarMeasures.lidar_frame_end_time,LidarMeasures.last_lio_update_time,m.lio_time,m.vio_time,
      double(m.imu.size()),double(p_imu->imu_need_init),double(gravity_align_en),double(gravity_align_finished),_first_lidar_time};
    double front=-1,back=-1,maxgap=0;
    if(!m.imu.empty()) {
      front=double(diag_stamp(m.imu.front()->header.stamp)); back=double(diag_stamp(m.imu.back()->header.stamp));
      for(size_t i=1;i<m.imu.size();++i)
        maxgap=std::max(maxgap,stamp2Sec(m.imu[i]->header.stamp)-stamp2Sec(m.imu[i-1]->header.stamp));
    }
    values.insert(values.end(),{front,back,maxgap});
    fastlivo_diag::state(values,_state); fastlivo_diag::append(values,_state.cov);
    diag.emit(10,-1,-1,std::move(values));
  }
  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  if(diag.enabled()) {
    std::vector<double> values{diag_receipt(),LidarMeasures.last_lio_update_time,double(p_imu->imu_need_init),
      double(gravity_align_finished),double(feats_undistort->size())};
    fastlivo_diag::state(values,_state); fastlivo_diag::append(values,_state.cov);
    diag.emit(11,-1,-1,std::move(values));
  }
  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
  // GPS / ground-truth fusion: correct ESIKF state with absolute position
  if (use_gps && gps_received) {
    gpsUpdate();
  }
}

void LIVMapper::handleVIO() 
{
// BOUNDARY_TIMING_BEGIN handle_vio_all
  fastlivo_timing::Scope timing_handle_vio_all(15);
// BOUNDARY_TIMING_END handle_vio_all
  // fprintf(stderr, "[ VIO ] handleVIO entered, img empty=%d\n", LidarMeasures.measures.empty() ? -1 : LidarMeasures.measures.back().img.empty());
  // fflush(stderr);
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (!pcl_w_wait_pub || pcl_w_wait_pub->empty()) 
  {
    diag_stage(12,0,_state);
    backendVioSnapshot(false);
    return;
  }

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

// BOUNDARY_TIMING_BEGIN vio_process_open
  { fastlivo_timing::Scope timing_vio_process(6);
// BOUNDARY_TIMING_END vio_process_open
  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);
// BOUNDARY_TIMING_BEGIN vio_process_close
  }
// BOUNDARY_TIMING_END vio_process_close

  backendVioSnapshot(vio_manager->last_frame_ekf_updated);
  diag_stage(12,1,_state);
  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{
  fastlivo_timing::frame_mark(4);    
// BOUNDARY_TIMING_BEGIN handle_lio_all
  fastlivo_timing::Scope timing_handle_lio_all(14);
// BOUNDARY_TIMING_END handle_lio_all
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (!feats_undistort || feats_undistort->empty()) 
  {
    diag_stage(13,0,_state);
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  diag_stage(13,1,_state);
  _pv_list = voxelmap_manager->pv_list_;
  backendLioSnapshot();

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  fastlivo_timing::frame_mark(5);
  publish_odometry(pubOdomAftMapped);
  fastlivo_timing::frame_mark(6);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  if(fastlivo_diag::Logger::instance().detail()) {
    std::vector<double> values{double(voxelmap_manager->pv_list_.size()),15};
    fastlivo_diag::state(values,_state);
    for(const auto& pv:voxelmap_manager->pv_list_) {
      fastlivo_diag::append(values,pv.point_b); fastlivo_diag::append(values,pv.point_w); fastlivo_diag::append(values,pv.var);
    }
    fastlivo_diag::Logger::instance().emit(14,-1,-1,std::move(values));
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  // Occupancy mapping needs the complete deskewed scan, including points
  // outside the camera field of view. Keep the coloured topic unchanged.
// BOUNDARY_TIMING_BEGIN navigation_cloud_open
  { fastlivo_timing::Scope timing_navigation_cloud(7);
// BOUNDARY_TIMING_END navigation_cloud_open
  fastlivo_timing::frame_mark(7);
  sensor_msgs::msg::PointCloud2 navigation_cloud;
  pcl::toROSMsg(*laserCloudWorld, navigation_cloud);
  navigation_cloud.header.frame_id = odom_frame;
  navigation_cloud.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  fastlivo_timing::frame_mark(8);
  pubNavigationCloud->publish(navigation_cloud);
  fastlivo_timing::frame_mark(9);
// BOUNDARY_TIMING_BEGIN navigation_cloud_close
  }
// BOUNDARY_TIMING_END navigation_cloud_close

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      try
      {
        pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      }
      catch (const pcl::IOException &error)
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to save final raw RGB PCD: %s", error.what());
        return;
      }
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      try
      {
        pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      }
      catch (const pcl::IOException &error)
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to save final downsampled PCD: %s", error.what());
        return;
      }
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      try
      {
        pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      }
      catch (const pcl::IOException &error)
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to save final intensity PCD: %s", error.what());
        return;
      }
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  int status_interval = 0;
  start_ingress();
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> serial_executor;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> imu_priority_executor;
  try {
  if (pipeline_mode == "serial") {
    serial_executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    serial_executor->add_node(this->node);
    if(serial_imu_priority) {
      imu_priority_executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      imu_priority_executor->add_callback_group(imu_priority_group,this->node->get_node_base_interface());
    }
  }
  while (rclcpp::ok()) 
  {
// BOUNDARY_TIMING_BEGIN spin_some_open
  { fastlivo_timing::Scope timing_spin_some(8);
// BOUNDARY_TIMING_END spin_some_open
    if(fast_livo2_core::mapping_stop_requested())break;
    // One owner and original callbacks, with a cooperative1ms ready-work
    // budget. Re-collect queued sensor work without dropping old IMU samples.
    // A running callback is not preempted by the executor time budget.
    // Both passes execute on this owner thread, before receive assigns a sequence.
    if(imu_priority_executor) {
      imu_priority_executor->spin_all(std::chrono::milliseconds(1));
      if(!rclcpp::ok())break;
    }
    if(pipeline_mode=="serial")serial_executor->spin_all(std::chrono::milliseconds(1));
    else commit_ingress();
// BOUNDARY_TIMING_BEGIN spin_some_close
  }
// BOUNDARY_TIMING_END spin_some_close
    // SIGINT may invalidate the context while spin_some() is returning.  Do
    // not enter one more synchronization / mapping cycle with that context.
    if (!rclcpp::ok()) break;
    // Periodic status: print buffer sizes every ~1s (5000 iterations)
    if (++status_interval >= 5000) {
      status_interval = 0;
      if (img_en || lidar_en || imu_en) {
        RCLCPP_INFO(this->node->get_logger(),
          "[ STATUS ] lid_buf=%zu img_buf=%zu imu_buf=%zu first_frame=%d lio_time=%.1f",
          lid_raw_data_buffer.size(), img_buffer.size(), imu_buffer.size(),
          (int)is_first_frame, LidarMeasures.last_lio_update_time);
      }
    }
    // fprintf(stderr, "[ Main ] after spin_some\n"); fflush(stderr);
    fastlivo_timing::frame_reset();
    if (!sync_packages(LidarMeasures)) 
    {
      // rclcpp::Rate::sleep() may throw after SIGINT invalidates the context,
      // turning a normal launch shutdown into an abort before final PCD save.
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    // fprintf(stderr, "[ Main ] after sync_packages\n"); fflush(stderr);
    fastlivo_timing::frame_mark(1);
    handleFirstFrame();

    // fprintf(stderr, "[ Main ] before processImu\n"); fflush(stderr);
    fastlivo_timing::frame_mark(2);
    processImu();
    fastlivo_timing::frame_mark(3);
    // fprintf(stderr, "[ Main ] before stateEstimation\n"); fflush(stderr);

    stateEstimationAndMapping();
    // fprintf(stderr, "[ Main ] after stateEstimation\n"); fflush(stderr);
  }
  }catch(...){
    // PRIMARY_EXCEPTION_PRESERVATION_BEGIN
    const auto primary_exception = std::current_exception();
    try { ingress_queue.fail("owner run exception"); } catch (...) {}
    try { stop_ingress(false); }
    catch (const std::exception& cleanup_error) {
      std::fprintf(stderr, "V19 secondary cleanup failure: %.512s\n", cleanup_error.what());
    } catch (...) {
      std::fprintf(stderr, "V19 secondary cleanup failure: non-std exception\n");
    }
    std::rethrow_exception(primary_exception);
    // PRIMARY_EXCEPTION_PRESERVATION_END
  }
  stop_ingress(rclcpp::ok()&&fast_livo2_core::mapping_stop_requested());
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制 propagate 频率和 IMU 频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stamp2Sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stamp2Sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
// BOUNDARY_TIMING_BEGIN standard_lidar_callback
  fastlivo_timing::Scope timing_standard_lidar_callback(9);
// BOUNDARY_TIMING_END standard_lidar_callback
  if(fastlivo_diag::Logger::instance().enabled()) {
    std::vector<double> values{committing_ingress ? committing_ingress->receipt : diag_receipt(),double(lidar_en),lidar_time_offset,double(msg->width),double(msg->height),
      double(msg->point_step),double(msg->row_step)};
    fastlivo_diag::Logger::instance().raw(2,diag_stamp(msg->header.stamp),std::move(values));
  }
  if (!lidar_en) return;
  std::unique_lock<std::mutex> sensor_buffer_lock(mtx_buffer);

  double cur_head_time = stamp2Sec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    throw std::runtime_error("R60 source timestamp rollback requires a new owned run");
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr=committing_ingress ? committing_ingress->cloud : PointCloudXYZI::Ptr(new PointCloudXYZI());
// BOUNDARY_TIMING_BEGIN standard_preprocess_open
  { fastlivo_timing::Scope timing_standard_preprocess(12);
// BOUNDARY_TIMING_END standard_preprocess_open
  if(committing_ingress && committing_ingress->decode_error) std::rethrow_exception(committing_ingress->decode_error);
  if(!committing_ingress) p_pre->process(msg, ptr);
// BOUNDARY_TIMING_BEGIN standard_preprocess_close
  }
// BOUNDARY_TIMING_END standard_preprocess_close
  if(fastlivo_diag::Logger::instance().detail_at(cur_head_time)) {
    std::vector<double> values{double(ptr->size()),5,cur_head_time};
    values.reserve(3+ptr->size()*5);
    for(const auto& point:ptr->points)
      values.insert(values.end(),{point.x,point.y,point.z,point.intensity,point.curvature});
    fastlivo_diag::Logger::instance().raw(4,diag_stamp(msg->header.stamp),std::move(values));
  }
  if(lid_raw_data_buffer.size()>=8 || ptr->size()>65536)throw std::runtime_error("LiDAR buffered source capacity");
  lid_raw_data_buffer.push_back(PairedCloud{ptr});
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  sensor_buffer_lock.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
// BOUNDARY_TIMING_BEGIN livox_lidar_callback
  fastlivo_timing::Scope timing_livox_lidar_callback(9);
// BOUNDARY_TIMING_END livox_lidar_callback
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  // RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
// BOUNDARY_TIMING_BEGIN livox_preprocess_open
  { fastlivo_timing::Scope timing_livox_preprocess(12);
// BOUNDARY_TIMING_END livox_preprocess_open
  p_pre->process(msg, ptr);
// BOUNDARY_TIMING_BEGIN livox_preprocess_close
  }
// BOUNDARY_TIMING_END livox_preprocess_close

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(PairedCloud{ptr});
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
// BOUNDARY_TIMING_BEGIN imu_callback
  fastlivo_timing::Scope timing_imu_callback(10);
// BOUNDARY_TIMING_END imu_callback
  if(fastlivo_diag::Logger::instance().enabled()) {
    std::vector<double> values{committing_ingress ? committing_ingress->receipt : diag_receipt(),double(imu_en),last_timestamp_lidar,imu_time_offset,double(ros_driver_fix_en),
      msg_in->angular_velocity.x,msg_in->angular_velocity.y,msg_in->angular_velocity.z,
      msg_in->linear_acceleration.x,msg_in->linear_acceleration.y,msg_in->linear_acceleration.z,
      msg_in->orientation.x,msg_in->orientation.y,msg_in->orientation.z,msg_in->orientation.w};
    values.insert(values.end(),msg_in->orientation_covariance.begin(),msg_in->orientation_covariance.end());
    values.insert(values.end(),msg_in->angular_velocity_covariance.begin(),msg_in->angular_velocity_covariance.end());
    values.insert(values.end(),msg_in->linear_acceleration_covariance.begin(),msg_in->linear_acceleration_covariance.end());
    fastlivo_diag::Logger::instance().raw(1,diag_stamp(msg_in->header.stamp),std::move(values));
  }
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // RCLCPP_INFO(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  {
    // A late subscription or a temporary sensor dropout legitimately creates
    // a forward gap. Rejecting this sample freezes last_timestamp_imu, which
    // makes every later IMU sample look like another gap and deadlocks SLAM.
    RCLCPP_WARN(
        this->node->get_logger(),
        "IMU forward time gap of %.4f s; accepting the new stream head",
        timestamp - last_timestamp_imu);
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  if (img_msg->data.empty()) return cv::Mat();
  // 手动构造 cv::Mat 绕过 cv_bridge SIGSEGV
  if (img_msg->encoding == "rgb8" || img_msg->encoding == "bgr8") {
    cv::Mat img(img_msg->height, img_msg->width, CV_8UC3,
                (void*)img_msg->data.data(), img_msg->step);
    if (img_msg->encoding == "rgb8") {
      cv::Mat bgr;
      cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
      return image_copy_opt ? bgr : bgr.clone();
    }
    return img.clone();
  }
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
  return cv_ptr->image;
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
// BOUNDARY_TIMING_BEGIN image_callback
  fastlivo_timing::Scope timing_image_callback(11);
// BOUNDARY_TIMING_END image_callback
  if(fastlivo_diag::Logger::instance().enabled()) {
    std::vector<double> values{committing_ingress ? committing_ingress->receipt : diag_receipt(),double(img_en),img_time_offset,double(msg_in->width),double(msg_in->height),
      double(msg_in->step),double(hilti_en)};
    fastlivo_diag::Logger::instance().raw(3,diag_stamp(msg_in->header.stamp),std::move(values));
  }
  if (!img_en) return;
  sensor_msgs::msg::Image::ConstSharedPtr msg=image_copy_opt?msg_in:sensor_msgs::msg::Image::ConstSharedPtr(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  // RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  // Allow buffering images even before LiDAR arrives (needed for Gazebo simulation
  // where camera may start before LiDAR). sync_packages will handle the alignment.
  // if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(this->node->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  if(committing_ingress && committing_ingress->decode_error) std::rethrow_exception(committing_ingress->decode_error);
  cv::Mat img_cur = committing_ingress ? committing_ingress->image : getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

// ===== GPS / Ground Truth Fusion =====
void LIVMapper::gps_cbk(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
  bool first_time = !gps_received;
  gps_position << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  // Extract Euler angles from quaternion
  tf2::Quaternion q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(gps_orientation(0), gps_orientation(1), gps_orientation(2));
  gps_received = true;
  if (first_time) {
    RCLCPP_INFO(this->node->get_logger(),
        "[ GPS ] First GPS received: pos=(%.2f, %.2f, %.2f)",
        gps_position.x(), gps_position.y(), gps_position.z());
  }
}

void LIVMapper::gpsUpdate()
{
  if (!ekf_finish_once) return;

  // ---- Position observation ----
  V3D pos_residual = gps_position - _state.pos_end;
  
  // Build H matrix: 3 rows x 19 cols, only position columns (3-5) = identity
  Eigen::Matrix<double, 3, 19> H_pos = Eigen::Matrix<double, 3, 19>::Zero();
  H_pos.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  // Measurement covariance R (3x3)
  Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * gps_cov;
  Eigen::Matrix3d R_pos_inv = R_pos.inverse();

  // ---- ESIKF update for position (direct measurement, zero prior) ----
  // Note: GPS update happens AFTER LIO already corrected _state.
  // Using state_propagat - _state as prior would fight the LIO result.
  // Instead, use zero prior → pure measurement correction.
  Eigen::Matrix<double, 19, 19> H_T_H = H_pos.transpose() * R_pos_inv * H_pos;
  Eigen::Matrix<double, 19, 19> I_STATE = Eigen::Matrix<double, 19, 19>::Identity();
  Eigen::Matrix<double, 19, 19> K_1 = (H_T_H + _state.cov.inverse()).inverse();
  Eigen::Matrix<double, 19, 19> G = K_1 * H_T_H;
  Eigen::Matrix<double, 19, 1> HTz = H_pos.transpose() * R_pos_inv * pos_residual;
  
  // Zero prior: solution = K_1 * HTz (no prior correction term)
  Eigen::Matrix<double, 19, 1> solution = K_1 * HTz;
  
  V3D pos_correction = solution.block<3, 1>(3, 0);
  RCLCPP_INFO_ONCE(this->node->get_logger(),
      "[ GPS ] Correction applied: pos_delta=(%.3f, %.3f, %.3f), residual=(%.3f, %.3f, %.3f)",
      pos_correction.x(), pos_correction.y(), pos_correction.z(),
      pos_residual.x(), pos_residual.y(), pos_residual.z());
  
  _state += solution;
  _state.cov = (I_STATE - G) * _state.cov;

  // ---- Orientation observation (optional) ----
  if (gps_fuse_orientation) {
    // Use current state orientation and GPS orientation
    V3D euler_state = _state.rot_end.eulerAngles(2, 1, 0);
    V3D ori_residual = gps_orientation - euler_state;
    
    // Wrap yaw to [-pi, pi]
    ori_residual(2) = fmod(ori_residual(2) + M_PI, 2 * M_PI) - M_PI;
    
    // H_ori: only rotation columns (0-2) = identity
    Eigen::Matrix<double, 3, 19> H_ori = Eigen::Matrix<double, 3, 19>::Zero();
    H_ori.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    
    Eigen::Matrix3d R_ori = Eigen::Matrix3d::Identity() * (gps_cov * 10.0);  // orientation less trusted
    Eigen::Matrix3d R_ori_inv = R_ori.inverse();
    
    Eigen::Matrix<double, 19, 19> H_T_H_ori = H_ori.transpose() * R_ori_inv * H_ori;
    Eigen::Matrix<double, 19, 19> K_1_ori = (H_T_H_ori + _state.cov.inverse()).inverse();
    Eigen::Matrix<double, 19, 19> G_ori = K_1_ori * H_T_H_ori;
    Eigen::Matrix<double, 19, 1> HTz_ori = H_ori.transpose() * R_ori_inv * ori_residual;
    
    // Zero prior: same reason as position update
    Eigen::Matrix<double, 19, 1> solution_ori = K_1_ori * HTz_ori;
    
    _state += solution_ori;
    _state.cov = (I_STATE - G_ori) * _state.cov;
  }
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
// BOUNDARY_TIMING_BEGIN sync_packages
  fastlivo_timing::Scope timing_sync_packages(13);
// BOUNDARY_TIMING_END sync_packages
  // fprintf(stderr, "[ sync ] enter, lid=%d img=%d imu=%d mode=%d\n",
  //  (int)!lid_raw_data_buffer.empty(), (int)!img_buffer.empty(), (int)!imu_buffer.empty(), (int)slam_mode_); fflush(stderr);
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) { /* fprintf(stderr, "[ sync ] no img, return false\n"); fflush(stderr); */ return false; }
  if (imu_buffer.empty() && imu_en) { /* fprintf(stderr, "[ sync ] no imu, return false\n"); fflush(stderr); */ return false; }

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front().cloud; // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // fprintf(stderr, "[ sync ] LIVO, last_flg=%d\n", (int)last_lio_vio_flg); fflush(stderr);
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      const bool instantaneous_lidar = p_pre->lidar_type == 0;
      // Conversion through double seconds can round an exact ROS stamp by
      // two nanoseconds. This tolerance applies only to instantaneous frames.
      const double instantaneous_time_tolerance = instantaneous_lidar ? 5e-9 : 0.0;
      // fprintf(stderr, "[ sync ] WAIT/VIO, img_t_buf=%zu\n", img_time_buffer.size()); fflush(stderr);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      // fprintf(stderr, "[ sync ] img_cap_time=%.6f last_lio=%.6f\n", img_capture_time, meas.last_lio_update_time); fflush(stderr);
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0)
      {
        meas.last_lio_update_time = lid_header_time_buffer.front();
        // In simulation, camera may start before LiDAR, causing buffered image
        // timestamps to be older than the first LiDAR time. Use the earlier of
        // the two as the reference to avoid discarding valid images.
        if (!img_time_buffer.empty() && img_time_buffer.front() < meas.last_lio_update_time)
          meas.last_lio_update_time = img_time_buffer.front();
      }

      // Safety: check for empty point cloud (Gazebo may emit empty scans)
      auto last_pcl = lid_raw_data_buffer.back().cloud;
      if (!last_pcl || last_pcl->points.empty()) {
        fprintf(stderr, "[ sync ] empty point cloud, skip (ptr=%p ptsize=%zu buf=%zu)\n",
          (void*)last_pcl.get(), last_pcl ? last_pcl->points.size() : 0UL, lid_raw_data_buffer.size()); fflush(stderr);
        return false;
      }
      double lid_newest_time = lid_header_time_buffer.back() + last_pcl->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);
      // fprintf(stderr, "[ sync ] lid_newest=%.6f imu_newest=%.6f\n", lid_newest_time, imu_newest_time); fflush(stderr);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time + instantaneous_time_tolerance)
      {
        // fprintf(stderr, "[ sync ] img too new (LiDAR), return false\n"); fflush(stderr);
        return false;
      }
      // Relaxed IMU check for simulation: IMU may lag behind LiDAR/Camera
      // in Gazebo due to physics vs rendering clock differences.
      // IMU data will be accumulated as available in processImu.
      if (imu_newest_time < (require_complete_imu ? img_capture_time : meas.last_lio_update_time + 0.01))
      {
        return false;
      }

      // fprintf(stderr, "[ sync ] creating MeasureGroup...\n"); fflush(stderr);
      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      if (sensor_sync_diagnostics)
      {
        RCLCPP_INFO(node->get_logger(),
          "[DEMO_SYNC] camera=%.9f lidar_newest=%.9f imu_newest=%.9f imu_last_used=%.9f imu_count=%zu complete=%d",
          img_capture_time, lid_newest_time, imu_newest_time,
          m.imu.empty() ? -1.0 : stamp2Sec(m.imu.back()->header.stamp), m.imu.size(),
          static_cast<int>(imu_newest_time >= img_capture_time));
      }

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time + instantaneous_time_tolerance) break;
        const auto&paired=lid_raw_data_buffer.front();
        if(paired.cloud->size()>65536 || meas.pcl_proc_cur->size()+meas.pcl_proc_next->size()+paired.cloud->size()>131072)
          throw std::runtime_error("LiDAR merge capacity");
        auto pcl(paired.cloud->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          const bool current_point = instantaneous_lidar
              ? frame_header_time <= img_capture_time + instantaneous_time_tolerance
              : pcl[i].curvature < max_offs_time_ms;
          if (current_point)
          {
            const double point_frame_time = instantaneous_lidar &&
                std::abs(frame_header_time - img_capture_time) <= instantaneous_time_tolerance
                ? img_capture_time : frame_header_time;
            pt.curvature += (point_frame_time - meas.last_lio_update_time) * 1000.0f;
            if(meas.pcl_proc_cur->size()>=65536)throw std::runtime_error("LiDAR current point capacity");
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            if(meas.pcl_proc_next->size()>=65536)throw std::runtime_error("LiDAR next point capacity");
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      if (sensor_sync_diagnostics && instantaneous_lidar && !meas.pcl_proc_cur->empty())
      {
        auto offsets = std::minmax_element(meas.pcl_proc_cur->begin(), meas.pcl_proc_cur->end(),
          [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
        RCLCPP_INFO(node->get_logger(),
          "[DEMO_LIDAR_SLICE] camera=%.9f previous=%.9f points=%zu offset_min_ms=%.6f offset_max_ms=%.6f pending=%zu",
          img_capture_time, meas.last_lio_update_time, meas.pcl_proc_cur->size(),
          offsets.first->curvature, offsets.second->curvature, meas.pcl_proc_next->size());
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = stamp2Sec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = stamp2Sec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   stamp2Sec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front().cloud; // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}

void LIVMapper::publish_img_rgb(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pubImage, VIOManagerPtr vio_manager)
{
// BOUNDARY_TIMING_BEGIN pub_publish_img_rgb
  fastlivo_timing::Scope timing_pub_publish_img_rgb(7);
// BOUNDARY_TIMING_END pub_publish_img_rgb
  cv::Mat img_rgb = vio_manager->img_cp;
  if (img_rgb.empty()) {
    RCLCPP_WARN(this->node->get_logger(), "[ RGB ] img_cp is empty, skipping publish");
    return;
  }
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = this->node->get_clock()->now();
  // out_msg.header.frame_id = odom_frame;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  sensor_msgs::msg::Image ros_img;
  out_msg.toImageMsg(ros_img);
  pubImage->publish(ros_img);
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
// BOUNDARY_TIMING_BEGIN pub_publish_frame_world
  fastlivo_timing::Scope timing_pub_publish_frame_world(7);
// BOUNDARY_TIMING_END pub_publish_frame_world
  if (pcl_w_wait_pub->empty()) return;
  const double update_time =
      (LidarMeasures.lio_vio_flg == VIO)
          ? LidarMeasures.measures.back().vio_time
          : LidarMeasures.measures.back().lio_time;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pending_scan_num = 0;
  bool projection_attempted = false;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    ++pending_scan_num;
    const int scans_per_publish = std::max(1, pub_scan_num);
    const bool camera_ready =
        vio_manager->new_frame_ && !vio_manager->img_rgb.empty();
    if(pending_scan_num >= scans_per_publish && camera_ready)
    {
      pending_scan_num = 0;
      projection_attempted = true;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w));
        if (!pf.allFinite() || pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (pc.allFinite() &&
            vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3))
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          if (!pixel.allFinite()) continue;
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  bool should_publish = false;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    // /cloud_registered keeps the upstream FAST-LIVO2 contract: a coloured
    // cloud in LIVO mode. Do not publish a header-only cloud when projection
    // produced no camera-visible points.
    if (!laserCloudWorldRGB->empty())
    {
      pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
      should_publish = true;
    }
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
    should_publish = true;
  }
  if (should_publish)
  {
    laserCloudmsg.header.stamp = sec2Stamp(update_time);
    laserCloudmsg.header.frame_id = odom_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg);
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      try
      {
        if (pcl_wait_save->points.size() > 0)
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        if(pcl_wait_save_intensity->points.size() > 0)
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }
        RCLCPP_INFO(this->node->get_logger(), "Saved PCD chunk: %s", all_points_dir.c_str());
      }
      catch (const pcl::IOException &error)
      {
        RCLCPP_ERROR(
            this->node->get_logger(),
            "PCD chunk write failed for '%s': %s. Disabling PCD saving to keep SLAM alive.",
            all_points_dir.c_str(), error.what());
        pcd_save_en = false;
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  // A projection batch has been consumed even if it produced zero RGB points.
  // Keeping it for a later camera frame mixes stale scans and grows memory.
  if(projection_attempted) PointCloudXYZI().swap(*pcl_wait_pub);
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = this->node->get_clock()->now();
    laserCloudmsg.header.frame_id = odom_frame;
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
// BOUNDARY_TIMING_BEGIN pub_publish_effect_world
  fastlivo_timing::Scope timing_pub_publish_effect_world(7);
// BOUNDARY_TIMING_END pub_publish_effect_world
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->node->get_clock()->now();
  laserCloudFullRes3.header.frame_id = odom_frame;
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
// BOUNDARY_TIMING_BEGIN pub_publish_odometry
  fastlivo_timing::Scope timing_pub_publish_odometry(7);
// BOUNDARY_TIMING_END pub_publish_odometry
  odomAftMapped.header.frame_id = odom_frame;
  odomAftMapped.child_frame_id = raw_body_frame;
  odomAftMapped.header.stamp = LidarMeasures.last_lio_update_time > 0.0
                                  ? sec2Stamp(LidarMeasures.last_lio_update_time)
                                  : this->node->get_clock()->now();
  set_posestamp(odomAftMapped.pose.pose);
  const V3D body_velocity=_state.rot_end.transpose()*_state.vel_end;
  odomAftMapped.twist.twist.linear.x=body_velocity.x();
  odomAftMapped.twist.twist.linear.y=body_velocity.y();
  odomAftMapped.twist.twist.linear.z=body_velocity.z();
  V3D body_omega=V3D::Zero();
  if(!LidarMeasures.measures.empty() && !LidarMeasures.measures.back().imu.empty()){
    const auto& gyro=LidarMeasures.measures.back().imu.back()->angular_velocity;
    body_omega=V3D(gyro.x,gyro.y,gyro.z)-_state.bias_g;
  }
  odomAftMapped.twist.twist.angular.x=body_omega.x();
  odomAftMapped.twist.twist.angular.y=body_omega.y();
  odomAftMapped.twist.twist.angular.z=body_omega.z();

  // Maintain a single TransformBroadcaster instance across calls
  static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  if (!br) {
    br = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);
  }
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  // TF for backward compatibility (RViz etc.)
  br->sendTransform(geometry_msgs::msg::TransformStamped(createTransformStamped(transform, odomAftMapped.header.stamp, odom_frame, raw_body_frame)));
  // The adapter/launch layer owns the choice between SLAM and ground-truth TF.
  if (publish_base_tf)
  {
    br->sendTransform(geometry_msgs::msg::TransformStamped(
        createTransformStamped(transform, odomAftMapped.header.stamp, odom_frame, base_frame)));
  }

  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
// BOUNDARY_TIMING_BEGIN pub_publish_mavros
  fastlivo_timing::Scope timing_pub_publish_mavros(7);
// BOUNDARY_TIMING_END pub_publish_mavros
  msg_body_pose.header.stamp = this->node->get_clock()->now();
  msg_body_pose.header.frame_id = odom_frame;
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
// BOUNDARY_TIMING_BEGIN pub_publish_path
  fastlivo_timing::Scope timing_pub_publish_path(7);
// BOUNDARY_TIMING_END pub_publish_path
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = this->node->get_clock()->now();
  msg_body_pose.header.frame_id = odom_frame;
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}

namespace {
uint64_t pipeline_wall(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))throw std::runtime_error("monotonic clock failure");return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;}
size_t pipeline_add(size_t a,size_t b){if(b>std::numeric_limits<size_t>::max()-a)throw std::overflow_error("pipeline reservation addition");return a+b;}
size_t pipeline_mul(size_t a,size_t b){if(a&&b>std::numeric_limits<size_t>::max()/a)throw std::overflow_error("pipeline reservation multiplication");return a*b;}
std::string pipeline_json(const std::string&s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;else o<<c;}o<<'"';return o.str();}
}
void LIVMapper::admit_ingress(const std::shared_ptr<IngressPacket>&p){
  p->raw_enqueue_wall_ns=pipeline_wall();if(p->kind<2)p->ready_enqueue_wall_ns=p->raw_enqueue_wall_ns;p->receive_cpu_end=ingress_thread_cpu();
  const auto sequence=ingress_queue.admit(p,p->reserved_bytes,p->kind);
  if(!sequence){const auto stats=ingress_queue.stats();if(!stats.failure.empty()){rejected_ingress=p;throw std::runtime_error(stats.failure);}return;}
  if(p->kind>=2&&pipeline_mode!="staged"){
    std::shared_ptr<IngressPacket>claimed;if(!ingress_queue.take(sequence,claimed))throw std::logic_error("inline decoder claim failed");decode_ingress(claimed);
  }
  if(pipeline_mode=="serial")commit_ingress();
}
void LIVMapper::receive_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&msg){
  if(!ingress_enabled){standard_pcl_cbk(msg);return;}
  auto p=std::make_shared<IngressPacket>();p->kind=2;p->receipt=diag_receipt();p->receipt_wall_ns=uint64_t(p->receipt*1e9);p->receive_cpu_begin=ingress_thread_cpu();p->receive_tid=syscall(SYS_gettid);p->source_ns=diag_stamp(msg->header.stamp);p->cloud_message=msg;
  try{const auto count=pipeline_mul(msg->width,msg->height);
    if(count>65536 || msg->data.size()>2*1024*1024)throw std::runtime_error("LiDAR raw message capacity");p->reserved_bytes=pipeline_add(sizeof(*p)+sizeof(*msg)*2,pipeline_add(pipeline_mul(msg->data.capacity(),2),pipeline_mul(count,3*sizeof(PointType)+2*sizeof(pcl::PointXYZI)+32)));
    p->reserved_bytes=pipeline_add(p->reserved_bytes,pipeline_mul(msg->fields.capacity(),sizeof(sensor_msgs::msg::PointField)));for(const auto&f:msg->fields)p->reserved_bytes=pipeline_add(p->reserved_bytes,f.name.capacity());p->reserved_bytes=pipeline_add(p->reserved_bytes,msg->header.frame_id.capacity());
  }catch(...){p->reserved_bytes=std::numeric_limits<size_t>::max();}
  admit_ingress(p);
}
void LIVMapper::receive_image(const sensor_msgs::msg::Image::ConstSharedPtr&msg){
  if(!ingress_enabled){img_cbk(msg);return;}
  auto p=std::make_shared<IngressPacket>();p->kind=3;p->receipt=diag_receipt();p->receipt_wall_ns=uint64_t(p->receipt*1e9);p->receive_cpu_begin=ingress_thread_cpu();p->receive_tid=syscall(SYS_gettid);p->source_ns=diag_stamp(msg->header.stamp);p->image_message=msg;
  try{p->reserved_bytes=pipeline_add(sizeof(*p)+sizeof(*msg)*2,pipeline_add(pipeline_mul(msg->data.capacity(),3),pipeline_mul(pipeline_mul(msg->width,msg->height),32)));p->reserved_bytes=pipeline_add(p->reserved_bytes,pipeline_add(msg->header.frame_id.capacity(),msg->encoding.capacity()));}
  catch(...){p->reserved_bytes=std::numeric_limits<size_t>::max();}admit_ingress(p);
}
void LIVMapper::receive_imu(const sensor_msgs::msg::Imu::ConstSharedPtr&msg){
  if(!ingress_enabled){imu_cbk(msg);return;}
  auto p=std::make_shared<IngressPacket>();p->kind=1;p->receipt=diag_receipt();p->receipt_wall_ns=uint64_t(p->receipt*1e9);p->receive_cpu_begin=ingress_thread_cpu();p->receive_tid=syscall(SYS_gettid);p->source_ns=diag_stamp(msg->header.stamp);p->imu_message=msg;p->reserved_bytes=sizeof(*p)+sizeof(*msg)*2+msg->header.frame_id.capacity();p->ready_enqueue_wall_ns=p->receipt_wall_ns;admit_ingress(p);
}
void LIVMapper::receive_tick(){
  if(!ingress_enabled){imu_prop_callback();return;}
  auto p=std::make_shared<IngressPacket>();p->kind=0;p->receipt=diag_receipt();p->receipt_wall_ns=uint64_t(p->receipt*1e9);p->receive_cpu_begin=ingress_thread_cpu();p->receive_tid=syscall(SYS_gettid);p->reserved_bytes=sizeof(*p);p->ready_enqueue_wall_ns=p->receipt_wall_ns;admit_ingress(p);
}
void LIVMapper::decode_ingress(const std::shared_ptr<IngressPacket>&p){
  p->decoder_pop_wall_ns=pipeline_wall();p->decode_tid=syscall(SYS_gettid);p->decode_begin_wall_ns=pipeline_wall();p->decode_cpu_begin=ingress_thread_cpu();
  try{if(p->kind==2){p->cloud.reset(new PointCloudXYZI());p_pre->process(p->cloud_message,p->cloud);
    }else if(p->kind==3)p->image=getImageFromMsg(p->image_message);else throw std::logic_error("invalid decoder kind");}
  catch(...){p->decode_error=std::current_exception();}
  if(p->kind==2){PointCloudXYZI empty;empty.swap(p_pre->pl_surf);PointCloudXYZI empty2;empty2.swap(p_pre->pl_corn);PointCloudXYZI empty3;empty3.swap(p_pre->pl_full);}
  p->decode_cpu_end=ingress_thread_cpu();p->decode_end_wall_ns=pipeline_wall();p->decoded_at=diag_receipt();p->ready_enqueue_wall_ns=pipeline_wall();ingress_queue.ready(p->sequence);
}
void LIVMapper::lifecycle_ingress(const char*event){
  if(!ingress_started||!ingress_lifecycle.is_open())return;const auto s=ingress_queue.stats();ingress_lifecycle<<event<<','<<pipeline_wall()<<','<<ingress_thread_cpu()<<','<<syscall(SYS_gettid)<<','<<s.accepted<<','<<s.delivered<<','<<s.committed<<','<<s.pending<<','<<s.inflight<<','<<s.ready<<','<<s.bytes<<','<<int(rclcpp::ok())<<'\n';if(!ingress_lifecycle)throw std::runtime_error("pipeline lifecycle trace failure");
}
void LIVMapper::start_ingress(){
  if(!ingress_enabled)return;const bool trace=std::getenv("FASTLIVO_INGRESS_TRACE") && std::string(std::getenv("FASTLIVO_INGRESS_TRACE"))=="1";
  if(trace){ingress_trace.open((log_directory/"pipeline_v19_events.csv").string());ingress_lifecycle.open((log_directory/"pipeline_v19_lifecycle.csv").string());
  if(!ingress_trace||!ingress_lifecycle)throw std::runtime_error("cannot open V19 traces");}
  ingress_started=true;
  if(trace){
  ingress_trace<<"sequence,kind,source_ns,receipt_wall_ns,raw_enqueue_wall_ns,decoder_pop_wall_ns,decode_begin_wall_ns,decode_end_wall_ns,ready_enqueue_wall_ns,owner_pop_wall_ns,commit_end_wall_ns,receive_tid,decode_tid,owner_tid,receive_cpu_begin_ns,receive_cpu_end_ns,decode_cpu_begin_ns,decode_cpu_end_ns,owner_cpu_begin_ns,owner_cpu_end_ns,reserved_bytes,pending_after_commit,bytes_after_commit\n";
  ingress_lifecycle<<"event,wall_ns,thread_cpu_ns,tid,accepted,delivered,committed,pending,inflight,ready,bytes,context_valid\n";lifecycle_ingress("start");}
  if(pipeline_mode=="staged"){
    cloud_decode_thread=std::thread([this]{pthread_setname_np(pthread_self(),"livo_cloud_dec");try{std::shared_ptr<IngressPacket>p;while(ingress_queue.wait_take(2,p))decode_ingress(p);}catch(const std::exception&e){ingress_queue.fail(std::string("cloud decoder: ")+e.what());}catch(...){ingress_queue.fail("unknown cloud decoder exception");}});
    image_decode_thread=std::thread([this]{pthread_setname_np(pthread_self(),"livo_image_dec");try{std::shared_ptr<IngressPacket>p;while(ingress_queue.wait_take(3,p))decode_ingress(p);}catch(const std::exception&e){ingress_queue.fail(std::string("image decoder: ")+e.what());}catch(...){ingress_queue.fail("unknown image decoder exception");}});
  }
  if(pipeline_mode!="serial"){
    ingress_executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();ingress_executor->add_node(this->node);
    ingress_thread=std::thread([this]{pthread_setname_np(pthread_self(),"livo_rx");try{ingress_executor->spin();}catch(const std::exception&e){ingress_queue.fail(std::string("receiver: ")+e.what());}catch(...){ingress_queue.fail("unknown receiver exception");}});
  }
}
void LIVMapper::commit_ingress(){
  const auto initial=ingress_queue.stats();if(!initial.failure.empty())throw std::runtime_error(initial.failure);
  for(size_t i=0;i<initial.pending;++i){
    if(!rclcpp::ok()){ingress_queue.fail("context invalid before owner commit");throw std::runtime_error("context invalid before owner commit");}
    std::shared_ptr<IngressPacket>p;if(!ingress_queue.pop(p))break;if(p->sequence!=ingress_committed+1)throw std::runtime_error("pipeline sequence discontinuity");
    const auto owner_wall=pipeline_wall(),owner_cpu=ingress_thread_cpu();committing_ingress=p.get();
    try{if(p->kind==2)standard_pcl_cbk(p->cloud_message);else if(p->kind==3)img_cbk(p->image_message);else if(p->kind==1)imu_cbk(p->imu_message);else if(p->kind==0)imu_prop_callback();else throw std::runtime_error("invalid pipeline kind");}
    catch(...){
      // PRIMARY_CALLBACK_DIAGNOSTIC_BEGIN
      const auto primary_exception = std::current_exception();
      committing_ingress = nullptr;
      try { std::rethrow_exception(primary_exception); }
      catch (const std::exception& callback_error) {
        std::fprintf(stderr,
          "V19 primary callback failure: seq=%llu kind=%d source_ns=%llu cloud_points=%zu lidar_cloud_queue=%zu image_queue=%zu imu_queue=%zu what=%.512s\n",
          static_cast<unsigned long long>(p->sequence), p->kind,
          static_cast<unsigned long long>(p->source_ns), p->cloud ? p->cloud->size() : 0,
          lid_raw_data_buffer.size(), img_buffer.size(), imu_buffer.size(), callback_error.what());
      } catch (...) {
        std::fprintf(stderr,
          "V19 primary callback failure: seq=%llu kind=%d source_ns=%llu cloud_points=%zu lidar_cloud_queue=%zu image_queue=%zu imu_queue=%zu what=non-std exception\n",
          static_cast<unsigned long long>(p->sequence), p->kind,
          static_cast<unsigned long long>(p->source_ns), p->cloud ? p->cloud->size() : 0,
          lid_raw_data_buffer.size(), img_buffer.size(), imu_buffer.size());
      }
      try { ingress_queue.fail("owner callback exception"); } catch (...) {}
      std::rethrow_exception(primary_exception);
      // PRIMARY_CALLBACK_DIAGNOSTIC_END
    }committing_ingress=nullptr;
    const auto end_cpu=ingress_thread_cpu(),end_wall=pipeline_wall();++ingress_committed;ingress_queue.acknowledge(p->sequence);const auto s=ingress_queue.stats();
    if(ingress_trace.is_open()){ingress_trace<<p->sequence<<','<<p->kind<<','<<p->source_ns<<','<<p->receipt_wall_ns<<','<<p->raw_enqueue_wall_ns<<','<<p->decoder_pop_wall_ns<<','<<p->decode_begin_wall_ns<<','<<p->decode_end_wall_ns<<','<<p->ready_enqueue_wall_ns<<','<<owner_wall<<','<<end_wall<<','<<p->receive_tid<<','<<p->decode_tid<<','<<syscall(SYS_gettid)<<','<<p->receive_cpu_begin<<','<<p->receive_cpu_end<<','<<p->decode_cpu_begin<<','<<p->decode_cpu_end<<','<<owner_cpu<<','<<end_cpu<<','<<p->reserved_bytes<<','<<s.pending<<','<<s.bytes<<'\n';if(!ingress_trace)throw std::runtime_error("pipeline event trace failure");}
  }
}
void LIVMapper::stop_ingress(bool normal){
  if(!ingress_started||ingress_stopped)return;ingress_stopped=true;
  const bool context_valid=rclcpp::ok();if(!normal||!context_valid)ingress_queue.fail(context_valid?"emergency/exception stop":"ROS context invalid at stop");
  std::exception_ptr cleanup_error;
  try{lifecycle_ingress("receiver_cancel_begin");if(imu_prop_timer)imu_prop_timer->cancel();if(ingress_executor)ingress_executor->cancel();if(ingress_thread.joinable())ingress_thread.join();lifecycle_ingress("receiver_joined");ingress_queue.stop_admission();lifecycle_ingress("admission_close");
    if(cloud_decode_thread.joinable())cloud_decode_thread.join();if(image_decode_thread.joinable())image_decode_thread.join();lifecycle_ingress("decoders_joined");
    if(normal&&rclcpp::ok()&&ingress_queue.stats().failure.empty()){lifecycle_ingress("owner_drain_begin");commit_ingress();ingress_queue.close();lifecycle_ingress("owner_drain_complete");}else ingress_queue.abandon();
  }catch(...){cleanup_error=std::current_exception();ingress_queue.fail("normal drain/cleanup exception");ingress_queue.stop_admission();if(ingress_executor){try{ingress_executor->cancel();}catch(...){}}if(ingress_thread.joinable())ingress_thread.join();if(cloud_decode_thread.joinable())cloud_decode_thread.join();if(image_decode_thread.joinable())image_decode_thread.join();ingress_queue.abandon();}
  if(ingress_executor){try{ingress_executor->remove_node(this->node);}catch(...){ingress_queue.fail("executor removal failure");}ingress_executor.reset();}
  if(ingress_trace.is_open()){ingress_trace.flush();if(!ingress_trace)ingress_queue.fail("input event trace flush failed");}
  if(ingress_lifecycle.is_open()){ingress_lifecycle.flush();if(!ingress_lifecycle)ingress_queue.fail("input lifecycle trace flush failed");}
  const auto s=ingress_queue.stats();const bool completed=normal&&context_valid&&rclcpp::ok()&&s.failure.empty()&&!s.canceled&&!s.closed_rejections&&!s.pending&&s.accepted==s.delivered&&s.accepted==s.committed&&s.committed==ingress_committed;
  std::ofstream out((log_directory/"pipeline_v19_summary.json").string());out<<"{\"schema\":\"staged_input_pipeline_v19/v1\",\"mode\":"<<pipeline_json(pipeline_mode)<<",\"image_copy_opt\":"<<int(image_copy_opt)<<",\"count_limit\":512,\"byte_limit\":67108864,\"trace_schema\":\"pipeline_v19_events/v1\",\"trace_columns\":23,\"normal_completed\":"<<(completed?"true":"false")<<",\"context_valid_at_drain\":"<<(context_valid?"true":"false")<<",\"accepted\":"<<s.accepted<<",\"delivered\":"<<s.delivered<<",\"committed\":"<<s.committed<<",\"pending\":"<<s.pending<<",\"inflight\":"<<s.inflight<<",\"ready\":"<<s.ready<<",\"bytes\":"<<s.bytes<<",\"peak_pending\":"<<s.peak_pending<<",\"peak_bytes\":"<<s.peak_bytes<<",\"canceled\":"<<s.canceled<<",\"rejected_capacity\":"<<s.rejected<<",\"closed_rejections\":"<<s.closed_rejections<<",\"failure\":"<<pipeline_json(s.failure)<<",\"uncommitted_packets\":[";
  bool first=true;for(const auto&row:ingress_queue.remaining()){if(!first)out<<',';first=false;out<<"{\"seq\":"<<row.sequence<<",\"kind\":"<<row.kind<<",\"source_ns\":"<<row.packet->source_ns<<",\"stage\":"<<int(row.stage)<<",\"reason\":"<<pipeline_json(s.failure)<<'}';}out<<"],\"rejected_packet\":";if(rejected_ingress)out<<"{\"kind\":"<<rejected_ingress->kind<<",\"source_ns\":"<<rejected_ingress->source_ns<<",\"reason\":"<<pipeline_json(s.failure)<<'}';else out<<"null";out<<"}\n";out.flush();if(!out)throw std::runtime_error("V19 summary write failed");
  ingress_trace.close();ingress_lifecycle.close();
  if(cleanup_error)std::rethrow_exception(cleanup_error);if(!completed)throw std::runtime_error("V19 pipeline stop failed: "+s.failure);
}


// Optional export only: the estimator remains the sole owner of _state and maps.
void LIVMapper::initializeBackendSnapshots()
{
  node->declare_parameter<std::string>("backend.snapshot_directory", "");
  node->declare_parameter<int>("backend.queue_capacity", 4);
  node->declare_parameter<int>("backend.max_points", 4096);
  node->declare_parameter<int>("backend.max_ready_directories", 16);
  node->declare_parameter<double>("backend.min_interval_s", 1.0);
  node->declare_parameter<double>("backend.max_interval_s", 2.0);
  node->declare_parameter<double>("backend.min_translation_m", 0.8);
  node->declare_parameter<double>("backend.min_rotation_rad", 0.2);
  const auto directory=node->get_parameter("backend.snapshot_directory").as_string();
  if(directory.empty())return;  // Default disabled: no worker, cloud copy or I/O.
  try {
    fastlivo_backend::BridgeOptions options;
    options.spool_directory=directory;
    options.epoch=uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
    const auto capacity=node->get_parameter("backend.queue_capacity").as_int();
    const auto points=node->get_parameter("backend.max_points").as_int();
    const auto ready=node->get_parameter("backend.max_ready_directories").as_int();
    const double interval=node->get_parameter("backend.min_interval_s").as_double();
    if(capacity<=0 || points<=0 || ready<=0 || !std::isfinite(interval) || interval<0 || interval>3600)
      throw std::invalid_argument("invalid backend export bounds");
    options.queue_capacity=size_t(capacity);options.max_points=size_t(points);
    options.max_ready_directories=size_t(ready);
    options.min_interval_ns=uint64_t(interval*1e9);
    const double maximum=node->get_parameter("backend.max_interval_s").as_double();
    if(!std::isfinite(maximum)||maximum<interval||maximum>3600)throw std::invalid_argument("invalid backend maximum interval");
    options.max_interval_ns=uint64_t(maximum*1e9);
    options.min_translation_m=node->get_parameter("backend.min_translation_m").as_double();
    options.min_rotation_rad=node->get_parameter("backend.min_rotation_rad").as_double();
    backend_snapshots=std::make_unique<fastlivo_backend::SnapshotBridge>(options);
    RCLCPP_INFO(node->get_logger(),"Backend keyframe snapshots enabled (bounded, asynchronous I/O): %s",directory.c_str());
  } catch(const std::exception& error) {
    RCLCPP_WARN(node->get_logger(),"Backend snapshots disabled: %s",error.what());
  }
}
namespace {
fastlivo_backend::Transform backend_transform(const M3D& rotation,const V3D& translation)
{
  auto result=fastlivo_backend::identity_transform();
  for(size_t r=0;r<3;++r){for(size_t c=0;c<3;++c)result[4*r+c]=rotation(r,c);result[4*r+3]=translation(r);}
  return result;
}
}
void LIVMapper::backendLioSnapshot()
{
  if(!backend_snapshots)return;
  // Constant-size state health, never a full-cloud integrity pass.
  if(p_imu->imu_need_init || !_state.rot_end.allFinite() || !_state.pos_end.allFinite() || !_state.cov.allFinite())return;
  const auto stamp_ns=diag_stamp(sec2Stamp(LidarMeasures.last_lio_update_time));
  backend_snapshots->on_lio(stamp_ns,backend_transform(_state.rot_end,_state.pos_end),
    backend_transform(extR,extT),slam_mode_==LIVO,feats_down_body->size(),
    [this](size_t i) noexcept {
      const auto& p=feats_down_body->points[i];
      return fastlivo_backend::PointXYZ{p.x,p.y,p.z};
    });
}
void LIVMapper::backendVioSnapshot(bool updated)
{
  if(!backend_snapshots)return;
  const bool finite=_state.rot_end.allFinite() && _state.pos_end.allFinite();
  backend_snapshots->on_vio(diag_stamp(sec2Stamp(LidarMeasures.last_lio_update_time)),
    updated && finite,backend_transform(_state.rot_end,_state.pos_end));
}
