
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/r107_transport_probe.h>
#include <plan_manage/reference_progress.h>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#include <limits>
#include <unistd.h>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }

  bool valid_reference_input(const nav_msgs::msg::Path &path, double body_height,
                             const Eigen::Vector3d &odom_position,
                             const Eigen::Vector3d &used_velocity,
                             const std::string &expected_frame)
  {
    if (path.header.frame_id != expected_frame || path.poses.empty() ||
        !std::isfinite(body_height) || !odom_position.allFinite() || !used_velocity.allFinite())
      return false;
    Eigen::Vector3d previous = odom_position;
    for (const auto &pose : path.poses)
    {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != expected_frame) return false;
      const auto &p = pose.pose.position;
      const auto &q = pose.pose.orientation;
      Eigen::Vector3d body_point(p.x, p.y, p.z + body_height);
      if (!body_point.allFinite() || !std::isfinite(p.z) || !std::isfinite(q.x) ||
          !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
          (body_point - previous).norm() <= 1e-6)
        return false; // A zero-duration segment makes the polynomial fit singular.
      previous = body_point;
    }
    return true;
  }

  bool valid_reference_curve(PolynomialTraj &curve, double duration)
  {
    const auto times = curve.getTimes();
    if (times.empty() || !std::isfinite(duration) || duration < 1e-3) return false;
    double sum = 0.0;
    for (const double t : times)
    {
      if (!std::isfinite(t) || t <= 0.0) return false;
      sum += t;
    }
    if (!std::isfinite(sum) || std::abs(sum - duration) > 1e-6 * std::max(1.0, sum)) return false;
    for (int axis = 0; axis < 3; ++axis)
    {
      const auto coefficients = curve.getCoef(axis);
      if (coefficients.size() != times.size()) return false;
      for (size_t segment = 0; segment < times.size(); ++segment)
      {
        if (coefficients[segment].size() < 3) return false;
        for (double c : coefficients[segment]) if (!std::isfinite(c)) return false;
        if (axis && coefficients[segment].size() != curve.getCoef(0)[segment].size()) return false;
      }
    }
    return true;
  }

  std::string reference_curve_json(PolynomialTraj &curve)
  {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"coefficient_order\":\"descending_power\",\"segment_times\":[";
    const auto times = curve.getTimes();
    for (size_t i = 0; i < times.size(); ++i) { if (i) out << ','; out << times[i]; }
    out << "],\"coefficients_xyz\":[";
    for (int axis = 0; axis < 3; ++axis)
    {
      if (axis) out << ',';
      out << '[';
      const auto coefficients = curve.getCoef(axis);
      for (size_t segment = 0; segment < coefficients.size(); ++segment)
      {
        if (segment) out << ',';
        out << '[';
        for (size_t i = 0; i < coefficients[segment].size(); ++i)
        { if (i) out << ','; out << coefficients[segment][i]; }
        out << ']';
      }
      out << ']';
    }
    out << "]}";
    return out.str();
  }

  std::string reference_content_json(const nav_msgs::msg::Path &path, double body_height,
                                     PolynomialTraj &curve)
  {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"frame_id\":\"" << path.header.frame_id << "\",\"stamp\":["
        << path.header.stamp.sec << ',' << path.header.stamp.nanosec
        << "],\"body_height_offset\":" << body_height << ",\"input_poses\":[";
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      if (i) out << ',';
      const auto &pose = path.poses[i];
      const auto &p = pose.pose.position; const auto &q = pose.pose.orientation;
      out << "{\"frame_id\":\"" << pose.header.frame_id << "\",\"stamp\":["
          << pose.header.stamp.sec << ',' << pose.header.stamp.nanosec << "],\"position\":["
          << p.x << ',' << p.y << ',' << p.z << "],\"orientation_xyzw\":["
          << q.x << ',' << q.y << ',' << q.z << ',' << q.w << "]}";
    }
    out << "],\"global_curve\":" << reference_curve_json(curve) << '}';
    return out.str();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    external_reference_protocol_ = load_parameter<bool>(node_, "fsm.external_reference_protocol", false);
    reference_run_id_ = load_parameter<std::string>(node_, "fsm.reference_run_id", "");
    const int64_t reference_epoch = load_parameter<int64_t>(node_, "fsm.reference_epoch", 0);
    const int64_t localization_generation = load_parameter<int64_t>(node_, "fsm.localization_generation", 0);
    const bool run_id_safe = !reference_run_id_.empty() && reference_run_id_.size() <= 256 &&
        std::all_of(reference_run_id_.begin(), reference_run_id_.end(), [](char c) {
          return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                 c == '_' || c == '-' || c == '.' || c == ':';
        });
    if (external_reference_protocol_ &&
        (navi_mode_ != NAVI_MODE::REFERENCE_PATH || !run_id_safe || reference_epoch < 0 || localization_generation < 0))
      throw std::runtime_error("External reference protocol requires mode3 and explicit run/epoch/generation");
    reference_epoch_ = static_cast<uint64_t>(reference_epoch);
    localization_generation_ = static_cast<uint64_t>(localization_generation);
    child_odometry_ack_enabled_ = load_parameter<bool>(node_, "fsm.child_odometry_consumption_ack", false);
    reference_protocol_ack_enabled_ = load_parameter<bool>(node_, "fsm.reference_protocol_ack", false);
    r74_protocol_observation_enabled_ = load_parameter<bool>(node_, "fsm.r74_protocol_observation", false);
    if (reference_protocol_ack_enabled_ && !child_odometry_ack_enabled_)
      throw std::runtime_error("Reference commit ACK requires child odometry consumption ACK");
    if (child_odometry_ack_enabled_)
    {
      child_odometry_ack_nonce_ = load_parameter<std::string>(node_, "fsm.child_odometry_ack_nonce", "");
      if (!external_reference_protocol_ || navi_mode_ != NAVI_MODE::REFERENCE_PATH ||
          !valid_child_ack_nonce(child_odometry_ack_nonce_))
        throw std::runtime_error("Child odometry ACK requires a fresh scoped external-reference nonce");
      child_odometry_ack_identity_ = FreshChildOdometryIdentity::capture();
      child_odometry_ack_pub_ = node_->create_publisher<std_msgs::msg::String>(
          "planning/odometry_consumption_ack", rclcpp::QoS(10).reliable());
      child_odometry_ack_gid_ = fresh_child_gid_hex(child_odometry_ack_pub_->get_gid());
    }
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    measured_start_.max_speed = load_parameter<double>(node_, "fsm.measured_max_speed", 0.12);
    measured_start_.time_constant = load_parameter<double>(node_, "fsm.measured_velocity_filter_tau", 0.4);
    if (!std::isfinite(measured_start_.max_speed) || !std::isfinite(measured_start_.time_constant) ||
        measured_start_.max_speed <= 0.0 || measured_start_.time_constant <= 0.0)
      throw std::runtime_error("measured velocity policy must have positive speed/tau");
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && !std::isfinite(body_height_))
      throw std::runtime_error("mode3 body height offset must be finite");
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    trajectory_metadata_pub_ = node_->create_publisher<std_msgs::msg::String>("planning/trajectory_metadata", 10);
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      reference_attempt_pub_ = node_->create_publisher<std_msgs::msg::String>("planning/reference_attempt", 10);
      route_definition_pub_ = node_->create_publisher<std_msgs::msg::String>(
          "planning/route_definition", rclcpp::QoS(1).reliable().transient_local());
    }
    if (r74_protocol_observation_enabled_) r74InitializeObservation();
    r107_transport::probe().configure(reference_run_id_, reference_epoch_,
        child_odometry_ack_identity_.start_ticks);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
      if (external_reference_protocol_)
        local_replan_sub_ = node_->create_subscription<std_msgs::msg::UInt64MultiArray>(
            "planning/local_replan_request", 10,
            std::bind(&SCANReplanFSM::localReplanCallback, this, std::placeholders::_1));
    }
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        measured_start_.velocity(go2_execution_frozen_),
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        !valid_reference_curve(planner_manager_->global_data_.global_traj_,
                               planner_manager_->global_data_.global_duration_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference fit contains invalid duration or coefficients");
      return false;
    }
    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      global_endpoint_local_occupancy_ = final_occ;
      // A supplied PCT global reference is not limited by visibility in the sliding LOCAL map.
      // Occupancy here remains diagnostic; every actual local target/curve is still checked.
      // This matches the existing external-reference protocol semantics in all mode3 clients.
      return true;
    }
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  bool SCANReplanFSM::mode3ReferenceUsable() const
  {
    return navi_mode_ != NAVI_MODE::REFERENCE_PATH ||
           (reference_ready_ && reference_revision_ > 0 && !reference_content_json_.empty());
  }

  void SCANReplanFSM::invalidateReference(const char *reason)
  {
    reference_ready_ = false;
    if (reference_protocol_ack_enabled_) reference_protocol_publication_.reset_reference();
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    // Keep emergency-stop ownership; old local_data is only a nominal candidate,
    // never evidence that an outer executor has accepted or stopped a trajectory.
    if (exec_state_ != EMERGENCY_STOP) changeFSMExecState(WAIT_TARGET, "REFERENCE_INVALID");
    RCLCPP_WARN(node_->get_logger(), "Reference disabled: %s; outer hold remains authoritative", reason);
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH) return;
    invalidateReference("replacement received");
    if (!msg || !have_odom_ ||
        (external_reference_protocol_ && msg->poses.size() > 4096) || // Explicit R85 input budget; no simplification.
        !valid_reference_input(*msg, body_height_, odom_pos_,
                               measured_start_.velocity(go2_execution_frozen_), self_inflation_frame_id_))
    {
      RCLCPP_WARN(node_->get_logger(), "Rejected reference: frame/finite/odometry/nonzero-segment validation failed");
      return;
    }
    if (reference_revision_ == std::numeric_limits<uint64_t>::max())
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference revision exhausted; restart with a fresh outer epoch");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());
    for (const auto &pose : msg->poses)
      waypoints.emplace_back(pose.pose.position.x, pose.pose.position.y,
                             pose.pose.position.z + body_height_);

    // The manager writes global_data while fitting. Roll back geometry on failure,
    // but leave readiness false: a rejected replacement cannot revive an old route.
    std::unique_ptr<GlobalTrajData> previous_global_data;
    if (reference_revision_ > 0)
      previous_global_data = std::make_unique<GlobalTrajData>(planner_manager_->global_data_);
    const Eigen::Vector3d previous_end = reference_revision_ ? end_pt_ : Eigen::Vector3d::Zero();
    const Eigen::Vector3d previous_end_velocity = reference_revision_ ? end_vel_ : Eigen::Vector3d::Zero();
    if (!planGlobalTrajByWaypoints(waypoints))
    {
      if (previous_global_data) planner_manager_->global_data_ = *previous_global_data;
      end_pt_ = previous_end;
      end_vel_ = previous_end_velocity;
      invalidateReference("replacement fit/endpoint rejected");
      return;
    }

    pending_external_replan_sequence_ = current_attempt_replan_sequence_ = 0;
    reference_stamp_ = msg->header.stamp;
    reference_body_goal_ = waypoints.back();
    reference_original_duration_ = planner_manager_->global_data_.global_duration_;
    reference_content_json_ = reference_content_json(*msg, body_height_, planner_manager_->global_data_.global_traj_);
    ++reference_revision_;
    publishRouteDefinition();
    reference_ready_ = true;
    if (reference_protocol_ack_enabled_)
      reference_protocol_publication_.commit(reference_run_id_, reference_epoch_, localization_generation_,
          static_cast<uint64_t>(getpid()), reference_revision_, reference_stamp_.sec, reference_stamp_.nanosec);
    have_target_ = true;
    have_new_target_ = true;
    trigger_ = true;
    if (exec_state_ != EMERGENCY_STOP) changeFSMExecState(GEN_NEW_TRAJ, "REFERENCE_COMMIT");
    RCLCPP_INFO(node_->get_logger(), "Committed reference revision %llu; local candidates still require outer admission",
                static_cast<unsigned long long>(reference_revision_));
  }

  void SCANReplanFSM::localReplanCallback(const std_msgs::msg::UInt64MultiArray::ConstSharedPtr &msg)
  {
    if (reference_protocol_ack_enabled_)
      reference_protocol_publication_.request_received(msg && msg->data.size()>2 ? msg->data[2] : 0,
          node_->now().nanoseconds(), last_odom_stamp_ns_,
          msg && msg->layout.dim.size()==1 ? msg->layout.dim[0].label : "",
          msg && msg->data.size()>0 ? msg->data[0] : 0, msg && msg->data.size()>1 ? msg->data[1] : 0,
          msg && msg->data.size()>3 ? msg->data[3] : 0, msg && msg->data.size()>4 ? msg->data[4] : 0,
          msg && msg->layout.dim.size()==1 && msg->layout.data_offset==0 &&
          msg->layout.dim[0].size==5 && msg->layout.dim[0].stride==5 && msg->data.size()==5);
    if (!external_reference_protocol_ || !msg || !mode3ReferenceUsable() || !have_odom_ ||
        exec_state_ == EMERGENCY_STOP || msg->layout.dim.size() != 1 || msg->layout.data_offset != 0 ||
        msg->layout.dim[0].label != reference_run_id_ || msg->layout.dim[0].size != 5 ||
        msg->layout.dim[0].stride != 5 || msg->data.size() != 5 ||
        msg->data[0] != reference_epoch_ || msg->data[1] != reference_revision_ ||
        msg->data[2] <= last_external_replan_sequence_ || reference_stamp_.sec < 0 ||
        msg->data[3] != static_cast<uint64_t>(reference_stamp_.sec) ||
        msg->data[4] != reference_stamp_.nanosec || msg->data[4] >= 1000000000ULL)
    {
      if (reference_protocol_ack_enabled_)
        reference_protocol_publication_.request_result(false, "scope_reference_sequence_or_state");
      RCLCPP_WARN(node_->get_logger(), "Local replan request rejected: scope/reference/sequence/state");
      return;
    }
    const int64_t age_ns = node_->now().nanoseconds() - last_odom_stamp_ns_;
    const double age_wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_odom_receive_time_).count();
    if (age_ns < 0 || age_ns >= 300000000LL || age_wall < 0.0 || age_wall >= .3 ||
        !odom_pos_.allFinite() || !measured_start_.velocity(go2_execution_frozen_).allFinite())
    {
      if (reference_protocol_ack_enabled_)
        reference_protocol_publication_.request_result(false, "measured_source_not_fresh_or_finite");
      RCLCPP_WARN(node_->get_logger(), "Local replan request rejected: measured source is not fresh/finite");
      return;
    }
    if (reference_protocol_ack_enabled_) reference_protocol_publication_.request_result(true, "accepted");
    last_external_replan_sequence_ = pending_external_replan_sequence_ = msg->data[2];
    have_target_ = true;
    trigger_ = true;
    changeFSMExecState(REPLAN_TRAJ, "EXTERNAL_LOCAL_REPLAN");
    RCLCPP_INFO(node_->get_logger(), "Local replan %llu accepted for unchanged reference revision %llu",
                static_cast<unsigned long long>(last_external_replan_sequence_),
                static_cast<unsigned long long>(reference_revision_));
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    last_odom_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
    last_odom_receive_time_ = std::chrono::steady_clock::now();
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
    measured_start_.update(odom_vel_, rclcpp::Time(msg->header.stamp).nanoseconds(), go2_execution_frozen_);

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
    publishChildOdometryAck(*msg);
  }

  void SCANReplanFSM::publishChildOdometryAck(const nav_msgs::msg::Odometry &message)
  {
    if (!child_odometry_ack_enabled_ || !have_odom_ || last_odom_stamp_ns_ <= 0 ||
        message.header.frame_id != self_inflation_frame_id_) return;
    // Source header is the odometry actually consumed above, never node.now().
    std::ostringstream out;
    out << "{\"schema\":\"r85_scan_child_odometry_consumed/v1\",\"run_id\":\"" << reference_run_id_
        << "\",\"planner_epoch\":" << reference_epoch_
        << ",\"localization_generation\":" << localization_generation_
        << ",\"planner_pid\":" << child_odometry_ack_identity_.pid
        << ",\"planner_ppid\":" << child_odometry_ack_identity_.ppid
        << ",\"planner_start_ticks\":" << child_odometry_ack_identity_.start_ticks
        << ",\"startup_nonce\":\"" << child_odometry_ack_nonce_
        << "\",\"source_stamp_ns\":" << last_odom_stamp_ns_
        << ",\"source_frame_id\":\"camera_init\",\"have_odom\":true,\"ack_sequence\":"
        << ++child_odometry_ack_sequence_ << ",\"publisher_gid\":\"" << child_odometry_ack_gid_ << "\"}";
    std_msgs::msg::String ack; ack.data = out.str();
    if (reference_protocol_ack_enabled_)
      ack.data.insert(ack.data.size() - 1, fresh_reference_commit_ack_fields(
          reference_ready_, reference_revision_, reference_stamp_.sec, reference_stamp_.nanosec));
    if (reference_protocol_ack_enabled_)
    {
      static const char *state_names[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
      const int state = static_cast<int>(exec_state_);
      ack.data.insert(ack.data.size() - 1, reference_protocol_publication_.json(
          state >= 0 && state < 6 ? state_names[state] : "UNKNOWN",
          reference_attempt_pub_->get_subscription_count(), bspline_pub_->get_subscription_count(),
          trajectory_metadata_pub_->get_subscription_count()));
    }
    child_odometry_ack_pub_->publish(ack);
    if (r74_protocol_observation_enabled_) r74MaybePublishObservation();
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
    if (go2_execution_frozen_) measured_start_.stop();
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();
    if (!mode3ReferenceUsable() && exec_state_ != EMERGENCY_STOP && exec_state_ != INIT)
    {
      if (exec_state_ != WAIT_TARGET) changeFSMExecState(WAIT_TARGET, "REFERENCE_NOT_READY");
      return;
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (reference_protocol_ack_enabled_)
        reference_protocol_publication_.gen_entered(node_->now().nanoseconds(), last_odom_stamp_ns_);
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      current_attempt_replan_sequence_ = 0;
      bool success = callReboundReplan(true, flag_random_poly_init, "generate_initial");
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        // A completed LOCAL segment is not completion of the full supplied PCT route.
        // Keep its endpoint and replan from measured pose until near the global endpoint.
        if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && reference_ready_ &&
            (end_pt_ - odom_pos_).norm() > 0.25)
        {
          changeFSMExecState(REPLAN_TRAJ, "PCT_LOCAL_SEGMENT_COMPLETE");
          return;
        }
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    setStartStateFromOdomOrCurrentTraj();

    // R84: REFERENCE_PATH owns the waypoint-defined global route.
    // Keep measured-start p/v/a policy; do not inherit unexecuted nominal derivatives.
    if (navi_mode_ == REFERENCE_PATH)
    {
      if (!reference_ready_)
      {
        RCLCPP_WARN(node_->get_logger(), "R84 reference route is not ready; replan denied");
        return false;
      }
    }
    else
    {
      if (!planner_manager_->planGlobalTraj(
              start_pt_,
              start_vel_,
              start_acc_,
              end_pt_,
              Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero()))
      {
        RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
        return false;
      }
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    current_attempt_replan_sequence_ = pending_external_replan_sequence_;
    pending_external_replan_sequence_ = 0;
    const bool external_attempt = current_attempt_replan_sequence_ != 0;
    bool success = callReboundReplan(true, false, external_attempt ? "external_local_replan_poly" : "replan_poly");
    if (!success)
    {
      success = callReboundReplan(true, true, external_attempt ? "external_local_replan_random_poly" : "replan_random_poly");
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = measured_start_.velocity(go2_execution_frozen_);
    // No measured acceleration is available in the Odometry interface. An old
    // nominal spline derivative is not actual acceleration after Go2 braking.
    start_acc_.setZero();
    RCLCPP_INFO(node_->get_logger(),
                "Measured replan state: frozen=%d pos=[%.6f,%.6f,%.6f] measured_vel=[%.6f,%.6f,%.6f] used_vel=[%.6f,%.6f,%.6f] acc=[0,0,0] body_goal=[%.6f,%.6f,%.6f]",
                go2_execution_frozen_, start_pt_(0), start_pt_(1), start_pt_(2),
                odom_vel_(0), odom_vel_(1), odom_vel_(2), start_vel_(0), start_vel_(1), start_vel_(2),
                end_pt_(0), end_pt_(1), end_pt_(2));
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, const std::string &origin)
  {
    if (!mode3ReferenceUsable()) return false;
    // All moving-candidate entry points, including GEN and safety replans, pass here.
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && !adjustGlobalTargetIfOccupied()) return false;
    ++candidate_attempt_id_;
    r107_transport::probe().attempt(candidate_attempt_id_, last_odom_stamp_ns_);
    attempt_origin_ = origin;
    if (reference_protocol_ack_enabled_)
      reference_protocol_publication_.begin_attempt(candidate_attempt_id_, origin, current_attempt_replan_sequence_);
    attempt_use_poly_init_ = flag_use_poly_init;
    attempt_random_poly_ = flag_randomPolyTraj;
    attempt_progress_before_ = planner_manager_->global_data_.last_progress_time_;
    if(!getLocalTarget()) {
      attempt_progress_after_=planner_manager_->global_data_.last_progress_time_;
      local_target_occupancy_=-1;
      publishReferenceAttempt(false,"connected_reference_recovery_unavailable");
      if(reference_protocol_ack_enabled_)reference_protocol_publication_.complete_attempt(false,"connected_reference_recovery_unavailable");
      return false;
    }
    attempt_progress_after_ = planner_manager_->global_data_.last_progress_time_;
    local_target_occupancy_ = planner_manager_->grid_map_->getInflateOccupancy(
        local_target_pt_, estimateYawFromSegment(odom_pos_, local_target_pt_));
    if (external_reference_protocol_ &&
        (local_target_occupancy_ != 0 || !local_target_pt_.allFinite() || !local_target_vel_.allFinite()))
    {
      if (r74_protocol_observation_enabled_) r74ObserveTargetFailure();
      publishReferenceAttempt(false, "local_target_not_free_or_nonfinite");
      if (reference_protocol_ack_enabled_)
        reference_protocol_publication_.complete_attempt(false, "local_target_not_free_or_nonfinite");
      return false;
    }

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;
    publishReferenceAttempt(plan_success);

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      {
        r107_transport::Publication observation(1, 2, reference_epoch_, localization_generation_,
            reference_revision_, candidate_attempt_id_, bspline.traj_id,
            bspline.start_time.sec, bspline.start_time.nanosec,
            64 + bspline.pos_pts.size()*24 + (bspline.knots.size()+bspline.yaw_pts.size())*8, 1);
        if (r107_transport::probe().enabled())
          try { observation.cached_source(last_odom_stamp_ns_, node_->now().nanoseconds()); }
          catch (...) { r107_transport::probe().observation_error(); }
        bspline_pub_->publish(bspline);
        observation.returned();
      }
      if (reference_protocol_ack_enabled_)
        reference_protocol_publication_.bspline_published(bspline.traj_id, bspline.start_time.sec, bspline.start_time.nanosec, true);
      publishTrajectoryMetadata(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    if (reference_protocol_ack_enabled_)
      reference_protocol_publication_.complete_attempt(plan_success, "optimizer_return");
    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    ++candidate_attempt_id_;
    r107_transport::probe().attempt(candidate_attempt_id_, last_odom_stamp_ns_);
    current_attempt_replan_sequence_ = 0;
    attempt_origin_ = "emergency_stop";
    attempt_use_poly_init_ = false;
    attempt_random_poly_ = false;
    attempt_progress_before_ = attempt_progress_after_ =
        reference_ready_ ? planner_manager_->global_data_.last_progress_time_ : 0.0;
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    {
      r107_transport::Publication observation(2, 2, reference_epoch_, localization_generation_,
          reference_revision_, candidate_attempt_id_, bspline.traj_id,
          bspline.start_time.sec, bspline.start_time.nanosec,
          64 + bspline.pos_pts.size()*24 + (bspline.knots.size()+bspline.yaw_pts.size())*8, -1);
      if (r107_transport::probe().enabled())
        try { observation.cached_source(last_odom_stamp_ns_, node_->now().nanoseconds()); }
        catch (...) { r107_transport::probe().observation_error(); }
      bspline_pub_->publish(bspline);
      observation.returned();
    }
    if (reference_protocol_ack_enabled_)
      reference_protocol_publication_.bspline_published(bspline.traj_id, bspline.start_time.sec, bspline.start_time.nanosec, false);
    publishTrajectoryMetadata(bspline, true);

    return true;
  }


  static bool r74_bounded_cstr(const char *value, std::size_t limit) noexcept
  {
    if (!value) return false;
    for (std::size_t i=0; i<=limit; ++i) if (value[i]=='\0') return true;
    return false;
  }

  void SCANReplanFSM::r74DisableObservation(const char *reason) noexcept
  {
    r74_protocol_observation_enabled_=false;
    if (r74_observation_warning_emitted_) return;
    r74_observation_warning_emitted_=true;
    // Exactly one fixed, short diagnostic warning; no raw input is interpolated.
    try { RCLCPP_WARN(node_->get_logger(), "R74 observation disabled: %s", reason); }
    catch (...) { /* A logger exception must not enter the business path. */ }
  }

  void SCANReplanFSM::r74InitializeObservation()
  {
    try {
      if (reference_run_id_.size()>256) throw std::length_error("r74_scope_limit");
      r74_observation_.reset(new r74_observation::Bounded());
      r74_observation_pub_ = node_->create_publisher<std_msgs::msg::String>(
          "/demo/navigation/r74_protocol_observation_cpp", rclcpp::QoS(1).best_effort());
      std::ostringstream endpoints; endpoints << '['; bool comma=false;
      const auto add = [&](const auto &pub) {
        if (!pub) return;
        const char *topic=pub->get_topic_name();
        if (!r74_bounded_cstr(topic,256)) throw std::length_error("r74_topic_limit");
        if (comma) endpoints << ',';
        comma=true;
        const auto qos=pub->get_actual_qos().get_rmw_qos_profile();
        endpoints << "{\"resolved_topic\":" << r74_observation::quoted(std::string(topic))
            << ",\"publisher_gid\":" << r74_observation::quoted(fresh_child_gid_hex(pub->get_gid()))
            << ",\"qos_reliability\":" << int(qos.reliability)
            << ",\"qos_durability\":" << int(qos.durability) << ",\"qos_depth\":" << qos.depth << '}';
      };
      add(reference_attempt_pub_); add(bspline_pub_); add(trajectory_metadata_pub_); add(route_definition_pub_);
      endpoints << ']'; r74_observation_->endpoints=endpoints.str();
    } catch (...) {
      // Diagnostic construction is never a planner/control failure.
      r74DisableObservation("initialization_or_scope_limit");
      r74_observation_pub_.reset(); r74_observation_.reset();
    }
  }

  std::string SCANReplanFSM::r74ObservationScope()
  {
    if (reference_run_id_.size()>256 || attempt_origin_.size()>96)
      throw std::length_error("r74_scope_limit");
    const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::ostringstream s;
    s << "{\"run_id\":" << r74_observation::quoted(reference_run_id_)
      << ",\"reference_epoch\":" << reference_epoch_
      << ",\"localization_generation\":" << localization_generation_
      << ",\"planner_process_id\":" << getpid()
      << ",\"planner_start_ticks\":" << child_odometry_ack_identity_.start_ticks
      << ",\"committed_reference_revision\":" << reference_revision_
      << ",\"reference_stamp\":[" << reference_stamp_.sec << ',' << reference_stamp_.nanosec << ']'
      << ",\"attempt_id\":" << candidate_attempt_id_
      << ",\"origin\":" << r74_observation::quoted(attempt_origin_)
      << ",\"local_replan_request_sequence\":" << current_attempt_replan_sequence_
      << ",\"source_stamp_ns\":" << last_odom_stamp_ns_
      << ",\"observed_ros_clock_ns\":" << node_->now().nanoseconds()
      << ",\"observed_monotonic_wall_ns\":" << wall << '}';
    return s.str();
  }

  void SCANReplanFSM::r74ObserveTargetFailure()
  {
    if (!r74_observation_) return;
    try {
      if (++r74_observation_->failed_targets>r74_observation::Bounded::TARGET_CAPTURE_CAP) {
        ++r74_observation_->target_omitted; return;
      }
      // Reuse the ONE existing query at callReboundReplan; no map access here.
      std::ostringstream s;
      s << "{\"identity\":" << r74ObservationScope()
        << ",\"existing_local_target_occupancy\":" << local_target_occupancy_
        << ",\"point\":[" << r74_observation::number(local_target_pt_.x()) << ','
        << r74_observation::number(local_target_pt_.y()) << ',' << r74_observation::number(local_target_pt_.z())
        << "],\"velocity\":[" << r74_observation::number(local_target_vel_.x()) << ','
        << r74_observation::number(local_target_vel_.y()) << ',' << r74_observation::number(local_target_vel_.z())
        << "],\"point_all_finite\":" << (local_target_pt_.allFinite() ? "true" : "false")
        << ",\"velocity_all_finite\":" << (local_target_vel_.allFinite() ? "true" : "false")
        << ",\"target_time\":" << r74_observation::number(local_target_time_)
        << ",\"duration\":" << r74_observation::number(planner_manager_->global_data_.global_duration_)
        << ",\"progress_before\":" << r74_observation::number(attempt_progress_before_)
        << ",\"progress_after\":" << r74_observation::number(attempt_progress_after_)
        << ",\"execution_frozen\":" << (go2_execution_frozen_ ? "true" : "false")
        << ",\"fsm_state\":" << int(exec_state_)
        << ",\"finite_flags_are_diagnostic_not_original_short_circuit_evaluations\":true}";
      r74_observation_->failure(s.str());
    } catch (...) { ++r74_observation_->errors; r74DisableObservation("target_capture_or_scope_limit"); }
  }

  void SCANReplanFSM::r74ObserveString(const char *topic, const std::string &data, bool priority)
  {
    if (!r74_observation_) return;
    try {
      if (!r74_bounded_cstr(topic,256)) throw std::length_error("r74_topic_limit");
      r74_observation_->latest_fingerprint.clear();
      const bool first=priority && !r74_observation_->first_failure_checksum_attempted;
      if (!r74_observation_->checksum_allowed(data.size(), priority)) return;
      std::ostringstream s;
      s << "{\"identity\":" << r74ObservationScope()
        << ",\"topic\":" << r74_observation::quoted(std::string(topic))
        << ",\"capture_phase\":\"after_original_publish_and_counter_if_enabled\",\"data_utf8_bytes\":" << data.size()
        << ",\"crc32_ieee_hex\":" << r74_observation::quoted(r74_observation::crc_hex(r74_observation::crc32_ieee(data)))
        << ",\"hash_is_cdr\":false,\"hash_is_authentication\":false}";
      r74_observation_->store_fingerprint(s.str(),first);
    } catch (...) { ++r74_observation_->errors; r74DisableObservation("checksum_capture_or_scope_limit"); }
  }

  void SCANReplanFSM::r74MaybePublishObservation(bool immediate)
  {
    if (!r74_observation_ || !r74_observation_pub_) return;
    try {
      auto &o=*r74_observation_;
      (void)immediate; // first failure is pending until an existing checkpoint can emit it.
      if (o.emitted>=r74_observation::Bounded::EMIT_CAP) return;
      const bool first=o.first_failure_pending;
      if (!first && o.emitted>=r74_observation::Bounded::EMIT_CAP-1) return;
      const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      if (!o.due(wall, first)) return;
      std::ostringstream s;
      s << "{\"schema\":\"r74_protocol_observation_cpp/v1\",\"event\":"
        << r74_observation::quoted(first ? "first_failed_target" : "bounded_summary")
        << ",\"identity\":" << r74ObservationScope() << ",\"emit_index\":" << o.emitted+1
        << ",\"endpoints\":" << o.endpoints
        << ",\"string_receipt\":" << o.next_fingerprint(first)
        << ",\"failed_target\":" << ((first ? o.first_failure : o.last_failure).empty() ? "null" : (first ? o.first_failure : o.last_failure))
        << ",\"counts\":{\"strings_seen\":" << o.strings_seen << ",\"checksums\":" << o.checksums
        << ",\"route_definitions_published\":" << route_definitions_published_
        << ",\"route_definition_last_bytes\":" << route_definition_last_bytes_
        << ",\"checksum_bytes\":" << o.checksum_bytes << ",\"checksum_omitted\":" << o.checksum_omitted
        << ",\"failed_targets\":" << o.failed_targets << ",\"target_omitted\":" << o.target_omitted
        << ",\"observation_errors\":" << o.errors
        << ",\"oversize_records\":" << o.oversize_records << "},\"limits\":{\"prefix\":8,\"tail\":8,\"period_wall_ns\":5000000000,\"min_emit_spacing_wall_ns\":2000000000,\"process_emit_cap\":128,\"record_bytes\":4096}"
        << ",\"publication_is_delivery_or_admission\":false}";
      std_msgs::msg::String receipt; receipt.data=s.str();
      if (receipt.data.size()>r74_observation::Bounded::RECORD_BYTES) {
        ++o.oversize_records; o.emitted_at(wall,first); return;
      }
      r74_observation_pub_->publish(receipt); o.emitted_at(wall,first);
    } catch (...) {
      // Never let an observer exception enter the original control/ACK path.
      if (r74_observation_) ++r74_observation_->errors;
      r74DisableObservation("diagnostic_publish_error");
    }
  }

  void SCANReplanFSM::publishRouteDefinition()
  {
    // One publication per real committed revision. Transient-local QoS supplies
    // the current definition to late subscribers; candidates carry only IDs.
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"schema\":\"scan_route_definition/v1\",\"run_id\":"
        << r74_observation::quoted(reference_run_id_)
        << ",\"reference_epoch\":" << reference_epoch_
        << ",\"localization_generation\":" << localization_generation_
        << ",\"planner_process_id\":" << getpid()
        << ",\"reference_revision_scope\":\"planner_process_lifetime_requires_outer_epoch\","
        << "\"committed_reference_revision\":" << reference_revision_
        << ",\"reference_stamp\":[" << reference_stamp_.sec << ',' << reference_stamp_.nanosec
        << "],\"route_id\":" << (int64_t(reference_stamp_.sec) * 1000000000LL + reference_stamp_.nanosec)
        << ",\"route_version\":" << reference_revision_
        << ",\"reference_content\":" << reference_content_json_
        << ",\"reference_original_duration\":" << reference_original_duration_
        << ",\"body_goal\":[" << reference_body_goal_.x() << ',' << reference_body_goal_.y()
        << ',' << reference_body_goal_.z() << "]}";
    std_msgs::msg::String definition; definition.data = out.str();
    route_definition_pub_->publish(definition);
    ++route_definitions_published_;
    route_definition_last_bytes_ = definition.data.size();
  }

  void SCANReplanFSM::publishReferenceAttempt(bool plan_success, const std::string &reason)
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || !reference_ready_) return;
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"schema\":3,\"event\":\"attempt_result\",\"publication_role\":\"candidate_only\","
        << "\"actual_active\":false,\"outer_admission_ack_received\":false,"
        << "\"run_id\":\"" << reference_run_id_ << "\",\"reference_epoch\":" << reference_epoch_
        << ",\"localization_generation\":" << localization_generation_ << ",\"planner_process_id\":" << getpid() << ','
        << "\"reference_revision_scope\":\"planner_process_lifetime_requires_outer_epoch\","
        << "\"committed_reference_revision\":" << reference_revision_
        << ",\"reference_stamp\":[" << reference_stamp_.sec << ',' << reference_stamp_.nanosec
        << "],\"route_id\":" << (int64_t(reference_stamp_.sec) * 1000000000LL + reference_stamp_.nanosec)
        << ",\"route_version\":" << reference_revision_
        << ",\"attempt\":{\"id\":" << candidate_attempt_id_ << ",\"origin\":\"" << attempt_origin_
        << "\",\"use_poly_init\":" << (attempt_use_poly_init_ ? "true" : "false")
        << ",\"random_poly\":" << (attempt_random_poly_ ? "true" : "false")
        << ",\"plan_success\":" << (plan_success ? "true" : "false")
        << ",\"result_reason\":\"" << reason << "\",\"local_replan_request_sequence\":" << current_attempt_replan_sequence_
        << ",\"local_target_occupancy\":" << local_target_occupancy_
        << ",\"progress_before_target_selection\":" << attempt_progress_before_
        << ",\"progress_after_target_selection\":" << attempt_progress_after_
        << ",\"progress_after_optimizer\":" << planner_manager_->global_data_.last_progress_time_
        << ",\"duration\":" << planner_manager_->global_data_.global_duration_
        << ",\"local_target_time\":" << local_target_time_
        << ",\"local_target_position\":[" << local_target_pt_.x() << ',' << local_target_pt_.y() << ',' << local_target_pt_.z()
        << "],\"local_target_velocity\":[" << local_target_vel_.x() << ',' << local_target_vel_.y() << ',' << local_target_vel_.z()
        << "]}}";
    std_msgs::msg::String receipt; receipt.data = out.str();
    {
      r107_transport::Publication observation(3, 1, reference_epoch_, localization_generation_,
          reference_revision_, candidate_attempt_id_, -1, -1, -1, receipt.data.size(),
          plan_success ? 1 : 0);
      if (r107_transport::probe().enabled())
        try { observation.cached_source(last_odom_stamp_ns_, node_->now().nanoseconds()); }
        catch (...) { r107_transport::probe().observation_error(); }
      reference_attempt_pub_->publish(receipt);
      observation.returned();
    }
    if (reference_protocol_ack_enabled_) reference_protocol_publication_.attempt_published();
    if (r74_protocol_observation_enabled_)
      r74ObserveString(reference_attempt_pub_->get_topic_name(), receipt.data,
          r74_observation_ && r74_observation_->first_failure_pending);
    if (r74_protocol_observation_enabled_) r74MaybePublishObservation(true);
  }

  void SCANReplanFSM::publishTrajectoryMetadata(const scan_planner_msgs::msg::Bspline &trajectory, bool emergency_stop)
  {
    if (navi_mode_ != NAVI_MODE::REFERENCE_PATH || (!reference_ready_ && !emergency_stop)) return;
    // Route scope plus trajectory id/start_time associates the unchanged
    // Bspline publication without duplicating its local geometry in JSON.
    std::ostringstream out;
    out << std::setprecision(17);
    auto vector = [&out](const Eigen::Vector3d &v) {
      out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
    };
    out << "{\"schema\":3,\"run_id\":\"" << reference_run_id_
        << "\",\"reference_epoch\":" << reference_epoch_ << ",\"localization_generation\":" << localization_generation_
        << ",\"planner_process_id\":" << getpid() << ",\"reference_revision_scope\":\"planner_process_lifetime_requires_outer_epoch\","
        << "\"publication_role\":\"candidate_only\",\"actual_active\":false,"
        << "\"outer_admission_ack_received\":false,\"candidate_kind\":\""
        << (emergency_stop ? "emergency_stop" : "motion") << "\",\"reference_ready\":"
        << (reference_ready_ ? "true" : "false") << ",\"committed_reference_revision\":"
        << reference_revision_
        << ",\"route_id\":" << (int64_t(reference_stamp_.sec) * 1000000000LL + reference_stamp_.nanosec)
        << ",\"route_version\":" << reference_revision_
        << ",\"reference_original_duration\":" << reference_original_duration_
        << ",\"global_endpoint_local_grid\":{\"occupancy\":" << global_endpoint_local_occupancy_
        << ",\"scope\":\"" << (external_reference_protocol_ ? "diagnostic_only_global_reference" : "endpoint_gate") << "\"}"
        << ",\"attempt\":{\"id\":" << candidate_attempt_id_ << ",\"origin\":\"" << attempt_origin_
        << "\",\"use_poly_init\":" << (attempt_use_poly_init_ ? "true" : "false")
        << ",\"random_poly\":" << (attempt_random_poly_ ? "true" : "false")
        << ",\"local_replan_request_sequence\":" << current_attempt_replan_sequence_
        << ",\"progress_before_target_selection\":" << attempt_progress_before_
        << ",\"progress_after_target_selection\":" << attempt_progress_after_ << '}';
    if (reference_ready_)
    {
      out << ",\"global_state\":{\"duration\":" << planner_manager_->global_data_.global_duration_
          << ",\"progress\":" << planner_manager_->global_data_.last_progress_time_ << '}';
    }
    out << ",\"local_target\":{\"valid\":" << (emergency_stop ? "false" : "true")
        << ",\"occupancy\":" << (emergency_stop ? -1 : local_target_occupancy_)
        << ",\"time\":" << (emergency_stop ? 0.0 : local_target_time_) << ",\"position\":";
    vector(emergency_stop ? Eigen::Vector3d::Zero() : local_target_pt_);
    out << ",\"velocity\":"; vector(emergency_stop ? Eigen::Vector3d::Zero() : local_target_vel_);
    out << "},\"reference_stamp\":[" << reference_stamp_.sec << ',' << reference_stamp_.nanosec
        << "],\"body_goal\":";
    vector(reference_body_goal_);
    out << ",\"adjusted_body_goal\":"; vector(reference_revision_ ? end_pt_ : Eigen::Vector3d::Zero());
    out << ",\"start_state\":{\"boundary_kind\":\"" << (emergency_stop ? "stationary_emergency" : "measured")
        << "\",\"position\":"; vector(emergency_stop ? odom_pos_ : start_pt_);
    out << ",\"measured_velocity\":"; vector(odom_vel_);
    out << ",\"used_velocity\":"; vector(emergency_stop ? Eigen::Vector3d::Zero() : start_vel_);
    out << ",\"used_acceleration\":"; vector(emergency_stop ? Eigen::Vector3d::Zero() : start_acc_);
    out << ",\"frozen_reference_boundary\":" << (go2_execution_frozen_ ? "true" : "false")
        << "},\"trajectory_id\":" << trajectory.traj_id
        << ",\"trajectory_start_time\":[" << trajectory.start_time.sec << ','
        << trajectory.start_time.nanosec << "]}";
    std_msgs::msg::String metadata; metadata.data = out.str();
    {
      r107_transport::Publication observation(4, 3, reference_epoch_, localization_generation_,
          reference_revision_, candidate_attempt_id_, trajectory.traj_id,
          trajectory.start_time.sec, trajectory.start_time.nanosec, metadata.data.size(),
          emergency_stop ? -1 : 1);
      if (r107_transport::probe().enabled())
        try { observation.cached_source(last_odom_stamp_ns_, node_->now().nanoseconds()); }
        catch (...) { r107_transport::probe().observation_error(); }
      trajectory_metadata_pub_->publish(metadata);
      observation.returned();
    }
    if (reference_protocol_ack_enabled_)
      reference_protocol_publication_.metadata_published(trajectory.traj_id, trajectory.start_time.sec,
          trajectory.start_time.nanosec, !emergency_stop);
    if (r74_protocol_observation_enabled_)
      r74ObserveString(trajectory_metadata_pub_->get_topic_name(), metadata.data);
  }

  bool SCANReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    if(navi_mode_==NAVI_MODE::REFERENCE_PATH){
      auto& global=planner_manager_->global_data_;
      if((global.getPosition(global.last_progress_time_)-start_pt_).norm()>planning_horizon_){
        const double recovered=recover_connected_reference_progress(global.last_progress_time_,
            global.global_duration_,t_step,start_pt_,planning_horizon_,[&](double time){return global.getPosition(time);});
        if(recovered<=global.last_progress_time_)return false;
        RCLCPP_INFO(node_->get_logger(),"Recovered connected reference progress %.3f -> %.3f",global.last_progress_time_,recovered);
        global.last_progress_time_=recovered;
      }
    }
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_ ||
        (navi_mode_ == NAVI_MODE::REFERENCE_PATH && t >= planner_manager_->global_data_.global_duration_)) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      adjusted_t=connected_free_reference_target(target_t,
          std::max(0.0,dist_min_t),planner_manager_->global_data_.global_duration_,
          t_step,planning_horizon_,[&](double time){return planner_manager_->global_data_.getPosition(time);},
          [&](const Eigen::Vector3d&point){return targetOccupancy(point)==0;});
      found_free_target=adjusted_t>=0.;
      if(found_free_target)local_target_pt_=planner_manager_->global_data_.getPosition(adjusted_t);

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "No collision-free target on bounded connected reference prefix");
        return false;
      }
    }

    local_target_time_ = target_t;
    // A stopped external-reference boundary must not require a moving
    // terminal tangent from an unexecuted global reference. Keep the exact
    // target point/time and let the original optimizer and outer gates decide.
    const bool stopped_external_reference = external_reference_protocol_ &&
        navi_mode_ == NAVI_MODE::REFERENCE_PATH && go2_execution_frozen_;
    if (stopped_external_reference ||
        (end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
    return true;
  }

} // namespace scan_planner
