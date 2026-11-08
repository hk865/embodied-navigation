#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <plan_manage/measured_start_state.h>
#include <plan_manage/fresh_child_odometry_ack.h>
#include <plan_manage/fresh_reference_protocol.h>
#include <plan_manage/r74_protocol_observation.h>
#include <memory>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;
    MeasuredStartState measured_start_;
    bool reference_ready_{false};
    bool external_reference_protocol_{false};
    std::string reference_run_id_;
    uint64_t reference_epoch_{0}, localization_generation_{0};
    bool child_odometry_ack_enabled_{false};
    bool reference_protocol_ack_enabled_{false};
    ReferenceProtocolPublication reference_protocol_publication_;
    std::string child_odometry_ack_nonce_, child_odometry_ack_gid_;
    FreshChildOdometryIdentity child_odometry_ack_identity_{};
    uint64_t child_odometry_ack_sequence_{0};
    uint64_t last_external_replan_sequence_{0}, pending_external_replan_sequence_{0};
    uint64_t current_attempt_replan_sequence_{0};
    int global_endpoint_local_occupancy_{-1}, local_target_occupancy_{-1};
    int64_t last_odom_stamp_ns_{0};
    std::chrono::steady_clock::time_point last_odom_receive_time_;
    // Process-local committed reference identity. This is not execution admission.
    uint64_t reference_revision_{0};
    uint64_t candidate_attempt_id_{0};
    std::string reference_content_json_;
    uint64_t route_definitions_published_{0}, route_definition_last_bytes_{0};
    double reference_original_duration_{0.0};
    double attempt_progress_before_{0.0}, attempt_progress_after_{0.0};
    double local_target_time_{0.0};
    bool attempt_use_poly_init_{false}, attempt_random_poly_{false};
    std::string attempt_origin_{"none"};
    builtin_interfaces::msg::Time reference_stamp_;
    Eigen::Vector3d reference_body_goal_{Eigen::Vector3d::Zero()};

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr local_replan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_metadata_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reference_attempt_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_definition_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr child_odometry_ack_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;

    // Default-off observation, separate from safety ACK and all motion topics.
    bool r74_protocol_observation_enabled_{false};
    bool r74_observation_warning_emitted_{false};
    std::unique_ptr<r74_observation::Bounded> r74_observation_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr r74_observation_pub_;
    void r74InitializeObservation();
    void r74DisableObservation(const char *reason) noexcept;
    std::string r74ObservationScope();
    void r74ObserveTargetFailure();
    void r74ObserveString(const char *topic, const std::string &data, bool priority=false);
    void r74MaybePublishObservation(bool immediate=false);

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, const std::string &origin); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    bool getLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    void publishTrajectoryMetadata(const scan_planner_msgs::msg::Bspline &trajectory, bool emergency_stop = false);
    void publishRouteDefinition();
    void publishReferenceAttempt(bool plan_success, const std::string &reason = "optimizer_return");
    bool mode3ReferenceUsable() const;
    void invalidateReference(const char *reason);
    void publishChildOdometryAck(const nav_msgs::msg::Odometry &message);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void localReplanCallback(const std_msgs::msg::UInt64MultiArray::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
