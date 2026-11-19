// Independent acceptance: real patched GridMap, never a substitute occupancy model.
#include <plan_env/grid_map.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <rcl/time.h>
#include <iostream>
#include <stdexcept>
using Json = nlohmann::json;

class GridMapTestPeer {
public:
    static void configure(GridMap &g, rclcpp::Node &node) {
        g.node_ = &node;
        g.mp_ = MappingParameters{}; g.md_ = MappingData{};
        auto &p = g.mp_; auto &d = g.md_;
        p.resolution_ = .08; p.resolution_inv_ = 12.5;
        p.map_voxel_num_ = Eigen::Vector3i::Constant(64);
        p.map_origin_idx_ = p.map_voxel_num_ / 2;
        g.updateMapBoundaryFromIndex();
        p.frame_id_ = "map"; p.sliding_map_frame_id_ = "map";
        p.local_update_range_ = Eigen::Vector3d::Constant(2.56);
        p.map_sliding_thresh_vox_ = 1;
        p.double_cylinder_radius_ = .25; p.double_cylinder_offset_ = .18;
        p.obstacles_inflation_z_up = .12; p.obstacles_inflation_z_down = .12;
        p.clamp_min_log_ = -2.; p.clamp_max_log_ = 4.; p.min_occupancy_log_ = 1.; p.unknown_flag_ = .01;
        int size = p.map_voxel_num_.prod();
        d.occupancy_buffer_.assign(size, -2.01);
        d.occupancy_buffer_inflate_.assign(size, 0);
        d.occupancy_buffer_inflate_cnt_.assign(size, 0);
        d.count_hit_.assign(size, 0); d.count_hit_and_miss_.assign(size, 0);
        d.flag_rayend_.assign(size, -1); d.flag_traverse_.assign(size, -1);
        d.ray_q_ = Eigen::Quaterniond::Identity();
        g.rebuildInflationOffsets();
        g.input_has_body_ = true; g.input_has_cloud_ = true; g.input_has_sensor_ = true;
        g.input_map_revision_ = 1;
        g.input_body_pose_.header.frame_id = "map";
        g.input_body_pose_.child_frame_id = "base_link";
        g.input_body_pose_.pose.pose.orientation.w = 1.;
        g.input_body_pose_.pose.pose.position.x = 2.04;
        g.input_body_pose_.pose.pose.position.y = 2.04;
        g.input_body_pose_.pose.pose.position.z = 2.06;
        refreshInputs(g, node);
    }
    static int64_t refreshInputs(GridMap &g, rclcpp::Node &node) {
        auto now = node.now();
        g.input_body_pose_.header.stamp = now;
        g.input_fusion_cloud_header_ = g.input_body_pose_.header;
        g.input_fusion_sensor_pose_ = g.input_body_pose_;
        g.input_fusion_sim_s_ = now.seconds();
        return now.nanoseconds();
    }
    static void occupied(GridMap &g, Eigen::Vector3i id, bool value=true) {
        g.applyOccupancyUpdate(id, value ? 4. : -2.);
    }
    static void label(GridMap &g, const Json &json) {
        auto msg = std::make_shared<std_msgs::msg::String>(); msg->data = json.dump();
        g.stairLabelsCallback(msg); g.refreshStairLabels();
    }
    static void refresh(GridMap &g) { g.refreshStairLabels(); }
    static void pitch(GridMap &g, double angle) {
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
        auto &o = g.input_body_pose_.pose.pose.orientation;
        o.x=q.x(); o.y=q.y(); o.z=q.z(); o.w=q.w();
    }
    static int inflated(GridMap &g, Eigen::Vector3i id) {
        return g.md_.occupancy_buffer_inflate_[g.toAddress(id)];
    }
    static int count(GridMap &g, Eigen::Vector3i id) {
        return g.md_.occupancy_buffer_inflate_cnt_[g.toAddress(id)];
    }
    static void resetCell(GridMap &g, Eigen::Vector3i id) { g.resetCellByAddress(g.toAddress(id)); }
    static void slide(GridMap &g, Eigen::Vector3d center) {
        g.mp_.map_sliding_en_=true; g.updateSlidingMap(center);
    }
};

static void check(bool yes, const std::string &name) {
    if (!yes) throw std::runtime_error(name);
    std::cout << "PASS " << name << '\n';
}
static Json label(int64_t stamp, Json voxels={{27,25,23}}) {
    return {{"version",1},{"localization_generation",0},{"run_id","acceptance"},{"seq",1},{"startup_nonce","acceptance_nonce"},
            {"frame_id","map"},{"child_frame_id","base_link"},{"source_stamp_ns",stamp},
            {"resolution_m",.08},{"voxel_indices",voxels}};
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stair_native_acceptance");
    node->declare_parameter("fsm.reference_run_id", "acceptance");
    node->declare_parameter("fsm.reference_epoch", 1);
    node->declare_parameter("fsm.localization_generation", 0);
    node->declare_parameter("fsm.child_odometry_ack_nonce", "acceptance_nonce");
    auto clock=node->get_clock()->get_clock_handle();
    check(rcl_enable_ros_time_override(clock)==RCL_RET_OK,"enable_deterministic_clock");
    check(rcl_set_ros_time_override(clock,10000000000LL)==RCL_RET_OK,"set_deterministic_clock");
    const Eigen::Vector3d q(2.04,2.04,2.06);
    const Eigen::Vector3i stair(27,25,23), target(27,25,25), wall(28,25,23);
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        GridMapTestPeer::occupied(g,stair);
        check(g.getInflateOccupancy(q,0.)==1,"unlabelled_source_retains_inflation");
        auto stamp=GridMapTestPeer::refreshInputs(g,*node);
        GridMapTestPeer::label(g,label(stamp));
        check(g.getOccupancy(stair)==1,"label_retains_raw_occupied_stair");
        check(g.getInflateOccupancy(q,0.)==0,"fresh_stair_no_longer_inflates_front_query");
        for (int x=-3;x<=3;++x) for(int y=-3;y<=3;++y) for(int z=-2;z<=2;++z) {
            if (x==0 && y==0 && z==0) continue;
            if (std::hypot(x*.08,y*.08)<.25)
                check(GridMapTestPeer::inflated(g,stair+Eigen::Vector3i(x,y,z))==0,
                      "all_stair_inflation_offsets_removed");
        }
    }
    for(const std::string field : {"frame_id","child_frame_id","run_id","startup_nonce","seq","localization_generation","source_stamp_ns","resolution_m"}) {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        Json message=label(GridMapTestPeer::refreshInputs(g,*node));
        if(field=="seq" || field=="localization_generation") message[field]=2;
        else if(field=="source_stamp_ns") message[field]=message[field].get<int64_t>()-1000000000;
        else if(field=="resolution_m") message[field]=.10;
        else message[field]="wrong";
        GridMapTestPeer::label(g,message);
        check(g.getInflateOccupancy(q,0.)==1,"invalid_label_preserves_original_inflation_"+field);
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        Json message=label(GridMapTestPeer::refreshInputs(g,*node));
        message["source_stamp_ns"]=message["source_stamp_ns"].get<int64_t>()+1000000000;
        GridMapTestPeer::label(g,message);
        check(g.getInflateOccupancy(q,0.)==1,"future_unfused_label_does_not_relax_inflation");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        GridMapTestPeer::occupied(g,stair); GridMapTestPeer::occupied(g,wall);
        const int before=GridMapTestPeer::count(g,target);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        check(before>=2 && GridMapTestPeer::count(g,target)>=1,"wall_shared_target_keeps_contribution");
        check(g.getInflateOccupancy(q,0.)==1,"wall_shared_target_still_blocks");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        GridMapTestPeer::occupied(g,stair,false); GridMapTestPeer::occupied(g,stair);
        check(g.getInflateOccupancy(q,0.)==1,"deleted_and_reoccupied_voxel_does_not_inherit_label");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        GridMapTestPeer::resetCell(g,stair); GridMapTestPeer::occupied(g,stair);
        check(g.getInflateOccupancy(q,0.)==1,"reset_cell_does_not_inherit_label");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        GridMapTestPeer::slide(g,Eigen::Vector3d(7.68,2.56,2.56));
        GridMapTestPeer::occupied(g,stair+Eigen::Vector3i(64,0,0));
        check(g.getInflateOccupancy(q+Eigen::Vector3d(5.12,0,0),0.)==1,"ring_slot_reuse_does_not_inherit_stair_label");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        Eigen::Vector3i inside(25,25,25); GridMapTestPeer::occupied(g,inside);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node),{{25,25,25}}));
        check(g.getInflateOccupancy(q,0.)==1,"raw_stair_intersecting_body_remains_collision");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        check(g.getInflateOccupancy(q,0.)==0,"expiry_precondition_active_label");
        check(rcl_set_ros_time_override(clock,11000000000LL)==RCL_RET_OK,"advance_clock_for_expiry");
        GridMapTestPeer::refreshInputs(g,*node); GridMapTestPeer::refresh(g);
        check(g.getInflateOccupancy(q,0.)==1,"expired_label_restores_inflation");
        check(rcl_set_ros_time_override(clock,10000000000LL)==RCL_RET_OK,"restore_test_clock");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        GridMapTestPeer::occupied(g,stair);
        check(g.getInflateOccupancy(q,0.)==1,"unknown_voxel_label_not_inherited_by_later_occupancy");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        GridMapTestPeer::occupied(g,Eigen::Vector3i(25,25,25));
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node),{{25,25,25}}));
        const Eigen::Vector3d a(1.04,2.04,2.04), b(3.04,2.04,2.04);
        check(g.getInflateOccupancy(a,0.)==0 && g.getInflateOccupancy(b,0.)==0,
              "translation_sweep_endpoints_clear");
        check(g.queryStairSweep(a,b,0.,0.)!=0,"translation_sweep_intermediate_raw_obstacle_blocks");
        Eigen::MatrixXd control(3,4);
        control.col(0)=a; control.col(1)=a; control.col(2)=b; control.col(3)=b;
        check(g.queryStairControlHull(control)!=0,"bspline_control_hull_cannot_skip_middle_obstacle");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        // Below the level chassis but inside its pitched envelope.
        GridMapTestPeer::occupied(g,Eigen::Vector3i(26,25,23));
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node),{{26,25,23}}));
        check(g.getInflateOccupancy(q,0.)==0,"level_body_above_low_raw_step");
        GridMapTestPeer::pitch(g,std::acos(-1.)/3.);
        check(g.getInflateOccupancy(q,0.)!=0,"current_pitch_body_raw_collision_is_not_ignored");
    }
    {
        GridMap g; GridMapTestPeer::configure(g,*node);
        GridMapTestPeer::occupied(g,Eigen::Vector3i(27,27,25));
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node),{{27,27,25}}));
        const double pi=std::acos(-1.);
        check(g.getInflateOccupancy(q,0.)==0 && g.getInflateOccupancy(q,pi/2.)==0,
              "rotation_sweep_endpoints_clear");
        check(g.queryStairSweep(q,q,0.,pi/2.)!=0,"rotation_sweep_intermediate_raw_obstacle_blocks");
    }
    for(const std::string parameter : {"fsm.reference_epoch", "fsm.localization_generation", "fsm.child_odometry_ack_nonce"}) {
        GridMap g; GridMapTestPeer::configure(g,*node); GridMapTestPeer::occupied(g,stair);
        GridMapTestPeer::label(g,label(GridMapTestPeer::refreshInputs(g,*node)));
        check(g.getInflateOccupancy(q,0.)==0,"identity_change_precondition_active_label");
        if(parameter=="fsm.child_odometry_ack_nonce") node->set_parameter(rclcpp::Parameter(parameter,"replacement_nonce"));
        else node->set_parameter(rclcpp::Parameter(parameter,2));
        GridMapTestPeer::refresh(g);
        check(g.getInflateOccupancy(q,0.)==1,"active_label_revoked_on_identity_change_"+parameter);
        if(parameter=="fsm.child_odometry_ack_nonce") node->set_parameter(rclcpp::Parameter(parameter,"acceptance_nonce"));
        else node->set_parameter(rclcpp::Parameter(parameter,parameter=="fsm.reference_epoch" ? 1 : 0));
    }
    std::cout << "ALL_NATIVE_STAIR_CASES_PASS\n";
    rclcpp::shutdown();
}
