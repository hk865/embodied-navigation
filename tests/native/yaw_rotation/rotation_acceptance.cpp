// Independent cases exercise the real GridMap, not another occupancy model.
#include <plan_env/grid_map.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

class GridMapTestPeer {
public:
    static void configure(GridMap &g, Eigen::Vector3i origin=Eigen::Vector3i::Zero()) {
        g.mp_=MappingParameters{};
        g.md_=MappingData{};
        auto &p=g.mp_; auto &d=g.md_;
        p.resolution_=.08; p.resolution_inv_=12.5;
        p.map_voxel_num_=Eigen::Vector3i::Constant(64);
        p.map_origin_idx_=origin+p.map_voxel_num_/2;
        g.updateMapBoundaryFromIndex();
        p.double_cylinder_radius_=.25;
        p.double_cylinder_offset_=.18;
        p.clamp_min_log_=-2.; p.min_occupancy_log_=1.; p.unknown_flag_=.01;
        const int size=p.map_voxel_num_.prod();
        // Same unobserved/zero-inflation state as the original mapper fixture.
        d.occupancy_buffer_.assign(size,p.clamp_min_log_-.01);
        d.occupancy_buffer_inflate_.assign(size,0);
        d.occupancy_buffer_inflate_cnt_.assign(size,0);
    }
    static void inflated(GridMap &g, const Eigen::Vector3d &position) {
        Eigen::Vector3i id; g.posToIndex(position,id);
        g.md_.occupancy_buffer_inflate_[g.toAddress(id)]=1;
        g.md_.occupancy_buffer_inflate_cnt_[g.toAddress(id)]=1;
    }
    static std::vector<char> buffer(GridMap &g) { return g.md_.occupancy_buffer_inflate_; }
};

static void check(bool condition,const std::string &name) {
    if(!condition) throw std::runtime_error(name);
    std::cout << "PASS " << name << "\n";
}

int main() {
    const double pi=std::acos(-1.);
    {
        GridMap map; GridMapTestPeer::configure(map);
        Eigen::Vector3d p(2.,2.,2.);
        auto before=GridMapTestPeer::buffer(map);
        auto result=map.queryRotationSpace(p,pi-.05,-pi+.05);
        check(result.occupancy==0,"unobserved_zero_inflation_remains_clear");
        check(std::abs(result.delta_yaw-.1)<1e-12,"pi_wrap_uses_short_positive_arc");
        check(result.queries>=2,"nonzero_arc_checks_more_than_start");
        check(before==GridMapTestPeer::buffer(map),"query_does_not_mutate_occupancy");
        auto reverse=map.queryRotationSpace(p,-pi+.05,pi-.05);
        check(reverse.occupancy==0 && std::abs(reverse.delta_yaw+.1)<1e-12,
              "pi_wrap_uses_short_negative_arc");
    }
    {
        GridMap map; GridMapTestPeer::configure(map);
        Eigen::Vector3d p(2.,2.,2.);
        GridMapTestPeer::inflated(map,p+Eigen::Vector3d(.18,0.,0.));
        auto result=map.queryRotationSpace(p,0.,pi/2);
        check(result.occupancy==1 && result.queries==1 && std::abs(result.first_blocking_yaw)<1e-12,
              "start_occupied_preserves_first_nonzero_short_circuit");
    }
    {
        GridMap map; GridMapTestPeer::configure(map);
        Eigen::Vector3d p(2.,2.,2.);
        GridMapTestPeer::inflated(map,p+Eigen::Vector3d(.18/std::sqrt(2.),.18/std::sqrt(2.),0.));
        check(map.getInflateOccupancy(p,0.)==0 && map.getInflateOccupancy(p,pi/2)==0,
              "middle_obstacle_has_clear_endpoints");
        auto result=map.queryRotationSpace(p,0.,pi/2);
        check(result.occupancy==1 && result.queries>1 && result.first_blocking_yaw>0. &&
              result.first_blocking_yaw<pi/2 && map.getInflateOccupancy(p,result.first_blocking_yaw)==1,
              "interior_rotation_obstacle_is_checked_by_original_gridmap");
    }
    {
        GridMap map; GridMapTestPeer::configure(map);
        Eigen::Vector3d p(.18005,2.,2.);
        check(map.getInflateOccupancy(p,pi-.1)==0 && map.getInflateOccupancy(p,-pi+.1)==0,
              "epsilon_map_exit_has_clear_endpoints");
        auto result=map.queryRotationSpace(p,pi-.1,-pi+.1);
        check(result.occupancy==-1 && map.getInflateOccupancy(p,result.first_blocking_yaw)==-1,
              "narrow_interior_map_exit_remains_minus_one");
    }
    {
        GridMap map; GridMapTestPeer::configure(map);
        Eigen::Vector3d p(.1,2.,2.);
        GridMapTestPeer::inflated(map,p+Eigen::Vector3d(.18,0.,0.));
        auto result=map.queryRotationSpace(p,pi,pi-.1);
        check(result.occupancy==-1 && result.queries==1,
              "front_outside_precedes_rear_occupied_exactly_as_original_query");
    }
    {
        GridMap map;
        Eigen::Vector3i origin(117,-65,24);
        GridMapTestPeer::configure(map,origin);
        Eigen::Vector3d p=origin.cast<double>()*.08+Eigen::Vector3d(2.,2.,2.);
        GridMapTestPeer::inflated(map,p+Eigen::Vector3d(.18/std::sqrt(2.),.18/std::sqrt(2.),0.));
        check(map.getInflateOccupancy(p,0.)==0 && map.getInflateOccupancy(p,pi/2)==0,
              "ring_wrapped_middle_obstacle_has_clear_endpoints");
        auto result=map.queryRotationSpace(p,0.,pi/2);
        check(result.occupancy==1 && map.getInflateOccupancy(p,result.first_blocking_yaw)==1,
              "positive_and_negative_global_indices_use_original_ring_addresses");
    }
    std::cout << "ALL_NATIVE_ROTATION_CASES_PASS\n";
}
