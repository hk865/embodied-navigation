// Independent numerical checks around mechanically extracted production code.
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <traj_utils/polynomial_traj.h>
using namespace std;
struct Output { PolynomialTraj trajectory; double duration, dt; vector<Eigen::Vector3d> points, derivatives; };
static bool generate(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                     Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                     Eigen::Vector3d local_target_vel, bool flag_randomPolyTraj,
                     double spacing, Output &output) {
    struct Parameters { double max_vel_=1., max_acc_=4., ctrl_pt_dist; } pp_;
    pp_.ctrl_pt_dist=spacing;
    int continuous_failures_count_=3;
#include "local_guard.inc"
    double ts=(start_pt-local_target_pt).norm()>.1 ? spacing/pp_.max_vel_*1.2 : spacing/pp_.max_vel_*5;
    vector<Eigen::Vector3d> point_set,start_end_derivatives;
#include "local_initialization.inc"
    output={gl_traj,time,ts,point_set,start_end_derivatives};
    return true;
}
static int failures=0,checks=0;
static void check(bool passed,const string &name) {
    ++checks;
    if(!passed){++failures;cout << "FAIL " << name << '\n';}
}
static void near(const Eigen::Vector3d &actual,const Eigen::Vector3d &expected,const string &name) {
    check(actual.allFinite() && (actual-expected).norm()<1e-7,name);
}
int main() {
    const Eigen::Vector3d p0(.17,-.23,.11),v0(.13,-.08,.02),a0(.11,.07,-.05),v1(.24,.09,-.03);
    for(double distance: {0.,.049}) {
        Output out;
        check(!generate(p0,v0,a0,p0+Eigen::Vector3d(distance,0,0),v1,false,.15,out),"existing_close_goal_early_return");
    }
    for(bool random: {false,true}) for(double distance: {.051,.18,.73,1.137,2.413}) for(double spacing:{.10,.23}) {
        // Non-axis-aligned displacement avoids an undefined randomized cross product.
        const Eigen::Vector3d p1=p0+distance*Eigen::Vector3d(.8,.5,.33166247903554).normalized();
        const double original_time=distance<.25 ? sqrt(distance/4.) : distance-.25+.5;
        Output out; srand(19);
        check(generate(p0,v0,a0,p1,v1,random,spacing,out),"nonzero_short_or_regular_generation");
        check(abs(out.duration-2*original_time)<1e-10,"local_total_time_exactly_doubled");
        auto durations=out.trajectory.getTimes();
        check(durations.size()==(random?2:1),"preserve_segment_count");
        for(double t:durations) check(abs(t-(random?original_time:2*original_time))<1e-10,"each_endpoint_segment_time_doubled");
        near(out.trajectory.evaluate(0),p0,"trajectory_initial_position");
        near(out.trajectory.evaluate(out.duration),p1,"trajectory_final_position");
        near(out.trajectory.evaluateVel(0),v0,"trajectory_initial_velocity");
        near(out.trajectory.evaluateVel(out.duration),v1,"trajectory_final_velocity");
        near(out.trajectory.evaluateAcc(0),a0,"trajectory_initial_acceleration");
        near(out.trajectory.evaluateAcc(out.duration),Eigen::Vector3d::Zero(),"trajectory_final_acceleration");
        check(out.points.size()>=7,"minimum_seven_points");
        check(abs((out.points.size()-1)*out.dt-out.duration)<1e-10,"uniform_sample_grid_covers_full_duration");
        near(out.points.front(),p0,"samples_include_initial_position");
        near(out.points.back(),p1,"samples_include_true_endpoint");
        for(size_t k=0;k<out.points.size();++k) {
            double t=out.duration*k/(out.points.size()-1);
            near(out.points[k],out.trajectory.evaluate(t),"samples_uniformly_cover_entire_polynomial");
            if(k)check((out.points[k]-out.points[k-1]).norm()<=spacing*1.5+1e-10,"original_spacing_bound");
        }
        check(out.derivatives.size()==4,"four_boundary_derivatives");
        near(out.derivatives[0],v0,"parameterization_initial_velocity");
        near(out.derivatives[1],out.trajectory.evaluateVel(out.duration),"parameterization_velocity_at_complete_goal_time");
        near(out.derivatives[2],a0,"parameterization_initial_acceleration");
        near(out.derivatives[3],out.trajectory.evaluateAcc(out.duration),"parameterization_acceleration_at_complete_goal_time");
        cout << "CASE " << random << ' ' << distance << ' ' << spacing << " duration=" << out.duration << " points=" << out.points.size() << " endpoint_error=" << (out.points.back()-p1).norm() << '\n';
    }
    cout << "CHECKS " << checks << " FAILURES " << failures << '\n';
    return failures?1:0;
}
