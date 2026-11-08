#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace scan_planner {
// Recover only on the next connected route prefix, not the whole future route.
// Arc budget, adjacent-sample continuity and the first local distance minimum
// prevent a nearby later floor/corridor from becoming a cursor shortcut.
template<class Position>
double recover_connected_reference_progress(double cursor,double end,double step,
    const Eigen::Vector3d& measured,double horizon,Position position) {
  if(!measured.allFinite()||!std::isfinite(cursor)||!std::isfinite(end)||
     !std::isfinite(step)||step<=0||end<cursor||!std::isfinite(horizon)||horizon<=0)return cursor;
  Eigen::Vector3d previous=position(cursor);
  if(!previous.allFinite())return cursor;
  double best=cursor,best_distance=(previous-measured).norm(),arc=0.;
  const double arc_budget=2*horizon,maximum_step=std::max(.2,horizon*.3);
  for(size_t samples=0;samples<512;++samples){
    const double time=std::min(end,cursor+(samples+1)*step);
    const Eigen::Vector3d point=position(time);
    if(!point.allFinite())break;
    const double segment=(point-previous).norm();
    if(segment>maximum_step||arc+segment>arc_budget)break;
    arc+=segment;previous=point;
    const double distance=(point-measured).norm();
    if(distance<best_distance-1e-6){best_distance=distance;best=time;}
    else if(best>cursor&&distance>best_distance+.05)break;
    if(time>=end)break;
  }
  return best_distance<horizon?best:cursor;
}

// An occupied target may only move along adjacent connected reference samples.
// Never search the whole route for a free point on a later corridor or floor.
template<class Position,class IsFree>
double connected_free_reference_target(double target,double minimum,double end,double step,
    double horizon,Position position,IsFree is_free) {
  if(!std::isfinite(target)||!std::isfinite(minimum)||!std::isfinite(end)||
     !std::isfinite(step)||step<=0||!std::isfinite(horizon)||horizon<=0||minimum>target||target>end)return -1.;
  Eigen::Vector3d previous[2]={position(target),position(target)};
  if(!previous[0].allFinite())return -1.;
  double arc[2]={0.,0.},last[2]={target,target};bool active[2]={target<end,target>minimum};
  const double budget=2*horizon,maximum_step=std::max(.2,.3*horizon);
  for(size_t sample=1;sample<=512&&(active[0]||active[1]);++sample){
    for(int direction=0;direction<2;++direction){
      if(!active[direction])continue;
      const double time=std::max(minimum,std::min(end,target+(direction?-1.:1.)*sample*step));
      if(time==last[direction]){active[direction]=false;continue;}
      const Eigen::Vector3d point=position(time);const double length=(point-previous[direction]).norm();
      if(!point.allFinite()||length>maximum_step||arc[direction]+length>budget){active[direction]=false;continue;}
      arc[direction]+=length;previous[direction]=point;last[direction]=time;
      if(is_free(point))return time;
      if(time==minimum||time==end)active[direction]=false;
    }
  }
  return -1.;
}
}
