#include "visual_snapshot.h"
#include "multicamera_shadow.hpp"
#include <iostream>
#include <stdexcept>
using namespace fastlivo_visual_snapshot;
void require(bool ok,const char *why) {if(!ok)throw std::runtime_error(why);}
int main() {
 try {
    StatesGroup state;state.cov=decltype(state.cov)::Identity()*.01;
    state.cov(0,6)=state.cov(6,0)=.002;state.inv_expo_time=1.7;
    auto prior=dynanav_shadow::camera_prior(state,.1);
    require((dynanav_shadow::physical_marginal(prior)-dynanav_shadow::physical_marginal(state)).norm()==0,
        "Exposure marginalized by conditioning or dropping physical correlations");
    require(prior.cov(0,6)==0 && prior.cov(6,0)==0 && prior.cov(6,6)==.1 && prior.inv_expo_time==1.,
        "Camera inherited front exposure nuisance/correlation");
    require(state.inv_expo_time==1.7 && state.cov(0,6)==.002,"Native prior changed");
    std::vector<pointWithVar> points(1);points[0].point_w=V3D(1,2,3);points[0].normal=V3D(0,0,1);points[0].var=M3D::Identity()*.01;
    auto root=std::make_unique<VoxelOctoTree>(2,0,5,50,.005);
    root->init_octo_=true;root->voxel_center_[0]=root->voxel_center_[1]=root->voxel_center_[2]=0.;root->quater_length_=.125;
    auto child=new VoxelOctoTree(2,1,5,50,.005);root->leaves_[7]=child;
    child->init_octo_=true;child->plane_ptr_->is_plane_=true;child->quater_length_=.0625;
    child->voxel_center_[0]=child->voxel_center_[1]=child->voxel_center_[2]=.125;
    child->plane_ptr_->center_=V3D(1,2,3);child->plane_ptr_->normal_=V3D(0,0,1);
    child->plane_ptr_->plane_var_.setIdentity();child->plane_ptr_->d_=-3.;
    PlaneMap map;map.emplace(VOXEL_LOCATION(0,0,0),root.get());
    auto encoded=encode(state,points,map,123456789,123456788,"generation","map","imu",.5);
    auto decoded=decode(Json::from_cbor(Json::to_cbor(encoded)));
    require((state.cov-decoded.state.cov).norm()==0,"Full native covariance changed on round-trip");
    require(decoded.planes.begin()->second!=root.get(),"Plane root shallow copied");
    auto *found=decoded.planes.begin()->second->find_correspond(V3D(1,1,1));
    require(found!=child && found->plane_ptr_->is_plane_ && found->plane_ptr_->d_==-3.,"Octree query semantics lost");
    require(encode(decoded.state,decoded.points,decoded.planes,123456789,123456788,"generation","map","imu",.5)==encoded,
        "Owned state/geometry snapshot is not bit-exact after CBOR round trip");
    points[0].point_w.setZero();child->plane_ptr_->normal_.setZero();state.cov.setZero();
    require(decoded.points[0].point_w.norm()>0 && found->plane_ptr_->normal_.norm()==1 && decoded.state.cov(0,0)==.01,
        "Snapshot aliases mutable owner buffers");
    std::cout<<"PASS native full covariance and VIO-query octree round trip; no pointer aliases; physical marginal and independent camera exposure\n";
    return 0;
 }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
