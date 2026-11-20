// Native FAST-LIVO2 execution, synthetic textured geometry. No sensor/GPU claim.
#include "multicamera_shadow.hpp"
#include <iostream>
#include <set>
#include <nlohmann/json.hpp>
#include <fstream>

using namespace dynanav_shadow;
void require(bool value,const char *why) { if (!value) throw std::runtime_error(why); }
Camera calibration(int i) {
    Camera c; c.id="camera_"+std::to_string(i);
    M3D Ropt; Ropt<<0,0,1,-1,0,0,0,-1,0;
    M3D yaw=Eigen::AngleAxisd(i*M_PI/3.,V3D::UnitZ()).toRotationMatrix();
    c.R_camera_from_imu=(yaw*Ropt).transpose();
    c.translation_camera_from_imu=-c.R_camera_from_imu*(yaw*V3D(.3,0,.13));
    return c;
}
cv::Mat texture() {
    cv::Mat im(512,512,CV_8UC3);
    for (int y=0;y<512;++y) for (int x=0;x<512;++x) {
        unsigned char v=100+50*std::sin(x*.19)*std::cos(y*.13)+20*std::sin((x+y)*.037);
        im.at<cv::Vec3b>(y,x)=cv::Vec3b(v,v,v);
    }
    return im;
}
int main(int argc,char **argv) {
    try {
        std::vector<pointWithVar> points;
        for (int i=0;i<6;++i) {
            auto c=calibration(i);
            for (int y=-12;y<=12;++y) for (int x=-12;x<=12;++x) {
                pointWithVar p;
                p.point_w=c.R_camera_from_imu.transpose()*(V3D(x*.09,y*.09,3.)-c.translation_camera_from_imu);
                p.normal=c.R_camera_from_imu.transpose()*V3D(0,0,1);
                p.var=M3D::Identity()*.001; // synthetic fixture's declared model
                points.push_back(p);
            }
        }
        const auto original=points;
        Epoch epoch{"synthetic_generation","lio_1","frozen_map_1",1000000000ULL,StatesGroup{},CovarianceSource::NativeInternal};
        epoch.state.cov.setIdentity(); epoch.state.cov*=.001;
        const auto prior=epoch.state;
        const cv::Mat image=texture(), unchanged=image.clone();
        std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> planes;
        std::vector<std::unique_ptr<Branch>> branches;
        Branch control(calibration(0));
        control.run(epoch,epoch.stamp_ns,image,points,planes);
        std::set<const void*> caches;
        for (int i=0;i<6;++i) {
            branches.emplace_back(std::make_unique<Branch>(calibration(i)));
            auto r=branches.back()->run(epoch,epoch.stamp_ns,image,points,planes);
            require(!r.retained_update,"first-frame cache seeding falsely counted as correction");
            caches.insert(branches.back()->manager().visual_submap);
        }
        require(caches.size()==6,"cameras share mutable patch caches");
        epoch.stamp_ns+=100000000; epoch.prior_id="lio_2";
        epoch.state.pos_end=V3D(.003,.002,0);
        const auto common=epoch.state;
        int updated=0;
        nlohmann::json records=nlohmann::json::array();
        Result front;
        for (int i=0;i<6;++i) {
            auto r=branches[i]->run(epoch,epoch.stamp_ns,image,points,planes);
            require(r.posterior.cov.allFinite() && r.pose_delta.allFinite(),"nonfinite numerical branch output");
            require(r.tracked_points>0,"textured fixture has no actual tracked points");
            updated+=r.retained_update;
            if (i==0) front=r;
            require(Eigen::LLT<Eigen::Matrix<double,6,6>>(r.posterior.cov.topLeftCorner<6,6>()).info()==Eigen::Success,
                    "native branch pose covariance is not SPD");
            std::vector<double> delta(6);
            std::vector<std::vector<double>> covariance(6,std::vector<double>(6));
            for (int row=0;row<6;++row) {
                delta[row]=r.pose_delta[row];
                for (int col=0;col<6;++col) covariance[row][col]=r.posterior.cov(row,col);
            }
            records.push_back({{"camera_id","camera_"+std::to_string(i)},
                {"generation",epoch.generation},{"prior_id",epoch.prior_id},{"map_id",epoch.map_id},
                {"source_stamp_ns",epoch.stamp_ns},{"prior_stamp_ns",epoch.stamp_ns},{"map_stamp_ns",epoch.stamp_ns},
                {"calibration_id","synthetic_calibration_"+std::to_string(i)},
                {"delta_rotation_body_position_world",delta},{"posterior_pose_covariance",covariance},
                {"covariance_source","native_internal"},{"retained_update",r.retained_update}});
            require((epoch.state-common).norm()==0 && (epoch.state.cov-common.cov).norm()==0,"branch mutated shared prior");
            std::cout<<"branch "<<i<<" tracked="<<r.tracked_points<<" retained="<<r.retained_update<<" delta="<<r.pose_delta.transpose()<<'\n';
        }
        require(updated==6,"not all six native branches produced numerical corrections");
        auto independent=control.run(epoch,epoch.stamp_ns,image,points,planes);
        require((independent.pose_delta-front.pose_delta).norm()<1e-13 &&
                (independent.posterior.cov-front.posterior.cov).norm()<1e-13,
                "front shadow changed when scheduled after the other five cameras");
        require(cv::norm(image,unchanged,cv::NORM_INF)==0,"native in-place grayscale changed caller RGB");
        for (size_t i=0;i<points.size();++i)
            require((points[i].point_w-original[i].point_w).norm()==0 &&
                    (points[i].normal-original[i].normal).norm()==0 &&
                    (points[i].var-original[i].var).norm()==0,"branch changed shared geometry");
        auto bad=epoch; bad.stamp_ns+=100000000; bad.covariance_source=CovarianceSource::UnavailableRosPose;
        bool rejected=false;
        try { branches[0]->run(bad,bad.stamp_ns,image,points,planes); } catch(const std::invalid_argument&) {rejected=true;}
        require(rejected,"placeholder covariance accepted");
        rejected=false; bad=epoch; bad.stamp_ns+=100000000;
        try { branches[0]->run(bad,bad.stamp_ns+1,image,points,planes); } catch(const std::invalid_argument&) {rejected=true;}
        require(rejected,"asynchronous image borrowed same-epoch prior");
        bad=epoch; bad.state.cov(0,6)=bad.state.cov(6,0)=1e-5;
        auto separate=camera_prior(bad.state,.1);
        require((physical_marginal(separate)-physical_marginal(bad.state)).norm()==0 &&
                separate.cov(0,6)==0 && separate.cov(6,0)==0,
                "front exposure was not marginalized before creating private camera nuisance");
        require((prior.pos_end).norm()==0,"fixture changed earlier prior");
        std::cout<<"PASS six isolated native numerical visual updates; immutable prior/points/RGB; exact epoch and covariance-source rejection\n";
        if (argc>1) {std::ofstream file(argv[1]);file<<nlohmann::json({{"fixture","synthetic_textured_geometry_native_solver"},{"packets",records}}).dump(2);}
        return 0;
    } catch(const std::exception &error) {std::cerr<<error.what()<<'\n';return 1;}
}
