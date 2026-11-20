#include "multicamera_shadow.hpp"
#include "visual_snapshot.h"
#include <fast_livo2_core/core/diagnostics.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

using Json=nlohmann::json;
using namespace dynanav_shadow;
namespace fs=std::filesystem;
void require(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
template<class M> Json matrix(const M&m){Json a=Json::array();for(int i=0;i<m.rows();++i){Json r=Json::array();for(int j=0;j<m.cols();++j)r.push_back(m(i,j));a.push_back(r);}return a;}
int main(int argc,char **argv) {
 try {
    require(argc==5,"usage: replay CAPTURE NATIVE_SNAPSHOTS OUTPUT fixed|exposure");
    const fs::path capture=fs::canonical(argv[1]), snapshots=fs::canonical(argv[2]), output=argv[3];
    require(!fs::exists(output),"replay output exists");
    const bool estimate=std::string(argv[4])=="exposure";
    require(estimate || std::string(argv[4])=="fixed","unknown exposure model");
    Json calibration;{std::ifstream f(capture/"multicamera/calibration.json");f>>calibration;}
    require(calibration.at("generation")==capture.string(),"wrong calibration generation");
    BranchOptions options;options.normal_en=true;options.pyramid_levels=4;
    options.estimate_per_frame_exposure=estimate;options.exposure_model_variance=.1;
    std::vector<std::unique_ptr<Branch>> branches;
    std::vector<Camera> cameras;
    for(const auto &c:calibration.at("cameras")) {
        Camera model;model.id=c.at("camera_id");const auto &k=c.at("intrinsics_fx_fy_cx_cy");
        model.fx=k[0];model.fy=k[1];model.cx=k[2];model.cy=k[3];
        for(int i=0;i<3;++i) {model.translation_camera_from_imu[i]=c.at("translation_camera_from_imu_m")[i];
            for(int j=0;j<3;++j)model.R_camera_from_imu(i,j)=c.at("R_camera_from_imu")[i][j];}
        cameras.push_back(model);branches.push_back(std::make_unique<Branch>(model,options));
    }
    require(branches.size()>=3 && branches.size()<=6,"expected 3..6 camera calibrations");
    Branch front_control(cameras[0],options);
    std::map<uint64_t,std::map<std::string,Json>> images;
    {std::ifstream file(capture/"multicamera/index.jsonl");std::string line;
     while(std::getline(file,line)) {auto j=Json::parse(line);require(j.at("generation")==capture.string(),"wrong image generation");
        require(!j.at("time_evidence").at("body_now_substituted").get<bool>(),"retimestamped camera frame");
        images[j.at("stamp_ns").get<uint64_t>()][j.at("camera_id")]=j;}}
    std::ofstream stream(output);require(bool(stream),"cannot open replay output");
    size_t frames=0;
    for(const auto &group:images) {
        std::ostringstream name;name<<std::setw(18)<<std::setfill('0')<<group.first;
        const auto source=snapshots/(name.str()+".cbor");
        if(group.second.size()!=branches.size() || !fs::is_regular_file(source))continue;
        auto owned=fastlivo_visual_snapshot::read(source);const auto &meta=owned.metadata;
        require(meta.at("generation")==capture.string() && meta.at("source_stamp_ns")==group.first,"wrong native snapshot generation/source");
        require(meta.at("map_frame")=="dynanav_lio_map" && meta.at("child_frame")=="go2_imu","wrong estimator frames");
        require(std::abs(meta.at("native_target_stamp_ns").get<int64_t>()-int64_t(group.first))<=1,"native target is another image epoch");
        const auto frozen=fastlivo_visual_snapshot::Json::to_cbor(fastlivo_visual_snapshot::encode(owned.state,owned.points,owned.planes,
            group.first,meta.at("native_target_stamp_ns"),capture.string(),"dynanav_lio_map","go2_imu",meta.at("voxel_size_m")));
        Epoch epoch{capture.string(),source.string(),source.string(),group.first,owned.state,CovarianceSource::NativeInternal};
        Json report={{"source_stamp_ns",group.first},{"native_target_stamp_ns",meta.at("native_target_stamp_ns")},
            {"generation",capture.string()},{"native_snapshot",source.string()},{"node_count",meta.at("node_count")},
            {"input_points",owned.points.size()},{"camera_count",branches.size()},{"exposure_model",estimate?"independent_per_image_N(1,0.1)":"fixed_unit"},
            {"common_covariance","native 18D physical marginal, excluding original front exposure"},
            {"confidence_calibrated",false},{"estimator_feedback_enabled",false},{"cameras",Json::array()}};
        Result front_result;bool front_ok=false;
        for(size_t i=0;i<branches.size();++i) {
            const auto &record=group.second.at(cameras[i].id);
            const auto image_path=fs::canonical(capture/"multicamera"/record.at("rgb_path").get<std::string>());
            require(image_path.string().find((capture/"multicamera").string()+"/")==0,"image path escapes generation");
            const auto image=cv::imread(image_path.string(),cv::IMREAD_COLOR);
            auto start=std::chrono::steady_clock::now();Json camera={{"camera_id",cameras[i].id}};
            try {
                fastlivo_diag::Logger::instance().set_context(100+i,group.first);
                auto result=branches[i]->run(epoch,record.at("stamp_ns"),image,owned.points,owned.planes);
                camera["branch_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
                camera["tracked_points"]=result.tracked_points;camera["retained_update"]=result.retained_update;
                // Read the actual selected native patch centres. These are not all
                // projected LIO points, nor statistically independent observations.
                camera["tracked_feature_centers_prior_uv"]=Json::array();
                camera["tracked_feature_centers_posterior_uv"]=Json::array();
                const auto &model=cameras[i];const auto &manager=branches[i]->manager();
                require(manager.visual_submap!=nullptr,"missing native visual submap");
                for(const auto *point:manager.visual_submap->voxel_points) {
                    require(point!=nullptr,"null selected native visual point");
                    const V3D before=model.R_camera_from_imu*(epoch.state.rot_end.transpose()*(point->pos_-epoch.state.pos_end))+model.translation_camera_from_imu;
                    const V3D after=model.R_camera_from_imu*(result.posterior.rot_end.transpose()*(point->pos_-result.posterior.pos_end))+model.translation_camera_from_imu;
                    require(before.allFinite() && after.allFinite() && before.z()>0 && after.z()>0,"invalid selected-point camera projection");
                    camera["tracked_feature_centers_prior_uv"].push_back({model.fx*before.x()/before.z()+model.cx,model.fy*before.y()/before.z()+model.cy});
                    camera["tracked_feature_centers_posterior_uv"].push_back({model.fx*after.x()/after.z()+model.cx,model.fy*after.y()/after.z()+model.cy});
                }
                require(camera["tracked_feature_centers_prior_uv"].size()==size_t(result.tracked_points),"selected visual point count mismatch");
                camera["delta_rotation_body_position_world"]=std::vector<double>(result.pose_delta.data(),result.pose_delta.data()+6);
                camera["posterior_pose_covariance"]=matrix(result.posterior.cov.topLeftCorner<6,6>());
                camera["posterior_physical_covariance"]=matrix(result.physical_covariance);
                camera["inverse_exposure"]=result.posterior.inv_expo_time;camera["inverse_exposure_variance"]=result.posterior.cov(6,6);
                camera["physical_covariance_source"]="native_internal_marginal_after_actual_visual_update";
                camera["exposure_prior_source"]="explicit per-image nuisance model; not calibrated hardware covariance";
                if(i==0){front_result=result;front_ok=true;
                    auto control_started=std::chrono::steady_clock::now();
                    fastlivo_diag::Logger::instance().set_context(200,group.first);
                    auto independent=front_control.run(epoch,group.first,image,owned.points,owned.planes);
                    camera["single_front_control_wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-control_started).count();
                    camera["single_vs_multi_pose_delta_max_difference"]=(independent.pose_delta-result.pose_delta).cwiseAbs().maxCoeff();
                    camera["single_vs_multi_covariance_max_difference"]=(independent.physical_covariance-result.physical_covariance).cwiseAbs().maxCoeff();}
            }catch(const std::exception &error){camera["error"]=error.what();camera["retained_update"]=false;}
            camera["wall_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
            report["cameras"].push_back(camera);
        }
        const auto after=fastlivo_visual_snapshot::Json::to_cbor(fastlivo_visual_snapshot::encode(owned.state,owned.points,owned.planes,
            group.first,meta.at("native_target_stamp_ns"),capture.string(),"dynanav_lio_map","go2_imu",meta.at("voxel_size_m")));
        require(frozen==after,"shadow solver mutated shared native snapshot");
        report["native_snapshot_unchanged"]=true;
        stream<<report.dump()<<'\n';stream.flush();++frames;
    }
    require(frames>1,"no mature paired native snapshots");
    std::cout<<"REPLAY_COMPLETED frames="<<frames<<" exposure="<<estimate<<'\n';return 0;
 }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
