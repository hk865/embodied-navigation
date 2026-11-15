#include "LIVMapper.h"
#include "multicamera_native.h"
#include "multicamera_joint.h"
#include <opencv2/imgcodecs.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <map>
#include <array>
namespace fastlivo_multicamera {
namespace fs=std::filesystem;using Json=nlohmann::json;
struct View {std::unique_ptr<vk::PinholeCamera> camera;std::unique_ptr<VIOManager> manager;StatesGroup local,propagated;};
struct Rig {
 std::array<View,3> extra;
 std::array<StatesGroup,4> local,propagated;
 std::map<uint64_t,std::array<fs::path,4>> images;std::ofstream log;
 Rig(LIVMapper&m,const fs::path&archive,const fs::path&output){
  if(fs::exists(output))throw std::runtime_error("native multicamera log must be new");log.open(output);if(!log)throw std::runtime_error("native multicamera log cannot open");
  const auto root=fs::canonical(archive);Json manifest;{std::ifstream in(root/"calibration.json");in>>manifest;}
  if(manifest.at("generation")!=root.parent_path().string()||manifest.at("cameras").size()!=4)throw std::runtime_error("native four-view generation/calibration mismatch");
  Json result;{std::ifstream in(root.parent_path()/"result.json");in>>result;}
  if(result.at("outcome")!="completed")throw std::runtime_error("archive interface requires a completed frozen motion generation; live late frames cannot be injected retrospectively");
  auto &front=*m.vio_manager;
  if(front.inverse_composition_en||front.raycast_en||front.colmap_output_en)throw std::runtime_error("experimental native factors require forward, non-raycast, non-colmap configuration");
  for(int i=0;i<4;++i){
   const auto &c=manifest.at("cameras").at(i);if(c.at("camera_id")!=std::string("camera_")+std::to_string(i))throw std::runtime_error("camera identity/order mismatch");
   const auto k=c.at("intrinsics_fx_fy_cx_cy");M3D r;V3D t;
   for(int a=0;a<3;++a){t[a]=c.at("translation_camera_from_imu_m")[a];for(int b=0;b<3;++b)r(a,b)=c.at("R_camera_from_imu")[a][b];}
   if(!(r.transpose()*r).isApprox(M3D::Identity(),1e-8)||std::abs(r.determinant()-1)>1e-8||!t.allFinite())throw std::runtime_error("invalid native camera extrinsic");
   if(i==0){if(std::abs(front.fx-k[0].get<double>())>1e-5||std::abs(front.fy-k[1].get<double>())>1e-5||!front.Rci.isApprox(r,1e-7)||!front.Pci.isApprox(t,1e-7))throw std::runtime_error("native front and archive calibration disagree");continue;}
   auto &v=extra[i-1];v.camera=std::make_unique<vk::PinholeCamera>(512,512,1.,k[0],k[1],k[2],k[3]);v.manager=std::make_unique<VIOManager>();auto &manager=*v.manager;
   manager.cam=v.camera.get();manager.state=&local[i];manager.state_propagat=&propagated[i];
   manager.grid_size=front.grid_size;manager.grid_n_height=front.grid_n_height;manager.patch_size=front.patch_size;manager.patch_pyrimid_level=front.patch_pyrimid_level;manager.max_iterations=front.max_iterations;manager.img_point_cov=front.img_point_cov;manager.outlier_threshold=front.outlier_threshold;manager.ncc_thre=front.ncc_thre;manager.normal_en=front.normal_en;manager.raycast_en=false;manager.inverse_composition_en=false;manager.exposure_estimate_en=front.exposure_estimate_en;manager.plot_flag=false;manager.has_ref_patch_cache=false;
   manager.setImuToLidarExtrinsic(V3D::Zero(),M3D::Identity());std::vector<double>rv(9),tv(3);for(int a=0;a<3;++a){tv[a]=t[a];for(int b=0;b<3;++b)rv[a*3+b]=r(a,b);}manager.setLidarToCameraExtrinsic(rv,tv);manager.initializeVIO();
  }
  std::ifstream index(root/"index.jsonl");std::string line;
  while(std::getline(index,line)){auto row=Json::parse(line);if(row.at("generation")!=root.parent_path().string()||row.at("time_evidence").at("body_now_substituted").get<bool>())throw std::runtime_error("native image source/generation invalid");
   const std::string name=row.at("camera_id");if(name.size()!=8||name.substr(0,7)!="camera_"||name[7]<'0'||name[7]>'3')throw std::runtime_error("bad camera identity");int id=name[7]-'0';auto file=fs::canonical(root/row.at("rgb_path").get<std::string>());if(file.string().find(root.string()+"/")!=0)throw std::runtime_error("image path escapes archive");auto &entry=images[row.at("stamp_ns").get<uint64_t>()];if(!entry[id].empty())throw std::runtime_error("duplicate camera/source factor");entry[id]=file;}
  for(const auto &entry:images)for(const auto &path:entry.second)if(path.empty())throw std::runtime_error("incomplete exact-source four-view archive group");
 }
 bool process(LIVMapper&m){
  const uint64_t stamp=m.pending_pair.source_stamp_valid?m.pending_pair.source_stamp_ns:0;
  auto group=images.find(stamp);if(group==images.end())return false;
  const auto prior=m._state;auto &front=*m.vio_manager;auto *saved=front.state,*saved_propagated=front.state_propagat;const bool saved_only=front.measurement_only;
  struct Restore {VIOManager &v;StatesGroup *s,*p;bool only;~Restore(){v.state=s;v.state_propagat=p;v.measurement_only=only;}}restore{front,saved,saved_propagated,saved_only};
  std::array<VIOManager*,4> managers{&front,extra[0].manager.get(),extra[1].manager.get(),extra[2].manager.get()};std::array<cv::Mat,4> rgb;std::array<Factor,4> factors;Json record={{"source_stamp_ns",stamp},{"mode","native_archive_marginal_ci_v1"},{"source_skew_ns",0},{"confidence_calibrated",false},{"formal_state_commit",false},{"cameras",Json::array()}};
  for(int i=0;i<4;++i){local[i]=propagated[i]=independent_image_prior(prior);auto &manager=*managers[i];manager.state=&local[i];manager.state_propagat=&propagated[i];manager.measurement_only=true;
   rgb[i]=cv::imread(group->second[i].string(),cv::IMREAD_COLOR);if(rgb[i].rows!=512||rgb[i].cols!=512)throw std::runtime_error("bad archived native RGB dimensions");
   if(i==0 && (m.LidarMeasures.measures.back().img.size()!=rgb[i].size() || cv::norm(m.LidarMeasures.measures.back().img,rgb[i],cv::NORM_INF)!=0.)) throw std::runtime_error("same-source primary packet pixels differ from native camera_0 archive");
   manager.processFrame(rgb[i],m._pv_list,m.voxelmap_manager->voxel_map_,stamp*1e-9);
   factors[i]={manager.measurement_information,manager.measurement_gradient,manager.measurement_pixels};record["cameras"].push_back({{"camera_id",i},{"pixels",manager.measurement_pixels},{"tracked_points",manager.total_points},{"mse",manager.measurement_mse}});
  }
  bool joint=false;std::string reason;
  try {auto result=solve(prior,factors);local=result.cameras;m._state=local[0];joint=true;record["physical_increment_norm"]=result.physical_increment.norm();record["physical_covariance_trace"]=result.physical_covariance.trace();}
  catch(const std::exception&e){reason=e.what();front.measurement_only=false;front.computeJacobianAndUpdateEKF(front.new_frame_->img_);front.measurement_only=true;m._state=local[0];for(int i=1;i<4;++i){local[i]=independent_image_prior(m._state);}}
  for(int i=0;i<4;++i){managers[i]->state=&local[i];managers[i]->finishMeasurementFrame(m._pv_list);managers[i]->last_frame_ekf_updated=joint||(i==0&&front.last_frame_ekf_updated);}
  record["prior_physical_trace"]=physical_marginal(prior).trace();
  const auto physical=physical_marginal(m._state);record["posterior_physical_trace"]=physical.trace();
  record["posterior_physical_minimum_eigenvalue"]=Eigen::SelfAdjointEigenSolver<Physical>(physical).eigenvalues().minCoeff();
  record["private_inverse_exposure"]=Json::array();record["private_exposure_variance"]=Json::array();
  for(int i=0;i<4;++i){record["private_inverse_exposure"].push_back(local[i].inv_expo_time);record["private_exposure_variance"].push_back(local[i].cov(6,6));}
  front.last_frame_ekf_updated=joint||front.last_frame_ekf_updated;record["formal_state_commit"]=joint;record["fallback_reason"]=reason;record["fallback"]="single_front_native_update_when_joint_invalid";record["equal_physical_likelihood_weights"]={.25,.25,.25,.25};record["per_image_exposure_variance_model"]=.1;record["shared_prior_count"]=1;record["map_state_cross_covariance_known"]=false;
  log<<record.dump()<<'\n';log.flush();return true;
 }
};
bool process_archive_frame(LIVMapper&m){
 const char *mode=std::getenv("FASTLIVO_MULTICAMERA_MODE");if(!mode||std::string(mode)=="off")return false;
 if(std::string(mode)!="native_archive_marginal_ci_v1")throw std::runtime_error("unsupported explicit multicamera mode");
 static std::map<LIVMapper*,std::unique_ptr<Rig>> rigs;auto &rig=rigs[&m];if(!rig){const char *path=std::getenv("FASTLIVO_MULTICAMERA_ARCHIVE"),*log=std::getenv("FASTLIVO_MULTICAMERA_LOG");if(!path||!log)throw std::runtime_error("experimental native mode requires frozen archive and new log");rig=std::make_unique<Rig>(m,path,log);}return rig->process(m);
}
}
