#pragma once
// Experimental serial shadow branches. No mapper mutation or ROS publication.
#include <fast_livo2_core/core/vio.h>
#include <Eigen/Cholesky>
#include <memory>
#include <stdexcept>
#include <string>

namespace dynanav_shadow {
enum class CovarianceSource { NativeInternal, UnavailableRosPose };
struct Epoch {
    std::string generation, prior_id, map_id;
    uint64_t stamp_ns;
    StatesGroup state;
    CovarianceSource covariance_source;
};
using PhysicalCovariance=Eigen::Matrix<double,18,18>;
inline int physical_index(int i) { return i<6 ? i : i+1; }
inline PhysicalCovariance physical_marginal(const StatesGroup &s) {
    PhysicalCovariance result;
    for(int i=0;i<18;++i)for(int j=0;j<18;++j)result(i,j)=s.cov(physical_index(i),physical_index(j));
    return result;
}
// Marginalize the original front-camera exposure. This is P_xx, never the
// conditional covariance P_xx-P_xe P_ee^-1 P_ex. A new per-image camera
// exposure is an independent nuisance under the declared experimental model.
inline StatesGroup camera_prior(const StatesGroup &native,double exposure_variance) {
    if(!std::isfinite(exposure_variance) || exposure_variance<=0)throw std::invalid_argument("invalid camera exposure model variance");
    StatesGroup out=native;out.cov.setZero();
    for(int i=0;i<18;++i)for(int j=0;j<18;++j)out.cov(physical_index(i),physical_index(j))=native.cov(physical_index(i),physical_index(j));
    out.inv_expo_time=1.;out.cov(6,6)=exposure_variance;return out;
}
struct BranchOptions {
    bool estimate_per_frame_exposure=false,normal_en=false;
    int pyramid_levels=3;
    double exposure_model_variance=.1;
};
struct Result {
    StatesGroup posterior;
    Eigen::Matrix<double, 6, 1> pose_delta;
    bool retained_update;
    int tracked_points;
    PhysicalCovariance physical_covariance;
};
struct Camera {
    std::string id;
    int width = 512, height = 512;
    double fx = 512.*24./42., fy = 512.*24./42., cx = 256., cy = 256.;
    M3D R_camera_from_imu = M3D::Identity();
    V3D translation_camera_from_imu = V3D::Zero();
};

// One instance owns one camera, state, propagation prior, images and patch map.
// VIO's process-global Frame counter is non-atomic: call branches serially.
// The pointer-valued plane map must be held stable by the caller for this call;
// const on the unordered_map alone does not make pointees immutable.
class Branch {
    Camera calibration_;
    BranchOptions options_;
    vk::PinholeCamera camera_;
    StatesGroup local_, propagated_;
    VIOManager manager_;
    uint64_t last_stamp_ = 0;
    std::string generation_;
public:
    explicit Branch(Camera calibration,BranchOptions options={})
        : calibration_(std::move(calibration)),options_(options),
          camera_(calibration_.width, calibration_.height, 1., calibration_.fx,
                  calibration_.fy, calibration_.cx, calibration_.cy) {
        const auto &R = calibration_.R_camera_from_imu;
        if (calibration_.id.empty() || !R.allFinite() ||
            !(R.transpose()*R).isApprox(M3D::Identity(),1e-8) ||
            std::abs(R.determinant()-1.)>1e-8 ||
            !calibration_.translation_camera_from_imu.allFinite())
            throw std::invalid_argument("invalid fixed camera calibration");
        manager_.cam = &camera_;
        manager_.state = &local_;
        manager_.state_propagat = &propagated_;
        manager_.grid_size = 5; manager_.grid_n_height = 17;
        manager_.patch_size = 8; manager_.patch_pyrimid_level = options_.pyramid_levels;
        manager_.max_iterations = 5; manager_.img_point_cov = 100.;
        manager_.outlier_threshold = 100.; manager_.ncc_thre = .8;
        manager_.normal_en = options_.normal_en; manager_.raycast_en = false;
        manager_.inverse_composition_en = false;
        // A per-frame nuisance variable, with its explicitly modelled variance.
        // It is never inherited from the front camera or fused between views.
        manager_.exposure_estimate_en = options_.estimate_per_frame_exposure;
        manager_.plot_flag = false; manager_.has_ref_patch_cache = false;
        manager_.setImuToLidarExtrinsic(V3D::Zero(),M3D::Identity());
        std::vector<double> r(9), t(3);
        for (int i=0;i<3;++i) {
            t[i]=calibration_.translation_camera_from_imu[i];
            for (int j=0;j<3;++j) r[i*3+j]=R(i,j);
        }
        manager_.setLidarToCameraExtrinsic(r,t);
        manager_.initializeVIO();
    }
    Branch(const Branch&) = delete;
    Branch& operator=(const Branch&) = delete;
    Result run(const Epoch &epoch, uint64_t image_stamp_ns, const cv::Mat &rgb,
               const std::vector<pointWithVar> &shared_points,
               const std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> &stable_planes) {
        if (epoch.covariance_source != CovarianceSource::NativeInternal)
            throw std::invalid_argument("ROS pose placeholder covariance cannot seed a branch");
        if (epoch.generation.empty() || epoch.prior_id.empty() || epoch.map_id.empty() ||
            image_stamp_ns != epoch.stamp_ns || image_stamp_ns <= last_stamp_ ||
            (!generation_.empty() && generation_ != epoch.generation))
            throw std::invalid_argument("generation/common-prior/map/source epoch mismatch");
        if (rgb.cols != calibration_.width || rgb.rows != calibration_.height ||
            (rgb.type()!=CV_8UC1 && rgb.type()!=CV_8UC3))
            throw std::invalid_argument("expected owned calibrated camera image");
        if (!epoch.state.cov.allFinite() ||
            !(epoch.state.cov-epoch.state.cov.transpose()).isZero(1e-8) ||
            Eigen::LLT<decltype(epoch.state.cov)>(epoch.state.cov).info()!=Eigen::Success)
            throw std::invalid_argument("invalid internal full-state covariance");
        local_ = propagated_ = camera_prior(epoch.state,options_.exposure_model_variance);
        cv::Mat image = rgb.clone();
        auto points = shared_points;
        manager_.processFrame(image,points,stable_planes,epoch.stamp_ns*1e-9);
        last_stamp_=image_stamp_ns;
        generation_=epoch.generation;
        auto delta = local_ - epoch.state;
        return {local_,delta.head<6>(),manager_.last_frame_ekf_updated,manager_.total_points,physical_marginal(local_)};
    }
    const VIOManager& manager() const { return manager_; }
};
} // namespace dynanav_shadow
