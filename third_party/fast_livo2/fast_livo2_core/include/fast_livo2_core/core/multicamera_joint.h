#pragma once
// Experimental native common-prior update. Numerical covariance checks do not
// certify the inherited pixel/map uncertainty model or map/state correlation.
#include "common_lib.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <array>
#include <stdexcept>
namespace fastlivo_multicamera {
using Physical=Eigen::Matrix<double,18,18>;
using PhysicalVector=Eigen::Matrix<double,18,1>;
struct Factor {
 Eigen::Matrix<double,7,7> information=Eigen::Matrix<double,7,7>::Zero();
 Eigen::Matrix<double,7,1> gradient=Eigen::Matrix<double,7,1>::Zero();
 int pixels=0;
};
struct JointResult {std::array<StatesGroup,4> cameras;PhysicalVector physical_increment;Physical physical_covariance;};
inline int index19(int i){return i<6?i:i+1;}
inline Physical physical_marginal(const StatesGroup&s){Physical p;for(int i=0;i<18;++i)for(int j=0;j<18;++j)p(i,j)=s.cov(index19(i),index19(j));return p;}
inline StatesGroup independent_image_prior(const StatesGroup &s,double exposure_variance=.1){
 StatesGroup o=s;o.cov.setZero();o.inv_expo_time=1.;o.cov(6,6)=exposure_variance;
 for(int i=0;i<18;++i)for(int j=0;j<18;++j)o.cov(index19(i),index19(j))=s.cov(index19(i),index19(j));return o;
}
inline JointResult solve(const StatesGroup&prior,const std::array<Factor,4>&f,double exposure_variance=.1){
 if(!std::isfinite(exposure_variance)||exposure_variance<=0)throw std::invalid_argument("invalid independent exposure variance");
 Physical p=physical_marginal(prior);p=.5*(p+p.transpose()).eval();
 Eigen::LLT<Physical> chol(p);if(chol.info()!=Eigen::Success)throw std::runtime_error("native physical prior not SPD");
 Physical information=chol.solve(Physical::Identity());PhysicalVector gradient=PhysicalVector::Zero();
 std::array<double,4>d;std::array<Eigen::Matrix<double,1,18>,4>b;
 for(int c=0;c<4;++c){
  if(f[c].pixels<=0||!f[c].information.allFinite()||!f[c].gradient.allFinite())throw std::runtime_error("four same-source valid native factors required");
  auto m=(.5*(f[c].information+f[c].information.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>> es(m);
  if(es.info()!=Eigen::Success||es.eigenvalues().minCoeff() < -1e-9*std::max(1.,es.eigenvalues().maxCoeff()))throw std::runtime_error("native photometric factor not PSD");
  // Integrate each private nuisance BEFORE the fractional likelihood pool.
  // Keeping four exposure priors in a pooled joint precision would change the
  // physical confidence even for cloned observations. Each common x prior is
  // used once; equal weights sum to one rather than four.
  d[c]=1./(1./exposure_variance+m(6,6));
  const Eigen::Matrix<double,6,6> marginal=m.topLeftCorner<6,6>()-m.block<6,1>(0,6)*d[c]*m.block<1,6>(6,0);
  const Eigen::Matrix<double,6,1> g=f[c].gradient.head<6>()-m.block<6,1>(0,6)*d[c]*f[c].gradient[6];
  information.topLeftCorner<6,6>()+=.25*marginal;gradient.head<6>()+=.25*g;
  b[c].setZero();b[c].head<6>()=d[c]*m.block<1,6>(6,0);
 }
 information=.5*(information+information.transpose()).eval();Eigen::LLT<Physical> post_chol(information);
 if(post_chol.info()!=Eigen::Success)throw std::runtime_error("joint precision not SPD");
 JointResult result;result.physical_covariance=post_chol.solve(Physical::Identity());result.physical_covariance=.5*(result.physical_covariance+result.physical_covariance.transpose()).eval();result.physical_increment=-post_chol.solve(gradient);
 if(!result.physical_increment.allFinite()||result.physical_increment.head<3>().norm()>.1||result.physical_increment.segment<3>(3).norm()>.5)throw std::runtime_error("joint step exceeds experimental numerical bound");
 if(Eigen::LLT<Physical>(result.physical_covariance).info()!=Eigen::Success)throw std::runtime_error("joint physical covariance not SPD");
 Eigen::SelfAdjointEigenSolver<Physical> gain(p-result.physical_covariance);
 if(gain.info()!=Eigen::Success||gain.eigenvalues().minCoeff() < -1e-8*std::max(1.,p.norm()))throw std::runtime_error("joint covariance expands relative to shared prior");
 const Physical prior_chart_covariance=result.physical_covariance;
 const V3D w=result.physical_increment.head<3>();const double theta=w.norm();M3D hat;hat<<0.,-w.z(),w.y(),w.z(),0.,-w.x(),-w.y(),w.x(),0.;
 const M3D jr=theta<1e-7 ? (M3D::Identity()-.5*hat+hat*hat/6.).eval() : (M3D::Identity()-(1-std::cos(theta))/(theta*theta)*hat+(theta-std::sin(theta))/(theta*theta*theta)*hat*hat).eval();
 Physical transport=Physical::Identity();transport.topLeftCorner<3,3>()=jr;
 result.physical_covariance=transport*prior_chart_covariance*transport.transpose();
 for(int c=0;c<4;++c){
  auto &s=result.cameras[c];s=prior;Eigen::Matrix<double,DIM_STATE,1> delta=Eigen::Matrix<double,DIM_STATE,1>::Zero();
  for(int i=0;i<18;++i)delta[index19(i)]=result.physical_increment[i];s+=delta;
  s.inv_expo_time=1.-d[c]*(f[c].gradient[6]+(f[c].information.block<1,6>(6,0)*result.physical_increment.head<6>())(0));
  if(!std::isfinite(s.inv_expo_time)||s.inv_expo_time<.2||s.inv_expo_time>5.)throw std::runtime_error("invalid private exposure posterior");
  s.cov.setZero();const PhysicalVector cross=-transport*prior_chart_covariance*b[c].transpose();
  for(int i=0;i<18;++i){for(int j=0;j<18;++j)s.cov(index19(i),index19(j))=result.physical_covariance(i,j);s.cov(index19(i),6)=s.cov(6,index19(i))=cross[i];}
  s.cov(6,6)=d[c]+(b[c]*prior_chart_covariance*b[c].transpose())(0);
  if(Eigen::LLT<decltype(s.cov)>(s.cov).info()!=Eigen::Success)throw std::runtime_error("private exposure augmentation not SPD");
 }
 return result;
}
}
