#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {
using Pose=Eigen::Isometry3d;
using V6=Eigen::Matrix<double,6,1>;
using J6=Eigen::Matrix<double,6,6>;
using Row4=Eigen::Matrix<double,4,4,Eigen::RowMajor>;
Eigen::Matrix3d skew(const Eigen::Vector3d&v){Eigen::Matrix3d m;m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return m;}
Pose exponential(const V6&x){Pose t=Pose::Identity();const Eigen::Vector3d w=x.tail<3>();const double a=w.norm();const auto W=skew(w);Eigen::Matrix3d V;
  if(a<1e-8){t.linear()=Eigen::Matrix3d::Identity()+W+.5*W*W;V=Eigen::Matrix3d::Identity()+.5*W+W*W/6.;}
  else{t.linear()=Eigen::AngleAxisd(a,w/a).toRotationMatrix();V=Eigen::Matrix3d::Identity()+(1-std::cos(a))/(a*a)*W+(a-std::sin(a))/(a*a*a)*W*W;}
  t.translation()=V*x.head<3>();return t;
}
V6 logarithm(const Pose&t){const Eigen::AngleAxisd aa(t.linear());const Eigen::Vector3d w=aa.axis()*aa.angle();const auto W=skew(w);const double a=w.norm();
  Eigen::Matrix3d V=Eigen::Matrix3d::Identity()-.5*W;
  V+=(a<1e-6?1./12.:(1-.5*a/std::tan(.5*a))/(a*a))*W*W;
  V6 out;out.head<3>()=V*t.translation();out.tail<3>()=w;return out;
}
struct Edge{int a,b;Pose measurement;double translation_sigma,rotation_sigma;bool loop;};
V6 residual(const Pose&a,const Pose&b,const Edge&e){V6 r=logarithm(e.measurement.inverse()*a.inverse()*b);r.head<3>()/=e.translation_sigma;r.tail<3>()/=e.rotation_sigma;return r;}
std::pair<double,double> costs(const std::vector<Pose>&p,const std::vector<Edge>&edges){double robust=0,raw=0;
  for(const auto&e:edges){const double n=residual(p[e.a],p[e.b],e).norm();raw+=n*n;robust+=e.loop&&n>2?4*n-4:n*n;}return{robust,raw};
}
}

// Small C ABI consumed only by the asynchronous Python backend. No ROS,
// frontend state, point clouds, file I/O or ownership crosses this boundary.
extern "C" int livo_optimize_graph(int count,double*pose_data,int edge_count,
    const int64_t*endpoints,const double*measurements,const double*sigmas,
    const uint8_t*loops,int window,int iterations,double*report) noexcept {
  const auto began=std::chrono::steady_clock::now();
  try{
    if(count<2||count>2000||edge_count<1||edge_count>6000||window<1||window>80||iterations<1||iterations>8)return -1;
    std::vector<Pose> poses(count);std::vector<Edge> edges;edges.reserve(edge_count);
    for(int i=0;i<count;++i){Eigen::Map<const Row4> m(pose_data+i*16);if(!m.allFinite())return -1;poses[i].matrix()=m;}
    for(int i=0;i<edge_count;++i){int64_t a=endpoints[i*2],b=endpoints[i*2+1];Eigen::Map<const Row4> m(measurements+i*16);
      if(a<0||a>=count||b<=a||b>=count||!m.allFinite()||!std::isfinite(sigmas[i*2])||!std::isfinite(sigmas[i*2+1])||sigmas[i*2]<=0||sigmas[i*2+1]<=0)return -1;
      Pose measurement=Pose::Identity();measurement.matrix()=m;edges.push_back({int(a),int(b),measurement,sigmas[i*2],sigmas[i*2+1],bool(loops[i])});}
    const int first=std::max(1,count-window),dimension=(count-first)*6;
    const auto initial=costs(poses,edges);auto previous=initial;bool changed=false,converged=false;int completed=0;
    for(int iteration=0;iteration<iterations;++iteration){std::vector<Eigen::Triplet<double>> entries;entries.reserve(size_t(window)*180);
      Eigen::VectorXd rhs=Eigen::VectorXd::Zero(dimension);
      for(const auto&e:edges){if(e.a<first&&e.b<first)continue;const V6 r=residual(poses[e.a],poses[e.b],e);const double norm=r.norm();const double weight=e.loop&&norm>2?2/norm:1.;
        const int ids[2]={e.a,e.b};J6 jacobian[2];
        for(int k=0;k<2;++k)for(int d=0;d<6;++d){V6 v=V6::Zero();v[d]=1e-6;Pose a=poses[e.a],b=poses[e.b];if(k)b=b*exponential(v);else a=a*exponential(v);jacobian[k].col(d)=(residual(a,b,e)-r)/1e-6;}
        for(int k=0;k<2;++k)if(ids[k]>=first){const int row=(ids[k]-first)*6;rhs.segment<6>(row)-=weight*jacobian[k].transpose()*r;
          for(int l=0;l<2;++l)if(ids[l]>=first){const int column=(ids[l]-first)*6;const J6 h=weight*jacobian[k].transpose()*jacobian[l];for(int i=0;i<6;++i)for(int j=0;j<6;++j)entries.emplace_back(row+i,column+j,h(i,j));}}
      }
      for(int i=0;i<dimension;++i)entries.emplace_back(i,i,1e-7);
      Eigen::SparseMatrix<double> H(dimension,dimension);H.setFromTriplets(entries.begin(),entries.end());Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;solver.compute(H);
      if(solver.info()!=Eigen::Success)return -2;
      const Eigen::VectorXd dx=solver.solve(rhs);
      if(!dx.allFinite())return -2;
      bool accepted=false;double alpha=1.;
      for(int line=0;line<5;++line,alpha*=.5){auto candidate=poses;
        for(int i=first;i<count;++i){V6 delta=alpha*dx.segment<6>((i-first)*6);const double scale=std::min({1.,.25/std::max(1e-12,delta.head<3>().norm()),.15/std::max(1e-12,delta.tail<3>().norm())});candidate[i]=poses[i]*exponential(delta*scale);}
        const auto next=costs(candidate,edges);
        if(std::isfinite(next.first)&&std::isfinite(next.second)&&next.first<=previous.first+1e-9&&next.second<=previous.second+1e-9){
          converged=dx.norm()*alpha<1e-6||std::abs(previous.first-next.first)<1e-9;changed=changed||next.first<previous.first-1e-8;poses.swap(candidate);previous=next;accepted=true;break;}
      }
      ++completed;if(!accepted||converged)break;
    }
    report[0]=initial.second;report[1]=previous.second;report[2]=completed;report[3]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();report[4]=converged?1.:0.;
    if(!changed)return 1;
    for(int i=0;i<count;++i){Eigen::Map<Row4> m(pose_data+i*16);m=poses[i].matrix();}
    return 0;
  }catch(...){return -3;}
}
