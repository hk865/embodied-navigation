// Executes the production GridMap::raycastProcess, not a reimplemented mapper.
#include <plan_env/grid_map.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>

class GridMapTestPeer {
public:
  static void configure(GridMap &g, double resolution=1., double max_range=5.) {
    g.mp_=MappingParameters{};g.md_=MappingData{};
    auto &p=g.mp_;auto &d=g.md_;
    p.resolution_=resolution;p.resolution_inv_=1./resolution;
    p.map_voxel_num_=Eigen::Vector3i::Constant(static_cast<int>(std::ceil(10./resolution)));
    p.map_origin_idx_=Eigen::Vector3i::Zero();g.updateMapBoundaryFromIndex();
    p.local_update_range_=Eigen::Vector3d::Constant(5.);p.max_ray_length_=max_range;
    p.map_sliding_en_=false;p.map_sliding_thresh_vox_=1;p.ground_height_=0.;
    p.prob_hit_log_=logit(.85);p.prob_miss_log_=logit(.30);
    p.clamp_min_log_=logit(.12);p.clamp_max_log_=logit(.98);
    p.min_occupancy_log_=logit(.80);p.unknown_flag_=.01;
    p.double_cylinder_radius_=.25;p.double_cylinder_offset_=.18;
    p.obstacles_inflation_z_up=.12;p.obstacles_inflation_z_down=.12;
    int size=p.map_voxel_num_.prod();
    d.occupancy_buffer_.assign(size,p.clamp_min_log_-.01);
    d.occupancy_buffer_inflate_.assign(size,0);d.occupancy_buffer_inflate_cnt_.assign(size,0);
    d.count_hit_.assign(size,0);d.count_hit_and_miss_.assign(size,0);
    d.flag_rayend_.assign(size,-1);d.flag_traverse_.assign(size,-1);d.raycast_num_=0;
    d.ray_q_=Eigen::Quaterniond::Identity();g.rebuildInflationOffsets();
  }
  static void frame(GridMap &g,const Eigen::Vector3d &origin,
                    const std::vector<Eigen::Vector3d> &points) {
    g.md_.ray_pos_=origin;g.md_.proj_points_=points;
    g.md_.proj_points_cnt=static_cast<int>(points.size());g.raycastProcess();
  }
  static double odds(GridMap &g,const Eigen::Vector3d &p) {
    Eigen::Vector3i id;g.posToIndex(p,id);return g.md_.occupancy_buffer_[g.toAddress(id)];
  }
  static void slide(GridMap &g,const Eigen::Vector3d &p) {g.mp_.map_sliding_en_=true;g.updateSlidingMap(p);}
};

int main(int argc,char **argv) {
  struct Check {std::string name;bool passed;};std::vector<Check> checks;
  auto check=[&](const char *name,bool passed){checks.push_back({name,passed});};
  const Eigen::Vector3d sensor(.5,.5,.5),hit(3.1,1.1,.5),other(3.9,1.9,.5),ghost(2.5,1.5,.5);
  {GridMap g;GridMapTestPeer::configure(g);const Eigen::Vector3d divergent_ghost(2.5,.5,.5);g.setOccupied(divergent_ghost);
    for(int i=0;i<4;i++)GridMapTestPeer::frame(g,{.1,.1,.1},{{3.05,1.95,.05},{3.95,1.05,.95}});
    check("shared endpoint rays clear their different observed paths",g.getOccupancy(divergent_ghost)==0);}
  {GridMap a,b;GridMapTestPeer::configure(a);GridMapTestPeer::configure(b);
    GridMapTestPeer::frame(a,sensor,{hit});GridMapTestPeer::frame(b,sensor,std::vector<Eigen::Vector3d>(1000,hit));
    check("duplicate rays have one-frame evidence weight",std::abs(GridMapTestPeer::odds(a,hit)-GridMapTestPeer::odds(b,hit))<1e-12);}
  {GridMap a,b;GridMapTestPeer::configure(a);GridMapTestPeer::configure(b);const Eigen::Vector3d free_cell(1.5,.5,.5);
    a.setOccupied(free_cell);b.setOccupied(free_cell);double before=GridMapTestPeer::odds(a,free_cell);
    GridMapTestPeer::frame(a,sensor,{{3.5,.5,.5}});GridMapTestPeer::frame(b,sensor,std::vector<Eigen::Vector3d>(1000,{3.5,.5,.5}));
    check("duplicate free rays contribute exactly one miss per frame",std::abs(GridMapTestPeer::odds(a,free_cell)-GridMapTestPeer::odds(b,free_cell))<1e-12 && std::abs(GridMapTestPeer::odds(a,free_cell)-before-logit(.30))<1e-12);}
  {GridMap a,b;GridMapTestPeer::configure(a);GridMapTestPeer::configure(b);
    const Eigen::Vector3d wall(1.04,.04,.04),origin(.04,.04,.04),behind(2.04,.12,.04);
    auto points=std::vector<Eigen::Vector3d>(1000,behind);points.push_back(wall);
    GridMapTestPeer::frame(a,origin,points);std::reverse(points.begin(),points.end());GridMapTestPeer::frame(b,origin,points);
    check("real hit wins over coarse-voxel free crossings in any order",std::abs(GridMapTestPeer::odds(a,wall)-GridMapTestPeer::odds(b,wall))<1e-12 && GridMapTestPeer::odds(a,wall)>logit(.12));}
  {GridMap g;GridMapTestPeer::configure(g);Eigen::Vector3d wall(1.5,.5,.5),hidden(2.5,.5,.5),unseen(.5,3.5,.5);
    g.setOccupied(hidden);g.setOccupied(unseen);double before=GridMapTestPeer::odds(g,hidden);
    for(int i=0;i<8;i++)GridMapTestPeer::frame(g,sensor,{wall});
    check("occluded and unobserved old surfaces are retained",GridMapTestPeer::odds(g,hidden)==before && g.getOccupancy(unseen)==1);
    check("space behind first measured surface stays unknown",g.isUnknown(Eigen::Vector3d(3.5,.5,.5)));}
  {GridMap g;GridMapTestPeer::configure(g,1.,1.5);GridMapTestPeer::frame(g,sensor,{{4.,.5,.5}});
    check("range clip touching next voxel does not clear it",g.isUnknown(Eigen::Vector3d(2.5,.5,.5)));
    check("range clip clears traversed space without endpoint hit",g.isKnownFree(Eigen::Vector3i(1,0,0)));}
  {GridMap g;GridMapTestPeer::configure(g);
    for(int i=0;i<320;i++)GridMapTestPeer::frame(g,sensor,{{1.5,.5,.5}});
    g.setOccupied(ghost);for(int i=0;i<4;i++)GridMapTestPeer::frame(g,sensor,{hit,other});
    check("more than 300 frames cannot reuse stale scratch epochs",g.getOccupancy(ghost)==0);}
  {GridMap g;GridMapTestPeer::configure(g);const Eigen::Vector3d revisit(2.5,.5,.5);
    for(int i=0;i<254;i++)GridMapTestPeer::frame(g,sensor,{{.5,2.5,.5}});
    g.setOccupied(revisit);double before=GridMapTestPeer::odds(g,revisit);
    GridMapTestPeer::frame(g,sensor,{{3.5,.5,.5}});
    check("frame 255 applies exactly one fresh miss despite initial scratch flags",std::abs(GridMapTestPeer::odds(g,revisit)-before-logit(.30))<1e-12);}
  {GridMap g;GridMapTestPeer::configure(g);g.setOccupied(Eigen::Vector3d(-3.5,.5,.5));
    GridMapTestPeer::slide(g,{6.5,.5,.5});
    check("sliding ring reuse clears old occupancy and inflation",g.isUnknown(Eigen::Vector3d(6.5,.5,.5)) && g.getInflateOccupancy({6.5,.5,.5},0.)==0);
    for(int i=0;i<3;i++)GridMapTestPeer::frame(g,{6.5,.5,.5},{{7.5,.5,.5}});
    check("sliding map continues to accept actual surface hits",g.getOccupancy(Eigen::Vector3d(7.5,.5,.5))==1);}
  {GridMap g;GridMapTestPeer::configure(g);GridMapTestPeer::frame(g,{-1.,0.,0.},{{-1.,0.,0.},{-2.2,0.,0.},{-1.,2.,0.}});
    check("zero length axes and negative boundary rays terminate",true);}
  int seed_count=0,remaining=0;
  if(argc>1) {
    std::ifstream input(argv[1]);size_t seeds,points_count;Eigen::Vector3d origin;
    input>>seeds>>points_count>>origin.x()>>origin.y()>>origin.z();
    std::vector<Eigen::Vector3d> old(seeds),points(points_count);
    for(auto &p:old)input>>p.x()>>p.y()>>p.z();for(auto &p:points)input>>p.x()>>p.y()>>p.z();
    if(!input)throw std::runtime_error("invalid measured snapshot fixture");
    GridMap g;GridMapTestPeer::configure(g,.08);for(const auto &p:old)g.setOccupied(p);
    for(int i=0;i<6;i++)GridMapTestPeer::frame(g,origin,points);
    for(const auto &p:old)remaining+=g.getOccupancy(p)==1;seed_count=static_cast<int>(seeds);
    check("observed run8 stale blocking cells clear with repeated actual rays",remaining==0);
  }
  bool passed=true;std::cout<<"{\"checks\":[";
  for(size_t i=0;i<checks.size();i++){if(i)std::cout<<",";std::cout<<"{\"name\":\""<<checks[i].name<<"\",\"passed\":"<<(checks[i].passed?"true":"false")<<"}";passed&=checks[i].passed;}
  std::cout<<"],\"seed_cells\":"<<seed_count<<",\"remaining_cells\":"<<remaining<<",\"passed\":"<<(passed?"true":"false")<<"}\n";
  return passed?0:1;
}
