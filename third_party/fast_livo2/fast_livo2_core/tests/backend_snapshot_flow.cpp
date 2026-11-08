#include "fast_livo2_core/backend/snapshot_bridge.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
using namespace fastlivo_backend;
namespace fs=std::filesystem;

std::string read(const fs::path& path) {
  std::ifstream file(path);return {std::istreambuf_iterator<char>(file),{}};
}
std::vector<fs::path> ready(const fs::path& directory) {
  std::vector<fs::path> result;
  for(const auto& e:fs::directory_iterator(directory))
    if(e.is_directory() && e.path().filename().string().rfind("kf_",0)==0)result.push_back(e.path());
  std::sort(result.begin(),result.end());return result;
}
BridgeOptions options(const fs::path& directory) {
  BridgeOptions o;o.spool_directory=directory;o.epoch=73;o.min_interval_ns=0;
  o.min_translation_m=0;o.min_rotation_rad=0;return o;
}
int main(int argc,char** argv) {
  const auto root=argc>1?fs::path(argv[1]):fs::temp_directory_path()/
    ("livo_snapshot_flow_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(root);
  {
    auto o=options(root/"paired");o.max_points=5;
    SnapshotBridge bridge(o);
    auto lio=identity_transform(),livo=lio,extrinsic=lio;lio[3]=2;livo[3]=4;extrinsic[7]=0.2;
    std::vector<PointXYZ> cloud(12);
    for(size_t i=0;i<cloud.size();++i)cloud[i]={float(i),float(i+1),float(i+2)};
    assert(bridge.on_lio(1'000'000'000,lio,extrinsic,true,cloud.size(),[&](size_t i){return cloud[i];}));
    // A duplicate LIO must not flush the pending successful pair.
    assert(!bridge.on_lio(1'000'000'000,lio,extrinsic,true,cloud.size(),[&](size_t i){return cloud[i];}));
    assert(!bridge.on_vio(999'999'999,true,livo));  // never bind a near-but-different stamp
    for(auto& p:cloud)p={999,999,999};
    assert(bridge.on_vio(1'000'000'000,true,livo));
    assert(!bridge.on_vio(1'000'000'000,true,livo));
    bridge.close();const auto stats=bridge.stats();
    assert(stats.submitted==1 && stats.written==1 && stats.duplicates==1 && stats.unmatched_vio==2);
    const auto paths=ready(o.spool_directory);assert(paths.size()==1);
    const auto metadata=read(paths[0]/"metadata.json"),pcd=read(paths[0]/"points_lidar.pcd");
    assert(metadata.find("\"final_stage\":\"livo\"")!=std::string::npos);
    assert(metadata.find("\"T_odom_body\":[[1,0,0,4]")!=std::string::npos);
    assert(metadata.find("\"point_count\":5")!=std::string::npos);
    assert(pcd.find("999")==std::string::npos && pcd.find("0 1 2\n")!=std::string::npos);
    assert(pcd.find("9 10 11\n")!=std::string::npos);
  }
  {
    auto o=options(root/"skipped");o.max_points=2;
    SnapshotBridge bridge(o);auto lio=identity_transform(),bad=lio;lio[3]=3;bad[3]=99;
    auto reader=[](size_t i){return PointXYZ{float(i),1,2};};
    assert(bridge.on_lio(2'000'000'000,lio,lio,true,2,reader));
    assert(bridge.on_vio(2'000'000'000,false,bad));
    assert(bridge.on_lio(3'000'000'000,lio,lio,true,2,reader));
    // Missing VIO is flushed once on the next successful LIO.
    assert(bridge.on_lio(4'000'000'000,lio,lio,false,2,reader));
    bridge.close();assert(bridge.stats().written==3);
    const auto paths=ready(o.spool_directory);assert(paths.size()==3);
    for(size_t i=0;i<2;++i) {
      const auto metadata=read(paths[i]/"metadata.json");
      assert(metadata.find("\"final_stage\":\"lio_vio_skipped\"")!=std::string::npos);
      assert(metadata.find("\"T_odom_body\":[[1,0,0,3]")!=std::string::npos);
    }
  }
  {
    auto o=options(root/"stalled_consumer");o.max_ready_directories=1;
    auto pose=identity_transform();auto reader=[](size_t){return PointXYZ{1,2,3};};
    { SnapshotBridge first(o);
      assert(first.on_lio(1,pose,pose,false,1,reader));first.close();
      assert(first.stats().written==1);
    }
    // A real unread keyframe survives producer restart. New work is dropped,
    // not appended without a bound and not allowed to block the estimator.
    o.epoch=74;
    { SnapshotBridge second(o);
      assert(second.on_lio(2,pose,pose,false,1,reader));
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
      bool live_overload_visible=false;
      while(std::chrono::steady_clock::now()<deadline) {
        const auto live=read(o.spool_directory/"bridge_status.json");
        if(live.find("\"spool_dropped\":1")!=std::string::npos &&
           live.find("\"owner_stopped\":false")!=std::string::npos) {
          live_overload_visible=true;break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      assert(live_overload_visible);second.close();
      assert(second.stats().spool_dropped==1 && second.stats().written==0);
      const auto status=read(o.spool_directory/"bridge_status.json");
      assert(status.find("\"spool_dropped\":1")!=std::string::npos);
      assert(status.find("\"owner_stopped\":true")!=std::string::npos);
      assert(status.find("\"last_measurement_ns\":2")!=std::string::npos);
    }
    assert(ready(o.spool_directory).size()==1);
  }
  {
    auto o=options(root/"selection");o.min_interval_ns=10;o.min_translation_m=0.8;o.min_rotation_rad=0.2;
    SnapshotBridge bridge(o);auto lio=identity_transform(),livo=lio;livo[3]=4;
    size_t reads=0;auto reader=[&](size_t){++reads;return PointXYZ{1,2,3};};
    assert(bridge.on_lio(10,lio,lio,true,1,reader));
    assert(bridge.on_vio(10,true,livo));
    assert(!bridge.on_lio(15,livo,lio,false,1,reader));  // time filter before copy
    assert(!bridge.on_lio(20,livo,lio,false,1,reader));  // final VIO pose is KF reference
    livo[3]=5;
    assert(bridge.on_lio(30,livo,lio,false,1,reader));
    bridge.close();assert(reads==2 && bridge.stats().written==2);
  }
  {
    auto o=options(root/"stationary_confirmation");o.min_interval_ns=1000000000;o.max_interval_ns=2000000000;o.min_translation_m=.8;o.min_rotation_rad=.2;
    SnapshotBridge bridge(o);auto pose=identity_transform();size_t copies=0;
    auto reader=[&](size_t){++copies;return PointXYZ{1,2,3};};
    assert(bridge.on_lio(1000000000,pose,pose,false,1,reader));
    assert(!bridge.on_lio(2000000000,pose,pose,false,1,reader));
    assert(bridge.on_lio(3000000000,pose,pose,false,1,reader));
    bridge.close();assert(copies==2 && bridge.stats().written==2);
  }
  std::vector<uint64_t> submit_ns;
  BridgeStats pressure;
  {
    auto o=options(root/"pressure");o.queue_capacity=1;o.max_points=4096;o.max_ready_directories=1;
    SnapshotBridge bridge(o);auto pose=identity_transform();
    std::vector<PointXYZ> frontend_cloud(200000);
    for(size_t i=0;i<frontend_cloud.size();++i)
      frontend_cloud[i]={float(i),float(i%17),float(i%29)};
    // Real 200k-point source is never traversed: only selected capped samples.
    for(uint64_t i=1;i<=10000;++i) {
      pose[3]=double(i);const auto begin=std::chrono::steady_clock::now();
      bridge.on_lio(i,pose,identity_transform(),false,frontend_cloud.size(),
        [&](size_t j) noexcept {return frontend_cloud[j];});
      submit_ns.push_back(uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-begin).count()));
    }
    bridge.close();pressure=bridge.stats();
    assert(pressure.candidates==10000 && pressure.queue_dropped>0);
    assert(pressure.written==1 && ready(o.spool_directory).size()==1);
    assert(pressure.copied_points==4096*(pressure.submitted));
    assert(pressure.spool_dropped+pressure.written==pressure.submitted && pressure.io_failed==0);
    const auto status=read(o.spool_directory/"bridge_status.json");
    assert(status.find("\"queue_dropped\":"+std::to_string(pressure.queue_dropped))!=std::string::npos);
    assert(status.find("\"status_failed\":0")!=std::string::npos);
  }
  {
    auto o=options(root/"io_failure");SnapshotBridge bridge(o);auto pose=identity_transform();
    // Break only this test's spool after startup. Errors stay on the worker.
    fs::rename(o.spool_directory,o.spool_directory.string()+"_detached");
    std::ofstream(o.spool_directory)<<"intentional non-directory fault injection";
    assert(bridge.on_lio(1,pose,pose,false,1,[](size_t){return PointXYZ{1,2,3};}));
    bridge.close();assert(bridge.stats().io_failed==1);
  }
  std::sort(submit_ns.begin(),submit_ns.end());
  std::cout<<"{\"result\":\"PASS\",\"scenarios\":[\"paired_owned_cloud\",\"duplicate_no_double_count\","
    <<"\"skipped_and_missing_vio\",\"bounded_nonblocking_pressure\",\"bounded_spool\",\"live_overload_status\",\"candidate_before_copy\",\"worker_io_failure\"],"
    <<"\"calls\":"<<submit_ns.size()<<",\"submit_p50_ns\":"<<submit_ns[submit_ns.size()/2]
    <<",\"submit_p99_ns\":"<<submit_ns[submit_ns.size()*99/100]<<",\"submit_max_ns\":"<<submit_ns.back()
    <<",\"queue_dropped\":"<<pressure.queue_dropped<<",\"spool_dropped\":"<<pressure.spool_dropped
    <<",\"max_selected_copy_ns\":"<<pressure.max_copy_ns<<",\"fixture_root\":\""<<root.string()<<"\"}\n";
}
