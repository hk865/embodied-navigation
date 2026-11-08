#include "preprocess.h"
#include "identity_ros.h"
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <set>
#include <filesystem>
template<class T>T read_cdr(const std::filesystem::path&p){
  std::ifstream f(p,std::ios::binary);std::string data((std::istreambuf_iterator<char>(f)),{});
  if(!f||data.empty())throw std::runtime_error("missing CDR fixture");
  rclcpp::SerializedMessage raw(data.size());auto&cdr=raw.get_rcl_serialized_message();
  std::copy(data.begin(),data.end(),cdr.buffer);cdr.buffer_length=data.size();
  rclcpp::Serialization<T>codec;T msg;codec.deserialize_message(&raw,&msg);return msg;
}
int main(int argc,char**argv){try{
  if(argc!=2)throw std::runtime_error("fixture directory required");
  const std::filesystem::path dir(argv[1]);nlohmann::json out;
  for(auto name:{"raw","adapted","registered"}){
    auto msg=read_cdr<sensor_msgs::msg::PointCloud2>(dir/(std::string(name)+".cdr"));
    out[std::string(name)+"_message_sha256"]=fastlivo_identity::message_sha(msg);
  }
  for(auto name:{"mapper","body"}){
    auto msg=read_cdr<nav_msgs::msg::Odometry>(dir/(std::string(name)+".cdr"));
    out[std::string(name)+"_message_sha256"]=fastlivo_identity::message_sha(msg);
  }
  auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>(read_cdr<sensor_msgs::msg::PointCloud2>(dir/"adapted.cdr"));
  Preprocess pre;pre.set(false,0,.3,1);PointCloudXYZI::Ptr cloud(new PointCloudXYZI());pre.process(msg,cloud);
  auto source=std::make_shared<fastlivo_identity::Source>();source->packet=7;source->stamp_ns=21155000000ULL;
  source->input_count=msg->width*msg->height;source->adapted_message_sha256=fastlivo_identity::message_sha(*msg);
  const auto refs=fastlivo_identity::selected(source,pre.identity_input_ordinals,cloud->size());
  out["retained_points"]=cloud->size();out["input_ordinals_base64"]=fastlivo_identity::base64(std::string(
      reinterpret_cast<const char*>(pre.identity_input_ordinals.data()),pre.identity_input_ordinals.size()*4));
  if(refs.size()!=29004)throw std::runtime_error("original fixture filtering differs");
  if(refs[16615].input_ordinal!=16615||refs[17095].input_ordinal!=17095)
    throw std::runtime_error("known duplicate point provenance differs");
  const auto&a=cloud->points[16615];const auto&b=cloud->points[17095];
  if(a.x!=b.x||a.y!=b.y||a.z!=b.z)throw std::runtime_error("expected duplicate XYZ absent");
  // True IDs remain distinct even if downstream geometry is equal or reorders.
  std::vector<fastlivo_identity::Ref>cur,next;
  fastlivo_identity::append_ref(cur,next,refs[16615],true);
  fastlivo_identity::append_ref(cur,next,refs[17095],false);
  if(cur[0].input_ordinal==next[0].input_ordinal)throw std::runtime_error("duplicate identity");
  auto other=std::make_shared<fastlivo_identity::Source>(*source);other->packet=8;
  fastlivo_identity::carry(cur,next);fastlivo_identity::append_ref(cur,next,{other,16615},true);
  if(cur.size()!=2||cur[0].source->packet!=7||cur[1].source->packet!=8||!next.empty())
    throw std::runtime_error("carry/merge source identity changed");
  std::reverse(cur.begin(),cur.end());fastlivo_identity::validate(cur,2);
  const auto binary=fastlivo_identity::binary(cur);
  if(binary.size()!=24||uint8_t(binary[0])!=8||uint8_t(binary[12])!=7)
    throw std::runtime_error("LE encoding or real order changed");
  bool cardinality=false,missing=false,outside=false;
  try{fastlivo_identity::validate(cur,1);}catch(...){cardinality=true;}
  try{fastlivo_identity::validate({{nullptr,0}},1);}catch(...){missing=true;}
  try{fastlivo_identity::validate({{source,source->input_count}},1);}catch(...){outside=true;}
  if(!cardinality||!missing||!outside)throw std::runtime_error("negative gate failed");
  out["source_identity_checks"]=true;out["navigation_PASS"]=false;
  out["representation"]="complete received message CDR reserialization, not original transport wire";
  std::cout<<out.dump()<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
