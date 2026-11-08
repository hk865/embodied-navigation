#pragma once
#include "point_identity.h"
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
#include <iomanip>
#include <array>
#include <random>
#include <cstring>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
namespace fastlivo_identity {
inline std::string hash(const unsigned char*data,size_t n) {
  std::array<unsigned char,SHA256_DIGEST_LENGTH>digest{};SHA256(data,n,digest.data());
  std::ostringstream out;for(auto v:digest)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(v);return out.str();
}
template<class Msg>std::string cdr_sha(const Msg&msg) {
  rclcpp::Serialization<Msg>serializer;rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&msg,&serialized);const auto&raw=serialized.get_rcl_serialized_message();
  return hash(raw.buffer,raw.buffer_length);
}
// Every semantic ROS field and each data byte; CDR alignment padding is excluded.
// Explicit LE encoding makes native/Python fingerprints independent of serializers.
inline void u32(std::string&s,uint32_t v){for(unsigned j=0;j<4;++j)s.push_back(char((v>>(8*j))&255));}
inline void u64(std::string&s,uint64_t v){for(unsigned j=0;j<8;++j)s.push_back(char((v>>(8*j))&255));}
inline void str(std::string&s,const std::string&v){if(v.size()>UINT32_MAX)throw std::overflow_error("identity string");u32(s,uint32_t(v.size()));s+=v;}
inline void f64(std::string&s,double v){static_assert(sizeof(double)==8,"double size");uint64_t bits;std::memcpy(&bits,&v,8);u64(s,bits);}
inline void header(std::string&s,const std_msgs::msg::Header&h){u32(s,uint32_t(h.stamp.sec));u32(s,h.stamp.nanosec);str(s,h.frame_id);}
inline std::string message_sha(const sensor_msgs::msg::PointCloud2&m){
  std::string s="go2_ros_semantic/v1:PointCloud2";s.push_back('\0');header(s,m.header);
  u32(s,m.height);u32(s,m.width);u32(s,uint32_t(m.fields.size()));
  for(const auto&f:m.fields){str(s,f.name);u32(s,f.offset);s.push_back(char(f.datatype));u32(s,f.count);}
  s.push_back(char(m.is_bigendian));u32(s,m.point_step);u32(s,m.row_step);u64(s,m.data.size());
  s.append(reinterpret_cast<const char*>(m.data.data()),m.data.size());s.push_back(char(m.is_dense));
  return hash(reinterpret_cast<const unsigned char*>(s.data()),s.size());
}
inline std::string message_sha(const nav_msgs::msg::Odometry&m){
  std::string s="go2_ros_semantic/v1:Odometry";s.push_back('\0');header(s,m.header);str(s,m.child_frame_id);
  const auto&p=m.pose.pose.position;const auto&q=m.pose.pose.orientation;
  for(double v:{p.x,p.y,p.z,q.x,q.y,q.z,q.w})f64(s,v);for(auto v:m.pose.covariance)f64(s,v);
  const auto&l=m.twist.twist.linear;const auto&a=m.twist.twist.angular;
  for(double v:{l.x,l.y,l.z,a.x,a.y,a.z})f64(s,v);for(auto v:m.twist.covariance)f64(s,v);
  return hash(reinterpret_cast<const unsigned char*>(s.data()),s.size());
}
inline std::string base64(const std::string&data) {
  if(data.size()>size_t(std::numeric_limits<int>::max()))throw std::overflow_error("identity base64 size");
  std::string out(4*((data.size()+2)/3)+1,'\0');
  int n=EVP_EncodeBlock(reinterpret_cast<unsigned char*>(&out[0]),
      reinterpret_cast<const unsigned char*>(data.data()),int(data.size()));out.resize(n);return out;
}
// This type can only be produced by the original EVP encoder. Its text is
// ASCII base64, so injecting it between existing JSON quotes needs no escaping.
class EncodedPointRefs {
 public:
  const std::string& text() const noexcept {return text_;}
 private:
  explicit EncodedPointRefs(std::string text):text_(std::move(text)){}
  std::string text_;
  friend EncodedPointRefs encode_point_refs(const std::string&);
};
inline EncodedPointRefs encode_point_refs(const std::string& bytes) {
  return EncodedPointRefs(base64(bytes));
}
inline std::string dump_point_commit(const nlohmann::json& metadata,const EncodedPointRefs& encoded) {
  const auto it=metadata.find("point_refs_base64");
  if(!metadata.is_object() || it==metadata.end() || !it->is_string() || !it->get_ref<const std::string&>().empty())
    throw std::logic_error("point commit requires empty root refs placeholder");
  // Same original default strict dump happens before the final message cap.
  auto out=metadata.dump();
  const std::string marker="\"point_refs_base64\":\"\"";
  const auto pos=out.find(marker);
  if(pos==std::string::npos || out.find(marker,pos+marker.size())!=std::string::npos)
    throw std::logic_error("point commit requires unique structural refs marker");
  out.insert(pos+marker.size()-1,encoded.text());
  return out;
}
inline std::string epoch(){std::random_device rd;std::ostringstream out;
  for(unsigned i=0;i<4;++i)out<<std::hex<<std::setw(8)<<std::setfill('0')<<uint32_t(rd());return out.str();}
}
