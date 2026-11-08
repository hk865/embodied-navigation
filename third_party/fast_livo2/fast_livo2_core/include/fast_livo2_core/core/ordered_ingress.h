#pragma once
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace fastlivo_ingress {
// Slots stay charged through raw/decoding/ready/owner until acknowledged.
// T is a retained immutable-input / single-decoder-owned packet reference.
template<class T> class OrderedQueue {
 public:
  enum class Stage {Raw, Decoding, Ready, Owner};
  struct Stats {uint64_t accepted=0,delivered=0,committed=0,canceled=0,rejected=0,closed_rejections=0;
    size_t pending=0,bytes=0,peak_pending=0,peak_bytes=0,inflight=0,ready=0,raw=0;
    bool closed=false;std::string failure;};
  struct Remaining {uint64_t sequence;int kind;Stage stage;T packet;};
  OrderedQueue(size_t count_limit,size_t byte_limit):count_limit_(count_limit),byte_limit_(byte_limit){if(!count_limit||!byte_limit)throw std::invalid_argument("zero ingress bound");}
  uint64_t admit(T packet,size_t bytes,int kind) {
    std::lock_guard<std::mutex>lock(mutex_);
    if(stats_.closed){++stats_.closed_rejections;return 0;}
    if(!stats_.failure.empty()){++stats_.rejected;return 0;}
    if(slots_.size()>=count_limit_||bytes>byte_limit_-stats_.bytes){++stats_.rejected;stats_.failure="total ingress capacity exceeded";cv_.notify_all();return 0;}
    const uint64_t seq=++stats_.accepted;packet->sequence=seq;slots_.emplace(seq,Slot{std::move(packet),bytes,kind,kind>=2?Stage::Raw:Stage::Ready});
    stats_.bytes+=bytes;update();cv_.notify_all();return seq;
  }
  bool take(uint64_t seq,T& packet){std::lock_guard<std::mutex>lock(mutex_);auto it=slots_.find(seq);if(it==slots_.end()||it->second.stage!=Stage::Raw)return false;it->second.stage=Stage::Decoding;packet=it->second.packet;update();return true;}
  bool wait_take(int kind,T& packet){std::unique_lock<std::mutex>lock(mutex_);cv_.wait(lock,[&]{return !stats_.failure.empty()||stats_.closed||has_raw(kind);});
    if(!stats_.failure.empty())return false;
    for(auto& row:slots_)if(row.second.kind==kind&&row.second.stage==Stage::Raw){row.second.stage=Stage::Decoding;packet=row.second.packet;update();return true;}
    return false;
  }
  void ready(uint64_t seq){std::lock_guard<std::mutex>lock(mutex_);auto it=slots_.find(seq);if(it==slots_.end()||it->second.stage!=Stage::Decoding)throw std::logic_error("decode completion identity differs");it->second.stage=Stage::Ready;update();cv_.notify_all();}
  bool pop(T& packet,Stats* snapshot=nullptr){std::lock_guard<std::mutex>lock(mutex_);if(!stats_.failure.empty())throw std::runtime_error(stats_.failure);
    auto it=slots_.find(stats_.delivered+1);if(it==slots_.end()||it->second.stage!=Stage::Ready)return false;
    packet=it->second.packet;it->second.stage=Stage::Owner;++stats_.delivered;update();if(snapshot)*snapshot=stats_;return true;
  }
  void acknowledge(uint64_t seq){std::lock_guard<std::mutex>lock(mutex_);auto it=slots_.find(seq);if(seq!=stats_.committed+1||it==slots_.end()||it->second.stage!=Stage::Owner)throw std::logic_error("owner acknowledgement order differs");
    stats_.bytes-=it->second.bytes;slots_.erase(it);++stats_.committed;update();}
  void stop_admission(){std::lock_guard<std::mutex>lock(mutex_);stats_.closed=true;cv_.notify_all();}
  void fail(const std::string&message){std::lock_guard<std::mutex>lock(mutex_);if(stats_.failure.empty())stats_.failure=message;cv_.notify_all();}
  void close(){std::lock_guard<std::mutex>lock(mutex_);stats_.closed=true;if(!slots_.empty()||stats_.accepted!=stats_.delivered||stats_.accepted!=stats_.committed||!stats_.failure.empty())throw std::runtime_error("normal close is not drained");cv_.notify_all();}
  // Emergency leaves packet references/identities until the owner has recorded them.
  void abandon(){std::lock_guard<std::mutex>lock(mutex_);stats_.closed=true;stats_.canceled=slots_.size();cv_.notify_all();}
  std::vector<Remaining> remaining()const{std::lock_guard<std::mutex>lock(mutex_);std::vector<Remaining>r;for(const auto&x:slots_)r.push_back({x.first,x.second.kind,x.second.stage,x.second.packet});return r;}
  Stats stats()const{std::lock_guard<std::mutex>lock(mutex_);return stats_;}
 private:
  struct Slot {T packet;size_t bytes;int kind;Stage stage;};
  bool has_raw(int kind)const{for(const auto&row:slots_)if(row.second.kind==kind&&row.second.stage==Stage::Raw)return true;return false;}
  void update(){stats_.pending=slots_.size();stats_.inflight=stats_.ready=stats_.raw=0;for(const auto&row:slots_){switch(row.second.stage){case Stage::Raw:++stats_.raw;break;case Stage::Ready:++stats_.ready;break;default:++stats_.inflight;}}
    if(stats_.pending>stats_.peak_pending)stats_.peak_pending=stats_.pending;if(stats_.bytes>stats_.peak_bytes)stats_.peak_bytes=stats_.bytes;}
  const size_t count_limit_,byte_limit_;mutable std::mutex mutex_;std::condition_variable cv_;std::map<uint64_t,Slot>slots_;Stats stats_;
};
}
