#pragma once
// Diagnostic copies only. No estimator state, branch or residual is modified.
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

namespace fastlivo_diag {
inline uint64_t nanoseconds(double sec) {
  return std::isfinite(sec) && sec >= 0 ? static_cast<uint64_t>(std::llround(sec*1e9)) : 0;
}
template<class Derived> inline void append(std::vector<double>& out, const Eigen::MatrixBase<Derived>& value) {
  for(Eigen::Index i=0;i<value.rows();++i)
    for(Eigen::Index j=0;j<value.cols();++j) out.push_back(value(i,j));
}
template<class State> inline void state(std::vector<double>& out, const State& s) {
  append(out,s.rot_end); append(out,s.pos_end); out.push_back(s.inv_expo_time);
  append(out,s.vel_end); append(out,s.bias_g); append(out,s.bias_a); append(out,s.gravity);
}

struct Context {
  uint64_t sequence=0, stamp_ns=0, stage=0;
};
struct Record {
  // kind, stage sequence, source/estimator ns, stage, iteration, level, values
  uint64_t header[7];
  std::vector<double> data;
};
class Logger {
 public:
  static Logger& instance() { static Logger logger; return logger; }
  bool enabled() const { return enabled_; }
  bool detail_at(double source_s) const {
    return enabled_ && source_s>=detail_begin_ && source_s<=detail_end_;
  }
  bool detail() const { return detail_at(context_.stamp_ns*1e-9); }
  Context context() const { return context_; }
  void set_context(uint64_t stage,uint64_t stamp_ns) {
    if(!enabled_) return;
    context_={++sequence_,stamp_ns,stage};
  }
  void emit(uint64_t kind,int iteration,int level,std::vector<double>&& data) {
    if(!enabled_) return;
    enqueue({{kind,context_.sequence,context_.stamp_ns,context_.stage,
              static_cast<uint64_t>(static_cast<int64_t>(iteration)),
              static_cast<uint64_t>(static_cast<int64_t>(level)),data.size()},std::move(data)});
  }
  void raw(uint64_t kind,uint64_t stamp_ns,std::vector<double>&& data) {
    if(!enabled_) return;
    enqueue({{kind,0,stamp_ns,0,0,0,data.size()},std::move(data)});
  }
  ~Logger() {
    if(!enabled_) return;
    {std::lock_guard<std::mutex> guard(mutex_); stopping_=true;}
    condition_.notify_all();
    if(writer_.joinable()) writer_.join();
  }
 private:
  Logger() {
    const char* directory=std::getenv("FASTLIVO_DIAGNOSTIC_DIR");
    if(!directory || !*directory) return;
    directory_=directory;
    if(const char* t=std::getenv("FASTLIVO_DIAGNOSTIC_BEGIN")) detail_begin_=std::stod(t);
    if(const char* t=std::getenv("FASTLIVO_DIAGNOSTIC_END")) detail_end_=std::stod(t);
    std::filesystem::create_directories(directory_);
    const auto path=directory_/"records.bin";
    if(std::filesystem::exists(path)) throw std::runtime_error("Diagnostic file already exists");
    binary_.open(path,std::ios::binary|std::ios::out);
    if(!binary_) throw std::runtime_error("Cannot open diagnostic binary");
    const char magic[16]={'F','L','I','V','O','D','I','A','G','0','0','0','1','L','E',0};
    binary_.write(magic,sizeof(magic));
    index_.open(directory_/"index.csv",std::ios::out);
    if(!index_) throw std::runtime_error("Cannot open diagnostic index");
    index_<<"offset,kind,sequence,stamp_ns,stage,iteration,level,n_values\n";
    enabled_=true;
    writer_=std::thread([this]{
      try { write_loop(); } catch(...) { io_failed_=true; }
    });
  }
  void enqueue(Record&& record) {
    attempted_++;
    const size_t bytes=sizeof(record.header)+record.data.size()*sizeof(double);
    std::unique_lock<std::mutex> guard(mutex_);
    if(stopping_ || queue_.size()>=256 || queued_bytes_+bytes>32*1024*1024) {
      dropped_++; return;
    }
    queued_bytes_+=bytes;
    queue_.push_back(std::move(record));
    const auto size=queue_.size();
    if(size>maximum_queue_) maximum_queue_=size;
    guard.unlock(); condition_.notify_one();
  }
  void save_stats(bool final) {
    try {
    const auto tmp=directory_/"writer_stats.tmp";
    std::ofstream out(tmp);
    out<<"{\"schema\":\"fastlivo_diagnostics/v1\",\"final\":"<<(final?"true":"false")
       <<",\"attempted\":"<<attempted_.load()<<",\"written\":"<<written_.load()
       <<",\"dropped\":"<<dropped_.load()<<",\"maximum_queue_records\":"<<maximum_queue_.load()
       <<",\"file_bytes\":"<<static_cast<uint64_t>(binary_.tellp())
       <<",\"detail_begin_s\":"<<detail_begin_<<",\"detail_end_s\":"<<detail_end_
       <<",\"writer_io_failed\":"<<(io_failed_?"true":"false")<<"}\n";
    out.close();
    if(!out) { io_failed_=true; return; }
    std::filesystem::rename(tmp,directory_/"writer_stats.json");
    } catch(...) { io_failed_=true; }
  }
  void write_loop() {
    auto last=std::chrono::steady_clock::now();
    while(true) {
      Record record;
      {
        std::unique_lock<std::mutex> guard(mutex_);
        condition_.wait_for(guard,std::chrono::milliseconds(200),[this]{return stopping_||!queue_.empty();});
        if(queue_.empty()) {
          if(stopping_) break;
          guard.unlock();
          if(std::chrono::steady_clock::now()-last>std::chrono::seconds(2)) {
            binary_.flush(); index_.flush(); save_stats(false); last=std::chrono::steady_clock::now();
          }
          continue;
        }
        record=std::move(queue_.front()); queue_.pop_front();
        queued_bytes_-=sizeof(record.header)+record.data.size()*sizeof(double);
      }
      const auto offset=static_cast<uint64_t>(binary_.tellp());
      binary_.write(reinterpret_cast<const char*>(record.header),sizeof(record.header));
      binary_.write(reinterpret_cast<const char*>(record.data.data()),record.data.size()*sizeof(double));
      index_<<offset;
      for(int i=0;i<7;++i) index_<<','<<record.header[i];
      index_<<'\n';
      written_++; if(!binary_||!index_) io_failed_=true;
      if(std::chrono::steady_clock::now()-last>std::chrono::seconds(2)) {
        binary_.flush(); index_.flush(); save_stats(false); last=std::chrono::steady_clock::now();
      }
    }
    binary_.flush(); index_.flush(); save_stats(true);
  }
  bool enabled_=false,stopping_=false,io_failed_=false;
  double detail_begin_=115,detail_end_=165;
  uint64_t sequence_=0;
  Context context_;
  std::filesystem::path directory_;
  std::ofstream binary_,index_;
  std::thread writer_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Record> queue_;
  size_t queued_bytes_=0;
  std::atomic<uint64_t> attempted_{0},written_{0},dropped_{0},maximum_queue_{0};
};
}
