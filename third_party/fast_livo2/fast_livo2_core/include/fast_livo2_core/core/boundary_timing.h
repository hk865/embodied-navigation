#pragma once
// Optional bounded observation only: never supplies an estimator input.
#include "diagnostics.h"
#include <array>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/syscall.h>
#include <sstream>
#include <iomanip>
#include <charconv>
#include <nlohmann/json.hpp>

namespace fastlivo_timing {
constexpr size_t boundary_count=17;
// Optional scalar output only; does not enable the general native logger.
// A software cap limits both logical size and predicted allocated blocks.
struct ScalarWriter {
  int fd=-1;
  std::mutex mutex;
  ScalarWriter() noexcept {
    const char* path=std::getenv("FASTLIVO_BOUNDARY_TIMING_FILE");
    if(path && *path) fd=::open(path,O_CREAT|O_EXCL|O_WRONLY|O_APPEND|O_NOFOLLOW|O_CLOEXEC,0600);
  }
  ~ScalarWriter() { if(fd>=0)::close(fd); }
  void emit(const std::vector<double>& values) noexcept {
    try {
      if(fd<0)return;
      std::ostringstream text;
      text<<"{\"schema\":\"R73_native_boundary_timing/v1\",\"tid\":"<<::syscall(SYS_gettid)<<",\"values\":[";
      for(size_t i=0;i<values.size();++i) {if(i)text<<',';text<<std::setprecision(17)<<values[i];}
      text<<"]}\n";
      emit_raw(text.str());
    } catch(...) {}  // Observation must never replace a primary exception.
  }
  void emit_raw(const std::string& raw) noexcept {
    try {
      if(fd<0 || raw.size()>4096)return;
      std::lock_guard<std::mutex> guard(mutex);
      struct stat st{};struct statvfs fs{};
      if(::fstat(fd,&st)||::fstatvfs(fd,&fs)||!S_ISREG(st.st_mode)||!fs.f_frsize)return;
      constexpr uint64_t cap=8*1024*1024;
      const uint64_t predicted=((uint64_t(st.st_size)+raw.size()+fs.f_frsize-1)/fs.f_frsize)*fs.f_frsize;
      if(uint64_t(st.st_blocks)*512>cap||predicted>cap)return;
      // No fsync or estimator mutation. Partial/failing diagnostic output is
      // never a receipt; the offline reader must reject incomplete lines.
      (void)::write(fd,raw.data(),raw.size());
    } catch(...) {}  // Observation must never replace a primary exception.
  }
};
inline ScalarWriter& scalar_writer() { static ScalarWriter writer;return writer; }
struct Sample { uint64_t wall=0,thread_cpu=0,process_cpu=0; bool valid=true; };
inline uint64_t read_clock(clockid_t id,bool& valid) noexcept {
  timespec value{};
  if(clock_gettime(id,&value)!=0) { valid=false; return 0; }
  return uint64_t(value.tv_sec)*1000000000ULL+uint64_t(value.tv_nsec);
}
inline Sample sample() noexcept {
  Sample s;
  s.wall=read_clock(CLOCK_MONOTONIC,s.valid);
  s.thread_cpu=read_clock(CLOCK_THREAD_CPUTIME_ID,s.valid);
  s.process_cpu=read_clock(CLOCK_PROCESS_CPUTIME_ID,s.valid);
  return s;
}
inline bool enabled() {
  static const bool selected=[] { const char* value=std::getenv("FASTLIVO_BOUNDARY_TIMING");
    return value && std::string(value)=="1"; }();
  return selected && (scalar_writer().fd>=0 || fastlivo_diag::Logger::instance().enabled());
}
struct Counters { uint64_t calls=0,wall=0,thread_cpu=0,process_cpu=0,invalid=0; };
struct Window { uint64_t begin=0,rows=0; std::array<Counters,boundary_count> totals{}; };
inline Window& window() { static thread_local Window value; return value; }
inline void completed(size_t boundary,const Sample& before,const Sample& after) {
  auto& w=window(); if(!w.begin)w.begin=before.wall;
  auto& c=w.totals[boundary]; ++c.calls;
  if(before.valid && after.valid && after.wall>=before.wall && after.thread_cpu>=before.thread_cpu && after.process_cpu>=before.process_cpu) {
    c.wall+=after.wall-before.wall;
    c.thread_cpu+=after.thread_cpu-before.thread_cpu;
    c.process_cpu+=after.process_cpu-before.process_cpu;
  } else ++c.invalid;
  if(after.wall>=w.begin && after.wall-w.begin>=1000000000ULL) {
    std::vector<double> values; values.reserve(5+boundary_count*5);
    values.insert(values.end(),{1,double(++w.rows),double(w.begin),double(after.wall),double(boundary_count)});
    for(const auto& total:w.totals)
      values.insert(values.end(),{double(total.calls),double(total.wall),double(total.thread_cpu),double(total.process_cpu),double(total.invalid)});
    const auto context=fastlivo_diag::Logger::instance().context();
    if(scalar_writer().fd>=0)scalar_writer().emit(values);
    else fastlivo_diag::Logger::instance().raw(300,context.stamp_ns,std::move(values));
    w.begin=after.wall; w.totals={};
  }
}
// Integers from the same CLOCK_MONOTONIC domain as original pipeline CSV.
// Diagnostic source selection does not select estimator data or grant health.
inline uint64_t frame_from_ns() noexcept {
  static const uint64_t from=[]() noexcept {
    const char* text=std::getenv("FASTLIVO_FRAME_TIMING_FROM_NS");
    if(!text || !*text)return UINT64_MAX;
    uint64_t out=0;const char* end=text+std::strlen(text);
    const auto parsed=std::from_chars(text,end,out);
    return parsed.ec==std::errc{} && parsed.ptr==end ? out : UINT64_MAX;
  }();return from;
}
inline std::array<uint64_t,11>& frame_wall() noexcept {
  static thread_local std::array<uint64_t,11> value{};return value;
}
inline std::array<uint64_t,11>& frame_cpu() noexcept {static thread_local std::array<uint64_t,11> value{};return value;}
inline void frame_mark(size_t i) noexcept {
  if(frame_from_ns()==UINT64_MAX || i>=frame_wall().size())return;
  const auto measured=sample();
  frame_wall()[i]=measured.valid?measured.wall:0;
  frame_cpu()[i]=measured.valid?measured.thread_cpu:0;
}
inline void frame_reset() noexcept {frame_wall().fill(0);frame_cpu().fill(0);frame_mark(0);}
inline void emit_frame(const nlohmann::json& identity) noexcept {
  try {
    const auto source=identity.at("stamp_ns").get<uint64_t>();
    if(frame_from_ns()==UINT64_MAX || source<frame_from_ns())return;
    const auto& times=frame_wall();
    for(size_t i=0;i<times.size();++i)if(!times[i] || (i && times[i]<times[i-1]))return;
    nlohmann::json row={{"schema","R73_native_source_frame_timing/v1"},
      {"tid",::syscall(SYS_gettid)},{"producer_epoch",identity.at("producer_epoch")},
      {"run_id",identity.at("run_id")},{"stage",identity.at("stage")},
      {"post_lio_header_ns",source},{"commit",identity.at("commit")},
      {"source_set",identity.at("sources")},{"point_count",identity.at("point_count")},
      {"point_refs_sha256",identity.at("point_refs_sha256")},
      {"registered_message_sha256",identity.at("registered_message_sha256")},
      {"mapper_message_sha256",identity.at("mapper_message_sha256")},
      {"wall_ns",times},{"thread_cpu_ns",frame_cpu()},{"clock_valid",true},
      {"wall_labels",{"sync_begin","sync_selected","imu_begin","imu_done","lio_begin",
        "odom_publish_begin","odom_publish_return","navigation_construct_begin","navigation_construct_ready","cloud_publish_return",
        "identity_publish_return"}},
      {"observation_only",true},{"publication_return_is_DDS_delivery_receipt",false}};
    scalar_writer().emit_raw(row.dump()+"\n");
  } catch(...) {} // Missing diagnostic fields never replace original result.
}
class Scope {
 public:
  explicit Scope(size_t boundary):boundary_(boundary),active_(enabled()) {
    if(active_) before_=sample();
  }
  ~Scope() noexcept { try { if(active_)completed(boundary_,before_,sample()); } catch(...) {} }
  Scope(const Scope&)=delete; Scope& operator=(const Scope&)=delete;
 private:
  size_t boundary_; bool active_; Sample before_;
};
}
