#ifndef R107_TRANSPORT_PROBE_H
#define R107_TRANSPORT_PROBE_H

// Default-OFF POD observer. No payloads, ROS publication, or hot-path file IO.
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <string>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace r107_transport
{
inline std::int64_t now(clockid_t which = CLOCK_MONOTONIC) noexcept
{
  timespec value{};
  if (clock_gettime(which, &value) != 0) return -1;
  return std::int64_t(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

class Probe
{
public:
  static constexpr std::size_t CAPACITY = 1024;
  static constexpr std::size_t WIDTH = 20;
  static constexpr std::size_t FILE_LIMIT = 256 * 1024;
  using Row = std::array<std::int64_t, WIDTH>;
  using Buffer = std::array<Row, CAPACITY>;

  void configure(const std::string &run, std::int64_t epoch,
                 std::int64_t start_ticks) noexcept
  {
    if (configured_) return;
    configured_ = true;
    const char *flag = std::getenv("R107_SCAN_TRANSPORT_PROBE");
    if (!flag || std::strcmp(flag, "1") != 0) return;
    requested_ = true;
    try {
      const char *directory = std::getenv("R107_SCAN_TRANSPORT_DIR");
      // Per-run total: 64 child epochs * 256KiB + Python <=2MiB <20MB.
      if (!directory || directory[0] != '/' || std::strlen(directory) > 4096 ||
          run.empty() || run.size() > 256 || epoch < 1 || epoch > 64) {
        ++errors_; return;
      }
      for (char c : run) {
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) {
          ++errors_; return;
        }
      }
      run_ = run;
      directory_ = directory;
      epoch_ = epoch;
      start_ticks_ = start_ticks;
      pid_ = getpid();
      tid_ = syscall(SYS_gettid);
      FILE *boot = std::fopen("/proc/sys/kernel/random/boot_id", "r");
      if (!boot) { ++errors_; return; }
      const bool got_boot = std::fgets(boot_, sizeof boot_, boot) != nullptr;
      std::fclose(boot);
      if (!got_boot) { ++errors_; return; }
      boot_[std::strcspn(boot_, "\r\n")] = '\0';
      rows_.reset(new Buffer{});
    } catch (...) { ++errors_; rows_.reset(); }
  }

  bool enabled() const noexcept { return bool(rows_); }
  void observation_error() noexcept { ++errors_; }

  static std::size_t slot(std::int64_t sequence) noexcept
  { return sequence <= 512 ? sequence - 1 : 512 + (sequence - 513) % 512; }

  void attempt(std::int64_t id, std::int64_t cached_odom_stamp) noexcept
  {
    if (!rows_) return;
    attempt_id_ = id;
    attempt_begin_mono_ = now();
    attempt_cached_odom_stamp_ = cached_odom_stamp;
  }

  void cached_source(std::int64_t sequence, std::int64_t cached_odom_stamp,
                     std::int64_t publish_ros_now) noexcept
  {
    if (!rows_ || sequence < 0) return;
    auto &row = (*rows_)[slot(sequence)];
    if (row[0] != sequence) { ++errors_; return; }
    row[16] = cached_odom_stamp;
    row[17] = publish_ros_now;
    if (row[9] == attempt_id_) {
      row[18] = attempt_begin_mono_;
      row[19] = attempt_cached_odom_stamp_;
    }
  }

  std::int64_t begin(int site, int kind, std::int64_t epoch,
                     std::int64_t generation, std::int64_t revision,
                     std::int64_t attempt, std::int64_t trajectory,
                     std::int64_t sec, std::int64_t nano,
                     std::int64_t size, int candidate_result) noexcept
  {
    if (!rows_) return -1;
    const auto wall = now();
    const auto cpu = now(CLOCK_THREAD_CPUTIME_ID);
    const auto sequence = ++recorded_;
    (*rows_)[slot(sequence)] = Row{{sequence, site, kind, wall, -1,
        tid_, epoch, generation, revision, attempt, trajectory, sec, nano,
        size, candidate_result, 0, -1, -1, -1, -1}};
    add_cost(wall, cpu);
    return sequence;
  }

  void end(std::int64_t sequence, bool returned) noexcept
  {
    if (sequence < 0 || !rows_) return;
    const auto wall = now();
    const auto cpu = now(CLOCK_THREAD_CPUTIME_ID);
    auto &row = (*rows_)[slot(sequence)];
    if (row[0] != sequence) { ++errors_; return; }
    row[4] = wall;
    row[15] = returned ? 1 : -1;
    ++finished_;
    add_cost(wall, cpu);
  }

  ~Probe() noexcept { flush(); }

private:
  void add_cost(std::int64_t wall, std::int64_t cpu) noexcept
  {
    const auto elapsed = now() - wall;
    cost_wall_ += elapsed;
    if (elapsed > max_cost_wall_) max_cost_wall_ = elapsed;
    cost_cpu_ += now(CLOCK_THREAD_CPUTIME_ID) - cpu;
  }

  void flush() noexcept
  {
    if (!requested_ || directory_.empty() || flushed_) return;
    flushed_ = true;
    try {
      const auto path = directory_ + "/r107_scan_transport_cpp_" + std::to_string(pid_) +
          "_" + std::to_string(start_ticks_) + ".json";
      int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
      if (fd < 0) { ++errors_; return; }
      FILE *stream = ::fdopen(fd, "w");
      if (!stream) { ::close(fd); ++errors_; return; }
      std::fprintf(stream,
          "{\"schema\":\"r107_scan_transport_cpp/v1\",\"pid\":%lld,\"start_ticks\":%lld,"
          "\"boot_id\":\"%s\",\"run_id\":\"%s\",\"epoch\":%lld,"
          "\"capacity\":1024,\"buffer_bytes\":%zu,\"recorded\":%lld,\"finished\":%lld,"
          "\"evicted\":%lld,\"errors\":%lld,\"cost_wall_ns\":%lld,\"cost_cpu_ns\":%lld,"
          "\"cost_wall_max_ns\":%lld,\"clock\":\"CLOCK_MONOTONIC\","
          "\"scope\":\"publish_call_interval_not_DDS_delivery\","
          "\"retention\":\"first512_plus_tail512\","
          "\"cached_odom_is_complete_planner_input_proof\":false,"
          "\"sites\":{\"1\":\"motion_spline\",\"2\":\"emergency_spline\","
          "\"3\":\"attempt_including_negative\",\"4\":\"metadata_motion_or_emergency\"},"
          "\"kinds\":{\"1\":\"attempt\",\"2\":\"spline\",\"3\":\"metadata\"},"
          "\"columns\":[\"seq\",\"site\",\"kind\",\"begin_mono_ns\",\"end_mono_ns\","
          "\"tid\",\"epoch\",\"generation\",\"revision\",\"attempt_id\",\"traj_id\","
          "\"start_sec\",\"start_nsec\",\"size\",\"candidate_result\",\"publish_returned\","
          "\"cached_odom_at_publish_SIM_ns\",\"publish_ROS_now_ns\",\"attempt_begin_MONO_ns\",\"cached_odom_at_attempt_SIM_ns\"],\"rows\":[",
          static_cast<long long>(pid_), static_cast<long long>(start_ticks_), boot_, run_.c_str(),
          static_cast<long long>(epoch_), rows_ ? sizeof(Buffer) : 0,
          static_cast<long long>(recorded_), static_cast<long long>(finished_),
          static_cast<long long>(recorded_ > 1024 ? recorded_ - 1024 : 0),
          static_cast<long long>(errors_), static_cast<long long>(cost_wall_),
          static_cast<long long>(cost_cpu_), static_cast<long long>(max_cost_wall_));
      const auto retained = recorded_ < 1024 ? recorded_ : 1024;
      std::int64_t emitted = 0;
      for (std::int64_t index = 0; rows_ && index < retained; ++index) {
        const auto prefix = recorded_ < 512 ? recorded_ : 512;
        const auto tail_first = recorded_ - 511 > 513 ? recorded_ - 511 : 513;
        const auto sequence = index < prefix ? index + 1 : tail_first + index - prefix;
        const auto &row = (*rows_)[slot(sequence)];
        char text[1024];
        int used = std::snprintf(text, sizeof text, "%s[", index ? "," : "");
        for (std::size_t column = 0; column < WIDTH; ++column) {
          used += std::snprintf(text + used, sizeof text - used, "%s%lld",
              column ? "," : "", static_cast<long long>(row[column]));
        }
        used += std::snprintf(text + used, sizeof text - used, "]");
        const auto position = std::ftell(stream);
        if (position < 0 || position + used + 128 > static_cast<long>(FILE_LIMIT)) break;
        if (std::fwrite(text, 1, used, stream) != static_cast<std::size_t>(used)) break;
        ++emitted;
      }
      std::fprintf(stream, "],\"rows_emitted\":%lld,\"output_omitted_rows\":%lld}\n",
                   static_cast<long long>(emitted), static_cast<long long>(retained - emitted));
      std::fclose(stream);
    } catch (...) { ++errors_; }
  }

  std::unique_ptr<Buffer> rows_;
  bool configured_{false}, requested_{false}, flushed_{false};
  std::string run_, directory_;
  char boot_[64]{};
  std::int64_t epoch_{-1}, start_ticks_{-1}, pid_{-1}, tid_{-1};
  std::int64_t attempt_id_{-1}, attempt_begin_mono_{-1}, attempt_cached_odom_stamp_{-1};
  std::int64_t recorded_{0}, finished_{0}, errors_{0};
  std::int64_t cost_wall_{0}, cost_cpu_{0}, max_cost_wall_{0};
};

inline Probe &probe() noexcept { static Probe value; return value; }

class Publication
{
public:
  Publication(int site, int kind, std::int64_t epoch, std::int64_t generation,
              std::int64_t revision, std::int64_t attempt, std::int64_t trajectory,
              std::int64_t sec, std::int64_t nano, std::int64_t size,
              int candidate_result) noexcept
    : sequence_(probe().begin(site, kind, epoch, generation, revision, attempt,
                             trajectory, sec, nano, size, candidate_result)) {}
  Publication(const Publication &) = delete;
  Publication &operator=(const Publication &) = delete;
  void cached_source(std::int64_t stamp, std::int64_t ros_now) noexcept
  { probe().cached_source(sequence_, stamp, ros_now); }
  void returned() noexcept { returned_ = true; }
  ~Publication() noexcept { probe().end(sequence_, returned_); }
private:
  std::int64_t sequence_;
  bool returned_{false};
};
}  // namespace r107_transport
#endif
