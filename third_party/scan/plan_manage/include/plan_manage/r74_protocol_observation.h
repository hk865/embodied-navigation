#pragma once
// Observation only. No ROS, map, planning, or acceptance dependencies.
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace scan_planner {
namespace r74_observation {
inline std::string quoted(const std::string &s) {
  std::ostringstream out; out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') out << '\\' << char(c);
    else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << unsigned(c) << std::dec;
    else out << char(c);
  }
  out << '"'; return out.str();
}
inline std::string number(double v) {
  if (std::isfinite(v)) { std::ostringstream s; s << std::setprecision(17) << v; return s.str(); }
  return quoted(std::isnan(v) ? "nan" : v > 0 ? "+inf" : "-inf");
}
inline std::uint32_t crc32_ieee(const std::string &data) {
  // Initialized only on the first enabled checksum call. IEEE reflected CRC32.
  static const std::array<std::uint32_t, 256> table = [] {
    std::array<std::uint32_t, 256> t{};
    for (std::uint32_t i=0; i<256; ++i) {
      std::uint32_t c=i;
      for (unsigned b=0; b<8; ++b) c=(c & 1) ? 0xedb88320U ^ (c >> 1) : c >> 1;
      t[i]=c;
    }
    return t;
  }();
  std::uint32_t c=0xffffffffU;
  for (unsigned char b : data) c=table[(c ^ b) & 255U] ^ (c >> 8);
  return c ^ 0xffffffffU;
}
inline std::string crc_hex(std::uint32_t c) {
  std::ostringstream s; s << std::hex << std::setw(8) << std::setfill('0') << c; return s.str();
}

struct Bounded {
  enum : std::size_t { PREFIX=8, TAIL=8, RECORD_BYTES=4096, EMIT_CAP=128,
    NORMAL_HASH_CAP=256, TARGET_CAPTURE_CAP=256,
    HASH_BYTES=16*1024*1024, MESSAGE_BYTES=1024*1024 };
  static constexpr std::int64_t PERIOD_NS=5000000000LL;
  static constexpr std::int64_t MIN_EMIT_SPACING_NS=2000000000LL;
  std::array<std::string, PREFIX> prefix{};
  std::array<std::string, TAIL> tail{};
  std::size_t prefix_count{0}, prefix_cursor{0}, tail_count{0}, tail_next{0}, tail_cursor{0};
  std::uint64_t strings_seen{0}, checksums{0}, normal_checksums{0}, checksum_bytes{0};
  std::uint64_t checksum_omitted{0}, errors{0}, failed_targets{0}, target_omitted{0}, emitted{0}, oversize_records{0};
  bool reserved_failure_checksum_used{false}, first_failure_checksum_attempted{false}, first_failure_pending{false};
  std::int64_t last_emit_ns{0};
  std::string first_failure, last_failure, first_failure_fingerprint, latest_fingerprint, endpoints{"[]"};

  // Per-process counters and caps never reset on reference/epoch changes.
  bool checksum_allowed(std::size_t bytes, bool first_failure) {
    ++strings_seen;
    const bool reserved=first_failure && !first_failure_checksum_attempted;
    if (reserved) first_failure_checksum_attempted=true;
    if (bytes > MESSAGE_BYTES) { ++checksum_omitted; return false; }
    if (reserved) {
      reserved_failure_checksum_used=true; ++checksums; checksum_bytes+=bytes; return true;
    }
    const std::uint64_t normal_bytes=checksum_bytes; // conservative: reserved bytes also consume budget
    if (normal_checksums >= NORMAL_HASH_CAP || normal_bytes+bytes > HASH_BYTES) {
      ++checksum_omitted; return false;
    }
    ++normal_checksums; ++checksums; checksum_bytes+=bytes; return true;
  }
  void store_fingerprint(const std::string &record, bool first_failure_record=false) {
    if (record.size()>RECORD_BYTES) { ++errors; return; }
    latest_fingerprint=record;
    if (first_failure_record) first_failure_fingerprint=record;
    if (prefix_count<PREFIX) prefix[prefix_count++]=record;
    tail[tail_next]=record; tail_next=(tail_next+1)%TAIL;
    if (tail_count<TAIL) ++tail_count;
  }
  void failure(const std::string &record) {
    if (record.size()>RECORD_BYTES) { ++errors; return; }
    if (first_failure.empty()) { first_failure=record; first_failure_pending=true; }
    last_failure=record;
  }
  bool due(std::int64_t now, bool immediate) const {
    if (emitted>=EMIT_CAP) return false;
    if (last_emit_ns!=0 && now-last_emit_ns<MIN_EMIT_SPACING_NS) return false;
    if (immediate && first_failure_pending) return true;
    // Reserve the final process emission for the first actual failed target.
    return emitted<EMIT_CAP-1 && (last_emit_ns==0 || now-last_emit_ns>=PERIOD_NS);
  }
  std::string next_fingerprint(bool immediate) {
    if (immediate) return first_failure_fingerprint.empty() ? "null" : first_failure_fingerprint;
    if (prefix_cursor<prefix_count) return prefix[prefix_cursor++];
    if (!tail_count) return "null";
    const std::size_t oldest=tail_count==TAIL ? tail_next : 0;
    return tail[(oldest+(tail_cursor++%tail_count))%TAIL];
  }
  void emitted_at(std::int64_t now, bool immediate) {
    ++emitted; last_emit_ns=now;
    if (immediate) first_failure_pending=false;
  }
};
} // namespace r74_observation
} // namespace scan_planner
