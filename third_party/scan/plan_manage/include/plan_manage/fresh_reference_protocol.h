#pragma once
#include <cstdint>
#include <sstream>
#include <string>

namespace scan_planner {
// Observation only. A returned publish call is neither DDS delivery nor admission.
// All fields belong to one committed reference and reset on every invalidation.
struct ReferenceProtocolPublication {
  struct Attempt {
    bool completed{false}, success{false};
    uint64_t id{0}, sequence{0};
    std::string origin, reason;
  };
  std::string run_id;
  uint64_t epoch{0}, generation{0}, pid{0}, revision{0};
  int32_t stamp_sec{0}; uint32_t stamp_nanosec{0};
  uint64_t attempt_publish_count{0}, bspline_publish_count{0}, metadata_publish_count{0};
  uint64_t motion_bundle_publish_count{0}, gen_entry_count{0};
  int64_t first_gen_clock_ns{0}, first_gen_source_stamp_ns{0};
  Attempt first_attempt, last_attempt, active_attempt, last_motion_bundle;
  int64_t bundle_trajectory_id{0};
  int32_t bundle_start_sec{0}; uint32_t bundle_start_nanosec{0};
  bool active_attempt_published{false}, active_bspline_published{false}, active_metadata_published{false};
  int64_t active_trajectory_id{0};
  int32_t active_start_sec{0}; uint32_t active_start_nanosec{0};
  // Cumulative bounded diagnostic counters survive reference reset.
  uint64_t local_request_received{0}, local_request_accepted{0}, local_request_rejected{0};
  uint64_t last_request_sequence{0};
  int64_t last_request_clock_ns{0}, last_request_source_stamp_ns{0};
  std::string last_request_result{"none"}, last_request_reason{"none"};
  std::string last_requested_run_id, last_request_reference_run_id;
  uint64_t last_requested_epoch{0}, last_requested_revision{0};
  uint64_t last_requested_stamp_sec{0}, last_requested_stamp_nanosec{0};
  uint64_t last_request_reference_epoch{0}, last_request_reference_generation{0};
  uint64_t last_request_reference_pid{0}, last_request_reference_revision{0};
  int32_t last_request_reference_stamp_sec{0}; uint32_t last_request_reference_stamp_nanosec{0};
  bool last_request_payload_shape_valid{false}, last_requested_run_id_truncated{false};

  static std::string quoted(const std::string &value) {
    std::ostringstream out; out << '"';
    for (const unsigned char c : value) {
      if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
      else if (c >= 32 && c < 127) out << static_cast<char>(c);
      else out << '?';
    }
    out << '"'; return out.str();
  }
  void reset_reference() {
    run_id.clear(); epoch=generation=pid=revision=0; stamp_sec=0; stamp_nanosec=0;
    attempt_publish_count=bspline_publish_count=metadata_publish_count=motion_bundle_publish_count=gen_entry_count=0;
    first_gen_clock_ns=first_gen_source_stamp_ns=0;
    first_attempt=last_attempt=active_attempt=last_motion_bundle=Attempt{};
    bundle_trajectory_id=active_trajectory_id=0;
    bundle_start_sec=active_start_sec=0; bundle_start_nanosec=active_start_nanosec=0;
    active_attempt_published=active_bspline_published=active_metadata_published=false;
  }
  void commit(const std::string &run, uint64_t ep, uint64_t gen, uint64_t process,
              uint64_t rev, int32_t sec, uint32_t nanosec) {
    reset_reference(); run_id=run; epoch=ep; generation=gen; pid=process;
    revision=rev; stamp_sec=sec; stamp_nanosec=nanosec;
  }
  void gen_entered(int64_t clock_ns, int64_t source_stamp_ns) {
    if (++gen_entry_count == 1) {
      first_gen_clock_ns=clock_ns; first_gen_source_stamp_ns=source_stamp_ns;
    }
  }
  void begin_attempt(uint64_t id, const std::string &origin, uint64_t sequence) {
    active_attempt=Attempt{}; active_attempt.id=id; active_attempt.origin=origin;
    active_attempt.sequence=sequence;
    active_attempt_published=active_bspline_published=active_metadata_published=false;
  }
  void attempt_published() { ++attempt_publish_count; active_attempt_published=true; }
  void bspline_published(int64_t id, int32_t sec, uint32_t nanosec, bool motion) {
    ++bspline_publish_count;
    if (motion) {
      active_bspline_published=true; active_trajectory_id=id;
      active_start_sec=sec; active_start_nanosec=nanosec;
    }
  }
  void metadata_published(int64_t id, int32_t sec, uint32_t nanosec, bool motion) {
    ++metadata_publish_count;
    if (motion && active_bspline_published && id==active_trajectory_id &&
        sec==active_start_sec && nanosec==active_start_nanosec)
      active_metadata_published=true;
  }
  void complete_attempt(bool success, const std::string &reason) {
    // Called only AFTER all actual publish functions for this outcome returned.
    if (!revision || !active_attempt.id || !active_attempt_published ||
        (success && (!active_bspline_published || !active_metadata_published))) return;
    active_attempt.completed=true; active_attempt.success=success; active_attempt.reason=reason;
    last_attempt=active_attempt;
    if (!first_attempt.completed) first_attempt=active_attempt;
    if (success) {
      last_motion_bundle=active_attempt; ++motion_bundle_publish_count;
      bundle_trajectory_id=active_trajectory_id;
      bundle_start_sec=active_start_sec; bundle_start_nanosec=active_start_nanosec;
    }
  }
  void request_received(uint64_t sequence, int64_t clock_ns, int64_t source_stamp_ns,
      const std::string &requested_run_id, uint64_t requested_epoch, uint64_t requested_revision,
      uint64_t requested_stamp_sec, uint64_t requested_stamp_nanosec, bool payload_shape_valid) {
    ++local_request_received; last_request_sequence=sequence;
    last_request_clock_ns=clock_ns; last_request_source_stamp_ns=source_stamp_ns;
    last_request_result="received"; last_request_reason="pending_callback_validation";
    last_requested_run_id=requested_run_id.substr(0,256); last_requested_run_id_truncated=requested_run_id.size()>256;
    last_requested_epoch=requested_epoch;
    last_requested_revision=requested_revision; last_requested_stamp_sec=requested_stamp_sec;
    last_requested_stamp_nanosec=requested_stamp_nanosec;
    last_request_payload_shape_valid=payload_shape_valid;
    last_request_reference_run_id=run_id; last_request_reference_epoch=epoch;
    last_request_reference_generation=generation; last_request_reference_pid=pid;
    last_request_reference_revision=revision; last_request_reference_stamp_sec=stamp_sec;
    last_request_reference_stamp_nanosec=stamp_nanosec;
  }
  void request_result(bool accepted, const char *reason) {
    if (accepted) ++local_request_accepted; else ++local_request_rejected;
    last_request_result=accepted ? "accepted" : "rejected"; last_request_reason=reason;
  }
  static void attempt_json(std::ostringstream &out, const Attempt &a) {
    out << "{\"completed\":" << (a.completed ? "true" : "false")
        << ",\"id\":" << a.id << ",\"plan_success\":" << (a.success ? "true" : "false")
        << ",\"origin\":" << quoted(a.origin) << ",\"local_replan_request_sequence\":" << a.sequence
        << ",\"result_reason\":" << quoted(a.reason) << '}';
  }
  std::string json(const std::string &fsm_state, size_t attempt_subscribers,
                   size_t bspline_subscribers, size_t metadata_subscribers) const {
    std::ostringstream out;
    out << ",\"reference_protocol\":{\"schema\":\"r85_scan_reference_protocol_snapshot/v2\""
        << ",\"run_id\":" << quoted(run_id) << ",\"planner_epoch\":" << epoch
        << ",\"localization_generation\":" << generation << ",\"planner_process_id\":" << pid
        << ",\"committed_reference_revision\":" << revision
        << ",\"reference_stamp\":[" << stamp_sec << ',' << stamp_nanosec << ']'
        << ",\"fsm_state\":" << quoted(fsm_state)
        << ",\"reference_attempt_publish_count\":" << attempt_publish_count
        << ",\"bspline_publish_count\":" << bspline_publish_count
        << ",\"trajectory_metadata_publish_count\":" << metadata_publish_count
        << ",\"motion_bundle_publish_count\":" << motion_bundle_publish_count
        << ",\"matched_subscription_counts\":{\"reference_attempt\":" << attempt_subscribers
        << ",\"bspline\":" << bspline_subscribers << ",\"trajectory_metadata\":" << metadata_subscribers << '}'
        << ",\"first_gen_entry_count\":" << gen_entry_count
        << ",\"first_gen_clock_ns\":" << first_gen_clock_ns
        << ",\"first_gen_source_stamp_ns\":" << first_gen_source_stamp_ns
        << ",\"first_attempt\":"; attempt_json(out, first_attempt);
    out << ",\"last_attempt\":"; attempt_json(out, last_attempt);
    out << ",\"last_motion_bundle\":"; attempt_json(out, last_motion_bundle);
    out << ",\"motion_bundle_trajectory\":{\"traj_id\":" << bundle_trajectory_id
        << ",\"start_time\":[" << bundle_start_sec << ',' << bundle_start_nanosec << "]}"
        << ",\"local_request\":{\"received_count\":" << local_request_received
        << ",\"accepted_count\":" << local_request_accepted << ",\"rejected_count\":" << local_request_rejected
        << ",\"last_sequence\":" << last_request_sequence << ",\"last_clock_ns\":" << last_request_clock_ns
        << ",\"last_source_stamp_ns\":" << last_request_source_stamp_ns
        << ",\"last_result\":" << quoted(last_request_result) << ",\"last_reason\":" << quoted(last_request_reason)
        << ",\"cumulative_diagnostic_only\":true,\"payload_shape_valid\":" << (last_request_payload_shape_valid ? "true" : "false")
        << ",\"requested_run_id_truncated\":" << (last_requested_run_id_truncated ? "true" : "false")
        << ",\"requested_identity\":{\"run_id\":" << quoted(last_requested_run_id)
        << ",\"planner_epoch\":" << last_requested_epoch << ",\"committed_reference_revision\":" << last_requested_revision
        << ",\"reference_stamp\":[" << last_requested_stamp_sec << ',' << last_requested_stamp_nanosec << "]}"
        << ",\"reference_at_receive\":{\"run_id\":" << quoted(last_request_reference_run_id)
        << ",\"planner_epoch\":" << last_request_reference_epoch
        << ",\"localization_generation\":" << last_request_reference_generation
        << ",\"planner_process_id\":" << last_request_reference_pid
        << ",\"committed_reference_revision\":" << last_request_reference_revision
        << ",\"reference_stamp\":[" << last_request_reference_stamp_sec << ',' << last_request_reference_stamp_nanosec << "]}"
        << "},\"publish_calls_are_delivery_or_admission\":false}";
    return out.str();
  }
};
} // namespace scan_planner
