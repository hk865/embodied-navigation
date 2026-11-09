"""Wall tick / ROS clock boundary. A duplicate clock can only emit exact zero.

This module has no ROS, policy, geometry or controller mathematics. The caller
must use ``before_control`` before arrival, heading, PI and native guard code.
Source headers / receipt times are compared as immutable evidence and are never
restamped. The old native geometry clock contract stays strictly increasing.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from embodied_navigation.common.input_freshness import (
    enabled as minimal_baseline_enabled,
    raw_imu_fresh,
    wall_stale,
    wall_stale_ns,
)

TTL_NS = 300_000_000
CONTRACT = {
    "schema": "teacher_control_clock_exact_zero_hold/v1",
    "same_clock_policy": "exact_zero_before_math_and_native_geometry",
    "freshness_sim_and_wall_s": 0.3,
    "continuous_stale_fail_wall_s": 8.0,
    "backward_clock_policy": "latch_failure",
    "resume_policy": "actual_clock_advance_and_fresh_original_sources_and_native_geometry",
    "short_hold_integral_and_dwell": "frozen",
    "long_hold_integral_and_dwell": "protect_reset",
    "source_receipt_refresh_permitted": False,
}


def identity(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


class ControlClockHold:
    """Compare accepted clock/header progress and decide hold/advance/backwards.

    ``classify`` returns "advance", "duplicate", "stalled" or "backwards"; a
    duplicate/stall forces exact-zero control before any math or native geometry.
    """

    def __init__(self):
        self.last_clock_ns = None
        self.last_wall_ns = None
        self.last_progress_wall_ns = None
        self.holding = False
        self.guard_required = False
        self.records = 0
        self.sources = {}

    def source(self, name, stamp, value):
        if value is None:
            return
        digest = identity(value)
        old = self.sources.get(name)
        if old is not None:
            if stamp < old[0]:
                raise ValueError(name + "_accepted_header_backwards")
            if stamp == old[0] and digest != old[1]:
                raise ValueError(name + "_same_header_changed_payload_or_receipt")
        self.sources[name] = (stamp, digest)

    def classify(self, clock_ns, wall_ns, actual_guard_ns):
        if any(type(x) is not int or x < 0 for x in (clock_ns, wall_ns)):
            raise ValueError("Original nonnegative integer clocks required")
        previous = self.last_clock_ns
        if (
            (previous is not None and clock_ns < previous)
            or (actual_guard_ns is not None and clock_ns < actual_guard_ns)
            or (self.last_wall_ns is not None and wall_ns < self.last_wall_ns)
        ):
            return "backwards"
        duplicate = (previous is not None and clock_ns == previous) or (
            actual_guard_ns is not None and clock_ns == actual_guard_ns
        )
        if previous is None or clock_ns > previous:
            self.last_progress_wall_ns = wall_ns
        self.last_clock_ns, self.last_wall_ns = clock_ns, wall_ns
        if duplicate:
            self.holding = self.guard_required = True
            return "stalled" if wall_ns - self.last_progress_wall_ns >= TTL_NS else "duplicate"
        self.holding = False
        return "advance"

    def guard_completed(self):
        self.guard_required = False

    @staticmethod
    def snapshot(node):
        c = node.cascade
        return dict(
            velocity_integral=None if c is None else c.velocity_integral.tolist(),
            hold_integral=None if c is None else c.hold_integral.tolist(),
            math_pose_stamp_ns=None if c is None else c.last_math_stamp,
            capture_dwell_pose_stamp_ns=None if c is None else c.capture_dwell,
            turn_dwell_pose_stamp_ns=None if c is None else c.turn_dwell,
            parking_hold_pose_stamp_ns=None if c is None else c.hold_stamp,
            obstacle_clear_since_s=node.obstacle_clear_since,
            arrival_since_s=node.arrival_since,
            region_arrival_since_stamp_ns=None
            if node.region_arrival is None
            else node.region_arrival.since,
            pid_records=node.pid_records,
            actual_guard_records=node.guard_sequence,
        )

    def before_control(self, node, clock_ns, wall_ns):
        """Return True when exact-zero publication completely consumed this tick.

        A short duplicate freezes all mathematics / dwell. A >=300 ms wall
        clock stall or original-source staleness resets protected dwell and PI,
        requiring the original two fresh source headers to recover. The same
        continuous-stale >8 s failure rule remains. Every hold still evaluates
        the original IMU/SLAM/bridge severe protections and evidence error.
        """
        if node.state not in ("running", "succeeded"):
            return False
        now = wall_ns / 1e9
        previous_clock = self.last_clock_ns
        previous_guard = node.last_actual_guard_clock_ns
        before = self.snapshot(node)
        try:
            self.source("SLAM", int(node.pose_stamp), node.feedback)
            c = node.guard_cloud_receipt
            if c is not None:
                self.source(
                    "registered_cloud",
                    int(node.cloud_stamp),
                    {
                        k: copy.deepcopy(c[k])
                        for k in (
                            "stamp_ns",
                            "received_monotonic_wall",
                            "filtered_xyz_float64_sha256",
                            "nearest_slam_pose_stamp_ns",
                        )
                    },
                )
            decision = self.classify(clock_ns, wall_ns, previous_guard)
        except (ValueError, TypeError, KeyError) as error:
            decision = "invalid_immutable_source:" + str(error)
        if decision == "advance":
            # Mandatory actual geometry check stays pending until an actual
            # checked path can be used; select_pid_velocity ORs this flag.
            return False
        self.guard_required = True
        protected = node.apply_tilt_guard(now, clock_ns / 1e9)
        stale = []
        timeout = node.profile["pose_cloud_timeout_s"]
        for name, stamp, receipt in (
            ("SLAM", node.pose_stamp, node.pose_updated),
            ("registered_cloud", node.cloud_stamp, node.cloud_updated),
        ):
            if wall_stale(
                node,
                name + "_clock_hold",
                now,
                receipt,
                timeout,
                not math.isfinite(receipt) or not -0.05 <= now - receipt < timeout,
                future_tolerance=0.05,
            ) or not -50_000_000 <= clock_ns - stamp < round(timeout * 1e9):
                stale.append(name + "_sim_or_wall_TTL")
        if not raw_imu_fresh(node, now, clock_ns / 1e9, node.raw_imu.fresh(now, clock_ns / 1e9)):
            stale.append("raw_IMU_sim_or_wall_TTL")
        # Paired sources carry original integer receipt times. Zero publication
        # never updates these, the bridge's cascade source file, or an ACK.
        for name, source in (("feedback", node.feedback), ("causal_IMU", node.paired_imu)):
            if (
                source is None
                or not 0 <= clock_ns - source["stamp_ns"] <= TTL_NS
                or wall_stale_ns(
                    node,
                    name + "_clock_hold",
                    wall_ns,
                    source["received_wall_ns"],
                    TTL_NS,
                    not 0 <= wall_ns - source["received_wall_ns"] <= TTL_NS,
                )
            ):
                stale.append(name + "_sim_or_wall_TTL")
        fatal = decision == "backwards" or decision.startswith("invalid_immutable_source:")
        if node.evidence.error:
            fatal = True
            stale.append("evidence_writer:" + str(node.evidence.error))
        if fatal:
            node.state = "failed"
            node.message = "控制时钟倒退或实际输入身份异常，停车：" + decision
        if decision == "stalled" and wall_stale(
            node, "control_clock_stall", now, self.last_progress_wall_ns / 1e9, TTL_NS / 1e9, True
        ):
            stale.append("control_ROS_clock_wall_stall_300ms")
        reset = fatal or protected or bool(stale) or node.state == "failed"
        if reset:
            node.obstacle_clear_since = None
            node.reset_region_arrival(
                "failed" if node.state == "failed" else "stale" if stale else "protected"
            )
            if node.cascade is not None:
                reason = (
                    "clock_backwards"
                    if decision == "backwards"
                    else "actual_source_identity_invalid"
                    if decision.startswith("invalid_immutable_source:")
                    else "evidence_writer"
                    if node.evidence.error
                    else "control_clock_stall_or_source_stale"
                    if stale
                    else "external_hold"
                )
                node.cascade._protect(reason, fatal=fatal)
            if stale:
                node.stale_since = node.stale_since or now
                if now - node.stale_since > 8.0:
                    node.state = "failed"
                    node.message = "实际SLAM/点云/时钟输入连续失联超过8秒，停车"
            elif not fatal:
                node.stale_since = None
        elif node.state != "failed":
            node.message = "ROS仿真时钟本帧未前进，零速度保护等待下一时刻完整重验点云"
        # Do not consume a pre-existing row as a newly computed command source.
        # Normal single-threaded control flushes these before the next tick.
        if node.pid_row is not None or node.pending_guard_row is not None:
            raise RuntimeError("Clock hold entered with an unflushed control/geometry row")
        old_command = list(node.command)
        node.pid_guard_needs_evaluation = True
        source_preservation = None
        if (
            getattr(node, "automatic_reference", None) is not None
            and decision == "duplicate"
            and not reset
        ):
            from embodied_navigation.control.safety.duplicate_source import CONTEXT_SCHEMA

            duplicate_context = dict(
                schema=CONTEXT_SCHEMA,
                decision=decision,
                protected_or_stale_reset=reset,
                source_stale_reasons=list(stale),
                compute_ros_clock_ns=clock_ns,
                compute_monotonic_wall_ns=wall_ns,
                command_before=list(old_command),
            )
            try:
                source_preservation = node.publish_command(
                    duplicate_ordinary_hold=duplicate_context
                )
            finally:
                # The capability is only this stack-local publication. No node
                # cache can carry it into a later explicit stop or protection.
                duplicate_context.clear()
        else:
            node.publish_command()
        self.records += 1
        node.evidence.append(
            node.run / "navigation_control_clock_hold.jsonl",
            dict(
                schema="teacher_control_clock_exact_zero_hold/v1",
                sequence=self.records,
                compute_ros_clock_ns=clock_ns,
                compute_monotonic_wall=now,
                publish_ros_clock_ns=node.last_publication_ros_clock_ns,
                # PUBLICATION_OBSERVER_BEGIN actual_publication_link
                command_publication_sequence=getattr(node, "publication_records", None),
                # PUBLICATION_OBSERVER_END actual_publication_link
                previous_control_clock_ns=previous_clock,
                previous_actual_guard_clock_ns=previous_guard,
                last_clock_progress_monotonic_wall=None
                if self.last_progress_wall_ns is None
                else self.last_progress_wall_ns / 1e9,
                decision=decision,
                source_stale_reasons=stale,
                original_300ms_TTL_preserved=not minimal_baseline_enabled(node.profile),
                **(
                    {
                        "r73_upper_wall_last_observations": copy.deepcopy(
                            getattr(node, "_r73_upper_wall_last_observations", {})
                        )
                    }
                    if minimal_baseline_enabled(node.profile)
                    else {}
                ),
                controller_math_called=False,
                native_geometry_called=False,
                source_receipts_refreshed=False,
                native_guard_required_before_nonzero_resume=True,
                source_pose_stamp_ns=int(node.pose_stamp),
                source_cloud_stamp_ns=int(node.cloud_stamp),
                source_pose_received_monotonic_wall=node.pose_updated
                if math.isfinite(node.pose_updated)
                else None,
                source_cloud_received_monotonic_wall=node.cloud_updated
                if math.isfinite(node.cloud_updated)
                else None,
                request_id=node.request_id,
                waypoint_index=node.waypoint_index,
                command_before=old_command,
                command_after=list(node.command),
                state_after=node.state,
                protected_or_stale_reset=reset,
                frozen_control_before=before,
                frozen_control_after=self.snapshot(node),
                navigation_ground_truth_used=False,
                **(
                    {}
                    if source_preservation is None
                    else {"r85_ordinary_source_preservation": source_preservation}
                ),
            ),
        )
        return True
