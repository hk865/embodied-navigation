"""Teacher stop/heading transitions proven from fresh SLAM and raw body gyro.

No truth, timer-only standstill, CHAMP phase reset or IMU yaw localization.

Independent simulation experiment: allow profile turn caps up to 0.6 rad/s.
The NAV 0.6 cap has no physical validation yet. At cap 0.3 all transition
mathematics, measured-stop conditions and 300 ms freshness are unchanged.
"""

from collections import deque
import copy
import math
from numbers import Real
from embodied_navigation.control.parking.navigation_stop_window import NavigationStopWindow, SPEC
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled


def stance_support_handoff_configuration(profile):
    """Explicit experimental internal target; original business stop owner remains."""
    if "r85_stance_support_handoff" not in profile:
        return None
    value = profile["r85_stance_support_handoff"]
    if not isinstance(value, dict) or type(value.get("enabled")) is not bool:
        raise ValueError("r85_stance_support_handoff requires an explicit boolean enabled")
    if value["enabled"] is False:
        if set(value) != {"enabled"}:
            raise ValueError("Disabled stance support admits only enabled false")
        return None
    if set(value) != {"enabled", "kind"} or value["kind"] != "locked_heading_PD_stance_support_v1":
        raise ValueError("Unknown stance-support handoff configuration")
    cascade = profile.get("cascade", {})
    if (
        any(
            not isinstance(profile.get(key), dict)
            or set(profile[key]) != {"enabled"}
            or profile[key]["enabled"] is not required
            for key, required in (
                ("r85_active_turn_braking", True),
                ("r85_quiet_handoff_target", True),
                ("r85_neutral_rate_brake_capture", False),
            )
        )
        or not isinstance(cascade, dict)
        or cascade.get("external_heading_gate") is not True
        or cascade.get("r85_turn_brake_reference") != {"kind": "locked_heading_pd_v1"}
    ):
        raise ValueError(
            "Stance support requires active quiet locked-heading PD and explicit neutral off"
        )
    from embodied_navigation.control.parking.integral_unload import configuration

    if configuration(profile) is None:
        raise ValueError("Stance support requires original coordinated integral unload")
    return copy.deepcopy(value)


class TeacherHeadingGate:
    def __init__(self, profile):
        self.minimal_simulation_baseline = minimal_baseline_enabled(profile)
        self.max_turn_rate = profile["max_yaw_rate_radps"]
        if not math.isfinite(self.max_turn_rate) or not 0 < self.max_turn_rate <= 0.6:
            raise ValueError(
                "Simulation experimental turn limit must be finite and within (0,0.6]rad/s; NAV0.6 is not physically validated"
            )
        self.config = profile["teacher_transition"]
        self.history = deque(maxlen=64)
        self.neutral_rate_brake_capture_config = None
        if "r85_neutral_rate_brake_capture" in profile:
            from embodied_navigation.control.tracking.cascade_core import (
                neutral_rate_brake_capture_configuration,
            )

            self.neutral_rate_brake_capture_config = neutral_rate_brake_capture_configuration(
                profile
            )
        self.stance_support_handoff_config = stance_support_handoff_configuration(profile)
        self.stance_old_brake_history = deque(maxlen=64)
        self.stance_old_quiet_history = deque(maxlen=64)
        self.stance_old_capture_last_decision = None
        self.stance_old_capture_last_sample = None
        self.neutral_first_natural_zero_publication = None
        self.neutral_first_natural_zero_ack = None
        self.neutral_old_brake_history = deque(maxlen=64)
        self.neutral_old_quiet_history = deque(maxlen=64)
        self.neutral_old_capture_last_decision = None
        self.neutral_old_capture_last_sample = None
        braking = profile.get("r85_active_turn_braking", {"enabled": False})
        if (
            not isinstance(braking, dict)
            or set(braking) != {"enabled"}
            or type(braking.get("enabled")) is not bool
        ):
            raise ValueError("r85_active_turn_braking requires only explicit boolean enabled")
        self.active_braking_enabled = braking["enabled"]
        continuous = profile.get("ordinary_turn_control_handoff", {"enabled": False})
        if not isinstance(continuous, dict) or type(continuous.get("enabled")) is not bool:
            raise ValueError("Explicit ordinary turn control handoff configuration required")
        expected = (
            {"enabled": True, "schema": "ordinary_turn_continuous_control/v1"}
            if continuous["enabled"]
            else {"enabled": False}
        )
        if continuous != expected:
            raise ValueError("Unknown ordinary turn control handoff configuration")
        self.continuous_turn_handoff_enabled = continuous["enabled"]
        if self.continuous_turn_handoff_enabled and (
            self.minimal_simulation_baseline
            or self.active_braking_enabled
            or profile.get("cascade", {}).get("external_heading_gate") is not True
        ):
            raise ValueError(
                "Continuous ordinary handoff requires original formal external heading control"
            )
        self.continuous_turn_handoff_count = 0
        if self.minimal_simulation_baseline and self.active_braking_enabled:
            raise ValueError("R73 diagnostic baseline requires active braking disabled")
        self.active_braking_lower_ack_config = None
        self.active_braking_command_limits = None
        if self.active_braking_enabled:
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                configuration as lower_ack_configuration,
            )

            cascade = profile.get("cascade")
            if not isinstance(cascade, dict):
                raise ValueError("Active braking requires the original cascade ACK configuration")
            self.active_braking_lower_ack_config = lower_ack_configuration(
                cascade.get("r85_student_lower_loop_ack")
            )
            limits = cascade.get("command_limits", [0.8, 0.35, 0.8])
            if (
                self.active_braking_lower_ack_config is None
                or not isinstance(limits, (list, tuple))
                or len(limits) != 3
                or any(
                    type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits
                )
            ):
                raise ValueError(
                    "Active braking requires frozen Student lower ACK and command limits"
                )
            self.active_braking_command_limits = copy.deepcopy(limits)
        quiet = profile.get("r85_quiet_handoff_target", {"enabled": False})
        if (
            not isinstance(quiet, dict)
            or set(quiet) != {"enabled"}
            or type(quiet.get("enabled")) is not bool
        ):
            raise ValueError("r85_quiet_handoff_target requires only explicit boolean enabled")
        self.quiet_target_enabled = quiet["enabled"]
        self.integral_unload_config = None
        self.integral_unload_event = None
        self.integral_unload_previous_zero = None
        self.integral_unload_failed_event_id = None
        if profile.get("r85_pre_zero_integral_unload", {}).get("enabled") is True:
            from embodied_navigation.control.parking.integral_unload import configuration

            self.integral_unload_config = configuration(profile)
        if self.quiet_target_enabled and (
            not self.active_braking_enabled
            or profile.get("cascade", {}).get("external_heading_gate") is not True
            or profile.get("cascade", {}).get("r85_turn_brake_reference")
            != {"kind": "locked_heading_pd_v1"}
        ):
            raise ValueError("Quiet handoff target requires active locked-heading braking")
        # Half the original drift budgets over5s are conservative command
        # targets, not a claim that velocity commands guarantee physical drift.
        self.quiet_limits = dict(xy=0.05 / 2 / 5.0, yaw=0.1 / 2 / 5.0, heading=0.005)
        if self.neutral_rate_brake_capture_config is not None:
            self.quiet_limits["heading"] = self.config["turn_complete_rad"]
        self.quiet_command_yaw_cap = self.quiet_limits["yaw"]
        if self.stance_support_handoff_config is not None:
            self.quiet_limits["heading"] = 0.25 * self.config["turn_complete_rad"]
            self.quiet_command_yaw_cap = 0.035
        self.quiet_history = deque(maxlen=64)
        self.quiet_publication = None
        self.quiet_binding = None
        self.quiet_pose_context = None
        self.quiet_observed_stamp_ns = None
        self.quiet_ack_watermark = None
        self.quiet_reason = "not_observed"
        self.last_quiet_qualification = None
        self.brake_history = deque(maxlen=64)
        self.brake_last_stamp = None
        self.handoff_failure = None
        self.handoff_event = None
        self.handoff_deadline_s = None
        self.handoff_deadline_candidate = None
        self.handoff_deadline_binding = None
        self.zero_ack_candidate = None
        self.zero_ack_error = None
        self.zero_ack_highwater = None
        self.last_brake_qualification = None
        self.final_zero_ack = None
        diagnostic = profile.get("diagnostic_observe_only", {})
        self.diagnostic_profile = dict(
            diagnostic_observe_only=dict(
                enabled=bool(
                    isinstance(diagnostic, dict)
                    and type(diagnostic.get("enabled")) is bool
                    and diagnostic["enabled"]
                )
            )
        )
        self.diagnostic_run = None
        self.diagnostic_zero_ack_ready = False
        self.stop_window = NavigationStopWindow(
            profile.get("navigation_settle_statistics_contract"), self.config
        )
        self.transition_purpose = "unknown"
        self.transition_context = {}
        self.last_release = None
        self.zero_reconvergence_count = 0
        self.last_zero_reconvergence = None
        self.last_stamp = None
        self.latest_motion = None
        self.zero_handoff_count = 0
        self.reset()

    @staticmethod
    def error(target, yaw):
        return math.atan2(math.sin(target - yaw), math.cos(target - yaw))

    def reset(self):
        if self.handoff_active:
            self.fail_handoff("active_handoff_reset")
        if self.handoff_failure:
            self.phase = "failed"
            return
        self.handoff_event = None
        self.handoff_deadline_s = None
        if self.quiet_target_enabled:
            self._clear_quiet("reset")
            self.quiet_publication = None
            self.quiet_binding = None
            self.quiet_ack_watermark = None
            self.quiet_pose_context = None
        self.brake_history.clear()
        self.brake_last_stamp = None
        self.final_zero_ack = None
        self.phase = "pre_turn"
        self.heading = None
        self.settle_until = None
        self.outside_since = None
        self.stop_window.clear()
        self.last_release = None
        self.settle_control_stage = None
        self.last_zero_handoff = None
        # Goal/replan resets heading, not the real observed standstill history.

    def set_transition_context(self, purpose, context):
        changed = context != self.transition_context
        old_identity = (
            self.transition_context.get("request_id"),
            self.transition_context.get("waypoint_index"),
        )
        new_identity = (context.get("request_id"), context.get("waypoint_index"))
        was_active = self.handoff_active
        if was_active and (changed or purpose != self.transition_purpose):
            self.fail_handoff("active_handoff_binding_changed")
        elif old_identity != new_identity:
            self.handoff_failure = None
            self.handoff_event = None
            self.handoff_deadline_s = None
            self.handoff_deadline_candidate = None
            self.handoff_deadline_binding = None
            self.zero_ack_candidate = None
            self.zero_ack_highwater = None
            self.final_zero_ack = None
            self.brake_history.clear()
            self.brake_last_stamp = None
            if self.phase == "failed":
                self.phase = "pre_turn"
                self.heading = None
                self.settle_control_stage = None
        if purpose != self.transition_purpose or changed:
            self.stop_window.clear()
        self.transition_purpose = purpose
        self.transition_context = dict(context)

    @property
    def controlled_braking(self):
        return self.phase == "settle" and self.settle_control_stage == "controlled_braking"

    @property
    def handoff_active(self):
        return bool(
            self.active_braking_enabled
            and self.handoff_event is not None
            and self.handoff_event.get("status") == "pending"
        )

    def fail_handoff(self, reason, sim_time=None):
        if not self.handoff_active:
            return False
        self.handoff_failure = self.handoff_failure or str(reason)
        if self.integral_unload_config is not None and self.integral_unload_event is not None:
            self.integral_unload_failed_event_id = (
                self.integral_unload_failed_event_id or self.integral_unload_event["event_id"]
            )
        self.handoff_event.update(
            status="failed", failure=self.handoff_failure, failure_control_s=sim_time
        )
        self.phase = "failed"
        self.settle_control_stage = None
        self.history.clear()
        self.stop_window.clear()
        self.brake_history.clear()
        if self.quiet_target_enabled:
            self._clear_quiet("handoff_failed")
        return True

    def bind_integral_unload(self, run_id, scope, path_sha256, goal_sha256, clock_ns):
        if self.integral_unload_config is None:
            return None
        if not self.handoff_active or not self.controlled_braking:
            return None
        if self.integral_unload_failed_event_id is not None:
            self.fail_handoff("pre_zero_integral_unload_previous_failure", clock_ns / 1e9)
            return None
        from embodied_navigation.control.parking.integral_unload import make_event

        old = self.integral_unload_event
        # Same owned pending entry always keeps its original path identity,
        # expiry, and hash. A legal path refresh only restarts quiet samples.
        initial_path = (
            old["initial_path_sha256"]
            if old is not None
            and old["brake_entry_clock_ns"]
            == round(self.handoff_event["brake_entry_control_s"] * 1e9)
            else path_sha256
        )
        event = make_event(
            run_id,
            scope,
            self.handoff_event,
            initial_path,
            goal_sha256,
            self.zero_handoff_count,
            self.integral_unload_previous_zero,
        )
        if old is None:
            self.integral_unload_event = event
        elif event != old:
            from embodied_navigation.control.parking.integral_unload import successor

            if successor(old, event):
                self.integral_unload_event = event
            else:
                self.fail_handoff("pre_zero_integral_unload_binding_changed", clock_ns / 1e9)
                return None
        self.check_integral_unload_deadline(clock_ns)
        return copy.deepcopy(self.integral_unload_event) if self.handoff_active else None

    def check_integral_unload_deadline(self, clock_ns):
        event = self.integral_unload_event
        if (
            event is not None
            and self.handoff_active
            and self.controlled_braking
            and clock_ns >= event["expires_clock_ns"]
        ):
            self.fail_handoff("pre_zero_integral_unload_timeout", clock_ns / 1e9)

    def _unloaded_actual_pair(self, publication, ack):
        if self.integral_unload_config is None:
            return True
        try:
            from embodied_navigation.control.parking.integral_unload import validate_receipt

            e = self.integral_unload_event
            if e is None:
                return False
            teacher = validate_receipt(
                publication.get("pre_zero_integral_unload"),
                event=e,
                side="teacher",
                scope=publication["scope"],
                clock_ns=publication["control_clock_ns"],
            )
            native = ack.get("pre_zero_integral_unload")
            if (
                isinstance(native, dict)
                and native.get("kind") == "original_exact_zero_I_observation"
            ):
                from embodied_navigation.control.parking.integral_unload import (
                    validate_native_extension,
                )

                validate_native_extension(
                    native, ack["run_id"], ack["input_clock_ns"], ack["input_frame_serial"]
                )
                # This observes actual original reset state; it does not itself
                # authorize an event. Current published Teacher event and all
                # original causal/quiet source predicates still bind this pair.
                return bool(
                    teacher["yaw_integral_after"] == 0.0
                    and ack["exact_zero"]
                    and ack["input_clock_ns"] > e["brake_entry_clock_ns"]
                    and teacher["path_sha256"] == publication["path_sha256"]
                    and teacher["goal_sha256"] == publication["goal_sha256"]
                )
            native = validate_receipt(
                native,
                event=e,
                side="student",
                scope=ack["run_id"],
                clock_ns=ack["input_clock_ns"],
                frame=ack["input_frame_serial"],
            )
            return bool(
                teacher["yaw_integral_after"] == 0.0
                and native["yaw_integral_after"] == 0.0
                and teacher["path_sha256"] == publication["path_sha256"] == native["path_sha256"]
                and teacher["goal_sha256"] == publication["goal_sha256"] == native["goal_sha256"]
            )
        except (ValueError, KeyError, TypeError):
            return False

    def _post_entry_brake_heading_band(self):
        return (
            self.config["turn_complete_rad"]
            if self.neutral_rate_brake_capture_config is not None
            else 0.25 * self.config["turn_complete_rad"]
        )

    def _clear_quiet(self, reason):
        self.quiet_history.clear()
        self.quiet_reason = str(reason)
        if self.neutral_rate_brake_capture_config is not None:
            self.neutral_old_quiet_history.clear()
            self.neutral_old_brake_history.clear()
        if self.stance_support_handoff_config is not None:
            self.stance_old_quiet_history.clear()
            self.stance_old_brake_history.clear()

    @staticmethod
    def _quiet_binding(context, purpose, scope, path_id, path_sha256, goal_sha256):
        return dict(
            context=copy.deepcopy(context),
            purpose=purpose,
            scope=scope,
            path_id=path_id,
            path_sha256=path_sha256,
            goal_sha256=goal_sha256,
        )

    def _low_total_command(self, value, yaw_cap=None):
        return bool(
            isinstance(value, (list, tuple))
            and len(value) == 3
            and all(
                isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v) for v in value
            )
            and math.hypot(value[0], value[1]) <= self.quiet_limits["xy"]
            and abs(value[2]) <= (self.quiet_command_yaw_cap if yaw_cap is None else yaw_cap)
        )

    @staticmethod
    def _quiet_publication_fresh(record, clock_ns, wall_ns):
        return bool(
            all(
                0 <= clock_ns - record[k] < 300_000_000
                for k in (
                    "publish_clock_ns",
                    "control_clock_ns",
                    "source_pose_stamp_ns",
                    "paired_imu_stamp_ns",
                )
            )
            and all(
                0 <= wall_ns - record[k] < 300_000_000
                for k in (
                    "publish_wall_ns",
                    "control_wall_ns",
                    "source_pose_received_wall_ns",
                    "source_imu_received_wall_ns",
                )
            )
        )

    def note_quiet_publication(self, record):
        if not self.quiet_target_enabled:
            return False
        self.quiet_publication = None
        try:
            if not (
                self.controlled_braking
                and self.handoff_active
                and isinstance(record, dict)
                and record["normal_braking"] is True
                and self._low_total_command(record["command_body"])
                and record["context"] == self.transition_context
                and record["purpose"] == self.transition_purpose
                and record["context"] == self.handoff_event["context"]
            ):
                raise ValueError("publication_not_quiet_brake_math")
            clocks = (
                "sequence",
                "publish_clock_ns",
                "publish_wall_ns",
                "control_clock_ns",
                "control_wall_ns",
                "source_pose_stamp_ns",
                "paired_imu_stamp_ns",
                "source_pose_received_wall_ns",
                "source_imu_received_wall_ns",
            )
            if any(type(record[k]) is not int or record[k] < 0 for k in clocks):
                raise ValueError("publication_integer_clocks_required")
            if not (
                self._quiet_publication_fresh(
                    record, record["publish_clock_ns"], record["publish_wall_ns"]
                )
                and 0
                <= record["source_pose_stamp_ns"] - record["paired_imu_stamp_ns"]
                <= 20_000_000
                and record["control_clock_ns"] / 1e9 >= self.handoff_event["brake_entry_control_s"]
            ):
                raise ValueError("publication_source_not_fresh")
            binding = self._quiet_binding(
                *(
                    record[k]
                    for k in (
                        "context",
                        "purpose",
                        "scope",
                        "path_id",
                        "path_sha256",
                        "goal_sha256",
                    )
                )
            )
            if not (
                isinstance(record["path_id"], str)
                and record["path_id"]
                and all(
                    isinstance(record[k], str) and len(record[k]) == 64
                    for k in ("scope", "path_sha256", "goal_sha256")
                )
            ):
                raise ValueError("publication_source_binding_missing")
            if self.quiet_binding is not None and binding != self.quiet_binding:
                if any(
                    binding[k] != self.quiet_binding[k]
                    for k in ("context", "purpose", "scope", "goal_sha256")
                ):
                    raise ValueError("publication_source_binding_changed")
                # A legitimate same-goal SCAN path refresh restarts only this
                # internal target window, never the handoff event or budget.
                self._clear_quiet("publication_path_refreshed")
            if self.integral_unload_config is not None:
                from embodied_navigation.control.parking.integral_unload import validate_receipt

                proof = validate_receipt(
                    record.get("pre_zero_integral_unload"),
                    event=self.integral_unload_event,
                    side="teacher",
                    scope=record["scope"],
                    clock_ns=record["control_clock_ns"],
                )
                if (
                    proof["yaw_integral_after"] != 0.0
                    or record.get("yaw_integral_state") != proof["yaw_integral_after"]
                    or record.get("integral_dt_s") != proof["dt_s"]
                ):
                    raise ValueError("actual_published_Teacher_I_not_zero")
            self.quiet_binding = copy.deepcopy(binding)
            self.quiet_publication = copy.deepcopy(record)
            # A high support publication between poses must interrupt the old
            # strict shadow even if a later low publication precedes the next pose.
            if self.stance_support_handoff_config is not None and not self._low_total_command(
                record["command_body"], 0.01
            ):
                self.stance_old_quiet_history.clear()
            if (
                self.neutral_rate_brake_capture_config is not None
                and not any(record["command_body"])
                and self.neutral_first_natural_zero_publication is None
            ):
                self.neutral_first_natural_zero_publication = copy.deepcopy(record)
            return True
        except (ValueError, TypeError, KeyError) as error:
            self._clear_quiet(error)
            return False

    def begin_quiet_pose(
        self,
        stamp_ns,
        control_clock_ns,
        wall_ns,
        context,
        purpose,
        scope,
        path_id,
        path_sha256,
        goal_sha256,
    ):
        if not self.quiet_target_enabled:
            return
        self.quiet_observed_stamp_ns = None
        self.quiet_pose_context = dict(
            stamp_ns=stamp_ns,
            clock_ns=control_clock_ns,
            wall_ns=wall_ns,
            binding=self._quiet_binding(context, purpose, scope, path_id, path_sha256, goal_sha256),
        )

    def finish_quiet_pose(self, stamp_ns, accepted):
        if self.quiet_target_enabled and accepted and self.quiet_observed_stamp_ns != stamp_ns:
            self._clear_quiet("accepted_pose_without_qualified_motion_observation")

    def _observe_quiet(self, stamp_s, wall_s, brake_valid, observed_yaw):
        stamp_ns = round(stamp_s * 1e9)
        self.quiet_observed_stamp_ns = stamp_ns
        p = self.quiet_publication
        obs = self.quiet_pose_context
        ack = self.zero_ack_candidate
        m = self.latest_motion
        try:
            if not (
                brake_valid
                and p is not None
                and obs is not None
                and ack is not None
                and obs["stamp_ns"] == stamp_ns
                and obs["binding"] == self.quiet_binding
                and p["context"] == self.transition_context
                and p["purpose"] == self.transition_purpose
                and abs(self.error(self.heading, observed_yaw)) <= self.quiet_limits["heading"]
                and abs(m["slam_wz"]) <= self.quiet_limits["yaw"]
                and abs(m["body_gyro"][2]) <= self.quiet_limits["yaw"]
            ):
                raise ValueError("motion_heading_or_publication_not_quiet")
            if not (
                self._quiet_publication_fresh(p, obs["clock_ns"], round(wall_s * 1e9))
                and 0 <= stamp_ns - ack["input_clock_ns"] < 300_000_000
                and 0 <= round(wall_s * 1e9) - ack["received_wall_ns"] < 300_000_000
                and ack["run_id"] == p["scope"]
                and (ack["request_id"], ack["waypoint_index"])
                == (
                    self.transition_context.get("request_id"),
                    self.transition_context.get("waypoint_index"),
                )
                and ack["input_clock_ns"] > round(self.handoff_event["brake_entry_control_s"] * 1e9)
                and self._low_total_command(ack["requested_command_body"])
                and self._low_total_command(ack["applied_command_body"])
            ):
                raise ValueError("actual_commands_or_causal_ACK_not_quiet")
            if not self._unloaded_actual_pair(p, ack):
                raise ValueError("actual_same_event_integrals_not_zero")
            watermark = tuple(
                ack[k]
                for k in ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
            )
            if self.quiet_ack_watermark is not None and any(
                a <= b for a, b in zip(watermark, self.quiet_ack_watermark)
            ):
                raise ValueError("quiet_ACK_frame_not_independent")
            self.quiet_ack_watermark = watermark
            if (
                self.quiet_history
                and stamp_s - self.quiet_history[-1]["stamp_s"] > self.config["stop_pose_gap_max_s"]
            ):
                self._clear_quiet("quiet_pose_gap")
            self.quiet_history.append(
                dict(
                    stamp_s=stamp_s,
                    wall_s=wall_s,
                    publication_sequence=p["sequence"],
                    ack_watermark=list(watermark),
                    heading_error_rad=self.error(self.heading, observed_yaw),
                    slam_wz=m["slam_wz"],
                    body_wz=m["body_gyro"][2],
                )
            )
            self.quiet_reason = None
            if self.neutral_rate_brake_capture_config is not None:
                error = self.quiet_history[-1]["heading_error_rad"]
                if abs(error) <= 0.25 * self.config["turn_complete_rad"]:
                    self.neutral_old_brake_history.append((stamp_s, wall_s))
                else:
                    self.neutral_old_brake_history.clear()
                if abs(error) <= 0.005:
                    self.neutral_old_quiet_history.append(copy.deepcopy(self.quiet_history[-1]))
                else:
                    self.neutral_old_quiet_history.clear()
                self.neutral_old_capture_last_sample.update(
                    source_and_ACK_qualified=True,
                    ACK_watermark=list(watermark),
                    old_brake_instantaneous=abs(error) <= 0.25 * self.config["turn_complete_rad"],
                    old_quiet_instantaneous=abs(error) <= 0.005,
                )
            if self.stance_support_handoff_config is not None:
                sample = copy.deepcopy(self.quiet_history[-1])
                error = sample["heading_error_rad"]
                old_commands = all(
                    self._low_total_command(command, 0.01)
                    for command in (
                        p["command_body"],
                        ack["requested_command_body"],
                        ack["applied_command_body"],
                    )
                )
                old_brake = abs(error) <= 0.25 * self.config["turn_complete_rad"]
                old_quiet = abs(error) <= 0.005 and old_commands
                if old_brake:
                    self.stance_old_brake_history.append((stamp_s, wall_s))
                else:
                    self.stance_old_brake_history.clear()
                if old_quiet:
                    self.stance_old_quiet_history.append(sample)
                else:
                    self.stance_old_quiet_history.clear()
                self.stance_old_capture_last_sample.update(
                    source_and_ACK_qualified=True,
                    ACK_watermark=list(watermark),
                    old_brake_instantaneous=old_brake,
                    old_quiet_instantaneous=old_quiet,
                    old_command_caps_passed=old_commands,
                    published_command_body=list(p["command_body"]),
                    ACK_requested_command_body=list(ack["requested_command_body"]),
                    ACK_applied_command_body=list(ack["applied_command_body"]),
                )
        except (ValueError, TypeError, KeyError) as error:
            self._clear_quiet(error)

    def _quiet_qualified(self, sim_time, wall_time, error):
        if not self.quiet_target_enabled:
            return True
        h = self.quiet_history
        p = self.quiet_publication
        ack = self.zero_ack_candidate
        clock_ns = round(sim_time * 1e9)
        wall_ns = round(wall_time * 1e9)
        if not (
            h
            and p is not None
            and ack is not None
            and abs(error) <= self.quiet_limits["heading"]
            and h[-1]["stamp_s"] == self.latest_motion["stamp_s"]
            and 0 <= wall_time - h[-1]["wall_s"] < 0.3
            and -0.05 <= sim_time - h[-1]["stamp_s"] < 0.3
            and self._quiet_publication_fresh(p, clock_ns, wall_ns)
            and 0 <= clock_ns - ack["input_clock_ns"] < 300_000_000
            and 0 <= wall_ns - ack["received_wall_ns"] < 300_000_000
            and ack["ack_native_clock_ns"] <= clock_ns
            and ack["input_clock_ns"] <= round(self.latest_motion["stamp_s"] * 1e9)
            and [
                ack[k]
                for k in ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
            ]
            == h[-1]["ack_watermark"]
            and ack["run_id"] == p["scope"]
            and (ack["request_id"], ack["waypoint_index"])
            == (
                self.transition_context.get("request_id"),
                self.transition_context.get("waypoint_index"),
            )
            and p["context"] == self.transition_context
            and p["purpose"] == self.transition_purpose
            and self._low_total_command(p["command_body"])
            and self._low_total_command(ack["requested_command_body"])
            and self._low_total_command(ack["applied_command_body"])
        ):
            self._clear_quiet("quiet_target_not_current")
            return False
        if not self._unloaded_actual_pair(p, ack):
            self._clear_quiet("actual_same_event_integrals_not_zero")
            return False
        ready = (
            len(h) >= self.config["stop_min_pose_samples"]
            and h[-1]["stamp_s"] - h[0]["stamp_s"] >= self.config["stop_window_sim_s"] - 1e-9
        )
        if ready:
            self.last_quiet_qualification = dict(
                first_stamp_s=h[0]["stamp_s"],
                last_stamp_s=h[-1]["stamp_s"],
                samples=len(h),
                actual_control_s=sim_time,
                last_publication=copy.deepcopy(p),
                last_pose=copy.deepcopy(h[-1]),
                low_total_command_target_only=True,
                original_stop_PASS=False,
            )
        return ready

    def _record_neutral_old_capture_decision(
        self, sim_time, wall_time, error, new_brake_ready, new_quiet_ready
    ):
        if self.neutral_rate_brake_capture_config is None:
            return
        b = self.neutral_old_brake_history
        q = self.neutral_old_quiet_history
        c = self.config
        latest = self.latest_motion["stamp_s"] if self.latest_motion is not None else None
        old_brake = bool(
            new_brake_ready
            and abs(error) <= 0.25 * c["turn_complete_rad"]
            and len(b) >= c["stop_min_pose_samples"]
            and b[-1][0] == latest
            and b[-1][0] - b[0][0] >= c["stop_window_sim_s"] - 1e-9
        )
        old_quiet = bool(
            new_quiet_ready
            and abs(error) <= 0.005
            and len(q) >= c["stop_min_pose_samples"]
            and q[-1]["stamp_s"] == latest
            and q[-1]["stamp_s"] - q[0]["stamp_s"] >= c["stop_window_sim_s"] - 1e-9
        )
        self.neutral_old_capture_last_decision = dict(
            control_s=sim_time,
            wall_s=wall_time,
            locked_heading_error_rad=error,
            new_brake_ready=bool(new_brake_ready),
            new_quiet_ready=bool(new_quiet_ready),
            old_brake_ready=old_brake,
            old_quiet_ready=old_quiet,
            old_internal_capture_ready=old_brake and old_quiet,
            diagnostic_only=True,
            ACK_watermark_consumed_again=False,
            affects_new_decision=False,
        )

    def _record_stance_old_capture_decision(
        self, sim_time, wall_time, error, new_brake_ready, new_quiet_ready
    ):
        if self.stance_support_handoff_config is None:
            return
        b = self.stance_old_brake_history
        q = self.stance_old_quiet_history
        c = self.config
        latest = self.latest_motion["stamp_s"] if self.latest_motion is not None else None
        old_brake = bool(
            new_brake_ready
            and abs(error) <= 0.25 * c["turn_complete_rad"]
            and len(b) >= c["stop_min_pose_samples"]
            and b[-1][0] == latest
            and b[-1][0] - b[0][0] >= c["stop_window_sim_s"] - 1e-9
        )
        old_quiet = bool(
            new_quiet_ready
            and abs(error) <= 0.005
            and len(q) >= c["stop_min_pose_samples"]
            and q[-1]["stamp_s"] == latest
            and q[-1]["stamp_s"] - q[0]["stamp_s"] >= c["stop_window_sim_s"] - 1e-9
        )
        self.stance_old_capture_last_decision = dict(
            control_s=sim_time,
            wall_s=wall_time,
            locked_heading_error_rad=error,
            new_brake_ready=bool(new_brake_ready),
            new_quiet_ready=bool(new_quiet_ready),
            old_brake_ready=old_brake,
            old_quiet_ready=old_quiet,
            old_internal_capture_ready=old_brake and old_quiet,
            diagnostic_only=True,
            ACK_watermark_consumed_again=False,
            affects_new_decision=False,
        )

    def bind_handoff_deadline(self, deadline_s, binding):
        if not self.active_braking_enabled:
            return
        if (
            isinstance(deadline_s, bool)
            or not isinstance(deadline_s, (int, float))
            or not math.isfinite(deadline_s)
            or deadline_s < 0
            or not isinstance(binding, dict)
        ):
            raise ValueError("Original finite absolute handoff deadline and binding required")
        bound = copy.deepcopy(binding)
        identity = (bound.get("request_id"), bound.get("waypoint_index"))
        expected = (
            self.transition_context.get("request_id"),
            self.transition_context.get("waypoint_index"),
        )
        if identity != expected:
            if self.handoff_active:
                self.fail_handoff("active_handoff_binding_changed")
            self.handoff_deadline_candidate = None
            return
        if self.handoff_active:
            if bound != self.handoff_event["deadline_binding"]:
                self.fail_handoff("active_handoff_binding_changed")
                return
            # Rebinding may tighten the frozen original deadline, never extend it.
            self.handoff_deadline_s = min(self.handoff_deadline_s, float(deadline_s))
            self.handoff_event["deadline_s"] = self.handoff_deadline_s
        self.handoff_deadline_candidate = float(deadline_s)
        self.handoff_deadline_binding = bound

    def bind_zero_ack(
        self, ack, expected_scope, request_id, waypoint_index, control_clock_ns, wall_ns
    ):
        self.zero_ack_candidate = None
        self.zero_ack_error = None
        try:
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import input_binding

            receipt, stamp, frame, inference, applied, scope = input_binding(ack)
            if self.active_braking_enabled:
                from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                    residual as lower_ack_residual,
                )

                # Final exact-zero uses core hold, whose protective early return
                # precedes its normal ACK checks. Reuse that same v1 validator.
                lower_ack_residual(
                    ack, self.active_braking_lower_ack_config, self.active_braking_command_limits
                )
            if (
                type(control_clock_ns) is not int
                or type(wall_ns) is not int
                or type(ack["sequence"]) is not int
                or ack["sequence"] <= 0
                or type(ack["received_wall_ns"]) is not int
                or type(ack["ack_native_clock_ns"]) is not int
            ):
                raise ValueError("Original ACK/control integer clocks and sequence required")
            context = (request_id, waypoint_index)
            expected = (
                self.transition_context.get("request_id"),
                self.transition_context.get("waypoint_index"),
            )
            if context != expected or scope != expected_scope:
                raise ValueError("ACK current context or frozen source scope differs")
            if (
                not 0 <= control_clock_ns - stamp < 300_000_000
                or not 0 <= wall_ns - ack["received_wall_ns"] < 300_000_000
                or ack["ack_native_clock_ns"] > control_clock_ns
                or ack["ack_native_clock_ns"] - stamp != 20_000_000
            ):
                raise ValueError("ACK stale, future, or causal native interval differs")
            request = list(ack["requested_command_body"])
            if len(request) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in request
            ):
                raise ValueError("Finite truthful ACK request required")
            watermark = dict(
                sequence=ack["sequence"],
                input_clock_ns=stamp,
                input_frame_serial=frame,
                inference_sequence=inference,
            )
            old = self.zero_ack_highwater
            if old is not None:
                keys = ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
                if any(watermark[k] < old[k] for k in keys):
                    raise ValueError("Actual ACK watermark went backwards")
                if watermark["sequence"] > old["sequence"] and any(
                    watermark[k] <= old[k] for k in keys[1:]
                ):
                    raise ValueError("New ACK reused an old actual inference frame")
                if watermark["sequence"] == old["sequence"] and watermark != old:
                    raise ValueError("Repeated ACK sequence changed its actual frame")
            self.zero_ack_highwater = copy.deepcopy(watermark)
            self.zero_ack_candidate = dict(
                **watermark,
                run_id=scope,
                request_id=request_id,
                waypoint_index=waypoint_index,
                received_wall_ns=ack["received_wall_ns"],
                ack_native_clock_ns=ack["ack_native_clock_ns"],
                requested_command_body=request,
                applied_command_body=list(applied),
                exact_zero=not any(request) and not any(applied),
            )
            if self.integral_unload_config is not None:
                extension = receipt.get("pre_zero_integral_unload")
                if extension is not None:
                    self.zero_ack_candidate["pre_zero_integral_unload"] = copy.deepcopy(extension)
            if (
                self.neutral_rate_brake_capture_config is not None
                and self.controlled_braking
                and self.handoff_active
                and self.zero_ack_candidate["exact_zero"]
                and stamp > round(self.handoff_event["brake_entry_control_s"] * 1e9)
                and self.neutral_first_natural_zero_ack is None
            ):
                self.neutral_first_natural_zero_ack = copy.deepcopy(self.zero_ack_candidate)
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.zero_ack_error = str(error)
            return False

    def note_zero_publication(self, clock_ns):
        if (
            not self.handoff_active
            or self.settle_control_stage != "authoritative_zero"
            or self.last_zero_handoff is None
        ):
            return False
        if type(clock_ns) is not int or clock_ns < round(
            self.last_zero_handoff["actual_control_s"] * 1e9
        ):
            raise ValueError("Final zero publication precedes its control edge")
        if self.last_zero_handoff.get("zero_publication_clock_ns") is None:
            self.last_zero_handoff["zero_publication_clock_ns"] = clock_ns
            self.handoff_event["zero_publication_clock_ns"] = clock_ns
        return True

    def _post_handoff_zero_ack(self, stamp_s, wall_s):
        edge = self.last_zero_handoff
        ack = self.zero_ack_candidate
        if not self.handoff_active or edge is None or ack is None or not ack["exact_zero"]:
            return False
        publication = edge.get("zero_publication_clock_ns")
        old = edge.get("ack_watermark")
        context = edge["context"]
        if (
            publication is None
            or old is None
            or ack["input_clock_ns"] <= publication
            or ack["input_clock_ns"] > round(stamp_s * 1e9)
            or not 0 <= wall_s - ack["received_wall_ns"] / 1e9 < 0.3
            or ack["run_id"] != edge.get("run_id")
            or ack["request_id"] != context.get("request_id")
            or ack["waypoint_index"] != context.get("waypoint_index")
            or any(
                ack[k] <= old[k]
                for k in ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
            )
        ):
            return False
        if self.final_zero_ack is None:
            self.final_zero_ack = copy.deepcopy(ack)
        return True

    def _begin_braking(self, sim_time):
        binding = self.handoff_deadline_binding
        if (
            self.handoff_deadline_candidate is None
            or binding is None
            or (binding.get("request_id"), binding.get("waypoint_index"))
            != (
                self.transition_context.get("request_id"),
                self.transition_context.get("waypoint_index"),
            )
        ):
            self.handoff_event = dict(status="pending")
            self.fail_handoff("active_handoff_deadline_missing", sim_time)
            return False
        self.handoff_deadline_s = self.handoff_deadline_candidate
        self.handoff_event = dict(
            status="pending",
            brake_entry_control_s=sim_time,
            deadline_s=self.handoff_deadline_s,
            deadline_binding=copy.deepcopy(binding),
            context=copy.deepcopy(self.transition_context),
            purpose=self.transition_purpose,
        )
        self.brake_history.clear()
        self.brake_last_stamp = self.last_stamp
        self.history.clear()
        self.stop_window.clear()
        self.last_release = None
        self.settle_control_stage = "controlled_braking"
        if self.neutral_rate_brake_capture_config is not None:
            self.neutral_first_natural_zero_publication = None
            self.neutral_first_natural_zero_ack = None
            self.neutral_old_brake_history.clear()
            self.neutral_old_quiet_history.clear()
            self.neutral_old_capture_last_decision = None
            self.neutral_old_capture_last_sample = None
        if self.stance_support_handoff_config is not None:
            self.stance_old_brake_history.clear()
            self.stance_old_quiet_history.clear()
            self.stance_old_capture_last_decision = None
            self.stance_old_capture_last_sample = None
        if self.quiet_target_enabled:
            self._clear_quiet("brake_entry")
            self.quiet_publication = None
            self.quiet_binding = None
            self.last_quiet_qualification = None
            self.quiet_ack_watermark = (
                tuple(
                    self.zero_ack_highwater[k]
                    for k in (
                        "sequence",
                        "input_clock_ns",
                        "input_frame_serial",
                        "inference_sequence",
                    )
                )
                if self.zero_ack_highwater is not None
                else None
            )
        if sim_time > self.handoff_deadline_s:
            self.fail_handoff("active_handoff_deadline_exceeded", sim_time)
            return False
        return True

    def _braking_qualified(self, sim_time, wall_time, error, center_band):
        m = self.latest_motion
        c = self.config
        if self.neutral_rate_brake_capture_config is not None:
            center_band = self._post_entry_brake_heading_band()
        valid = abs(error) <= center_band and self._motion_ready_for_zero(sim_time, wall_time)
        if not valid:
            self.brake_history.clear()
            return False
        return bool(
            len(self.brake_history) >= c["stop_min_pose_samples"]
            and self.brake_history[-1][0] - self.brake_history[0][0]
            >= c["stop_window_sim_s"] - 1e-9
        )

    def observe(
        self,
        stamp_s,
        wall_s,
        velocity,
        slam_wz,
        body_gyro,
        gyro_stamp_s,
        zero_command,
        position=None,
        quaternion=None,
        *,
        positive_qualification_allowed=True,
    ):
        values = list(velocity) + [stamp_s, wall_s, slam_wz, gyro_stamp_s] + list(body_gyro)
        if len(velocity) != 3 or len(body_gyro) != 3 or not all(math.isfinite(x) for x in values):
            return False
        if self.last_stamp is not None and stamp_s <= self.last_stamp:
            return False
        self.last_stamp = stamp_s
        c = self.config
        self.latest_motion = dict(
            stamp_s=stamp_s,
            wall_s=wall_s,
            velocity=list(velocity),
            slam_wz=slam_wz,
            body_gyro=list(body_gyro),
            gyro_stamp_s=gyro_stamp_s,
            zero_command=bool(zero_command),
            positive_qualification_allowed=positive_qualification_allowed is True,
        )
        # Delayed input from another command/request scope remains a motion
        # observation but cannot seed new brake/quiet/zero qualification.
        zero_command = bool(zero_command and positive_qualification_allowed)
        # Every accepted header participates: a bad sample cannot disappear
        # when another odometry callback arrives before the control callback.
        if self.controlled_braking:
            self.brake_last_stamp = stamp_s
            q = list(quaternion or [])
            qnorm = (
                sum(v * v for v in q) if len(q) == 4 and all(math.isfinite(v) for v in q) else 0.0
            )
            heading_valid = False
            observed_yaw = None
            if 0.98 <= qnorm <= 1.02:
                x, y, z, w = [v / math.sqrt(qnorm) for v in q]
                observed_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                heading_valid = (
                    abs(self.error(self.heading, observed_yaw))
                    <= self._post_entry_brake_heading_band()
                )
            brake_valid = (
                positive_qualification_allowed
                and self._motion_ready_for_zero(stamp_s, wall_s)
                and heading_valid
                and self.handoff_event is not None
                and stamp_s > self.handoff_event["brake_entry_control_s"]
            )
            if (
                not brake_valid
                or self.brake_history
                and stamp_s - self.brake_history[-1][0] > c["stop_pose_gap_max_s"]
            ):
                self.brake_history.clear()
            if brake_valid:
                self.brake_history.append((stamp_s, wall_s))
            if self.quiet_target_enabled:
                if self.neutral_rate_brake_capture_config is not None:
                    error = None if observed_yaw is None else self.error(self.heading, observed_yaw)
                    self.neutral_old_capture_last_sample = dict(
                        pose_stamp_s=stamp_s,
                        wall_s=wall_s,
                        locked_heading_error_rad=error,
                        new_brake_motion_valid=bool(brake_valid),
                        source_and_ACK_qualified=False,
                        old_brake_instantaneous=False,
                        old_quiet_instantaneous=False,
                    )
                if self.stance_support_handoff_config is not None:
                    error = None if observed_yaw is None else self.error(self.heading, observed_yaw)
                    self.stance_old_capture_last_sample = dict(
                        pose_stamp_s=stamp_s,
                        wall_s=wall_s,
                        locked_heading_error_rad=error,
                        new_brake_motion_valid=bool(brake_valid),
                        source_and_ACK_qualified=False,
                        old_brake_instantaneous=False,
                        old_quiet_instantaneous=False,
                        old_command_caps_passed=False,
                    )
                self._observe_quiet(stamp_s, wall_s, brake_valid, observed_yaw)
        # Brake output (including natural exact zero) is never parking evidence.
        if self.active_braking_enabled and self.handoff_failure:
            zero_command = False
        if self.active_braking_enabled and self.phase == "settle":
            if self.settle_control_stage in ("controlled_convergence", "controlled_braking"):
                zero_command = False
            elif (
                self.settle_control_stage == "authoritative_zero" and self.handoff_event is not None
            ):
                zero_command = bool(zero_command and self._post_handoff_zero_ack(stamp_s, wall_s))
        self.latest_motion["zero_command"] = bool(zero_command)
        self.stop_window.observe(
            stamp_s,
            wall_s,
            velocity,
            slam_wz,
            body_gyro,
            gyro_stamp_s,
            zero_command,
            position,
            quaternion,
        )
        valid = (
            zero_command
            and math.hypot(*velocity[:2]) <= c["stop_planar_speed_mps"]
            and abs(slam_wz) <= c["stop_slam_yaw_rate_radps"]
            and math.sqrt(sum(x * x for x in body_gyro)) <= c["stop_body_gyro_radps"]
            and abs(stamp_s - gyro_stamp_s) <= c["gyro_pose_pair_max_s"]
        )
        if not valid or self.history and stamp_s - self.history[-1][0] > c["stop_pose_gap_max_s"]:
            self.history.clear()
        if valid:
            self.history.append((stamp_s, wall_s))
        return valid

    @property
    def controlled_settle(self):
        return self.phase == "settle" and self.settle_control_stage == "controlled_convergence"

    def _ordinary_handoff_enabled(self):
        # Existing dynamic/tilt/unknown safety holds retain their exact-zero path.
        return (
            self.transition_purpose in SPEC["eligible_purposes"]
            and isinstance(self.transition_context.get("request_id"), str)
            and type(self.transition_context.get("waypoint_index")) is int
            and self.transition_context.get("dynamic_recovery") is False
        )

    def _motion_ready_for_zero(self, sim_time, wall_time):
        # Same instantaneous limits/freshness as the original measured-stop
        # predicate, with no claim of zero-command dwell before the handoff.
        m = self.latest_motion
        c = self.config
        return bool(
            m is not None
            and 0 <= wall_time - m["wall_s"] < 0.3
            and -0.05 <= sim_time - m["stamp_s"] < 0.3
            and abs(m["stamp_s"] - m["gyro_stamp_s"]) <= c["gyro_pose_pair_max_s"]
            and math.hypot(*m["velocity"][:2]) <= c["stop_planar_speed_mps"]
            and abs(m["slam_wz"]) <= c["stop_slam_yaw_rate_radps"]
            and math.sqrt(sum(x * x for x in m["body_gyro"])) <= c["stop_body_gyro_radps"]
        )

    def _observe_stop_wait(self, sim_time, wall_time, phase):
        config = self.diagnostic_profile.get("diagnostic_observe_only", {})
        if not (
            isinstance(config, dict)
            and type(config.get("enabled")) is bool
            and config["enabled"]
            and self.diagnostic_run is not None
            and self.diagnostic_zero_ack_ready is True
            and self._motion_ready_for_zero(sim_time, wall_time)
            and self.latest_motion.get("zero_command") is True
        ):
            return False
        from embodied_navigation.common.observation import api as _observer_api

        record = _observer_api("diagnostic_observation").record
        record(
            self.diagnostic_run,
            self.diagnostic_profile,
            "teacher_heading_" + phase,
            "original_measured_stop_business_wait",
            dict(
                actual_control_s=sim_time,
                original_legacy_stop_passed=False,
                statistics=self.stop_window.evidence(sim_time, wall_time),
                actual_fresh_zero_actor_ack=True,
                fresh_instantaneous_stop_limits_passed=True,
                zero_command=True,
                control_heading_math_unchanged=True,
            ),
        )
        return True

    def stopped(self, sim_time, wall_time):
        c = self.config
        return (
            len(self.history) >= c["stop_min_pose_samples"]
            and 0 <= wall_time - self.history[-1][1] < 0.3
            and -0.05 <= sim_time - self.history[-1][0] < 0.3
            and self.history[-1][0] - self.history[0][0] >= c["stop_window_sim_s"] - 1e-9
        )

    def resume_after_stop(self, yaw, fresh_path_heading, stopped_duration):
        # Caller retains unchanged cloud-clearance/new-SCAN guard. A duration
        # alone never establishes physical stopping for the Teacher.
        self.reset()
        self.transition_purpose = "dynamic_recovery"
        self.heading = fresh_path_heading
        return False

    def update(self, sim_time, yaw, current_path_heading, wall_time=None):
        import time

        wall_time = time.monotonic() if wall_time is None else wall_time
        c = self.config
        if self.minimal_simulation_baseline and not all(
            math.isfinite(v) for v in (sim_time, yaw, current_path_heading, wall_time)
        ):
            raise ValueError("Finite actual diagnostic heading and clocks required")
        current_error = self.error(current_path_heading, yaw)
        if self.handoff_failure:
            return False, 0.0
        if self.handoff_active and sim_time > self.handoff_deadline_s:
            self.fail_handoff("active_handoff_deadline_exceeded", sim_time)
            return False, 0.0
        if self.phase == "drive":
            if abs(current_error) <= c["drive_heading_max_rad"]:
                return True, max(-0.08, min(0.08, 0.5 * current_error))
            self.reset()  # Stop immediately before a substantial new heading.
            if self.transition_purpose not in ("dynamic_recovery", "tilt_recovery"):
                self.transition_purpose = "heading_change"
        if self.phase == "pre_turn":
            if (
                not self.minimal_simulation_baseline
                and not self.stopped(sim_time, wall_time)
                and not self._observe_stop_wait(sim_time, wall_time, "pre_turn")
            ):
                return False, 0.0
            self.phase = "align"
            self.heading = current_path_heading
        if self.heading is None:
            self.heading = current_path_heading
        error = self.error(self.heading, yaw)
        if self.phase == "align":
            if abs(error) > c["turn_complete_rad"]:
                return False, max(-self.max_turn_rate, min(self.max_turn_rate, 1.3 * error))
            if self.minimal_simulation_baseline:
                self.phase = "drive"
                self.heading = None
                self.settle_control_stage = None
                self.last_release = dict(
                    predicate="r73_geometric_heading_only_diagnostic",
                    actual_control_s=sim_time,
                    original_measured_stop_qualified=False,
                    original_quiet_handoff_qualified=False,
                    formal_acceptance_pass=False,
                )
                return True, max(-0.08, min(0.08, 0.5 * current_error))
            self.phase = "settle"
            self.stop_window.clear()
            self.last_release = None
            self.settle_control_stage = (
                "controlled_convergence"
                if self._ordinary_handoff_enabled()
                else "authoritative_zero"
            )
        if self.phase == "settle":
            if abs(current_error) > c["drive_heading_max_rad"]:
                if self.handoff_active:
                    self.fail_handoff("active_handoff_heading_rearm", sim_time)
                    return False, 0.0
                self.phase = "pre_turn"
                self.heading = None
                self.settle_control_stage = None
                self.stop_window.clear()
                self.last_release = None
                return False, 0.0
            if (
                self.settle_control_stage == "authoritative_zero"
                and not self.active_braking_enabled
                and self._ordinary_handoff_enabled()
                and abs(error) > c["turn_complete_rad"]
            ):
                # The completed turn was lost while waiting at zero. Resume the
                # same locked target before drive release; old zero evidence
                # cannot qualify another attempt with yaw control resumed.
                self.zero_reconvergence_count += 1
                self.last_zero_reconvergence = dict(
                    reason="turn_complete_band_lost_during_authoritative_zero",
                    actual_control_s=sim_time,
                    source_pose_stamp_s=self.last_stamp,
                    locked_heading_rad=self.heading,
                    locked_heading_error_rad=error,
                    original_turn_complete_rad=c["turn_complete_rad"],
                    discarded_stop_samples=len(self.history),
                )
                self.history.clear()
                self.stop_window.clear()
                self.last_release = None
                self.last_zero_handoff = None
                self.final_zero_ack = None
                self.zero_ack_candidate = None
                self.last_brake_qualification = None
                self.last_quiet_qualification = None
                self._clear_quiet("heading_reconvergence")
                self.quiet_publication = None
                self.quiet_binding = None
                self.quiet_pose_context = None
                self.quiet_ack_watermark = None
                self.settle_control_stage = "controlled_convergence"
            if self.controlled_settle:
                # This is a changed control handoff: keep the existing cascade
                # yaw PD/PI and Student correction active toward the SAME locked
                # center. It is not a new acceptance threshold or a timed wait.
                # The center target is inside the original 0.1-rad band to leave
                # room for the observed ~0.11-rad Actor zero-input recoil.
                center_band = 0.25 * c["turn_complete_rad"]
                if abs(error) > center_band or not self._motion_ready_for_zero(sim_time, wall_time):
                    return False, max(-self.max_turn_rate, min(self.max_turn_rate, 1.3 * error))
                if (
                    self.continuous_turn_handoff_enabled
                    and self._ordinary_handoff_enabled()
                    and self.transition_purpose in ("path_rebind", "heading_change")
                ):
                    if self.latest_motion.get("positive_qualification_allowed") is not True:
                        return False, max(-self.max_turn_rate, min(self.max_turn_rate, 1.3 * error))
                    # Ordinary motion continues through the existing checked PID
                    # drive path. No open-loop zero edge or standstill claim is
                    # needed between two active control modes.
                    self.continuous_turn_handoff_count += 1
                    self.last_release = dict(
                        predicate="ordinary_turn_continuous_control/v1",
                        actual_control_s=sim_time,
                        locked_heading_rad=self.heading,
                        locked_heading_error_rad=error,
                        fresh_motion=dict(self.latest_motion),
                        purpose=self.transition_purpose,
                        context=dict(self.transition_context),
                        legacy_measured_stop=False,
                        statistical_stop=False,
                        zero_command_dwell=False,
                        parking_acceptance=False,
                    )
                    self.history.clear()
                    self.stop_window.clear()
                    self.phase = "drive"
                    self.heading = None
                    self.settle_control_stage = None
                    return True, max(-0.08, min(0.08, 0.5 * current_error))
                if self.active_braking_enabled:
                    self._begin_braking(sim_time)
                    return False, 0.0
                self.settle_control_stage = "authoritative_zero"
                self.zero_handoff_count += 1
                self.history.clear()
                self.stop_window.clear()
                self.last_release = None
                self.last_zero_handoff = dict(
                    actual_control_s=sim_time,
                    locked_heading_rad=self.heading,
                    locked_heading_error_rad=error,
                    center_target_band_rad=center_band,
                    fresh_motion=dict(self.latest_motion),
                    zero_command_dwell_begins_after_this_edge=True,
                )
                return False, 0.0
            if self.controlled_braking:
                center_band = self._post_entry_brake_heading_band()
                brake_ready = self._braking_qualified(sim_time, wall_time, error, center_band)
                if not brake_ready:
                    self._record_neutral_old_capture_decision(
                        sim_time, wall_time, error, False, False
                    )
                    self._record_stance_old_capture_decision(
                        sim_time, wall_time, error, False, False
                    )
                    return False, 0.0
                quiet_ready = self._quiet_qualified(sim_time, wall_time, error)
                self._record_neutral_old_capture_decision(
                    sim_time, wall_time, error, brake_ready, quiet_ready
                )
                self._record_stance_old_capture_decision(
                    sim_time, wall_time, error, brake_ready, quiet_ready
                )
                if not quiet_ready:
                    return False, 0.0
                if self.zero_ack_candidate is None:
                    return False, 0.0  # No final edge without a real frozen Actor watermark.
                self.last_brake_qualification = dict(
                    actual_control_s=sim_time,
                    first_stamp_s=self.brake_history[0][0],
                    last_stamp_s=self.brake_history[-1][0],
                    samples=len(self.brake_history),
                    original_stop_pass=False,
                    center_target_band_rad=center_band,
                    locked_heading_error_rad=error,
                )
                self.settle_control_stage = "authoritative_zero"
                self.zero_handoff_count += 1
                if self.integral_unload_config is not None:
                    self.integral_unload_previous_zero = dict(
                        event_id=self.integral_unload_event["event_id"],
                        zero_control_clock_ns=round(sim_time * 1e9),
                        zero_handoff_count=self.zero_handoff_count,
                    )
                self.history.clear()
                self.stop_window.clear()
                self.last_release = None
                self.final_zero_ack = None
                self.last_zero_handoff = dict(
                    actual_control_s=sim_time,
                    locked_heading_rad=self.heading,
                    locked_heading_error_rad=error,
                    center_target_band_rad=center_band,
                    fresh_motion=dict(self.latest_motion),
                    zero_command_dwell_begins_after_this_edge=True,
                    zero_publication_clock_ns=None,
                    ack_watermark=copy.deepcopy(self.zero_ack_highwater),
                    run_id=self.zero_ack_candidate["run_id"],
                    context=copy.deepcopy(self.transition_context),
                    deadline_s=self.handoff_deadline_s,
                )
                self.handoff_event.update(final_zero_control_s=sim_time)
                return False, 0.0
            legacy = self.stopped(sim_time, wall_time)
            stats = self.stop_window.evidence(sim_time, wall_time)
            eligible = (
                self.transition_purpose in SPEC["eligible_purposes"]
                and isinstance(self.transition_context.get("request_id"), str)
                and type(self.transition_context.get("waypoint_index")) is int
                and self.transition_context.get("dynamic_recovery") is False
            )
            statistical = bool(eligible and stats["enabled"] and stats["passed"])
            observed = (
                not self.handoff_active
                and not legacy
                and not statistical
                and self._observe_stop_wait(sim_time, wall_time, "settle")
            )
            if not legacy and not statistical and not observed:
                return False, 0.0
            self.last_release = dict(
                predicate=(
                    "diagnostic_observe_only_original_stop_not_passed"
                    if observed
                    else "legacy_continuous_300ms"
                    if legacy
                    else SPEC["schema"]
                ),
                legacy_measured_stop=legacy,
                statistical_stop=statistical,
                purpose=self.transition_purpose,
                context=dict(self.transition_context),
                actual_control_s=sim_time,
                statistics=stats,
            )
            if self.handoff_active:
                self.handoff_event.update(status="passed", release_control_s=sim_time)
            self.phase = "drive"
            self.heading = None
            return True, max(-0.08, min(0.08, 0.5 * current_error))
        return False, 0.0

    def evidence(self, sim_time, wall_time):
        result = {
            "transition": "teacher_measured_stop_heading_v1",
            "phase": self.phase,
            "measured_stop": self.stopped(sim_time, wall_time),
            "stop_valid_pose_samples": len(self.history),
            "stop_first_stamp_s": self.history[0][0] if self.history else None,
            "stop_last_stamp_s": self.history[-1][0] if self.history else None,
            "navigation_heading_source": "SLAM body yaw only",
            "gyro_use": "stop/tilt only; never localization",
            "transition_purpose": self.transition_purpose,
            "transition_context": dict(self.transition_context),
            "navigation_settle_statistics": self.stop_window.evidence(sim_time, wall_time),
            "last_settle_release": self.last_release,
            "settle_control_stage": self.settle_control_stage,
            "controlled_settle": self.controlled_settle,
            "controlled_braking": self.controlled_braking,
            "active_braking_enabled": self.active_braking_enabled,
            "handoff_active": self.handoff_active,
            "handoff_failure": self.handoff_failure,
            "handoff_event": copy.deepcopy(self.handoff_event),
            "brake_valid_pose_samples": len(self.brake_history),
            "last_brake_qualification": copy.deepcopy(self.last_brake_qualification),
            "final_zero_ack": copy.deepcopy(self.final_zero_ack),
            "zero_ack_error": self.zero_ack_error,
            "zero_handoff_count": self.zero_handoff_count,
            "last_zero_handoff": self.last_zero_handoff,
            "zero_reconvergence_count": self.zero_reconvergence_count,
            "last_zero_reconvergence": self.last_zero_reconvergence,
            "continuous_turn_handoff_enabled": self.continuous_turn_handoff_enabled,
            "continuous_turn_handoff_count": self.continuous_turn_handoff_count,
            "settle_center_target_band_rad": 0.25 * self.config["turn_complete_rad"],
            "acceptance_thresholds_changed": False,
        }
        if self.neutral_rate_brake_capture_config is not None:
            result["neutral_rate_brake_capture"] = dict(
                configuration=copy.deepcopy(self.neutral_rate_brake_capture_config),
                new_internal_capture_changed=True,
                original_business_acceptance_unchanged=True,
                acceptance_scope="Original teacher_transition fields, source/motion/ACK and terminal fixed5 owner; R16 internal capture is changed.",
                brake_entry_heading_band_rad=0.25 * self.config["turn_complete_rad"],
                post_entry_capture_heading_band_rad=self._post_entry_brake_heading_band(),
                locked_heading_rad=self.heading,
                locked_heading_target_moved=False,
                original_R16_internal_capture_claimed=False,
                old_internal_capture_shadow=dict(
                    diagnostic_only=True,
                    brake_heading_band_rad=0.25 * self.config["turn_complete_rad"],
                    quiet_heading_band_rad=0.005,
                    brake_samples=len(self.neutral_old_brake_history),
                    brake_first_stamp_s=self.neutral_old_brake_history[0][0]
                    if self.neutral_old_brake_history
                    else None,
                    brake_last_stamp_s=self.neutral_old_brake_history[-1][0]
                    if self.neutral_old_brake_history
                    else None,
                    quiet_samples=len(self.neutral_old_quiet_history),
                    quiet_first_stamp_s=self.neutral_old_quiet_history[0]["stamp_s"]
                    if self.neutral_old_quiet_history
                    else None,
                    quiet_last_stamp_s=self.neutral_old_quiet_history[-1]["stamp_s"]
                    if self.neutral_old_quiet_history
                    else None,
                    last_control_evaluation=copy.deepcopy(self.neutral_old_capture_last_decision),
                    last_pose_observation=copy.deepcopy(self.neutral_old_capture_last_sample),
                    old_numeric_conditions_on_new_control_trajectory=True,
                    old_R16_control_counterfactual_reproduced=False,
                    same_validated_source_and_ACK_samples=True,
                    independent_ACK_consumption=False,
                    new_decision_affected=False,
                ),
                heading_error_band_is_not_relative_T0_drift=True,
                first_qualified_brake_zero_publication=copy.deepcopy(
                    self.neutral_first_natural_zero_publication
                ),
                first_observed_validated_brake_zero_ack=copy.deepcopy(
                    self.neutral_first_natural_zero_ack
                ),
                natural_zero_witness_scope="First validator-qualified publication or selected ACK observed here; not earliest actual wire zero in the full stream.",
                natural_zero_is_not_formal_T0=True,
                natural_zero_adds_stop_evidence=False,
                terminal_fixed5_owner_unchanged=True,
            )
        if self.stance_support_handoff_config is not None:
            b = self.stance_old_brake_history
            q = self.stance_old_quiet_history
            result["stance_support_handoff"] = dict(
                configuration=copy.deepcopy(self.stance_support_handoff_config),
                new_internal_capture_changed=True,
                original_business_acceptance_unchanged=True,
                reference_selection="restored locked_heading_pd_v1; neutral rate explicitly off",
                reference_restored_from_R34_neutral=True,
                core_controller_Student_sources_unchanged=True,
                existing_PD_PI_formulas_unchanged=True,
                locked_heading_rad=self.heading,
                goal_or_locked_target_moved=False,
                formal_T0_moved=False,
                original_R16_internal_capture_claimed=False,
                internal_limits=dict(
                    brake_entry_heading_rad=0.25 * self.config["turn_complete_rad"],
                    post_entry_brake_heading_rad=self._post_entry_brake_heading_band(),
                    quiet_heading_rad=self.quiet_limits["heading"],
                    command_yaw_cap_radps=self.quiet_command_yaw_cap,
                    motion_SLAM_and_body_yaw_cap_radps=self.quiet_limits["yaw"],
                    command_XY_norm_cap_mps=self.quiet_limits["xy"],
                ),
                old_internal_limits=dict(
                    brake_heading_rad=0.25 * self.config["turn_complete_rad"],
                    quiet_heading_rad=0.005,
                    command_yaw_cap_radps=0.01,
                    motion_SLAM_and_body_yaw_cap_radps=0.01,
                ),
                old_strict_shadow=dict(
                    diagnostic_only=True,
                    brake_samples=len(b),
                    quiet_samples=len(q),
                    current_publication_old_command_quiet=bool(
                        self.quiet_publication is not None
                        and self._low_total_command(self.quiet_publication["command_body"], 0.01)
                    ),
                    brake_first_stamp_s=b[0][0] if b else None,
                    brake_last_stamp_s=b[-1][0] if b else None,
                    quiet_first_stamp_s=q[0]["stamp_s"] if q else None,
                    quiet_last_stamp_s=q[-1]["stamp_s"] if q else None,
                    last_control_evaluation=copy.deepcopy(self.stance_old_capture_last_decision),
                    last_pose_observation=copy.deepcopy(self.stance_old_capture_last_sample),
                    history_scope="Subset of the same primary-qualified source/ACK samples, not an independent full old brake history.",
                    old_numeric_conditions_on_candidate_trajectory=True,
                    old_control_counterfactual_reproduced=False,
                    same_validated_source_and_ACK_samples=True,
                    independent_ACK_consumption=False,
                    new_decision_affected=False,
                ),
                physical_stance_support_proven=False,
                post_turn_fixed5_success_claimed=False,
                absolute_heading_and_relative_T0_drift_are_distinct=True,
                natural_wire_zero_diagnosed_from_original_stream=True,
                terminal_fixed5_owner_unchanged=True,
            )
        if self.quiet_target_enabled:
            h = self.quiet_history
            result["quiet_handoff_target"] = dict(
                enabled=True,
                target="low_total_command_only",
                limits=copy.deepcopy(self.quiet_limits),
                samples=len(h),
                first_stamp_s=h[0]["stamp_s"] if h else None,
                last_stamp_s=h[-1]["stamp_s"] if h else None,
                reason=self.quiet_reason,
                last_qualification=copy.deepcopy(self.last_quiet_qualification),
                original_stop_PASS=False,
                physical_drift_guaranteed=False,
            )
        if self.integral_unload_config is not None:
            result["pre_zero_integral_unload"] = dict(
                enabled=True,
                configuration=copy.deepcopy(self.integral_unload_config),
                event=copy.deepcopy(self.integral_unload_event),
                previous_owned_zero_identity=copy.deepcopy(self.integral_unload_previous_zero),
                failed_event_id=self.integral_unload_failed_event_id,
                new_deployment_math=True,
                stop_PASS=False,
                event_hash_is_publication_proof=False,
            )
        return result
