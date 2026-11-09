"""Explicit sim2sim Student45/GRU adapter; no ROS, actuator or physics owner.

Importing this module does not load torch or a model. The legacy loop retains
joint targets, bootstrap, previous raw action, all-static safety and IPC.
"""

import json
from pathlib import Path

import numpy as np
import embodied_navigation.robot.unitree.policy.speed_tiers as speed_tiers
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled

QDEFAULT = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])
PID_SPEC = dict(
    kp=[0.5, 0.5, 0.5],
    ki=[0.2, 0.2, 0.15],
    kd=[0.02, 0.02, 0.015],
    integral_bounds=[0.15, 0.4, 0.2],
    derivative_filter_tau_s=0.08,
    correction_bounds=[0.08, 0.1, 0.08],
)
RESET_SCHEMA = "student_velocity_pid_motion_epoch_zero_reset/v2"


def exact_student45(state, command, previous_action):
    """Saved no-clip/no-noise Student terms, same native request and raw history."""
    state, command, previous_action = (
        np.asarray(v, dtype=float) for v in (state, command, previous_action)
    )
    if (
        state.shape != (64,)
        or command.shape != (3,)
        or previous_action.shape != (12,)
        or not np.isfinite(np.r_[state, command, previous_action]).all()
    ):
        raise ValueError("Student native state64/command3/raw_action12 invalid")
    q = state[4:8]
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("Student quaternion invalid")
    w, x, y, z = q / norm
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    out = np.r_[
        state[11:14] * 0.2,
        R.T @ np.array([0.0, 0.0, -1.0]),
        command,
        state[14:26] - QDEFAULT,
        state[26:38] * 0.05,
        previous_action,
    ].astype(np.float32)
    if out.shape != (45,) or not np.isfinite(out).all():
        raise ValueError("Student45 nonfinite")
    return out


class CorrectionOnlyPID:
    """Original P/I/D math with selected B capacity/reset; one legacy worker slew."""

    def __init__(self, contract, navigation_limits, *, speed_tier=None):
        for key in ("kp", "ki", "kd", "integral_bounds", "correction_bounds"):
            values = np.asarray(contract["pid"][key], dtype=float)
            if values.shape != (3,) or not np.isfinite(values).all() or np.any(values < 0):
                raise ValueError("PID finite nonnegative vectors required")
        if not 0 < float(contract["pid"]["derivative_filter_tau_s"]) <= 1.0:
            raise ValueError("PID derivative filter invalid")
        for key, maximum in [
            ("command_bounds", [0.3, 0.2, 0.3]),
            ("command_slew_acceleration", [0.6, 0.6, 0.8]),
        ]:
            values = np.asarray(contract[key], dtype=float)
            if (
                values.shape != (3,)
                or not np.isfinite(values).all()
                or np.any(values <= 0)
                or np.any(values > maximum)
            ):
                raise ValueError("Student command safety envelope invalid")
        limits = np.asarray(navigation_limits, dtype=float)
        if limits.shape != (3,) or not np.isfinite(limits).all() or np.any(limits <= 0):
            raise ValueError("Navigation limiter invalid")
        self.speed_tier = None if speed_tier is None else speed_tiers.validate(speed_tier)
        self.limits = np.minimum(limits, np.asarray(speed_tiers.student_bounds(self.speed_tier)))
        self.cfg = contract["pid"]
        self.integral = np.zeros(3)
        self.filtered_derivative = np.zeros(3)
        self.previous_measured = None
        self.previous_requested = None
        self.previous_stamp = None
        self.epoch = None
        self.new_reset_count = 0
        self.old_would_reset_count = 0
        self.frame_count = 0
        self.last_diagnostic = None
        self.yaw_pd_fade = None
        self.yaw_measurement_feedback = None

    def update(
        self,
        stamp,
        requested,
        measured,
        motion_epoch,
        authoritative_zero,
        pre_zero_unload=None,
        pre_zero_pd_fade_frame=None,
    ):
        requested, measured = (np.asarray(v, dtype=float) for v in (requested, measured))
        if (
            requested.shape != (3,)
            or measured.shape != (3,)
            or not np.isfinite(np.r_[stamp, requested, measured]).all()
        ):
            raise ValueError("Invalid PID native clock/request/COM feedback")
        dt = 0.02 if self.previous_stamp is None else stamp - self.previous_stamp
        if not 0.019999 <= dt <= 0.020001:
            raise ValueError("PID requires fresh 50Hz native state")
        if type(authoritative_zero) is not bool:
            raise ValueError("Typed zero state required")
        holding = not np.any(requested)
        if authoritative_zero != holding:
            raise ValueError("Zero authority differs from accepted request")
        if not holding and (not isinstance(motion_epoch, str) or not motion_epoch):
            raise ValueError("Nonzero request needs explicit accepted motion epoch")
        old_reset = (
            self.previous_requested is None
            or not np.array_equal(requested, self.previous_requested)
            or holding
        )
        reasons = []
        if self.previous_stamp is None:
            reasons.append("run_start")
        if not holding and motion_epoch != self.epoch:
            reasons.append("accepted_request_waypoint_epoch_change")
        if authoritative_zero:
            reasons.append("authoritative_exact_zero")
        new_reset = bool(reasons)
        if new_reset:
            self.integral[:] = 0.0
            self.filtered_derivative[:] = 0.0
            self.previous_measured = measured.copy()
            self.new_reset_count += 1
        if old_reset:
            self.old_would_reset_count += 1
        self.previous_stamp = stamp
        if not holding:
            self.epoch = motion_epoch
        error = requested - measured
        derivative = -(measured - self.previous_measured) / dt
        alpha = dt / (self.cfg["derivative_filter_tau_s"] + dt)
        self.filtered_derivative += alpha * (derivative - self.filtered_derivative)
        kp, ki, kd = (np.asarray(self.cfg[k]) for k in ("kp", "ki", "kd"))
        fade_diagnostic = None
        if self.yaw_pd_fade is not None:
            frame = pre_zero_pd_fade_frame
            if (
                not isinstance(frame, dict)
                or set(frame) != {"input_clock_ns", "input_frame_serial"}
                or type(frame["input_clock_ns"]) is not int
                or frame["input_clock_ns"] != round((float(stamp) + 0.005) * 1e9)
            ):
                raise ValueError("Fade requires the actual native input clock/frame")
            fade_diagnostic = self.yaw_pd_fade.step(
                dt,
                frame["input_clock_ns"],
                frame["input_frame_serial"],
                motion_epoch,
                authoritative_zero,
                pre_zero_unload,
            )
            kp, kd = (np.asarray(v, dtype=float) for v in self.yaw_pd_fade.gain_vectors(kp, kd))
            fade_diagnostic.update(
                base_yaw_Kp=0.5,
                base_yaw_Kd=0.015,
                effective_yaw_Kp=float(kp[2]),
                effective_yaw_Kd=float(kd[2]),
                Ki_unchanged=float(ki[2]),
                base_PID_math_claim=False,
            )
        ib, cb = (np.asarray(self.cfg[k]) for k in ("integral_bounds", "correction_bounds"))
        candidate = np.clip(self.integral + error * dt, -ib, ib)
        unload_before = None
        if pre_zero_unload is not None and not holding:
            helper = pre_zero_unload["helper"]
            authorization = pre_zero_unload["authorization"]
            helper.validate_event(
                authorization["event"], clock_ns=authorization["source_control_clock_ns"]
            )
            unload_before = float(self.integral[2])
            candidate[2] = helper.toward_zero(unload_before, 0.04, dt)
        p, d = kp * error, kd * self.filtered_derivative
        measurement_diagnostic = None
        if self.yaw_measurement_feedback is not None:
            frame = pre_zero_pd_fade_frame
            if not isinstance(frame, dict):
                raise ValueError("Measurement P requires actual native frame")
            replacement, measurement_diagnostic = self.yaw_measurement_feedback.compute(
                dt,
                frame["input_clock_ns"],
                frame["input_frame_serial"],
                motion_epoch,
                authoritative_zero,
                requested,
                measured,
                fade_diagnostic,
                pre_zero_unload,
                0.0 if holding else float(p[2]),
            )
            # Outside the current event do not even rewrite the original term.
            if measurement_diagnostic["retention_active"]:
                p[2] = replacement
        raw_correction = p + ki * candidate + d
        correction = np.clip(raw_correction, -cb, cb)
        raw_command = requested + correction
        clipped = np.clip(raw_command, -self.limits, self.limits)
        blocked = error * ((raw_correction - correction) + (raw_command - clipped)) > 1e-12
        candidate = np.where(blocked, self.integral, candidate)
        if unload_before is not None:
            # Release an existing state even if accumulation would be blocked.
            # This is the explicit new recurrence, not an unchanged PID claim.
            candidate[2] = helper.toward_zero(unload_before, 0.04, dt)
        self.integral = candidate if not holding else np.zeros(3)
        raw_correction = p + ki * self.integral + d if not holding else np.zeros(3)
        correction = np.clip(raw_correction, -cb, cb)
        corrected = np.clip(requested + correction, -self.limits, self.limits)
        self.previous_measured = measured.copy()
        self.previous_requested = requested.copy()
        self.frame_count += 1
        diagnostic = dict(
            schema=RESET_SCHEMA,
            feedback_source="privileged Gazebo native COM body vx/vy and body angular wz; sim2sim correction only",
            motion_epoch=motion_epoch,
            authoritative_zero=authoritative_zero,
            dt_s=dt,
            requested_command_body=requested.tolist(),
            measured_native_com_body=measured.tolist(),
            pid_error=error.tolist(),
            pid_integral=self.integral.tolist(),
            pid_P=p.tolist() if not holding else [0.0, 0.0, 0.0],
            pid_I=(ki * self.integral).tolist(),
            pid_D=d.tolist() if not holding else [0.0, 0.0, 0.0],
            pid_raw_correction=raw_correction.tolist(),
            pid_correction=correction.tolist(),
            corrected_before_legacy_slew_body=corrected.tolist(),
            effective_command_bounds=self.limits.tolist(),
            antiwindup_blocked_axes=blocked.tolist() if not holding else [False] * 3,
            command_saturated_axes=(np.abs(requested + correction - corrected) > 1e-12).tolist(),
            pid_correction_saturated_axes=(np.abs(raw_correction - correction) > 1e-12).tolist(),
            reset_reasons=reasons,
            new_reset=new_reset,
            old_request_change_would_reset=old_reset,
            new_reset_count=self.new_reset_count,
            old_would_reset_count=self.old_would_reset_count,
            worker_slew_owner="original legacy prepare_velocity_input",
            pid_has_slew_state=False,
        )
        if unload_before is not None:
            diagnostic[helper.KEY] = helper.receipt(
                authorization["event"],
                "student",
                unload_before,
                self.integral[2],
                dt,
                pre_zero_unload["input_clock_ns"],
                authorization["path_sha256"],
                authorization["goal_sha256"],
                pre_zero_unload["input_frame_serial"],
                authorization["source_control_clock_ns"],
            )
        if fade_diagnostic is not None:
            diagnostic["student_yaw_pd_fade"] = fade_diagnostic
        if measurement_diagnostic is not None:
            diagnostic[self.yaw_measurement_feedback.KEY] = measurement_diagnostic
            diagnostic[self.yaw_measurement_feedback.REQUIRED] = True
        if self.speed_tier is not None:
            diagnostic[speed_tiers.KEY] = dict(self.speed_tier)
        self.last_diagnostic = diagnostic
        return corrected, diagnostic

    def synchronize_authorized_zero(self):
        self.integral[:] = 0.0
        self.filtered_derivative[:] = 0.0
        if self.last_diagnostic is not None:
            self.last_diagnostic["ordinary_zero_after_legacy_slew_synchronized"] = True
            self.last_diagnostic["synchronized_pid_integral"] = [0.0, 0.0, 0.0]
        # There is no secondary PID command/slew cache to restore on resume.


def baseline_student_options(profile):
    """Select the existing base PID; never silently enable optional new laws."""
    selected = minimal_baseline_enabled(profile)
    if not selected:
        return False
    for key in (
        "r85_pre_zero_integral_unload",
        "r85_student_yaw_pd_fade",
        "r85_student_yaw_measurement_p_retention",
    ):
        option = profile.get(key)
        if option is not None and not (
            isinstance(option, dict) and set(option) == {"enabled"} and option["enabled"] is False
        ):
            raise ValueError("R73 base Student PID requires disabled " + key)
    ack = profile.get("cascade", {}).get("r85_student_lower_loop_ack")
    base = dict(enabled=True)
    if not isinstance(ack, dict):
        raise ValueError("R73 requires actual base Student ACK ledger")
    ack, speed_tier = speed_tiers.split_ack(ack)
    speed_tiers.configuration(profile)
    if "xy_stop_takeover_enabled" in ack:
        if ack.pop("xy_stop_takeover_enabled") is not False:
            raise ValueError("R73 XY stop takeover must be disabled")
    if ack.get("enabled") is not True:
        raise ValueError("Actual base Student ACK ledger required")
    return True


class StudentRuntime:
    def __init__(self, run, scope, binding, navigation_limits):
        self.minimal_baseline = baseline_student_options(scope["profile"])
        self.run = Path(run)
        self.scope = scope
        self.binding = binding
        self.contract_path = Path(binding["student_contract_snapshot"])
        self.model_path = Path(binding["student_export_snapshot"])
        from embodied_navigation.common.config import read

        self.contract = read()["student_runtime"]
        if (
            self.contract.get("actor_kind") != "student"
            or self.contract.get("observation_dimension") != 45
            or self.contract.get("recurrent_shape") != [1, 1, 256]
            or self.contract.get("action_scale") != 0.25
            or self.contract.get("action_clip") is not None
            or self.contract.get("policy_step_s") != 0.02
            or self.contract.get("physics_step_s") != 0.005
            or self.contract.get("native_response_deadline_ms") != 200
            or self.contract.get("source_freshness_ms") != 300
            or self.contract.get("observation_clip") is not None
            or self.contract.get("noise") is not False
        ):
            raise ValueError("Student model shape, timing, or input settings invalid")
        self.speed_tier = speed_tiers.configuration(scope["profile"])
        self.pid = CorrectionOnlyPID(self.contract, navigation_limits, speed_tier=self.speed_tier)
        self.unload_helper = None
        self.integral_unload = None
        option = scope["profile"].get("r85_pre_zero_integral_unload")
        if option is not None and not (
            option == {"enabled": False} and type(option.get("enabled")) is bool
        ):
            # Verify the real archive mapping BEFORE executing adjacent helper.
            import importlib.util

            helper_path = Path(__file__).with_name("pre_zero_integral_unload.py")
            expected_snapshot = Path(binding["student_adapter_snapshot"]).with_name(
                helper_path.name
            )
            helper_origin = Path(binding["student_adapter_origin"]).with_name(helper_path.name)
            expected_sha = scope["references"].get(str(helper_origin))
            scope_bytes = (self.run / "runtime_context.json").read_bytes()
            spec = importlib.util.spec_from_file_location(
                "actual_snapshot_pre_zero_integral_unload", helper_path
            )
            helper = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(helper)
            if helper.configuration(scope["profile"]) is not None:
                self.unload_helper = helper
                self.integral_unload = helper.NativeUnload(scope, None, self.run.name)
                self.integral_unload_identity = dict(
                    schema="r85_coordinated_unload_deployment_extension/v1",
                    helper_path=str(helper_path),
                    helper_source=str(helper_path),
                    scope_sha256=None,
                    configuration=helper.configuration(scope["profile"]),
                    base_contract_path=str(self.contract_path),
                    base_contract_does_not_prove_new_recurrence=True,
                )
        self.native_state = None
        self.obs45 = None
        self.frame_clock_ns = None
        self.inferred = False
        self.frame_serial = 0
        self.inference_count = 0
        self.hidden = None
        self.model = None
        self.teacher_slice_difference = None
        self.input_receipt = None
        self.actual_action = None
        self.stream = None
        self.configure_yaw_pd_fade_candidate()
        self.configure_yaw_measurement_p_retention_candidate()
        self.xy_stop_takeover = None
        self.configure_xy_stop_takeover_candidate()

    def configure_xy_stop_takeover_candidate(self):
        """New mode, only after actual adjacent helper/source/scope binding."""
        ack_option = (
            self.scope["profile"].get("cascade", {}).get("r85_student_lower_loop_ack") or {}
        )
        if "xy_stop_takeover_enabled" not in ack_option:
            return False
        option = ack_option["xy_stop_takeover_enabled"]
        if option is False:
            return False
        if type(option) is not bool:
            raise ValueError("XY stop mode flag must be an explicit bool")
        if self.frame_serial or self.pid.frame_count or self.pid.previous_stamp is not None:
            raise ValueError("XY stop mode configuration only before original first input")
        if self.unload_helper is None or self.integral_unload is None:
            raise ValueError("XY stop mode requires original bound owned-prezero authority")
        import importlib.util

        helper_path = Path(__file__).with_name("student_xy_stop_takeover.py")
        expected_snapshot = Path(self.binding["student_adapter_snapshot"]).with_name(
            helper_path.name
        )
        origin = Path(self.binding["student_adapter_origin"]).with_name(helper_path.name)
        expected_sha = self.scope["references"].get(str(origin))
        scope_bytes = (self.run / "runtime_context.json").read_bytes()
        spec = importlib.util.spec_from_file_location(
            "actual_snapshot_student_xy_stop_takeover", helper_path
        )
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        identity = dict(
            origin_path=str(origin), snapshot_path=str(helper_path), sha256=expected_sha
        )
        self.xy_stop_takeover = helper.XYStopTakeover(
            helper.configuration(self.scope["profile"]),
            self.unload_helper,
            self.integral_unload.run_id,
            self.integral_unload.scope_sha,
            identity,
        )
        self.xy_stop_takeover_identity = dict(
            schema="r49_actual_student_XY_stop_mode_extension/v1",
            **identity,
            scope_sha256=self.integral_unload.scope_sha,
            configuration=helper.configuration(self.scope["profile"]),
            new_control_mode=True,
            base_contract_path=str(self.contract_path),
            base_contract_does_not_prove_new_mode=True,
            stop_or_ACK_credit=False,
        )
        return True

    def configure_yaw_measurement_p_retention_candidate(self):
        key = "r85_student_yaw_measurement_p_retention"
        option = self.scope["profile"].get(key)
        if key not in self.scope["profile"] or (
            isinstance(option, dict) and set(option) == {"enabled"} and option["enabled"] is False
        ):
            return False
        if self.frame_serial or self.pid.frame_count or self.pid.previous_stamp is not None:
            raise ValueError("Measurement P configuration only before original first input")
        if (
            self.unload_helper is None
            or self.integral_unload is None
            or self.pid.yaw_pd_fade is None
        ):
            raise ValueError("Original accepted unload and yaw PD fade required")
        import importlib.util

        helper_path = Path(__file__).with_name("student_yaw_measurement_feedback.py")
        expected_snapshot = Path(self.binding["student_adapter_snapshot"]).with_name(
            helper_path.name
        )
        origin = Path(self.binding["student_adapter_origin"]).with_name(helper_path.name)
        expected_sha = self.scope["references"].get(str(origin))
        scope_bytes = (self.run / "runtime_context.json").read_bytes()
        spec = importlib.util.spec_from_file_location(
            "actual_snapshot_student_yaw_measurement_feedback", helper_path
        )
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        config = helper.configuration(self.scope["profile"])
        identity = dict(
            origin_path=str(origin), snapshot_path=str(helper_path), sha256=expected_sha
        )
        self.pid.yaw_measurement_feedback = helper.MeasurementPRetention(
            config,
            self.unload_helper,
            self.integral_unload.run_id,
            self.integral_unload.scope_sha,
            identity,
        )
        self.yaw_measurement_feedback_identity = dict(
            schema="r85_actual_yaw_measurement_P_extension/v1",
            **identity,
            scope_sha256=self.integral_unload.scope_sha,
            configuration=config,
            new_two_degree_of_freedom_P=True,
            base_contract_path=str(self.contract_path),
            base_contract_does_not_prove_new_P=True,
            stop_or_ACK_credit=False,
        )
        return True

    def configure_yaw_pd_fade_candidate(self):
        """Source-only draft: require real snapshot binding before helper import."""
        option = self.scope["profile"].get("r85_student_yaw_pd_fade")
        if option is None or (
            isinstance(option, dict) and set(option) == {"enabled"} and option["enabled"] is False
        ):
            return False
        if self.frame_serial or self.pid.frame_count or self.pid.previous_stamp is not None:
            raise ValueError("Fade configuration only before original first input")
        if self.unload_helper is None or self.integral_unload is None:
            raise ValueError("Original NativeUnload authority required")
        import importlib.util

        helper_path = Path(__file__).with_name("student_yaw_pd_fade.py")
        expected_snapshot = Path(self.binding["student_adapter_snapshot"]).with_name(
            helper_path.name
        )
        helper_origin = Path(self.binding["student_adapter_origin"]).with_name(helper_path.name)
        scope_bytes = (self.run / "runtime_context.json").read_bytes()
        expected_sha = self.scope["references"].get(str(helper_origin))
        spec = importlib.util.spec_from_file_location(
            "actual_snapshot_student_yaw_pd_fade", helper_path
        )
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        config = helper.configuration(self.scope["profile"], self.unload_helper)
        self.pid.yaw_pd_fade = helper.StudentYawPDFade(
            config, self.unload_helper, self.integral_unload.run_id, self.integral_unload.scope_sha
        )
        self.yaw_pd_fade_identity = dict(
            schema="r85_student_yaw_PD_fade_deployment_extension/v1",
            helper_path=str(helper_path),
            helper_source=str(helper_path),
            scope_sha256=None,
            configuration=config,
            base_student_contract_path=str(self.contract_path),
            base_contract_does_not_prove_new_PD_math=True,
        )
        return True

    def native_frame(self, state):
        state = np.asarray(state, dtype=float)
        if state.shape != (64,) or not np.isfinite(state).all():
            raise ValueError("Student state64 invalid")
        clock = round(float(state[0]) * 1e9)
        if self.frame_clock_ns is not None and clock <= self.frame_clock_ns:
            raise ValueError("Student native clock repeated/backwards")
        self.native_state = state.copy()
        self.frame_clock_ns = clock
        self.frame_serial += 1
        self.inferred = False
        self.obs45 = None
        self.input_receipt = None
        self.actual_action = None

    def prepare_command(
        self, cmd, request, input_clock_ns, ordinary_adapter, evidence, run_id, original_prepare
    ):
        if self.native_state is None or input_clock_ns != self.frame_clock_ns:
            raise ValueError("PID input not current native frame")
        envelope = (evidence or {}).get("actual_original_envelope") or {}
        lineage = envelope.get("controller_source") or {}
        epoch = None
        if np.any(request):
            rid = lineage.get("request_id")
            waypoint = lineage.get("waypoint_index")
            if (
                not isinstance(rid, str)
                or not rid.startswith(run_id + ":")
                or type(waypoint) is not int
                or waypoint < 0
                or (evidence or {}).get("rejected") is not False
            ):
                raise ValueError(
                    "PID nonzero motion epoch is not accepted original request/waypoint"
                )
            epoch = rid + ":waypoint:" + str(waypoint)
        unload = None
        if self.integral_unload is not None:
            authorization = self.integral_unload.authorize(evidence, request, input_clock_ns)
            if authorization is not None:
                unload = dict(
                    helper=self.unload_helper,
                    authorization=authorization,
                    input_clock_ns=input_clock_ns,
                    input_frame_serial=self.frame_serial,
                )
        if self.xy_stop_takeover is not None:
            self.xy_stop_takeover.before_pid(
                evidence,
                request,
                self.native_state[[8, 9, 13]],
                epoch,
                input_clock_ns,
                self.frame_serial,
                None if unload is None else unload["authorization"],
                self.pid,
            )
        if self.pid.yaw_pd_fade is not None:
            corrected, _ = self.pid.update(
                float(self.native_state[0] - 0.005),
                request,
                self.native_state[[8, 9, 13]],
                epoch,
                not bool(np.any(request)),
                pre_zero_unload=unload,
                pre_zero_pd_fade_frame=dict(
                    input_clock_ns=input_clock_ns, input_frame_serial=self.frame_serial
                ),
            )
        elif unload is None:
            corrected, _ = self.pid.update(
                float(self.native_state[0] - 0.005),
                request,
                self.native_state[[8, 9, 13]],
                epoch,
                not bool(np.any(request)),
            )
        else:
            corrected, _ = self.pid.update(
                float(self.native_state[0] - 0.005),
                request,
                self.native_state[[8, 9, 13]],
                epoch,
                not bool(np.any(request)),
                pre_zero_unload=unload,
            )
        if self.xy_stop_takeover is not None:
            self.xy_stop_takeover.after_pid(self.pid)
        # Exact original legacy worker slew. Request remains the producer request
        # for causal downstream residual ACK; do not mutate it with correction.
        original_prepare(cmd, corrected, input_clock_ns, None)
        proof = None if ordinary_adapter is None else ordinary_adapter(cmd, request, input_clock_ns)
        if proof is not None:
            if np.any(cmd):
                raise ValueError("Authorized ordinary zero failed")
            self.pid.synchronize_authorized_zero()
        xy_stop_receipt = None
        if self.xy_stop_takeover is not None:
            xy_stop_receipt = self.xy_stop_takeover.after_legacy_slew(cmd, request, proof, self.pid)
        if not np.isfinite(cmd).all() or np.any(np.abs(cmd) > self.pid.limits + 1e-10):
            raise ValueError("Final Student command exceeds original/Student limiter")
        self.pid.last_diagnostic.update(
            final_after_legacy_slew_body=cmd.tolist(),
            authoritative_source_rejected=bool((evidence or {}).get("rejected", True)),
            actual_input_clock_ns=input_clock_ns,
            actual_frame_serial=self.frame_serial,
            ordinary_zero_takeover=proof is not None,
        )
        if xy_stop_receipt is not None:
            self.pid.last_diagnostic["student_xy_stop_takeover"] = xy_stop_receipt
            self.pid.last_diagnostic["student_xy_stop_takeover_required"] = True
        if self.unload_helper is not None and not np.any(request):
            self.pid.last_diagnostic[self.unload_helper.KEY] = self.unload_helper.zero_observation(
                self.integral_unload.scope_sha,
                input_clock_ns,
                self.frame_serial,
                self.pid.last_diagnostic["dt_s"],
                self.pid.integral[2],
            )
        return proof

    def observation_frame(self, state, cmd, last_action, teacher_obs, noisy):
        if noisy:
            raise ValueError("Student clean exact45 contract forbids noisy Teacher proxy")
        if self.native_state is None or not np.array_equal(np.asarray(state), self.native_state):
            raise ValueError("Student build source differs from current native state")
        self.obs45 = exact_student45(state, cmd, last_action)
        teacher_obs = np.asarray(teacher_obs, dtype=np.float32)
        if teacher_obs.shape != (247,) or not np.isfinite(teacher_obs).all():
            raise ValueError("Original diagnostic247 invalid")
        clipped_proxy = np.r_[teacher_obs[3:36], teacher_obs[48:60]].astype(np.float32)
        self.teacher_slice_difference = float(np.max(np.abs(self.obs45 - clipped_proxy)))
        self.input_receipt = dict(
            schema="student_actual_input_frame/v1",
            actor_kind="student",
            frame_serial=self.frame_serial,
            input_clock_ns=self.frame_clock_ns,
            actor_observation_dimension=45,
            observation_clip=None,
            observation_noise=False,
            actual_observation_command_body=self.obs45[6:9].tolist(),
            teacher_proxy_max_abs_difference=self.teacher_slice_difference,
            exact_native45_used=True,
            teacher247_used_for_student_inference=False,
        )
        if self.pid.speed_tier is not None:
            self.input_receipt[speed_tiers.KEY] = dict(self.pid.speed_tier)
        if self.unload_helper is not None and self.unload_helper.KEY in self.pid.last_diagnostic:
            import copy

            self.input_receipt[self.unload_helper.KEY] = copy.deepcopy(
                self.pid.last_diagnostic[self.unload_helper.KEY]
            )
        if "student_yaw_pd_fade" in self.pid.last_diagnostic:
            import copy

            self.input_receipt["student_yaw_pd_fade"] = copy.deepcopy(
                self.pid.last_diagnostic["student_yaw_pd_fade"]
            )
        if self.pid.yaw_measurement_feedback is not None:
            helper = self.pid.yaw_measurement_feedback
            self.input_receipt[helper.KEY] = copy.deepcopy(self.pid.last_diagnostic[helper.KEY])
            self.input_receipt[helper.REQUIRED] = True
        if self.xy_stop_takeover is not None:
            import copy

            self.input_receipt["student_xy_stop_takeover"] = copy.deepcopy(
                self.pid.last_diagnostic["student_xy_stop_takeover"]
            )
            self.input_receipt["student_xy_stop_takeover_required"] = True

    def load_actor(self, torch):
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)
        self.model = torch.jit.load(str(self.model_path), map_location="cpu").eval()
        self.hidden = torch.zeros(1, 1, 256)
        with torch.inference_mode():
            action, hidden_probe = self.model(torch.zeros(1, 45), torch.zeros(1, 1, 256))
        if (
            tuple(action.shape) != (1, 12)
            or tuple(hidden_probe.shape) != (1, 1, 256)
            or not torch.isfinite(action).all()
            or not torch.isfinite(hidden_probe).all()
        ):
            raise RuntimeError("Actual Student JIT interface/probe rejected")
        runtime = self

        class Actor:
            def __call__(self, teacher_tensor):
                if (
                    tuple(teacher_tensor.shape) != (1, 247)
                    or runtime.obs45 is None
                    or runtime.inferred
                ):
                    raise RuntimeError("Student inference not one actual prepared frame")
                action, next_hidden = runtime.model(
                    torch.from_numpy(runtime.obs45.copy()).reshape(1, 45), runtime.hidden
                )
                if (
                    tuple(action.shape) != (1, 12)
                    or tuple(next_hidden.shape) != (1, 1, 256)
                    or not torch.isfinite(action).all()
                    or not torch.isfinite(next_hidden).all()
                ):
                    raise RuntimeError("Student actual action/GRU nonfinite or wrong shape")
                runtime.hidden = next_hidden
                runtime.inferred = True
                runtime.inference_count += 1
                runtime.actual_action = action.numpy()[0].copy()
                runtime.input_receipt.update(
                    actor_inferred_this_frame=True,
                    action_finite=True,
                    hidden_finite=True,
                    inference_sequence=runtime.inference_count,
                    hidden_norm=float(torch.linalg.vector_norm(next_hidden)),
                )
                return action

        return Actor()

    def telemetry(self, row):
        # Expose actual current state; optional telemetry serialization is not
        # an admission proof or a source of permission to run an inference.
        if not self.inferred or self.input_receipt is None:
            row.update(actor_kind="student", actor_inferred_this_frame=False)
            return row
        row.update(
            actor_kind="student",
            actor_inferred_this_frame=True,
            actual_actor_input=dict(self.input_receipt),
            student_pid=dict(self.pid.last_diagnostic),
            actual_actor_observation_dimension=45,
            teacher247_diagnostic_only=True,
            native_com_feedback_used_by_student_pid=True,
            observation_source="native gyro/quaternion/q/qd sim2sim proxies, actual command, previous raw action; exact45",
        )
        zero = row.get("ordinary_zero_before_actor_observation")
        if zero is not None:
            actual = self.obs45[6:9].tolist()
            zero.update(
                actual_observation_command_body=actual,
                observation_command_exact_zero=bool(
                    np.array_equal(self.obs45[6:9], np.zeros(3, np.float32))
                ),
                actor_inferred_this_frame=True,
                actual_actor_kind="student",
                actual_actor_observation_dimension=45,
                student_inference_sequence=self.inference_count,
            )
            if not zero["observation_command_exact_zero"]:
                raise RuntimeError("Student actual ordinary zero was not zero")
        return row

    def metadata(self):
        out = dict(
            actor_kind="student",
            controller_kind="student",
            actual_export_path=str(self.model_path),
            actor_observation_dimension=45,
            diagnostic_teacher_observation_dimension=247,
            teacher247_diagnostic_only=True,
            teacher_actor_loaded=False,
            privileged_actor_observation_dimensions=30,
            observation_clip=None,
            observation_noise="disabled clean Student evaluation",
            actor_observations_remain_privileged=True,
            outer_pid_feedback="privileged native COM body vx/vy/body omega_z; sim2sim",
            student_pid_reset_schema=RESET_SCHEMA,
            student_contract_path=str(self.contract_path),
            observation_source="native gyro/quaternion/q/qd simulation proxy + actual command + previous raw action; Student45",
            stop_strategy="exact authorized ordinary zero after original slew; continue Student/GRU; no target/action freezing",
            actual_input_evidence="current in-memory Actor state and live native ACK; raw recording is external",
        )
        if self.integral_unload is not None:
            out["pre_zero_integral_unload_extension"] = dict(self.integral_unload_identity)
        if self.pid.yaw_pd_fade is not None:
            out["student_yaw_PD_fade_extension"] = dict(self.yaw_pd_fade_identity)
        if self.pid.yaw_measurement_feedback is not None:
            out["student_yaw_measurement_P_retention_extension"] = dict(
                self.yaw_measurement_feedback_identity
            )
        if self.xy_stop_takeover is not None:
            out["student_XY_stop_takeover_mode_extension"] = dict(self.xy_stop_takeover_identity)
        if self.speed_tier is not None:
            out[speed_tiers.KEY] = dict(self.speed_tier)
            out["effective_student_command_bounds"] = self.pid.limits.tolist()
            out["base_contract_does_not_prove_selected_speed_bound"] = True
        return out

    def close(self):
        if self.stream is not None:
            self.stream.close()
