"""Versioned, causal 1s navigation-settle evidence; legacy stop remains separate."""

from collections import deque
import math

SPEC = dict(
    schema="navigation_settle_statistics/v1",
    enabled=True,
    window_s=1.0,
    gyro_mean_norm_radps=0.005,
    gyro_rms_norm_radps=0.05,
    gyro_peak_norm_radps=0.10,
    euler_peak_to_peak_rad=0.003,
    euler_net_change_rad=0.0015,
    xy_diameter_m=0.003,
    xy_net_change_m=0.002,
    eligible_purposes=["request_start", "region_transition", "path_rebind", "heading_change"],
    eligible_phase="settle",
    legacy_stop_predicate_retained=True,
    dynamic_recovery_path_admission_localization_hold_use_legacy=True,
    source="Actual accepted SLAM pose and causal body gyro; no native/truth pose gate",
    original_per_sample_zero_speed_yaw_pair_gap_freshness_retained=True,
)


def quaternion_euler(q):
    x, y, z, w = q
    return [
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ]


def bind_transition_context(node, purpose=None):
    """Keep actual recovery ownership stronger than a reset's nominal purpose."""
    gate = node.heading_gate
    if not hasattr(gate, "set_transition_context"):
        return
    token = getattr(node, "_teacher_recovery_token", None)
    dynamic = (
        bool(getattr(node, "obstacle_resume_pending", False))
        or isinstance(token, dict)
        and token.get("request_id") == getattr(node, "request_id", None)
        and token.get("waypoint_index") == getattr(node, "waypoint_index", None)
        and token.get("phase") in ("await_new_SCAN", "aligning")
    )
    context = dict(
        request_id=getattr(node, "request_id", None),
        waypoint_index=getattr(node, "waypoint_index", None),
        dynamic_recovery=dynamic,
        dynamic_event_sequence=token.get("sequence")
        if dynamic and isinstance(token, dict)
        else None,
    )
    if dynamic:
        purpose = "dynamic_recovery"
    elif getattr(node, "tilt_hold", False):
        purpose = "tilt_recovery"
    elif purpose is None:
        old = gate.transition_context
        if (
            old.get("request_id") != context["request_id"]
            or old.get("waypoint_index") != context["waypoint_index"]
        ):
            purpose = "unknown"  # Never infer motion eligibility from a changed identity.
        elif old.get("dynamic_recovery") and gate.phase == "drive":
            purpose = "route_drive"
        else:
            purpose = gate.transition_purpose
    gate.set_transition_context(purpose, context)


def reset_transition(node, purpose):
    node.heading_gate.reset()
    bind_transition_context(node, purpose)


class NavigationStopWindow:
    def __init__(self, config, legacy):
        if config is not None:
            from embodied_navigation.control.safety.safety_configuration import settle

            settle(config)
        self.enabled = config is not None
        self.legacy = legacy
        self.rows = deque(maxlen=128)
        self._cached = None

    def clear(self):
        self.rows.clear()
        self._cached = None

    def observe(self, stamp, wall, velocity, wz, gyro, gyro_stamp, zero, position, quaternion):
        if not self.enabled:
            return
        values = (
            [stamp, wall, wz, gyro_stamp]
            + list(velocity)
            + list(gyro)
            + list(position or [])
            + list(quaternion or [])
        )
        valid = (
            len(position or []) == 3
            and len(quaternion or []) == 4
            and all(math.isfinite(v) for v in values)
            and zero is True
            and math.hypot(*velocity[:2]) <= self.legacy["stop_planar_speed_mps"]
            and abs(wz) <= self.legacy["stop_slam_yaw_rate_radps"]
            and abs(stamp - gyro_stamp) <= self.legacy["gyro_pose_pair_max_s"]
        )
        if not valid:
            self.clear()
            return
        if self.rows and stamp <= self.rows[-1]["t"]:
            return
        if self.rows and stamp - self.rows[-1]["t"] > self.legacy["stop_pose_gap_max_s"]:
            self.clear()
        self.rows.append(
            dict(t=stamp, wall=wall, g=list(gyro), p=list(position), a=quaternion_euler(quaternion))
        )
        # Retain exactly one causal predecessor at or before the left boundary.
        left = stamp - SPEC["window_s"]
        while len(self.rows) > 2 and self.rows[1]["t"] <= left:
            self.rows.popleft()
        self._cached = None

    def metrics(self):
        if self._cached is not None:
            return self._cached
        out = dict(
            schema=SPEC["schema"],
            complete=False,
            passed=False,
            samples=len(self.rows),
            source="SLAM_and_body_gyro",
        )
        if len(self.rows) < 3 or self.rows[-1]["t"] - self.rows[0]["t"] < 1.0 - 1e-9:
            return out
        rows = list(self.rows)
        left = rows[-1]["t"] - 1.0
        if rows[0]["t"] > left + 1e-9:
            return out
        unwrapped = []
        for i, r in enumerate(rows):
            a = list(r["a"])
            if i:
                a = [
                    unwrapped[-1][j]
                    + math.atan2(
                        math.sin(a[j] - rows[i - 1]["a"][j]), math.cos(a[j] - rows[i - 1]["a"][j])
                    )
                    for j in range(3)
                ]
            unwrapped.append(a)
        first, second = rows[0], rows[1]
        alpha = (left - first["t"]) / (second["t"] - first["t"])
        alpha = max(0.0, min(1.0, alpha))
        window = [
            dict(
                t=left,
                g=[first["g"][j] + alpha * (second["g"][j] - first["g"][j]) for j in range(3)],
                p=[first["p"][j] + alpha * (second["p"][j] - first["p"][j]) for j in range(3)],
                a=[unwrapped[0][j] + alpha * (unwrapped[1][j] - unwrapped[0][j]) for j in range(3)],
            )
        ]
        window.extend(
            dict(t=r["t"], g=r["g"], p=r["p"], a=unwrapped[i])
            for i, r in enumerate(rows)
            if r["t"] > left
        )
        mean = [0.0, 0.0, 0.0]
        squares = [0.0, 0.0, 0.0]
        for prev, row in zip(window, window[1:]):
            dt = row["t"] - prev["t"]
            for j in range(3):
                mean[j] += 0.5 * dt * (prev["g"][j] + row["g"][j])
                squares[j] += 0.5 * dt * (prev["g"][j] ** 2 + row["g"][j] ** 2)
        norm = lambda v: math.sqrt(sum(x * x for x in v))
        peak = max(norm(r["g"]) for r in rows)  # Predecessor included, conservatively.
        pp = [max(r["a"][j] for r in window) - min(r["a"][j] for r in window) for j in range(3)]
        net = [abs(window[-1]["a"][j] - window[0]["a"][j]) for j in range(3)]
        diameter = max(
            math.hypot(a["p"][0] - b["p"][0], a["p"][1] - b["p"][1]) for a in window for b in window
        )
        xy_net = math.hypot(
            window[-1]["p"][0] - window[0]["p"][0], window[-1]["p"][1] - window[0]["p"][1]
        )
        conditions = dict(
            gyro_mean=norm(mean) <= SPEC["gyro_mean_norm_radps"],
            gyro_rms=math.sqrt(sum(squares)) <= SPEC["gyro_rms_norm_radps"],
            gyro_peak=peak <= SPEC["gyro_peak_norm_radps"],
            angles_peak_to_peak=max(pp) <= SPEC["euler_peak_to_peak_rad"],
            angles_net=max(net) <= SPEC["euler_net_change_rad"],
            xy_diameter=diameter <= SPEC["xy_diameter_m"],
            xy_net=xy_net <= SPEC["xy_net_change_m"],
        )
        out.update(
            complete=True,
            passed=all(conditions.values()),
            window_first_s=left,
            window_last_s=rows[-1]["t"],
            original_predecessor_s=rows[0]["t"],
            actual_source_span_s=rows[-1]["t"] - rows[0]["t"],
            signed_gyro_mean_radps=mean,
            gyro_mean_norm_radps=norm(mean),
            gyro_rms_norm_radps=math.sqrt(sum(squares)),
            gyro_peak_norm_radps=peak,
            euler_peak_to_peak_rad=pp,
            euler_net_change_rad=net,
            xy_diameter_m=diameter,
            xy_net_change_m=xy_net,
            conditions=conditions,
            legacy_threshold_result_is_not_relabelled=True,
        )
        self._cached = out
        return out

    def evidence(self, sim_time, wall_time):
        out = dict(self.metrics())
        fresh = bool(
            self.rows
            and 0 <= wall_time - self.rows[-1]["wall"] < 0.3
            and -0.05 <= sim_time - self.rows[-1]["t"] < 0.3
        )
        out.update(fresh=fresh, passed=bool(out["passed"] and fresh), enabled=self.enabled)
        return out
