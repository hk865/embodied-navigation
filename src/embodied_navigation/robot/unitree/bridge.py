#!/usr/bin/env python3
"""Gate reviewed SLAM/SCAN Twist commands into an atomic local JSON file.

This is a prepared integration boundary, not evidence of navigation success.
It never subscribes to Gazebo ground truth and never actuates joints itself.
"""

from __future__ import annotations

from embodied_navigation.project import project_root as _project_root

from embodied_navigation.common.observation import api as _observer_api

_r118_source_age = _observer_api("r118_source_age_probe")
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
import sys
from embodied_navigation.common.runtime_io import (
    BackgroundCheck,
    LatestCommandWriter,
    latest_sensor_qos,
)
from embodied_navigation.control.io import make_command_writer
from embodied_navigation.control.safety.gate_reasons import GateReasons
from embodied_navigation.common.integration_limits import (
    COMMAND_SOURCE_FRESHNESS_NS,
    COMMAND_SOURCE_FRESHNESS_S,
)
from embodied_navigation.control.parking.parking_zero_contract import validate_event
from embodied_navigation.common.observation import api as _observer_api

diagnostic_enabled = _observer_api("diagnostic_observation").enabled
diagnostic_record = _observer_api("diagnostic_observation").record
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled


ROOT = _project_root()
SLAM_TOPIC = "/demo/slam/body_odom"
COMMAND_TOPIC = "/demo/cmd_vel"
STATUS_TOPIC = "/demo/teacher/navigation_bridge/status"
STOP_TOPIC = "/demo/teacher/navigation_bridge/stop"
TIMEOUT = COMMAND_SOURCE_FRESHNESS_S
LIMITS = (0.3, 0.2, 0.3)


def runtime_description(configuration):
    """Minimal run-description dict handed to the local command writer."""
    return dict(run_dir=configuration["run_dir"], actor_kind="student")


def load_configuration(path):
    """Load the frozen runtime context and mark it as the live controller run."""
    from embodied_navigation.common.runtime_context import load

    value = load(path)
    return dict(value, current_runtime=True, controller_kind="student")


def finite_values(values, length):
    """True only for a length-``length`` sequence of finite non-bool numbers."""
    if not isinstance(values, (list, tuple)) or len(values) != length:
        return False
    return all(
        not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in values
    )


def _diagnostic_route_score_valid(value, points, config):
    """Validate the existing score/projection identity; do not change its math."""
    if not isinstance(value, dict) or type(value.get("inside")) is not bool:
        return False
    index = value.get("segment")
    fraction = value.get("fraction")
    if (
        type(index) is not int
        or not isinstance(points, (list, tuple))
        or not 0 <= index < len(points) - 1
        or not finite_values(
            [
                fraction,
                value.get("horizontal_error_m"),
                value.get("height_error_m"),
                value.get("horizontal_radius_m"),
                value.get("maximum_height_error_m"),
            ],
            5,
        )
        or not 0 <= fraction <= 1
        or value["horizontal_error_m"] < 0
        or value["height_error_m"] < 0
        or value["horizontal_radius_m"] != config["horizontal_radius_m"]
        or value["maximum_height_error_m"] != config["height_error_m"]
        or value.get("navigation_ground_truth_used") is not False
        or not finite_values(value.get("nearest_camera_init_xyz"), 3)
    ):
        return False
    start, end = points[index], points[index + 1]
    if not finite_values(start, 3) or not finite_values(end, 3):
        return False
    delta = [b - a for a, b in zip(start, end)]
    norm2 = sum(x * x for x in delta)
    projection = [a + fraction * d for a, d in zip(start, delta)]
    return (
        math.isfinite(norm2)
        and norm2 >= 1e-12
        and finite_values(projection, 3)
        and projection == value["nearest_camera_init_xyz"]
        and value["inside"]
        == (
            value["horizontal_error_m"] <= value["horizontal_radius_m"]
            and value["height_error_m"] <= value["maximum_height_error_m"]
        )
    )


def command_limits(profile):
    """The reference speed is not the cascade's compensation command limit."""
    if profile is not None and isinstance(profile.get("cascade"), dict):
        limits = profile["cascade"]["command_limits"]
        if not finite_values(limits, 3) or not all(
            0 < value <= maximum for value, maximum in zip(limits, (1.0, 0.4, 1.0))
        ):
            raise ValueError("Invalid closed-loop Teacher command envelope")
        return tuple(limits)
    return LIMITS


def validate_controller_source(
    data, requested, clock_ns, wall_ns, run_id=None, *, observe_wall=False, wall_observations=None
):
    """Check original source times without refreshing them on file reads."""
    if not isinstance(data, dict) or data.get("navigation_ground_truth_used") is not False:
        raise ValueError("No original actual-SLAM cascade provenance")
    if not finite_values(data.get("command_after_slew"), 3) or data["command_after_slew"] != list(
        requested
    ):
        raise ValueError("Controller source does not exactly match received Twist")
    keys = (
        "source_pose_stamp_ns",
        "paired_imu_stamp_ns",
        "control_stamp_ns",
        "source_pose_received_wall_ns",
        "source_imu_received_wall_ns",
        "cascade_sequence",
    )
    if (
        any(type(data.get(key)) is not int or data[key] < 0 for key in keys)
        or data["cascade_sequence"] < 1
    ):
        raise ValueError("Original cascade source timestamps/sequence are malformed")
    if type(clock_ns) is not int or type(wall_ns) is not int:
        raise ValueError("Source clock unavailable")
    pose, gyro, control = (data[key] for key in keys[:3])
    if not (
        0 <= clock_ns - pose <= COMMAND_SOURCE_FRESHNESS_NS
        and 0 <= pose - gyro <= 20_000_000
        and pose <= control <= clock_ns + 50_000_000
    ):
        raise ValueError("Original cascade source age or causality rejected")
    wall_ages = {key: wall_ns - data[key] for key in keys[3:5]}
    if observe_wall:
        if any(age < 0 for age in wall_ages.values()):
            raise ValueError("Original SLAM/IMU cascade wall source is future")
        if wall_observations is not None:
            wall_observations.extend(
                {"field": key, "age_ns": age, "original_limit_ns": COMMAND_SOURCE_FRESHNESS_NS}
                for key, age in wall_ages.items()
                if age > COMMAND_SOURCE_FRESHNESS_NS
            )
    elif not all(0 <= age <= COMMAND_SOURCE_FRESHNESS_NS for age in wall_ages.values()):
        raise ValueError("Original SLAM/IMU cascade wall source expired")
    # Preserve optional seconds aliases exactly, while refusing inconsistent
    # representations if the executor's reader uses those aliases.
    aliases = (
        ("source_pose_received_monotonic_wall", "source_pose_received_wall_ns"),
        ("paired_imu_received_monotonic_wall", "source_imu_received_wall_ns"),
    )
    for seconds, nanos in aliases:
        if seconds in data and (
            not finite_values([data[seconds]], 1) or abs(data[seconds] - data[nanos] / 1e9) > 1e-9
        ):
            raise ValueError("Cascade wall-time aliases disagree")
    if data.get("healthy") is False:
        raise ValueError("Controller source explicitly unhealthy")
    from embodied_navigation.control.parking.terminal_reference import validate_fields

    validate_fields(data, run_id)
    marker = data.get("ordinary_terminal_zero_event")
    if marker is not None:
        if run_id is None:
            raise ValueError("No independent current run for ordinary zero")
        validate_event(
            marker, data, run_id, requested, clock_ns, source_max_age_ns=COMMAND_SOURCE_FRESHNESS_NS
        )
    return data


def read_controller_source(
    path, requested, clock_ns, wall_ns, allow_untyped_zero=False, *, observe_wall=False
):
    """Return the unmodified decoded source and raw-file receipt on failure too."""
    data = None
    receipt = {
        "path": str(path.resolve()),
        "sha256": None,
        "read_begin_wall_ns": wall_ns,
        "read_wall_ns": None,
    }
    try:
        raw = path.read_bytes()
        if len(raw) > 256 * 1024:
            raise ValueError("Cascade source exceeds 256 KiB limit")
        receipt["sha256"] = hashlib.sha256(raw).hexdigest()
        data = json.loads(raw)
        if (
            allow_untyped_zero
            and not any(requested)
            and (not isinstance(data, dict) or data.get("ordinary_terminal_zero_event") is None)
        ):
            return None, receipt, None
        # A producer may publish between read begin and actual read. Evaluate
        # wall TTL after decoding, preserving begin/completion for audit, as
        # the already validated V5 command consumer does.
        receipt["read_wall_ns"] = time.monotonic_ns()
        observations = []
        validate_controller_source(
            data,
            requested,
            clock_ns,
            receipt["read_wall_ns"],
            Path(path).parent.name,
            observe_wall=observe_wall,
            wall_observations=observations,
        )
        if observe_wall:
            receipt["simulation_wall_age_observations"] = observations
        return data, receipt, None
    except (OSError, ValueError, TypeError, KeyError) as error:
        if allow_untyped_zero and not any(requested) and data is None:
            return None, receipt, None
        return data, receipt, type(error).__name__ + ": " + str(error)


class CommandGate:
    """Wall time remains authoritative during paused or missing ROS clocks."""

    def __init__(self, profile=None, run_id=None):
        self.run_id = run_id
        self.profile = profile
        self.minimal_baseline = minimal_baseline_enabled(profile) if profile is not None else False
        self.limits = command_limits(profile)
        self.controller_source = None
        self.controller_source_receipt = None
        self.controller_source_error = None
        self.execution_health = None
        self.execution_health_wall = None
        self.position = None
        self.sensor_health = None
        self.sensor_health_wall = None
        self.anchor = None
        self.scope_graph_ok = False
        self.clock_ns = None
        self.clock_wall = None
        self.pose_stamp_ns = None
        self.pose_wall = None
        self.pose_valid_samples = 0
        self.pose_error = "SLAM pose not received"
        self.command_wall = None
        self.command_sim_ns = None
        self.requested = [0.0, 0.0, 0.0]
        self.command_error = "SCAN command not received"
        self.graph = {"slam": False, "command": False, "details": {}}
        self.graph_wall = None
        self.failed = None

    def clock(self, stamp_ns, now):
        """Update when simulation time advances; ignore duplicates and latch backward moves."""
        if not isinstance(stamp_ns, int) or stamp_ns < 0:
            self.failed = "Invalid simulation clock"
        elif self.clock_ns is not None and stamp_ns < self.clock_ns:
            self.failed = "Simulation clock moved backward; restart the integration run"
        elif self.clock_ns is None or stamp_ns > self.clock_ns:
            self.clock_ns, self.clock_wall = stamp_ns, now

    def pose(self, stamp_ns, position, quaternion, frame, child_frame, now):
        """Accept a measured SLAM body pose; invalid samples set an error that a valid pose clears."""
        error = None
        if frame != "camera_init" or child_frame != "demo_slam_body":
            error = "Navigation pose must be camera_init / demo_slam_body"
        elif not finite_values(position, 3) or not finite_values(quaternion, 4):
            error = "SLAM pose is nonfinite or malformed"
        elif not 0.98 <= sum(v * v for v in quaternion) <= 1.02:
            error = "SLAM quaternion is not normalized"
        elif not isinstance(stamp_ns, int) or stamp_ns < 0:
            error = "SLAM pose timestamp is invalid"
        elif self.pose_stamp_ns is not None and stamp_ns <= self.pose_stamp_ns:
            error = "SLAM pose timestamps must strictly increase"
        elif self.clock_ns is not None and stamp_ns > self.clock_ns + 50_000_000:
            error = "SLAM pose timestamp is ahead of simulation clock"
        if error:
            self.pose_error = error
            self.pose_valid_samples = 0
            return
        if self.pose_stamp_ns is not None and stamp_ns - self.pose_stamp_ns >= int(TIMEOUT * 1e9):
            self.pose_valid_samples = 0
        self.pose_stamp_ns, self.pose_wall = stamp_ns, now
        self.position = list(position)
        self.pose_valid_samples += 1
        self.pose_error = None

    def command(
        self,
        values,
        unsupported_axes,
        now,
        controller_source=None,
        source_receipt=None,
        source_error=None,
    ):
        """Accept one finite SCAN body command; z/roll/pitch requests latch failure."""
        if not finite_values(values, 3) or not finite_values(unsupported_axes, 3):
            self.failed = "SCAN command contains a nonfinite value"
            self.command_error = self.failed
            return
        if any(abs(v) > 1e-9 for v in unsupported_axes):
            self.failed = "SCAN command requests unsupported z/roll/pitch motion"
            self.command_error = self.failed
            return
        self.requested = list(values)
        self.controller_source = controller_source
        self.controller_source_receipt = source_receipt
        self.controller_source_error = source_error
        self.command_wall, self.command_sim_ns = now, self.clock_ns
        self.command_error = None

    def snapshot(self, now):
        """Return the gate decision and every age/reason that produced it."""
        ages = {
            "clock_wall_s": None if self.clock_wall is None else now - self.clock_wall,
            "pose_wall_s": None if self.pose_wall is None else now - self.pose_wall,
            "command_wall_s": None if self.command_wall is None else now - self.command_wall,
            "pose_sim_s": None
            if self.clock_ns is None or self.pose_stamp_ns is None
            else (self.clock_ns - self.pose_stamp_ns) / 1e9,
            "command_sim_s": None
            if self.clock_ns is None or self.command_sim_ns is None
            else (self.clock_ns - self.command_sim_ns) / 1e9,
        }
        reasons = GateReasons()
        wall_observations = []
        if self.failed:
            reasons.reject("latched_failure", self.failed)
        if self.profile is not None:
            if self.profile.get("cascade") is not None and (
                any(self.requested)
                or isinstance(self.controller_source, dict)
                and self.controller_source.get("ordinary_terminal_zero_event") is not None
            ):
                try:
                    if any(abs(value) > limit for value, limit in zip(self.requested, self.limits)):
                        raise ValueError("Cascade Twist exceeds its frozen command envelope")
                    if self.controller_source_error:
                        raise ValueError(self.controller_source_error)
                    validate_controller_source(
                        self.controller_source,
                        self.requested,
                        self.clock_ns,
                        round(now * 1e9),
                        self.run_id,
                        observe_wall=self.minimal_baseline,
                        wall_observations=wall_observations,
                    )
                except (ValueError, TypeError, KeyError) as error:
                    reasons.reject(
                        "cascade_provenance", "Cascade provenance rejected: " + str(error)
                    )
            if self.profile["controller_kind"] == "champ":
                h = self.execution_health
                ready = (
                    h is not None
                    and self.execution_health_wall is not None
                    and now - self.execution_health_wall < TIMEOUT
                    and -0.05 <= now - float(h["monotonic_wall"]) < TIMEOUT
                    and self.clock_ns is not None
                    and -50_000_000
                    <= self.clock_ns - int(h["ros_sim_time_ns"])
                    < COMMAND_SOURCE_FRESHNESS_NS
                    and h.get("state") == "ready"
                    and h.get("ready") is True
                )
                if not ready:
                    reasons.reject(
                        "champ_feedback_freshness",
                        "Actual CHAMP executor calibration/controller feedback is not healthy and fresh",
                    )
                if h is not None and h.get("state") == "failed":
                    self.failed = "CHAMP executor failed"
                    reasons.reject("champ_failed", self.failed)
            if not self.scope_graph_ok:
                reasons.reject(
                    "sensor_anchor_publisher_chain",
                    "Finite experiment sensor gate / SLAM anchor publisher chain is not unique",
                )
            sensor_age = None if self.sensor_health_wall is None else now - self.sensor_health_wall
            if (
                self.minimal_baseline
                and sensor_age is not None
                and math.isfinite(sensor_age)
                and sensor_age >= TIMEOUT
            ):
                wall_observations.append(
                    {
                        "field": "sensor_health_wall_s",
                        "age_s": sensor_age,
                        "original_limit_s": TIMEOUT,
                    }
                )
            if (
                self.sensor_health is None
                or self.sensor_health_wall is None
                or (not self.minimal_baseline and sensor_age >= TIMEOUT)
                or (self.minimal_baseline and (not math.isfinite(sensor_age) or sensor_age < 0))
                or self.sensor_health.get("ready") is not True
            ):
                reasons.reject(
                    "sensor_warmup", "Actual vehicle sensor SLAM has not completed healthy warmup"
                )
            if self.anchor is None:
                reasons.reject(
                    "immutable_slam_anchor", "Waiting for immutable SLAM-derived route anchor"
                )
            elif self.position is not None:
                a = self.anchor
                dx = self.position[0] - a["origin"][0]
                dy = self.position[1] - a["origin"][1]
                c, s = math.cos(a["yaw"]), math.sin(a["yaw"])
                along = c * dx + s * dy
                lateral = -s * dx + c * dy
                fence = self.profile.get("fence")
                route_score_valid = False
                if self.profile.get("registered_route_fence"):
                    from embodied_navigation.planning.reference.route_fence import tube_evidence

                    config = self.profile["registered_route_fence"]
                    try:
                        self.registered_fence = tube_evidence(
                            self.position,
                            a["registered_route_camera_init_xyz"],
                            config["horizontal_radius_m"],
                            config["height_error_m"],
                        )
                        outside = not self.registered_fence["inside"]
                        route_score_valid = diagnostic_enabled(
                            self.profile
                        ) and _diagnostic_route_score_valid(
                            self.registered_fence, a["registered_route_camera_init_xyz"], config
                        )
                    except (KeyError, ValueError, TypeError):
                        outside = True
                else:
                    outside = (
                        not fence["along_min_m"] <= along <= fence["along_max_m"]
                        or abs(lateral) > fence["lateral_abs_m"]
                        or abs(self.position[2] - a["origin"][2]) > fence["height_abs_m"]
                    )
                if outside:
                    route_reason = (
                        "Measured SLAM body pose left finite route fence; restart after review"
                    )
                    # Observe only this successfully computed business score.
                    # Malformed/degenerate geometry and every other hard bridge
                    # condition retain their original failure/zero-command path.
                    run_dir = a.get("run_dir")
                    continued = (
                        route_score_valid
                        and diagnostic_enabled(self.profile)
                        and isinstance(run_dir, str)
                        and Path(run_dir).is_absolute()
                        and Path(run_dir).name == self.run_id
                        and diagnostic_record(
                            Path(run_dir),
                            self.profile,
                            "bridge_route_fence",
                            route_reason,
                            dict(
                                original_check_passed=False,
                                registered_route_fence=self.registered_fence,
                                measured_body_camera_init_xyz=list(self.position),
                                source_pose_stamp_ns=self.pose_stamp_ns,
                                clock_ns=self.clock_ns,
                                controller_source_pose_stamp_ns=(
                                    self.controller_source.get("source_pose_stamp_ns")
                                    if isinstance(self.controller_source, dict)
                                    else None
                                ),
                                original_thresholds_unchanged=True,
                                path_coordinates_unchanged=True,
                            ),
                        )
                    )
                    if not continued:
                        self.failed = route_reason
                        reasons.reject("route_fence", self.failed)
        graph_age = None if self.graph_wall is None else now - self.graph_wall
        if (
            self.minimal_baseline
            and graph_age is not None
            and math.isfinite(graph_age)
            and graph_age >= TIMEOUT
        ):
            wall_observations.append(
                {"field": "publisher_graph_wall_s", "age_s": graph_age, "original_limit_s": TIMEOUT}
            )
        if (
            self.graph_wall is None
            or (not self.minimal_baseline and graph_age >= TIMEOUT)
            or (self.minimal_baseline and (not math.isfinite(graph_age) or graph_age < 0))
        ):
            reasons.reject("publisher_graph_freshness", "ROS publisher graph check is stale")
        if not self.graph["slam"]:
            reasons.reject(
                "slam_publisher_identity",
                "SLAM source must have exactly one demo_slam_odom_adapter publisher",
            )
        if not self.graph["command"]:
            reasons.reject(
                "command_publisher_identity",
                "Command source must have exactly one demo_navigation publisher",
            )
        if self.pose_error:
            reasons.reject("slam_pose_validity", self.pose_error)
        if self.pose_valid_samples < 2:
            reasons.reject("two_valid_slam_poses", "Waiting for two consecutive valid SLAM poses")
        if self.command_error:
            reasons.reject("command_validity", self.command_error)
        for name, age in ages.items():
            observed = (
                self.minimal_baseline
                and name.endswith("_wall_s")
                and age is not None
                and math.isfinite(age)
                and age >= TIMEOUT
            )
            if observed:
                wall_observations.append({"field": name, "age_s": age, "original_limit_s": TIMEOUT})
            if not observed and (
                age is None
                or age >= TIMEOUT
                or age < -0.05
                or self.minimal_baseline
                and not math.isfinite(age)
            ):
                reasons.reject("dual_source_age", f"{name} is missing, stale, or inconsistent")
        healthy = not reasons
        command = (
            [max(-limit, min(limit, value)) for value, limit in zip(self.requested, self.limits)]
            if healthy
            else [0.0, 0.0, 0.0]
        )
        if self.profile is not None and self.profile.get("cascade") is None:
            scoped_limits = [
                self.profile["max_speed_mps"],
                self.profile["max_lateral_speed_mps"],
                self.profile["max_yaw_rate_radps"],
            ]
            command = [
                max(-limit, min(limit, value)) for value, limit in zip(command, scoped_limits)
            ]
        status = {
            "state": "failed" if self.failed else "ready" if healthy else "hold",
            "healthy": healthy,
            "stop_requested": not healthy or not any(command),
            "reason": "; ".join(dict.fromkeys(reasons)) if reasons else "Fresh SLAM/SCAN inputs",
            "requested": self.requested.copy(),
            "command": command,
            "ages": ages,
            "slam_valid_samples": self.pose_valid_samples,
            "command_received_monotonic_wall": self.command_wall,
            "command_received_ros_sim_time_ns": self.command_sim_ns,
            "slam_stamp_ns": self.pose_stamp_ns,
            "publisher_graph": self.graph,
            "controller_source": self.controller_source,
            "controller_source_receipt": self.controller_source_receipt,
            "registered_route_fence": getattr(self, "registered_fence", None),
            "controller_source_error": self.controller_source_error,
            "gate_events": reasons.events,
            "ordinary_terminal_zero_event": (
                self.controller_source.get("ordinary_terminal_zero_event")
                if healthy and isinstance(self.controller_source, dict)
                else None
            ),
            "command_intent": (
                "gate_hold"
                if not healthy
                else "ordinary_parking"
                if isinstance(self.controller_source, dict)
                and self.controller_source.get("ordinary_terminal_zero_event") is not None
                else "tracking"
                if any(command)
                else "untyped_zero"
            ),
        }
        if self.minimal_baseline:
            status["simulation_wall_age_observations"] = wall_observations
            if healthy:
                status["reason"] = (
                    "Simulation baseline: structural/simulation-time input allowed; original wall ages observed"
                )
        return status


class CommandFile:
    def __init__(self, path):
        self.path = path.resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = (self.path.parent / (self.path.name + ".lock")).open("a")
        try:
            fcntl.flock(self.lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.lock.close()
            raise RuntimeError("Another navigation bridge owns this command file") from error

    def write(self, value):
        payload = (
            json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
        )
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self.path.parent,
                prefix="." + self.path.name + ".",
                delete=False,
            ) as stream:
                temporary = Path(stream.name)
                stream.write(payload)
                stream.flush()
                # Transient transport: close + atomic replace provide complete
                # old/new JSON reads. No power-loss durability is promised.
                # Original stamps remain unchanged; late delivery still expires.
            temporary.replace(self.path)
            if _r118_source_age.ENABLED:
                _r118_source_age.envelope("command.atomic_replace", value)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def close(self):
        self.lock.close()


def make_node(rclpy, writer, configuration_path, configuration):
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String

    if _r118_source_age.ENABLED:
        _r118_source_age.bind("bridge", configuration["run_dir"])

    class Bridge(Node):
        def __init__(self):
            super().__init__("teacher_scan_slam_bridge")
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
            self.gate = CommandGate(
                configuration.get("profile") if configuration.get("current_runtime") else None,
                Path(configuration["run_dir"]).name
                if configuration.get("current_runtime")
                else None,
            )
            self.sequence = 0
            self.output_configuration = runtime_description(configuration)
            self.configuration_checked_wall = None
            self.last_status_wall = -math.inf
            self.last_published_state = None
            history = (
                Path(configuration["run_dir"]) / "navigation_command_history.jsonl"
                if configuration.get("current_runtime")
                else None
            )
            self.transport = make_command_writer(writer, history, configuration)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
            self.stop_pub = self.create_publisher(Bool, STOP_TOPIC, 10)
            self.safety_pub = self.create_publisher(String, "/demo/control/safety", 10)
            if configuration.get("current_runtime"):
                self.create_subscription(
                    String, "/demo/teacher/navigation/sensor_health", self.sensor_health, 1
                )
                latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.create_subscription(
                    String, "/demo/teacher/navigation/anchor", self.anchor, latched
                )
                if configuration["controller_kind"] == "champ":
                    self.create_subscription(
                        String, "/demo/champ/execution_health", self.execution_health, 1
                    )
            self.pose_sub = self.create_subscription(
                Odometry, SLAM_TOPIC, self.pose, latest_sensor_qos()
            )
            self.command_sub = self.create_subscription(Twist, COMMAND_TOPIC, self.command, 1)
            if (
                self.pose_sub.topic_name != SLAM_TOPIC
                or self.command_sub.topic_name != COMMAND_TOPIC
            ):
                self.destroy_node()
                raise RuntimeError(
                    "Remapping SLAM/SCAN input topics is forbidden; ground truth cannot be a navigation source"
                )
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.checker = BackgroundCheck(self.check_integrity)
            self.timer = self.create_timer(0.02, self.tick, clock=self.wall_clock)

        def sync_clock(self):
            # rclpy's single ROS TimeSource subscription retains the latest
            # real /clock. Never stamp commands from a second lagging queue.
            self.gate.clock(self.get_clock().now().nanoseconds, time.monotonic())

        def pose(self, message):
            self.sync_clock()
            p, q = message.pose.pose.position, message.pose.pose.orientation
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            self.gate.pose(
                stamp,
                [p.x, p.y, p.z],
                [q.x, q.y, q.z, q.w],
                message.header.frame_id,
                message.child_frame_id,
                time.monotonic(),
            )
            if self.gate.pose_error:
                self.tick()

        def command(self, message):
            self.sync_clock()
            was_moving = any(self.gate.requested)
            values = [message.linear.x, message.linear.y, message.angular.z]
            source = source_receipt = source_error = None
            if configuration.get("current_runtime"):
                source, source_receipt, source_error = read_controller_source(
                    Path(configuration["run_dir"]) / "cascade_command_source.json",
                    values,
                    self.gate.clock_ns,
                    time.monotonic_ns(),
                    allow_untyped_zero=True,
                    observe_wall=self.gate.minimal_baseline,
                )
            self.gate.command(
                values,
                [message.linear.z, message.angular.x, message.angular.y],
                time.monotonic(),
                source,
                source_receipt,
                source_error,
            )
            if _r118_source_age.ENABLED:
                _r118_source_age.bridge_received(self.gate.clock_ns, source, values, source_error)
            # 50Hz wall heartbeat writes ordinary commands. Stop/fault edges
            # enqueue an immediate zero without creating a callback feedback.
            if self.gate.failed or source_error or was_moving and not any(self.gate.requested):
                self.tick()

        def sensor_health(self, message):
            self.sync_clock()
            try:
                data = json.loads(message.data)
                now = time.monotonic()
                if (
                    data.get("mode") != configuration["controller_kind"]
                    or data.get("ground_truth_navigation_used") is not False
                    or not math.isfinite(float(data["monotonic_wall"]))
                    or not -0.05 <= now - float(data["monotonic_wall"])
                    or (
                        not self.gate.minimal_baseline
                        and now - float(data["monotonic_wall"]) >= TIMEOUT
                    )
                ):
                    return
                self.gate.sensor_health = data
                self.gate.sensor_health_wall = now
            except (ValueError, TypeError, KeyError):
                return

        def execution_health(self, message):
            self.sync_clock()
            now = time.monotonic()
            try:
                d = json.loads(message.data)
                if (
                    d.get("schema") != 1
                    or d.get("mode") != "champ"
                    or d.get("source") != "scan_slam"
                    or d.get("ground_truth_navigation_used") is not False
                    or d.get("actor_started") is not False
                    or d.get("state") not in ("hold", "ready", "failed")
                    or not -0.05 <= now - float(d["monotonic_wall"]) < TIMEOUT
                    or self.gate.clock_ns is None
                    or not -50_000_000
                    <= self.gate.clock_ns - int(d["ros_sim_time_ns"])
                    < COMMAND_SOURCE_FRESHNESS_NS
                ):
                    return
                self.gate.execution_health = d
                self.gate.execution_health_wall = now
                if d["state"] != "ready" and any(self.gate.requested):
                    self.tick()
            except (ValueError, TypeError, KeyError):
                return

        def anchor(self, message):
            try:
                data = json.loads(message.data)
                if (
                    data.get("schema") != 1
                    or data.get("frame_id") != "camera_init"
                    or data.get("source") != SLAM_TOPIC
                    or data.get("ground_truth_navigation_used") is not False
                    or data.get("run_dir") != configuration["run_dir"]
                    or not finite_values(data.get("origin"), 3)
                    or not finite_values([data.get("yaw")], 1)
                ):
                    return
                if self.gate.anchor is not None and self.gate.anchor != data:
                    self.gate.failed = "Attempt to change finite route SLAM anchor"
                    return
                self.gate.anchor = data
            except (OSError, ValueError, TypeError, KeyError):
                return

        def publishers(self, topic, allowed):
            infos = self.get_publishers_info_by_topic(topic)
            names = [item.node_namespace.rstrip("/") + "/" + item.node_name for item in infos]
            return len(infos) == 1 and infos[0].node_name == allowed and infos[
                0
            ].node_namespace == "/", names

        def envelope(self, now):
            status = self.gate.snapshot(now)
            self.sequence += 1
            status.update(
                {
                    "schema_version": 1,
                    "sequence": self.sequence,
                    "mode": "scan_slam",
                    "source": "scan_slam",
                    "monotonic_wall": now,
                    "sim_time": None if self.gate.clock_ns is None else self.gate.clock_ns / 1e9,
                    "stamp": {
                        "monotonic_wall": now,
                        "ros_sim_time_ns": self.gate.clock_ns,
                        "ros_sim_time": None
                        if self.gate.clock_ns is None
                        else self.gate.clock_ns / 1e9,
                    },
                    "valid_for_wall_s": TIMEOUT,
                    "limits": {
                        "vx_mps": self.gate.limits[0],
                        "vy_mps": self.gate.limits[1],
                        "wz_radps": self.gate.limits[2],
                    },
                    "observation_source": {
                        "navigation_pose": SLAM_TOPIC,
                        "navigation_frame": "camera_init",
                        "navigation_ground_truth_used": False,
                        "velocity_request": COMMAND_TOPIC,
                        "policy_inputs": configuration["profile"]["policy_observations"],
                    },
                    "runtime": self.output_configuration,
                    "health_scope": "SLAM/SCAN input health only; this does not establish actuator health or physical stopping",
                }
            )
            status["clock_source"] = "single rclpy ROS TimeSource /clock, KEEP_LAST depth=1"
            status["transport"] = {
                "type": "latest_pending_atomic_file",
                "superseded": self.transport.superseded,
                "written": self.transport.written,
                "max_queue_delay_wall_s": self.transport.max_delay_s,
            }
            if _r118_source_age.ENABLED:
                _r118_source_age.envelope("bridge.envelope", status)
            return status

        def publish(self, status):
            if rclpy.ok():
                self.status_pub.publish(
                    String(data=json.dumps(status, ensure_ascii=False, allow_nan=False))
                )
                self.stop_pub.publish(Bool(data=status["stop_requested"]))
                self.safety_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                **status,
                                "mode": configuration["controller_kind"],
                                "source": "scan_slam",
                            },
                            allow_nan=False,
                        )
                    )
                )

        def check_integrity(self):
            slam_ok, slam_nodes = self.publishers(SLAM_TOPIC, "demo_slam_odom_adapter")
            command_ok, command_nodes = self.publishers(COMMAND_TOPIC, "demo_navigation")
            scope_ok = True
            if configuration.get("current_runtime"):
                health_ok, _ = self.publishers(
                    "/demo/teacher/navigation/sensor_health", "teacher_navigation_sensor_gate"
                )
                anchor_ok, _ = self.publishers(
                    "/demo/teacher/navigation/anchor", "teacher_navigation_request"
                )
                scope_ok = health_ok and anchor_ok
                if configuration["controller_kind"] == "champ":
                    execution_ok, _ = self.publishers(
                        "/demo/champ/execution_health", "champ_reviewed_command_reader"
                    )
                    scope_ok = scope_ok and execution_ok
            return {
                "graph": {
                    "slam": slam_ok,
                    "command": command_ok,
                    "details": {"slam_publishers": slam_nodes, "command_publishers": command_nodes},
                },
                "scope_ok": scope_ok,
            }

        def tick(self):
            self.sync_clock()
            now = time.monotonic()
            checked = self.checker.result
            if checked is not None:
                if checked["error"]:
                    self.gate.failed = "Integrity check failed: " + checked["error"]
                else:
                    self.gate.graph = checked["data"]["graph"]
                    self.gate.scope_graph_ok = checked["data"]["scope_ok"]
                    self.gate.graph_wall = checked["started_wall"]
            if self.transport.error:
                self.gate.failed = self.transport.error
            status = self.envelope(now)
            try:
                self.transport.submit(status)
            except RuntimeError as error:
                self.gate.failed = f"Command file write failed: {error}"
                status = self.envelope(now)
                self.publish(status)
                raise RuntimeError(self.gate.failed) from error
            # A normal health heartbeat is 20Hz. State/stop edges publish
            # immediately; repeated already-zero updates do not flood DDS.
            published = (status["state"], status["stop_requested"])
            if now - self.last_status_wall >= 0.05 or published != self.last_published_state:
                self.publish(status)
                self.last_status_wall = now
                self.last_published_state = published

        def shutdown_stop(self):
            if getattr(self, "shutdown_started", False):
                return
            self.shutdown_started = True
            self.gate.failed = "Navigation command bridge stopped"
            status = self.envelope(time.monotonic())
            try:
                self.transport.close(status)
            finally:
                self.publish(status)
                self.checker.close()

    return Bridge()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-config", dest="configuration", type=Path, required=True)
    parser.add_argument("--command-file", type=Path, default=None)
    parser.add_argument(
        "--prepare",
        action="store_true",
        help="Review the gate and print an UNVERIFIED receipt; start no ROS nodes",
    )
    args, ros_args = parser.parse_known_args()
    try:
        configuration = load_configuration(args.configuration)
        refusal = None
    except RuntimeError as error:
        configuration, refusal = None, str(error)
    if args.command_file is None:
        args.command_file = (
            Path(configuration["run_dir"]) / "navigation_command.json"
            if configuration and configuration.get("current_runtime")
            else Path(os.environ.get("TEACHER_COMMAND_FILE", str(ROOT / "runs/nav_command.json")))
        )
    if (
        configuration
        and configuration.get("current_runtime")
        and args.command_file.resolve()
        != Path(configuration["run_dir"]) / "navigation_command.json"
    ):
        refusal = (
            "Finite navigation command output must belong to this run: navigation_command.json"
        )
    if args.prepare:
        print(
            json.dumps(
                {
                    "status": "prepared_unverified",
                    "navigation": "unverified",
                    "starts_ros": False,
                    "writes_commands": False,
                    "runtime_eligible": configuration is not None,
                    "refusal": refusal,
                    "configuration": configuration,
                    "command_file": str(args.command_file.resolve()),
                    "slam_source": SLAM_TOPIC,
                    "command_source": COMMAND_TOPIC,
                    "timeout_s": TIMEOUT,
                    "scope": "Code/interface preparation only; does not verify navigation or physical stopping",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0
    if refusal:
        print(refusal)
        return 2
    # Initialize ROS after ordinary configuration parsing.
    writer = CommandFile(args.command_file)
    node = None
    try:
        import rclpy

        rclpy.init(args=ros_args)
        node = make_node(rclpy, writer, args.configuration, configuration)
        node.tick()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            from embodied_navigation.common.runtime_io import bounded_shutdown_signals

            with bounded_shutdown_signals():
                node.shutdown_stop()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    finally:
        from embodied_navigation.common.runtime_io import bounded_shutdown_signals

        with bounded_shutdown_signals():
            writer.close()
        if _r118_source_age.ENABLED:
            _r118_source_age.finish(args.command_file.parent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
