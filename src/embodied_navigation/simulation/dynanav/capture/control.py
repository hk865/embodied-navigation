"""Route/command control for the capture probe.

This module owns the 50 Hz control tick and the terminal/parking logic: native
state assembly, stand gates, approved-command consumption with freshness holds,
route trackers (native truth or interactive measured-SLAM waypoints), and the
policy action call. Thresholds, gate values, timeout windows and journal payloads
are unchanged.
"""

import json
import os
import pathlib
import time

import numpy as np


def resolve_limits(experiment):
    """Resolve the target and physical speed bounds from the experiment."""
    target_speed = float(experiment.get("speed_mps", 0.5))
    physical_speed_guard = float(experiment.get("physical_speed_guard_mps", 0.65))
    return target_speed, physical_speed_guard


def create_route_controller(
    experiment,
    *,
    gt_diagnostic,
    interactive_control=False,
    target_speed,
    spawn,
):
    """Create the native-pose or interactive-waypoint tracker.

    interactive_control is the explicit simulation-only exploration mode: the
    operator publishes waypoints and only a measured estimator pose may drive
    them.
    """
    tracker = None
    if interactive_control:
        from embodied_navigation.simulation.dynanav.native_waypoints import (
            NativeWaypointTracker,
        )

        if experiment.get("interactive_controller") == "pct_scan":
            from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker
            tracker = PctScanWaypointTracker(experiment, speed_mps=target_speed)
        else:
            tracker = NativeWaypointTracker(experiment, speed_mps=target_speed)
    if gt_diagnostic:
        from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker

        tracker = RouteTracker(
            experiment["waypoints_xyz"],
            spawn[:2].tolist(),
            speed=target_speed,
            motion_end=experiment["motion_end_s"],
            stop_on_stall=experiment.get("stop_on_stall", True),
        )
    return tracker


def annotate_policy_receipt(
    result, experiment, gt_diagnostic, robogauge, target_speed,
    physical_speed_guard, interactive_control=False
):
    """Record the unchanged speed_test/policy receipts before walking."""
    is_truth = experiment.get("slam_pose_source") == "simulation_truth"
    result["speed_test"] = {
        "target_mps": target_speed,
        "physical_speed_guard_mps": physical_speed_guard,
        "tilt_guard_rad": 0.5,
        "yaw_command_limit_radps": 0.15,
        "stair_specific_speed_limit": None,
        "native_COM_velocity_PID": not robogauge,
        "command_slew": not robogauge,
        "interactive_waypoint_interface": interactive_control,
        "interactive_command_source": (
            (
                "pct_global_scan_local_over_simulation_truth"
                if is_truth
                else "pct_global_scan_local_over_live_post_vio"
            )
            if experiment.get("interactive_controller") == "pct_scan"
            else (
                "native_route_tracker_over_simulation_truth_pose"
                if is_truth
                else "native_route_tracker_over_live_measured_pose"
            )
        )
        if interactive_control
        else None,
    }
    result["policy_loaded"] = True
    result["policy"].update(
        physics_hz=200,
        policy_hz=50,
        command_source="native_pose_diagnostic"
        if gt_diagnostic
        else "interactive_waypoint_exploration"
        if interactive_control
        else "/dynanav/nav/approved_cmd",
        ground_truth_navigation=gt_diagnostic or is_truth,
        SLAM_control_enabled=not gt_diagnostic and not is_truth,
        state_time_semantics="Isaac post-step actual measurement time",
        contacts="native foot spheres; nonfoot contacts reported separately",
    )


class ControlLoop:
    """Own the 50 Hz control tick, route tracker and stop/parking state."""

    def __init__(
        self,
        *,
        experiment,
        gt_diagnostic,
        interactive_control=False,
        robogauge,
        target_speed,
        indices,
        target,
        tracker,
        policy,
        receipts,
        latest_nav,
        latest_odom,
    ):
        self.experiment = experiment
        self.gt_diagnostic = gt_diagnostic
        self.interactive_control = interactive_control
        self.robogauge = robogauge
        self.target_speed = target_speed
        self.indices = indices
        self.target = target
        self.tracker = tracker
        self.policy = policy
        self.receipts = receipts
        self.latest_nav = latest_nav
        self.latest_odom = latest_odom
        self.stand_gate_done = False
        self.policy_stand_gate_done = False
        self.odometry_hold_started = None
        self.source_terminal_reason = None
        self.reading = {}
        self.reading_wall = None
        self.stop_detected_at = None
        self.park_since = None

    def observe_imu(self, reading):
        """Keep the previous IMU reading and its real arrival for the gyro gate."""
        self.reading = reading
        self.reading_wall = time.perf_counter()

    def tick(
        self,
        *,
        iteration,
        q,
        qd,
        art,
        rows,
        contact_state,
        previous_completed_tau,
        ros,
        policy_frames,
    ):
        """Run one 50 Hz control tick exactly as the original loop body."""
        from isaacsim.core.simulation_manager import SimulationManager
        from embodied_navigation.robot.unitree.policy.native.observation import (
            quaternion_rotation,
        )

        i = iteration
        experiment = self.experiment
        gt_diagnostic = self.gt_diagnostic
        interactive_control = self.interactive_control
        robogauge = self.robogauge
        target_speed = self.target_speed
        indices = self.indices
        target = self.target
        tracker = self.tracker
        policy = self.policy
        qdefault = policy.qdefault
        result = self.receipts.result
        journal = self.receipts.journal
        save = self.receipts.save
        latest_nav = self.latest_nav
        latest_odom = self.latest_odom
        reading = self.reading
        stand_gate_done = self.stand_gate_done
        policy_stand_gate_done = self.policy_stand_gate_done
        odometry_hold_started = self.odometry_hold_started
        source_terminal_reason = self.source_terminal_reason
        if i % 4 == 0:
            for _callback_drain in range(8):
                ros.spin_once(0.0)
            p0, r0 = art.get_world_poses()
            v0, w0 = art.get_velocities()
            pos0 = p0.numpy()[0]
            quat0 = r0.numpy()[0]
            R = quaternion_rotation(quat0)
            state = np.zeros(64)
            state[0] = float(SimulationManager.get_simulation_time())
            state[1:4] = pos0
            state[4:8] = quat0
            state[8:11] = R.T @ v0.numpy()[0]
            state[11:14] = R.T @ w0.numpy()[0]
            state[14:26] = q[indices]
            state[26:38] = qd[indices]
            state[38:50] = previous_completed_tau
            state[50:55] = contact_state
            tcontrol = i * 0.005
            if tcontrol >= 2.0 and not stand_gate_done:
                rr = rows[-100:]
                vv = np.array([r["root_velocity_world"][0][0] for r in rr])
                ww = np.array([r["root_velocity_world"][1][0] for r in rr])
                pp = np.array([r["root_pose_wxyz"][0] for r in rr])
                cc = np.array([r["contacts"] for r in rr])
                gate = {
                    "v95": float(np.percentile(np.linalg.norm(vv, axis=1), 95)),
                    "omega95": float(np.percentile(np.linalg.norm(ww, axis=1), 95)),
                    "xy_span_m": float(np.linalg.norm(np.ptp(pp[:, :2], axis=0))),
                    "body_contact_max": int(cc[:, 0].max()),
                    "support_fraction": float(
                        (np.sum(cc[:, 1:] > 0, axis=1) >= 3).mean()
                    ),
                }
                result["stand_gate"] = gate
                save()
                assert (
                    gate["v95"] < 0.02
                    and gate["omega95"] < 0.05
                    and gate["xy_span_m"] < 0.02
                    and gate["body_contact_max"] == 0
                    and gate["support_fraction"] > 0.9
                ), "Stand gate failed; refuse walking"
                stand_gate_done = True
            if robogauge and tcontrol >= 6.0 and not policy_stand_gate_done:
                rr = rows[-200:]
                vv = np.array([r["root_velocity_world"][0][0] for r in rr])
                ww = np.array([r["root_velocity_world"][1][0] for r in rr])
                pp = np.array([r["root_pose_wxyz"][0] for r in rr])
                cc = np.array([r["contacts"] for r in rr])
                gate = {
                    "v95": float(np.percentile(np.linalg.norm(vv, axis=1), 95)),
                    "omega95": float(np.percentile(np.linalg.norm(ww, axis=1), 95)),
                    "xy_span_m": float(np.linalg.norm(np.ptp(pp[:, :2], axis=0))),
                    "body_contact_max": int(cc[:, 0].max()),
                    "support_fraction": float(
                        (np.sum(cc[:, 1:] > 0, axis=1) >= 3).mean()
                    ),
                }
                result["policy_stand_gate"] = gate
                save()
                assert (
                    gate["v95"] < 0.02
                    and gate["omega95"] < 0.05
                    and gate["xy_span_m"] < 0.02
                    and gate["body_contact_max"] == 0
                    and gate["support_fraction"] > 0.9
                ), "RoboGauge policy stand gate failed"
                policy_stand_gate_done = True
            request = np.zeros(3)
            if (
                not gt_diagnostic
                and not interactive_control
                and 7 <= tcontrol < float(os.environ.get("NAV_END_SIM", "38"))
                and latest_nav[0] is not None
                and state[0] - latest_nav[0]["time"] < 0.25
                and time.perf_counter() - latest_nav[0]["arrival_wall"] < 0.25
            ):
                request = np.array(latest_nav[0]["request"])
            if tracker is not None:
                if interactive_control:
                    odom = latest_odom[0] or {}
                    gyro = locals().get("reading", {}).get("angular_velocity")
                    tracker.observe_native_guards(
                        tilt_rad=float(np.arccos(np.clip(R[2, 2], -1.0, 1.0))),
                        native_speed_mps=float(np.linalg.norm(v0.numpy()[0])),
                    )
                    measured_command, control_receipt = tracker.tick(
                        state[0],
                        odom.get("time"),
                        odom.get("position"),
                        odom.get("orientation_wxyz"),
                        odom.get("estimated_velocity"),
                        float(gyro[2])
                        if gyro is not None and len(gyro) == 3
                        else None,
                        ready=policy_stand_gate_done,
                        now_wall=time.perf_counter(),
                        arrival_wall=odom.get("arrival_wall"),
                        frame_id=odom.get("frame_id"),
                        child_frame_id=odom.get("child_frame_id"),
                        pose_source=odom.get("pose_source"),
                        imu_source_stamp_s=(
                            locals().get("reading", {}).get("time")
                            if isinstance(locals().get("reading", {}), dict)
                            else None
                        ),
                        imu_arrival_wall_s=self.reading_wall,
                        imu_frame=(
                            locals().get("reading", {}).get("frame_id")
                            if isinstance(locals().get("reading", {}), dict)
                            else None
                        ),
                        imu_valid=bool(
                            gyro is not None
                            and len(gyro) == 3
                            and np.isfinite(float(gyro[2]))
                        ),
                    )
                    request = np.asarray(measured_command)
                    result["interactive_waypoint_control"] = control_receipt
                    journal("interactive_waypoint_control.jsonl", control_receipt)
                else:
                    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
                    request = np.asarray(
                        tracker.tick(
                            state[0],
                            pos0.tolist(),
                            yaw,
                            float(np.linalg.norm(v0.numpy()[0])),
                            float(w0.numpy()[0][2]),
                            ready=policy_stand_gate_done,
                        )
                    )
                result["route_tracker"] = {
                    "phase": tracker.phase,
                    "stage": tracker.stage,
                    "arrived": tracker.arrived,
                    "failure": tracker.failure,
                    "events": tracker.events,
                }
                interactive_terminal = getattr(tracker, "terminal_reason", None)
                if interactive_terminal is not None:
                    source_terminal_reason = (
                        source_terminal_reason or interactive_terminal
                    )
                elif tracker.arrived or tracker.failure:
                    source_terminal_reason = (
                        "route_arrived" if tracker.arrived else tracker.failure
                    )
            assert np.isfinite(request).all() and np.all(
                np.abs(request) <= [target_speed + 1e-6, 0.050001, 0.150001]
            ), "Invalid navigation command"
            if (
                not gt_diagnostic
                and not interactive_control
                and 7 <= tcontrol < float(os.environ.get("NAV_END_SIM", "210"))
                and (np.any(request) or odometry_hold_started is not None)
            ):
                for _freshness_drain in range(4):
                    ros.spin_once(0.0)
                odom_wall_age = (
                    time.perf_counter() - latest_odom[0]["arrival_wall"]
                    if latest_odom[0]
                    else float("inf")
                )
                odom_sim_age = (
                    state[0] - latest_odom[0]["time"]
                    if latest_odom[0]
                    else float("inf")
                )
                fresh = bool(
                    latest_odom[0] is not None
                    and odom_sim_age < 0.25
                    and odom_wall_age < 0.5
                )
                result["last_freshness_check"] = {
                    "physics_s": state[0],
                    "odom_s": latest_odom[0]["time"] if latest_odom[0] else None,
                    "wall_age_s": odom_wall_age,
                    "sim_age_s": odom_sim_age,
                    "fresh": fresh,
                }
                save()
                if not fresh:
                    request[:] = 0.0
                    if odometry_hold_started is None:
                        odometry_hold_started = time.perf_counter()
                        result["freshness_hold_events"].append(
                            {
                                "event": "hold_started",
                                "physics_s": state[0],
                                "wall_age_s": odom_wall_age,
                                "sim_age_s": odom_sim_age,
                            }
                        )
                    if (
                        time.perf_counter() - odometry_hold_started > 2.0
                        and source_terminal_reason is None
                    ):
                        source_terminal_reason = "persistent_odometry_staleness"
                        result["source_terminal_reason"] = source_terminal_reason
                elif odometry_hold_started is not None:
                    result["freshness_hold_events"].append(
                        {
                            "event": "freshness_recovered",
                            "physics_s": state[0],
                            "hold_wall_seconds": time.perf_counter()
                            - odometry_hold_started,
                        }
                    )
                    odometry_hold_started = None
            if pathlib.Path(
                os.environ.get("NAV_STOP_REQUEST", "/tmp/no_multistage_stop_request")
            ).exists():
                source_terminal_reason = (
                    source_terminal_reason or "external_stop_request"
                )
            if source_terminal_reason is not None:
                request[:] = 0.0
            ac, obs, hidden_norm, qc = policy.step(i, state, request)
            target[indices] = qc
            cmd = policy.cmd
            policy_frames.append(
                {
                    "measured_time": state[0],
                    "obs45": obs.tolist(),
                    "action": ac.tolist(),
                    "q_target_canonical": qc.tolist(),
                    "hidden_norm": hidden_norm,
                    "request": request.tolist(),
                    "navigation_source": latest_odom[0]
                    if interactive_control
                    else latest_nav[0],
                    "navigation_pose_source": experiment.get(
                        "navigation_pose_source", "measured_lio"
                    )
                    if interactive_control
                    else "native_truth"
                    if gt_diagnostic
                    else "ROS_approved_command",
                    "actual_command": cmd.tolist(),
                    "native_state64": state.tolist(),
                }
            )
            journal("policy_frames.jsonl", policy_frames[-1])

        self.stand_gate_done = stand_gate_done
        self.policy_stand_gate_done = policy_stand_gate_done
        self.odometry_hold_started = odometry_hold_started
        self.source_terminal_reason = source_terminal_reason

    def check_termination(self, *, iteration, actual_time, v, w):
        """Apply the unchanged stop-request/parking termination logic."""
        i = iteration
        tracker = self.tracker
        result = self.receipts.result
        save = self.receipts.save
        source_terminal_reason = self.source_terminal_reason
        stop_detected_at = self.stop_detected_at
        park_since = self.park_since
        stop_path = pathlib.Path(
            os.environ.get("NAV_STOP_REQUEST", "/tmp/no_multistage_stop_request")
        )
        if (i + 1) % 20 == 0 and (
            stop_path.exists() or source_terminal_reason is not None
        ):
            if stop_detected_at is None:
                stop_detected_at = actual_time
                result["navigation_terminal_request"] = (
                    json.loads(stop_path.read_text())
                    if stop_path.exists()
                    else {
                        "failure": None
                        if tracker is not None and tracker.arrived
                        else source_terminal_reason,
                        "arrived": bool(tracker is not None and tracker.arrived),
                    }
                )
                save()
            if (
                float(np.linalg.norm(v.numpy()[0])) < 0.02
                and float(np.linalg.norm(w.numpy()[0])) < 0.05
            ):
                if park_since is None:
                    park_since = actual_time
            else:
                park_since = None
            if (
                actual_time - stop_detected_at >= 5.0
                and park_since is not None
                and actual_time - park_since >= 1.0
            ):
                result["end_reason"] = "navigation_terminal_and_parked"
                return True

        self.stop_detected_at = stop_detected_at
        self.park_since = park_since
        return False
