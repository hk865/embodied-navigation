"""SLAM-only Student simulation adapter; legacy truth wire names do not grant truth navigation."""

import argparse
import json
import sys
import time
import signal
from pathlib import Path
import numpy as np
from embodied_navigation.robot.unitree.policy.native import worker as legacy
from embodied_navigation.robot.unitree.policy.student import StudentRuntime
from embodied_navigation.control.tracking.policy_selection import record_actual_loaded
from embodied_navigation.control.safety.contact_mailbox import ContactPublisher
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--socket", required=True)
    a = ap.parse_args()
    run = a.run
    scope = json.loads((run / "runtime_context.json").read_text())
    p = scope["profile"]
    slam = p.get("localization_mode") == "fast_livo2_async"
    if not slam:
        raise ValueError("Production Building Student requires fast_livo2_async navigation")
    safety = ContactPublisher(os.environ["BUILDING_SAFETY_SOCKET"]) if slam else None
    student = StudentRuntime(
        run, scope, record_actual_loaded(run, legacy)["selected_binding"], np.array([0.6, 0.2, 0.3])
    )
    recv = legacy.recv_exact
    publish0 = legacy.atomic_json
    prepare0 = legacy.prepare_velocity_input
    evidence = {}
    latest = {}
    log = (run / "native_pose_source.jsonl").open("w", buffering=1)
    ack = (run / "actual_command_ack.jsonl").open("w", buffering=1)
    stopping = False

    def request_stop(*unused):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGUSR1, request_stop)

    def receive(conn, n):
        d = recv(conn, n)
        if n == 512:
            s = np.frombuffer(d, dtype="<f8")
            student.native_frame(s)
            latest["ns"] = round(float(s[0]) * 1e9)
            log.write(
                json.dumps(
                    {
                        "stamp_ns": round(float(s[0] - 0.005) * 1e9),
                        "world_sim_s": float(s[0]),
                        "position": s[1:4].tolist(),
                        "quaternion_wxyz": s[4:8].tolist(),
                        "velocity_body_com": s[8:11].tolist(),
                        "angular_velocity_body": s[11:14].tolist(),
                        "contacts": s[50:55].tolist(),
                        "source": "Gazebo native state for offline scoring only"
                        if slam
                        else "Gazebo native previous Physics state, explicitly authorized truth navigation",
                    }
                )
                + "\n"
            )
        return d

    def request(test, t):
        error = None
        record = {}
        cmd = np.zeros(3)
        try:
            record = json.loads((run / "truth_navigation_command.json").read_text())
            v = np.asarray(record["command"], float)
            age = (latest["ns"] - record["stamp_ns"]) / 1e9
            wall = time.monotonic() - record["monotonic_wall"]
            if (
                record["run_id"] != run.name
                or v.shape != (3,)
                or not np.isfinite(v).all()
                or np.any(abs(v) > [0.6, 0.2, 0.3])
                or not (-0.05 <= age < 0.3)
                or not (0 <= wall < 0.3)
            ):
                raise ValueError("command identity/bounds/freshness")
            cmd = v
        except (OSError, ValueError, KeyError, TypeError) as e:
            error = str(e)
        evidence.clear()
        evidence.update(
            rejected=error is not None,
            reason=error,
            actual_original_envelope={
                "controller_source": {
                    "reference_kind": "slam_SCAN_feedback/v1" if slam else "truth_SCAN_feedback/v1",
                    "request_id": run.name + ":truth_route",
                    "waypoint_index": int(record.get("goal_index", 0)),
                }
            },
        )
        latest["record"] = record
        return cmd, "truth_route_tracking" if np.any(cmd) else "stop_transition"

    def prepare(cmd, request, clock, unused=None):
        def zero_adapter(cmd, requested, clock):
            if not np.any(requested):
                cmd[:] = 0.0
                return {"observation_command_exact_zero": True, "input_clock_ns": clock}

        return student.prepare_command(
            cmd, request, clock, zero_adapter, evidence, run.name, prepare0
        )

    def observation(*args, **kwargs):
        o, x = legacy.build_observation(*args, **kwargs)
        student.observation_frame(args[0], args[1], args[2], o, False)
        return o, x

    def finalize(row):
        if safety is not None:
            safety.publish(
                round(row["world_sim_time"] * 1e9),
                [row["contacts"][key] for key in ["body", "FR", "FL", "RR", "RL"]],
                row["fault"] is not None,
            )
        row = student.telemetry(dict(row))
        row.update(
            navigation_ground_truth_used=not slam,
            command_source="FAST_LIVO2 -> native SCAN -> measured path feedback"
            if slam
            else "Actual Gazebo truth -> PCT route -> native SCAN -> measured path feedback",
            navigation_command_rejected=evidence.get("rejected"),
            navigation_command_reason=evidence.get("reason"),
        )
        ack.write(
            json.dumps(
                {
                    "stamp_ns": round(row["world_sim_time"] * 1e9),
                    "command": row["command"],
                    "requested": row["requested"],
                    "goal_index": latest.get("record", {}).get("goal_index"),
                    "command_sequence": latest.get("record", {}).get("sequence"),
                    "actor_inferred": row["actor_inferred_this_frame"],
                    "fault": row["fault"],
                    "native_effective_source_ns": round(row["state_physics_world_time"] * 1e9),
                    "navigation_ground_truth_used": not slam,
                }
            )
            + "\n"
        )
        return row

    def publish(path, obj):
        if path.name == "policy_manifest.json":
            obj.update(student.metadata())
            obj.update(
                navigation_truth_used=not slam,
                navigation_ground_truth_used=not slam,
                source="FAST_LIVO2 navigation with privileged COM speed inner loop"
                if slam
                else "Gazebo truth route navigation isolation",
                Student_input_dimension=45,
                Student_terrain_input=False,
                existing_native_COM_velocity_PID=True,
            )
        if path.name == "worker_result.json":
            obj.update(
                navigation_ground_truth_used=not slam, Student_model_path=str(student.model_path)
            )
        publish0(path, obj)

    sys.argv = [
        "truth_worker",
        "--run",
        str(run),
        "--socket",
        a.socket,
        "--terrain-target-manifest",
        str(run / "terrain_target_manifest.json"),
    ]
    try:
        legacy.main(
            actor_loader=lambda: student.load_actor(legacy.torch),
            observation_builder=observation,
            command_source=request,
            receive=receive,
            publish=publish,
            finalize=finalize,
            prepare=prepare,
            runtime_duration=p["duration_s"],
            termination_requested=lambda: stopping,
        )
    except KeyboardInterrupt:
        pass
    finally:
        student.close()
        log.close()
        ack.close()
        if safety is not None:
            safety.close()


if __name__ == "__main__":
    main()
