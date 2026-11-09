"""Independent real-SLAM navigation actor. Never imports StudentRuntime."""

import argparse, json, sys, time, signal, os
from pathlib import Path
import numpy as np
from . import native_loop as loop
from embodied_navigation.robot.unitree.robogauge.runtime import Runtime
from embodied_navigation.robot.unitree.robogauge.contract import observation
from embodied_navigation.navigation.robogauge.commands import read_command
from embodied_navigation.control.safety.contact_mailbox import ContactPublisher


def main(argv=None):
    """Bind the run scope, then drive the shared native loop with real SLAM commands."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--socket", required=True)
    ap.add_argument("--model", type=Path)
    a = ap.parse_args(argv)
    run = a.run
    scope = json.loads((run / "runtime_context.json").read_text())
    p = scope["profile"]
    if p["localization_mode"] != "fast_livo2_async":
        raise ValueError("RoboGauge Gazebo worker requires real SLAM mode")
    safety = ContactPublisher(os.environ["BUILDING_SAFETY_SOCKET"])
    runtime = Runtime(a.model)
    runtime.reset()
    recv = loop.recv_exact
    publish0 = loop.atomic_json
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
                        "source": "Gazebo native state for offline scoring only; no navigation or COM policy feedback",
                    }
                )
                + "\n"
            )
        return d

    def request(test, t):
        cmd, record, error = read_command(
            run / "truth_navigation_command.json", run_id=run.name, source_ns=latest.get("ns", 0)
        )
        evidence.clear()
        evidence.update(rejected=error is not None, reason=error)
        latest["record"] = record
        return cmd, "slam_route_tracking" if np.any(cmd) else "stop_transition"

    def prepare(cmd, requested, clock, unused=None):
        loop.prepare_velocity_input(cmd, requested, clock)
        if not np.any(requested):
            cmd[:] = 0.0
        return None

    def finalize(row):
        safety.publish(
            round(row["world_sim_time"] * 1e9),
            [row["contacts"][key] for key in ["body", "FR", "FL", "RR", "RL"]],
            row["fault"] is not None,
        )
        row.update(
            actor_kind="robogauge_moe_cts",
            navigation_ground_truth_used=False,
            command_source="FAST_LIVO2 -> native SCAN -> measured path feedback; direct body velocity policy command, no COM PID",
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
                    "navigation_ground_truth_used": False,
                    "receipt_stage": "pre-send inference receipt, requires matching actuator policy_exchange and physics_step",
                }
            )
            + "\n"
        )
        return row

    def publish(path, obj):
        if path.name in ("policy_manifest.json", "worker_result.json"):
            obj.update(runtime.metadata())
            obj.update(navigation_ground_truth_used=False)
        publish0(path, obj)

    try:
        loop.main(
            actor_loader=lambda: runtime,
            observation_builder=observation,
            command_source=request,
            receive=receive,
            publish=publish,
            finalize=finalize,
            prepare=prepare,
            runtime_duration=p["duration_s"],
            termination_requested=lambda: stopping,
            argv=["--run", str(run), "--socket", a.socket],
        )
    except KeyboardInterrupt:
        pass
    finally:
        log.close()
        ack.close()
        safety.close()


if __name__ == "__main__":
    main()
