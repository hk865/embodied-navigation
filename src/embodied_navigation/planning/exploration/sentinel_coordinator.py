"""Operator-side closed-loop coordinator tying async LLM/VLM sentinel to mailbox actions."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

from embodied_navigation.planning.exploration.llm_agent_client import (
    ActiveSlamSpinTracker,
    AsyncDecisionRunner,
    SentinelVerdict,
)
from embodied_navigation.planning.exploration import waypoint_mailbox


class SentinelCoordinator:
    """Bridge observations, active-SLAM health, async sentinel verdicts, and mailbox actions."""

    def __init__(
        self,
        mailbox: str | Path,
        runner: AsyncDecisionRunner,
        *,
        pose_source: str = "livo_post_vio",
        run_id: str | None = None,
        spin_delta_rad: float = math.pi,
        max_travel_since_loop_m: float = 20.0,
        max_loop_age_s: float = 180.0,
        spin_tracker: ActiveSlamSpinTracker | None = None,
        ttl_s: float = 30.0,
        yaw_rate_rps: float | None = None,
        max_yaw_rate_rps: float | None = None,
        memory_store: Any | None = None,
    ) -> None:
        if not math.isfinite(spin_delta_rad) or spin_delta_rad == 0.0 or abs(spin_delta_rad) > 3.15:
            raise ValueError("spin_delta_rad must be nonzero and within [-3.15, 3.15]")
        self.mailbox = Path(mailbox)
        self.runner = runner
        self.pose_source = pose_source
        self.run_id = run_id
        self.spin_delta_rad = float(spin_delta_rad)
        self.max_travel_since_loop_m = float(max_travel_since_loop_m)
        self.max_loop_age_s = float(max_loop_age_s)
        self.spin_tracker = spin_tracker or ActiveSlamSpinTracker()
        self.ttl_s = float(ttl_s)
        self.yaw_rate_rps = yaw_rate_rps
        self.max_yaw_rate_rps = max_yaw_rate_rps
        self.memory_store = memory_store

        self._frame_seq = 0
        self._action_epoch = 0
        self._candidates_by_frame: dict[int, dict[str, dict[str, Any]]] = {}
        self._pose_z_by_frame: dict[int, float] = {}
        self._last_backend_status: dict[str, Any] | None = None
        self._last_slam_health: dict[str, Any] | None = None
        self._in_flight_spin_seq: int | None = None

    @property
    def action_epoch(self) -> int:
        return self._action_epoch

    @property
    def frame_seq(self) -> int:
        return self._frame_seq

    def _sync_from_observation(self, observation: dict[str, Any] | None) -> float:
        if not isinstance(observation, dict):
            return 0.0
        waypoints = observation.get("waypoints")
        if isinstance(waypoints, dict):
            obs_epoch = waypoints.get("current_epoch")
            if obs_epoch is None:
                obs_epoch = waypoints.get("action_epoch")
            if isinstance(obs_epoch, int) and not isinstance(obs_epoch, bool) and obs_epoch > self._action_epoch:
                self._action_epoch = obs_epoch
            if self._in_flight_spin_seq is not None:
                active_seq = waypoints.get("active_seq")
                phase = waypoints.get("phase")
                last_result = waypoints.get("last_result")
                if (
                    (active_seq != self._in_flight_spin_seq and phase in ("idle", "terminal"))
                    or (isinstance(last_result, dict) and last_result.get("seq") == self._in_flight_spin_seq)
                ):
                    self.spin_tracker.mark_spin_finished()
                    self._in_flight_spin_seq = None
        pose = observation.get("pose")
        if isinstance(pose, dict):
            xyz = pose.get("position_xyz_m")
            if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
                try:
                    z = float(xyz[2])
                    if math.isfinite(z):
                        return z
                except (TypeError, ValueError):
                    pass
        return 0.0

    def submit_observation(
        self,
        observation: dict[str, Any] | None = None,
        candidates: Iterable[dict[str, Any]] | None = None,
        backend_status: dict[str, Any] | None = None,
        *,
        rgb_path: str | Path | None = None,
        local_map_path: str | Path | None = None,
        now_s: float | None = None,
    ) -> dict[str, Any]:
        """Build/submit one observation frame to the async sentinel runner."""
        obs = (
            observation
            if observation is not None
            else waypoint_mailbox.read_observation(self.mailbox)
        )
        pose_z = self._sync_from_observation(obs)
        health = self.spin_tracker.assess(
            backend_status,
            max_travel_since_loop_m=self.max_travel_since_loop_m,
            max_loop_age_s=self.max_loop_age_s,
            now_s=now_s,
        )
        self._last_backend_status = (
            dict(backend_status) if isinstance(backend_status, dict) else None
        )
        self._last_slam_health = health

        candidate_list = list(candidates) if candidates is not None else []
        blocked_candidates: list[dict[str, Any]] = []
        obs_payload: dict[str, Any] = dict(obs) if isinstance(obs, dict) else {}
        if self.memory_store is not None:
            allowed_cands, blocked_candidates = self.memory_store.filter_candidates(
                candidate_list, default_z=pose_z
            )
            candidate_list = allowed_cands
            if blocked_candidates and not allowed_cands:
                obs_payload["all_candidates_blacklisted"] = True
                obs_payload["blacklisted_count"] = len(blocked_candidates)
            obs_payload["memory_l0"] = self.memory_store.l0_global_topology()

        cand_map: dict[str, dict[str, Any]] = {}
        for item in candidate_list:
            if not isinstance(item, dict):
                continue
            cid = item.get("id")
            if isinstance(cid, str) and cid.strip():
                cand_map[cid.strip()] = dict(item)

        self._frame_seq += 1
        f_seq = self._frame_seq
        self._candidates_by_frame[f_seq] = cand_map
        self._pose_z_by_frame[f_seq] = pose_z
        if len(self._candidates_by_frame) > 16:
            oldest = min(self._candidates_by_frame)
            self._candidates_by_frame.pop(oldest, None)
            self._pose_z_by_frame.pop(oldest, None)

        submitted = self.runner.submit(
            {
                "observation_summary": obs_payload,
                "candidates": candidate_list,
                "backend_health": health,
                "rgb_path": rgb_path,
                "local_map_path": local_map_path,
            },
            frame_seq=f_seq,
            action_epoch=self._action_epoch,
            candidate_ids=list(cand_map.keys()),
        )
        return {
            "frame_seq": f_seq,
            "action_epoch": self._action_epoch,
            "slam_health": health,
            "blacklisted_candidates": len(blocked_candidates),
            "has_rgb": bool(rgb_path),
            "has_memory_l0": "memory_l0" in obs_payload,
            "submitted": submitted,
        }

    def _resolve_candidate_goal(
        self, frame_seq: int | None, target_id: str | None
    ) -> list[float] | None:
        if frame_seq is None or not target_id:
            return None
        candidate = self._candidates_by_frame.get(frame_seq, {}).get(target_id)
        if candidate is None:
            return None
        raw_goal = candidate.get("goal_xyz_m")
        if isinstance(raw_goal, (list, tuple)) and len(raw_goal) == 3:
            xyz = [float(v) for v in raw_goal]
            return xyz if all(math.isfinite(v) for v in xyz) else None
        raw_xy = candidate.get("xy")
        if isinstance(raw_xy, (list, tuple)) and len(raw_xy) >= 2:
            raw_z = candidate.get("z", self._pose_z_by_frame.get(frame_seq, 0.0))
            xyz = [float(raw_xy[0]), float(raw_xy[1]), float(raw_z)]
            return xyz if all(math.isfinite(v) for v in xyz) else None
        return None

    def poll_and_dispatch(
        self,
        *,
        now_s: float | None = None,
        wait_timeout_s: float | None = None,
    ) -> dict[str, Any] | None:
        """Consume latest fresh sentinel verdict and submit any preemptive mailbox action."""
        verdict: SentinelVerdict | None = self.runner.poll_latest()
        if verdict is None:
            return None

        if verdict.action_epoch is not None and verdict.action_epoch < self._action_epoch:
            return {
                "verdict": verdict,
                "dispatched": False,
                "reason": "stale_action_epoch",
                "payload": None,
            }

        if verdict.act == "CONT":
            return {
                "verdict": verdict,
                "dispatched": False,
                "reason": verdict.reason,
                "payload": None,
            }

        payload: dict[str, Any] | None = None
        if verdict.act == "STOP":
            self._action_epoch += 1
            payload = waypoint_mailbox.submit(
                self.mailbox,
                action="stop",
                pose_source=self.pose_source,
                run_id=self.run_id,
                extra={"action_epoch": self._action_epoch},
            )
        elif verdict.act == "SPIN":
            health = self._last_slam_health
            if self.spin_tracker.in_flight_spin or (
                isinstance(health, dict)
                and health.get("status") == "needs_active_loop"
                and health.get("should_trigger_spin") is False
            ):
                return {
                    "verdict": verdict,
                    "dispatched": False,
                    "reason": (
                        health.get("suppressed_reason", "spin_deduplicated")
                        if isinstance(health, dict)
                        else "spin_in_flight"
                    ),
                    "payload": None,
                }
            self._action_epoch += 1
            extra: dict[str, Any] = {
                "delta_yaw_rad": self.spin_delta_rad,
                "preempt": True,
                "action_epoch": self._action_epoch,
            }
            if self.yaw_rate_rps is not None:
                extra["yaw_rate_rps"] = self.yaw_rate_rps
            if self.max_yaw_rate_rps is not None:
                extra["max_yaw_rate_rps"] = self.max_yaw_rate_rps
            payload = waypoint_mailbox.submit(
                self.mailbox,
                action="yaw",
                ttl_s=self.ttl_s,
                pose_source=self.pose_source,
                run_id=self.run_id,
                extra=extra,
            )
            status_dict = self._last_backend_status or {}
            keyframes = status_dict.get("keyframes")
            loops = status_dict.get("loops")
            travel_m = health.get("travel_since_last_loop_m") if isinstance(health, dict) else None
            self.spin_tracker.mark_spin_started(
                keyframes=keyframes if isinstance(keyframes, int) else None,
                loops=loops if isinstance(loops, int) else None,
                travel_m=travel_m,
                now_s=now_s,
            )
            self._in_flight_spin_seq = payload["seq"]
        elif verdict.act == "PREEMPT":
            if verdict.fallback:
                return {
                    "verdict": verdict,
                    "dispatched": False,
                    "reason": "fallback_verdict",
                    "payload": None,
                }
            goal_xyz = self._resolve_candidate_goal(verdict.frame_seq, verdict.target_id)
            if goal_xyz is None:
                return {
                    "verdict": verdict,
                    "dispatched": False,
                    "reason": "unknown_or_stale_candidate",
                    "payload": None,
                }
            if self.memory_store is not None and self.memory_store.match_blacklist(goal_xyz):
                return {
                    "verdict": verdict,
                    "dispatched": False,
                    "reason": "blacklisted_goal",
                    "payload": None,
                }
            self._action_epoch += 1
            payload = waypoint_mailbox.submit(
                self.mailbox,
                action="goto",
                goal=goal_xyz,
                ttl_s=self.ttl_s,
                pose_source=self.pose_source,
                run_id=self.run_id,
                extra={"preempt": True, "action_epoch": self._action_epoch},
            )
        else:
            return {
                "verdict": verdict,
                "dispatched": False,
                "reason": f"unsupported_act:{verdict.act}",
                "payload": None,
            }

        result: dict[str, Any] = {
            "verdict": verdict,
            "dispatched": True,
            "payload": payload,
        }
        if wait_timeout_s is not None and payload is not None:
            receipt = waypoint_mailbox.wait_for_action(
                self.mailbox, payload["seq"], wait_timeout_s, poll_s=0.01
            )
            result["receipt"] = receipt
            if verdict.act == "SPIN":
                self.spin_tracker.mark_spin_finished()
                self._in_flight_spin_seq = None
        return result
