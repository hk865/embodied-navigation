"""Closed-loop static embodied task agent combining L0-L3 memory and OpenCode Go SSE streaming.

Architecture (strictly obeys the 7-layer boundary in AGENTS.md):
- Reads ONLY live sensor evidence (front RGB, annotated 2D operator maps, F*/W*
  candidates, post-VIO / backend_corrected pose, and backend_status). Never reads
  simulator ground truth or writes cmd_vel directly.
- Maintains a versioned :class:`HierarchicalMemoryStore` (L0-L3 retrieval,
  Attempt & Failure Ledger geographic blacklist, and pre-STOP geometric gate).
- Runs a main multimodal SSE streaming planner (e.g. OpenCode Go ``glm-5.3-flash``
  or ``qwen3.8-flash``) together with an optional in-motion async streaming sentinel
  (:class:`SentinelCoordinator` + :class:`AsyncDecisionRunner`) that can preempt
  active goals via ``action_epoch``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import http.client
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Callable, Sequence
import urllib.parse

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
    build_scene_graph_from_map_dir,
)
from embodied_navigation.planning.exploration import waypoint_mailbox
from embodied_navigation.planning.exploration.llm_agent_client import (
    AsyncDecisionRunner,
    FastSentinelClient,
    LlmClientConfig,
    _encode_image_part,
    _extract_choice_content,
    _extract_json_object,
    _extract_sse_delta_content,
)
from embodied_navigation.planning.exploration.sentinel_coordinator import (
    SentinelCoordinator,
)

VALID_TASK_ACTIONS = (
    "goto_candidate",
    "goto_xyz",
    "maneuver",
    "scan",
    "query_memory",
    "stop_claim",
)

_TASK_PLANNER_SYSTEM_PROMPT = (
    "You are an autonomous quadruped navigation agent operating in a static indoor "
    "multi-floor building using real SLAM poses, front RGB, annotated 2D maps, and "
    "hierarchical L0-L3 spatial memory.\n"
    "Images attached (when present): image 1 = front RGB camera; image 2 = annotated "
    "2D occupancy map with robot footprint, turning circle (green=safe to scan, red=tight), "
    "and numbered frontier/waypoint candidates (F1..F*, W1..W*).\n"
    "Rules:\n"
    "- Prefer clear, non-blacklisted F*/W* candidates (goto_candidate) to explore or approach.\n"
    "- Avoid candidates marked blacklisted unless no alternative exists.\n"
    "- If front RGB shows a doorway/wall jam or stairs to climb directly, use maneuver "
    "(bounded: forward in [-0.45, 0.50], yaw_rate in [-0.15, 0.15], duration in [0.5, 2.0]).\n"
    "- Report any salient furniture/landmark you clearly see in front RGB inside observed_objects "
    "(each item: {\"label\": \"short name\", \"dist_m\": float, \"bearing_deg\": float, \"confidence\": 0.5-1.0}; "
    "bearing_deg=0 straight ahead, +left, -right).\n"
    "- Call stop_claim ONLY when you have reached and confirmed the task target within ~1.5 m on the same floor.\n"
    "- Reply ONLY with ONE compact JSON object matching:\n"
    '{"action":"goto_candidate"|"goto_xyz"|"maneuver"|"scan"|"query_memory"|"stop_claim",'
    '"target_id":"F1"|null,"xyz":[x,y,z]|null,"forward":0.0,"yaw_rate":0.0,"duration":1.0,'
    '"query":null,"target_label":null,"observed_objects":[],"reason":"short"}'
)


def _pose_from_xyz_yaw(xyz: Sequence[float], yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    mat = np.eye(4, dtype=np.float64)
    mat[0, 0], mat[0, 1] = c, -s
    mat[1, 0], mat[1, 1] = s, c
    mat[:3, 3] = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
    return mat


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(val)))


@dataclass(frozen=True)
class TaskDecision:
    """Structured action decision produced by the streaming task planner."""

    action: str
    target_id: str | None = None
    xyz: tuple[float, float, float] | None = None
    forward: float = 0.0
    yaw_rate: float = 0.0
    duration: float = 1.0
    query: str | None = None
    target_label: str | None = None
    observed_objects: tuple[dict[str, Any], ...] = ()
    reason: str = ""
    latency_ms: float = 0.0
    fallback: bool = False
    raw_text: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "action": self.action,
            "target_id": self.target_id,
            "xyz": (None if self.xyz is None else [round(v, 3) for v in self.xyz]),
            "forward": round(self.forward, 3),
            "yaw_rate": round(self.yaw_rate, 3),
            "duration": round(self.duration, 3),
            "query": self.query,
            "target_label": self.target_label,
            "observed_objects": list(self.observed_objects),
            "reason": self.reason,
            "latency_ms": round(self.latency_ms, 1),
            "fallback": self.fallback,
        }


def _extract_outer_json_object(raw_text: str) -> dict[str, Any] | None:
    """Extract the first complete top-level JSON object (ignoring nested objects)."""
    start = raw_text.find("{")
    if start < 0:
        return None
    depth = 0
    in_str = False
    escaped = False
    for idx in range(start, len(raw_text)):
        ch = raw_text[idx]
        if in_str:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                candidate = raw_text[start : idx + 1]
                try:
                    parsed = json.loads(candidate)
                except ValueError:
                    return None
                return parsed if isinstance(parsed, dict) else None
    return None


def parse_task_decision(
    raw_text: str,
    *,
    valid_candidate_ids: Sequence[str] = (),
    latency_ms: float = 0.0,
) -> TaskDecision:
    """Parse one compact JSON task decision with fail-safe validation."""
    obj = _extract_outer_json_object(raw_text)
    if obj is None:
        obj = _extract_json_object(raw_text)
    if not isinstance(obj, dict):
        return _fallback_decision(
            valid_candidate_ids,
            reason="parse_error_fallback",
            latency_ms=latency_ms,
            raw_text=raw_text,
        )

    action = str(obj.get("action", "")).strip().lower()
    if action not in VALID_TASK_ACTIONS:
        return _fallback_decision(
            valid_candidate_ids,
            reason=f"invalid_action:{action}",
            latency_ms=latency_ms,
            raw_text=raw_text,
        )

    target_id_raw = obj.get("target_id")
    target_id = (
        str(target_id_raw).strip()
        if isinstance(target_id_raw, str) and target_id_raw.strip()
        else None
    )
    if action == "goto_candidate":
        if valid_candidate_ids and target_id not in valid_candidate_ids:
            target_id = valid_candidate_ids[0]
        elif not target_id:
            return _fallback_decision(
                valid_candidate_ids,
                reason="missing_candidate_id",
                latency_ms=latency_ms,
                raw_text=raw_text,
            )

    xyz_val: tuple[float, float, float] | None = None
    raw_xyz = obj.get("xyz")
    if isinstance(raw_xyz, (list, tuple)) and len(raw_xyz) == 3:
        try:
            nums = tuple(float(v) for v in raw_xyz)
            if all(math.isfinite(v) for v in nums):
                xyz_val = nums  # type: ignore[assignment]
        except (TypeError, ValueError):
            xyz_val = None
    if action == "goto_xyz" and xyz_val is None:
        return _fallback_decision(
            valid_candidate_ids,
            reason="invalid_goto_xyz",
            latency_ms=latency_ms,
            raw_text=raw_text,
        )

    def _safe_float(key: str, default: float) -> float:
        try:
            v = float(obj.get(key, default))
            return v if math.isfinite(v) else default
        except (TypeError, ValueError):
            return default

    observed_clean: list[dict[str, Any]] = []
    raw_obs = obj.get("observed_objects")
    if isinstance(raw_obs, list):
        for item in raw_obs[:8]:
            if not isinstance(item, dict):
                continue
            label = str(item.get("label", "")).strip()
            if not label:
                continue
            try:
                dist_m = _clamp(float(item.get("dist_m", 2.0)), 0.4, 10.0)
                bearing_deg = _clamp(float(item.get("bearing_deg", 0.0)), -75.0, 75.0)
                conf = _clamp(float(item.get("confidence", 0.8)), 0.1, 1.0)
            except (TypeError, ValueError):
                continue
            observed_clean.append(
                {
                    "label": label,
                    "dist_m": round(dist_m, 2),
                    "bearing_deg": round(bearing_deg, 1),
                    "confidence": round(conf, 2),
                }
            )

    query_raw = obj.get("query")
    target_label_raw = obj.get("target_label")
    return TaskDecision(
        action=action,
        target_id=target_id,
        xyz=xyz_val,
        forward=_clamp(_safe_float("forward", 0.0), -0.45, 0.50),
        yaw_rate=_clamp(_safe_float("yaw_rate", 0.0), -0.15, 0.15),
        duration=_clamp(_safe_float("duration", 1.2), 0.2, 2.0),
        query=(str(query_raw).strip() if isinstance(query_raw, str) and query_raw.strip() else None),
        target_label=(
            str(target_label_raw).strip()
            if isinstance(target_label_raw, str) and target_label_raw.strip()
            else None
        ),
        observed_objects=tuple(observed_clean),
        reason=str(obj.get("reason", "")).strip()[:200],
        latency_ms=latency_ms,
        fallback=False,
        raw_text=raw_text[:500],
    )


def _fallback_decision(
    valid_candidate_ids: Sequence[str],
    *,
    reason: str,
    latency_ms: float,
    raw_text: str,
) -> TaskDecision:
    if valid_candidate_ids:
        return TaskDecision(
            action="goto_candidate",
            target_id=valid_candidate_ids[0],
            reason=reason,
            latency_ms=latency_ms,
            fallback=True,
            raw_text=raw_text[:500],
        )
    return TaskDecision(
        action="scan",
        reason=reason,
        latency_ms=latency_ms,
        fallback=True,
        raw_text=raw_text[:500],
    )


class OpenCodeTaskPlanner:
    """Multimodal SSE-streaming task planner backed by OpenCode Go / OpenAI-compatible API."""

    def __init__(self, config: LlmClientConfig) -> None:
        self.config = config
        self._latencies_ms: list[float] = []

    def build_messages(
        self,
        *,
        task_instruction: str,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        candidates: Sequence[dict[str, Any]],
        memory_l0: dict[str, Any],
        memory_l1: Sequence[dict[str, Any]],
        memory_l2: dict[str, Any],
        last_feedback: str = "",
        rgb_path: str | Path | None = None,
        local_map_path: str | Path | None = None,
        extra_rgb_paths: Sequence[str | Path] = (),
    ) -> list[dict[str, Any]]:
        compact_cands = [
            {
                "id": c.get("id"),
                "kind": c.get("kind"),
                "xy": c.get("xy"),
                "z": c.get("z"),
                "dist_m": c.get("dist_m"),
                "clearance_m": c.get("clearance_m"),
                "turn_safe": c.get("turn_safe"),
                "blacklisted": c.get("blacklisted", False),
            }
            for c in list(candidates)[:10]
        ]
        state_obj = {
            "task": task_instruction,
            "step": step_index,
            "robot_xyz": [round(float(v), 2) for v in robot_xyz],
            "yaw_deg": round(math.degrees(yaw_rad), 1),
            "candidates": compact_cands,
            "memory_l0": memory_l0,
            "memory_l1_top": list(memory_l1)[:4],
            "memory_l2_local": {
                "objects": memory_l2.get("objects", [])[:8],
                "failures": memory_l2.get("failures", [])[:6],
            },
            "last_feedback": last_feedback,
        }
        text_payload = json.dumps(state_obj, ensure_ascii=False, separators=(",", ":"))
        if not self.config.supports_vision:
            return [
                {"role": "system", "content": _TASK_PLANNER_SYSTEM_PROMPT},
                {"role": "user", "content": text_payload},
            ]

        parts: list[dict[str, Any]] = []
        for img in [rgb_path, local_map_path, *list(extra_rgb_paths)[:2]]:
            if img is None:
                continue
            part = _encode_image_part(img)
            if part is not None:
                parts.append(part)
        parts.append({"type": "text", "text": text_payload})
        return [
            {"role": "system", "content": _TASK_PLANNER_SYSTEM_PROMPT},
            {"role": "user", "content": parts},
        ]

    def _stream_completion(self, messages: list[dict[str, Any]]) -> str:
        payload: dict[str, Any] = {
            "model": self.config.model,
            "messages": messages,
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "stream": self.config.stream,
        }
        if self.config.extra_body:
            payload.update(self.config.extra_body)
        body = json.dumps(payload, allow_nan=False).encode("utf-8")

        headers = {
            "Content-Type": "application/json",
            "Accept": "text/event-stream, application/json",
            "User-Agent": self.config.user_agent,
        }
        if self.config.api_key:
            headers["Authorization"] = f"Bearer {self.config.api_key}"
        if self.config.session_id:
            headers["x-opencode-session"] = self.config.session_id

        parsed = urllib.parse.urlparse(self.config.base_url)
        req_path = f"{parsed.path.rstrip('/')}/chat/completions"
        conn_cls = (
            http.client.HTTPSConnection
            if parsed.scheme == "https"
            else http.client.HTTPConnection
        )
        conn = conn_cls(parsed.hostname, parsed.port, timeout=self.config.timeout_s)
        try:
            conn.request("POST", req_path, body=body, headers=headers)
            resp = conn.getresponse()
            if resp.status >= 400:
                err = resp.read(256).decode("utf-8", errors="replace")
                raise RuntimeError(f"HTTP_{resp.status}:{err}")
            ctype = (resp.getheader("Content-Type") or "").lower()
            if not self.config.stream or "text/event-stream" not in ctype:
                raw = resp.read().decode("utf-8", errors="replace")
                return _extract_choice_content(json.loads(raw))

            chunks: list[str] = []
            deadline = time.monotonic() + self.config.timeout_s
            while time.monotonic() < deadline:
                raw_line = resp.readline()
                if not raw_line:
                    break
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line.startswith("data:"):
                    continue
                data_str = line[5:].strip()
                if data_str == "[DONE]":
                    break
                try:
                    chunk_json = json.loads(data_str)
                except ValueError:
                    continue
                delta = _extract_sse_delta_content(chunk_json)
                if delta:
                    chunks.append(delta)
                    joined = "".join(chunks)
                    if _extract_outer_json_object(joined) is not None:
                        break
            return "".join(chunks)
        finally:
            conn.close()

    def decide(
        self,
        *,
        task_instruction: str,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        candidates: Sequence[dict[str, Any]],
        memory_l0: dict[str, Any],
        memory_l1: Sequence[dict[str, Any]],
        memory_l2: dict[str, Any],
        last_feedback: str = "",
        rgb_path: str | Path | None = None,
        local_map_path: str | Path | None = None,
        extra_rgb_paths: Sequence[str | Path] = (),
    ) -> TaskDecision:
        valid_ids = [
            str(c["id"]) for c in candidates if isinstance(c, dict) and c.get("id")
        ]
        messages = self.build_messages(
            task_instruction=task_instruction,
            step_index=step_index,
            robot_xyz=robot_xyz,
            yaw_rad=yaw_rad,
            candidates=candidates,
            memory_l0=memory_l0,
            memory_l1=memory_l1,
            memory_l2=memory_l2,
            last_feedback=last_feedback,
            rgb_path=rgb_path,
            local_map_path=local_map_path,
            extra_rgb_paths=extra_rgb_paths,
        )
        t0 = time.perf_counter()
        try:
            raw_text = self._stream_completion(messages)
        except Exception as exc:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._latencies_ms.append(dt_ms)
            return _fallback_decision(
                valid_ids,
                reason=f"planner_error:{type(exc).__name__}:{exc}",
                latency_ms=dt_ms,
                raw_text="",
            )
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self._latencies_ms.append(dt_ms)
        return parse_task_decision(
            raw_text, valid_candidate_ids=valid_ids, latency_ms=dt_ms
        )

    @property
    def latencies_ms(self) -> list[float]:
        return list(self._latencies_ms)


def select_active_chart_and_candidates(
    snapshot: dict[str, Any], robot_z: float
) -> tuple[str | None, list[dict[str, Any]]]:
    """Select the height chart closest to robot_z with visual map and candidates."""
    charts = snapshot.get("products", {}).get("height_charts", [])
    if not isinstance(charts, list) or not charts:
        return None, []
    best_chart = None
    best_key = (1, float("inf"))
    for ch in charts:
        if not isinstance(ch, dict):
            continue
        cands = ch.get("candidates") or []
        has_cands = 0 if cands else 1
        try:
            level_z = float(str(ch.get("level_key", "0")).replace("+", ""))
        except ValueError:
            level_z = robot_z
        dz = abs(level_z - robot_z)
        score = (has_cands, dz)
        if score < best_key:
            best_key = score
            best_chart = ch
    if best_chart is None:
        return None, []
    map_img = best_chart.get("local_image") or best_chart.get("image")
    cands = [dict(c) for c in (best_chart.get("candidates") or []) if isinstance(c, dict)]
    return (str(map_img) if map_img else None), cands


def _extract_robot_pose(observation: dict[str, Any] | None) -> tuple[list[float], float]:
    if not isinstance(observation, dict):
        return [0.0, 0.0, 0.0], 0.0
    pose = observation.get("pose")
    if isinstance(pose, dict):
        xyz = pose.get("position_xyz_m")
        yaw = pose.get("yaw_rad", 0.0)
        if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
            return [float(xyz[0]), float(xyz[1]), float(xyz[2])], float(yaw or 0.0)
    return [0.0, 0.0, 0.0], 0.0


@dataclass
class TaskStepLog:
    step: int
    robot_xyz: list[float]
    yaw_deg: float
    decision: dict[str, Any]
    execution: dict[str, Any]
    memory_diag: dict[str, Any]
    sentinel_events: list[dict[str, Any]] = field(default_factory=list)


class StaticStreamingTaskRunner:
    """Run a closed-loop static embodied task over a live DynaNav run directory."""

    def __init__(
        self,
        run_dir: str | Path,
        *,
        task_instruction: str,
        target_label: str | None = None,
        planner: OpenCodeTaskPlanner,
        memory_store: HierarchicalMemoryStore | None = None,
        pose_source: str = "backend_corrected",
        sentinel_config: LlmClientConfig | None = None,
        snapshot_fn: Callable[[str], dict[str, Any]] | None = None,
        sop_script: str | Path | None = None,
        stop_radius_m: float = 1.5,
        step_timeout_s: float = 180.0,
        sentinel_poll_interval_s: float = 4.0,
        stop_simulator_on_exit: bool = True,
    ) -> None:
        self.run_dir = Path(run_dir)
        self.capture_dir = self.run_dir / "capture"
        self.mailbox = waypoint_mailbox.mailbox_for(self.capture_dir)
        self.task_instruction = task_instruction
        self.target_label = target_label
        self.planner = planner
        self.pose_source = pose_source
        self.stop_radius_m = float(stop_radius_m)
        self.step_timeout_s = float(step_timeout_s)
        self.sentinel_poll_interval_s = float(sentinel_poll_interval_s)
        self.stop_simulator_on_exit = bool(stop_simulator_on_exit)
        self.sop_script = (
            Path(sop_script)
            if sop_script is not None
            else Path(".agents/skills/dynanav-explore/scripts/dynanav_sop.py")
        )
        self._snapshot_fn = snapshot_fn

        if memory_store is not None:
            self.memory = memory_store
        else:
            map_dir = self.run_dir / "map"
            if (map_dir / "map.json").is_file():
                graph = build_scene_graph_from_map_dir(map_dir)
            else:
                graph = KeyframeSceneGraph()
            self.memory = HierarchicalMemoryStore(graph, default_stop_radius_m=self.stop_radius_m)

        self._sentinel_runner: AsyncDecisionRunner | None = None
        self._sentinel_coord: SentinelCoordinator | None = None
        if sentinel_config is not None:
            self._sentinel_runner = AsyncDecisionRunner(FastSentinelClient(sentinel_config))
            self._sentinel_coord = SentinelCoordinator(
                self.mailbox,
                self._sentinel_runner,
                pose_source=self.pose_source,
                memory_store=self.memory,
            )

    def _capture_snapshot(self, tag: str) -> dict[str, Any]:
        if self._snapshot_fn is not None:
            return self._snapshot_fn(tag)
        evidence_dir = self.run_dir / "agent_evidence"
        evidence_dir.mkdir(parents=True, exist_ok=True)
        repo_root = Path.cwd().resolve()
        cmd = [
            sys.executable,
            str(self.sop_script),
            "--repo",
            str(repo_root),
            "--run-dir",
            str(self.run_dir),
            "--evidence-dir",
            str(evidence_dir),
            "snapshot",
            "--tag",
            tag,
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False, timeout=90)
        if proc.returncode != 0:
            return {"error": f"sop_exit_{proc.returncode}", "stderr": proc.stderr[-300:], "products": {}}
        try:
            return json.loads(proc.stdout)
        except ValueError:
            return {"error": "sop_invalid_json", "products": {}}

    def _read_backend_status(self) -> dict[str, Any]:
        status_path = self.run_dir / "backend_status.json"
        if status_path.is_file():
            try:
                return json.loads(status_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return {}
        return {}

    def _record_live_step_in_memory(
        self,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        rgb_path: str | None,
        observed_objects: Sequence[dict[str, Any]],
    ) -> str:
        kf_id = f"live_step_{step_index:04d}"
        pose = _pose_from_xyz_yaw(robot_xyz, yaw_rad)
        self.memory.graph.add_keyframe(
            kf_id,
            pose,
            clearance_m=1.2,
            rgb_path=rgb_path,
        )
        if len(self.memory.graph.keyframe_poses) >= 3:
            self.memory.graph.segment_floors()
            self.memory.graph.partition_rooms()

        for idx, item in enumerate(observed_objects):
            bearing_rad = math.radians(float(item.get("bearing_deg", 0.0)))
            dist_m = float(item.get("dist_m", 2.0))
            local_xyz = [
                dist_m * math.cos(bearing_rad),
                dist_m * math.sin(bearing_rad),
                0.0,
            ]
            self.memory.add_object(
                object_id=f"obs_{step_index:04d}_{idx}",
                label=str(item["label"]),
                keyframe_id=kf_id,
                local_xyz=local_xyz,
                confidence=float(item.get("confidence", 0.8)),
                producer="opencode_vlm",
            )
        return kf_id

    def _current_action_epoch(self) -> int:
        epoch = 0
        if self._sentinel_coord is not None:
            epoch = max(epoch, int(self._sentinel_coord.action_epoch))
        obs = waypoint_mailbox.read_observation(self.mailbox)
        if isinstance(obs, dict):
            wps = obs.get("waypoints")
            if isinstance(wps, dict):
                obs_ep = wps.get("current_epoch", wps.get("action_epoch", 0))
                if isinstance(obs_ep, int) and not isinstance(obs_ep, bool):
                    epoch = max(epoch, obs_ep)
        return epoch

    def _wait_with_sentinel(
        self,
        payload: dict[str, Any],
        candidates: Sequence[dict[str, Any]],
    ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        """Wait for an active mailbox action while polling the async streaming sentinel."""
        seq = int(payload["seq"])
        run_id = payload.get("run_id")
        cursor = waypoint_mailbox.ReceiptCursor(self.mailbox)
        deadline = time.monotonic() + self.step_timeout_s
        last_sentinel_submit = 0.0
        sentinel_events: list[dict[str, Any]] = []

        while time.monotonic() < deadline:
            res = waypoint_mailbox._match_action_result(
                cursor.poll(), seq=seq, run_id=run_id
            )
            if res is not None:
                return res, sentinel_events

            if self._sentinel_coord is not None:
                now = time.monotonic()
                if now - last_sentinel_submit >= self.sentinel_poll_interval_s:
                    obs = waypoint_mailbox.read_observation(self.mailbox)
                    sim_frame = waypoint_mailbox.latest_sim_frame(self.mailbox.parent)
                    rgb_live = sim_frame.get("rgb_path") if isinstance(sim_frame, dict) else None
                    self._sentinel_coord.submit_observation(
                        observation=obs,
                        candidates=candidates,
                        backend_status=self._read_backend_status(),
                        rgb_path=rgb_live,
                    )
                    last_sentinel_submit = now

                dispatched = self._sentinel_coord.poll_and_dispatch()
                if dispatched is not None and dispatched.get("dispatched"):
                    verdict = dispatched["verdict"]
                    event = {
                        "act": verdict.act,
                        "target_id": verdict.target_id,
                        "reason": verdict.reason,
                        "latency_ms": round(verdict.latency_ms, 1),
                    }
                    sentinel_events.append(event)
                    if verdict.act in ("STOP", "PREEMPT", "SPIN"):
                        return {
                            "status": f"sentinel_{verdict.act.lower()}",
                            "event": f"sentinel_{verdict.act.lower()}",
                            "success": verdict.act != "STOP",
                            "reason": verdict.reason,
                        }, sentinel_events
            time.sleep(0.25)

        # Cancel the timed-out active action so the next step starts cleanly.
        try:
            epoch = self._current_action_epoch()
            waypoint_mailbox.submit(
                self.mailbox,
                action="cancel",
                pose_source=self.pose_source,
                extra={"action_epoch": epoch} if epoch > 0 else None,
            )
            time.sleep(0.5)
        except OSError:
            pass
        return (
            waypoint_mailbox.timeout_result(seq, self.step_timeout_s, str(run_id or "")),
            sentinel_events,
        )

    def run(self, *, max_steps: int = 12) -> dict[str, Any]:
        """Execute a bounded closed-loop static task and return full diagnostic report."""
        t_start = time.monotonic()
        step_logs: list[dict[str, Any]] = []
        last_feedback = "Episode started."
        extra_l3_rgb: list[str] = []
        task_succeeded = False
        terminal_status = "max_steps_reached"
        prev_xyz: list[float] | None = None
        total_travel_m = 0.0

        try:
            for step in range(1, max(1, int(max_steps)) + 1):
                obs = waypoint_mailbox.read_observation(self.mailbox)
                robot_xyz, yaw_rad = _extract_robot_pose(obs)
                if prev_xyz is not None:
                    total_travel_m += float(
                        np.linalg.norm(np.asarray(robot_xyz) - np.asarray(prev_xyz))
                    )
                prev_xyz = list(robot_xyz)

                snap = self._capture_snapshot(f"task_step_{step:03d}")
                rgb_path = snap.get("products", {}).get("rgb")
                local_map_path, raw_cands = select_active_chart_and_candidates(snap, robot_xyz[2])

                allowed_cands, blocked_cands = self.memory.filter_candidates(
                    raw_cands, default_z=robot_xyz[2]
                )
                cands_for_planner = allowed_cands if allowed_cands else blocked_cands

                l0 = self.memory.l0_global_topology()
                l1 = self.memory.l1_submap_summaries(
                    query=self.target_label or self.task_instruction,
                    near_xyz=robot_xyz,
                    top_k=4,
                )
                l2 = self.memory.l2_submap_entities(
                    label=self.target_label,
                    near_xyz=robot_xyz,
                    radius_m=8.0,
                )

                decision = self.planner.decide(
                    task_instruction=self.task_instruction,
                    step_index=step,
                    robot_xyz=robot_xyz,
                    yaw_rad=yaw_rad,
                    candidates=cands_for_planner,
                    memory_l0=l0,
                    memory_l1=l1,
                    memory_l2=l2,
                    last_feedback=last_feedback,
                    rgb_path=rgb_path,
                    local_map_path=local_map_path,
                    extra_rgb_paths=extra_l3_rgb,
                )
                extra_l3_rgb = []

                kf_id = self._record_live_step_in_memory(
                    step, robot_xyz, yaw_rad, rgb_path, decision.observed_objects
                )

                exec_info: dict[str, Any] = {}
                sentinel_events: list[dict[str, Any]] = []

                if decision.action == "query_memory":
                    q = decision.query or self.target_label or self.task_instruction
                    q_l1 = self.memory.l1_submap_summaries(query=q, near_xyz=robot_xyz, top_k=3)
                    q_l2 = self.memory.l2_submap_entities(label=q)
                    q_l3 = self.memory.l3_on_demand_keyframes(near_xyz=robot_xyz, max_images=2)
                    extra_l3_rgb = [row["rgb_path"] for row in q_l3 if row.get("rgb_path")]
                    exec_info = {"status": "memory_queried", "l1_hits": len(q_l1), "l2_objects": len(q_l2["objects"]), "l3_images": len(extra_l3_rgb)}
                    last_feedback = f"Memory query '{q}': L1={q_l1[:2]}, L2_objs={q_l2['objects'][:4]}"

                elif decision.action == "stop_claim":
                    claim_label = decision.target_label or self.target_label
                    verify = self.memory.verify_stop_claim(
                        robot_xyz,
                        target_label=claim_label,
                        stop_radius_m=self.stop_radius_m,
                    )
                    exec_info = verify.to_dict()
                    if verify.allowed:
                        task_succeeded = True
                        terminal_status = "verified_stop_success"
                        last_feedback = f"STOP verified: {verify.reason}"
                        step_logs.append(
                            TaskStepLog(
                                step=step,
                                robot_xyz=[round(v, 3) for v in robot_xyz],
                                yaw_deg=round(math.degrees(yaw_rad), 1),
                                decision=decision.to_dict(),
                                execution=exec_info,
                                memory_diag=self.memory.diagnostic_metrics(),
                            ).__dict__
                        )
                        break
                    else:
                        self.memory.record_attempt(
                            "stop_claim",
                            robot_xyz,
                            "rejected",
                            reason=verify.reason,
                            keyframe_id=kf_id,
                        )
                        last_feedback = (
                            f"STOP REJECTED ({verify.reason}, dist={verify.distance_m}). "
                            "Move closer to the target on the same floor and confirm visually."
                        )

                elif decision.action in ("goto_candidate", "goto_xyz"):
                    goal_xyz: list[float] | None = None
                    if decision.action == "goto_candidate":
                        for c in cands_for_planner:
                            if c.get("id") == decision.target_id:
                                xy = c.get("xy")
                                if isinstance(xy, (list, tuple)) and len(xy) >= 2:
                                    goal_xyz = [float(xy[0]), float(xy[1]), float(c.get("z", robot_xyz[2]))]
                                elif isinstance(c.get("goal_xyz_m"), (list, tuple)):
                                    goal_xyz = [float(v) for v in c["goal_xyz_m"]]
                                break
                    elif decision.xyz is not None:
                        goal_xyz = [float(v) for v in decision.xyz]

                    if goal_xyz is None:
                        exec_info = {"status": "skipped", "reason": "unresolved_goal"}
                        last_feedback = "Could not resolve target coordinates; pick a valid candidate ID."
                    else:
                        epoch = self._current_action_epoch()
                        payload = waypoint_mailbox.submit(
                            self.mailbox,
                            action="goto",
                            goal=goal_xyz,
                            ttl_s=min(120.0, max(30.0, self.step_timeout_s)),
                            pose_source=self.pose_source,
                            extra={"action_epoch": epoch} if epoch > 0 else None,
                        )
                        exec_info, sentinel_events = self._wait_with_sentinel(
                            payload, cands_for_planner
                        )
                        outcome = str(exec_info.get("status", "unknown"))
                        self.memory.record_attempt(
                            "goto",
                            goal_xyz,
                            outcome,
                            reason=str(exec_info.get("reason", "")),
                            keyframe_id=kf_id,
                        )
                        last_feedback = (
                            f"goto {goal_xyz} -> {outcome} ({exec_info.get('reason', '')}); "
                            f"sentinel_events={len(sentinel_events)}"
                        )

                elif decision.action == "maneuver":
                    epoch = self._current_action_epoch()
                    m_extra: dict[str, Any] = {
                        "body_forward_mps": decision.forward,
                        "body_lateral_mps": 0.0,
                        "yaw_rate_rps": decision.yaw_rate,
                        "duration_s": decision.duration,
                    }
                    if epoch > 0:
                        m_extra["action_epoch"] = epoch
                    payload = waypoint_mailbox.submit(
                        self.mailbox,
                        action="maneuver",
                        ttl_s=min(120.0, max(30.0, decision.duration * 20.0)),
                        pose_source=self.pose_source,
                        extra=m_extra,
                    )
                    exec_info = waypoint_mailbox.wait_for_action(
                        self.mailbox,
                        int(payload["seq"]),
                        timeout_s=max(30.0, decision.duration * 15.0),
                    )
                    outcome = str(exec_info.get("status", "unknown"))
                    self.memory.record_attempt(
                        "maneuver",
                        robot_xyz,
                        outcome,
                        reason=str(exec_info.get("reason", "")),
                        keyframe_id=kf_id,
                    )
                    last_feedback = f"maneuver -> {outcome} ({exec_info.get('reason', '')})"

                elif decision.action == "scan":
                    epoch = self._current_action_epoch()
                    y_extra: dict[str, Any] = {
                        "delta_yaw_rad": math.pi / 2.0,
                        "yaw_rate_rps": 0.15,
                    }
                    if epoch > 0:
                        y_extra["action_epoch"] = epoch
                    payload = waypoint_mailbox.submit(
                        self.mailbox,
                        action="yaw",
                        ttl_s=90.0,
                        pose_source=self.pose_source,
                        extra=y_extra,
                    )
                    exec_info = waypoint_mailbox.wait_for_action(
                        self.mailbox,
                        int(payload["seq"]),
                        timeout_s=90.0,
                    )
                    last_feedback = f"scan yaw +90deg -> {exec_info.get('status')}"

                print(
                    f"[step {step}] pos={[round(v, 2) for v in robot_xyz]} "
                    f"act={decision.action} target={decision.target_id or decision.target_label} "
                    f"obs={len(decision.observed_objects)} llm={decision.latency_ms:.0f}ms "
                    f"-> {exec_info.get('status', exec_info.get('reason', 'ok'))} "
                    f"sentinel={len(sentinel_events)}",
                    flush=True,
                )
                step_logs.append(
                    TaskStepLog(
                        step=step,
                        robot_xyz=[round(v, 3) for v in robot_xyz],
                        yaw_deg=round(math.degrees(yaw_rad), 1),
                        decision=decision.to_dict(),
                        execution=exec_info,
                        memory_diag=self.memory.diagnostic_metrics(),
                        sentinel_events=sentinel_events,
                    ).__dict__
                )
        finally:
            if self._sentinel_runner is not None:
                self._sentinel_runner.shutdown(timeout_s=2.0)
            if self.stop_simulator_on_exit and self.mailbox.is_dir():
                try:
                    epoch = self._current_action_epoch()
                    waypoint_mailbox.submit(
                        self.mailbox,
                        action="stop",
                        pose_source=self.pose_source,
                        extra={"action_epoch": epoch} if epoch > 0 else None,
                    )
                except OSError:
                    pass

        report = {
            "task_instruction": self.task_instruction,
            "target_label": self.target_label,
            "task_succeeded": task_succeeded,
            "terminal_status": terminal_status,
            "steps_executed": len(step_logs),
            "wall_s": round(time.monotonic() - t_start, 2),
            "estimated_travel_m": round(total_travel_m, 2),
            "planner_config": self.planner.config.to_safe_dict(),
            "planner_latencies_ms": [round(v, 1) for v in self.planner.latencies_ms],
            "memory_diagnostics": self.memory.diagnostic_metrics(),
            "sentinel_stats": (
                self._sentinel_runner.stats() if self._sentinel_runner is not None else None
            ),
            "steps": step_logs,
        }
        report_path = self.run_dir / "static_task_report.json"
        if self.run_dir.is_dir():
            report_path.write_text(
                json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
            )
            self.memory.save(self.run_dir / "hierarchical_memory.json")
        return report
