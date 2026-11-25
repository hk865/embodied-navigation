"""Closed-loop static embodied task agent combining L0-L3 memory and OpenCode Go SSE streaming.

Architecture (strictly obeys the 7-layer boundary in AGENTS.md):
- Reads ONLY live sensor evidence (front RGB, annotated 2D operator maps, F*/W*
  candidates, post-VIO / backend_corrected pose, and backend_status). Never reads
  simulator ground truth or writes cmd_vel directly.
- Maintains a versioned :class:`HierarchicalMemoryStore` (L0-L3 retrieval,
  Attempt & Failure Ledger geographic blacklist, and pre-STOP geometric gate).
- Runs a main multimodal/reasoning SSE streaming planner (e.g. OpenCode Go
  ``qwen3.8-flash`` for vision or ``deepseek-v4.1-flash`` for high-reasoning)
  together with an optional in-motion async streaming sentinel
  (:class:`SentinelCoordinator` + :class:`AsyncDecisionRunner`) that can preempt
  active goals via ``action_epoch``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import http.client
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Any, Callable, Iterable, Sequence
import urllib.parse
import uuid

import numpy as np

from embodied_navigation.common.runtime_io import atomic_write_json
from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
    build_scene_graph_from_map_dir,
)
from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
    _label_strict_match,
)
from embodied_navigation.planning.exploration import waypoint_mailbox
from embodied_navigation.planning.exploration.llm_agent_client import (
    AsyncDecisionRunner,
    FastSentinelClient,
    LlmClientConfig,
    _encode_image_part,
    _extract_choice_content,
    _extract_sse_delta_content,
    validate_image_path,
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
    "noop",
)

_TASK_PLANNER_SYSTEM_PROMPT = (
    "## [SYSTEM]\n"
    "You are an autonomous quadruped navigation agent operating in a static indoor "
    "multi-floor building using real SLAM poses, front RGB, annotated 2D maps, and "
    "versioned hierarchical spatial memory. Never invent coordinates outside sensors/memory.\n"
    "Images attached (when present): image 1 = front RGB camera; image 2 = annotated "
    "2D occupancy map with robot footprint, turning circle (green=safe to scan, red=tight), "
    "and numbered frontier/waypoint candidates (F1..F*, W1..W*). Additional images (if any) "
    "are L3 historical keyframe RGBs recalled by query_memory.\n"
    "Output ONLY ONE compact JSON object matching:\n"
    '{"action":"goto_candidate"|"goto_xyz"|"maneuver"|"scan"|"query_memory"|"stop_claim",'
    '"target_id":"M1"|"F1"|null,"xyz":[x,y,z]|null,"forward":0.0,"yaw_rate":0.0,"duration":1.0,'
    '"query":null,"target_label":null,"observed_objects":[],"reason":"short"}\n\n'
    "## [SKILL]\n"
    "1. Memory-first approach: If an M* candidate (memory_target) matches the task goal and is "
    "non-blacklisted, prioritize goto_candidate on M* instead of wandering unrelated frontiers.\n"
    "2. Exploration: When the target is not yet in memory, pick a clear, non-blacklisted F*/W* "
    "candidate within ~6 m; avoid blacklisted spots unless no alternative exists.\n"
    "3. On-demand memory recall: [MEMORY_SUMMARY] is intentionally a 1-paragraph summary. Call "
    "query_memory (with a room/object query) ONLY when you need detailed room object lists, "
    "exact coordinates, or L3 historical RGB images.\n"
    "4. Visual logging: Always report salient landmarks clearly visible in front RGB inside "
    "observed_objects ({\"label\":\"short name\",\"dist_m\":float,\"bearing_deg\":float,"
    "\"confidence\":0.5-1.0}; bearing_deg=0 straight ahead, +left, -right).\n"
    "5. Anti-trapping & stairs: Use maneuver (forward in [-0.45,0.50], yaw_rate in [-0.15,0.15], "
    "duration in [0.5,2.0]) for door jams or stair climbing.\n"
    "6. Active STOP gate: Call stop_claim ONLY when within stop_radius_m on the same floor and "
    "target is confirmed; if rejected in [TOOL_RESULTS], move closer and re-verify."
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
    """Parse one compact top-level JSON task decision with fail-safe validation.

    Truncated outer objects are rejected strictly (never falling back to inner
    nested objects such as observed_objects entries).
    """
    obj = _extract_outer_json_object(raw_text)
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
        if not target_id or (valid_candidate_ids and target_id not in valid_candidate_ids):
            return _fallback_decision(
                valid_candidate_ids,
                reason="invalid_or_missing_candidate_id",
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
    """Fail safe to a non-moving noop on parse/transport error (never blind-goto M1)."""
    del valid_candidate_ids
    return TaskDecision(
        action="noop",
        reason=reason,
        latency_ms=latency_ms,
        fallback=True,
        raw_text=raw_text[:500],
    )


class OpenCodeTaskPlanner:
    """Multimodal SSE-streaming task planner backed by OpenCode Go / OpenAI-compatible API."""

    def __init__(
        self,
        config: LlmClientConfig,
        *,
        allowed_image_roots: Iterable[str | Path] | None = None,
    ) -> None:
        self.config = config
        self.allowed_image_roots: tuple[Path, ...] | None = (
            tuple(Path(r).resolve() for r in allowed_image_roots)
            if allowed_image_roots is not None
            else None
        )
        self._latencies_ms: list[float] = []
        self.last_rejected_images: list[dict[str, str]] = []

    def build_messages(
        self,
        *,
        task_instruction: str,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        candidates: Sequence[dict[str, Any]],
        memory_summary: str = "",
        memory_l0: dict[str, Any] | None = None,
        memory_l1: Sequence[dict[str, Any]] = (),
        memory_l2: dict[str, Any] | None = None,
        target_label: str | None = None,
        stop_radius_m: float = 1.5,
        estimated_travel_m: float = 0.0,
        last_feedback: str = "",
        tool_recall_result: dict[str, Any] | None = None,
        rgb_path: str | Path | None = None,
        local_map_path: str | Path | None = None,
        extra_rgb_paths: Sequence[str | Path] = (),
    ) -> list[dict[str, Any]]:
        compact_cands = [
            {
                "id": c.get("id"),
                "kind": c.get("kind"),
                "label": c.get("label"),
                "xy": c.get("xy"),
                "z": c.get("z"),
                "dist_m": c.get("dist_m"),
                "clearance_m": c.get("clearance_m"),
                "turn_safe": c.get("turn_safe"),
                "blacklisted": c.get("blacklisted", False),
            }
            for c in list(candidates)[:12]
        ]

        summary_text = memory_summary.strip() if memory_summary else ""
        if not summary_text and memory_l0 is not None:
            summary_text = (
                f"当前空间记忆版本 rev={memory_l0.get('rev', 0)}，"
                f"楼层数={len(memory_l0.get('floors', []))}，"
                f"房间数={len(memory_l0.get('rooms', []))}，"
                f"活跃失败黑名单={memory_l0.get('fail_cnt', 0)}。"
                "如需详细房间物体清单或历史关键帧原图，请调用 query_memory 工具回想。"
            )

        state_block = {
            "step": step_index,
            "robot_xyz": [round(float(v), 2) for v in robot_xyz],
            "yaw_deg": round(math.degrees(yaw_rad), 1),
            "estimated_travel_m": round(float(estimated_travel_m), 2),
            "candidates": compact_cands,
        }
        tool_block: dict[str, Any] = {"last_action_receipt": last_feedback}
        if tool_recall_result is not None:
            tool_block["memory_tool_recall"] = tool_recall_result
        elif memory_l1 or (memory_l2 and (memory_l2.get("objects") or memory_l2.get("failures"))):
            # Backward-compatible compact fallback if caller passed explicit L1/L2 without recall
            tool_block["active_local_failures"] = (memory_l2 or {}).get("failures", [])[:4]

        text_payload = (
            f"## [TASK]\n"
            f"instruction: {task_instruction}\n"
            f"target_label: {target_label or 'unspecified'}\n"
            f"stop_verification: same floor & distance <= {stop_radius_m:.1f} m\n\n"
            f"## [MEMORY_SUMMARY]\n"
            f"{summary_text}\n\n"
            f"## [INTERMEDIATE_STATE]\n"
            f"{json.dumps(state_block, ensure_ascii=False, separators=(',', ':'))}\n\n"
            f"## [TOOL_RESULTS]\n"
            f"{json.dumps(tool_block, ensure_ascii=False, separators=(',', ':'))}"
        )
        if not self.config.supports_vision:
            return [
                {"role": "system", "content": _TASK_PLANNER_SYSTEM_PROMPT},
                {"role": "user", "content": text_payload},
            ]

        self.last_rejected_images = []
        parts: list[dict[str, Any]] = [{"type": "text", "text": text_payload}]
        for img in [rgb_path, local_map_path, *list(extra_rgb_paths)[:2]]:
            if img is None:
                continue
            resolved, reject_reason = validate_image_path(
                img, allowed_roots=self.allowed_image_roots
            )
            if resolved is None:
                self.last_rejected_images.append(
                    {"path": str(img), "reason": str(reject_reason or "rejected")}
                )
                continue
            part = _encode_image_part(resolved, allowed_roots=self.allowed_image_roots)
            if part is not None:
                parts.append(part)
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
        deadline = time.monotonic() + self.config.timeout_s
        conn = conn_cls(parsed.hostname, parsed.port, timeout=self.config.timeout_s)
        try:
            rem = deadline - time.monotonic()
            if rem <= 0.0:
                raise TimeoutError("request_deadline_exceeded")
            conn.timeout = rem
            conn.request("POST", req_path, body=body, headers=headers)
            if conn.sock is not None:
                rem = deadline - time.monotonic()
                if rem <= 0.0:
                    raise TimeoutError("request_deadline_exceeded")
                conn.sock.settimeout(rem)
            resp = conn.getresponse()
            if resp.status >= 400:
                # Never copy upstream HTTP error body into exception or persisted reports.
                raise RuntimeError(f"http_{resp.status}")
            ctype = (resp.getheader("Content-Type") or "").lower()
            if not self.config.stream or "text/event-stream" not in ctype:
                if conn.sock is not None:
                    rem = deadline - time.monotonic()
                    if rem <= 0.0:
                        raise TimeoutError("request_deadline_exceeded")
                    conn.sock.settimeout(rem)
                raw = resp.read().decode("utf-8", errors="replace")
                return _extract_choice_content(json.loads(raw))

            chunks: list[str] = []
            while True:
                rem = deadline - time.monotonic()
                if rem <= 0.0:
                    raise TimeoutError("request_deadline_exceeded")
                if conn.sock is not None:
                    conn.sock.settimeout(rem)
                try:
                    raw_line = resp.readline()
                except (TimeoutError, socket.timeout) as exc:
                    raise TimeoutError("request_deadline_exceeded") from exc
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
        memory_summary: str = "",
        memory_l0: dict[str, Any] | None = None,
        memory_l1: Sequence[dict[str, Any]] = (),
        memory_l2: dict[str, Any] | None = None,
        target_label: str | None = None,
        stop_radius_m: float = 1.5,
        estimated_travel_m: float = 0.0,
        last_feedback: str = "",
        tool_recall_result: dict[str, Any] | None = None,
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
            memory_summary=memory_summary,
            memory_l0=memory_l0,
            memory_l1=memory_l1,
            memory_l2=memory_l2,
            target_label=target_label,
            stop_radius_m=stop_radius_m,
            estimated_travel_m=estimated_travel_m,
            last_feedback=last_feedback,
            tool_recall_result=tool_recall_result,
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
            safe_msg = str(exc) if isinstance(exc, (RuntimeError, TimeoutError)) else type(exc).__name__
            if len(safe_msg) > 64:
                safe_msg = type(exc).__name__
            return _fallback_decision(
                valid_ids,
                reason=f"planner_error:{type(exc).__name__}:{safe_msg}",
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
    best_chart = _select_active_chart_entry(snapshot, robot_z)
    if best_chart is None:
        return None, []
    map_img = best_chart.get("local_image") or best_chart.get("image")
    cands = [dict(c) for c in (best_chart.get("candidates") or []) if isinstance(c, dict)]
    return (str(map_img) if map_img else None), cands


def _select_active_chart_entry(
    snapshot: dict[str, Any], robot_z: float
) -> dict[str, Any] | None:
    charts = snapshot.get("products", {}).get("height_charts", [])
    if not isinstance(charts, list) or not charts:
        return None
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
    return best_chart


def _measure_bearing_range_from_snapshot(
    snapshot: dict[str, Any],
    robot_xyz: Sequence[float],
    yaw_rad: float,
    bearing_deg: float,
    *,
    allowed_roots: Sequence[Path] | None = None,
    half_angle_deg: float = 6.0,
    min_range_m: float = 0.25,
    max_range_m: float = 12.0,
) -> dict[str, Any] | None:
    """Measure independent LiDAR/occupancy ray distance from snapshot height_chart NPZ.

    Returns None when no valid deskewed occupancy chart is present or no
    occupied return falls along the queried bearing fan.
    """
    if not isinstance(snapshot, dict):
        return None
    chart = _select_active_chart_entry(snapshot, float(robot_xyz[2]))
    if chart is None:
        return None
    base_img = chart.get("image") or chart.get("local_image")
    if not isinstance(base_img, str) or not base_img:
        return None
    img_path = Path(base_img)
    stem = img_path.stem
    if stem.endswith("_local"):
        stem = stem[:-6]
    json_path = img_path.with_name(f"{stem}.json")
    npz_path = img_path.with_name(f"{stem}.npz")
    try:
        json_resolved = json_path.resolve(strict=True)
        npz_resolved = npz_path.resolve(strict=True)
    except OSError:
        return None
    if allowed_roots is not None:
        if not any(json_resolved.is_relative_to(r) for r in allowed_roots):
            return None
        if not any(npz_resolved.is_relative_to(r) for r in allowed_roots):
            return None
    try:
        meta = json.loads(json_resolved.read_text(encoding="utf-8"))
        res = float(meta["resolution_m"])
        if not math.isfinite(res) or res <= 0.0:
            return None
        with np.load(npz_resolved) as data:
            ix = data["ix"]
            iy = data["iy"]
            code = data["code"]
            occ = code == 1
            if not np.any(occ):
                return None
            ox = (ix[occ].astype(np.float64) + 0.5) * res
            oy = (iy[occ].astype(np.float64) + 0.5) * res
    except (OSError, ValueError, KeyError):
        return None

    dx = ox - float(robot_xyz[0])
    dy = oy - float(robot_xyz[1])
    dist = np.hypot(dx, dy)
    ang = np.arctan2(dy, dx) - float(yaw_rad)
    ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
    target_rad = math.radians(float(bearing_deg))
    ang_diff = np.abs((ang - target_rad + math.pi) % (2.0 * math.pi) - math.pi)
    mask = (ang_diff <= math.radians(half_angle_deg)) & (dist >= min_range_m) & (dist <= max_range_m)
    if not np.any(mask):
        return None
    d_min = float(np.min(dist[mask]))
    stamp_raw = meta.get("source_clock_used")
    stamp_s = float(stamp_raw) if isinstance(stamp_raw, (int, float)) and math.isfinite(float(stamp_raw)) else None
    return {
        "distance_m": round(d_min, 3),
        "range_source": "height_chart_lidar_raycast",
        "evidence_frame": str(meta.get("map_frame") or "map"),
        "evidence_stamp_s": stamp_s,
    }


def _extract_robot_pose(observation: dict[str, Any] | None) -> tuple[list[float], float] | None:
    """Return validated live robot pose or None if observation is stale/missing/terminal."""
    if not isinstance(observation, dict):
        return None
    wps = observation.get("waypoints")
    if isinstance(wps, dict) and str(wps.get("phase", "")).lower() == "terminal":
        return None
    pose = observation.get("pose")
    if not isinstance(pose, dict):
        return None
    if pose.get("fresh") is False:
        return None
    xyz = pose.get("position_xyz_m")
    yaw = pose.get("yaw_rad")
    if not isinstance(xyz, (list, tuple)) or len(xyz) < 3:
        return None
    try:
        pos = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        yaw_f = 0.0 if yaw is None else float(yaw)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in pos) or not math.isfinite(yaw_f):
        return None
    return pos, yaw_f


@dataclass
class TaskStepLog:
    step: int
    robot_xyz: list[float]
    yaw_deg: float
    decision: dict[str, Any]
    execution: dict[str, Any]
    memory_diag: dict[str, Any]
    sentinel_events: list[dict[str, Any]] = field(default_factory=list)


_VISION_OBSERVER_SYSTEM_PROMPT = (
    "You are a concise robot camera landmark observer. "
    "Given the front RGB view, describe the scene briefly and list up to 4 clearly visible "
    "salient architectural landmarks or objects with estimated distance (m) and bearing_deg "
    "(0 straight ahead, +left, -right). Reply ONLY with 1 compact JSON object: "
    '{"scene_caption":"short description","observed_objects":'
    '[{"label":"short name","dist_m":2.5,"bearing_deg":0.0,"confidence":0.9}]}.'
)


class VisionLandmarkObserver:
    """Dedicated VLM perception observer providing structured visual landmarks for text-only planners."""

    def __init__(
        self,
        config: LlmClientConfig,
        *,
        allowed_image_roots: Iterable[str | Path] | None = None,
    ) -> None:
        self.config = config
        self._client = OpenCodeTaskPlanner(config, allowed_image_roots=allowed_image_roots)
        self._latencies_ms: list[float] = []

    @property
    def allowed_image_roots(self) -> tuple[Path, ...] | None:
        return self._client.allowed_image_roots

    @allowed_image_roots.setter
    def allowed_image_roots(self, roots: tuple[Path, ...] | None) -> None:
        self._client.allowed_image_roots = roots

    @property
    def latencies_ms(self) -> list[float]:
        return list(self._latencies_ms)

    def observe(
        self,
        rgb_path: str | Path | None,
        *,
        target_label: str | None = None,
    ) -> dict[str, Any]:
        if rgb_path is None or not self.config.supports_vision:
            return {"scene_caption": "", "observed_objects": [], "latency_ms": 0.0, "ok": False}
        part = _encode_image_part(rgb_path, allowed_roots=self._client.allowed_image_roots)
        if part is None:
            return {
                "scene_caption": "",
                "observed_objects": [],
                "latency_ms": 0.0,
                "ok": False,
                "error": "image_rejected",
            }
        hint = (
            f"Pay special attention to whether '{target_label}' is visible."
            if target_label
            else "Report salient doors, stairs, signs, and landmarks."
        )
        messages = [
            {"role": "system", "content": _VISION_OBSERVER_SYSTEM_PROMPT},
            {"role": "user", "content": [{"type": "text", "text": hint}, part]},
        ]
        t0 = time.perf_counter()
        try:
            raw = self._client._stream_completion(messages)
        except Exception as exc:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self._latencies_ms.append(dt_ms)
            return {
                "scene_caption": "",
                "observed_objects": [],
                "latency_ms": round(dt_ms, 1),
                "ok": False,
                "error": type(exc).__name__,
            }
        dt_ms = (time.perf_counter() - t0) * 1000.0
        self._latencies_ms.append(dt_ms)
        parsed = _extract_outer_json_object(raw)
        if not isinstance(parsed, dict):
            return {
                "scene_caption": "",
                "observed_objects": [],
                "latency_ms": round(dt_ms, 1),
                "ok": False,
                "error": "parse_error",
            }
        obs_clean: list[dict[str, Any]] = []
        raw_obs = parsed.get("observed_objects")
        if isinstance(raw_obs, list):
            for item in raw_obs[:6]:
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
                obs_clean.append(
                    {
                        "label": label,
                        "dist_m": round(dist_m, 2),
                        "bearing_deg": round(bearing_deg, 1),
                        "confidence": round(conf, 2),
                    }
                )
        return {
            "scene_caption": str(parsed.get("scene_caption", "")).strip()[:200],
            "observed_objects": obs_clean,
            "latency_ms": round(dt_ms, 1),
            "ok": True,
            "observer_model": self.config.model,
        }


class StaticStreamingTaskRunner:
    """Run a closed-loop static embodied task over a live DynaNav run directory."""

    def __init__(
        self,
        run_dir: str | Path,
        *,
        task_instruction: str,
        target_label: str | None = None,
        target_aliases: Sequence[str] = (),
        planner: OpenCodeTaskPlanner,
        vision_observer: VisionLandmarkObserver | None = None,
        memory_store: HierarchicalMemoryStore | None = None,
        pose_source: str = "backend_corrected",
        sentinel_config: LlmClientConfig | None = None,
        snapshot_fn: Callable[[str], dict[str, Any]] | None = None,
        sop_script: str | Path | None = None,
        stop_radius_m: float = 1.5,
        step_timeout_s: float = 180.0,
        sentinel_poll_interval_s: float = 4.0,
        control_receipt_timeout_s: float = 5.0,
        pose_wait_timeout_s: float = 2.0,
        stop_simulator_on_exit: bool = True,
        allowed_image_roots: Iterable[str | Path] | None = None,
        episode_id: str | None = None,
    ) -> None:
        self.run_dir = Path(run_dir)
        self.capture_dir = self.run_dir / "capture"
        self.mailbox = waypoint_mailbox.mailbox_for(self.capture_dir)
        self.task_instruction = task_instruction
        self.target_label = target_label
        self.target_aliases: tuple[str, ...] = tuple(
            str(a).strip() for a in target_aliases if str(a).strip()
        )
        self.planner = planner
        self.vision_observer = vision_observer
        self.sentinel_config = sentinel_config
        self.pose_source = pose_source
        self.stop_radius_m = float(stop_radius_m)
        self.step_timeout_s = float(step_timeout_s)
        self.sentinel_poll_interval_s = float(sentinel_poll_interval_s)
        self.control_receipt_timeout_s = float(control_receipt_timeout_s)
        self.pose_wait_timeout_s = float(pose_wait_timeout_s)
        self.stop_simulator_on_exit = bool(stop_simulator_on_exit)
        self.episode_id = episode_id or f"ep_{uuid.uuid4().hex[:8]}"
        self.sop_script = (
            Path(sop_script)
            if sop_script is not None
            else Path(".agents/skills/dynanav-explore/scripts/dynanav_sop.py")
        )
        self._snapshot_fn = snapshot_fn
        self._sentinel_stop_locked = False
        self._sentinel_stop_confirmed = False
        self._last_synced_backend_rev = -1

        roots: list[Path] = [self.run_dir.resolve()]
        if allowed_image_roots is not None:
            for r in allowed_image_roots:
                rp = Path(r).resolve()
                if rp not in roots:
                    roots.append(rp)
        self.allowed_image_roots: tuple[Path, ...] = tuple(roots)
        if self.planner.allowed_image_roots is None:
            self.planner.allowed_image_roots = self.allowed_image_roots
        if self.vision_observer is not None and self.vision_observer.allowed_image_roots is None:
            self.vision_observer.allowed_image_roots = self.allowed_image_roots

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
            self._sentinel_runner = AsyncDecisionRunner(
                FastSentinelClient(
                    sentinel_config,
                    allowed_image_roots=self.allowed_image_roots,
                )
            )
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

    def _sync_memory_from_backend_map(self) -> dict[str, Any] | None:
        """Synchronize memory poses when the live SLAM backend publishes a newer map revision."""
        status = self._read_backend_status()
        backend_rev = status.get("map_revision", status.get("loop_count"))
        if not isinstance(backend_rev, int) or isinstance(backend_rev, bool):
            return None
        if backend_rev <= self._last_synced_backend_rev:
            return None
        kf_path = self.run_dir / "map" / "keyframes.json"
        if not kf_path.is_file():
            return None
        try:
            raw = json.loads(kf_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        rows = raw.get("keyframes", raw) if isinstance(raw, dict) else raw
        if not isinstance(rows, list):
            return None
        updated_poses: dict[str, np.ndarray] = {}
        for row in rows:
            if not isinstance(row, dict):
                continue
            kf_id = row.get("keyframe_id") or row.get("id")
            pose_raw = row.get("pose_matrix") or row.get("pose")
            if not kf_id or pose_raw is None:
                continue
            try:
                mat = np.asarray(pose_raw, dtype=np.float64)
                if mat.shape == (4, 4) and np.all(np.isfinite(mat)):
                    updated_poses[str(kf_id)] = mat
            except (TypeError, ValueError):
                continue
        if not updated_poses:
            return None
        self._last_synced_backend_rev = backend_rev
        return self.memory.sync_poses(updated_poses)

    def _record_live_step_in_memory(
        self,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        rgb_path: str | None,
        observed_objects: Sequence[dict[str, Any]],
        *,
        snapshot: dict[str, Any] | None = None,
    ) -> str:
        del step_index
        seq = self.memory.allocate_step_seq()
        kf_id = f"{self.episode_id}_step_{seq:04d}"
        pose = _pose_from_xyz_yaw(robot_xyz, yaw_rad)
        validated_rgb: str | None = None
        if rgb_path is not None:
            resolved_rgb, _ = validate_image_path(
                rgb_path, allowed_roots=self.allowed_image_roots
            )
            if resolved_rgb is not None:
                validated_rgb = str(resolved_rgb)
        self.memory.graph.add_keyframe(
            kf_id,
            pose,
            clearance_m=1.2,
            rgb_path=validated_rgb,
        )
        if len(self.memory.graph.keyframe_poses) >= 3:
            self.memory.graph.segment_floors()
            self.memory.graph.partition_rooms()

        for item in observed_objects:
            bearing_deg = float(item.get("bearing_deg", 0.0))
            bearing_rad = math.radians(bearing_deg)
            measured = (
                _measure_bearing_range_from_snapshot(
                    snapshot,
                    robot_xyz,
                    yaw_rad,
                    bearing_deg,
                    allowed_roots=self.allowed_image_roots,
                )
                if snapshot is not None
                else None
            )
            if measured is not None:
                dist_m = float(measured["distance_m"])
                producer = "vlm_lidar_grounded"
                range_source = str(measured["range_source"])
                evidence_frame = measured.get("evidence_frame")
                evidence_stamp_s = measured.get("evidence_stamp_s")
            else:
                dist_m = float(item.get("dist_m", 2.0))
                producer = "opencode_vlm_unverified"
                range_source = "vlm_unverified_guess"
                evidence_frame = None
                evidence_stamp_s = None

            local_xyz = [
                dist_m * math.cos(bearing_rad),
                dist_m * math.sin(bearing_rad),
                0.0,
            ]
            obj_seq = self.memory.allocate_object_seq()
            self.memory.add_object(
                object_id=f"{self.episode_id}_obs_{obj_seq:04d}",
                label=str(item["label"]),
                keyframe_id=kf_id,
                local_xyz=local_xyz,
                confidence=float(item.get("confidence", 0.8)),
                producer=producer,
                range_source=range_source,
                evidence_frame=evidence_frame,
                evidence_stamp_s=evidence_stamp_s,
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

    def _read_control_token(self, obs: dict[str, Any] | None = None) -> tuple[str | None, Any, int]:
        """Snapshot (run_id, active_seq, epoch) to bind planner decisions to current control state."""
        data = obs if isinstance(obs, dict) else waypoint_mailbox.read_observation(self.mailbox)
        run_id = waypoint_mailbox.run_id_for(self.mailbox)
        active_seq = None
        epoch = self._current_action_epoch()
        if isinstance(data, dict):
            if isinstance(data.get("run_id"), str) and data["run_id"]:
                run_id = data["run_id"]
            wps = data.get("waypoints")
            if isinstance(wps, dict):
                active_seq = wps.get("active_seq")
                obs_ep = wps.get("current_epoch", wps.get("action_epoch", 0))
                if isinstance(obs_ep, int) and not isinstance(obs_ep, bool):
                    epoch = max(epoch, obs_ep)
        return run_id, active_seq, epoch

    def _wait_for_valid_pose(self) -> tuple[dict[str, Any] | None, tuple[list[float], float] | None]:
        """Wait up to pose_wait_timeout_s for a fresh non-terminal pose (never fake [0,0,0])."""
        deadline = time.monotonic() + max(0.0, self.pose_wait_timeout_s)
        last_obs = None
        while True:
            obs = waypoint_mailbox.read_observation(self.mailbox)
            last_obs = obs
            parsed = _extract_robot_pose(obs)
            if parsed is not None:
                return obs, parsed
            if isinstance(obs, dict):
                wps = obs.get("waypoints")
                if isinstance(wps, dict) and str(wps.get("phase", "")).lower() == "terminal":
                    return obs, None
            if time.monotonic() >= deadline:
                return last_obs, None
            time.sleep(0.1)

    def _wait_for_control_receipt(
        self,
        seq: int,
        expected_events: set[str],
        *,
        run_id: str | None = None,
        timeout_s: float | None = None,
        cursor: waypoint_mailbox.ReceiptCursor | None = None,
    ) -> dict[str, Any]:
        """Wait for a closing receipt for a stop/cancel control request."""
        active_cursor = cursor if cursor is not None else waypoint_mailbox.ReceiptCursor(self.mailbox)
        wait_limit = self.control_receipt_timeout_s if timeout_s is None else float(timeout_s)
        deadline = time.monotonic() + max(0.1, wait_limit)
        eff_run = str(run_id or "")
        while True:
            receipts = active_cursor.poll()
            for receipt in receipts:
                if eff_run and receipt.get("run_id") != eff_run:
                    continue
                ev = receipt.get("event")
                r_seq = receipt.get("seq")
                if isinstance(r_seq, int) and not isinstance(r_seq, bool) and r_seq == seq:
                    if ev in expected_events:
                        return {
                            "status": str(ev),
                            "event": str(ev),
                            "seq": seq,
                            "receipt": dict(receipt),
                        }
                    if isinstance(ev, str) and ev.endswith("_rejected"):
                        return {
                            "status": "rejected",
                            "event": ev,
                            "reason": receipt.get("reason"),
                            "seq": seq,
                            "receipt": dict(receipt),
                        }
            obs = waypoint_mailbox.read_observation(self.mailbox)
            if isinstance(obs, dict):
                wps = obs.get("waypoints")
                if isinstance(wps, dict) and str(wps.get("phase", "")).lower() == "terminal":
                    return {
                        "status": "simulator_terminal",
                        "event": "terminal",
                        "seq": seq,
                    }
            if time.monotonic() >= deadline:
                return {
                    "status": "receipt_timeout",
                    "event": "timeout",
                    "seq": seq,
                }
            time.sleep(0.05)

    def _wait_with_sentinel(
        self,
        payload: dict[str, Any],
        candidates: Sequence[dict[str, Any]],
    ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        """Wait for an active mailbox action while polling the async streaming sentinel.

        Guarantees receipt closure:
        - Poll completed sentinel verdicts BEFORE submitting new observations.
        - On sentinel STOP: lock main loop immediately and wait for stop_accepted receipt.
        - On sentinel PREEMPT/SPIN: switch active tracking to the new seq/epoch and wait
          until that action reaches a real terminal receipt (never mark dispatch as success).
        - On timeout: submit cancel and wait for cancel/terminal receipt without swallowing errors.
        """
        active_seq = int(payload["seq"])
        run_id = payload.get("run_id")
        cursor = waypoint_mailbox.ReceiptCursor(self.mailbox)
        deadline = time.monotonic() + self.step_timeout_s
        last_sentinel_submit = 0.0
        sentinel_events: list[dict[str, Any]] = []

        while time.monotonic() < deadline:
            res = waypoint_mailbox._match_action_result(
                cursor.poll(), seq=active_seq, run_id=run_id
            )
            if res is not None:
                return res, sentinel_events

            if self._sentinel_coord is not None:
                # Consume any completed verdict BEFORE submitting a new observation so
                # latest-wins submission cannot overwrite an unconsumed verdict.
                try:
                    dispatched = self._sentinel_coord.poll_and_dispatch(
                        expected_active_seq=active_seq
                    )
                except TypeError:
                    dispatched = self._sentinel_coord.poll_and_dispatch()
                if dispatched is not None and dispatched.get("dispatched"):
                    verdict = dispatched["verdict"]
                    disp_payload = dispatched.get("payload") or {}
                    disp_seq = disp_payload.get("seq")
                    event = {
                        "act": verdict.act,
                        "target_id": verdict.target_id,
                        "reason": verdict.reason,
                        "latency_ms": round(verdict.latency_ms, 1),
                        "dispatched_seq": disp_seq,
                        "action_epoch": disp_payload.get("action_epoch"),
                    }
                    sentinel_events.append(event)
                    if verdict.act == "STOP":
                        self._sentinel_stop_locked = True
                        stop_receipt = (
                            self._wait_for_control_receipt(
                                int(disp_seq),
                                {"stop_accepted"},
                                run_id=run_id,
                                cursor=cursor,
                            )
                            if isinstance(disp_seq, int)
                            else {"status": "missing_stop_seq"}
                        )
                        self._sentinel_stop_confirmed = (
                            stop_receipt.get("event") == "stop_accepted"
                        )
                        event["closing_receipt"] = stop_receipt
                        return {
                            "status": "sentinel_stop",
                            "event": "sentinel_stop",
                            "success": False,
                            "locked_terminal": True,
                            "stop_confirmed": self._sentinel_stop_confirmed,
                            "seq": disp_seq,
                            "stop_receipt": stop_receipt,
                            "reason": verdict.reason,
                        }, sentinel_events
                    if verdict.act in ("PREEMPT", "SPIN") and isinstance(disp_seq, int):
                        # Track the newly dispatched preempt/spin seq until its real receipt!
                        active_seq = int(disp_seq)
                        continue

                now = time.monotonic()
                if now - last_sentinel_submit >= self.sentinel_poll_interval_s:
                    obs = waypoint_mailbox.read_observation(self.mailbox)
                    sim_frame = waypoint_mailbox.latest_sim_frame(self.mailbox.parent)
                    rgb_live = (
                        (sim_frame.get("latest") or sim_frame.get("rgb_path"))
                        if isinstance(sim_frame, dict)
                        else None
                    )
                    self._sentinel_coord.submit_observation(
                        observation=obs,
                        candidates=candidates,
                        backend_status=self._read_backend_status(),
                        rgb_path=rgb_live,
                    )
                    last_sentinel_submit = now

            time.sleep(0.1)

        # Cancel the timed-out active action and wait for its closing receipt.
        cancel_receipt: dict[str, Any] = {}
        try:
            epoch = self._current_action_epoch()
            cancel_payload = waypoint_mailbox.submit(
                self.mailbox,
                action="cancel",
                pose_source=self.pose_source,
                extra={"action_epoch": epoch} if epoch > 0 else None,
            )
            cancel_receipt = self._wait_for_control_receipt(
                int(cancel_payload["seq"]),
                {"cancel_accepted"},
                run_id=run_id,
                cursor=cursor,
            )
        except OSError as exc:
            cancel_receipt = {"status": "cancel_submit_error", "error": type(exc).__name__}
        result = waypoint_mailbox.timeout_result(active_seq, self.step_timeout_s, str(run_id or ""))
        result["cancel_receipt"] = cancel_receipt
        return result, sentinel_events

    def run(self, *, max_steps: int = 12) -> dict[str, Any]:
        """Execute a bounded closed-loop static task and return full diagnostic report."""
        t_start = time.monotonic()
        step_logs: list[dict[str, Any]] = []
        last_feedback = "Episode started."
        pending_recall: dict[str, Any] | None = None
        extra_l3_rgb: list[str] = []
        stop_verified = False
        terminal_status = "max_steps_reached"
        prev_xyz: list[float] | None = None
        total_travel_m = 0.0
        cleanup_status: dict[str, Any] = {"requested": False}

        try:
            for step in range(1, max(1, int(max_steps)) + 1):
                if self._sentinel_stop_locked:
                    terminal_status = "sentinel_stop_locked"
                    break

                obs, pose_pair = self._wait_for_valid_pose()
                if pose_pair is None:
                    terminal_status = "pose_unavailable_or_terminal"
                    break
                robot_xyz, yaw_rad = pose_pair

                # Sync backend loop-closure pose updates into hierarchical memory when available.
                self._sync_memory_from_backend_map()

                if prev_xyz is not None:
                    total_travel_m += float(
                        np.linalg.norm(np.asarray(robot_xyz) - np.asarray(prev_xyz))
                    )
                prev_xyz = list(robot_xyz)

                token_before = self._read_control_token(obs)
                snap = self._capture_snapshot(f"task_step_{step:03d}")
                if isinstance(snap, dict) and snap.get("error"):
                    last_feedback = f"snapshot unavailable ({snap.get('error')}); retrying safely."
                    step_logs.append(
                        TaskStepLog(
                            step=step,
                            robot_xyz=[round(v, 3) for v in robot_xyz],
                            yaw_deg=round(math.degrees(yaw_rad), 1),
                            decision={"action": "noop", "reason": "snapshot_error"},
                            execution={"status": "snapshot_error", "error": snap.get("error")},
                            memory_diag=self.memory.diagnostic_metrics(),
                        ).__dict__
                    )
                    continue

                rgb_path = snap.get("products", {}).get("rgb")
                local_map_path, raw_cands = select_active_chart_and_candidates(snap, robot_xyz[2])

                # For text-only planners (e.g. DeepSeek high), run dedicated VLM observer
                # on current RGB and ground landmarks with independent LiDAR range BEFORE planning.
                pre_kf_id: str | None = None
                vision_obs_summary: dict[str, Any] | None = None
                if (
                    not self.planner.config.supports_vision
                    and self.vision_observer is not None
                    and rgb_path is not None
                ):
                    vision_obs_summary = self.vision_observer.observe(
                        rgb_path, target_label=self.target_label
                    )
                    if vision_obs_summary.get("ok"):
                        pre_kf_id = self._record_live_step_in_memory(
                            step,
                            robot_xyz,
                            yaw_rad,
                            rgb_path,
                            vision_obs_summary.get("observed_objects", ()),
                            snapshot=snap,
                        )
                        cap = vision_obs_summary.get("scene_caption", "")
                        objs_txt = ", ".join(
                            f"{o['label']}({o['bearing_deg']:+.0f}deg)"
                            for o in vision_obs_summary.get("observed_objects", [])
                        )
                        obs_note = (
                            f"Front camera ({self.vision_observer.config.model}): {cap}; "
                            f"detected=[{objs_txt or 'none'}]."
                        )
                        last_feedback = f"{last_feedback} | {obs_note}".strip(" |")

                mem_cands = self.memory.memory_goal_candidates(
                    robot_xyz,
                    target_label=self.target_label,
                    standoff_m=min(1.0, max(0.5, self.stop_radius_m * 0.6)),
                )
                combined_cands = [*mem_cands, *raw_cands]
                allowed_cands, blocked_cands = self.memory.filter_candidates(
                    combined_cands, default_z=robot_xyz[2]
                )
                allowed_mem_cands = [c for c in allowed_cands if c.get("kind") == "memory_target"]
                # Never silently re-enable blacklisted candidates when all are blocked.
                cands_for_planner = allowed_cands
                if not cands_for_planner and blocked_cands:
                    last_feedback = (
                        f"{last_feedback} All {len(blocked_cands)} goto candidates are blacklisted; "
                        "use query_memory, scan, or maneuver to escape."
                    ).strip()

                mem_summary = self.memory.narrative_summary(
                    robot_xyz=robot_xyz,
                    target_label=self.target_label,
                    memory_candidates=allowed_mem_cands,
                )
                l0 = self.memory.l0_global_topology()

                decision = self.planner.decide(
                    task_instruction=self.task_instruction,
                    step_index=step,
                    robot_xyz=robot_xyz,
                    yaw_rad=yaw_rad,
                    candidates=cands_for_planner,
                    memory_summary=mem_summary,
                    memory_l0=l0,
                    target_label=self.target_label,
                    stop_radius_m=self.stop_radius_m,
                    estimated_travel_m=total_travel_m,
                    last_feedback=last_feedback,
                    tool_recall_result=pending_recall,
                    rgb_path=rgb_path,
                    local_map_path=local_map_path,
                    extra_rgb_paths=extra_l3_rgb,
                )
                pending_recall = None
                extra_l3_rgb = []

                # Verify full control token (run_id, active_seq, epoch) did not change during inference.
                token_after = self._read_control_token()
                if token_after != token_before:
                    exec_info = {
                        "status": "discarded_stale_epoch",
                        "token_before": list(token_before),
                        "token_after": list(token_after),
                    }
                    last_feedback = (
                        f"Decision discarded due to control state shift "
                        f"({token_before}->{token_after})."
                    )
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
                    continue

                if decision.action == "noop":
                    exec_info = {
                        "status": "noop",
                        "fallback": decision.fallback,
                        "reason": decision.reason,
                    }
                    last_feedback = f"Planner returned noop ({decision.reason}); holding position."
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
                    continue

                # Only vision-capable planners may log directly observed landmarks;
                # text-only planners rely on the dedicated VisionLandmarkObserver above.
                if pre_kf_id is not None:
                    kf_id = pre_kf_id
                else:
                    obs_to_record = (
                        decision.observed_objects if self.planner.config.supports_vision else ()
                    )
                    kf_id = self._record_live_step_in_memory(
                        step,
                        robot_xyz,
                        yaw_rad,
                        rgb_path,
                        obs_to_record,
                        snapshot=snap,
                    )

                exec_info = {}
                sentinel_events: list[dict[str, Any]] = []

                if decision.action == "query_memory":
                    q = decision.query or self.target_label or self.task_instruction
                    q_l1 = self.memory.l1_submap_summaries(query=q, near_xyz=robot_xyz, top_k=3)
                    q_l2 = self.memory.l2_submap_entities(label=q)
                    # Ground L3 imagery on matched target object anchor frame first!
                    q_l3: list[dict[str, Any]] = []
                    if q_l2["objects"]:
                        top_obj_id = q_l2["objects"][0]["object_id"]
                        q_l3 = self.memory.l3_on_demand_keyframes(
                            object_id=top_obj_id, max_images=2
                        )
                    if not q_l3:
                        q_l3 = self.memory.l3_on_demand_keyframes(
                            near_xyz=robot_xyz, max_images=2
                        )
                    validated_l3: list[str] = []
                    rejected_l3: list[dict[str, str]] = []
                    sanitized_l3_meta: list[dict[str, Any]] = []
                    for row in q_l3:
                        raw_p = row.get("rgb_path")
                        kf_ref = str(row.get("keyframe_id", "unknown"))
                        if not raw_p:
                            continue
                        res_p, rej_r = validate_image_path(
                            raw_p, allowed_roots=self.allowed_image_roots
                        )
                        if res_p is not None:
                            validated_l3.append(str(res_p))
                            sanitized_l3_meta.append(
                                {
                                    "keyframe_id": kf_ref,
                                    "pose_xyz": row.get("pose_xyz"),
                                    "map_revision": row.get("map_revision"),
                                }
                            )
                        else:
                            # Never leak rejected raw local filesystem paths into model prompts.
                            rejected_l3.append({"keyframe_id": kf_ref, "reason": str(rej_r)})
                    extra_l3_rgb = validated_l3
                    pending_recall = {
                        "query": q,
                        "l1_rooms": q_l1[:3],
                        "l2_objects": q_l2["objects"][:6],
                        "l2_failures": q_l2["failures"][:4],
                        "l3_keyframes": sanitized_l3_meta,
                        "l3_rejected": rejected_l3,
                    }
                    exec_info = {
                        "status": "memory_queried",
                        "l1_hits": len(q_l1),
                        "l2_objects": len(q_l2["objects"]),
                        "l3_images": len(extra_l3_rgb),
                        "l3_rejected": len(rejected_l3),
                    }
                    last_feedback = (
                        f"query_memory('{q}') returned {len(q_l1)} rooms, "
                        f"{len(q_l2['objects'])} objects, {len(extra_l3_rgb)} L3 images."
                    )

                elif decision.action == "stop_claim":
                    # Freeze task target: require frozen target_label and reject overrides!
                    if self.target_label is None:
                        exec_info = {
                            "allowed": False,
                            "reason": "missing_frozen_target_label",
                        }
                        self.memory.record_attempt(
                            "stop_claim",
                            robot_xyz,
                            "rejected",
                            reason="missing_frozen_target_label",
                            keyframe_id=kf_id,
                            blocks_location=False,
                        )
                        last_feedback = "STOP REJECTED (task has no frozen target_label configured)."
                    elif (
                        decision.target_label is not None
                        and not _label_strict_match(
                            self.target_label,
                            decision.target_label,
                            self.target_aliases,
                        )
                    ):
                        exec_info = {
                            "allowed": False,
                            "reason": "target_label_mismatch",
                            "expected_target": self.target_label,
                            "claimed_target": decision.target_label,
                        }
                        self.memory.record_attempt(
                            "stop_claim",
                            robot_xyz,
                            "rejected",
                            reason="target_label_mismatch",
                            keyframe_id=kf_id,
                            blocks_location=False,
                        )
                        last_feedback = (
                            f"STOP REJECTED (target_label_mismatch: claimed '{decision.target_label}', "
                            f"task requires '{self.target_label}')."
                        )
                    else:
                        verify = self.memory.verify_stop_claim(
                            robot_xyz,
                            target_label=self.target_label,
                            stop_radius_m=self.stop_radius_m,
                            allowed_aliases=self.target_aliases,
                            excluded_producers=("opencode_vlm", "opencode_vlm_unverified"),
                        )
                        exec_info = verify.to_dict()
                        if verify.allowed:
                            stop_verified = True
                            terminal_status = "verified_stop_success"
                            last_feedback = (
                                f"STOP verified: {verify.reason} "
                                f"(dist={verify.distance_m}m, source={verify.range_source})"
                            )
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
                        self.memory.record_attempt(
                            "stop_claim",
                            robot_xyz,
                            "rejected",
                            reason=verify.reason,
                            keyframe_id=kf_id,
                            blocks_location=False,
                        )
                        last_feedback = (
                            f"STOP REJECTED ({verify.reason}, dist={verify.distance_m}). "
                            "Move closer to the target on the same floor and confirm with measured range."
                        )

                elif decision.action in ("goto_candidate", "goto_xyz"):
                    goal_xyz: list[float] | None = None
                    if decision.action == "goto_candidate":
                        for c in cands_for_planner:
                            if c.get("id") == decision.target_id:
                                xy = c.get("xy")
                                if isinstance(xy, (list, tuple)) and len(xy) >= 2:
                                    goal_xyz = [
                                        float(xy[0]),
                                        float(xy[1]),
                                        float(c.get("z", robot_xyz[2])),
                                    ]
                                elif isinstance(c.get("goal_xyz_m"), (list, tuple)):
                                    goal_xyz = [float(v) for v in c["goal_xyz_m"]]
                                break
                    elif decision.xyz is not None:
                        goal_xyz = [float(v) for v in decision.xyz]

                    if goal_xyz is None:
                        exec_info = {"status": "skipped", "reason": "unresolved_goal"}
                        last_feedback = "Could not resolve target coordinates; pick a valid candidate ID."
                    elif self.memory.match_blacklist(goal_xyz):
                        # Unified final-dispatch blacklist check covering both goto_candidate and goto_xyz!
                        exec_info = {
                            "status": "blacklisted_goal",
                            "reason": "goal_in_active_failure_blacklist",
                            "goal_xyz": [round(v, 3) for v in goal_xyz],
                        }
                        last_feedback = (
                            f"goto {[round(v, 2) for v in goal_xyz]} BLOCKED by active failure blacklist; "
                            "choose another route or maneuver."
                        )
                    else:
                        epoch = token_after[2]
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
                        if exec_info.get("locked_terminal") or self._sentinel_stop_locked:
                            terminal_status = "sentinel_stop_locked"
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
                            break

                elif decision.action == "maneuver":
                    epoch = token_after[2]
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
                    epoch = token_after[2]
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
                if self._sentinel_stop_locked and self._sentinel_stop_confirmed:
                    cleanup_status = {
                        "requested": False,
                        "reason": "already_stopped_by_sentinel",
                        "stop_confirmed": True,
                    }
                else:
                    cleanup_status = {"requested": True}
                    try:
                        epoch = self._current_action_epoch()
                        stop_payload = waypoint_mailbox.submit(
                            self.mailbox,
                            action="stop",
                            pose_source=self.pose_source,
                            extra={"action_epoch": epoch} if epoch > 0 else None,
                        )
                        cleanup_status["stop_seq"] = stop_payload["seq"]
                        receipt = self._wait_for_control_receipt(
                            int(stop_payload["seq"]),
                            {"stop_accepted"},
                            run_id=stop_payload.get("run_id"),
                        )
                        cleanup_status["receipt"] = receipt
                        cleanup_status["stop_confirmed"] = receipt.get("event") == "stop_accepted"
                    except OSError as exc:
                        cleanup_status["error"] = type(exc).__name__
                        cleanup_status["stop_confirmed"] = False

        if self.stop_simulator_on_exit:
            stop_confirmed = bool(cleanup_status.get("stop_confirmed", False))
        else:
            stop_confirmed = True
        task_succeeded = bool(stop_verified and stop_confirmed)
        if stop_verified and not stop_confirmed:
            terminal_status = "stop_unconfirmed"

        report = {
            "episode_id": self.episode_id,
            "task_instruction": self.task_instruction,
            "target_label": self.target_label,
            "target_aliases": list(self.target_aliases),
            "stop_verified": stop_verified,
            "stop_confirmed": stop_confirmed,
            "task_succeeded": task_succeeded,
            "terminal_status": terminal_status,
            "steps_executed": len(step_logs),
            "wall_s": round(time.monotonic() - t_start, 2),
            "estimated_travel_m": round(total_travel_m, 2),
            "memory_map_revision": self.memory.map_revision,
            "planner_config": self.planner.config.to_safe_dict(),
            "sentinel_config": (
                self.sentinel_config.to_safe_dict() if self.sentinel_config is not None else None
            ),
            "vision_observer_config": (
                self.vision_observer.config.to_safe_dict()
                if self.vision_observer is not None
                else None
            ),
            "vision_observer_latencies_ms": (
                [round(v, 1) for v in self.vision_observer.latencies_ms]
                if self.vision_observer is not None
                else []
            ),
            "planner_latencies_ms": [round(v, 1) for v in self.planner.latencies_ms],
            "memory_diagnostics": self.memory.diagnostic_metrics(),
            "sentinel_stats": (
                self._sentinel_runner.stats() if self._sentinel_runner is not None else None
            ),
            "cleanup": cleanup_status,
            "steps": step_logs,
        }
        if self.run_dir.is_dir():
            # Save versioned memory atomically first, then atomically publish the report referencing it.
            self.memory.save(self.run_dir / "hierarchical_memory.json")
            atomic_write_json(self.run_dir / "static_task_report.json", report)
        return report
