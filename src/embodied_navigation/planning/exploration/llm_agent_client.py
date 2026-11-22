"""Fast stdlib-only async LLM/VLM sentinel client for embodied navigation.

Provides OpenAI-compatible chat-completion transport for text-only reasoning
(e.g. DeepSeek) and multimodal RGB + visual-map evaluation (e.g. Qwen3.8-Flash
or local SGLang/vLLM servers), backed by a single-worker latest-wins mailbox
and fail-open parsing so inference hiccups never stall System 0 control.
"""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any, Callable, Iterable
import urllib.error
import urllib.request

VALID_SENTINEL_ACTS = ("CONT", "PREEMPT", "STOP", "SPIN")

_QWEN_PRESET = {
    "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
    "default_model": "qwen3.8-flash",
    "api_key_env": "DASHSCOPE_API_KEY",
    "supports_vision": True,
}

PROVIDER_PRESETS: dict[str, dict[str, Any]] = {
    "deepseek": {
        "base_url": "https://api.deepseek.com/v1",
        "default_model": "deepseek-chat",
        "api_key_env": "DEEPSEEK_API_KEY",
        "supports_vision": False,
    },
    "qwen3.8-flash": dict(_QWEN_PRESET),
    "qwen": dict(_QWEN_PRESET),
    "openai_compatible": {
        "base_url": "",
        "default_model": "qwen3.8-flash",
        "api_key_env": "OPENAI_API_KEY",
        "supports_vision": True,
    },
}

_EMPTY = object()

_SYSTEM_PROMPT = (
    "You are an embodied navigation safety sentinel. "
    "Decide CONT (keep course), PREEMPT (switch to a better F*/W* candidate), "
    "STOP (unsafe/stuck), or SPIN (in-place active SLAM loop spin). "
    'Reply ONLY with 1 compact JSON line: {"act":"CONT"|"PREEMPT"|"STOP"|"SPIN",'
    '"target_id":null|"F1"|"W1","reason":"short"}.'
)


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


@dataclass(frozen=True)
class LlmClientConfig:
    """Validated OpenAI-compatible client configuration."""

    provider: str = "deepseek"
    model: str | None = None
    base_url: str | None = None
    api_key: str | None = None
    timeout_s: float = 2.5
    max_tokens: int = 64
    temperature: float = 0.1
    supports_vision: bool | None = None

    def __post_init__(self) -> None:
        if self.provider not in PROVIDER_PRESETS:
            raise ValueError(f"unsupported provider: {self.provider!r}")
        preset = PROVIDER_PRESETS[self.provider]

        raw_model = preset["default_model"] if self.model is None else str(self.model)
        model = raw_model.strip()
        if not model:
            raise ValueError("model must be non-empty")

        raw_url = preset["base_url"] if self.base_url is None else str(self.base_url)
        base_url = raw_url.strip().rstrip("/")
        if not base_url:
            raise ValueError("base_url must be non-empty")

        timeout_s = _finite_float(self.timeout_s, "timeout_s")
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")

        if isinstance(self.max_tokens, bool) or not isinstance(self.max_tokens, int):
            raise ValueError("max_tokens must be a positive integer")
        if self.max_tokens <= 0:
            raise ValueError("max_tokens must be a positive integer")

        temperature = _finite_float(self.temperature, "temperature")
        if temperature < 0.0:
            raise ValueError("temperature must be non-negative")

        api_key = self.api_key
        if api_key is None:
            env_name = preset.get("api_key_env")
            if env_name:
                api_key = os.environ.get(env_name)
        elif not isinstance(api_key, str):
            raise ValueError("api_key must be a string or None")

        supports_vision = (
            bool(preset["supports_vision"])
            if self.supports_vision is None
            else bool(self.supports_vision)
        )

        object.__setattr__(self, "model", model)
        object.__setattr__(self, "base_url", base_url)
        object.__setattr__(self, "api_key", api_key)
        object.__setattr__(self, "timeout_s", timeout_s)
        object.__setattr__(self, "temperature", temperature)
        object.__setattr__(self, "supports_vision", supports_vision)


@dataclass(frozen=True)
class SentinelVerdict:
    """Structured sentinel decision with latency and fail-open flag."""

    act: str
    target_id: str | None = None
    reason: str = ""
    latency_ms: float = 0.0
    fallback: bool = False


def active_slam_health(
    backend_status: dict[str, Any] | None,
    *,
    max_travel_since_loop_m: float = 20.0,
    max_loop_age_s: float = 180.0,
) -> dict[str, Any]:
    """Summarize backend loop freshness and recommend an active relocalize spin when needed."""
    max_travel = _finite_float(max_travel_since_loop_m, "max_travel_since_loop_m")
    max_age = _finite_float(max_loop_age_s, "max_loop_age_s")
    if max_travel <= 0.0:
        raise ValueError("max_travel_since_loop_m must be positive")
    if max_age <= 0.0:
        raise ValueError("max_loop_age_s must be positive")

    status = backend_status if isinstance(backend_status, dict) else {}
    nested = status.get("stats")
    stats = nested if isinstance(nested, dict) else status

    raw_travel = stats.get("travel_since_last_loop_m")
    if raw_travel is None:
        raw_travel = status.get("travel_since_last_loop_m")
    raw_age = stats.get("last_loop_age_s")
    if raw_age is None:
        raw_age = status.get("last_loop_age_s")

    travel_m = _safe_optional_float(raw_travel)
    age_s = _safe_optional_float(raw_age)
    needs_loop = (
        (travel_m is not None and travel_m > max_travel)
        or (age_s is not None and age_s > max_age)
    )
    return {
        "needs_active_loop": bool(needs_loop),
        "recommended_action": "active_relocalize_spin" if needs_loop else "none",
        "travel_since_last_loop_m": travel_m,
        "last_loop_age_s": age_s,
    }


def _safe_optional_float(value: Any) -> float | None:
    if value is None or isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    out = float(value)
    return out if math.isfinite(out) else None


def _round_seq(values: Any, digits: int = 2) -> list[float] | None:
    if not isinstance(values, (list, tuple)):
        return None
    rounded: list[float] = []
    for item in values:
        num = _safe_optional_float(item)
        if num is None:
            return None
        rounded.append(round(num, digits))
    return rounded


def _compact_candidates(candidates: Iterable[dict[str, Any]] | None) -> list[dict[str, Any]]:
    compact: list[dict[str, Any]] = []
    if not candidates:
        return compact
    for item in candidates:
        if not isinstance(item, dict):
            continue
        cid = item.get("id")
        if not isinstance(cid, str) or not cid:
            continue
        entry: dict[str, Any] = {"id": cid}
        if "kind" in item:
            entry["kind"] = item["kind"]
        xy = _round_seq(item.get("xy") or item.get("goal_xyz_m"), 2)
        if xy is not None:
            entry["xy"] = xy[:2]
        dist = _safe_optional_float(item.get("dist_m", item.get("distance_m")))
        if dist is not None:
            entry["d"] = round(dist, 2)
        clr = _safe_optional_float(item.get("clearance_m"))
        if clr is not None:
            entry["clr"] = round(clr, 2)
        compact.append(entry)
        if len(compact) >= 10:
            break
    return compact


def _encode_image_part(rgb_path: str | Path) -> dict[str, Any] | None:
    path = Path(rgb_path)
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    if not raw:
        return None
    suffix = path.suffix.lower()
    if suffix in (".jpg", ".jpeg"):
        mime = "image/jpeg"
    elif suffix == ".webp":
        mime = "image/webp"
    else:
        mime = "image/png"
    encoded = base64.b64encode(raw).decode("ascii")
    return {
        "type": "image_url",
        "image_url": {
            "url": f"data:{mime};base64,{encoded}",
            "detail": "low",
        },
    }


def build_visual_first_messages(
    observation_summary: dict[str, Any] | None,
    candidates: Iterable[dict[str, Any]] | None,
    backend_health: dict[str, Any] | None,
    *,
    rgb_path: str | Path | None = None,
    supports_vision: bool = True,
) -> list[dict[str, Any]]:
    """Build an ultra-compact (< 350 tokens) OpenAI-compatible sentinel prompt."""
    obs = observation_summary if isinstance(observation_summary, dict) else {}
    pose = obs.get("pose") if isinstance(obs.get("pose"), dict) else {}
    waypoints = obs.get("waypoints") if isinstance(obs.get("waypoints"), dict) else {}
    turning = obs.get("turning_circle") if isinstance(obs.get("turning_circle"), dict) else {}

    pose_compact = {
        "xyz": _round_seq(pose.get("position_xyz_m") or obs.get("position_xyz_m"), 2),
        "yaw": _safe_optional_float(
            round(float(pose["yaw_rad"]), 2)
            if _safe_optional_float(pose.get("yaw_rad")) is not None
            else _safe_optional_float(obs.get("yaw_rad"))
        ),
    }
    nav_compact = {
        "seq": waypoints.get("active_seq"),
        "phase": waypoints.get("phase"),
        "dist_m": (
            round(float(waypoints["distance_to_goal_m"]), 2)
            if _safe_optional_float(waypoints.get("distance_to_goal_m")) is not None
            else None
        ),
    }
    tc_compact = {
        "safe": turning.get("safe"),
        "min_d": turning.get("min_obstacle_dist_m"),
        "obs": turning.get("obstacle_count"),
    }
    health = backend_health if isinstance(backend_health, dict) else {}
    slam_compact = {
        "need_loop": bool(health.get("needs_active_loop", False)),
        "rec": health.get("recommended_action", "none"),
        "travel_m": health.get("travel_since_last_loop_m"),
        "age_s": health.get("last_loop_age_s"),
    }
    payload = {
        "pose": pose_compact,
        "nav": nav_compact,
        "turning_circle": tc_compact,
        "candidates": _compact_candidates(candidates),
        "slam_health": slam_compact,
    }
    user_text = json.dumps(payload, separators=(",", ":"), allow_nan=False)

    image_part = _encode_image_part(rgb_path) if (supports_vision and rgb_path is not None) else None
    if image_part is not None:
        user_content: Any = [
            {"type": "text", "text": user_text},
            image_part,
        ]
    else:
        user_content = user_text

    return [
        {"role": "system", "content": _SYSTEM_PROMPT},
        {"role": "user", "content": user_content},
    ]


def _extract_json_object(raw_text: str) -> dict[str, Any] | None:
    text = raw_text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip().startswith("```"):
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    try:
        parsed = json.loads(text)
        return parsed if isinstance(parsed, dict) else None
    except ValueError:
        pass
    decoder = json.JSONDecoder()
    for index, ch in enumerate(text):
        if ch != "{":
            continue
        try:
            parsed, _ = decoder.raw_decode(text[index:])
        except ValueError:
            continue
        if isinstance(parsed, dict):
            return parsed
    return None


def parse_sentinel_verdict(
    raw_text: str | None, *, latency_ms: float = 0.0
) -> SentinelVerdict:
    """Parse JSON verdict from model output, failing open to CONT on malformed text."""
    latency = float(latency_ms) if math.isfinite(float(latency_ms)) else 0.0
    if not isinstance(raw_text, str) or not raw_text.strip():
        return SentinelVerdict(
            act="CONT",
            target_id=None,
            reason="empty_response",
            latency_ms=latency,
            fallback=True,
        )
    obj = _extract_json_object(raw_text)
    if obj is None:
        return SentinelVerdict(
            act="CONT",
            target_id=None,
            reason="malformed_json",
            latency_ms=latency,
            fallback=True,
        )
    act = str(obj.get("act", "")).strip().upper()
    if act not in VALID_SENTINEL_ACTS:
        return SentinelVerdict(
            act="CONT",
            target_id=None,
            reason=f"invalid_act:{act or 'missing'}",
            latency_ms=latency,
            fallback=True,
        )
    raw_target = obj.get("target_id")
    target_id = None if raw_target is None else (str(raw_target).strip() or None)
    reason = str(obj.get("reason", "")).strip()[:160]
    return SentinelVerdict(
        act=act,
        target_id=target_id,
        reason=reason,
        latency_ms=latency,
        fallback=False,
    )


def _is_timeout_error(exc: BaseException) -> bool:
    if isinstance(exc, (TimeoutError, socket.timeout)):
        return True
    if isinstance(exc, urllib.error.URLError):
        reason = getattr(exc, "reason", None)
        if isinstance(reason, (TimeoutError, socket.timeout)):
            return True
        if isinstance(reason, str) and "timed out" in reason.lower():
            return True
    return "timed out" in str(exc).lower()


def _extract_choice_content(response_json: dict[str, Any]) -> str:
    choices = response_json.get("choices")
    if not isinstance(choices, list) or not choices:
        raise ValueError("missing choices in chat completion response")
    message = choices[0].get("message")
    if not isinstance(message, dict):
        raise ValueError("missing message in chat completion choice")
    content = message.get("content")
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts = [
            str(block.get("text", ""))
            for block in content
            if isinstance(block, dict) and block.get("type") == "text"
        ]
        return "".join(parts)
    raise ValueError("unsupported message content type")


class FastSentinelClient:
    """OpenAI-compatible HTTP client for low-latency embodied sentinel decisions."""

    def __init__(self, config: LlmClientConfig) -> None:
        self.config = config

    def build_request_body(self, messages: list[dict[str, Any]]) -> dict[str, Any]:
        """Build the OpenAI-compatible /chat/completions JSON request payload."""
        return {
            "model": self.config.model,
            "messages": messages,
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "stream": False,
        }

    def evaluate_messages(self, messages: list[dict[str, Any]]) -> SentinelVerdict:
        """Send pre-built messages to /chat/completions; fails open to CONT on error."""
        body = json.dumps(
            self.build_request_body(messages), allow_nan=False
        ).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json",
        }
        if self.config.api_key:
            headers["Authorization"] = f"Bearer {self.config.api_key}"
        url = f"{self.config.base_url}/chat/completions"
        req = urllib.request.Request(url, data=body, headers=headers, method="POST")

        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=self.config.timeout_s) as resp:
                raw = resp.read().decode("utf-8")
            latency_ms = (time.perf_counter() - t0) * 1000.0
            parsed = json.loads(raw)
            content = _extract_choice_content(parsed)
            return parse_sentinel_verdict(content, latency_ms=latency_ms)
        except Exception as exc:
            latency_ms = (time.perf_counter() - t0) * 1000.0
            prefix = "timeout" if _is_timeout_error(exc) else "client_error"
            return SentinelVerdict(
                act="CONT",
                target_id=None,
                reason=f"{prefix}:{type(exc).__name__}",
                latency_ms=latency_ms,
                fallback=True,
            )

    def evaluate(
        self,
        observation_summary: dict[str, Any] | None,
        candidates: Iterable[dict[str, Any]] | None,
        backend_health: dict[str, Any] | None,
        *,
        rgb_path: str | Path | None = None,
    ) -> SentinelVerdict:
        """Build a visual-first prompt respecting provider vision support and evaluate."""
        messages = build_visual_first_messages(
            observation_summary,
            candidates,
            backend_health,
            rgb_path=rgb_path,
            supports_vision=bool(self.config.supports_vision),
        )
        return self.evaluate_messages(messages)


def _percentile(sorted_vals: list[float], quantile: float) -> float:
    if not sorted_vals:
        return 0.0
    idx = min(
        len(sorted_vals) - 1,
        max(0, int(math.ceil(quantile * len(sorted_vals)) - 1)),
    )
    return round(sorted_vals[idx], 3)


class AsyncDecisionRunner:
    """Single-worker background thread with a 1-slot latest-wins mailbox."""

    def __init__(self, evaluator: FastSentinelClient | Callable[[Any], SentinelVerdict]) -> None:
        self._evaluator = evaluator
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._pending_slot: Any = _EMPTY
        self._latest_verdict: SentinelVerdict | None = None
        self._latencies_ms: deque[float] = deque(maxlen=1000)
        self._calls = 0
        self._timeouts = 0
        self._dropped_stale = 0
        self._closed = False
        self._thread = threading.Thread(
            target=self._worker_loop,
            name="sentinel-async-runner",
            daemon=True,
        )
        self._thread.start()

    def _run_one(self, payload: Any) -> SentinelVerdict:
        t0 = time.perf_counter()
        try:
            if isinstance(self._evaluator, FastSentinelClient):
                if isinstance(payload, list):
                    return self._evaluator.evaluate_messages(payload)
                if isinstance(payload, dict) and "messages" in payload:
                    return self._evaluator.evaluate_messages(payload["messages"])
                if isinstance(payload, dict):
                    obs = payload.get("observation_summary")
                    if obs is None:
                        obs = payload.get("observation")
                    return self._evaluator.evaluate(
                        obs,
                        payload.get("candidates"),
                        payload.get("backend_health"),
                        rgb_path=payload.get("rgb_path"),
                    )
                raise TypeError("unsupported sentinel payload type")
            result = self._evaluator(payload)
            if not isinstance(result, SentinelVerdict):
                raise TypeError("evaluator must return SentinelVerdict")
            return result
        except Exception as exc:
            latency_ms = (time.perf_counter() - t0) * 1000.0
            prefix = "timeout" if _is_timeout_error(exc) else "runner_error"
            return SentinelVerdict(
                act="CONT",
                target_id=None,
                reason=f"{prefix}:{type(exc).__name__}",
                latency_ms=latency_ms,
                fallback=True,
            )

    def _worker_loop(self) -> None:
        while True:
            with self._cond:
                while self._pending_slot is _EMPTY and not self._closed:
                    self._cond.wait()
                if self._closed and self._pending_slot is _EMPTY:
                    return
                payload = self._pending_slot
                self._pending_slot = _EMPTY

            verdict = self._run_one(payload)
            with self._cond:
                self._calls += 1
                if verdict.fallback and "timeout" in verdict.reason.lower():
                    self._timeouts += 1
                self._latencies_ms.append(float(verdict.latency_ms))
                self._latest_verdict = verdict

    def submit(self, payload: Any) -> bool:
        """Overwrite the 1-slot mailbox with the newest payload without blocking."""
        with self._cond:
            if self._closed:
                return False
            if self._pending_slot is not _EMPTY:
                self._dropped_stale += 1
            self._pending_slot = payload
            self._cond.notify()
            return True

    def poll_latest(self) -> SentinelVerdict | None:
        """Consume and return the latest completed verdict without blocking, or None."""
        with self._lock:
            verdict = self._latest_verdict
            self._latest_verdict = None
            return verdict

    def stats(self) -> dict[str, Any]:
        """Return latency percentiles and call/timeout/drop counters."""
        with self._lock:
            sorted_lat = sorted(self._latencies_ms)
            return {
                "calls": self._calls,
                "timeouts": self._timeouts,
                "dropped_stale": self._dropped_stale,
                "p50_ms": _percentile(sorted_lat, 0.50),
                "p95_ms": _percentile(sorted_lat, 0.95),
            }

    @property
    def calls(self) -> int:
        with self._lock:
            return self._calls

    @property
    def timeouts(self) -> int:
        with self._lock:
            return self._timeouts

    @property
    def dropped_stale(self) -> int:
        with self._lock:
            return self._dropped_stale

    @property
    def p50_ms(self) -> float:
        return self.stats()["p50_ms"]

    @property
    def p95_ms(self) -> float:
        return self.stats()["p95_ms"]

    def shutdown(self, timeout_s: float = 2.0) -> None:
        """Stop the background worker cleanly."""
        with self._cond:
            self._closed = True
            self._pending_slot = _EMPTY
            self._cond.notify_all()
        self._thread.join(timeout=timeout_s)
