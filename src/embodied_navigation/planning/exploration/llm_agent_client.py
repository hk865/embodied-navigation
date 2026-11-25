"""Fast stdlib-only async LLM/VLM sentinel client for embodied navigation.

Provides OpenAI-compatible chat-completion transport for text-only reasoning
(e.g. DeepSeek) and multimodal RGB + visual-map evaluation (e.g. Qwen3.8-Flash
or local SGLang/vLLM servers), backed by SSE early-stop streaming, monotonic
deadline enforcement, abortable connections, a single-worker latest-wins mailbox,
and fail-open parsing so inference hiccups never stall System 0 control.
"""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import dataclass, field
import http.client
import json
import math
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any, Callable, Iterable
import urllib.parse
import uuid

VALID_SENTINEL_ACTS = ("CONT", "PREEMPT", "STOP", "SPIN")

_QWEN_PRESET = {
    "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
    "default_model": "qwen3.8-flash",
    "api_key_env": "DASHSCOPE_API_KEY",
    "supports_vision": True,
}

_OPENCODE_GO_PRESET = {
    "base_url": "https://opencode.ai/zen/go/v1",
    "default_model": "qwen3.8-flash",
    "api_key_env": "OPENCODE_API_KEY",
    "supports_vision": True,
    "opencode_auth_provider": "opencode",
    "opencode_auth_providers": ("opencode-go", "opencode"),
    "requires_session_header": True,
}

_OPENCODE_ZEN_PRESET = {
    "base_url": "https://opencode.ai/zen/v1",
    "default_model": "qwen3.8-flash",
    "api_key_env": "OPENCODE_API_KEY",
    "supports_vision": True,
    "opencode_auth_provider": "opencode",
    "opencode_auth_providers": ("opencode",),
    "requires_session_header": True,
}

PROVIDER_PRESETS: dict[str, dict[str, Any]] = {
    "deepseek": {
        "base_url": "https://api.deepseek.com/v1",
        "default_model": "deepseek-flash",
        "api_key_env": "DEEPSEEK_API_KEY",
        "supports_vision": False,
    },
    "qwen3.8-flash": dict(_QWEN_PRESET),
    "qwen": dict(_QWEN_PRESET),
    "opencode": dict(_OPENCODE_GO_PRESET),
    "opencode_go": dict(_OPENCODE_GO_PRESET),
    "opencode_zen": dict(_OPENCODE_ZEN_PRESET),
    "openai_compatible": {
        "base_url": "",
        "default_model": "qwen3.8-flash",
        "api_key_env": "OPENAI_API_KEY",
        "supports_vision": True,
    },
}


_SENSITIVE_KEY_SUBSTRINGS = (
    "key",
    "token",
    "secret",
    "auth",
    "password",
    "bearer",
    "cookie",
    "credential",
)

MAX_IMAGE_UPLOAD_BYTES = 10 * 1024 * 1024


def _redact_sensitive(obj: Any) -> Any:
    """Recursively redact credential-like fields inside nested dicts and lists."""
    if isinstance(obj, dict):
        redacted: dict[Any, Any] = {}
        for k, v in obj.items():
            key_lower = str(k).lower()
            if any(sub in key_lower for sub in _SENSITIVE_KEY_SUBSTRINGS):
                redacted[k] = "[REDACTED]"
            else:
                redacted[k] = _redact_sensitive(v)
        return redacted
    if isinstance(obj, (list, tuple)):
        return [_redact_sensitive(item) for item in obj]
    return obj


def _redact_url(url: str) -> str:
    """Strip userinfo, query parameters, and fragments, keeping scheme://host[:port]/path."""
    parsed = urllib.parse.urlparse(url)
    if not parsed.scheme or not parsed.netloc:
        return url
    host_port = parsed.netloc.rsplit("@", 1)[-1]
    return urllib.parse.urlunparse((parsed.scheme, host_port, parsed.path, "", "", ""))


def _strip_matching_quotes(value: str) -> str:
    cleaned = value.strip()
    if len(cleaned) >= 2 and cleaned[0] == cleaned[-1] and cleaned[0] in ('"', "'"):
        return cleaned[1:-1]
    return cleaned


def _parse_kv_line(line: str) -> tuple[str, str] | None:
    candidate = line
    if candidate.startswith("export") and len(candidate) > 6 and candidate[6].isspace():
        candidate = candidate[6:].lstrip()
    if "=" not in candidate:
        return None
    key, raw_val = candidate.split("=", 1)
    key = key.strip()
    if not key or not key.isidentifier():
        return None
    return key, _strip_matching_quotes(raw_val)


def _read_key_file(path: Path, env_name: str | None = None) -> str | None:
    """Read an API token from a KEY=VALUE or single-line raw token file."""
    try:
        raw_lines = [
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.strip().startswith("#")
        ]
    except OSError:
        return None
    if not raw_lines:
        return None

    if env_name:
        has_kv = False
        for line in raw_lines:
            parsed = _parse_kv_line(line)
            if parsed is None:
                continue
            has_kv = True
            key, val = parsed
            if key == env_name:
                return val or None
        if has_kv:
            return None
        return _strip_matching_quotes(raw_lines[0]) or None

    parsed = _parse_kv_line(raw_lines[0])
    if parsed is not None:
        return parsed[1] or None
    return _strip_matching_quotes(raw_lines[0]) or None


def _read_opencode_auth_key(provider_name: str) -> str | None:
    """Read an API key from OpenCode's XDG auth.json store when present."""
    xdg_data = os.environ.get("XDG_DATA_HOME")
    base = Path(xdg_data) if xdg_data else (Path.home() / ".local" / "share")
    auth_path = base / "opencode" / "auth.json"
    try:
        payload = json.loads(auth_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    entry = payload.get(provider_name) if isinstance(payload, dict) else None
    if isinstance(entry, dict):
        key = entry.get("key")
        if isinstance(key, str) and key.strip():
            return key.strip()
    return None


def _resolve_preset_api_key(preset: dict[str, Any]) -> str | None:
    env_name = preset.get("api_key_env")
    if isinstance(env_name, str) and env_name:
        env_val = os.environ.get(env_name)
        if env_val and env_val.strip():
            return env_val.strip()
        file_env = os.environ.get(f"{env_name}_FILE")
        if file_env and file_env.strip():
            token = _read_key_file(Path(file_env).expanduser(), env_name=env_name)
            if token:
                return token
    auth_providers = preset.get("opencode_auth_providers")
    if isinstance(auth_providers, (list, tuple)) and auth_providers:
        for auth_provider in auth_providers:
            if isinstance(auth_provider, str) and auth_provider:
                token = _read_opencode_auth_key(auth_provider)
                if token:
                    return token
        return None
    auth_provider = preset.get("opencode_auth_provider")
    if isinstance(auth_provider, str) and auth_provider:
        return _read_opencode_auth_key(auth_provider)
    return None

_EMPTY = object()

_SYSTEM_PROMPT = (
    "You are an embodied navigation safety sentinel. "
    "Decide CONT (keep course), PREEMPT (switch to a better F*/W* candidate), "
    "STOP (imminent collision or confirmed entrapment ONLY), or SPIN (in-place active SLAM loop spin). "
    "Never output STOP solely because nav.phase is brake/idle or candidates is empty; "
    "if turning_circle.safe is true and no imminent obstacle blocks the robot, output CONT. "
    "When two images are attached, image 1 is the front RGB view and image 2 is "
    "the local top-down occupancy map with F*/W* candidates. "
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
    api_key: str | None = field(default=None, repr=False)
    timeout_s: float = 2.5
    max_tokens: int = 64
    temperature: float = 0.1
    supports_vision: bool | None = None
    stream: bool = True
    extra_body: dict[str, Any] | None = field(default=None, repr=False)
    session_id: str | None = None
    user_agent: str = "embodied-navigation-agent/1.0"

    def __post_init__(self) -> None:
        if self.provider not in PROVIDER_PRESETS:
            raise ValueError(f"unsupported provider: {self.provider!r}")
        preset = PROVIDER_PRESETS[self.provider]

        raw_model = preset["default_model"] if self.model is None else str(self.model)
        model = raw_model.strip()
        if not model:
            raise ValueError("model must be non-empty")

        preset_base_url = str(preset.get("base_url") or "").strip().rstrip("/")
        raw_url = preset["base_url"] if self.base_url is None else str(self.base_url)
        base_url = raw_url.strip().rstrip("/")
        if not base_url:
            raise ValueError("base_url must be non-empty")
        parsed_url = urllib.parse.urlparse(base_url)
        if parsed_url.scheme not in ("http", "https") or not parsed_url.netloc:
            raise ValueError("base_url must be a valid http or https URL")

        host_overridden = False
        if self.base_url is not None and preset_base_url:
            preset_parsed = urllib.parse.urlparse(preset_base_url)
            if parsed_url.netloc.lower() != preset_parsed.netloc.lower():
                host_overridden = True

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
            if not host_overridden:
                api_key = _resolve_preset_api_key(preset)
        elif not isinstance(api_key, str):
            raise ValueError("api_key must be a string or None")

        if (
            api_key
            and parsed_url.scheme == "http"
            and (parsed_url.hostname or "").lower() not in ("localhost", "127.0.0.1", "::1")
        ):
            raise ValueError("refusing to send api_key over insecure http to non-localhost host")

        supports_vision = (
            bool(preset["supports_vision"])
            if self.supports_vision is None
            else bool(self.supports_vision)
        )
        stream = bool(self.stream)
        extra_body = None
        if self.extra_body is not None:
            if not isinstance(self.extra_body, dict):
                raise ValueError("extra_body must be a dict or None")
            extra_body = dict(self.extra_body)

        session_id = self.session_id
        if session_id is None and preset.get("requires_session_header"):
            session_id = str(uuid.uuid4())
        elif session_id is not None:
            session_id = str(session_id).strip() or None
        user_agent = str(self.user_agent or "embodied-navigation-agent/1.0").strip()
        if not user_agent:
            user_agent = "embodied-navigation-agent/1.0"

        object.__setattr__(self, "model", model)
        object.__setattr__(self, "base_url", base_url)
        object.__setattr__(self, "api_key", api_key)
        object.__setattr__(self, "timeout_s", timeout_s)
        object.__setattr__(self, "temperature", temperature)
        object.__setattr__(self, "supports_vision", supports_vision)
        object.__setattr__(self, "stream", stream)
        object.__setattr__(self, "extra_body", extra_body)
        object.__setattr__(self, "session_id", session_id)
        object.__setattr__(self, "user_agent", user_agent)

    def to_safe_dict(self) -> dict[str, Any]:
        """Export a redacted configuration dictionary free of credentials."""
        safe = {
            "provider": self.provider,
            "model": self.model,
            "base_url": _redact_url(self.base_url or ""),
            "timeout_s": self.timeout_s,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
            "supports_vision": self.supports_vision,
            "stream": self.stream,
            "has_api_key": bool(self.api_key),
        }
        if self.extra_body is not None:
            safe["extra_body"] = _redact_sensitive(self.extra_body)
        return safe


@dataclass(frozen=True)
class SentinelVerdict:
    """Structured sentinel decision with latency, frame/epoch identity and fail-open flag."""

    act: str
    target_id: str | None = None
    reason: str = ""
    latency_ms: float = 0.0
    fallback: bool = False
    frame_seq: int | None = None
    action_epoch: int | None = None


class ActiveSlamSpinTracker:
    """Deduplicate active-SLAM spin recommendations across repeated status polls."""

    def __init__(
        self,
        *,
        min_retry_travel_delta_m: float = 5.0,
        min_retry_keyframe_delta: int = 5,
        min_cooldown_s: float = 10.0,
    ) -> None:
        self.min_retry_travel_delta_m = _finite_float(
            min_retry_travel_delta_m, "min_retry_travel_delta_m"
        )
        if self.min_retry_travel_delta_m < 0.0:
            raise ValueError("min_retry_travel_delta_m must be non-negative")
        if isinstance(min_retry_keyframe_delta, bool) or not isinstance(
            min_retry_keyframe_delta, int
        ) or min_retry_keyframe_delta < 0:
            raise ValueError("min_retry_keyframe_delta must be a non-negative integer")
        self.min_retry_keyframe_delta = min_retry_keyframe_delta
        self.min_cooldown_s = _finite_float(min_cooldown_s, "min_cooldown_s")
        if self.min_cooldown_s < 0.0:
            raise ValueError("min_cooldown_s must be non-negative")

        self.in_flight_spin: bool = False
        self.last_spin_keyframe_count: int | None = None
        self.last_spin_loops: int | None = None
        self.last_spin_travel_m: float | None = None
        self.last_spin_time_s: float | None = None

    def mark_spin_started(
        self,
        *,
        keyframes: int | None = None,
        loops: int | None = None,
        travel_m: float | None = None,
        now_s: float | None = None,
    ) -> None:
        self.in_flight_spin = True
        if keyframes is not None and not isinstance(keyframes, bool):
            self.last_spin_keyframe_count = int(keyframes)
        if loops is not None and not isinstance(loops, bool):
            self.last_spin_loops = int(loops)
        self.last_spin_travel_m = _safe_optional_float(travel_m)
        self.last_spin_time_s = _safe_optional_float(now_s)

    def mark_spin_finished(self) -> None:
        self.in_flight_spin = False

    def assess(
        self,
        backend_status: dict[str, Any] | None,
        *,
        max_travel_since_loop_m: float = 20.0,
        max_loop_age_s: float = 180.0,
        now_s: float | None = None,
    ) -> dict[str, Any]:
        health = active_slam_health(
            backend_status,
            max_travel_since_loop_m=max_travel_since_loop_m,
            max_loop_age_s=max_loop_age_s,
        )
        status_dict = backend_status if isinstance(backend_status, dict) else {}
        keyframes = status_dict.get("keyframes")
        if isinstance(keyframes, bool) or not isinstance(keyframes, int):
            keyframes = None
        loops = status_dict.get("loops")
        if isinstance(loops, bool) or not isinstance(loops, int):
            loops = None
        travel_m = health["travel_since_last_loop_m"]

        if health["status"] != "needs_active_loop":
            if health["status"] == "healthy" and not self.in_flight_spin:
                self.last_spin_keyframe_count = None
                self.last_spin_loops = None
                self.last_spin_travel_m = None
                self.last_spin_time_s = None
            return dict(health, should_trigger_spin=False, suppressed_reason=None)

        if self.in_flight_spin:
            return dict(
                health,
                recommended_action="none",
                should_trigger_spin=False,
                suppressed_reason="spin_in_flight",
            )

        if (
            self.last_spin_keyframe_count is not None
            or self.last_spin_travel_m is not None
            or self.last_spin_time_s is not None
        ):
            kf_ok = (
                keyframes is not None
                and self.last_spin_keyframe_count is not None
                and (keyframes - self.last_spin_keyframe_count) >= self.min_retry_keyframe_delta
            )
            travel_ok = (
                travel_m is not None
                and self.last_spin_travel_m is not None
                and (travel_m - self.last_spin_travel_m) >= self.min_retry_travel_delta_m
            )
            time_ok = (
                now_s is not None
                and self.last_spin_time_s is not None
                and (float(now_s) - self.last_spin_time_s) >= self.min_cooldown_s
            )
            loop_changed = (
                loops is not None
                and self.last_spin_loops is not None
                and loops != self.last_spin_loops
            )
            if not (loop_changed or (kf_ok and travel_ok) or (time_ok and (kf_ok or travel_ok))):
                return dict(
                    health,
                    recommended_action="none",
                    should_trigger_spin=False,
                    suppressed_reason="duplicate_spin_state",
                )

        return dict(health, should_trigger_spin=True, suppressed_reason=None)


def active_slam_health(
    backend_status: dict[str, Any] | None,
    *,
    max_travel_since_loop_m: float = 20.0,
    max_loop_age_s: float = 180.0,
    spin_tracker: ActiveSlamSpinTracker | None = None,
    now_s: float | None = None,
) -> dict[str, Any]:
    """Summarize backend loop freshness and distinguish unknown/prior/healthy/needs_active_loop."""
    max_travel = _finite_float(max_travel_since_loop_m, "max_travel_since_loop_m")
    max_age = _finite_float(max_loop_age_s, "max_loop_age_s")
    if max_travel <= 0.0:
        raise ValueError("max_travel_since_loop_m must be positive")
    if max_age <= 0.0:
        raise ValueError("max_loop_age_s must be positive")

    if spin_tracker is not None:
        return spin_tracker.assess(
            backend_status,
            max_travel_since_loop_m=max_travel,
            max_loop_age_s=max_age,
            now_s=now_s,
        )

    status = backend_status if isinstance(backend_status, dict) else {}
    nested = status.get("stats")
    stats = nested if isinstance(nested, dict) else status

    mode = stats.get("mode") or status.get("mode")
    prior_map_declared = bool(
        mode == "prior_localization"
        or status.get("prior_map") is not None
        or (
            mode != "mapping"
            and "travel_m" in stats
            and stats.get("travel_m") is None
            and stats.get("travel_since_last_loop_m") is None
            and stats.get("last_loop_age_s") is None
            and "zupt_active" in stats
        )
    )

    raw_travel = stats.get("travel_since_last_loop_m")
    if raw_travel is None:
        raw_travel = status.get("travel_since_last_loop_m")
    raw_age = stats.get("last_loop_age_s")
    if raw_age is None:
        raw_age = status.get("last_loop_age_s")

    travel_m = _safe_optional_float(raw_travel)
    age_s = _safe_optional_float(raw_age)

    if prior_map_declared:
        health_status = "prior_localization"
        needs_loop = False
    elif travel_m is None and age_s is None:
        health_status = "unknown"
        needs_loop = False
    elif (travel_m is not None and travel_m > max_travel) or (
        age_s is not None and age_s > max_age
    ):
        health_status = "needs_active_loop"
        needs_loop = True
    else:
        health_status = "healthy"
        needs_loop = False

    return {
        "status": health_status,
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


def _candidate_ids_from(candidates: Iterable[dict[str, Any]] | None) -> list[str]:
    if not candidates:
        return []
    ids: list[str] = []
    for item in candidates:
        if not isinstance(item, dict):
            continue
        cid = item.get("id")
        if isinstance(cid, str) and cid.strip():
            ids.append(cid.strip())
        if len(ids) >= 10:
            break
    return ids


def _compact_candidates(candidates: Iterable[dict[str, Any]] | None) -> list[dict[str, Any]]:
    compact: list[dict[str, Any]] = []
    if not candidates:
        return compact
    for item in candidates:
        if not isinstance(item, dict):
            continue
        cid = item.get("id")
        if not isinstance(cid, str) or not cid.strip():
            continue
        entry: dict[str, Any] = {"id": cid.strip()}
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


def _detect_image_mime(raw: bytes) -> str | None:
    """Detect image MIME type strictly from magic bytes."""
    if raw.startswith(b"\x89PNG\r\n\x1a\n"):
        return "image/png"
    if raw.startswith(b"\xff\xd8\xff"):
        return "image/jpeg"
    if len(raw) >= 12 and raw[:4] == b"RIFF" and raw[8:12] == b"WEBP":
        return "image/webp"
    return None


def _read_validated_image_bytes(
    image_path: str | Path,
    *,
    allowed_roots: Iterable[str | Path] | None = None,
    max_bytes: int = MAX_IMAGE_UPLOAD_BYTES,
) -> tuple[Path | None, bytes | None, str | None, str | None]:
    """Read and validate image bytes ONCE (bound to the exact bytes encoded)."""
    try:
        resolved = Path(image_path).resolve(strict=True)
    except (OSError, RuntimeError, ValueError):
        return None, None, None, "missing_or_unreadable"
    if not resolved.is_file():
        return None, None, None, "not_a_file"
    if allowed_roots is not None:
        roots = [Path(r).resolve() for r in allowed_roots]
        if not any(resolved.is_relative_to(root) for root in roots):
            return None, None, None, "outside_allowed_roots"
    try:
        with resolved.open("rb") as fh:
            raw = fh.read(max_bytes + 1)
    except OSError:
        return None, None, None, "missing_or_unreadable"
    if not raw or len(raw) > max_bytes:
        return None, None, None, "invalid_file_size"
    mime = _detect_image_mime(raw)
    if mime is None:
        return None, None, None, "invalid_image_header"
    return resolved, raw, mime, None


def validate_image_path(
    image_path: str | Path,
    *,
    allowed_roots: Iterable[str | Path] | None = None,
    max_bytes: int = MAX_IMAGE_UPLOAD_BYTES,
) -> tuple[Path | None, str | None]:
    """Validate image existence, root containment, byte budget, and magic header."""
    resolved, _raw, _mime, err = _read_validated_image_bytes(
        image_path, allowed_roots=allowed_roots, max_bytes=max_bytes
    )
    return resolved, err


def _encode_image_part(
    image_path: str | Path,
    *,
    allowed_roots: Iterable[str | Path] | None = None,
    max_bytes: int = MAX_IMAGE_UPLOAD_BYTES,
) -> dict[str, Any] | None:
    resolved, raw, mime, err = _read_validated_image_bytes(
        image_path, allowed_roots=allowed_roots, max_bytes=max_bytes
    )
    if resolved is None or raw is None or mime is None or err is not None:
        return None
    encoded = base64.b64encode(raw).decode("ascii")
    return {
        "type": "image_url",
        "image_url": {
            "url": f"data:{mime};base64,{encoded}",
            "detail": "low",
        },
    }


def _compact_memory_l0(raw_l0: dict[str, Any]) -> dict[str, Any]:
    raw_rev = raw_l0.get("map_revision", raw_l0.get("rev", 0))
    if isinstance(raw_rev, bool) or not isinstance(raw_rev, (int, float)):
        rev = 0
    else:
        rev = int(raw_rev) if math.isfinite(float(raw_rev)) else 0

    floors: list[dict[str, Any]] = []
    raw_floors = raw_l0.get("floors")
    if isinstance(raw_floors, (list, tuple)):
        for fl in raw_floors[:4]:
            if not isinstance(fl, dict):
                continue
            entry: dict[str, Any] = {}
            fid = fl.get("id", fl.get("floor_id"))
            if isinstance(fid, str) and fid.strip():
                entry["id"] = fid.strip()[:32]
            z_val = _safe_optional_float(fl.get("z", fl.get("z_mean_m")))
            if z_val is not None:
                entry["z"] = round(z_val, 2)
            r_cnt = fl.get("rooms")
            if isinstance(r_cnt, int) and not isinstance(r_cnt, bool):
                entry["rooms"] = r_cnt
            floors.append(entry)

    rooms: list[dict[str, Any]] = []
    raw_rooms = raw_l0.get("rooms")
    if isinstance(raw_rooms, (list, tuple)):
        for rm in raw_rooms[:8]:
            if not isinstance(rm, dict):
                continue
            entry = {}
            rid = rm.get("id", rm.get("room_id"))
            if isinstance(rid, str) and rid.strip():
                entry["id"] = rid.strip()[:32]
            fid = rm.get("f", rm.get("floor_id"))
            if isinstance(fid, str) and fid.strip():
                entry["f"] = fid.strip()[:24]
            xy = _round_seq(rm.get("c", rm.get("centroid_xyz")), 1)
            if xy is not None:
                entry["c"] = xy[:2]
            obj_cnt = rm.get("o", rm.get("object_count"))
            if isinstance(obj_cnt, int) and not isinstance(obj_cnt, bool):
                entry["o"] = obj_cnt
            alias = rm.get("a", rm.get("alias"))
            if isinstance(alias, str) and alias.strip():
                entry["a"] = alias.strip()[:32]
            rooms.append(entry)

    compact: dict[str, Any] = {"rev": rev, "floors": floors, "rooms": rooms}
    fail_cnt = raw_l0.get("fail_cnt")
    if isinstance(fail_cnt, int) and not isinstance(fail_cnt, bool):
        compact["fail_cnt"] = fail_cnt

    def _encoded_size(obj: dict[str, Any]) -> int:
        return len(json.dumps(obj, separators=(",", ":"), allow_nan=False).encode("utf-8"))

    while _encoded_size(compact) > 1024:
        if compact["rooms"]:
            compact["rooms"].pop()
        elif compact["floors"]:
            compact["floors"].pop()
        elif "fail_cnt" in compact:
            compact.pop("fail_cnt", None)
        else:
            break
    return compact


def build_visual_first_messages(
    observation_summary: dict[str, Any] | None,
    candidates: Iterable[dict[str, Any]] | None,
    backend_health: dict[str, Any] | None,
    *,
    rgb_path: str | Path | None = None,
    local_map_path: str | Path | None = None,
    supports_vision: bool = True,
    allowed_image_roots: Iterable[str | Path] | None = None,
) -> list[dict[str, Any]]:
    """Build an ultra-compact OpenAI-compatible sentinel prompt with optional dual images."""
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
        "status": health.get("status", "unknown"),
        "need_loop": bool(health.get("needs_active_loop", False)),
        "rec": health.get("recommended_action", "none"),
        "travel_m": health.get("travel_since_last_loop_m"),
        "age_s": health.get("last_loop_age_s"),
    }

    image_parts: list[dict[str, Any]] = []
    image_labels: list[str] = []
    if supports_vision:
        if rgb_path is not None:
            rgb_part = _encode_image_part(rgb_path, allowed_roots=allowed_image_roots)
            if rgb_part is not None:
                image_parts.append(rgb_part)
                image_labels.append("front_rgb")
        if local_map_path is not None:
            map_part = _encode_image_part(
                local_map_path, allowed_roots=allowed_image_roots
            )
            if map_part is not None:
                image_parts.append(map_part)
                image_labels.append("local_topdown_candidates")

    payload: dict[str, Any] = {
        "pose": pose_compact,
        "nav": nav_compact,
        "turning_circle": tc_compact,
        "candidates": _compact_candidates(candidates),
        "slam_health": slam_compact,
    }
    if isinstance(obs.get("memory_l0"), dict):
        payload["memory_l0"] = _compact_memory_l0(obs["memory_l0"])
    if obs.get("all_candidates_blacklisted") is True:
        payload["all_candidates_blacklisted"] = True
    if image_labels:
        payload["images"] = image_labels
    user_text = json.dumps(payload, separators=(",", ":"), allow_nan=False)

    if image_parts:
        user_content: Any = [{"type": "text", "text": user_text}, *image_parts]
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
    raw_text: str | None,
    *,
    latency_ms: float = 0.0,
    valid_candidate_ids: Iterable[str] | None = None,
    frame_seq: int | None = None,
    action_epoch: int | None = None,
) -> SentinelVerdict:
    """Parse JSON verdict from model output, failing open to CONT on malformed or invalid target."""
    latency = float(latency_ms) if math.isfinite(float(latency_ms)) else 0.0
    if not isinstance(raw_text, str) or not raw_text.strip():
        return SentinelVerdict(
            act="CONT",
            target_id=None,
            reason="empty_response",
            latency_ms=latency,
            fallback=True,
            frame_seq=frame_seq,
            action_epoch=action_epoch,
        )
    obj = _extract_json_object(raw_text)
    if obj is None:
        return SentinelVerdict(
            act="CONT",
            target_id=None,
            reason="malformed_json",
            latency_ms=latency,
            fallback=True,
            frame_seq=frame_seq,
            action_epoch=action_epoch,
        )
    act = str(obj.get("act", "")).strip().upper()
    if act not in VALID_SENTINEL_ACTS:
        return SentinelVerdict(
            act="CONT",
            target_id=None,
            reason=f"invalid_act:{act or 'missing'}",
            latency_ms=latency,
            fallback=True,
            frame_seq=frame_seq,
            action_epoch=action_epoch,
        )

    raw_target = obj.get("target_id")
    if act == "PREEMPT":
        if not isinstance(raw_target, str) or not raw_target.strip():
            return SentinelVerdict(
                act="CONT",
                target_id=None,
                reason="invalid_preempt_target",
                latency_ms=latency,
                fallback=True,
                frame_seq=frame_seq,
                action_epoch=action_epoch,
            )
        target_id = raw_target.strip()
        if valid_candidate_ids is not None:
            allowed = {
                cid.strip()
                for cid in valid_candidate_ids
                if isinstance(cid, str) and cid.strip()
            }
            if target_id not in allowed:
                return SentinelVerdict(
                    act="CONT",
                    target_id=None,
                    reason=f"unknown_preempt_target:{target_id}",
                    latency_ms=latency,
                    fallback=True,
                    frame_seq=frame_seq,
                    action_epoch=action_epoch,
                )
    else:
        target_id = None

    reason = str(obj.get("reason", "")).strip()[:160]
    return SentinelVerdict(
        act=act,
        target_id=target_id,
        reason=reason,
        latency_ms=latency,
        fallback=False,
        frame_seq=frame_seq,
        action_epoch=action_epoch,
    )


def _is_timeout_error(exc: BaseException) -> bool:
    if isinstance(exc, (TimeoutError, socket.timeout)):
        return True
    reason = getattr(exc, "reason", None)
    if isinstance(reason, (TimeoutError, socket.timeout)):
        return True
    if isinstance(reason, str) and "timed out" in reason.lower():
        return True
    return "timed out" in str(exc).lower()


def _extract_text_value(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return "".join(
            str(block.get("text", ""))
            for block in value
            if isinstance(block, dict) and block.get("type") == "text"
        )
    return ""


def _extract_choice_content(response_json: dict[str, Any]) -> str:
    choices = response_json.get("choices")
    if not isinstance(choices, list) or not choices:
        raise ValueError("missing choices in chat completion response")
    message = choices[0].get("message")
    if not isinstance(message, dict):
        raise ValueError("missing message in chat completion choice")
    content = message.get("content")
    if isinstance(content, (str, list)):
        return _extract_text_value(content)
    raise ValueError("unsupported message content type")


def _extract_sse_delta_content(chunk_json: dict[str, Any]) -> str:
    choices = chunk_json.get("choices")
    if not isinstance(choices, list) or not choices:
        return ""
    first = choices[0]
    if not isinstance(first, dict):
        return ""
    delta = first.get("delta")
    if isinstance(delta, dict):
        return _extract_text_value(delta.get("content"))
    message = first.get("message")
    if isinstance(message, dict):
        return _extract_text_value(message.get("content"))
    return ""


class _RequestCancelToken:
    """Thread-safe abort handle that shuts down an in-flight HTTP socket immediately."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cancelled = threading.Event()
        self._conn: http.client.HTTPConnection | None = None

    @property
    def is_cancelled(self) -> bool:
        return self._cancelled.is_set()

    def attach(self, conn: http.client.HTTPConnection) -> bool:
        with self._lock:
            if self._cancelled.is_set():
                return False
            self._conn = conn
            return True

    def detach(self) -> None:
        with self._lock:
            self._conn = None

    def cancel(self) -> None:
        self._cancelled.set()
        with self._lock:
            conn = self._conn
            sock = conn.sock if conn is not None else None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


class _DeadlineExceededError(TimeoutError):
    """Raised when the monotonic end-to-end request deadline elapses."""


class _RequestCancelledError(RuntimeError):
    """Raised when an active request is aborted by shutdown or cancellation."""


class FastSentinelClient:
    """OpenAI-compatible HTTP client for low-latency embodied sentinel decisions."""

    def __init__(
        self,
        config: LlmClientConfig,
        *,
        allowed_image_roots: Iterable[str | Path] | None = None,
    ) -> None:
        self.config = config
        self.allowed_image_roots = (
            tuple(Path(root) for root in allowed_image_roots)
            if allowed_image_roots is not None
            else None
        )
        self.last_request_modalities: dict[str, Any] = {
            "images": [],
            "has_memory_l0": False,
            "l0_rev": None,
        }

    def _record_modalities_from_messages(self, messages: list[dict[str, Any]]) -> None:
        images: list[str] = []
        has_memory_l0 = False
        l0_rev: int | None = None
        if messages and isinstance(messages[-1], dict):
            content = messages[-1].get("content")
            text_part = ""
            image_count = 0
            if isinstance(content, str):
                text_part = content
            elif isinstance(content, list):
                for block in content:
                    if not isinstance(block, dict):
                        continue
                    btype = block.get("type")
                    if btype == "text" and not text_part:
                        text_part = str(block.get("text", ""))
                    elif btype == "image_url":
                        image_count += 1
            if text_part:
                try:
                    parsed = json.loads(text_part)
                except ValueError:
                    parsed = None
                if isinstance(parsed, dict):
                    raw_imgs = parsed.get("images")
                    if isinstance(raw_imgs, list):
                        images = [str(item) for item in raw_imgs if isinstance(item, str)]
                    l0 = parsed.get("memory_l0")
                    if isinstance(l0, dict):
                        has_memory_l0 = True
                        raw_rev = l0.get("rev")
                        if isinstance(raw_rev, int) and not isinstance(raw_rev, bool):
                            l0_rev = raw_rev
            if not images and image_count > 0:
                images = [f"image_{i}" for i in range(image_count)]
        self.last_request_modalities = {
            "images": images,
            "has_memory_l0": has_memory_l0,
            "l0_rev": l0_rev,
        }

    def build_request_body(self, messages: list[dict[str, Any]]) -> dict[str, Any]:
        """Build the OpenAI-compatible /chat/completions JSON request payload."""
        body: dict[str, Any] = {
            "model": self.config.model,
            "messages": messages,
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "stream": self.config.stream,
        }
        if self.config.provider == "deepseek":
            body["thinking"] = {"type": "disabled"}
        elif self.config.provider in ("qwen", "qwen3.8-flash"):
            body["enable_thinking"] = False
        if self.config.extra_body:
            body.update(self.config.extra_body)
        return body

    def _remaining_timeout(
        self, deadline: float, cancel_token: _RequestCancelToken | None
    ) -> float:
        if cancel_token is not None and cancel_token.is_cancelled:
            raise _RequestCancelledError("request_cancelled")
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise _DeadlineExceededError("request_deadline_exceeded")
        return max(0.005, remaining)

    def _apply_sock_timeout(
        self,
        conn: http.client.HTTPConnection,
        deadline: float,
        cancel_token: _RequestCancelToken | None,
    ) -> None:
        timeout = self._remaining_timeout(deadline, cancel_token)
        sock = conn.sock
        if sock is not None:
            sock.settimeout(timeout)

    def _read_sse_verdict(
        self,
        conn: http.client.HTTPConnection,
        resp: http.client.HTTPResponse,
        *,
        deadline: float,
        cancel_token: _RequestCancelToken | None,
        t0: float,
        valid_candidate_ids: Iterable[str] | None,
        frame_seq: int | None,
        action_epoch: int | None,
    ) -> SentinelVerdict:
        content_chunks: list[str] = []
        accumulated = ""
        while True:
            self._apply_sock_timeout(conn, deadline, cancel_token)
            raw_line = resp.readline()
            if not raw_line:
                break
            if cancel_token is not None and cancel_token.is_cancelled:
                raise _RequestCancelledError("request_cancelled")
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line or line.startswith(":") or not line.startswith("data:"):
                continue
            data_str = line[5:].strip()
            if data_str == "[DONE]":
                break
            try:
                chunk_obj = json.loads(data_str)
            except ValueError:
                continue
            if not isinstance(chunk_obj, dict):
                continue
            delta_text = _extract_sse_delta_content(chunk_obj)
            if not delta_text:
                continue
            content_chunks.append(delta_text)
            accumulated = "".join(content_chunks)
            if _extract_json_object(accumulated) is not None:
                latency_ms = (time.perf_counter() - t0) * 1000.0
                return parse_sentinel_verdict(
                    accumulated,
                    latency_ms=latency_ms,
                    valid_candidate_ids=valid_candidate_ids,
                    frame_seq=frame_seq,
                    action_epoch=action_epoch,
                )

        latency_ms = (time.perf_counter() - t0) * 1000.0
        return parse_sentinel_verdict(
            accumulated,
            latency_ms=latency_ms,
            valid_candidate_ids=valid_candidate_ids,
            frame_seq=frame_seq,
            action_epoch=action_epoch,
        )

    def _read_json_verdict(
        self,
        conn: http.client.HTTPConnection,
        resp: http.client.HTTPResponse,
        *,
        deadline: float,
        cancel_token: _RequestCancelToken | None,
        t0: float,
        valid_candidate_ids: Iterable[str] | None,
        frame_seq: int | None,
        action_epoch: int | None,
    ) -> SentinelVerdict:
        chunks: list[bytes] = []
        while True:
            self._apply_sock_timeout(conn, deadline, cancel_token)
            piece = resp.read1(8192)
            if not piece:
                break
            chunks.append(piece)
        if cancel_token is not None and cancel_token.is_cancelled:
            raise _RequestCancelledError("request_cancelled")
        raw = b"".join(chunks).decode("utf-8")
        latency_ms = (time.perf_counter() - t0) * 1000.0
        parsed = json.loads(raw)
        content = _extract_choice_content(parsed)
        return parse_sentinel_verdict(
            content,
            latency_ms=latency_ms,
            valid_candidate_ids=valid_candidate_ids,
            frame_seq=frame_seq,
            action_epoch=action_epoch,
        )

    def evaluate_messages(
        self,
        messages: list[dict[str, Any]],
        *,
        valid_candidate_ids: Iterable[str] | None = None,
        frame_seq: int | None = None,
        action_epoch: int | None = None,
        deadline_monotonic: float | None = None,
        cancel_token: _RequestCancelToken | None = None,
    ) -> SentinelVerdict:
        """Send pre-built messages to /chat/completions; fails open to CONT on error."""
        self._record_modalities_from_messages(messages)
        body = json.dumps(
            self.build_request_body(messages), allow_nan=False
        ).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Accept": "text/event-stream, application/json",
            "User-Agent": self.config.user_agent,
        }
        if self.config.api_key:
            headers["Authorization"] = f"Bearer {self.config.api_key}"
        if self.config.session_id:
            headers["x-opencode-session"] = self.config.session_id

        parsed_url = urllib.parse.urlparse(self.config.base_url)
        base_path = parsed_url.path.rstrip("/")
        req_path = f"{base_path}/chat/completions"

        t0 = time.perf_counter()
        deadline = (
            deadline_monotonic
            if deadline_monotonic is not None
            else (time.monotonic() + self.config.timeout_s)
        )
        conn: http.client.HTTPConnection | None = None
        try:
            init_timeout = self._remaining_timeout(deadline, cancel_token)
            if parsed_url.scheme == "https":
                conn = http.client.HTTPSConnection(
                    parsed_url.hostname or "",
                    parsed_url.port,
                    timeout=init_timeout,
                )
            else:
                conn = http.client.HTTPConnection(
                    parsed_url.hostname or "",
                    parsed_url.port,
                    timeout=init_timeout,
                )
            if cancel_token is not None and not cancel_token.attach(conn):
                raise _RequestCancelledError("request_cancelled")

            conn.request("POST", req_path, body=body, headers=headers)
            self._apply_sock_timeout(conn, deadline, cancel_token)
            resp = conn.getresponse()
            if resp.status < 200 or resp.status >= 300:
                latency_ms = (time.perf_counter() - t0) * 1000.0
                return SentinelVerdict(
                    act="CONT",
                    target_id=None,
                    reason=f"client_error:HTTP_{resp.status}",
                    latency_ms=latency_ms,
                    fallback=True,
                    frame_seq=frame_seq,
                    action_epoch=action_epoch,
                )
            content_type = (resp.getheader("Content-Type") or "").lower()
            if "text/event-stream" in content_type:
                return self._read_sse_verdict(
                    conn,
                    resp,
                    deadline=deadline,
                    cancel_token=cancel_token,
                    t0=t0,
                    valid_candidate_ids=valid_candidate_ids,
                    frame_seq=frame_seq,
                    action_epoch=action_epoch,
                )
            return self._read_json_verdict(
                conn,
                resp,
                deadline=deadline,
                cancel_token=cancel_token,
                t0=t0,
                valid_candidate_ids=valid_candidate_ids,
                frame_seq=frame_seq,
                action_epoch=action_epoch,
            )
        except _RequestCancelledError:
            latency_ms = (time.perf_counter() - t0) * 1000.0
            return SentinelVerdict(
                act="CONT",
                target_id=None,
                reason="cancelled",
                latency_ms=latency_ms,
                fallback=True,
                frame_seq=frame_seq,
                action_epoch=action_epoch,
            )
        except Exception as exc:
            latency_ms = (time.perf_counter() - t0) * 1000.0
            if cancel_token is not None and cancel_token.is_cancelled:
                reason = "cancelled"
            elif _is_timeout_error(exc) or isinstance(exc, _DeadlineExceededError):
                reason = f"timeout:{type(exc).__name__}"
            else:
                reason = f"client_error:{type(exc).__name__}"
            return SentinelVerdict(
                act="CONT",
                target_id=None,
                reason=reason,
                latency_ms=latency_ms,
                fallback=True,
                frame_seq=frame_seq,
                action_epoch=action_epoch,
            )
        finally:
            if cancel_token is not None:
                cancel_token.detach()
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass

    def evaluate(
        self,
        observation_summary: dict[str, Any] | None,
        candidates: Iterable[dict[str, Any]] | None,
        backend_health: dict[str, Any] | None,
        *,
        rgb_path: str | Path | None = None,
        local_map_path: str | Path | None = None,
        frame_seq: int | None = None,
        action_epoch: int | None = None,
        deadline_monotonic: float | None = None,
        cancel_token: _RequestCancelToken | None = None,
    ) -> SentinelVerdict:
        """Build a visual-first prompt respecting provider vision support and evaluate."""
        candidate_list = list(candidates) if candidates is not None else None
        candidate_ids = (
            _candidate_ids_from(candidate_list) if candidate_list is not None else None
        )
        messages = build_visual_first_messages(
            observation_summary,
            candidate_list,
            backend_health,
            rgb_path=rgb_path,
            local_map_path=local_map_path,
            supports_vision=bool(self.config.supports_vision),
            allowed_image_roots=self.allowed_image_roots,
        )
        return self.evaluate_messages(
            messages,
            valid_candidate_ids=candidate_ids,
            frame_seq=frame_seq,
            action_epoch=action_epoch,
            deadline_monotonic=deadline_monotonic,
            cancel_token=cancel_token,
        )


def _percentile(sorted_vals: list[float], quantile: float) -> float:
    if not sorted_vals:
        return 0.0
    idx = min(
        len(sorted_vals) - 1,
        max(0, int(math.ceil(quantile * len(sorted_vals)) - 1)),
    )
    return round(sorted_vals[idx], 3)


@dataclass(frozen=True)
class _QueuedDecisionJob:
    payload: Any
    frame_seq: int
    action_epoch: int | None
    candidate_ids: tuple[str, ...] | None
    deadline_monotonic: float | None


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
        self._next_frame_seq = 0
        self._latest_submitted_frame_seq = 0
        self._latest_submitted_action_epoch: int | None = None
        self._active_cancel: _RequestCancelToken | None = None
        self._thread = threading.Thread(
            target=self._worker_loop,
            name="sentinel-async-runner",
            daemon=True,
        )
        self._thread.start()

    def _run_one(
        self, job: _QueuedDecisionJob, cancel_token: _RequestCancelToken
    ) -> SentinelVerdict:
        payload = job.payload
        t0 = time.perf_counter()
        try:
            if isinstance(self._evaluator, FastSentinelClient):
                if isinstance(payload, list):
                    return self._evaluator.evaluate_messages(
                        payload,
                        valid_candidate_ids=job.candidate_ids,
                        frame_seq=job.frame_seq,
                        action_epoch=job.action_epoch,
                        deadline_monotonic=job.deadline_monotonic,
                        cancel_token=cancel_token,
                    )
                if isinstance(payload, dict) and "messages" in payload:
                    return self._evaluator.evaluate_messages(
                        payload["messages"],
                        valid_candidate_ids=job.candidate_ids,
                        frame_seq=job.frame_seq,
                        action_epoch=job.action_epoch,
                        deadline_monotonic=job.deadline_monotonic,
                        cancel_token=cancel_token,
                    )
                if isinstance(payload, dict):
                    obs = payload.get("observation_summary")
                    if obs is None:
                        obs = payload.get("observation")
                    return self._evaluator.evaluate(
                        obs,
                        payload.get("candidates"),
                        payload.get("backend_health"),
                        rgb_path=payload.get("rgb_path"),
                        local_map_path=payload.get("local_map_path"),
                        frame_seq=job.frame_seq,
                        action_epoch=job.action_epoch,
                        deadline_monotonic=job.deadline_monotonic,
                        cancel_token=cancel_token,
                    )
                raise TypeError("unsupported sentinel payload type")
            result = self._evaluator(payload)
            if not isinstance(result, SentinelVerdict):
                raise TypeError("evaluator must return SentinelVerdict")
            if result.frame_seq is None or result.action_epoch is None:
                return SentinelVerdict(
                    act=result.act,
                    target_id=result.target_id,
                    reason=result.reason,
                    latency_ms=result.latency_ms,
                    fallback=result.fallback,
                    frame_seq=job.frame_seq if result.frame_seq is None else result.frame_seq,
                    action_epoch=(
                        job.action_epoch
                        if result.action_epoch is None
                        else result.action_epoch
                    ),
                )
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
                frame_seq=job.frame_seq,
                action_epoch=job.action_epoch,
            )

    def _worker_loop(self) -> None:
        while True:
            with self._cond:
                while self._pending_slot is _EMPTY and not self._closed:
                    self._cond.wait()
                if self._closed:
                    return
                job: _QueuedDecisionJob = self._pending_slot
                self._pending_slot = _EMPTY
                cancel_token = _RequestCancelToken()
                self._active_cancel = cancel_token

            verdict = self._run_one(job, cancel_token)
            with self._cond:
                self._active_cancel = None
                self._calls += 1
                if verdict.fallback and "timeout" in verdict.reason.lower():
                    self._timeouts += 1
                self._latencies_ms.append(float(verdict.latency_ms))
                if self._closed:
                    return
                stale_frame = job.frame_seq < self._latest_submitted_frame_seq
                stale_epoch = (
                    job.action_epoch is not None
                    and self._latest_submitted_action_epoch is not None
                    and job.action_epoch < self._latest_submitted_action_epoch
                )
                if not stale_frame and not stale_epoch:
                    self._latest_verdict = verdict

    def submit(
        self,
        payload: Any,
        *,
        frame_seq: int | None = None,
        action_epoch: int | None = None,
        candidate_ids: Iterable[str] | None = None,
    ) -> bool:
        """Overwrite the 1-slot mailbox with the newest frame/epoch payload without blocking."""
        resolved_candidates: tuple[str, ...] | None = None
        if candidate_ids is not None:
            resolved_candidates = tuple(
                cid.strip()
                for cid in candidate_ids
                if isinstance(cid, str) and cid.strip()
            )
        elif isinstance(payload, dict) and payload.get("candidates") is not None:
            resolved_candidates = tuple(_candidate_ids_from(payload.get("candidates")))

        deadline = (
            time.monotonic() + self._evaluator.config.timeout_s
            if isinstance(self._evaluator, FastSentinelClient)
            else None
        )

        with self._cond:
            if self._closed:
                return False
            if frame_seq is None:
                self._next_frame_seq += 1
                resolved_seq = self._next_frame_seq
            else:
                if isinstance(frame_seq, bool) or not isinstance(frame_seq, int) or frame_seq <= 0:
                    raise ValueError("frame_seq must be a positive integer")
                if frame_seq < self._latest_submitted_frame_seq:
                    return False
                resolved_seq = frame_seq
                self._next_frame_seq = max(self._next_frame_seq, resolved_seq)

            if action_epoch is not None:
                if (
                    isinstance(action_epoch, bool)
                    or not isinstance(action_epoch, int)
                    or action_epoch < 0
                ):
                    raise ValueError("action_epoch must be a non-negative integer")
                if (
                    self._latest_submitted_action_epoch is not None
                    and action_epoch < self._latest_submitted_action_epoch
                ):
                    return False
                self._latest_submitted_action_epoch = action_epoch

            self._latest_submitted_frame_seq = max(
                self._latest_submitted_frame_seq, resolved_seq
            )
            # Never expose a stale verdict once a newer frame or epoch is submitted.
            self._latest_verdict = None
            if self._pending_slot is not _EMPTY:
                self._dropped_stale += 1
            self._pending_slot = _QueuedDecisionJob(
                payload=payload,
                frame_seq=resolved_seq,
                action_epoch=action_epoch,
                candidate_ids=resolved_candidates,
                deadline_monotonic=deadline,
            )
            self._cond.notify()
            return True

    def poll_latest(self) -> SentinelVerdict | None:
        """Consume and return the latest fresh completed verdict without blocking, or None."""
        with self._lock:
            if self._closed:
                self._latest_verdict = None
                return None
            verdict = self._latest_verdict
            self._latest_verdict = None
            if verdict is None:
                return None
            if (
                verdict.frame_seq is not None
                and verdict.frame_seq < self._latest_submitted_frame_seq
            ):
                return None
            if (
                verdict.action_epoch is not None
                and self._latest_submitted_action_epoch is not None
                and verdict.action_epoch < self._latest_submitted_action_epoch
            ):
                return None
            return verdict

    def stats(self) -> dict[str, Any]:
        """Return latency percentiles and call/timeout/drop counters."""
        with self._lock:
            sorted_lat = sorted(self._latencies_ms)
            result: dict[str, Any] = {
                "calls": self._calls,
                "timeouts": self._timeouts,
                "dropped_stale": self._dropped_stale,
                "p50_ms": _percentile(sorted_lat, 0.50),
                "p95_ms": _percentile(sorted_lat, 0.95),
            }
            if isinstance(self._evaluator, FastSentinelClient):
                result["last_request_modalities"] = dict(
                    self._evaluator.last_request_modalities
                )
            return result

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

    def shutdown(self, timeout_s: float = 2.0) -> bool:
        """Abort any in-flight request, stop the worker, and report whether it exited."""
        with self._cond:
            self._closed = True
            self._pending_slot = _EMPTY
            self._latest_verdict = None
            active_cancel = self._active_cancel
            self._cond.notify_all()
        if active_cancel is not None:
            active_cancel.cancel()
        self._thread.join(timeout=timeout_s)
        return not self._thread.is_alive()
