"""Shared stdlib-only LLM client configuration, credential redaction, and safe image encoding.

Lives in ``common/`` so both the top-level ``agent/`` orchestration layer and legacy
``planning/exploration/`` clients share a single source of truth without cross-layer cycles.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable
import urllib.parse
import uuid

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


def encode_image_part(
    image_path: str | Path,
    *,
    allowed_roots: Iterable[str | Path] | None = None,
    max_bytes: int = MAX_IMAGE_UPLOAD_BYTES,
) -> dict[str, Any] | None:
    """Encode a validated local PNG/JPEG/WEBP file into an OpenAI vision ``image_url`` part."""
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


_encode_image_part = encode_image_part

__all__ = [
    "LlmClientConfig",
    "MAX_IMAGE_UPLOAD_BYTES",
    "PROVIDER_PRESETS",
    "_detect_image_mime",
    "_encode_image_part",
    "_finite_float",
    "_parse_kv_line",
    "_read_key_file",
    "_read_opencode_auth_key",
    "_read_validated_image_bytes",
    "_redact_sensitive",
    "_redact_url",
    "_resolve_preset_api_key",
    "_strip_matching_quotes",
    "encode_image_part",
    "validate_image_path",
]
