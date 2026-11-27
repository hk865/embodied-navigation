"""OpenAI-compatible streaming multi-turn tool & reasoning transport for EmbodiedAgentRuntime."""

from __future__ import annotations

import http.client
import json
import socket
import threading
import time
from typing import Any, Sequence
import urllib.parse

from embodied_navigation.agent.runtime.types import (
    AssistantTurn,
    ToolCall,
    ToolSpec,
    TurnUsage,
)
from embodied_navigation.common.llm_config import LlmClientConfig, PROVIDER_PRESETS

_MAX_RESPONSE_BYTES = 8 * 1024 * 1024
_READ_CHUNK_BYTES = 16384
_VALID_FINISH_REASONS = frozenset({"stop", "tool_calls", "length", "end_turn", "function_call"})
_REFUSAL_FINISH_REASONS = frozenset({"content_filter", "refusal"})
_RESERVED_PAYLOAD_KEYS = frozenset({"model", "messages", "tools", "stream"})


def resolve_agent_thinking(
    model_id: str,
    mode: str,
    *,
    base_timeout_s: float = 60.0,
    base_max_tokens: int = 16384,
) -> tuple[dict[str, Any] | None, float, int]:
    """Return ``(extra_body, timeout_s, max_tokens)`` for multi-turn tool-calling models.

    Unlike single-shot JSON mode, ``deepseek-v4.1-flash`` streams native
    ``reasoning_content`` cleanly alongside ``tool_calls`` when passed
    ``{"thinking": {"type": "enabled"}}``, achieving high prefix KV-cache reuse
    when ``reasoning_content`` is echoed back on assistant tool-call turns.
    """
    m = str(model_id or "").lower()
    norm = str(mode or "default").strip().lower()
    if "deepseek" in m:
        if norm == "disabled":
            return (
                {"thinking": {"type": "disabled"}},
                max(45.0, float(base_timeout_s)),
                max(4096, int(base_max_tokens)),
            )
        return (
            {"thinking": {"type": "enabled"}},
            max(180.0, float(base_timeout_s)),
            max(16384, int(base_max_tokens)),
        )
    if "qwen" in m:
        if norm == "disabled":
            return (
                {"enable_thinking": False},
                max(180.0, float(base_timeout_s)),
                max(4096, int(base_max_tokens)),
            )
        return (
            {"enable_thinking": True},
            max(180.0, float(base_timeout_s)),
            max(16384, int(base_max_tokens)),
        )
    return (None, max(45.0, float(base_timeout_s)), max(4096, int(base_max_tokens)))


def build_default_deepseek_fallback(
    primary_cfg: LlmClientConfig,
    *,
    thinking_mode: str = "default",
    model: str = "deepseek-flash",
    timeout_s: float | None = None,
    supports_vision: bool | None = None,
) -> LlmClientConfig | None:
    """Build a validated DSH / DeepSeek official fallback route when credentials exist.

    Used to fail over seamlessly when the OpenCode Go gateway times out or returns 5xx/429
    on ``deepseek-v4.1-flash``. Returns ``None`` if no DeepSeek API key is configured.
    """
    extra_body, resolved_to, max_toks = resolve_agent_thinking(
        model,
        thinking_mode,
        base_timeout_s=(timeout_s if timeout_s is not None else primary_cfg.timeout_s),
        base_max_tokens=primary_cfg.max_tokens,
    )
    if supports_vision is None:
        m_low = str(model or "").strip().lower()
        preset_vis = bool(PROVIDER_PRESETS["deepseek_official"].get("supports_vision", False))
        resolved_vis = (
            False
            if any(tok in m_low for tok in ("deepseek-chat", "deepseek-reasoner"))
            else preset_vis
        )
    else:
        resolved_vis = bool(supports_vision)
    cfg = LlmClientConfig(
        provider="deepseek_official",
        model=model,
        timeout_s=resolved_to,
        max_tokens=max_toks,
        temperature=primary_cfg.temperature,
        supports_vision=resolved_vis,
        stream=primary_cfg.stream,
        extra_body=extra_body,
    )
    if not cfg.api_key:
        return None
    return cfg


def _is_retryable_failover_error(error_reason: str) -> bool:
    """True for gateway timeouts, 429, any 5xx, stream truncation/corruption, and network errors."""
    err = str(error_reason or "").lower()
    if not err:
        return False
    if any(
        non_retry in err
        for non_retry in (
            "http_400",
            "http_401",
            "http_403",
            "http_404",
            "http_422",
            "content_filter",
            "refusal",
            "invalid_request",
            "context_length",
        )
    ):
        return False
    # Match any HTTP 5xx status code (http_500..http_599)
    import re

    if re.search(r"http_5\d\d", err):
        return True
    return any(
        tok in err
        for tok in (
            "timeout",
            "request_deadline_exceeded",
            "truncated_sse_stream",
            "corrupted_sse_stream",
            "sse_upstream_error",
            "http_429",
            "connection",
            "remotedisconnected",
            "brokenpipe",
            "eof",
            "oserror",
            "gaierror",
        )
    )


def _classify_http_error(status: int, body_bytes: bytes) -> str:
    """Produce a bounded, sanitized error code from an HTTP >= 400 status & body."""
    snippet = body_bytes[:400].decode("utf-8", errors="replace").lower()
    if status == 400:
        if any(
            kw in snippet
            for kw in ("context_length", "maximum context", "too long", "token limit", "max_tokens")
        ):
            return "http_400:context_length"
        if any(kw in snippet for kw in ("content_filter", "policy", "safety", "refusal")):
            return "http_400:content_filter"
        return "http_400:invalid_request"
    return f"http_{int(status)}"


def _classify_sse_error(err_obj: dict[str, Any]) -> str:
    """Classify an inline SSE ``error`` JSON object into retryable vs non-retryable codes."""
    raw = json.dumps(err_obj, ensure_ascii=False)[:300].lower()
    if any(kw in raw for kw in ("content_filter", "policy", "safety", "refusal")):
        return "content_filter_refusal"
    if any(kw in raw for kw in ("context_length", "maximum context", "too long")):
        return "http_400:context_length"
    if any(kw in raw for kw in ("invalid_request", "authentication", "unauthorized", "invalid_api_key")):
        return "http_401:invalid_request"
    return "sse_upstream_error"


def _strip_images_for_text_only(messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Strip ``image_url`` blocks when a fallback route declares ``supports_vision=False``."""
    out: list[dict[str, Any]] = []
    for m in messages:
        content = m.get("content")
        if isinstance(content, list):
            text_parts: list[str] = []
            img_count = 0
            for part in content:
                if isinstance(part, dict) and part.get("type") == "image_url":
                    img_count += 1
                elif isinstance(part, dict) and isinstance(part.get("text"), str):
                    text_parts.append(part["text"])
            if img_count > 0:
                text_parts.append(f"[{img_count} image(s) omitted on text-only fallback route]")
            new_m = dict(m)
            new_m["content"] = "\n".join(text_parts)
            out.append(new_m)
        else:
            out.append(m)
    return out


def _parse_tool_arguments(raw_args: str) -> dict[str, Any]:
    text = str(raw_args or "").strip()
    if not text:
        return {}
    try:
        parsed = json.loads(text)
    except ValueError:
        return {"_parse_error": True, "_raw": text[:400]}
    if not isinstance(parsed, dict):
        return {"_parse_error": True, "_raw": text[:400]}
    return parsed


def _extract_usage(usage_obj: Any, *, latency_ms: float) -> TurnUsage:
    if not isinstance(usage_obj, dict):
        return TurnUsage(latency_ms=latency_ms)
    prompt_tok = int(usage_obj.get("prompt_tokens") or 0)
    comp_tok = int(usage_obj.get("completion_tokens") or 0)
    pt_details = usage_obj.get("prompt_tokens_details") or {}
    ct_details = usage_obj.get("completion_tokens_details") or {}
    oc_tokens = (usage_obj.get("opencode") or {}).get("tokens") or {}
    cached = int(
        pt_details.get("cached_tokens")
        or usage_obj.get("prompt_cache_hit_tokens")
        or oc_tokens.get("cache_read")
        or 0
    )
    reasoning_tok = int(
        ct_details.get("reasoning_tokens")
        or oc_tokens.get("reasoning")
        or 0
    )
    return TurnUsage(
        prompt_tokens=prompt_tok,
        completion_tokens=comp_tok,
        reasoning_tokens=reasoning_tok,
        cached_tokens=cached,
        latency_ms=latency_ms,
    )


def _read_chunk_with_deadline(resp: http.client.HTTPResponse, conn: Any, deadline: float) -> bytes:
    rem = deadline - time.monotonic()
    if rem <= 0.0:
        raise TimeoutError("request_deadline_exceeded")
    if conn.sock is not None:
        conn.sock.settimeout(rem)
    try:
        chunk = resp.read1(_READ_CHUNK_BYTES)
    except (TimeoutError, socket.timeout) as exc:
        raise TimeoutError("request_deadline_exceeded") from exc
    if time.monotonic() > deadline:
        raise TimeoutError("request_deadline_exceeded")
    return chunk


class _SharedCircuitState:
    """Thread-safe 3-state (closed / open / half_open) circuit breaker shared across clones."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.state: str = "closed"  # "closed" | "open" | "half_open"
        self.cooldown_until: float = 0.0
        self.probe_in_flight: bool = False

    def acquire_primary_decision(self, now: float) -> tuple[bool, bool]:
        """Return ``(try_primary, is_half_open_probe)`` atomically under lock."""
        with self.lock:
            if self.state == "closed":
                return True, False
            if self.state == "open":
                if now >= self.cooldown_until:
                    self.state = "half_open"
                    self.probe_in_flight = True
                    return True, True
                return False, False
            # state == "half_open": only one probe in flight; concurrent workers use fallback
            if not self.probe_in_flight:
                self.probe_in_flight = True
                return True, True
            return False, False

    def record_primary_success(self) -> None:
        with self.lock:
            self.state = "closed"
            self.cooldown_until = 0.0
            self.probe_in_flight = False

    def record_primary_failure(self, now: float, cooldown_s: float) -> None:
        with self.lock:
            self.probe_in_flight = False
            if cooldown_s > 0.0:
                self.state = "open"
                self.cooldown_until = now + cooldown_s
            else:
                self.state = "closed"
                self.cooldown_until = 0.0


class OpenCodeToolChatClient:
    """SSE streaming OpenAI-compatible client with native ``tools``, reasoning, and failover routing."""

    def __init__(
        self,
        config: LlmClientConfig,
        *,
        fallback_configs: Sequence[LlmClientConfig] = (),
        circuit_open_s: float = 120.0,
        _circuit_state: _SharedCircuitState | None = None,
    ) -> None:
        self.config = config
        self.fallback_configs: tuple[LlmClientConfig, ...] = tuple(fallback_configs)
        self.circuit_open_s = max(0.0, float(circuit_open_s))
        self._circuit = _circuit_state if _circuit_state is not None else _SharedCircuitState()
        self._switch_lock = threading.Lock()
        self.fallback_switches: int = 0
        self.fallback_served_turns: int = 0

    def clone_for_session(self, *, session_id: str | None = None) -> OpenCodeToolChatClient:
        """Create a per-session client sharing circuit-breaker state with isolated session IDs."""
        import uuid

        def _clone_cfg(c: LlmClientConfig, explicit_sid: str | None = None) -> LlmClientConfig:
            needs_sid = bool(PROVIDER_PRESETS.get(c.provider, {}).get("requires_session_header"))
            sid = explicit_sid if (explicit_sid is not None and needs_sid) else (
                str(uuid.uuid4()) if needs_sid else None
            )
            return LlmClientConfig(
                provider=c.provider,
                model=c.model,
                base_url=c.base_url,
                api_key=c.api_key,
                timeout_s=c.timeout_s,
                max_tokens=c.max_tokens,
                temperature=c.temperature,
                supports_vision=c.supports_vision,
                stream=c.stream,
                extra_body=c.extra_body,
                session_id=sid,
                user_agent=c.user_agent,
            )

        return OpenCodeToolChatClient(
            _clone_cfg(self.config, explicit_sid=session_id),
            fallback_configs=[_clone_cfg(fb) for fb in self.fallback_configs],
            circuit_open_s=self.circuit_open_s,
            _circuit_state=self._circuit,
        )

    def complete_turn(
        self,
        messages: list[dict[str, Any]],
        tools: Sequence[ToolSpec],
    ) -> AssistantTurn:
        """Execute one multi-turn completion with unified failover & 3-state circuit breaker."""
        if not self.fallback_configs:
            return self._complete_on_config(self.config, messages, tools)

        t_total = time.perf_counter()
        try_primary, is_probe = self._circuit.acquire_primary_decision(time.monotonic())
        if try_primary:
            attempt_order: list[tuple[bool, LlmClientConfig]] = [(True, self.config)] + [
                (False, fb) for fb in self.fallback_configs
            ]
        else:
            attempt_order = [(False, fb) for fb in self.fallback_configs]

        first_primary_err = ""
        last_turn: AssistantTurn | None = None
        for is_primary, cfg in attempt_order:
            turn = self._complete_on_config(cfg, messages, tools)
            if not turn.fallback:
                if is_primary:
                    self._circuit.record_primary_success()
                    return turn
                # Served by a fallback route
                total_ms = (time.perf_counter() - t_total) * 1000.0
                if try_primary and first_primary_err:
                    switched_note = f"{self.config.provider}:{self.config.model} ({first_primary_err})"
                    with self._switch_lock:
                        self.fallback_switches += 1
                        self.fallback_served_turns += 1
                else:
                    switched_note = f"{self.config.provider}:{self.config.model} (circuit_open)"
                    with self._switch_lock:
                        self.fallback_served_turns += 1
                return AssistantTurn(
                    content=turn.content,
                    reasoning_content=turn.reasoning_content,
                    tool_calls=turn.tool_calls,
                    usage=TurnUsage(
                        prompt_tokens=turn.usage.prompt_tokens,
                        completion_tokens=turn.usage.completion_tokens,
                        reasoning_tokens=turn.usage.reasoning_tokens,
                        cached_tokens=turn.usage.cached_tokens,
                        latency_ms=total_ms,
                    ),
                    fallback=False,
                    error_reason="",
                    provider_used=cfg.provider,
                    model_used=cfg.model or "",
                    fallback_switched_from=switched_note,
                )

            # Route failed
            last_turn = turn
            retryable = _is_retryable_failover_error(turn.error_reason)
            if is_primary:
                first_primary_err = turn.error_reason
                if retryable:
                    self._circuit.record_primary_failure(time.monotonic(), self.circuit_open_s)
                elif is_probe:
                    # Release half-open probe lock on non-retryable client error without opening circuit
                    self._circuit.record_primary_success()
            if not retryable:
                return turn

        return last_turn or AssistantTurn(fallback=True, error_reason="all_routes_failed")

    def _complete_on_config(
        self,
        cfg: LlmClientConfig,
        messages: list[dict[str, Any]],
        tools: Sequence[ToolSpec],
    ) -> AssistantTurn:
        """Execute one streaming multi-turn completion on a specific ``LlmClientConfig``."""
        req_messages = messages if cfg.supports_vision else _strip_images_for_text_only(messages)
        payload: dict[str, Any] = {
            "model": cfg.model,
            "messages": req_messages,
            "tools": [t.to_openai_tool() for t in tools],
            "max_tokens": cfg.max_tokens,
            "temperature": cfg.temperature,
            "stream": cfg.stream,
        }
        if cfg.stream:
            payload["stream_options"] = {"include_usage": True}
        if cfg.extra_body:
            for ek, ev in cfg.extra_body.items():
                if ek not in _RESERVED_PAYLOAD_KEYS:
                    payload[ek] = ev

        body = json.dumps(payload, allow_nan=False, ensure_ascii=False).encode("utf-8")
        headers = {
            "Content-Type": "application/json",
            "Accept": "text/event-stream, application/json",
            "User-Agent": cfg.user_agent,
        }
        if cfg.api_key:
            headers["Authorization"] = f"Bearer {cfg.api_key}"
        if cfg.session_id and PROVIDER_PRESETS.get(cfg.provider, {}).get("requires_session_header"):
            headers["x-opencode-session"] = cfg.session_id

        parsed = urllib.parse.urlparse(cfg.base_url)
        req_path = f"{parsed.path.rstrip('/')}/chat/completions"
        conn_cls = (
            http.client.HTTPSConnection
            if parsed.scheme == "https"
            else http.client.HTTPConnection
        )
        t0 = time.perf_counter()
        deadline = time.monotonic() + cfg.timeout_s
        conn = conn_cls(parsed.hostname, parsed.port, timeout=cfg.timeout_s)
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
            if time.monotonic() > deadline:
                raise TimeoutError("request_deadline_exceeded")
            if resp.status >= 400:
                err_body = b""
                try:
                    err_body = resp.read(1024)
                except Exception:
                    err_body = b""
                raise RuntimeError(_classify_http_error(resp.status, err_body))
            ctype = (resp.getheader("Content-Type") or "").lower()

            if not cfg.stream or "text/event-stream" not in ctype:
                chunks: list[bytes] = []
                total_bytes = 0
                while True:
                    chunk = _read_chunk_with_deadline(resp, conn, deadline)
                    if not chunk:
                        break
                    total_bytes += len(chunk)
                    if total_bytes > _MAX_RESPONSE_BYTES:
                        raise RuntimeError("response_too_large")
                    chunks.append(chunk)
                data = json.loads(b"".join(chunks).decode("utf-8", errors="replace"))
                dt_ms = (time.perf_counter() - t0) * 1000.0
                return self._parse_non_stream_response(
                    data, latency_ms=dt_ms, provider_used=cfg.provider, model_used=cfg.model or ""
                )

            content_parts: list[str] = []
            reasoning_parts: list[str] = []
            tc_accum: dict[int, dict[str, str]] = {}
            usage_obj: dict[str, Any] | None = None
            saw_done = False
            finish_reason: str | None = None
            stream_err = False
            buf = b""
            total_bytes = 0

            while True:
                chunk = _read_chunk_with_deadline(resp, conn, deadline)
                if not chunk:
                    break
                total_bytes += len(chunk)
                if total_bytes > _MAX_RESPONSE_BYTES:
                    raise RuntimeError("response_too_large")
                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line.startswith("data:"):
                        continue
                    data_str = line[5:].strip()
                    if data_str == "[DONE]":
                        saw_done = True
                        break
                    try:
                        chunk_json = json.loads(data_str)
                    except ValueError:
                        stream_err = True
                        continue
                    if isinstance(chunk_json.get("error"), dict):
                        raise RuntimeError(_classify_sse_error(chunk_json["error"]))
                    if isinstance(chunk_json.get("usage"), dict):
                        usage_obj = chunk_json["usage"]
                    choices = chunk_json.get("choices")
                    if not isinstance(choices, list) or not choices:
                        continue
                    first = choices[0]
                    if not isinstance(first, dict):
                        continue
                    fr = first.get("finish_reason")
                    if isinstance(fr, str) and fr.strip():
                        finish_reason = fr.strip()
                    delta = first.get("delta") or first.get("message") or {}
                    if not isinstance(delta, dict):
                        continue
                    c_delta = delta.get("content")
                    if isinstance(c_delta, str) and c_delta:
                        content_parts.append(c_delta)
                    r_delta = delta.get("reasoning_content") or delta.get("reasoning")
                    if isinstance(r_delta, str) and r_delta:
                        reasoning_parts.append(r_delta)
                    raw_tcs = delta.get("tool_calls")
                    if isinstance(raw_tcs, list):
                        for tc in raw_tcs:
                            if not isinstance(tc, dict):
                                continue
                            idx = int(tc.get("index", 0))
                            slot = tc_accum.setdefault(
                                idx, {"id": f"call_{idx}", "name": "", "arguments": ""}
                            )
                            if isinstance(tc.get("id"), str) and tc["id"]:
                                slot["id"] = tc["id"]
                            fn = tc.get("function") or {}
                            if isinstance(fn, dict):
                                if isinstance(fn.get("name"), str) and fn["name"]:
                                    slot["name"] += fn["name"]
                                if isinstance(fn.get("arguments"), str) and fn["arguments"]:
                                    slot["arguments"] += fn["arguments"]
                if saw_done:
                    break

            dt_ms = (time.perf_counter() - t0) * 1000.0
            if finish_reason in _REFUSAL_FINISH_REASONS:
                return AssistantTurn(
                    content="",
                    reasoning_content="",
                    tool_calls=(),
                    usage=_extract_usage(usage_obj, latency_ms=dt_ms),
                    fallback=True,
                    error_reason="content_filter_refusal",
                    provider_used=cfg.provider,
                    model_used=cfg.model or "",
                )
            if stream_err:
                return AssistantTurn(
                    content="",
                    reasoning_content="",
                    tool_calls=(),
                    usage=_extract_usage(usage_obj, latency_ms=dt_ms),
                    fallback=True,
                    error_reason="corrupted_sse_stream",
                    provider_used=cfg.provider,
                    model_used=cfg.model or "",
                )
            clean_termination = saw_done or (finish_reason in _VALID_FINISH_REASONS)
            if not clean_termination:
                return AssistantTurn(
                    content="",
                    reasoning_content="",
                    tool_calls=(),
                    usage=_extract_usage(usage_obj, latency_ms=dt_ms),
                    fallback=True,
                    error_reason="truncated_sse_stream",
                    provider_used=cfg.provider,
                    model_used=cfg.model or "",
                )

            calls: list[ToolCall] = []
            for idx in sorted(tc_accum):
                slot = tc_accum[idx]
                if not slot["name"].strip():
                    continue
                calls.append(
                    ToolCall(
                        call_id=slot["id"],
                        name=slot["name"].strip(),
                        arguments=_parse_tool_arguments(slot["arguments"]),
                        raw_arguments=slot["arguments"],
                    )
                )
            return AssistantTurn(
                content="".join(content_parts),
                reasoning_content="".join(reasoning_parts),
                tool_calls=tuple(calls),
                usage=_extract_usage(usage_obj, latency_ms=dt_ms),
                provider_used=cfg.provider,
                model_used=cfg.model or "",
            )
        except Exception as exc:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            safe_msg = (
                str(exc)
                if isinstance(exc, (RuntimeError, TimeoutError))
                else type(exc).__name__
            )
            return AssistantTurn(
                content="",
                reasoning_content="",
                tool_calls=(),
                usage=TurnUsage(latency_ms=dt_ms),
                fallback=True,
                error_reason=f"transport_error:{type(exc).__name__}:{safe_msg[:64]}",
                provider_used=cfg.provider,
                model_used=cfg.model or "",
            )
        finally:
            conn.close()

    @staticmethod
    def _parse_non_stream_response(
        data: dict[str, Any],
        *,
        latency_ms: float,
        provider_used: str = "",
        model_used: str = "",
    ) -> AssistantTurn:
        choices = data.get("choices") or []
        first = choices[0] if choices and isinstance(choices[0], dict) else {}
        fr = str(first.get("finish_reason") or "").strip().lower() if isinstance(first, dict) else ""
        if fr in _REFUSAL_FINISH_REASONS:
            return AssistantTurn(
                content="",
                reasoning_content="",
                tool_calls=(),
                usage=_extract_usage(data.get("usage"), latency_ms=latency_ms),
                fallback=True,
                error_reason="content_filter_refusal",
                provider_used=provider_used,
                model_used=model_used,
            )
        msg = first.get("message") if isinstance(first, dict) else {}
        if not isinstance(msg, dict):
            msg = {}
        content = str(msg.get("content") or "")
        reasoning = str(msg.get("reasoning_content") or msg.get("reasoning") or "")
        calls: list[ToolCall] = []
        raw_tcs = msg.get("tool_calls")
        if isinstance(raw_tcs, list):
            for idx, tc in enumerate(raw_tcs):
                if not isinstance(tc, dict):
                    continue
                fn = tc.get("function") or {}
                name = str(fn.get("name") or "").strip()
                if not name:
                    continue
                raw_args = str(fn.get("arguments") or "")
                calls.append(
                    ToolCall(
                        call_id=str(tc.get("id") or f"call_{idx}"),
                        name=name,
                        arguments=_parse_tool_arguments(raw_args),
                        raw_arguments=raw_args,
                    )
                )
        return AssistantTurn(
            content=content,
            reasoning_content=reasoning,
            tool_calls=tuple(calls),
            usage=_extract_usage(data.get("usage"), latency_ms=latency_ms),
            provider_used=provider_used,
            model_used=model_used,
        )
