"""OpenAI-compatible streaming multi-turn tool & reasoning transport for EmbodiedAgentRuntime."""

from __future__ import annotations

import http.client
import json
import socket
import time
from typing import Any, Sequence
import urllib.parse

from embodied_navigation.agent_runtime.types import (
    AssistantTurn,
    ToolCall,
    ToolSpec,
    TurnUsage,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig

_MAX_RESPONSE_BYTES = 8 * 1024 * 1024
_READ_CHUNK_BYTES = 16384
_VALID_FINISH_REASONS = frozenset({"stop", "tool_calls", "length", "end_turn", "function_call"})


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
            max(120.0, float(base_timeout_s)),
            max(16384, int(base_max_tokens)),
        )
    if "qwen" in m:
        if norm == "disabled":
            return (
                {"enable_thinking": False},
                max(45.0, float(base_timeout_s)),
                max(4096, int(base_max_tokens)),
            )
        return (
            {"enable_thinking": True},
            max(90.0, float(base_timeout_s)),
            max(16384, int(base_max_tokens)),
        )
    return (None, max(45.0, float(base_timeout_s)), max(4096, int(base_max_tokens)))


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


class OpenCodeToolChatClient:
    """SSE streaming OpenAI-compatible client supporting native ``tools`` & reasoning traces."""

    def __init__(self, config: LlmClientConfig) -> None:
        self.config = config

    def complete_turn(
        self,
        messages: list[dict[str, Any]],
        tools: Sequence[ToolSpec],
    ) -> AssistantTurn:
        """Execute one streaming multi-turn completion and capture full usage + reasoning."""
        payload: dict[str, Any] = {
            "model": self.config.model,
            "messages": messages,
            "tools": [t.to_openai_tool() for t in tools],
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "stream": self.config.stream,
        }
        if self.config.stream:
            payload["stream_options"] = {"include_usage": True}
        if self.config.extra_body:
            payload.update(self.config.extra_body)

        body = json.dumps(payload, allow_nan=False, ensure_ascii=False).encode("utf-8")
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
        t0 = time.perf_counter()
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
            if time.monotonic() > deadline:
                raise TimeoutError("request_deadline_exceeded")
            if resp.status >= 400:
                raise RuntimeError(f"http_{resp.status}")
            ctype = (resp.getheader("Content-Type") or "").lower()

            if not self.config.stream or "text/event-stream" not in ctype:
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
                return self._parse_non_stream_response(data, latency_ms=dt_ms)

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
                        raise RuntimeError("sse_upstream_error")
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
            clean_termination = saw_done or (finish_reason in _VALID_FINISH_REASONS)
            if not clean_termination or (stream_err and not tc_accum and not content_parts):
                return AssistantTurn(
                    content="",
                    reasoning_content="",
                    tool_calls=(),
                    usage=_extract_usage(usage_obj, latency_ms=dt_ms),
                    fallback=True,
                    error_reason="truncated_sse_stream",
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
            )
        finally:
            conn.close()

    @staticmethod
    def _parse_non_stream_response(
        data: dict[str, Any], *, latency_ms: float
    ) -> AssistantTurn:
        choices = data.get("choices") or []
        first = choices[0] if choices and isinstance(choices[0], dict) else {}
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
        )
