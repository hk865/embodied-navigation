"""Lifecycle hook interfaces and default safety hooks for EmbodiedAgentRuntime.

Agent hooks operate strictly on the cognitive/tool lifecycle (tail state delta
injection, tool payload sanitation/bounding, and task-level finish verification).
Perception/SLAM/memory updates remain asynchronous inside the world daemon.
"""

from __future__ import annotations

import json
from typing import Any, Callable, Iterable, Protocol

from embodied_navigation.agent_runtime.types import (
    FinishCheckResult,
    ToolCall,
    ToolResult,
)


class BeforeTurnHook(Protocol):
    """Produces an append-only tail status delta string right before an observation."""

    def __call__(self, turn_index: int) -> str: ...


class AfterToolHook(Protocol):
    """Inspects, redacts, or bounds a tool result before entering conversation context."""

    def __call__(self, call: ToolCall, result: ToolResult) -> ToolResult: ...


class BeforeFinishHook(Protocol):
    """Validates whether ``finish_task`` satisfies the active task predicate."""

    def __call__(self, call: ToolCall) -> FinishCheckResult: ...


def strip_host_paths_recursive(
    value: Any, *, forbidden_keys: Iterable[str] = ("rgb_path", "local_map_path", "file_path")
) -> Any:
    """Recursively remove host filesystem path keys before tool payloads reach the LLM."""
    bad = set(forbidden_keys)
    if isinstance(value, dict):
        return {
            k: strip_host_paths_recursive(v, forbidden_keys=bad)
            for k, v in value.items()
            if str(k) not in bad
        }
    if isinstance(value, list):
        return [strip_host_paths_recursive(v, forbidden_keys=bad) for v in value]
    if isinstance(value, tuple):
        return [strip_host_paths_recursive(v, forbidden_keys=bad) for v in value]
    return value


def default_after_tool_hook(
    max_payload_chars: int = 6000,
) -> Callable[[ToolCall, ToolResult], ToolResult]:
    """Return an ``AfterToolHook`` that strips host paths and caps serialized payload size."""
    cap = max(512, int(max_payload_chars))

    def _hook(call: ToolCall, result: ToolResult) -> ToolResult:
        _ = call
        cleaned = strip_host_paths_recursive(result.payload)
        encoded = json.dumps(cleaned, ensure_ascii=False, separators=(",", ":"))
        if len(encoded) > cap:
            summary_keys = (
                list(cleaned.keys())[:10] if isinstance(cleaned, dict) else []
            )
            cleaned = {
                "truncated": True,
                "original_chars": len(encoded),
                "keys": summary_keys,
                "preview": encoded[: max(256, cap - 160)],
            }
        return ToolResult(
            call_id=result.call_id,
            name=result.name,
            payload=cleaned if isinstance(cleaned, dict) else {"value": cleaned},
            images=result.images,
            is_error=result.is_error,
        )

    return _hook
