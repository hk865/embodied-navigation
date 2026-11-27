"""Core value types for the standalone Layer-7 EmbodiedAgentRuntime."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ToolSpec:
    """JSON-schema definition of one agent-callable tool."""

    name: str
    description: str
    parameters: dict[str, Any]

    def to_openai_tool(self) -> dict[str, Any]:
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.parameters,
            },
        }


@dataclass(frozen=True)
class ToolCall:
    """Normalized tool invocation emitted by the LLM planner."""

    call_id: str
    name: str
    arguments: dict[str, Any]
    raw_arguments: str = ""


@dataclass(frozen=True)
class ImageAttachment:
    """Validated local image path with a short semantic label for the multimodal turn."""

    path: Path
    label: str = ""


@dataclass(frozen=True)
class ToolResult:
    """Result returned by a tool handler (plus optional fresh image observations)."""

    call_id: str
    name: str
    payload: dict[str, Any]
    images: tuple[ImageAttachment, ...] = ()
    is_error: bool = False


@dataclass(frozen=True)
class TurnUsage:
    """Token and latency accounting reported by the model gateway for one turn."""

    prompt_tokens: int = 0
    completion_tokens: int = 0
    reasoning_tokens: int = 0
    cached_tokens: int = 0
    latency_ms: float = 0.0


@dataclass(frozen=True)
class AssistantTurn:
    """Parsed multi-turn response containing optional reasoning, text, and tool calls."""

    content: str = ""
    reasoning_content: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    usage: TurnUsage = field(default_factory=TurnUsage)
    fallback: bool = False
    error_reason: str = ""
    provider_used: str = ""
    model_used: str = ""
    fallback_switched_from: str = ""


@dataclass(frozen=True)
class FinishCheckResult:
    """Decision returned by a pluggable task completion verifier hook."""

    allowed: bool
    reason: str
    metadata: dict[str, Any] = field(default_factory=dict)
