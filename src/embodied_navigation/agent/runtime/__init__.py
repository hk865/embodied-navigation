"""Standalone Layer-7 EmbodiedAgentRuntime package (multi-turn, KV-cache friendly, task-agnostic)."""

from embodied_navigation.agent.runtime.context import PrefixStableContextManager
from embodied_navigation.agent.runtime.hooks import (
    AfterToolHook,
    BeforeFinishHook,
    BeforeTurnHook,
    default_after_tool_hook,
    strip_host_paths_recursive,
)
from embodied_navigation.agent.runtime.llm_transport import (
    OpenCodeToolChatClient,
    resolve_agent_thinking,
)
from embodied_navigation.agent.runtime.runtime import (
    AgentRunResult,
    EmbodiedAgentRuntime,
)
from embodied_navigation.agent.runtime.types import (
    AssistantTurn,
    FinishCheckResult,
    ImageAttachment,
    ToolCall,
    ToolResult,
    ToolSpec,
    TurnUsage,
)

__all__ = [
    "AfterToolHook",
    "AgentRunResult",
    "AssistantTurn",
    "BeforeFinishHook",
    "BeforeTurnHook",
    "EmbodiedAgentRuntime",
    "FinishCheckResult",
    "ImageAttachment",
    "OpenCodeToolChatClient",
    "PrefixStableContextManager",
    "ToolCall",
    "ToolResult",
    "ToolSpec",
    "TurnUsage",
    "default_after_tool_hook",
    "resolve_agent_thinking",
    "strip_host_paths_recursive",
]
