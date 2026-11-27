"""Top-level Embodied Agent Layer (Layer 7).

Orchestrates multi-turn LLM reasoning, prefix-stable context & lazy compaction,
generic embodied tools/verifiers, and pluggable environment adapters while
strictly depending downward on perception, localization_mapping, planning,
control/navigation, robot, and simulation layers.
"""

from embodied_navigation.agent.adapters import (
    HabitatAgentRunner,
    HabitatSessionProxy,
    HabitatWorldDaemon,
    MotionFrameIndex,
    SharedHabitatSceneHost,
    run_concurrent_scene_sessions,
)
from embodied_navigation.agent.prompts import (
    AGENT_RUNTIME_SYSTEM_PROMPT,
    build_vlm_region_summarizer,
)
from embodied_navigation.agent.runtime import (
    AfterToolHook,
    AgentRunResult,
    AssistantTurn,
    BeforeFinishHook,
    BeforeTurnHook,
    EmbodiedAgentRuntime,
    FinishCheckResult,
    ImageAttachment,
    OpenCodeToolChatClient,
    PrefixStableContextManager,
    ToolCall,
    ToolResult,
    ToolSpec,
    TurnUsage,
    build_default_deepseek_fallback,
    default_after_tool_hook,
    resolve_agent_thinking,
)
from embodied_navigation.agent.tools import (
    ObjectNavFinishVerifier,
    ReportFinishVerifier,
    build_agent_tools,
    format_agent_candidates,
)

__all__ = [
    "AGENT_RUNTIME_SYSTEM_PROMPT",
    "AfterToolHook",
    "AgentRunResult",
    "AssistantTurn",
    "BeforeFinishHook",
    "BeforeTurnHook",
    "EmbodiedAgentRuntime",
    "FinishCheckResult",
    "HabitatAgentRunner",
    "HabitatSessionProxy",
    "HabitatWorldDaemon",
    "ImageAttachment",
    "MotionFrameIndex",
    "ObjectNavFinishVerifier",
    "OpenCodeToolChatClient",
    "PrefixStableContextManager",
    "ReportFinishVerifier",
    "SharedHabitatSceneHost",
    "ToolCall",
    "ToolResult",
    "ToolSpec",
    "TurnUsage",
    "build_agent_tools",
    "build_default_deepseek_fallback",
    "build_vlm_region_summarizer",
    "default_after_tool_hook",
    "format_agent_candidates",
    "resolve_agent_thinking",
    "run_concurrent_scene_sessions",
]
