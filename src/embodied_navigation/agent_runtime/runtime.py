"""Task-agnostic multi-turn ReAct / tool-calling EmbodiedAgentRuntime."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

from embodied_navigation.agent_runtime.context import PrefixStableContextManager
from embodied_navigation.agent_runtime.hooks import (
    AfterToolHook,
    BeforeFinishHook,
    BeforeTurnHook,
    default_after_tool_hook,
)
from embodied_navigation.agent_runtime.types import (
    AssistantTurn,
    FinishCheckResult,
    ImageAttachment,
    ToolCall,
    ToolResult,
    ToolSpec,
)

ToolHandler = Callable[[ToolCall], ToolResult]


@dataclass
class AgentRunResult:
    """Execution outcome, tool history, and full CoT/token telemetry for one task."""

    completed: bool
    termination_reason: str
    finish_metadata: dict[str, Any] = field(default_factory=dict)
    turns_used: int = 0
    tool_calls_count: int = 0
    traces: list[dict[str, Any]] = field(default_factory=list)
    context_metrics: dict[str, Any] = field(default_factory=dict)


class EmbodiedAgentRuntime:
    """General multi-turn tool-calling agent runtime decoupled from specific tasks."""

    def __init__(
        self,
        *,
        chat_client: Any,
        context: PrefixStableContextManager,
        before_turn_hook: BeforeTurnHook | None = None,
        after_tool_hook: AfterToolHook | None = None,
        before_finish_hook: BeforeFinishHook | None = None,
        world_milestone_fn: Callable[[str], str] | None = None,
        budget_exhausted_fn: Callable[[], bool] | None = None,
        finish_tool_name: str = "finish_task",
    ) -> None:
        self.chat_client = chat_client
        self.context = context
        self.before_turn_hook = before_turn_hook
        self.after_tool_hook = after_tool_hook or default_after_tool_hook()
        self.before_finish_hook = before_finish_hook
        self.world_milestone_fn = world_milestone_fn
        self.budget_exhausted_fn = budget_exhausted_fn
        self.finish_tool_name = str(finish_tool_name)
        self._specs: dict[str, ToolSpec] = {}
        self._handlers: dict[str, ToolHandler] = {}

    def register_tool(self, spec: ToolSpec, handler: ToolHandler) -> None:
        """Register an agent-callable tool with its JSON schema and handler."""
        self._specs[spec.name] = spec
        self._handlers[spec.name] = handler

    def _tool_specs_list(self) -> list[ToolSpec]:
        return list(self._specs.values())

    def _bind_after_hook(self, call: ToolCall, res: ToolResult) -> ToolResult:
        post = self.after_tool_hook(call, res)
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload=post.payload,
            images=post.images,
            is_error=post.is_error,
        )

    def _run_tool(self, call: ToolCall) -> tuple[ToolResult, FinishCheckResult | None]:
        if call.arguments.get("_parse_error"):
            res = ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={
                    "error": "invalid_json_arguments",
                    "raw_preview": str(call.arguments.get("_raw", ""))[:180],
                    "hint": "Call the tool again with valid JSON arguments.",
                },
                is_error=True,
            )
            return self._bind_after_hook(call, res), None

        if call.name == self.finish_tool_name and self.before_finish_hook is not None:
            ver = self.before_finish_hook(call)
            if not ver.allowed:
                rej_payload = {"allowed": False, "reason": ver.reason, **ver.metadata}
                res = ToolResult(
                    call_id=call.call_id,
                    name=call.name,
                    payload=rej_payload,
                    is_error=False,
                )
                return self._bind_after_hook(call, res), ver
            ok_payload = {"allowed": True, "reason": ver.reason, **ver.metadata}
            res = ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload=ok_payload,
                is_error=False,
            )
            return self._bind_after_hook(call, res), ver

        handler = self._handlers.get(call.name)
        if handler is None:
            res = ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={
                    "error": "unknown_tool",
                    "tool": call.name,
                    "available_tools": list(self._specs.keys()),
                },
                is_error=True,
            )
            return self._bind_after_hook(call, res), None

        try:
            raw_res = handler(call)
        except Exception as exc:
            raw_res = ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": f"tool_execution_error:{type(exc).__name__}"},
                is_error=True,
            )
        return self._bind_after_hook(call, raw_res), None

    def run_task(
        self,
        *,
        task_label: str = "",
        initial_observation_text: str = "",
        initial_images: Sequence[ImageAttachment] = (),
        max_turns: int = 25,
        append_transition: bool = False,
        transition_prompt: str = "",
    ) -> AgentRunResult:
        """Run one multi-turn agent loop on the live persistent context until finished or capped."""
        if append_transition:
            self.context.append_subtask_transition(
                transition_prompt,
                initial_images=initial_images,
                state_text=initial_observation_text,
            )
        elif initial_observation_text.strip() or initial_images:
            self.context.append_observation_turn(
                initial_observation_text.strip(), images=initial_images
            )

        traces: list[dict[str, Any]] = []
        tool_calls_count = 0
        consecutive_non_tool = 0
        consecutive_transport_errors = 0
        completed = False
        termination_reason = "max_turns_reached"
        finish_meta: dict[str, Any] = {}

        for turn_idx in range(1, max(1, int(max_turns)) + 1):
            if self.budget_exhausted_fn is not None and self.budget_exhausted_fn():
                termination_reason = "environment_budget_exhausted"
                break

            compaction_receipt: dict[str, Any] | None = None
            if self.context.needs_compaction():
                ms_text = (
                    self.world_milestone_fn(task_label)
                    if self.world_milestone_fn is not None
                    else f"Active task: {task_label or 'unspecified'}."
                )
                compaction_receipt = self.context.compact(ms_text)

            messages = self.context.build_messages()
            turn: AssistantTurn = self.chat_client.complete_turn(
                messages, self._tool_specs_list()
            )
            self.context.record_usage(turn.usage)

            trace_row: dict[str, Any] = {
                "turn": turn_idx,
                "reasoning_content": turn.reasoning_content,
                "content": turn.content,
                "tool_calls": [
                    {"id": tc.call_id, "name": tc.name, "arguments": tc.arguments}
                    for tc in turn.tool_calls
                ],
                "prompt_tokens": turn.usage.prompt_tokens,
                "cached_tokens": turn.usage.cached_tokens,
                "completion_tokens": turn.usage.completion_tokens,
                "reasoning_tokens": turn.usage.reasoning_tokens,
                "latency_ms": round(turn.usage.latency_ms, 1),
                "fallback": turn.fallback,
            }
            if turn.error_reason:
                trace_row["error_reason"] = turn.error_reason
            if compaction_receipt is not None:
                trace_row["compaction"] = compaction_receipt

            if turn.fallback:
                consecutive_transport_errors += 1
                traces.append(trace_row)
                if "http_400" in turn.error_reason and len(self.context._turn_groups) > 1:
                    ms_text = (
                        self.world_milestone_fn(task_label)
                        if self.world_milestone_fn is not None
                        else f"Active task: {task_label or 'unspecified'}."
                    )
                    self.context.compact(ms_text, force_shrink=True)
                if consecutive_transport_errors >= 3:
                    termination_reason = turn.error_reason or "repeated_transport_errors"
                    break
                continue
            consecutive_transport_errors = 0

            if not turn.tool_calls:
                consecutive_non_tool += 1
                traces.append(trace_row)
                if consecutive_non_tool >= 3:
                    termination_reason = "no_tool_calls_after_nudges"
                    break
                self.context.append_nudge(
                    turn,
                    "Please act by calling one of the registered tools "
                    f"({', '.join(self._specs.keys())}). Do not output plain prose without a tool call.",
                )
                continue
            consecutive_non_tool = 0

            results: list[ToolResult] = []
            finish_verdict: FinishCheckResult | None = None
            stop_batch_reason: str | None = None
            for tc in turn.tool_calls:
                if stop_batch_reason is not None:
                    results.append(
                        ToolResult(
                            call_id=tc.call_id,
                            name=tc.name,
                            payload={"skipped": True, "reason": stop_batch_reason},
                            is_error=False,
                        )
                    )
                    continue
                tool_calls_count += 1
                tr, fchk = self._run_tool(tc)
                results.append(tr)
                if tc.name == self.finish_tool_name and fchk is not None:
                    finish_verdict = fchk
                    if fchk.allowed:
                        stop_batch_reason = "task_finished"
                        continue
                if self.budget_exhausted_fn is not None and self.budget_exhausted_fn():
                    stop_batch_reason = "environment_budget_exhausted"

            trace_row["tool_results"] = [
                {"name": r.name, "is_error": r.is_error, "payload": r.payload}
                for r in results
            ]
            traces.append(trace_row)

            tail_delta = (
                self.before_turn_hook(turn_idx)
                if self.before_turn_hook is not None
                else ""
            )
            self.context.append_tool_exchange(
                turn, results, followup_text=tail_delta
            )

            if finish_verdict is not None and finish_verdict.allowed:
                completed = True
                termination_reason = finish_verdict.reason or "task_finished"
                finish_meta = dict(finish_verdict.metadata)
                break
            if self.budget_exhausted_fn is not None and self.budget_exhausted_fn():
                termination_reason = "environment_budget_exhausted"
                break

        return AgentRunResult(
            completed=completed,
            termination_reason=termination_reason,
            finish_metadata=finish_meta,
            turns_used=len(traces),
            tool_calls_count=tool_calls_count,
            traces=traces,
            context_metrics=self.context.metrics_dict(),
        )
