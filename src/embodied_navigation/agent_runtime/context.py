"""Prefix-stable multi-turn context manager with lazy 200K compaction for KV-cache efficiency."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Sequence

from embodied_navigation.agent_runtime.types import (
    AssistantTurn,
    ImageAttachment,
    ToolCall,
    ToolResult,
    TurnUsage,
)
from embodied_navigation.planning.exploration.llm_agent_client import _encode_image_part


def _estimate_message_tokens(msg: dict[str, Any]) -> tuple[int, int]:
    """Return conservative ``(token_estimate, image_count)`` for one OpenAI message."""
    chars = 0
    imgs = 0
    content = msg.get("content")
    if isinstance(content, str):
        chars += len(content)
    elif isinstance(content, list):
        for part in content:
            if not isinstance(part, dict):
                continue
            if part.get("type") == "image_url":
                imgs += 1
            elif part.get("type") == "text":
                chars += len(str(part.get("text", "")))
    reas = msg.get("reasoning_content")
    if isinstance(reas, str):
        chars += len(reas)
    tcs = msg.get("tool_calls")
    if isinstance(tcs, list):
        chars += len(json.dumps(tcs, ensure_ascii=False, separators=(",", ":")))
    return (max(1, chars // 3) + imgs * 320, imgs)


class PrefixStableContextManager:
    """Maintains an append-only multi-turn message list inside a 200K token budget.

    Layout between compactions (strictly immutable prefix for server KV-cache hits):
      1. ``_system_msg`` (FROZEN system prompt)
      2. ``_initial_user_msg`` (FROZEN initial goal prompt)
      3. ``_milestone_msg`` (optional, set ONLY when lazy compaction fires)
      4. ``_turn_groups`` (atomic append-only groups; never mutated after emission)

    Tool-call atomicity invariant: an ``assistant`` message carrying ``tool_calls``
    and all its corresponding ``tool`` response messages (plus any trailing ``user``
    multimodal observation) reside inside the SAME group element so compaction never
    orphans tool results from their tool call IDs.
    """

    def __init__(
        self,
        *,
        system_prompt: str,
        initial_task_prompt: str,
        allowed_image_roots: Iterable[str | Path] | None = None,
        max_context_tokens: int = 200_000,
        soft_compaction_tokens: int = 155_000,
        keep_recent_groups: int = 4,
    ) -> None:
        self.allowed_image_roots = (
            tuple(Path(r) for r in allowed_image_roots)
            if allowed_image_roots is not None
            else None
        )
        self.max_context_tokens = max(8192, int(max_context_tokens))
        self.soft_compaction_tokens = max(
            4096, min(self.max_context_tokens - 2048, int(soft_compaction_tokens))
        )
        self.keep_recent_groups = max(1, int(keep_recent_groups))

        self._system_msg: dict[str, Any] = {
            "role": "system",
            "content": str(system_prompt),
        }
        self._initial_user_msg: dict[str, Any] = {
            "role": "user",
            "content": str(initial_task_prompt),
        }
        self._milestone_msg: dict[str, Any] | None = None
        self._turn_groups: list[list[dict[str, Any]]] = []

        # Telemetry metrics
        self.last_prompt_tokens: int = 0
        self._last_measured_group_count: int = 0
        self.max_prompt_tokens_seen: int = 0
        self.total_prompt_tokens: int = 0
        self.total_cached_tokens: int = 0
        self.total_completion_tokens: int = 0
        self.total_reasoning_tokens: int = 0
        self.compaction_count: int = 0
        self.rejected_images: list[dict[str, str]] = []

    def _build_user_multimodal_msg(
        self,
        text: str,
        images: Sequence[ImageAttachment] = (),
    ) -> dict[str, Any]:
        if not images:
            return {"role": "user", "content": text}

        seen_paths: set[Path] = set()
        img_labels: list[str] = []
        img_parts: list[dict[str, Any]] = []
        for idx, att in enumerate(images, start=1):
            try:
                resolved = Path(att.path).resolve()
            except (OSError, ValueError):
                self.rejected_images.append(
                    {"path": str(att.path), "reason": "resolve_error"}
                )
                continue
            if resolved in seen_paths:
                continue
            part = _encode_image_part(resolved, allowed_roots=self.allowed_image_roots)
            if part is None:
                self.rejected_images.append(
                    {"path": str(att.path), "reason": "rejected_by_guard"}
                )
                continue
            seen_paths.add(resolved)
            lbl = att.label.strip() if att.label else resolved.name
            img_labels.append(f"Image {len(img_parts) + 1}: {lbl}")
            img_parts.append(part)

        if not img_parts:
            return {"role": "user", "content": text}
        header = f"[ATTACHED_IMAGES: {'; '.join(img_labels)}]\n"
        return {
            "role": "user",
            "content": [{"type": "text", "text": header + text}, *img_parts],
        }

    def append_observation_turn(
        self,
        text: str,
        *,
        images: Sequence[ImageAttachment] = (),
    ) -> None:
        """Append a fresh user observation group without modifying prior prefix."""
        msg = self._build_user_multimodal_msg(text, images)
        self._turn_groups.append([msg])

    def append_subtask_transition(
        self,
        subtask_prompt: str,
        *,
        initial_images: Sequence[ImageAttachment] = (),
        state_text: str = "",
    ) -> None:
        """Append a sequential subtask switch into the live conversation preserving KV-cache."""
        parts = [p for p in (subtask_prompt.strip(), state_text.strip()) if p]
        self.append_observation_turn("\n".join(parts), images=initial_images)

    def append_tool_exchange(
        self,
        turn: AssistantTurn,
        tool_results: Sequence[ToolResult],
        *,
        followup_text: str = "",
        followup_images: Sequence[ImageAttachment] = (),
    ) -> None:
        """Atomically append assistant tool calls, serialized tool outputs, and new images.

        Note: OpenAI/OpenCode multimodal gateways expect ``image_url`` blocks inside
        ``role="user"`` messages rather than ``role="tool"``; therefore ``role="tool"``
        always carries compact JSON text, and any newly rendered RGB/floorplan images
        are attached in a trailing ``role="user"`` message within the same atomic group.
        """
        group: list[dict[str, Any]] = []
        asst_msg: dict[str, Any] = {
            "role": "assistant",
            "content": turn.content if turn.content else "",
        }
        if turn.reasoning_content:
            asst_msg["reasoning_content"] = turn.reasoning_content
        if turn.tool_calls:
            asst_msg["tool_calls"] = [
                {
                    "id": tc.call_id,
                    "type": "function",
                    "function": {
                        "name": tc.name,
                        "arguments": (
                            tc.raw_arguments
                            if tc.raw_arguments
                            else json.dumps(tc.arguments, ensure_ascii=False)
                        ),
                    },
                }
                for tc in turn.tool_calls
            ]
        group.append(asst_msg)

        all_images: list[ImageAttachment] = []
        for tr in tool_results:
            group.append(
                {
                    "role": "tool",
                    "tool_call_id": tr.call_id,
                    "content": json.dumps(
                        tr.payload, ensure_ascii=False, separators=(",", ":")
                    ),
                }
            )
            if tr.images:
                all_images.extend(tr.images)

        if followup_images:
            all_images.extend(followup_images)

        if all_images or followup_text.strip():
            group.append(
                self._build_user_multimodal_msg(
                    followup_text.strip() or "[UPDATED_OBSERVATION]",
                    all_images,
                )
            )
        self._turn_groups.append(group)

    def append_nudge(self, turn: AssistantTurn, nudge_text: str) -> None:
        """Append a non-tool assistant reply followed by a compact tool-use nudge."""
        asst_msg: dict[str, Any] = {
            "role": "assistant",
            "content": turn.content if turn.content else "",
        }
        if turn.reasoning_content:
            asst_msg["reasoning_content"] = turn.reasoning_content
        self._turn_groups.append(
            [asst_msg, {"role": "user", "content": str(nudge_text)}]
        )

    @staticmethod
    def _merge_adjacent_user_messages(
        msgs: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        """Merge consecutive ``role='user'`` entries cleanly without altering storage."""
        merged: list[dict[str, Any]] = []
        for msg in msgs:
            if msg.get("role") == "user" and merged and merged[-1].get("role") == "user":
                prev = merged[-1]
                prev_c = prev.get("content")
                cur_c = msg.get("content")
                if isinstance(prev_c, str) and isinstance(cur_c, str):
                    merged[-1] = {"role": "user", "content": f"{prev_c}\n{cur_c}"}
                else:
                    blocks: list[dict[str, Any]] = []
                    for c in (prev_c, cur_c):
                        if isinstance(c, str) and c:
                            blocks.append({"type": "text", "text": c})
                        elif isinstance(c, list):
                            blocks.extend(c)
                    merged[-1] = {"role": "user", "content": blocks}
            else:
                merged.append(msg)
        return merged

    def build_messages(self) -> list[dict[str, Any]]:
        """Assemble full OpenAI ``messages`` array preserving exact prefix identity."""
        msgs: list[dict[str, Any]] = [self._system_msg, self._initial_user_msg]
        if self._milestone_msg is not None:
            msgs.append(self._milestone_msg)
        for grp in self._turn_groups:
            msgs.extend(grp)
        return self._merge_adjacent_user_messages(msgs)

    def record_usage(self, usage: TurnUsage) -> None:
        pt = max(0, int(usage.prompt_tokens))
        ct = max(0, int(usage.completion_tokens))
        rt = max(0, int(usage.reasoning_tokens))
        cached = max(0, int(usage.cached_tokens))
        if pt > 0:
            self.last_prompt_tokens = pt
            self._last_measured_group_count = len(self._turn_groups)
            if pt > self.max_prompt_tokens_seen:
                self.max_prompt_tokens_seen = pt
            self.total_prompt_tokens += pt
        self.total_cached_tokens += cached
        self.total_completion_tokens += ct
        self.total_reasoning_tokens += rt

    def estimate_tokens(self) -> int:
        total = 0
        for msg in self.build_messages():
            tok, _imgs = _estimate_message_tokens(msg)
            total += tok
        return total

    def estimate_next_turn_tokens(
        self, *, tool_schema_overhead: int = 1200, completion_reserve: int = 4096
    ) -> int:
        """Estimate next request size including deltas added after last measured prompt."""
        if self.last_prompt_tokens > 0 and self._last_measured_group_count <= len(self._turn_groups):
            delta = 0
            for grp in self._turn_groups[self._last_measured_group_count :]:
                for msg in grp:
                    tok, _imgs = _estimate_message_tokens(msg)
                    delta += tok
            return self.last_prompt_tokens + delta + int(completion_reserve)
        return self.estimate_tokens() + int(tool_schema_overhead) + int(completion_reserve)

    def needs_compaction(self) -> bool:
        if len(self._turn_groups) <= 1:
            return False
        return self.estimate_next_turn_tokens() >= self.soft_compaction_tokens

    def compact(self, milestone_summary_text: str, *, force_shrink: bool = False) -> dict[str, Any]:
        """Fold older turn groups into one goal-conditioned world milestone."""
        before_groups = len(self._turn_groups)
        before_est = self.estimate_next_turn_tokens()
        self._milestone_msg = {
            "role": "user",
            "content": "## [COMPACTED_WORLD_MILESTONE]\n" + str(milestone_summary_text).strip(),
        }
        keep_n = (
            max(1, min(self.keep_recent_groups, before_groups) - 1)
            if (force_shrink or before_groups <= self.keep_recent_groups)
            else self.keep_recent_groups
        )
        kept = list(self._turn_groups[-keep_n:]) if keep_n > 0 else []
        self._turn_groups = kept
        self.compaction_count += 1
        self.last_prompt_tokens = 0
        self._last_measured_group_count = 0
        return {
            "compaction_index": self.compaction_count,
            "dropped_groups": max(0, before_groups - len(kept)),
            "kept_groups": len(kept),
            "before_tokens": before_est,
            "after_est_tokens": self.estimate_tokens(),
        }

    def metrics_dict(self) -> dict[str, Any]:
        cache_rate = (
            round(self.total_cached_tokens / float(self.total_prompt_tokens), 4)
            if self.total_prompt_tokens > 0
            else 0.0
        )
        return {
            "max_context_tokens": self.max_context_tokens,
            "soft_compaction_tokens": self.soft_compaction_tokens,
            "max_prompt_tokens_seen": self.max_prompt_tokens_seen,
            "total_prompt_tokens": self.total_prompt_tokens,
            "total_cached_tokens": self.total_cached_tokens,
            "cache_hit_rate": cache_rate,
            "total_completion_tokens": self.total_completion_tokens,
            "total_reasoning_tokens": self.total_reasoning_tokens,
            "compactions_count": self.compaction_count,
            "active_turn_groups": len(self._turn_groups),
        }
