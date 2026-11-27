"""Unit tests for standalone EmbodiedAgentRuntime, 200K context, hooks, and world upgrades."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np
from PIL import Image

from embodied_navigation.agent import (
    AssistantTurn,
    EmbodiedAgentRuntime,
    FinishCheckResult,
    HabitatWorldDaemon,
    ImageAttachment,
    OpenCodeToolChatClient,
    PrefixStableContextManager,
    ReportFinishVerifier,
    ToolCall,
    ToolResult,
    ToolSpec,
    TurnUsage,
    default_after_tool_hook,
    resolve_agent_thinking,
)
from embodied_navigation.common.llm_config import LlmClientConfig
from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.perception.vision_tools import (
    VisionDetection,
    cluster_multilabel_detections,
    ground_detections_with_depth,
)
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    DepthOccupancyGrid,
)


class TestPrefixStableContextManager(unittest.TestCase):
    def test_prefix_immutability_images_only_in_user_and_lazy_compaction(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            img1 = root / "f1.png"
            img2 = root / "f2.png"
            Image.new("RGB", (32, 32), (10, 20, 30)).save(img1)
            Image.new("RGB", (32, 32), (40, 50, 60)).save(img2)

            ctx = PrefixStableContextManager(
                system_prompt="SYS_FROZEN",
                initial_task_prompt="GOAL_FROZEN",
                allowed_image_roots=[root],
                max_context_tokens=200_000,
                soft_compaction_tokens=155_000,
                keep_recent_groups=2,
            )
            ctx.append_observation_turn(
                "Step 1 obs", images=[ImageAttachment(img1, "front RGB")]
            )
            snap_initial = json.dumps(ctx.build_messages(), sort_keys=True)
            prefix_len = len(ctx.build_messages())

            # Append 5 multi-turn tool exchanges; earlier prefix bytes MUST stay bit-identical
            for step in range(1, 6):
                turn = AssistantTurn(
                    content=f"calling nav {step}",
                    reasoning_content=f"CoT step {step}",
                    tool_calls=(
                        ToolCall(
                            call_id=f"call_{step}",
                            name="navigate_to",
                            arguments={"target_id": f"F{step}"},
                        ),
                    ),
                    usage=TurnUsage(prompt_tokens=1200 * step, cached_tokens=900 * step),
                )
                ctx.record_usage(turn.usage)
                tr = ToolResult(
                    call_id=f"call_{step}",
                    name="navigate_to",
                    payload={"status": "completed", "step": step},
                    images=(ImageAttachment(img2, "front RGB"),),
                )
                ctx.append_tool_exchange(turn, [tr], followup_text=f"[map_rev={step}]")
                cur_msgs = ctx.build_messages()
                self.assertEqual(
                    json.dumps(cur_msgs[:prefix_len], sort_keys=True),
                    snap_initial,
                )
                self.assertFalse(ctx.needs_compaction())

            msgs = ctx.build_messages()
            # Verify no role="tool" message contains image blocks
            for m in msgs:
                if m["role"] == "tool":
                    self.assertIsInstance(m["content"], str)
            # Verify reasoning_content preserved on assistant messages
            asst_msgs = [m for m in msgs if m["role"] == "assistant"]
            self.assertEqual(len(asst_msgs), 5)
            self.assertEqual(asst_msgs[2]["reasoning_content"], "CoT step 3")

            # Force compaction when hitting soft limit and verify atomic assistant+tool preservation
            ctx.record_usage(TurnUsage(prompt_tokens=160_000))
            self.assertTrue(ctx.needs_compaction())
            receipt = ctx.compact("Zones: Z1(explored), Z2(partial)")
            self.assertEqual(receipt["compaction_index"], 1)
            self.assertEqual(receipt["kept_groups"], 2)
            post = ctx.build_messages()
            self.assertEqual(post[0]["content"], "SYS_FROZEN")
            self.assertIn("GOAL_FROZEN", str(post[1]["content"]))
            self.assertIn("COMPACTED_WORLD_MILESTONE", str(post[1]["content"]))
            self.assertEqual(post[2]["role"], "assistant")

            # Every tool response in post-compaction messages must have its matching assistant tool_call
            seen_call_ids: set[str] = set()
            for m in post:
                if m["role"] == "assistant":
                    for tc in m.get("tool_calls") or ():
                        seen_call_ids.add(tc["id"])
                elif m["role"] == "tool":
                    self.assertIn(m["tool_call_id"], seen_call_ids)

            # Subtask transition appends cleanly (merged with preceding user block if adjacent)
            ctx.append_subtask_transition("NEXT_GOAL: find bed", state_text="zone=Z2")
            last_msg = ctx.build_messages()[-1]
            self.assertEqual(last_msg["role"], "user")
            self.assertIn("NEXT_GOAL: find bed", json.dumps(last_msg["content"], ensure_ascii=False))

    def test_adjacent_user_messages_merge_cleanly(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            ctx = PrefixStableContextManager(
                system_prompt="SYS",
                initial_task_prompt="GOAL",
                allowed_image_roots=[root],
            )
            ctx.append_observation_turn("obs A")
            ctx.append_subtask_transition("NEXT GOAL", state_text="obs B")
            msgs = ctx.build_messages()
            for a, b in zip(msgs, msgs[1:]):
                self.assertFalse(
                    a.get("role") == "user" and b.get("role") == "user",
                    msg=f"Consecutive user messages found: {msgs}",
                )
            self.assertIn("obs A", str(msgs[-1]["content"]))
            self.assertIn("obs B", str(msgs[-1]["content"]))


class TestMultiLabelVisionAndWorldUpgrades(unittest.TestCase):
    def test_cluster_multilabel_and_abstain(self) -> None:
        m = np.zeros((64, 64), dtype=bool)
        m[10:50, 10:50] = True
        m_low = np.zeros((64, 64), dtype=bool)
        m_low[2:14, 2:14] = True

        dets = [
            VisionDetection("desk", 0.82, (10, 10, 50, 50), mask=m, backend="sam3"),
            VisionDetection("table", 0.65, (10, 10, 50, 50), mask=m, backend="sam3"),
            VisionDetection("stair railing", 0.18, (2, 2, 14, 14), mask=m_low, backend="sam3"),
        ]
        clustered = cluster_multilabel_detections(dets, abstain_threshold=0.25)
        self.assertEqual(len(clustered), 2)
        self.assertEqual(clustered[0][1], {"desk": 0.82, "table": 0.65})
        self.assertFalse(clustered[0][2])
        self.assertTrue(clustered[1][2])

        depth = np.full((64, 64), 1.5, dtype=np.float32)
        grounded = ground_detections_with_depth(
            dets, depth, multilabel_cluster=True, abstain_threshold=0.25, min_valid_pixels=16
        )
        self.assertEqual(len(grounded), 2)
        pd0 = grounded[0].to_prompt_dict()
        self.assertEqual(pd0["candidate_scores"], {"desk": 0.82, "table": 0.65})
        self.assertNotIn("abstain", pd0)
        self.assertTrue(grounded[1].to_prompt_dict()["abstain"])

    def test_update_object_belief_refute_and_zone_coverage(self) -> None:
        mem = HierarchicalMemoryStore(KeyframeSceneGraph(), default_stop_radius_m=1.8)
        mem.graph.add_keyframe("kf1", np.eye(4), rgb_path="kf1.png")
        node, _ = mem.add_or_fuse_object(
            label="chair",
            keyframe_id="kf1",
            local_xyz=[1.0, 0.0, 0.0],
            producer="sam3_depth_grounded",
            range_source="vision_mask_depth_backproj",
        )
        self.assertEqual(len(mem.memory_goal_candidates([0.0, 0.0, 0.0], target_label="chair")), 1)

        # Refute as pink bedskirt -> disappears from 'chair' candidates and stop verification,
        # and suppresses re-adding a duplicate 'chair' at that spot.
        res = mem.update_object_belief(
            node.object_id,
            status="refuted",
            new_label="pink bedskirt",
            note="inspected close up",
        )
        self.assertEqual(res["epistemic_status"], "refuted")
        self.assertEqual(res["refuted_labels"], ["chair"])
        self.assertEqual(mem.memory_goal_candidates([0.0, 0.0, 0.0], target_label="chair"), [])
        ver = mem.verify_stop_claim(
            [0.0, 0.0, 0.0], target_label="chair", require_current_keyframe_id="kf1"
        )
        self.assertFalse(ver.allowed)
        # Re-querying with relabeled class still works
        self.assertEqual(
            len(mem.memory_goal_candidates([0.0, 0.0, 0.0], target_label="pink bedskirt")), 1
        )
        # Same-spot duplicate 'chair' is suppressed
        dup, fused = mem.add_or_fuse_object(
            label="chair",
            keyframe_id="kf1",
            local_xyz=[1.05, 0.02, 0.0],
            producer="sam3_depth_grounded",
        )
        self.assertTrue(fused)
        self.assertEqual(dup.object_id, node.object_id)

        # Zone coverage tracking distinguishes glimpsed vs inspected_closeup
        z1 = mem.record_zone_observation([0.0, 0.0, 0.0], median_depth_m=4.5, zone_type="living_room", step_seq=1)
        z2 = mem.record_zone_observation([5.0, 0.0, 0.0], median_depth_m=1.4, zone_type="bedroom", step_seq=2)
        mem.record_zone_observation([5.1, 0.2, 0.0], median_depth_m=1.5, step_seq=3)
        c2f = mem.coarse_to_fine_search("chair", robot_xyz=[0.0, 0.0, 0.0])
        zcov = {z["zone_id"]: z for z in c2f["zone_coverage"]}
        self.assertEqual(zcov[z1]["status"], "glimpsed")
        self.assertEqual(zcov[z2]["status"], "inspected_closeup")

        # Global 2.5D floorplan renders cleanly
        grid = DepthOccupancyGrid()
        grid.update(np.full((64, 64), 2.5, dtype=np.float32), [0.0, 0.0, 0.88], 0.0)
        with tempfile.TemporaryDirectory() as td:
            fp = grid.render_global_floorplan(
                Path(td) / "fp.png",
                [0.0, 0.0, 0.88],
                0.0,
                zones=c2f["zone_coverage"],
                objects=mem.l2_submap_entities(include_stale=False)["objects"],
            )
            self.assertTrue(fp.is_file())

    def test_cognitive_relabel_producer_guard(self) -> None:
        mem = HierarchicalMemoryStore(KeyframeSceneGraph(), default_stop_radius_m=1.8)
        mem.graph.add_keyframe("kf1", np.eye(4), rgb_path="kf1.png")
        node, _ = mem.add_or_fuse_object(
            label="table",
            keyframe_id="kf1",
            local_xyz=[1.0, 0.0, 0.0],
            producer="sam3_depth_grounded",
            range_source="vision_mask_depth_backproj",
        )
        res = mem.update_object_belief(node.object_id, status="confirmed", new_label="chair")
        self.assertEqual(res["label"], "chair")
        self.assertEqual(node.producer, "agent_relabeled")
        meta = mem._object_meta[node.object_id]
        self.assertEqual(meta.last_verified_label, "table")
        ver = mem.verify_stop_claim(
            [0.0, 0.0, 0.0],
            target_label="chair",
            require_current_keyframe_id="kf1",
            excluded_producers=("agent_relabeled",),
        )
        self.assertFalse(ver.allowed)


class TestEmbodiedAgentRuntimeLoopAndTransport(unittest.TestCase):
    def test_runtime_end_to_end_refute_batch_skip_and_report_verifier(self) -> None:
        class _ScriptedChatClient:
            def __init__(self, turns: list[AssistantTurn]) -> None:
                self._turns = list(turns)
                self.seen_messages: list[list[dict]] = []

            def complete_turn(self, messages, tools):
                self.seen_messages.append(messages)
                return self._turns.pop(0)

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            img = root / "s1.png"
            Image.new("RGB", (32, 32), (80, 90, 100)).save(img)

            mem = HierarchicalMemoryStore(KeyframeSceneGraph())
            mem.graph.add_keyframe("kf1", np.eye(4), rgb_path=str(img))
            mem.add_or_fuse_object(
                "chair", "kf1", [1.0, 0.0, 0.0], producer="sam3_depth_grounded"
            )

            ctx = PrefixStableContextManager(
                system_prompt="SYS",
                initial_task_prompt="GOAL 1: chair",
                allowed_image_roots=[root],
            )
            gate_calls = [0]

            def _finish_hook(call: ToolCall) -> FinishCheckResult:
                gate_calls[0] += 1
                if gate_calls[0] == 1:
                    return FinishCheckResult(allowed=False, reason="not_confirmed_yet")
                return FinishCheckResult(allowed=True, reason="verified_stop")

            budget_done = [False]
            chat = _ScriptedChatClient(
                [
                    AssistantTurn(
                        reasoning_content="Rejecting premature finish or checking",
                        tool_calls=(
                            ToolCall("c1", "finish_task", {"target_label": "chair"}),
                            ToolCall("c2", "update_belief", {"object_id": "obj_0001", "status": "refuted", "new_label": "bedskirt"}),
                        ),
                        usage=TurnUsage(prompt_tokens=100, cached_tokens=0),
                    ),
                    AssistantTurn(
                        reasoning_content="Stopping after move",
                        tool_calls=(
                            ToolCall("c3", "finish_task", {"target_label": "chair"}),
                            ToolCall("c4", "navigate_to", {"target_id": "F1"}),  # skipped because c3 finished!
                        ),
                        usage=TurnUsage(prompt_tokens=180, cached_tokens=90),
                    ),
                ]
            )
            rt = EmbodiedAgentRuntime(
                chat_client=chat,
                context=ctx,
                before_turn_hook=lambda i: f"[map_rev={mem.map_revision}]",
                after_tool_hook=default_after_tool_hook(max_payload_chars=400),
                before_finish_hook=_finish_hook,
                budget_exhausted_fn=lambda: budget_done[0],
            )
            rt.register_tool(
                ToolSpec("update_belief", "update", {"type": "object", "properties": {}}),
                lambda call: ToolResult(
                    call.call_id,
                    call.name,
                    {"rgb_path": "/secret/path.png", **mem.update_object_belief(call.arguments["object_id"], status=call.arguments["status"], new_label=call.arguments.get("new_label"))},
                ),
            )
            rt.register_tool(
                ToolSpec("navigate_to", "nav", {"type": "object", "properties": {}}),
                lambda call: ToolResult(call.call_id, call.name, {"status": "completed"}),
            )
            rt.register_tool(
                ToolSpec("finish_task", "finish", {"type": "object", "properties": {}}),
                lambda call: ToolResult(call.call_id, call.name, {"allowed": True}),
            )

            res = rt.run_task(
                task_label="chair",
                initial_observation_text="start",
                initial_images=[ImageAttachment(img, "front")],
                max_turns=5,
            )
            self.assertTrue(res.completed)
            self.assertEqual(res.turns_used, 2)
            # Host rgb_path stripped by AfterToolHook
            tr_c2 = res.traces[0]["tool_results"][1]["payload"]
            self.assertNotIn("rgb_path", tr_c2)
            self.assertEqual(tr_c2["epistemic_status"], "refuted")
            # Second tool in batch (c4) received synthetic skipped tool result so tool_call_id parity holds
            final_msgs = ctx.build_messages()
            asst_call_ids = {
                tc["id"]
                for m in final_msgs
                if m["role"] == "assistant"
                for tc in (m.get("tool_calls") or ())
            }
            tool_resp_ids = {m["tool_call_id"] for m in final_msgs if m["role"] == "tool"}
            self.assertEqual(asst_call_ids, tool_resp_ids)

            # ReportFinishVerifier validates non-ObjectNav EQA/report task cleanly
            class _DummyDaemon:
                memory = mem

            rep_hook = ReportFinishVerifier(_DummyDaemon())  # type: ignore[arg-type]
            bad_rep = rep_hook(ToolCall("r1", "finish_task", {"summary": ""}))
            self.assertFalse(bad_rep.allowed)
            ok_rep = rep_hook(
                ToolCall("r2", "finish_task", {"summary": "Bedroom has bedskirt", "evidence_ids": ["kf1", "missing_id"]})
            )
            self.assertTrue(ok_rep.allowed)
            self.assertEqual(ok_rep.metadata["validated_evidence_ids"], ["kf1"])

    def test_opencode_tool_chat_client_sse_streaming_with_reasoning_and_usage(self) -> None:
        sse_events = [
            {"choices": [{"delta": {"reasoning_content": "First step reasoning. "}}]},
            {"choices": [{"delta": {"content": "Checking.", "tool_calls": [{"index": 0, "id": "tc_1", "function": {"name": "inspect_vision", "arguments": '{"prompts":'}}]}}]},
            {"choices": [{"delta": {"tool_calls": [{"index": 0, "function": {"arguments": '["bed"]}'}}]}}]},
            {"choices": [], "usage": {"prompt_tokens": 500, "completion_tokens": 40, "prompt_tokens_details": {"cached_tokens": 400}, "completion_tokens_details": {"reasoning_tokens": 12}}},
        ]

        class _SseHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                _ = self.rfile.read(n)
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.end_headers()
                for ev in sse_events:
                    self.wfile.write(f"data: {json.dumps(ev)}\n\n".encode("utf-8"))
                self.wfile.write(b"data: [DONE]\n\n")

        srv = ThreadingHTTPServer(("127.0.0.1", 0), _SseHandler)
        port = srv.server_address[1]
        th = threading.Thread(target=srv.serve_forever, daemon=True)
        th.start()
        try:
            cfg = LlmClientConfig(
                provider="openai_compatible",
                base_url=f"http://127.0.0.1:{port}/v1",
                model="deepseek-v4.1-flash",
                api_key="test-key",
                stream=True,
                timeout_s=5.0,
            )
            cli = OpenCodeToolChatClient(cfg)
            turn = cli.complete_turn(
                [{"role": "user", "content": "hi"}],
                [ToolSpec("inspect_vision", "seg", {"type": "object", "properties": {}})],
            )
            self.assertFalse(turn.fallback)
            self.assertEqual(turn.reasoning_content, "First step reasoning. ")
            self.assertEqual(turn.content, "Checking.")
            self.assertEqual(len(turn.tool_calls), 1)
            self.assertEqual(turn.tool_calls[0].name, "inspect_vision")
            self.assertEqual(turn.tool_calls[0].arguments, {"prompts": ["bed"]})
            self.assertEqual(turn.usage.prompt_tokens, 500)
            self.assertEqual(turn.usage.cached_tokens, 400)
            self.assertEqual(turn.usage.reasoning_tokens, 12)

            ex_ds, to_ds, mt_ds = resolve_agent_thinking("deepseek-v4.1-flash", "default")
            self.assertEqual(ex_ds, {"thinking": {"type": "enabled"}})
            self.assertGreaterEqual(mt_ds, 16384)
            self.assertGreaterEqual(to_ds, 120.0)
        finally:
            srv.shutdown()
            srv.server_close()

    def test_prose_json_without_tool_calls_never_auto_executes(self) -> None:
        class _ScriptedChatClient:
            def __init__(self, turns: list[AssistantTurn]) -> None:
                self._turns = list(turns)

            def complete_turn(self, messages, tools):
                return self._turns.pop(0)

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            ctx = PrefixStableContextManager(
                system_prompt="SYS",
                initial_task_prompt="GOAL",
                allowed_image_roots=[root],
            )
            executed: list[str] = []
            chat = _ScriptedChatClient(
                [
                    AssistantTurn(
                        content='{"name": "navigate_to", "arguments": {"target_id": "F1"}}',
                        tool_calls=(),
                    ),
                    AssistantTurn(
                        content='{"name": "navigate_to", "arguments": {"target_id": "F1"}}',
                        tool_calls=(),
                    ),
                    AssistantTurn(
                        content='{"name": "navigate_to", "arguments": {"target_id": "F1"}}',
                        tool_calls=(),
                    ),
                ]
            )
            rt = EmbodiedAgentRuntime(chat_client=chat, context=ctx)
            rt.register_tool(
                ToolSpec("navigate_to", "nav", {"type": "object", "properties": {}}),
                lambda call: (executed.append(call.name) or ToolResult(call.call_id, call.name, {"ok": True})),
            )
            res = rt.run_task(task_label="chair", initial_observation_text="start", max_turns=4)
            self.assertFalse(res.completed)
            self.assertEqual(executed, [])
            self.assertEqual(res.tool_calls_count, 0)
            self.assertEqual(res.termination_reason, "no_tool_calls_after_nudges")

    def test_sse_truncation_returns_fallback_empty_tool_calls(self) -> None:
        partial_events = [
            {"choices": [{"delta": {"tool_calls": [{"index": 0, "id": "tc_1", "function": {"name": "navigate_to", "arguments": '{"target_id":'}}]}}]},
        ]

        class _TruncatedSseHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                _ = self.rfile.read(n)
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.end_headers()
                for ev in partial_events:
                    self.wfile.write(f"data: {json.dumps(ev)}\n\n".encode("utf-8"))
                # Intentionally omit finish_reason and [DONE] to simulate mid-stream truncation.

        srv = ThreadingHTTPServer(("127.0.0.1", 0), _TruncatedSseHandler)
        port = srv.server_address[1]
        th = threading.Thread(target=srv.serve_forever, daemon=True)
        th.start()
        try:
            cfg = LlmClientConfig(
                provider="openai_compatible",
                base_url=f"http://127.0.0.1:{port}/v1",
                model="deepseek-v4.1-flash",
                api_key="test-key",
                stream=True,
                timeout_s=5.0,
            )
            cli = OpenCodeToolChatClient(cfg)
            turn = cli.complete_turn(
                [{"role": "user", "content": "hi"}],
                [ToolSpec("navigate_to", "nav", {"type": "object", "properties": {}})],
            )
            self.assertTrue(turn.fallback)
            self.assertEqual(turn.tool_calls, ())
            self.assertEqual(turn.error_reason, "truncated_sse_stream")
        finally:
            srv.shutdown()
            srv.server_close()

    def test_runtime_unlimited_turns_bounded_by_atomic_budget_and_deadlock_guard(self) -> None:
        from embodied_navigation.agent.runtime import (
            EmbodiedAgentRuntime,
            PrefixStableContextManager,
        )

        # 1) max_turns=None must run past 25 turns and stop strictly when atomic budget hits 500
        prims = [0]

        class _StepClient:
            def __init__(self) -> None:
                self.n = 0

            def complete_turn(self, messages: list[dict], tools: list[ToolSpec]) -> AssistantTurn:
                self.n += 1
                return AssistantTurn(
                    content=f"step {self.n}",
                    tool_calls=(ToolCall(f"c{self.n}", "navigate_to", {"target_id": "F1"}),),
                    usage=TurnUsage(prompt_tokens=100, cached_tokens=80, completion_tokens=10),
                )

        ctx = PrefixStableContextManager(system_prompt="sys", initial_task_prompt="task")
        rt = EmbodiedAgentRuntime(
            chat_client=_StepClient(),
            context=ctx,
            budget_exhausted_fn=lambda: prims[0] >= 500,
            motion_progress_fn=lambda: prims[0],
        )

        def _nav(call: ToolCall) -> ToolResult:
            prims[0] += 16
            return ToolResult(call.call_id, call.name, {"primitives": 16, "atomic_steps_used": prims[0]})

        rt.register_tool(ToolSpec("navigate_to", "nav", {"type": "object", "properties": {}}), _nav)
        res = rt.run_task(task_label="chair", initial_observation_text="init", max_turns=None)
        self.assertFalse(res.completed)
        self.assertEqual(res.termination_reason, "environment_budget_exhausted")
        self.assertGreater(res.turns_used, 25)
        self.assertGreaterEqual(prims[0], 500)

        # 2) Pure zero-motion deadlock breaks after max_consecutive_zero_motion_turns
        prims2 = [0]
        ctx2 = PrefixStableContextManager(system_prompt="sys", initial_task_prompt="task")
        rt2 = EmbodiedAgentRuntime(
            chat_client=_StepClient(),
            context=ctx2,
            budget_exhausted_fn=lambda: prims2[0] >= 500,
            motion_progress_fn=lambda: prims2[0],
            max_consecutive_zero_motion_turns=5,
        )
        rt2.register_tool(
            ToolSpec("navigate_to", "nav", {"type": "object", "properties": {}}),
            lambda call: ToolResult(call.call_id, call.name, {"primitives": 0}),
        )
        res2 = rt2.run_task(task_label="chair", initial_observation_text="init", max_turns=None)
        self.assertEqual(res2.termination_reason, "zero_motion_deadlock")
        self.assertEqual(res2.turns_used, 5)

    def test_depth_occupancy_grid_extracts_descending_and_ascending_stair_portals(self) -> None:
        import math
        import numpy as np
        from embodied_navigation.planning.exploration.habitat_memory_runner import DepthOccupancyGrid

        # Descending staircase ahead: lower image rows see surface 0.7..1.4m below robot floor
        grid_down = DepthOccupancyGrid()
        h, w = 64, 64
        depth_down = np.full((h, w), 3.0, dtype=np.float32)
        fx = (0.5 * w) / math.tan(math.radians(grid_down.hfov_deg) * 0.5)
        for v in range(44, 62):
            # Choose depth so z_cam = -(v - cy)*d/fx + 0.88 is around -0.8m
            dy_pix = v - 0.5 * h
            d_val = (0.88 + 0.8) * fx / max(1.0, dy_pix)
            depth_down[v, 20:44] = np.clip(d_val, 1.2, 3.8)
        grid_down.update(depth_down, [0.0, 0.0, 2.8], 0.0, stride=2)
        portals_down = grid_down.extract_stair_portals([0.0, 0.0, 2.8], min_samples=3)
        self.assertTrue(any(p["kind"] == "stair_down" and p["id"].startswith("S") for p in portals_down))

        # Ascending staircase ahead: higher image rows have monotonically increasing depth (slope ~ 1.2)
        grid_up = DepthOccupancyGrid()
        depth_up = np.full((h, w), 9.0, dtype=np.float32)
        for v in range(18, 42):
            # Target z_cam increasing from 0.25 to 1.10 as v decreases, with d = 1.6 + 1.2 * z_cam
            z_target = 0.25 + (41 - v) * (0.85 / 23.0)
            d_target = 1.6 + 1.2 * z_target
            # Solve v consistency approx by setting depth band directly
            depth_up[v, 20:44] = d_target
        grid_up.update(depth_up, [0.0, 0.0, 0.0], 0.0, stride=2)
        portals_up = grid_up.extract_stair_portals([0.0, 0.0, 0.0], min_samples=3)
        self.assertTrue(any(p["kind"] == "stair_up" and p["id"].startswith("S") for p in portals_up))

    def test_objectnav_finish_verifier_rejects_nightstand_and_low_score_table(self) -> None:
        import numpy as np
        from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
            HierarchicalMemoryStore,
        )
        from embodied_navigation.localization_mapping.semantic_mapping.scene_graph import (
            KeyframeSceneGraph,
        )
        from embodied_navigation.perception.vision_tools import GroundedDetection
        from embodied_navigation.agent.tools import (
            ObjectNavFinishVerifier,
        )

        mem = HierarchicalMemoryStore(KeyframeSceneGraph(), default_stop_radius_m=1.8)
        pose = np.eye(4, dtype=np.float64)
        mem.graph.add_keyframe("kf_0001", pose)
        mem.add_or_fuse_object(
            label="table",
            keyframe_id="kf_0001",
            local_xyz=[1.0, 0.0, 0.0],
            confidence=0.72,
            producer="sam3",
        )

        class _FakeDaemon:
            vision_has_seg = True
            vision_backend = "sam3"
            current_kf_id = "kf_0001"
            latest_disambiguated_ok = False
            latest_disambiguated_kf = None

            def __init__(self, dets: list[GroundedDetection]) -> None:
                self.memory = mem
                self.current_detections = dets

            def agent_pose(self) -> tuple[list[float], float, np.ndarray]:
                return [0.0, 0.0, 0.0], 0.0, np.zeros(3)

        # 1) Reject when nightstand outscores table in contrastive candidate_scores
        det_nightstand = GroundedDetection(
            det_id="V1",
            label="table",
            score=0.52,
            box_xyxy=(10.0, 10.0, 60.0, 60.0),
            local_xyz=(1.1, 0.0, 0.0),
            extent_xyz_m=(0.5, 0.5, 0.6),
            surface_distance_m=1.1,
            planar_depth_m=1.1,
            bearing_deg=0.0,
            valid_pixels=400,
            backend="sam3",
            candidate_scores={"table": 0.52, "nightstand": 0.61},
        )
        ver1 = ObjectNavFinishVerifier(daemon=_FakeDaemon([det_nightstand]), target_category="table", stop_radius_m=1.8)  # type: ignore[arg-type]
        chk1 = ver1(ToolCall("f1", "finish_task", {"result_type": "arrived", "target_label": "table"}))
        self.assertFalse(chk1.allowed)
        self.assertEqual(chk1.reason, "confusable_class_override")

        # 2) Reject low-score 0.38 table flicker without disambiguation
        det_low = GroundedDetection(
            det_id="V1",
            label="table",
            score=0.38,
            box_xyxy=(10.0, 10.0, 60.0, 60.0),
            local_xyz=(1.0, 0.0, 0.0),
            extent_xyz_m=(0.6, 0.6, 0.7),
            surface_distance_m=1.0,
            planar_depth_m=1.0,
            bearing_deg=0.0,
            valid_pixels=400,
            backend="sam3",
            candidate_scores={"table": 0.38},
        )
        ver2 = ObjectNavFinishVerifier(daemon=_FakeDaemon([det_low]), target_category="table", stop_radius_m=1.8)  # type: ignore[arg-type]
        chk2 = ver2(ToolCall("f2", "finish_task", {"result_type": "arrived", "target_label": "table"}))
        self.assertFalse(chk2.allowed)
        self.assertEqual(chk2.reason, "low_confidence_target_detection")

        # 3) Allow genuine high-confidence table
        det_ok = GroundedDetection(
            det_id="V1",
            label="table",
            score=0.72,
            box_xyxy=(10.0, 10.0, 120.0, 90.0),
            local_xyz=(1.0, 0.0, 0.0),
            extent_xyz_m=(1.4, 0.9, 0.75),
            surface_distance_m=1.0,
            planar_depth_m=1.0,
            bearing_deg=0.0,
            valid_pixels=900,
            backend="sam3",
            candidate_scores={"table": 0.72, "dining table": 0.78, "nightstand": 0.12},
        )
        ver3 = ObjectNavFinishVerifier(daemon=_FakeDaemon([det_ok]), target_category="table", stop_radius_m=1.8)  # type: ignore[arg-type]
        chk3 = ver3(ToolCall("f3", "finish_task", {"result_type": "arrived", "target_label": "table"}))
        self.assertTrue(chk3.allowed)

    def test_resolve_agent_thinking_qwen_timeout_floor_180s(self) -> None:
        from embodied_navigation.agent.runtime import resolve_agent_thinking

        _, t_dis, _ = resolve_agent_thinking("qwen3.8-flash", "disabled", base_timeout_s=90.0)
        _, t_en, _ = resolve_agent_thinking("qwen3.8-flash", "enabled", base_timeout_s=90.0)
        self.assertGreaterEqual(t_dis, 180.0)
        self.assertGreaterEqual(t_en, 180.0)

    def test_agent_layer_unidirectional_dependency_guard(self) -> None:
        """Enforce top-level agent/ boundary including package aliases and relative imports."""
        import ast

        def _extract_targets(tree: ast.AST, file_mod_parts: tuple[str, ...]) -> list[str]:
            targets: list[str] = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        targets.append(alias.name)
                elif isinstance(node, ast.ImportFrom):
                    if node.level > 0:
                        base_parts = file_mod_parts[: max(0, len(file_mod_parts) - node.level)]
                        base = ".".join(base_parts)
                        mod = f"{base}.{node.module}" if (base and node.module) else (node.module or base)
                    else:
                        mod = node.module or ""
                    if mod:
                        targets.append(mod)
                    for alias in node.names:
                        if alias.name != "*":
                            targets.append(f"{mod}.{alias.name}" if mod else alias.name)
            return targets

        # Self-test: verify package-alias and relative imports are detected
        sample_alias = _extract_targets(
            ast.parse("from embodied_navigation import agent"), ("embodied_navigation", "planning", "x")
        )
        self.assertIn("embodied_navigation.agent", sample_alias)
        sample_rel = _extract_targets(
            ast.parse("from ..agent import runtime"), ("embodied_navigation", "planning", "x")
        )
        self.assertIn("embodied_navigation.agent.runtime", sample_rel)

        pkg_root = Path(__file__).resolve().parents[2] / "src" / "embodied_navigation"
        lower_layers = (
            "common",
            "perception",
            "localization_mapping",
            "planning",
            "navigation",
            "robot",
            "simulation",
            "visualization",
        )
        lower_violations: list[str] = []
        for layer in lower_layers:
            ldir = pkg_root / layer
            if not ldir.is_dir():
                continue
            for py in sorted(ldir.rglob("*.py")):
                rel = py.relative_to(pkg_root)
                mod_parts = ("embodied_navigation", *rel.with_suffix("").parts)
                if mod_parts[-1] == "__init__":
                    mod_parts = mod_parts[:-1]
                tree = ast.parse(py.read_text(encoding="utf-8"), filename=str(py))
                for tgt in _extract_targets(tree, mod_parts):
                    if tgt == "embodied_navigation.agent" or tgt.startswith(
                        ("embodied_navigation.agent.", "embodied_navigation.agent_runtime")
                    ):
                        lower_violations.append(f"{rel} -> {tgt}")
        self.assertEqual(lower_violations, [], f"Lower layers imported agent/: {lower_violations}")

        # Also enforce pure agent/runtime/ never imports lower domain/visualization layers
        rt_dir = pkg_root / "agent" / "runtime"
        rt_violations: list[str] = []
        forbidden_rt = (
            "embodied_navigation.planning",
            "embodied_navigation.perception",
            "embodied_navigation.localization_mapping",
            "embodied_navigation.simulation",
            "embodied_navigation.navigation",
            "embodied_navigation.robot",
            "embodied_navigation.visualization",
        )
        for py in sorted(rt_dir.rglob("*.py")):
            rel = py.relative_to(pkg_root)
            mod_parts = ("embodied_navigation", *rel.with_suffix("").parts)
            if mod_parts[-1] == "__init__":
                mod_parts = mod_parts[:-1]
            tree = ast.parse(py.read_text(encoding="utf-8"), filename=str(py))
            for tgt in _extract_targets(tree, mod_parts):
                if tgt == forbidden_rt or tgt.startswith(forbidden_rt):
                    rt_violations.append(f"{rel} -> {tgt}")
        self.assertEqual(rt_violations, [], f"agent/runtime/ leaked domain imports: {rt_violations}")

    def test_llm_transport_timeout_failover_shared_circuit_and_text_strip(self) -> None:
        primary_hits = [0]
        fallback_hits = [0]
        fallback_bodies: list[dict] = []
        primary_mode = ["504"]

        class _PrimaryHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                _ = self.rfile.read(n)
                primary_hits[0] += 1
                code = 400 if primary_mode[0] == "400" else 504
                self.send_response(code)
                self.end_headers()

        class _FallbackHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(n).decode("utf-8"))
                fallback_bodies.append(body)
                fallback_hits[0] += 1
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                resp = {
                    "choices": [
                        {
                            "message": {
                                "content": "ok from fallback",
                                "reasoning_content": "thought",
                                "tool_calls": [
                                    {
                                        "id": "call_fb",
                                        "type": "function",
                                        "function": {"name": "navigate_to", "arguments": '{"target_id":"F1"}'},
                                    }
                                ],
                            }
                        }
                    ],
                    "usage": {"prompt_tokens": 120, "completion_tokens": 15},
                }
                self.wfile.write(json.dumps(resp).encode("utf-8"))

        srv1 = ThreadingHTTPServer(("127.0.0.1", 0), _PrimaryHandler)
        srv2 = ThreadingHTTPServer(("127.0.0.1", 0), _FallbackHandler)
        t1 = threading.Thread(target=srv1.serve_forever, daemon=True)
        t2 = threading.Thread(target=srv2.serve_forever, daemon=True)
        t1.start()
        t2.start()
        try:
            p_cfg = LlmClientConfig(
                provider="opencode",
                base_url=f"http://127.0.0.1:{srv1.server_address[1]}/v1",
                model="deepseek-v4.1-flash",
                api_key="k1",
                timeout_s=5.0,
                stream=False,
            )
            fb_cfg = LlmClientConfig(
                provider="deepseek_official",
                base_url=f"http://127.0.0.1:{srv2.server_address[1]}/v1",
                model="deepseek-flash",
                api_key="k2",
                timeout_s=5.0,
                supports_vision=False,
                stream=False,
            )
            cli = OpenCodeToolChatClient(p_cfg, fallback_configs=[fb_cfg], circuit_open_s=60.0)
            cloned = cli.clone_for_session()
            self.assertNotEqual(cli.config.session_id, cloned.config.session_id)

            msgs = [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "look"},
                        {"type": "image_url", "image_url": {"url": "data:image/png;base64,AAAA"}},
                    ],
                }
            ]
            tools = [ToolSpec("navigate_to", "nav", {"type": "object", "properties": {}})]

            # 1) Primary 504 -> fails over to fallback, strips image_url for text-only fallback
            res1 = cli.complete_turn(msgs, tools)
            self.assertFalse(res1.fallback)
            self.assertEqual(res1.provider_used, "deepseek_official")
            self.assertIn("http_504", res1.fallback_switched_from)
            self.assertEqual(cli.fallback_switches, 1)
            self.assertEqual(primary_hits[0], 1)
            self.assertEqual(fallback_hits[0], 1)
            self.assertIsInstance(fallback_bodies[0]["messages"][0]["content"], str)
            self.assertIn("image(s) omitted", fallback_bodies[0]["messages"][0]["content"])

            # 2) Cloned concurrent session immediately honors shared circuit cooldown (0 extra primary hits!)
            res2 = cloned.complete_turn(msgs, tools)
            self.assertFalse(res2.fallback)
            self.assertIn("circuit_open", res2.fallback_switched_from)
            self.assertEqual(primary_hits[0], 1)
            self.assertEqual(fallback_hits[0], 2)

            # 3) Non-retryable http_400 must NOT fail over (preserves runtime compaction path)
            primary_mode[0] = "400"
            cli_400 = OpenCodeToolChatClient(p_cfg, fallback_configs=[fb_cfg], circuit_open_s=0.0)
            res3 = cli_400.complete_turn(msgs, tools)
            self.assertTrue(res3.fallback)
            self.assertIn("http_400", res3.error_reason)
            self.assertEqual(fallback_hits[0], 2)
        finally:
            srv1.shutdown()
            srv2.shutdown()
            srv1.server_close()
            srv2.server_close()

    def test_shared_habitat_scene_host_multiplexes_isolated_concurrent_sessions(self) -> None:
        from concurrent.futures import ThreadPoolExecutor
        from embodied_navigation.agent.adapters import SharedHabitatSceneHost

        owner_tid: list[int] = []
        cross_thread_violations: list[tuple[int, int]] = []

        class _State:
            def __init__(self, pos: np.ndarray, rot: float = 0.0) -> None:
                self.position = pos.copy()
                self.rotation = rot

        class _FakeAgent:
            def __init__(self) -> None:
                self._st = _State(np.zeros(3, dtype=np.float32), 0.0)

            def get_state(self) -> _State:
                tid = threading.get_ident()
                if owner_tid and tid != owner_tid[0]:
                    cross_thread_violations.append((owner_tid[0], tid))
                return _State(self._st.position, self._st.rotation)

            def set_state(self, st: _State) -> None:
                tid = threading.get_ident()
                if owner_tid and tid != owner_tid[0]:
                    cross_thread_violations.append((owner_tid[0], tid))
                self._st = _State(st.position, st.rotation)

        class _FakeSim:
            def __init__(self) -> None:
                owner_tid.append(threading.get_ident())
                self.agents = [_FakeAgent()]
                self.closed_on_owner = False

            def step(self, act: str) -> dict[str, np.ndarray]:
                tid = threading.get_ident()
                if tid != owner_tid[0]:
                    cross_thread_violations.append((owner_tid[0], tid))
                st = self.agents[0].get_state()
                if act == "move_forward":
                    st.position = st.position + np.array([0.25, 0.0, 0.0], dtype=np.float32)
                self.agents[0].set_state(st)
                return self.get_sensor_observations()

            def get_sensor_observations(self) -> dict[str, np.ndarray]:
                tid = threading.get_ident()
                if tid != owner_tid[0]:
                    cross_thread_violations.append((owner_tid[0], tid))
                val = int(round(float(self.agents[0].get_state().position[2])))
                return {"color_sensor": np.full((4, 4, 3), val, dtype=np.uint8)}

            def close(self) -> None:
                self.closed_on_owner = threading.get_ident() == owner_tid[0]

        sim_ref: list[_FakeSim] = []

        def _factory() -> _FakeSim:
            s = _FakeSim()
            sim_ref.append(s)
            return s

        with SharedHabitatSceneHost(sim_factory=_factory, precompute_gt=False) as host:
            def _worker(idx: int) -> tuple[int, float, int]:
                proxy = host.create_session(
                    f"sess_{idx}",
                    position=np.array([0.0, 0.0, float(idx * 10)], dtype=np.float32),
                    rotation=float(idx),
                )
                steps = 4 + idx * 3
                for _ in range(steps):
                    obs = proxy.step("move_forward")
                    self.assertEqual(int(obs["color_sensor"][0, 0, 0]), idx * 10)
                st_end = proxy.agents[0].get_state()
                host.drop_session(f"sess_{idx}")
                return idx, float(st_end.position[0]), int(round(float(st_end.position[2])))

            with ThreadPoolExecutor(max_workers=4) as pool:
                outs = list(pool.map(_worker, [1, 2, 3, 4]))

        self.assertEqual(cross_thread_violations, [])
        self.assertTrue(sim_ref[0].closed_on_owner)
        for idx, end_x, end_z in outs:
            self.assertAlmostEqual(end_x, (4 + idx * 3) * 0.25, places=4)
            self.assertEqual(end_z, idx * 10)

    def test_failover_chain_stops_on_non_retryable_fallback_and_headers(self) -> None:
        """504 -> 400 must stop before 3rd route; session header only sent to opencode; top-level cache hit parsed."""
        hits = {"r1": 0, "r2": 0, "r3": 0}
        seen_headers: dict[str, str | None] = {}
        r2_mode = ["400"]

        def _make_handler(tag: str):
            class _H(BaseHTTPRequestHandler):
                def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                    return

                def do_POST(self) -> None:
                    n = int(self.headers.get("Content-Length", "0"))
                    _ = self.rfile.read(n)
                    hits[tag] += 1
                    seen_headers[tag] = self.headers.get("x-opencode-session")
                    if tag == "r1":
                        self.send_response(504)
                        self.end_headers()
                    elif tag == "r2" and r2_mode[0] == "400":
                        self.send_response(400)
                        self.end_headers()
                        self.wfile.write(b'{"error":{"message":"invalid parameter"}}')
                    else:
                        self.send_response(200)
                        self.send_header("Content-Type", "application/json")
                        self.end_headers()
                        body = {
                            "choices": [{"finish_reason": "stop", "message": {"content": "ok"}}],
                            "usage": {
                                "prompt_tokens": 500,
                                "prompt_cache_hit_tokens": 420,
                                "completion_tokens": 12,
                            },
                        }
                        self.wfile.write(json.dumps(body).encode("utf-8"))

            return _H

        s1 = ThreadingHTTPServer(("127.0.0.1", 0), _make_handler("r1"))
        s2 = ThreadingHTTPServer(("127.0.0.1", 0), _make_handler("r2"))
        s3 = ThreadingHTTPServer(("127.0.0.1", 0), _make_handler("r3"))
        for s in (s1, s2, s3):
            threading.Thread(target=s.serve_forever, daemon=True).start()
        try:
            c1 = LlmClientConfig(
                provider="opencode",
                base_url=f"http://127.0.0.1:{s1.server_address[1]}/v1",
                model="deepseek-v4.1-flash",
                api_key="k1",
                timeout_s=5.0,
                stream=False,
            )
            c2 = LlmClientConfig(
                provider="deepseek_official",
                base_url=f"http://127.0.0.1:{s2.server_address[1]}/v1",
                model="deepseek-flash",
                api_key="k2",
                session_id="should-not-be-sent",
                timeout_s=5.0,
                stream=False,
            )
            c3 = LlmClientConfig(
                provider="deepseek_official",
                base_url=f"http://127.0.0.1:{s3.server_address[1]}/v1",
                model="deepseek-v4-pro",
                api_key="k3",
                timeout_s=5.0,
                stream=False,
            )
            cli = OpenCodeToolChatClient(c1, fallback_configs=[c2, c3], circuit_open_s=0.0)
            res = cli.complete_turn([{"role": "user", "content": "hi"}], [])
            self.assertTrue(res.fallback)
            self.assertIn("http_400", res.error_reason)
            self.assertEqual(hits["r1"], 1)
            self.assertEqual(hits["r2"], 1)
            self.assertEqual(hits["r3"], 0)
            self.assertIsNotNone(seen_headers["r1"])
            self.assertIsNone(seen_headers["r2"])

            # Now make r2 succeed and verify top-level prompt_cache_hit_tokens is parsed
            r2_mode[0] = "200"
            res_ok = cli.complete_turn([{"role": "user", "content": "hi"}], [])
            self.assertFalse(res_ok.fallback)
            self.assertEqual(res_ok.usage.cached_tokens, 420)
            self.assertEqual(res_ok.usage.prompt_tokens, 500)
        finally:
            for s in (s1, s2, s3):
                s.shutdown()
                s.server_close()

    def test_circuit_breaker_half_open_single_probe_concurrency(self) -> None:
        """When cooldown expires, ONLY ONE concurrent worker probes primary while others use fallback."""
        from concurrent.futures import ThreadPoolExecutor

        primary_mode = ["503"]
        primary_probe_hits = [0]
        probe_entered = threading.Event()
        release_probe = threading.Event()

        class _PHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                _ = self.rfile.read(n)
                if primary_mode[0] == "503":
                    self.send_response(503)
                    self.end_headers()
                    return
                primary_probe_hits[0] += 1
                probe_entered.set()
                release_probe.wait(timeout=2.0)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(
                    json.dumps(
                        {"choices": [{"finish_reason": "stop", "message": {"content": "primary_ok"}}]}
                    ).encode("utf-8")
                )

        class _FHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                _ = self.rfile.read(n)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(
                    json.dumps(
                        {"choices": [{"finish_reason": "stop", "message": {"content": "fb_ok"}}]}
                    ).encode("utf-8")
                )

        s1 = ThreadingHTTPServer(("127.0.0.1", 0), _PHandler)
        s2 = ThreadingHTTPServer(("127.0.0.1", 0), _FHandler)
        threading.Thread(target=s1.serve_forever, daemon=True).start()
        threading.Thread(target=s2.serve_forever, daemon=True).start()
        try:
            c1 = LlmClientConfig(
                provider="opencode",
                base_url=f"http://127.0.0.1:{s1.server_address[1]}/v1",
                model="deepseek-v4.1-flash",
                api_key="k1",
                timeout_s=5.0,
                stream=False,
            )
            c2 = LlmClientConfig(
                provider="deepseek_official",
                base_url=f"http://127.0.0.1:{s2.server_address[1]}/v1",
                model="deepseek-flash",
                api_key="k2",
                timeout_s=5.0,
                stream=False,
            )
            base_cli = OpenCodeToolChatClient(c1, fallback_configs=[c2], circuit_open_s=0.06)
            # Trip circuit to open
            r0 = base_cli.complete_turn([{"role": "user", "content": "trip"}], [])
            self.assertEqual(r0.provider_used, "deepseek_official")

            # Wait for cooldown expiry so next concurrent wave enters half_open
            time.sleep(0.08)
            primary_mode[0] = "200"
            clones = [base_cli.clone_for_session() for _ in range(3)]
            barrier = threading.Barrier(3)

            def _call(cl: OpenCodeToolChatClient) -> str:
                barrier.wait(timeout=2.0)
                turn = cl.complete_turn([{"role": "user", "content": "probe"}], [])
                return turn.provider_used

            with ThreadPoolExecutor(max_workers=3) as pool:
                futs = [pool.submit(_call, cl) for cl in clones]
                self.assertTrue(probe_entered.wait(timeout=2.0))
                # Give other 2 workers a moment to observe half_open and route to fallback
                time.sleep(0.05)
                self.assertEqual(primary_probe_hits[0], 1)
                release_probe.set()
                providers = [f.result(timeout=3.0) for f in futs]

            self.assertEqual(providers.count("opencode"), 1)
            self.assertEqual(providers.count("deepseek_official"), 2)
            # Circuit is now closed again; subsequent call hits primary
            r_after = base_cli.complete_turn([{"role": "user", "content": "after"}], [])
            self.assertEqual(r_after.provider_used, "opencode")
        finally:
            release_probe.set()
            s1.shutdown()
            s2.shutdown()
            s1.server_close()
            s2.server_close()

    def test_sse_content_filter_and_corrupted_stream_discard_partial_tool_calls(self) -> None:
        """Partial tool_calls followed by content_filter or malformed SSE JSON must NEVER execute."""
        sse_mode = ["content_filter"]

        class _SSEHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                _ = self.rfile.read(n)
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.end_headers()
                chunk1 = {
                    "choices": [
                        {
                            "delta": {
                                "tool_calls": [
                                    {
                                        "index": 0,
                                        "id": "call_partial",
                                        "function": {"name": "danger_tool", "arguments": '{"x":1}'},
                                    }
                                ]
                            },
                            "finish_reason": None,
                        }
                    ]
                }
                self.wfile.write(f"data: {json.dumps(chunk1)}\n\n".encode("utf-8"))
                if sse_mode[0] == "content_filter":
                    chunk2 = {"choices": [{"delta": {}, "finish_reason": "content_filter"}]}
                    self.wfile.write(f"data: {json.dumps(chunk2)}\n\n".encode("utf-8"))
                else:
                    self.wfile.write(b"data: {corrupted-json-frame\n\n")
                self.wfile.write(b"data: [DONE]\n\n")

        srv = ThreadingHTTPServer(("127.0.0.1", 0), _SSEHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            cfg = LlmClientConfig(
                provider="deepseek_official",
                base_url=f"http://127.0.0.1:{srv.server_address[1]}/v1",
                model="deepseek-flash",
                api_key="k",
                timeout_s=5.0,
                stream=True,
            )
            cli = OpenCodeToolChatClient(cfg)
            executed = [0]

            for mode_name, expected_err in (
                ("content_filter", "content_filter"),
                ("corrupt", "corrupted_sse_stream"),
            ):
                sse_mode[0] = mode_name
                ctx = PrefixStableContextManager(system_prompt="sys", initial_task_prompt="task")
                rt = EmbodiedAgentRuntime(chat_client=cli, context=ctx)
                rt.register_tool(
                    ToolSpec("danger_tool", "test", {"type": "object", "properties": {}}),
                    lambda _args: (executed.__setitem__(0, executed[0] + 1) or {"ok": True}),
                )
                res = rt.run_task(task_label="test", initial_observation_text="obs", max_turns=4)
                self.assertEqual(executed[0], 0, f"Tool executed despite {mode_name}!")
                self.assertIn(expected_err, res.termination_reason)
        finally:
            srv.shutdown()
            srv.server_close()

    def test_shared_habitat_scene_host_safety_guards_and_plot_cleanup(self) -> None:
        """Verify stale proxy rejection, duplicate ID rejection, re-entrancy guard, partial step persistence, dir traversal rejection, and matplotlib figure cleanup."""
        from embodied_navigation.agent.adapters import (
            SharedHabitatSceneHost,
            run_concurrent_scene_sessions,
        )
        from embodied_navigation.planning.exploration.habitat_memory_runner import (
            DepthOccupancyGrid,
            HabitatSubtaskSpec,
        )

        class _St:
            def __init__(self, pos: np.ndarray, rot: float = 0.0) -> None:
                self.position = pos.copy()
                self.rotation = rot

        class _Ag:
            def __init__(self) -> None:
                self._st = _St(np.zeros(3, dtype=np.float32), 0.0)

            def get_state(self) -> _St:
                return _St(self._st.position, self._st.rotation)

            def set_state(self, st: _St) -> None:
                self._st = _St(st.position, st.rotation)

        class _Sim:
            def __init__(self) -> None:
                self.agents = [_Ag()]

            def step(self, act: str) -> dict[str, np.ndarray]:
                st = self.agents[0].get_state()
                if act == "move_forward":
                    st.position = st.position + np.array([0.25, 0.0, 0.0], dtype=np.float32)
                self.agents[0].set_state(st)
                return {"color_sensor": np.zeros((2, 2, 3), dtype=np.uint8)}

            def close(self) -> None:
                return

        with SharedHabitatSceneHost(sim_factory=_Sim, precompute_gt=False) as host:
            p_a = host.create_session("A", position=np.array([1.0, 0.0, 0.0], dtype=np.float32))
            p_b = host.create_session("B", position=np.array([9.0, 0.0, 0.0], dtype=np.float32))

            # Duplicate active ID rejected
            with self.assertRaises(ValueError):
                host.create_session("A")

            # Re-entrant RPC from owner thread raises RuntimeError instead of deadlocking
            with self.assertRaises(RuntimeError):
                p_a.run_atomic(lambda _sim: host.execute_raw(lambda _s: 1))

            # Drop A, move B, then call stale p_a -> must raise and NOT corrupt B!
            p_a.drop()
            p_b.step("move_forward")
            with self.assertRaises(RuntimeError):
                p_a.agents[0].get_state()

            # Re-registering "A" gives a new generation; old p_a MUST still be rejected!
            p_a2 = host.create_session("A", position=np.array([3.0, 0.0, 0.0], dtype=np.float32))
            with self.assertRaises(RuntimeError):
                p_a.step("move_forward")
            self.assertAlmostEqual(float(p_a2.agents[0].get_state().position[0]), 3.0, places=4)
            self.assertAlmostEqual(float(p_b.agents[0].get_state().position[0]), 9.25, places=4)

            # Mid-RPC failure after 1 step still persists moved pose into B's slot
            def _step_then_fail(sim: Any) -> None:
                sim.step("move_forward")
                raise RuntimeError("injected_mid_batch_error")

            with self.assertRaises(RuntimeError):
                p_b.run_atomic(_step_then_fail)
            self.assertAlmostEqual(float(p_b.agents[0].get_state().position[0]), 9.50, places=4)

        # Path traversal & duplicate session directory validation BEFORE starting sim/threads
        with tempfile.TemporaryDirectory() as td:
            sub = [HabitatSubtaskSpec(subtask_id="g1", target_category="chair", instruction="find chair")]
            for bad_specs in (
                [("../escape", sub, 1)],
                [("/tmp/abs_escape", sub, 1)],
                [("dup", sub, 1), ("dup", sub, 2)],
            ):
                with self.assertRaises(ValueError):
                    run_concurrent_scene_sessions(
                        scene_dataset_config="unused",
                        scene_id="unused",
                        output_root=td,
                        session_specs=bad_specs,
                        chat_client=object(),
                    )

        # Matplotlib figure cleanup on save failure
        import matplotlib.pyplot as plt

        grid = DepthOccupancyGrid()
        before_figs = len(plt.get_fignums())
        with tempfile.TemporaryDirectory() as td:
            bad_out = Path(td) / "dir_as_file.png"
            bad_out.mkdir()
            with self.assertRaises(Exception):
                grid.render_local_map(bad_out, (0.0, 0.0, 0.0), 0.0, [])
        self.assertEqual(len(plt.get_fignums()), before_figs)


if __name__ == "__main__":
    unittest.main()
