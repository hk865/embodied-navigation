# 具身导航 Agent 架构教程：Skill、Tool、实时视频流控制与记忆系统

> **整理日期**：2026-11-25
> **对应代码基线**：
> - 静态记忆与流式全栈集成主分支：`feat/static-memory-streaming`（合流 V4 在线平面 BA + 3D 场景图 + 异步可抢占哨兵 + OpenCode Go 流式 + 修复版 30 题公平记忆消融基准，904 单测 / 60 回归全绿）
> - 主线探索工具与 Skill：`.agents/skills/dynanav-explore/`、`scripts/explore_waypoint_client.py`、`scripts/run_static_streaming_task.py`、`scripts/eval_static_task_suite.py`、`src/embodied_navigation/planning/exploration/`
> - 层次空间记忆与场景图：`src/embodied_navigation/localization_mapping/semantic_mapping/`（`scene_graph.py`、`hierarchical_memory.py`、`backend_adapter.py`、`object_mesh_memory.py`）
> - 历史专题分支：`feat/agent-preemptable-active-slam`（`8aebc95`）、`feat/semantic-keyframe-graph`（`46e0b48`）、`exp/slam-v4-oriented-planar-ba`（`6da00f3`）
> - 设计决策参考：[ADR-0002](../plan/adr/0002-embodied-agent-async-and-map-outlook.md)、[ADR-0003](../plan/adr/0003-streaming-fork-control-and-semantic-scene-graph.md)、[ADR-0004](../plan/adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)
>
> **阅读说明**：本目录与 [docs/slam-tutorial/](../slam-tutorial/README.md) 互补——`slam-tutorial` 聚焦「机器人如何把位姿与 3D 地图算准」，本目录聚焦「大模型 Agent 如何通过 Skill 与 Tool 感知环境、安全控制机器人，以及如何通过层次空间记忆、六分区上下文与三层流式控制完成长程具身任务」。

## 推荐阅读顺序

| 序号 | 文档 | 核心内容 |
| --- | --- | --- |
| 01 | [01_skills_tools_and_control_pipeline.md](01_skills_tools_and_control_pipeline.md) | 当下具身 Agent 的 Skill 设计规范、全部 CLI Tool 与观测产物详解、原子 Mailbox 协议、`goto / scan / maneuver / cancel / stop` 控制闭环源码与伪代码、以及当前回合制架构的 4 大瓶颈 |
| 02 | [02_streaming_video_control_and_memory_system.md](02_streaming_video_control_and_memory_system.md) | System 0/1/2 三层异步流式视频控制（`action_epoch` 抢占 + 阅后即焚 SSE 哨兵 + 连续视觉流演进路线），以及面向「探索 / 问答召回 / 语义导航 / 跨时刻巡检找不同」的三层空间记忆系统与 3D 场景图架构 |
| 03 | [03_hierarchical_memory_6zone_context_and_eval.md](03_hierarchical_memory_6zone_context_and_eval.md) | **最新落地实战**：L0–L3 版本化层次空间记忆与语义精确匹配、六分区上下文解耦与一段话记忆总结、OpenCode Go 双主力模型（`qwen3.8-flash` 视觉 / `deepseek-v4.1-flash` high 思考）SSE 闭环验收、修复版 30 题公平消融实测表，以及 `0.2 s` 流式延迟容量预算 |
