# 具身导航 Agent 架构教程：Skill、Tool、实时视频流控制与记忆系统

> **整理日期**：2026-11-23  
> **对应代码基线**：
> - 主线探索工具与 Skill：`.agents/skills/dynanav-explore/`、`scripts/explore_waypoint_client.py`、`scripts/dynanav_exploration.py`、`src/embodied_navigation/planning/exploration/`
> - 流式哨兵与可抢占控制分支：`feat/agent-preemptable-active-slam`（`8aebc95`）
> - 关键帧锚定 3D 语义场景图分支：`feat/semantic-keyframe-graph`（`46e0b48`）
> - 高精度在线 SLAM 后端分支：`exp/slam-v4-oriented-planar-ba`（`639e9dc`）
> - 设计决策参考：[ADR-0002](../plan/adr/0002-embodied-agent-async-and-map-outlook.md)、[ADR-0003](../plan/adr/0003-streaming-fork-control-and-semantic-scene-graph.md)、[ADR-0004](../plan/adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)
>
> **阅读说明**：本目录与 [docs/slam-tutorial/](../slam-tutorial/README.md) 互补——`slam-tutorial` 聚焦「机器人如何把位姿与 3D 地图算准」，本目录聚焦「大模型 Agent 如何通过 Skill 与 Tool 感知环境、安全控制机器人，以及未来如何演进到实时视频流控制与长程空间记忆系统」。

## 推荐阅读顺序

| 序号 | 文档 | 核心内容 |
| --- | --- | --- |
| 01 | [01_skills_tools_and_control_pipeline.md](01_skills_tools_and_control_pipeline.md) | 当下具身 Agent 的 Skill 设计规范、全部 CLI Tool 与观测产物详解、原子 Mailbox 协议、`goto / scan / maneuver / cancel / stop` 控制闭环源码与伪代码、以及当前回合制架构的 4 大瓶颈 |
| 02 | [02_streaming_video_control_and_memory_system.md](02_streaming_video_control_and_memory_system.md) | System 0/1/2 三层异步流式视频控制（`action_epoch` 抢占 + 阅后即焚 SSE 哨兵 + 连续视觉流演进路线），以及面向「探索 / 问答召回 / 语义导航 / 跨时刻巡检找不同」的三层空间记忆系统与 3D 场景图架构 |
