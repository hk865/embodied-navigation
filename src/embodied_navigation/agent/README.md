# agent · 顶层具身 Agent 编排层（Layer 7）

> 盘点日期：2026-11-27 · 分支 `feat/static-memory-streaming`

本包是仓库的**顶层独立 Agent 层**，遵循**单向向下依赖**原则：仅向上对接 CLI 入口，单向向下调用 `common/`、`perception/`、`localization_mapping/`、`planning/` 等能力层；底层 6 层严禁反向导入 `embodied_navigation.agent`（由 `tests/unit/test_agent_runtime.py::test_agent_layer_unidirectional_dependency_guard` 机器校验）。

## 目录分层与职责边界

```text
src/embodied_navigation/agent/
├── README.md                     # 本文件
├── __init__.py                   # 顶层统一导出契约
├── runtime/                      # 纯任务无关、环境无关的多轮执行内核（禁止 import planning/perception/simulation）
│   ├── __init__.py
│   ├── types.py                  # ImageAttachment、ToolSpec/Call/Result、TurnUsage、AssistantTurn、FinishCheckResult
│   ├── hooks.py                  # BeforeTurnHook、AfterToolHook（剥离宿主路径键 + 裁剪）、BeforeFinishHook
│   ├── llm_transport.py          # OpenCodeToolChatClient（SSE 流式、180s 超时下限、截断检测、思考模式适配）
│   ├── context.py                # PrefixStableContextManager（冻结前缀 + Append-Only + 155K/200K 惰性压缩）
│   └── runtime.py                # EmbodiedAgentRuntime（纯 500 原子步预算驱动 + 零位移死循环熔断 + 工具调度）
├── prompts/                      # 系统提示词与事件驱动 VLM 区域摘要器
│   ├── __init__.py
│   └── embodied_prompts.py       # AGENT_RUNTIME_SYSTEM_PROMPT、build_vlm_region_summarizer
├── tools/                        # 通用具身工具集与可插拔终止验证门
│   ├── __init__.py
│   ├── embodied_tools.py         # build_agent_tools（navigate_to / traverse_stairs / scan / maneuver / inspect_vision / query_world / get_evidence / update_belief / finish_task）
│   └── verifiers.py              # ObjectNavFinishVerifier（易混淆类一票否决 + table 低分门控）、ReportFinishVerifier
└── adapters/                     # 环境适配器（将底层仿真/机器人栈接入统一 WorldDaemon 接口）
    ├── __init__.py
    ├── habitat_daemon.py         # HabitatWorldDaemon（自更新感知、Set-Cover 纹理帧、几何+语义区域记忆、2.5D 平面图）
    └── habitat_runner.py         # HabitatAgentRunner（HM3D 顺序/独立闭环评测、跨层楼梯通行、零 GT 泄露评分）
```

## 关键架构约定

1. **反向依赖彻底清零**：共享的 `LlmClientConfig` 与安全图像编码 `encode_image_part` 已下沉至 `common/llm_config.py`；`agent/runtime/` 仅依赖标准库与 `common/llm_config.py`，不再导入任何规划、感知或仿真模块。
2. **原子动作预算驱动**：`EmbodiedAgentRuntime.run_task(max_turns=None)` 默认不设人为的 25 轮上限，严格以环境侧的物理原子动作预算（标准 HM3D 为 500 原子步）终止，并配连续零位移死循环熔断保护。
3. **跨环境扩展口**：目前 `adapters/` 已接入 Habitat-Sim HM3D 评测支线（GPS+Compass 协议，不含 SLAM）；后续接入 Isaac Sim / DynaNav（D 路径真实 SLAM + PCT/SCAN）或实机只需新增 `adapters/dynanav_adapter.py`，无需改动 `runtime/`、`prompts/` 或 `tools/`。
4. **单元测试与架构守卫**：[`tests/unit/test_agent_runtime.py`](../../../tests/unit/test_agent_runtime.py) 覆盖前缀冻结与压缩、SSE 截断检测、500 原子步预算、楼梯门户检测、易混淆类拒停以及全仓 AST 依赖方向检查。
