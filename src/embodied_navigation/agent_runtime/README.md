# agent_runtime · 通用具身 Agent 运行时（Layer 7）

与具体任务、具体仿真器彻底解耦的通用多轮 Tool-Calling 运行时：

- **上下文与缓存工程（`context.py`）**：严格 Prefix-Stable 追加式消息历史（System + Tool Schemas + Goal 冻结于前缀，仅尾部追加新轮次），支持 `200K` 上下文软阈值惰性压缩（Compaction），并强制保留 DeepSeek `reasoning_content` 与工具调用原子组。
- **流式传输与推理捕获（`llm_transport.py`）**：基于 SSE 的 OpenAI 原生 `tools / tool_calls` 客户端，完整捕获 `reasoning_content`、`cached_tokens` 命中率与实时耗时。
- **生命周期 Hooks（`hooks.py`）**：只负责 Agent 自身认知/工具边界（首尾增量注入、工具回执裁剪与宿主路径剥离、可插拔任务完成验证器 `BeforeFinishHook`），绝不与异步感知建图守护进程耦合。
- **环境适配**：Habitat-Sim 适配见 [habitat_agent_adapter.py](../planning/exploration/habitat_agent_adapter.py) 与薄入口 [run_habitat_agent_eval.py](../../../scripts/run_habitat_agent_eval.py)。
