# 2026-11-11 探索工具原件

这是从 `/home/hyh001/Documents/Codex/2026-11-11/task` 找回的 20 个 Python 工具与 8 个小型 JSON 配置，逐字节保留，不是重新编写的探索器。来源、SHA256、实际调用和使用方法见 [工具说明](../../../docs/experiments/dynanav-exploration-tools-20261111.md)。

- 根目录：当时的状态/快照脚本及按 run 绑定的观察入口。
- `coverage_audit/`：SOP 整理前的实测 LIO collector 与 3D 渲染原件。当前可复用版本已在 `.agents/skills/dynanav-explore/`，不应以历史 collector 覆盖它。
- `coverage_111150/`：最后 32 航点代的全域、ROI、上层相对高度与下层阶段渲染，以及这代的显示配置/缺口 ledger。
- `repaired_coverage/`：较早 102621 代的覆盖渲染和最终整理工具，保留 run 边界。

**不要把历史脚本当作开箱即用的当前配置。** 原文件的绝对路径和默认 run 保留了下来；部分脚本在顶层直接读写文件，连 `--help` 也不会阻止这些动作。`render_final_lower_stage.py` 和 `build_final.py` 没有无动作 CLI。根目录观察入口依赖原归档位置；它们用于溯源，当前操作优先用已归档 skill。

只有带 main 保护的 `coverage_111150/render_generation.py`、`render_observed_roi.py`、`render_upper_region.py` 可先运行 `--help`。重放时创建外部 binding，显式指定同代输入和新的外部输出，不覆盖原证据，不在此目录生成图、点云或日志。
