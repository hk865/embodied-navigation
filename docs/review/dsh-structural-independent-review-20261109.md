# 原有代码结构重构：独立核查（2026-11-09）

上一轮主要整理入口、文档和新增编排，没有完成这两处原有大文件的结构改造。本轮由 DSH 实际拆分，Codex 对照重构前工作树核查，退回兼容和运行接线问题后再次验证。

## 实际代码变化

| 原文件 | 重构前 → 当前 | 职责拆分 |
| --- | --- | --- |
| `navigation/pid_navigation.py` | 4718 → 249 行 | 保留类组合、初始化和兼容入口；回调入口、准入、路线推进、控制发布、证据 helper、进程生命周期进入 7 个职责模块 |
| `simulation/dynanav/capture_go2_rgbd.py` | 1785 → 485 行 | 显式 `main/run` 编排；场景、物理、传感器、ROS、控制、帧处理、建图、策略、视频、收据进入 `capture/` 包 |

模块职责与入口见 [导航 README](../../src/embodied_navigation/navigation/README.md)、[capture README](../../src/embodied_navigation/simulation/dynanav/capture/README.md)。详细实施记录见 [DSH 报告](dsh-structural-refactor-20261109.md)。

- 核查基线：`improve/module-docs-readability`，HEAD `43cba338e83220401047139c09045034e6a8ebc1` **加当时全部未提交工作**；已另存 812 文件内容及哈希，目录名 `dsh-structural-refactor-baseline-20261109T091315Z`。没有以 HEAD 覆盖原工作树。
- DSH 会话 turn 6 完成主体拆分，turn 7 实施独立核查退回项，最终均为 `completed`。
- 控制公式、数值阈值、传感频率、位姿来源与动态滤点逻辑没有为这轮重构放宽或改写。普通导入 capture 现在成功且不启动采集；真实执行仍通过显式入口。

## 发现并修复的问题

1. **旧导航导入/脚本兼容遗漏**：恢复旧模块的 helper、生命周期工具和 `_main(shutdown_signals)`；直接脚本与 `-m` 入口均恢复。独立确认旧 helper 指向真实实现，不是空兼容函数。
2. **采集入口漏传 `mount`**：真实 `PhysicsStepper` 构造会报 TypeError；已补相机安装偏移，零步 fake 编排实际通过。
3. **首步故障丢失原异常**：运行上下文漏绑定 `art`，错误处理再抛 AttributeError，收据和清理随之中断。已补上下文，逐项尝试错误记录和已拥有资源的清理；secondary cleanup 错误单独记录。
4. **二次关闭异常遮蔽 primary**：app/视频/建图/归档/收据写入失败不再替换最初异常，继续尝试其余清理。
5. **最终审查补正**：Codex 额外复现 stdout 关闭造成 traceback 输出抛 BrokenPipeError、打断 app 清理；将该输出纳入同一保护并加入回归。另去掉重复 `art` 字段和未使用导入，补本次新包 README。主体重构及上述前四项均由 DSH 实施。

ContactMonitor 第一版拼写错误由 DSH 自查修正；真实 fake 接触回调与双实例隔离已复核。新增 11 个 capture 源模块均已进入 experiment 的源码快照/哈希清单。

## 独立验证结果

| 检查 | 结果 |
| --- | --- |
| 最终完整 unit | `Ran 323 tests in 7.912s`，`OK (skipped=3)` |
| 最终完整 regression | `Ran 86 tests in 7.303s`，`OK (skipped=5)` |
| PID 结构与行为 | 49 方法无遗漏/重复；14 个方法内 16 处父调用目标保持；15 个方法行为用例、7 组 helper 差分一致 |
| PID 兼容 | 27 个旧 helper、生命周期入口与签名通过；两种真实 argparse `--help` 路径在 stub 下退出 0、ROS init 次数 0 |
| capture 故障/状态 | 8 项独立 CPU fake 通过：正常导入、早期失败、节流与状态隔离、零步成功、首步异常、二次清理异常等；输出失败回归另包含在最终 unit 中 |
| 接线与测试迁移 | 入口可静态解析的 61 处真实 factory/method 签名绑定无不匹配；原 18 项相关测试及断言保留，定位到实际新实现 |
| 静态检查 | 重构模块未定义名称检查（Ruff F821/F822/F823）、Python AST 解析、shell 语法与 `git diff --check` 通过 |

本机 3 项 unit skip 需要 USD 运行时/本地人形资产，5 项 regression skip 均因可选 CPU Torch 缺失。它们没有被计为已执行通过。fake 进程/对象不启动真实 ROS 节点、SDK、模型或 GPU。

可复算资料在本次 Codex 工作目录：`pid-structural-differential/` 保存 helper/MRO/兼容对照；`capture-refactor-review-20261109T094841Z/` 保存独立 fake 脚本、源码冻结 SHA 和结果。最后仅 receipts 的输出保护及对应测试由 Codex 收尾，随后重跑两套完整检查。

## 实际边界

本轮完成上述两处既有代码的职责拆分和必要修复。`pid_support.py` 仍约 1030 行，准入与控制内部仍有长方法，类组合仍共享节点状态；这不代表全仓库已没有可读性问题。

真实 ROS/Isaac 时序、GPU 性能和导航轨迹没有在本轮重跑，仿真继续暂停。文件系统本身写入失败时只能保持原异常与内存记录，不能保证失败收据落盘。代码和文档留在工作树，未提交/推送；临时 DSH 主机已收尾，未改用户 API/网络配置。
