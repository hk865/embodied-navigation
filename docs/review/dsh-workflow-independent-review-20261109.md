# DSH 流程修复：独立核查（2026-11-09）

DSH 已实际实施修改，Codex 独立复核后退回两轮修正。当前离线检查通过，已复现的问题均已修复；这不是恢复仿真或真实在线编排验收。

## 核查基线与归属

- 基线为分支 `improve/module-docs-readability`、HEAD `43cba338e83220401047139c09045034e6a8ebc1` **加本轮开始前的完整工作树**，不是只与 HEAD 比较。开始前已有 29 项修改/未跟踪文件。
- UTC 08:00:03 保存了 802 个文件的内容快照与 SHA 清单，保存在本次 Codex 工作目录 `dsh-workflow-fixes-baseline-20261109T080003Z/`；据此区分旧在线感知成果和 DSH 新改动。未发现基线文件被删除。
- DSH 会话 `mailbox-2026-11-09T07-25-26-735Z-81d359de`：turn 3 实施，turn 4 修独立核查项，turn 5 修同时退出的成功误判；最终状态为 `completed`。
- 代码实施由 DSH 完成。Codex 最终补正配置读取说明、测试依赖与报告摘要，并处理三项静态检查：显式 `check=False`（仍由原逻辑检查返回码）、删除未使用导入、合并等价的测试 context manager。未改断言、控制参数或验收阈值。

实施清单与暂缓理由见 [DSH 修复记录](dsh-workflow-fixes-20261109.md)；流程背景见 [流程导读](dsh-workflow-and-readability-20261109.md)。

## 实际核查与退回修正

| 项目 | 独立发现/验证 | 最终状态 |
| --- | --- | --- |
| 在线编排的超时/中断 | 第一版只停 worker/observer，漏停 capture；假进程可复现 | 逆序回收 capture、observer、worker；SIGINT/SIGTERM、日志关闭、信号 handler 恢复通过独立检查 |
| 编排成功判定 | capture=0、observer=7 或 worker=9 时曾误报 `ok=true` | 两例均返回失败；正常完成后的主动收尾仍成功；收据 v3 记录 `unexpected_exits` |
| 只读检查 | 缺少参考路径时曾 `Path(None)` 抛 TypeError | `--check` 返回 2 并报告缺项，不创建输出目录；打印模式保留明确占位 |
| 数据与 ROS 环境 | `--data-root` 未传入子进程；实际启动与打印命令的 ROS 环境不一致 | 数据根传递、显式 ROS setup、启动日志及命令父目录准备已补齐；用 fake setup/子进程检查 |
| 查询提取与兼容 | 新包函数与旧函数 AST 相同；10 个函数样本、4 个 CLI 样本的输出/错误/退出码一致 | 旧 CLI 与兼容导出保留；在线/离线调用直接导入库 |
| 来源记录 | 提取后的 query 实现漏进源码快照与重建哈希 | `experiment.sources` 和离线 `provenance.source_files` 均保留旧 CLI、加入实际新实现 |
| capture 入口 | 导入守卫是否破坏 `runpy.__main__`、是否引入 SDK 副作用 | 普通 import 立即抛明确错误且无目录/进程副作用；fake SDK 比较保留原入口及异常收据行为 |
| Native 与既有感知 | 核对状态机、wrapper、配置覆盖及旧未提交 helper | 相关 Native 执行 AST 未改；六个 pin 生效值不变；person worker 执行逻辑未改；已抽查旧 helper/tests 保留 |
| 文档事实 | 默认状态机、worker 所有者、配置冻结范围、归档与回放、IMU 安装偏移等说明有误 | 已按实际消费者更正；禁用观察器 hook 不再被扩大成整个 wrapper 是空操作 |

## 实际检查结果

均为离线 CPU 检查；没有启动 ROS 节点、Gazebo、Isaac、真实模型、GPU 推理、训练或机器人。进程生命周期测试使用临时 Python fake 子进程。

| 检查 | Codex 独立结果 |
| --- | --- |
| 完整 unit | `Ran 311 tests in 6.967s`，`OK (skipped=3)` |
| 完整 regression | `Ran 86 tests in 7.261s`，`OK (skipped=5)` |
| 最终静态收尾后的 supervisor 子集 | 17 项通过；属于上面的 unit，不另加总 |
| 独立 supervisor 探针 | 正常完成、超时、两类同时退出失败、SIGINT、SIGTERM、缺项只读检查均符合预期 |
| 新增实现/测试 Ruff | supervisor、query、三份新增测试文件通过 |
| 语法与差分 | 18 个变更 Python 文件 AST 解析、`bash -n scripts/run_dynanav_lio.sh`、`git diff --check` 通过 |
| 文档链接 | 收尾前变更文档的 150 条相对文件链接均存在；不代表核查了外部网络链接 |

本机 unit 的 3 项跳过需要 USD 运行时/本地人形资产；regression 的 5 项跳过均因可选 CPU Torch 缺失。其它机器还可能因私有测量资产或编译工具缺失而跳过不同用例，不能据此表宣称这些可选路径已通过。

完整套件在最终三项等价静态收尾前执行，随后重跑受影响的 supervisor 17 项并通过 Ruff；没有把历史仿真结果当成本轮验证。

## 范围与剩余工作

- 新 [统一编排入口](../../scripts/run_dynanav_online_person.py) 的真实 ROS/Isaac/模型组合尚未运行。离线监督测试通过，不能据此宣称在线路线、性能或感知质量改善。
- `pid_navigation.py` 的大文件拆分、capture 整体 `main()` 化、策略 232/247 维诊断说明、部分历史配置清理等仍在 DSH 记录中列为暂缓，未把导读报告全部候选当成已修逻辑 bug。
- 本轮没有修改导航状态机、速度/安全阈值或此前实验指标。仿真、GPU 推理、训练仍暂停；源码与文档保留在工作树，未提交或推送。
- 已确认 DSH 无运行中的 mailbox 任务，并仅停止本任务临时启动的 host（PID 3541879），恢复启动前未运行状态；没有修改 DSH/API/网络配置或其它服务。
