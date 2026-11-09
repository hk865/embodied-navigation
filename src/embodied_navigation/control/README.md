# control

本目录是控制侧的实现集合：单步跟踪控制律、停车阶段的测量与零速度契约、仿真安全门禁，以及供 launch 组装的进程入口；它不含任务状态机，也不生成参考路线。

## 边界

- 做：把 canonical SLAM 位姿、IMU、点云与 SCAN 轨迹换算成有界机体速度；维护停车窗口与零输入 ACK 契约；在仿真安全门禁上给出 fault/stop/reason；检查停车反馈来源。
- 不做：任务级状态机（默认主路径在 `../navigation/base_node.py` 的 `Navigation.control()`；mission 目标层在 `../navigation/mission/mission_runtime.py`；`../navigation/task.py` 只服务独立 `relative-slam-navigation` 入口，见 [navigation/README.md](../navigation/README.md)）、路径规划与准入（在 [planning/README.md](../planning/README.md)）、进程装配（见 [launch/README.md](launch/README.md)）。
- 不提供实机安全后端；`safety/` 包含仿真物理门禁，以及 SLAM 来源、时钟和 IMU 等运行保护，见 [safety/README.md](safety/README.md)。
- 本目录的 6 个文件里没有配置文件读取点，也不写业务日志；`configs/control/settings.json` 由 `tracking/` 子包消费，见下文与 [tracking/README.md](tracking/README.md)。

## 目录结构

```text
src/embodied_navigation/control/
├── launch/                 # 子模块，见 launch/README.md
├── parking/                # 子模块，见 parking/README.md
├── safety/                 # 子模块，见 safety/README.md
├── tracking/               # 子模块，见 tracking/README.md
├── README.md               # 本文件
├── __init__.py             # 1 行，只重导出 relative_pid 的 RelativePidConfig / follow_segment
├── collision.py            # 61 行，支撑栅格、制动扫掠、深度碰撞三个纯几何检查
├── executor_command.py     # 220 行，CommandReader：命令生产者、来源时钟与零命令绑定校验
├── io.py                   # 14 行，证据写入器与最新命令写入器的构造入口
├── publication_ledger.py   # 85 行，命令实际发布前后的只读审计记录
└── relative_pid.py         # 83 行，相对 SLAM 直线段的纯驾驶指令
```

实测合计：6 个 Python 文件 464 行（不含 4 个子目录）。`launch/`、`parking/`、`safety/`、`tracking/` 各自有 README，本文件不递归展开。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `relative_pid.py` | 活代码，纯函数；默认 `demo.launch.py` 不经过它，只服务独立 `relative-slam-navigation` 入口（`../navigation/task.py`）与 `scripts/replay_controller.py` |
| 代码 | `collision.py` | 活代码，纯几何；只被独立入口 `../navigation/relative_slam_node.py`、`../navigation/task.py` 使用 |
| 代码 | `executor_command.py` | 活代码；策略 worker 读命令文件时构造 `CommandReader`（`../robot/unitree/policy/worker.py` 内延迟导入） |
| 代码 | `publication_ledger.py` | 活代码；`../navigation/pid_control.py` 在每次实际 `cmd_pub.publish` 前后调用 `begin` / `publication_started` / `completed` |
| 代码 | `io.py` | 活代码接线层；`make_evidence_writer` 给 `../navigation/ros_node.py`，`make_command_writer` 给 `../robot/unitree/bridge.py` |
| 代码 | `__init__.py` | 兼容门面，只做两个符号重导出，无逻辑 |

**日志/输出**

- 本目录 6 个文件都没有直接写盘调用（`grep -n "write_text\|open("` 在 `control/*.py` 无命中）。
- `publication_ledger.PublicationLedger.completed` 会把一行 `schema=teacher_actual_command_publication/v1` 追加到 `runs/<run>/navigation_command_publications.jsonl`（字段含 `sequence`、`publish_ros_clock_ns`、`command_after`、`commands_published_count`、`protection_flags_after`、`navigation_ground_truth_used=False` 以及 `begin` 快照）。但它走 `node.evidence.append`，而默认 `io.make_evidence_writer` 返回 [common/runtime_telemetry.py](../common/runtime_telemetry.py) 的 `RuntimeTelemetry`，其 `append()` 是空操作——**默认运行不产生这个文件**，只有 `.atomic()` 才真正写盘。
- `io.make_command_writer` 返回 `LatestCommandWriter(writer, None)`；`history=None` 表示不写传输历史，`navigation_command.json` 本体由 bridge 侧传入的 writer 写，见 [robot/unitree/README.md](../robot/unitree/README.md)。

**配置**

- 本目录没有配置读取点：`relative_pid.py` 的参数是 `RelativePidConfig` 的 dataclass 默认值；`executor_command.py` 的 `run_id` / `scope_sha` / `limits` 由调用方传入；`publication_ledger.py`、`io.py`、`collision.py` 不读 `configs/`。
- `configs/control/relative_pid.yaml` 当前**无 Python 读取点**：它是独立 `relative-slam-navigation` 入口的保留配置，值与 `RelativePidConfig` 默认值一致但未接线，见 [configs/README.md](../../../configs/README.md) 的"当前无 Python 读取点"表；改它不会生效。
- cascade 速度界等 `configs/control/settings.json` 的键不在本目录消费，由 `tracking/` 读取。

## 模块内文件耦合

依赖方向：`__init__.py → relative_pid` 是唯一的包内 import；其余 5 个文件互不 import。跨包方向是"根目录向下依赖子包"：`executor_command.py` 在模块级 import `parking/parking_zero_contract.py`，运行时延迟 import `parking/terminal_reference.py`。39 个非 launch 模块可单独 import（`PYTHONPATH=src` 冒烟，无环错误）；`launch/` 下 2 个模块需要 ROS `launch` 包，本机未安装，未做导入冒烟。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `relative_pid.py` | 83 | `..planning.route` | `control/__init__`、`../navigation/task.py`、`scripts/replay_controller.py` | 清楚 |
| `collision.py` | 61 | numpy | `../navigation/relative_slam_node.py`、`../navigation/task.py` | 清楚 |
| `executor_command.py` | 220 | `common.observation`、`common.integration_limits`、`common.input_freshness`、`parking.parking_zero_contract`（模块级）、`parking.terminal_reference`（延迟） | `../robot/unitree/policy/worker.py`（函数内延迟） | 偏重：单函数 163 行 |
| `publication_ledger.py` | 85 | 标准库 | `../navigation/pid_control.py`（函数内延迟） | 可疑：反射式读取调用方状态 |
| `io.py` | 14 | `common.runtime_io`、`common.runtime_telemetry` | `../navigation/ros_node.py`、`../robot/unitree/bridge.py` | 清楚 |
| `__init__.py` | 1 | `relative_pid` | 无 | 清楚 |

四类问题：

- **上帝文件**：无（最大 220 行）。
- **重复实现**：角度归一化有三份 —— `relative_pid.wrap_angle`、`tracking/pid_core.py` 的 `wrap`、`tracking/cascade_core.py` 的 `wrap`，都是 `atan2(sin, cos)`；`relative_pid` 的 docstring 明确 π 端点可能取 ±π。
- **跨层直达**：`executor_command.py` 直接 import parking 契约（契约是纯校验函数，可接受）；`publication_ledger.py` 不 import 但直接读调用方实例上的 10 余个属性名（`command`、`state`、`obstacle_hold`、`alignment_hold`、`tilt_hold`、`tilt_source`、`bridge_safety`、`evidence`、`pid_row`、`heading_gate`、`control_clock_hold`），并用 `sys._getframe(2).f_code.co_name` 推断 `trigger`。
- **隐式契约**：`executor_command.CommandReader.read` 依赖命令 envelope 的字符串键与固定顺序（`schema_version=1`、`mode=scan_slam`、`source=scan_slam`、`sequence`、`command`、`sim_time`、`monotonic_wall`、`controller_source` 的 5 个时间戳、`ordinary_terminal_zero_event`）；`publication_ledger._flags` 全部用 `getattr(node, ..., None)` 兜底，node 属性改名不会报错、只会静默记 `None`。

结论：耦合**清楚且方向合理**（纯函数 + 两个接线点，无环、无上帝文件），唯一偏重点是 `executor_command.py`；`publication_ledger.py` 的反射式读取是可读性隐患，不是分层错误。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `relative_pid.py` | 良好 | 2 个函数 docstring 比 1.0；`RelativePidConfig` 即参数表；最长 51 行 |
| `collision.py` | 良好 | 3 个函数 docstring 比 1.0，最长 24 行；0.25 s / 0.25 m/s² / 0.15 m / 0.8 s / 0.30 m / 0.48 m 等魔法数都在 docstring 里解释 |
| `io.py` | 良好（薄） | 14 行、2 函数 docstring 比 1.0；薄到几乎没有内部结构，但两个工厂函数都没用到的 `run`/`profile` 参数没有说明 |
| `publication_ledger.py` | 可接受 | 85 行、docstring 比 0.75、2 行注释；`_flags` 用 9 个 `getattr` 拼字典，无类型标注 |
| `executor_command.py` | 需改进 | 220 行、docstring 比 0.0、`CommandReader.read` 163 行、0 行注释；`except (OSError, ValueError, KeyError, TypeError, OverflowError)` 把 5 类异常压成一条拒绝记录，record 中只保留 `type(error).__name__ + ": " + str(error)` |

- 结构问题 vs 文档问题：`executor_command.py` 两者都有（长函数 + 零 docstring）；其余文件基本只有文档缺口。
- 抽查复核：`collision.braking_sweep` 实测 24 行，docstring 与实现一致（0.25 s 反应、0.25 m/s² 减速、0.15 m 余量、15 点、≥0.8 s 视界），说明事实包指标可信。

## 重构建议

本轮只写建议、不改代码；任何动作都要"先有测试/对照再动"。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `CommandReader.read()` 拆成 `_check_envelope` / `_check_sequence` / `_check_freshness` / `_check_source_binding` / `_authorize_zero` 五个私有方法，共用一个 `record` 与一个 `reject()` | 163 行单函数、0 注释、拒绝分支 20+ | 拒绝原因字符串被日志与排查依赖，改动会改变历史对照 | 先固定 10~20 条真实 `raw` 字节 + `(clock, wall)` 的 golden 拒绝原因；当前 `../../../tests/unit/` 没有 `executor_command` 用例，需先补 |
| P2 | `publication_ledger` 改为显式传参或在 node 上提供只读快照方法，去掉 `sys._getframe(2)` | 靠调用栈推断 `trigger`，调用方函数改名会静默改变证据字段 | 已录证据里的 `trigger` 取值会变 | 对照一份已有的 `navigation_command_publications.jsonl` 的 `trigger` 取值集合 |
| P2 | 三处 `wrap`/`wrap_angle` 合并到一个公共工具函数 | 同一语义三份实现 | π 端点符号可能不同（`relative_pid` 明确允许 ±π） | 对 `[-3π, 3π]` 采样比对三者逐位一致后再合并 |
| P2 | 给 `executor_command.py` 每个拒绝分支补一行注释；给 `io.make_evidence_writer` 写明 `run`/`profile` 未使用的原因 | docstring 比 0.0 | 无 | 自检脚本 + 人工 review |

没有 P0：本层是纯函数与只读审计，未发现会改变控制输出或安全判定的问题。

## 输入输出与调用

输入是 `/demo/slam/body_odom` 的 canonical SLAM 反馈（`pose_producer=fast_livo2_async_backend`、`navigation_ground_truth_used=false`）、IMU、点云、SCAN 轨迹与路线/目标；输出是有界速度、零命令与实时状态。代码保留 JSONL 证据接口，但当前默认 `RuntimeTelemetry` 不写这些历史记录。`launch/components.py` 用 `pid_navigation`、`robot.unitree.bridge`、`mission_runtime` 三个受管进程组成控制组：bridge 把 `/demo/cmd_vel` 落成 `navigation_command.json`，策略 worker 再读取执行。

术语（canonical SLAM 反馈、provenance、时钟域）见 [docs/glossary.md](../../../docs/glossary.md)；四条执行路径与默认状态机见 [docs/architecture.md](../../../docs/architecture.md)。

## 测试

- `../../../tests/unit/test_control.py`：`relative_pid.follow_segment` 与原表达式 1000 组逐位等价，另覆盖支撑/制动/碰撞边界。
- `../../../tests/unit/test_simulation_safety.py`：仿真安全门禁的 fault/stop 判定边界。
- `../../../tests/regression/test_navigation_safety.py`：停车零契约、10Hz 位姿栅格与障碍解除窗口。
- 未覆盖：`executor_command.py` 与 `publication_ledger.py` 在本检出里没有直接单测（`grep -rl` 于 `tests/` 为 0 命中），改动这两处只能靠运行链路验证。

## 限制

控制律与门禁都是仿真/离线可用代码；完整路线、Building 实际运行与实机未验证。子包中仍有大量 profile/环境变量扩展（模块内默认关闭，部分由 launch 的 features 打开），改动前先确认对应配置项。本目录的 JSONL 证据在默认运行下大多不落盘，不能用"文件不存在"反推"没有发生"。
