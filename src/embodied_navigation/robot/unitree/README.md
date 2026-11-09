# robot/unitree：Go2 部署层

`robot/unitree` 是 Unitree Go2 的部署实现层：`bridge.py` 把导航命令门控成执行文件，`policy/` 管理当前 Student 策略部署，`robogauge/` 管理 RoboGauge 45D 部署。

## 目录结构

```text
src/embodied_navigation/robot/unitree/
├── README.md      # 本文件：部署层职责、命令桥与速度上限分层
├── __init__.py    # 1 行，包 docstring：Unitree 部署
├── bridge.py      # 1068 行，命令门控 ROS 节点（command_bridge）
├── policy/        # 子模块，见 policy/README.md
└── robogauge/     # 子模块，见 robogauge/README.md
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `bridge.py`（1068 行） | 活代码：ROS 节点 `teacher_scan_slam_bridge`，把 `/demo/cmd_vel` 经 `CommandGate` 校验后原子写入 `navigation_command.json`；不含策略推理与关节驱动 |
| 代码 | `policy/`、`robogauge/` | 两个子模块，分别见 [policy/README.md](policy/README.md)、[robogauge/README.md](robogauge/README.md) |
| 代码 | `__init__.py`（1 行） | 只有包 docstring "Unitree deployment: observations, joint order and inference."，无符号 |
| 日志/输出 | `<run>/navigation_command.json` | 本层唯一业务输出：`CommandFile.write()` 用同目录临时文件 + `replace()` 原子替换，内容是一帧 Twist 门控后的 JSON；不是导航成功证据 |
| 日志/输出 | `<run>/navigation_command.json.lock` | `CommandFile.__init__()` 用 `fcntl.flock(LOCK_EX\|LOCK_NB)` 占锁，防止第二个 bridge 同时写同一文件 |
| 日志/输出 | 无日志文件、无 logging | `grep -rn "import logging\|getLogger" src/embodied_navigation/robot` 无命中；`bridge.py` 只有 2 处 `print`（`--prepare` 收据与 refusal），不写日志目录 |
| 日志/输出 | `<run>/navigation_command_history.jsonl` | **当前不产出**：`bridge.py` 声明了该 history 变量并传给 `make_command_writer`，但 [control/io.py](../../control/io.py) 的 `make_command_writer(writer, history, configuration)` 固定构造 `LatestCommandWriter(writer, None)`，history 被丢弃。不要把它当作可用证据；要恢复需先接线（见重构建议 P2） |
| 配置 | `<run>/runtime_context.json` | `main()` 经 `common.runtime_context.load()` 读本次 run 的冻结上下文；`configuration["current_runtime"]`、`run_dir`、`profile` 都来自这里 |
| 配置 | `profile["cascade"]["command_limits"]` | `command_limits(profile)` 从 runtime context 里的 profile 读，并校验 `0 < vx <= 1.0`、`0 < vy <= 0.4`、`0 < wz <= 1.0`；权威来源是 `configs/control/settings.json`（由启动器读入并写入本次 profile），见 [configs/README.md](../../../../configs/README.md) |
| 配置 | 无直接 `configs/` 读取 | `bridge.py` 不 import `common.config`；无 `read()` 调用。不读 runtime context 的 fallback 分支才用环境变量 `TEACHER_COMMAND_FILE`，默认 `ROOT/runs/nav_command.json` |

补充证据：`bridge.py` 里的 `diagnostic_observation` hook 经 `common/observation.py` 的 `api()` 取得，而正常运行时该端口是 `DisabledObservation`（`ENABLED=False`，`record()` 是 no-op），因此这些诊断调用不落盘。

## 模块内文件耦合

- 依赖方向：`bridge.py` → `common/{observation,input_freshness,integration_limits,runtime_context,runtime_io}`、`control/{io,safety.gate_reasons,parking.parking_zero_contract,parking.terminal_reference}`、`planning/reference/route_fence`（共 11 个内部模块）；反向依赖是 `control/parking/causal_join.py` 与 `control/safety/duplicate_source.py` 都 import `bridge.validate_controller_source`，`control/launch/components.py` 以 `command_bridge` 角色启动它。**包级存在 `robot ↔ control` 的双向依赖**（文件级未形成 import 环）。
- 两个子目录各自独立，`policy/` 与 `robogauge/` 互不 import（robogauge → policy/native 的数值依赖见下级 README）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `bridge.py` | 1068 | `common/`×5、`control/`×4、`planning/`×1 | `control.parking.causal_join`、`control.safety.duplicate_source`；`command_bridge` 进程 | 偏重：一个文件同时做 ROS 接线、命令门控、来源时间校验与文件传输 |
| `policy/`（7 py） | 2550 | 见 [policy/README.md](policy/README.md) | 见 [policy/README.md](policy/README.md) | 见下级（含 0 行空 `__init__.py`） |
| `robogauge/`（4 py） | 253 | 见 [robogauge/README.md](robogauge/README.md) | 见 [robogauge/README.md](robogauge/README.md) | 见下级 |
| `__init__.py` | 1 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：有——`bridge.py` 1068 行（唯一 >800 行文件）。
- **重复实现**：本目录内没有；旋转矩阵公式在 `policy/native/observation.py` 与 `policy/student.py` 中各有一份，见下级 README。
- **跨层直达**：有——`bridge.py` 直接 import `control/` 与 `planning/` 的内部实现（`control.io`、`control.safety.gate_reasons`、`control.parking.*`、`planning.reference.route_fence`），同时 `control/` 又反向 import 它。`validate_controller_source` 只做数据/时间契约校验、不依赖 ROS，却住在 ROS 节点文件里，正是这个环的来源。
- **隐式契约**：有——跨进程靠字符串键与固定文件名：`navigation_command.json` 的 JSON 字段、`cascade_command_source.json` 的来源字段（`source_pose_stamp_ns`、`paired_imu_stamp_ns`、`cascade_sequence` 等，`validate_controller_source` 逐键校验）、300 ms 新鲜度、状态/停止话题名。

结论：**偏重但边界可读**。`bridge.py` 本身职责单一（门控 + 写文件），但它同时是 ROS 接线点、来源校验库和跨层依赖的入口，导致 1068 行里真正的门控逻辑与接线样板混在一起。

## 可读性评估

| 对象 | 分级 | 证据 |
| --- | --- | --- |
| `bridge.py` | **需改进** | 1068 行 / 30 个函数；最长函数 `make_node` 319 行（内嵌整个 `Bridge` 类：`__init__` + 12 个回调/辅助方法）；>80 行函数 3 个；注释仅 19 行；docstring 覆盖 0.367；无 logger、仅 2 处 `print`。模块 docstring 写清了“不是导航成功证据”，这点良好 |
| `__init__.py` | **良好** | 1 行 docstring，无代码 |
| `policy/`、`robogauge/` | 见下级 | 各自 README 给出分级 |

结构问题与文档问题要分开：`bridge.py` 的主要问题是**结构**（319 行的 `make_node` 把 ROS 依赖、QoS、节点类、`tick()` 混在一个函数），文档问题次要（模块/函数 docstring 已有，缺的是“哪个函数属于哪层职责”的说明）。异常处理并非全显式：`bridge.py` 共 10 处 `except`，多数记录 reason 或 fail closed（如 fence 解析失败按 `outside=True` 处理），但 `sensor_health`/`execution_health`/`anchor` 三个 ROS 回调是 `except (…): return` 直接丢弃非法消息、不写原因，属于需注意的静默分支。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `make_node` 里的 `class Bridge` 提到模块级，`make_node(rclpy, ...)` 只做依赖注入与实例化 | 319 行的内嵌类让“门控逻辑”和“ROS 装配”无法分开阅读 | 中：ROS 生命周期/关闭顺序容易改错；**当前没有任何直接测试覆盖 `bridge.py`**（`grep -rln "unitree.bridge\|CommandGate" tests` 无命中） | 先补 `CommandGate` 纯逻辑单测 + `--prepare` 收据测试，再拆；跑 `tests/regression` |
| P1 | 把 `validate_controller_source` 移到 `common/` 下的时间/来源契约模块，`bridge.py` 与 `control/` 都从那里 import | 它是纯数据校验却住在 ROS 节点文件里，造成 `robot ↔ control` 包级双向依赖 | 中：两个 control 消费者 + bridge 内部两处调用 | 改后 `grep -rn validate_controller_source src tests` 全部指向新位置；`tests/unit` 与 `tests/regression` 全绿（这两个消费者目前只有间接覆盖） |
| P2 | 接通或删除 `navigation_command_history.jsonl`：让 `make_command_writer` 真正接收 history，或删掉变量并在本 README 标注“不产出” | 现在声明但不写文件，运维/复核会误以为有历史收据 | 低 | 跑一个 run 后检查 run 目录；补一个不依赖 ROS 的 `LatestCommandWriter` 单测 |
| P2 | 给 `bridge.py` 加一组不启动 ROS 的测试（`CommandGate` 的 300 ms 过期、来源不一致拒绝、`command_limits` 边界） | 本层是默认主路径的集成边界，却零直接测试 | 低 | `PYTHONPATH=src python3 -B -m unittest discover -s tests/regression` |

没有 P0：`bridge.py` 是集成边界，本身不直接驱动关节、不订阅真值；它的问题是可维护性与证据缺口，不是正确性缺陷。本轮只写建议、不改代码；P1 两项都必须先有测试再动。

## 关键文件与接口

- `bridge.py`：进程角色 `command_bridge`（ROS 节点名 `teacher_scan_slam_bridge`），由 [control/launch/components.py](../../control/launch/components.py) 启动。订阅 `/demo/slam/body_odom`、`/demo/cmd_vel`，经 `CommandGate` 校验后把 Twist 原子写成 `navigation_command.json`，并发布 `/demo/teacher/navigation_bridge/status`、`/stop`。`TIMEOUT = COMMAND_SOURCE_FRESHNESS_S`（300 ms）。它不订阅 Gazebo 真值，也不直接驱动关节；`validate_controller_source` 同时被 control 的重复源与停车因果代码复用。
- `policy/`：历史 Teacher native 中间观测（247D，当前用于诊断）与 Student 推理、底层控制环 ACK 扩展，见 [policy/README.md](policy/README.md)。
- `robogauge/`：RoboGauge MoE-CTS 的 45D 直接观测、CPU TorchScript 推理与 Go2HV 数值契约，见 [robogauge/README.md](robogauge/README.md)。

**速度上限不是单一常量，按层不同（不要为了统一而改值）**：

| 层 | vx | vy | wz | 位置 |
| --- | --- | --- | --- | --- |
| `cascade.command_limits`（控制器输出界的权威配置） | 0.6 | 0.2 | 0.6 | `configs/control/settings.json`，经 `bridge.command_limits(profile)` 读入 |
| bridge 无 profile 时的 fallback | 0.3 | 0.2 | 0.3 | `bridge.py` 的 `LIMITS` |
| Student 策略侧 `command_bounds` | 0.3 | 0.2 | 0.3 | `policy/student.py` |
| RoboGauge benchmark 命令门 | 0.6 | 0.2 | 0.3 | [navigation/robogauge/commands.py](../../navigation/robogauge/commands.py) |
| 控制器发布前 `max_speed` 参数（默认） | 0.12 | — | — | [navigation/base_node.py](../../navigation/base_node.py) |

各层职责不同：cascade 限制控制器输出，bridge 限制写入命令文件，Student 限制策略动作，RoboGauge 是另一条部署；`configs/scenarios/README.md` 里写的 “student 有效命令界” 是运行档位摘要，不是 bridge fallback。

## 两条部署路径不要混用

- `policy/native` 保留历史 Teacher 的 247D 中间观测适配；当前普通导航要求 Student，不加载 Teacher actor。Student 实际推理走 `policy/student.py` 从同一 native64 状态算出的 exact45，247D 仅作诊断。
- `robogauge` 的 45D 是独立直接观测接口，有自己的 scale 与关节顺序，不是 247D 的裁剪，也不含 Student 的 COM 速度 PID。
- 两者不能当成同一个策略输入或 actuator 契约。

## 测试与限制

本层无独立目录测试；`policy/native` 与 `robogauge` 分别由 [tests/regression/test_native_safety_integration.py](../../../../tests/regression/test_native_safety_integration.py)、[tests/regression/test_robogauge_contract.py](../../../../tests/regression/test_robogauge_contract.py) 等覆盖。`bridge` 是集成边界，不代表导航成功；没有实机后端，本会话未启动仿真或机器人。`bridge.py` 本身没有直接测试（见重构建议 P1/P2），测试边界与跳过项见 [tests/README.md](../../../../tests/README.md)。
