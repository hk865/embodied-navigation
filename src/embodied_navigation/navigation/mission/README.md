# navigation/mission

本目录是任务的目标与到达层：把路线定义变成带区域、到达窗口与通过证据的任务，并把 anchor、request 与 global reference 发布给控制链路；它不产生速度命令。

## 边界

- 做：目标几何与到达判定（`goal_regions.py`）、三段平台与 46 区域定义（`route_regions.py`）、连续路线进度（`continuous_route.py`，默认关闭）、任务运行时节点（`mission_runtime.py`）、协调门与 stop 事件证据（`mission46_guard.py`、`dynamic_stop_event.py`）。`state_machine.py` 是历史迁移实现，当前没有被实例化。
- 不做：控制、规划与物理安全；到达证据不等于运动许可。

## 目录结构

```text
src/embodied_navigation/navigation/mission/
├── README.md                # 本文件（模块说明；行数随改写变化，不标注）
├── __init__.py              # 0 行（空文件，无重导出；所有导入都写完整模块路径）
├── continuous_route.py      # 430 行，固定 original46 路线进度与实测通过证据（默认关闭）
├── dynamic_stop_event.py    # 257 行，把已发布的 stop 事件绑定到 guard 证据，无控制权
├── goal_regions.py          # 361 行，不可变目标几何、坐标变换与 measured-stamp dwell
├── mission46_guard.py       # 64 行，保持时间戳的 controller 协调门
├── mission_runtime.py       # 296 行，默认 mission 角色进程：冻结 anchor/request、发布参考
├── route_regions.py         # 154 行，三段平台物理路线的预注册到达区域
└── state_machine.py         # 357 行，与 ROS/仿真时间解耦的任务迁移（本检出无实例化点）
```

## 代码、日志与配置

### 代码

| 文件 | 性质 | 判断依据 |
| --- | --- | --- |
| `mission_runtime.py` | 活代码（进程入口） | 由 `../../control/launch/components.py:25` 以 `mission` 角色启动 `embodied_navigation.navigation.mission.mission_runtime`；`main()` 内延迟 import rclpy 与消息类型（L21–28），`Mission(Node)` 定义在 `main()` 内部（L40） |
| `goal_regions.py` | 活代码（纯几何库） | 无 ROS/rclpy import；被 `base_node`、`pid_support`、`continuous_route`、`route_regions`、`state_machine` 五个模块使用 |
| `route_regions.py` | 活代码（数据定义） | 3 个函数（`add_route_regions`/`validate_route_regions`/`transformed_route_goals`），只被 `state_machine` 使用，因此当前随休眠路径一起不生效 |
| `mission46_guard.py` | 活代码（gate） | `mission_hold` 被 `pid_control` 调用，保持时间戳 |
| `dynamic_stop_event.py` | 活代码（证据绑定） | 被 `ros_node` 调用；模块 docstring 明写 "no control authority" |
| `continuous_route.py` | 休眠但被 import | `pid_support.py:1012-1016` 在导入期取 `configuration`/`RouteProgress`/`CausalHandoffProjectionController`；只有 `enabled` 为真才构造 |
| `state_machine.py` | 历史保留 | 模块 docstring 与正文都注明本检出无实例化点；唯一子类 `ContinuousMission`（`continuous_route.py:414`）也没有构造点 |
| `__init__.py` | 空 | 0 字节 |

### 日志与输出

本子包**不写日志**：8 个文件里没有 `getLogger()`、没有 `print()`（实测均为 0），也没有 logging 配置。唯一的写盘点是 `mission_runtime.py` 的 `atomic()`（L13–17，`write_text` + `replace`），写的是**运行产物**而不是日志：

| 文件（均在 run 目录） | 写入点 | 内容与消费者 |
| --- | --- | --- |
| `navigation_anchor.json` | `mission_runtime.py:216` | schema=1 的冻结 anchor（`schema`/`run_dir`/`frame_id`/`source`/`ground_truth_navigation_used=false`）；消费者是 `planning/admission/path_admission.py:93` 的 `FrozenFence.load()`（fallback 分支，见"限制"） |
| `navigation_request.json` | `mission_runtime.py:217` | `schema_version=2` 的目标请求；消费者 `navigation/ros_node.py:581`、`simulation/terrain/mission46.py:92`、`simulation/terrain/provider.py:277` |
| `global_reference.json` | `mission_runtime.py:263` | `schema="r85_pct_scan_global_reference/v1"` 的全局参考；文件名由 `prepare.py:72-76` pin 进 profile，消费者是 `planning/reference/automatic_reference_runtime.py`（`REFERENCE_SCHEMA` 同值，按 epoch/generation 校验） |

同时 `mission_runtime.py` 收发以下话题（都在 `/demo/*` 主路径）：发布 `/demo/navigation/request`、`/demo/teacher/navigation/anchor`（TRANSIENT_LOCAL）、`/demo/navigation/stop`；订阅 `/demo/slam/body_odom`、`/livox/imu`、`/demo/navigation/localization_status`、`/demo/navigation/scan_epoch_ready`、`/demo/teacher/navigation/sensor_health`（L56–78）。就绪后一次性冻结，不是周期重发。

### 配置

| 来源 | 读取点 | 说明 |
| --- | --- | --- |
| run 内 `runtime_context.json` 的 `profile` | `mission_runtime.py:34` | 只取 `profile`，用于 `attach_reference_heading(profile, document)`（终端航向契约） |
| run 内 `global_route.json` | `mission_runtime.py:35` | 取 `body_points`/`ground_points`/`T_navigation_world` 等，是 prepare 从 `assets/geometry/route.json` 生成的本 run 路线 |
| profile 的 `continuous_route_contract` | `continuous_route.py:42-44` | `enabled` 必须是布尔，否则 `ValueError`；当前 `configs/planning/settings.json` 为 `false` |
| 其余文件 | 无 | `goal_regions.py`/`route_regions.py`/`state_machine.py`/`mission46_guard.py`/`dynamic_stop_event.py` 都没有配置读取点，也不读 `configs/` |

注意：本子包**没有直接读 `configs/`**（`grep -rn "configs/" src/embodied_navigation/navigation/mission/*.py` 无命中）。`continuous_route_contract.enabled=false` 这个值之所以生效，路径是 `configs/planning/settings.json` → `launch/prepare.py` 合并进 profile → `runtime_context.json`；读取顺序与 pin 规则见 [`../../../../configs/README.md`](../../../../configs/README.md)。

## 模块内文件耦合

依赖方向：`goal_regions` ← `route_regions` ← `state_machine` ← `continuous_route`（纯 Python，单向、无环）；`mission_runtime` 是叶子进程入口，**不被任何模块 import**；`mission46_guard`、`dynamic_stop_event` 是叶子库，分别被 `pid_control`、`ros_node` 使用。没有 import 环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `goal_regions.py` | 361 | 无内部依赖 | `base_node`、`pid_support`、`continuous_route`、`route_regions`、`state_machine` | 清楚（有意做成中心几何契约） |
| `route_regions.py` | 154 | `goal_regions` | `state_machine` | 清楚，但只服务休眠路径 |
| `state_machine.py` | 357 | `goal_regions`、`route_regions` | `continuous_route` | 可疑（历史保留：有被 import、无实例化） |
| `continuous_route.py` | 430 | `state_machine`、`goal_regions`、`control.tracking.cascade_core`、`planning.reference.global_route_reference` | `pid_support` | 可疑（休眠但导入期被取值；向上依赖 control 层） |
| `mission_runtime.py` | 296 | `common.config`、`common.runtime_context`、`common.runtime_io`、`control.parking.terminal_heading_contract` | 无（进程入口） | 清楚 |
| `mission46_guard.py` | 64 | 无 | `pid_control` | 清楚 |
| `dynamic_stop_event.py` | 257 | 无 | `ros_node` | 清楚 |
| `__init__.py` | 0 | 无 | 无 | 清楚（空文件，导入全部走完整路径） |

四类问题：

1. **上帝文件（>800 行）**：无，最大 `continuous_route.py` 430 行。
2. **重复实现**：有一处**同名不同物**——`state_machine.py:58` 的 `class Mission`（任务迁移）与 `mission_runtime.py:40` 的 `class Mission(Node)`（ROS 节点）同名；两者没有继承或调用关系，读者极易把 `mission_runtime` 的 `Mission` 当成任务状态机。这是本子包最容易误读的点。
3. **跨层直达**：`continuous_route.py:20-21` 从任务层直接 import `control.tracking.cascade_core.Controller` 与 `planning.reference` 的具体类，而不是通过项目/速度契约；同目录其余模块保持纯函数式，只有它跨层。
4. **隐式契约**：三份 run JSON 是跨进程契约，键名（`schema=1`、`source`、`original_region_count` 等）由消费方 `FrozenFence.load()` 写死校验，没有共享的 schema 常量；`mission_runtime` 只写出其中一部分键（见"限制"）。

结论：**耦合清楚、层次合理，但有一类"半休眠"状态**——`state_machine`/`continuous_route`/`route_regions` 三个文件占了本子包 941 行（49%），当前既没有被实例化，也不能删（`continuous_route` 的 `configuration`/`RouteProgress` 仍在导入期被取用）。对默认主路径而言，本子包真正在跑的是 `mission_runtime.py`（296 行）+ `goal_regions.py`（被 `base_node` 用）+ 两个叶子 gate。

## 可读性评估

指标为实测 AST 值（函数总数 / 有 docstring 函数比 / 最长函数 / >80 行函数数），抽查过 `goal_regions.parse_goal`（L142–197，56 行）与 `mission_runtime.main`（L20–292）确认可信。

**良好**

- `mission46_guard.py`：64 行 / 1 函数 / docstring 1/1 / 50 行；职责单一（"保持时间戳的协调门"），命名与注释一致。
- `route_regions.py`：154 行 / 3 函数 / docstring 3/3；`add_route_regions` 86 行（L15–100）是唯一 >80 行函数，但线性数据装配，易读。
- `dynamic_stop_event.py`：257 行 / 6 函数 / docstring 5/6，最长 `select_event_preroll` 131 行（L127–257）；模块 docstring 直接写了边界（"no control authority"）。

**可接受**

- `goal_regions.py`：20 个函数、最长 56 行、0 个 >80 行函数——**结构好**；问题纯属文档：docstring 只有 2/20（10%），`parse_goal`/`contains` 这类几何判定没有说明单位、帧与容差，只能读实现。
- `continuous_route.py`：18 函数 / docstring 6/18，最长 `completion_valid` 101 行（L311–411）；`RouteProgress` 的进度语义靠字段名表达，没有 docstring。
- `state_machine.py`：13 函数 / docstring 4/13，最长 68 行（`region_completion_valid`，L168–235）；模块 docstring 已诚实标注"本检出未实例化"，这是本子包最好的防误读说明。

**需改进**

- `mission_runtime.py`：296 行里 `main()` 占 273 行（L20–292），并且在函数体内定义 `class Mission(Node)`（L40）——argparse、节点类、就绪判断、发布与退出路径全在一个函数里；没有测试（见下），任何改动只能靠启动进程验证。

区分说明：`goal_regions.py`/`continuous_route.py` 是**文档问题**（结构没毛病，缺 docstring 与单位/帧说明）；`mission_runtime.py` 是**结构问题**（单函数 273 行 + 内联类）。

## 重构建议

本轮只写建议、不改代码。**没有 P0/P1**：本子包不产生速度命令，关键判定 fail-closed（`continuous_route.py:43-44` 对非布尔 `enabled` 抛错，`mission_runtime.py:35-38` 对非有限/形状错误的路线几何抛错），且不写日志、无静默异常；3 个休眠文件是历史迁移的明确保留，不算结构缺陷。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把 `mission_runtime.py:40` 的内联 `class Mission(Node)` 提到模块级，或用 `MissionRuntime` 命名并在 docstring 里与 `state_machine.Mission` 区分 | 同名不同物 + 273 行单函数是本子包最主要的可读性问题 | 中：提类会碰 rclpy 延迟 import 与节点生命周期，需保持"不被 import 即不建节点" | 提类前先补一个最小 `--check`/纯函数化入口；改后启动一次 mission 角色比对三份 run JSON |
| P2 | 给 `goal_regions.py` 的 `parse_goal`、`contains`、`contains_control`、`relative_world_transform` 补 docstring，写明帧/单位/容差与 dwell 用 measured stamp 的约束 | 20 函数只有 2 个 docstring，几何判定缺单位说明是最容易误用的地方 | 无（纯文档） | 只改文档，不跑测试 |
| P2 | 给 `continuous_route.py` 的 `RouteProgress` 与 `completion_valid` 补 docstring，并注明"仅在 `enabled=true` 时使用" | 休眠文件仍需可读；否则读者会以为它是默认行为 | 无（纯文档） | 只改文档 |
| P2 | 在 `state_machine.py` 头部补一行"唯一子类 `ContinuousMission` 无构造点、不要据此推断默认行为"，或把该结论同步到本 README（已写） | 防误读已有 docstring，但入口文档仍需一致 | 无 | 交叉阅读 `../../../docs/architecture.md` §"默认主路径的任务状态机" |

## 关键文件

- `mission_runtime.py`：ROS 节点，由 `../../control/launch/components.py` 以 `mission` 角色启动。等定位、epoch 与健康都 ready 后冻结 `navigation_anchor.json` 和 `navigation_request.json`，周期性发布 `/demo/navigation/request` 与 `/demo/teacher/navigation/anchor`，退出时向 stop 话题发 `True`；读 run 目录的 `global_route.json`，生成 schema 为 r85_pct_scan_global_reference/v1 的 `global_reference.json`。
- `goal_regions.py`：`Goal`、`RigidTransform`、`ArrivalWindow`。目标几何不可变，到达用 measured stamp 的 dwell 判定而非 wall timer；本文件不建 ROS 节点、不接受真值控制输入。
- `continuous_route.py`：`RouteProgress`、`ContinuousMission`、`CausalHandoffProjectionController`，把普通区域完成改为实测通过证据；是否启用由 `../../../../configs/planning/settings.json` 的 `continuous_route_contract.enabled`（当前 `false`）决定。
- `route_regions.py`：`add_route_regions`、`validate_route_regions`、`transformed_route_goals`，在准备阶段校验区域歧义。
- `state_machine.py`、`mission46_guard.py`、`dynamic_stop_event.py`：与 ROS 时间解耦的历史任务迁移（未实例化，唯一子类 `ContinuousMission` 也没有构造点）、保持时间戳的 controller 协调门、以及把已发布 stop 事件绑定到 guard 证据（无控制权）。

## 测试

本子包无专用测试：`grep -rln "goal_regions\|mission_runtime\|continuous_route\|state_machine\|route_regions\|mission46_guard\|dynamic_stop_event" tests/ --include=*.py` **无命中**（`tests/` 下没有任何文件引用这 7 个模块名）。`mission_runtime.py` 由 `../../control/launch/components.py` 以 `mission` 角色启动、进入默认主路径；`state_machine.py` 与 `continuous_route.py` 当前未被任何运行链路实例化（`configs/planning/settings.json` 的 `continuous_route_contract.enabled=false`）。

## 限制

`mission_runtime.py` 写出的 anchor 字段集与 `../../planning/admission/path_admission.py` 中 `FrozenFence` 的期望并不一致，因此 `FrozenFence` 这条 **fallback** 分支在本检出中未闭合：`FrozenFence.load()` 要求 `frozen_once is True`、`original_region_count == 46`、`registered_route_reference == "original46 centers with single actualSLAM/IMU yaw and initial measured origin"`、`scene_axis_registration_file` 及其 sha256（`path_admission.py:103-116`），而 `mission_runtime.py:199-215` 写出的 anchor 只有 `schema`/`run_dir`/`frame_id`/`source`/`ground_truth_navigation_used`/`initial_actual_slam_source` 等键，**缺上述字段**。默认 profile 启用 `r85_auto_reference_contract`，实际准入走 `AutomaticRouteFence` + `R85PhysicalPathGate`（见 [`../../planning/admission/README.md`](../../planning/admission/README.md)）。连续路线契约默认关闭，完整 46 区域未验证。
