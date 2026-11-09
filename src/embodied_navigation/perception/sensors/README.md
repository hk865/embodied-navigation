# perception/sensors：感知侧 ROS 传感流适配

本目录把相机、LiDAR、IMU 与 SLAM 反馈适配成导航可用的 ROS 话题、健康门和有界交接结构，属于感知侧运行组件，不是顶层数据契约层。

## 边界

- 只做转发、过滤、配对校验与交接，不做控制决策、不写导航状态机；`feedback_mailbox.py` 只暂存 corrected message，提交仍由 control 组负责。
- 不读 SLAM 世界位姿或真值做传感器过滤（`self_echo_filter.py` 只用固定 URDF/scenario 外参）；`sensor_gate.py` 的 `ground_truth_navigation_used` 恒为 `False`。
- 运行需要完整 ROS 图与原始 `/livox`、`/demo` 话题；本会话未启动 ROS/仿真，行为未在线复验。

## 目录结构

```text
src/embodied_navigation/perception/sensors/
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），空包标记
├── async_writers.py  # 220 行，R119 metadata/body 异步 writer 选择与实测 QoS 回执
├── feedback_mailbox.py  # 683 行，有界不可变 corrected-message 交接
├── metadata_qos.py  # 12 行，corrected metadata 订阅深度
├── ready_commit.py  # 67 行，R92 ready 通知（commit 仍归 control 组）
├── sampling.py  # 97 行，fresh 场景的已审核采样覆盖
├── self_echo_filter.py  # 196 行，剔除 Go2 自身包络的 LiDAR 点
├── sensor_gate.py  # 520 行，传感器/SLAM 预热与健康门
└── sensor_relay.py  # 249 行，IMU/RGB 原样转发与 writer 选择
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | 9 个 `.py` 共 2045 行 | 4 个是库/入口混合：`sensor_relay`、`self_echo_filter`、`sensor_gate` 由 [localization_mapping/launch/components.py](../../localization_mapping/launch/components.py) 以 `python -m` 子进程启动；`sampling` 由 [launch/prepare.py](../../launch/prepare.py) 调用；`feedback_mailbox`/`metadata_qos`/`ready_commit`/`async_writers` 被定位与控制侧 import。`__init__.py` 是空包标记。无兼容门面或生成物。 |
| 日志/输出 | 写 run 目录，位置由 `--run`/调用方传入 | `sensor_relay.py` 写 `sensor_relay_loaded.json`（pid、`source_sha256`、transport、实测 writer QoS）与 `sensor_relay_exit.json`（计数/时间戳/发布耗时）；`sensor_gate.py` 原子写 `navigation_sensor_gate.json` 并追加 `navigation_sensor_gate_history.jsonl`；`sampling.py` 写 `world_before_sensor_override.sdf`、重写 `sensor_contract.json`、写 `sensor_sampling_contract.json`；`async_writers.py` 写 `r119_metadata_async_actual.json`。`self_echo_filter.py` 不写文件，只发布身份证明字符串；`feedback_mailbox.py`/`ready_commit.py`/`metadata_qos.py` 不写文件。 |
| 配置 | 环境变量 + profile 键 + run 文件 | 环境变量（启动前必须设置）：`R119_METADATA_ASYNC`、`R119_BODY_ASYNC`（`async_writers` 在 **模块导入时**读取，非法值直接 `ValueError`）、`R92_READY_COMMIT`（`ready_commit`）。profile 键：`r86_prepared_feedback`、`r86_expired_prefix`（`feedback_mailbox`）、`r74_corrected_metadata_latest_only`（`metadata_qos`）、`sensor_sampling`（`sampling`）、`sensor_warmup_sim_s`/`recovery_fresh_pose_samples`/`recovery_min_pose_span_sim_s`/`warmup_history`（`sensor_gate` 从 `runtime_context.json` 的 profile 读）。`sensor_gate` 还读本次 run 的 `sensor_contract.json`、`navigation_camera.yaml`；`self_echo_filter` 读 `configs/simulation/scenario.json`（`--scenario` 默认值）。 |

本目录的开关默认值与 run 环境来源见 [configs/README.md](../../../../configs/README.md)；当前 `configs/launch/settings.json` 的 `features` 把 `R92_READY_COMMIT`、`R119_METADATA_ASYNC`、`R119_BODY_ASYNC` 都设为 `1`，由启动器作为环境变量注入。

## 模块内文件耦合

依赖方向：包内只有一条依赖 `sensor_relay.py → async_writers.py`（复用只读 `writer_qos.so` 读取），其余文件互不 import；跨层依赖从感知指向定位/控制（`feedback_mailbox.py` 运行期 import `control.tracking.cascade_core` 与 `control.parking.parking_pose10hz`，`self_echo_filter.py` import `localization_mapping.geometry`/`point_provenance`），反向由 `localization_mapping`、`navigation` 消费本目录。图是无环的，但**存在跨层直达**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `async_writers.py` | 220 | `common.config`（ROOT）、`writer_qos.so` | `sensor_relay.py`、`localization_mapping/localization_runtime.py` | 可接受（ctypes + 环境变量，副作用在导入时） |
| `feedback_mailbox.py` | 683 | `common.observation`、`control.tracking.cascade_core`、`control.parking.parking_pose10hz` | `localization_mapping/known_scene_inputs.py`、`navigation/base_node.py`、`navigation/pid_ingress.py`、`navigation/ros_node.py` | 偏重（683 行 + 感知→控制跨层） |
| `metadata_qos.py` | 12 | 无 | `localization_mapping/known_scene_inputs.py` | 清楚 |
| `ready_commit.py` | 67 | 无 | `localization_mapping/known_scene_inputs.py` | 清楚 |
| `sampling.py` | 97 | 无（stdlib + XML/JSON） | `launch/prepare.py` | 清楚 |
| `self_echo_filter.py` | 196 | `localization_mapping.geometry`、`point_provenance`、`project` | 无（由 launch/components 以子进程启动） | 偏重（感知→定位跨层） |
| `sensor_gate.py` | 520 | `common.runtime_io`、`runtime_telemetry`、`input_freshness`、`shutdown`、`runtime_context` | 无（子进程入口） | 偏重（`_main` 内嵌 363 行节点类） |
| `sensor_relay.py` | 249 | `async_writers`、ROS | 无（子进程入口） | 清楚 |
| `__init__.py` | 0 | 无 | 无 | 清楚 |

四类问题核查：

- 上帝文件：没有文件 >800 行（最大 `feedback_mailbox.py` 683 行），但 `sensor_gate.py` 的 `_main()` 单函数 363 行（149–512 行），把 `Gate` 节点类整个嵌在函数体内。
- 重复实现：有跨模块重复——原子 JSON 写至少 4 份（本目录 `sensor_gate.atomic`、`common/runtime_io.py`、`common/runtime_telemetry.py`、`perception/person_worker.py`），实现相同但各处自行维护。
- 跨层直达：有。`feedback_mailbox.py:595,618` 在函数内 import control；`self_echo_filter.py:17,31` 顶层 import localization_mapping。
- 隐式契约：有。`Mailbox` 依赖 `ROLES = ("body","cloud","metadata")` 的固定顺序、`SLOTS/FRAMES/BYTES/PART_BYTES = 24/16/32 MiB/4 MiB`；`control_scope(node)`/`receipt_ns(node)`/`check_negative_parking(node)` 读取 `node.request_id`、`node.waypoint_index`、`node.heading_gate`、`node._r86_committing_bundle`、`node.cascade.ordinary_parking` 等**由 navigation/control 侧提供的属性名**，改名即静默失效。

结论：包内耦合清楚、方向单一，真正的问题是两个"大而跨层"的文件（`feedback_mailbox.py` 与 `sensor_gate.py`）以及与 control 的隐式属性契约，而不是包内互相纠缠。

## 可读性评估

按 AST 实测指标（函数总数 / 有 docstring 函数比 / 最长函数 / >80 行函数数 / 注释行），抽查 `Mailbox.receive`（196–359 行）与 `_main`（149–512 行）确认指标可信。

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `sampling.py` | 良好 | 3 函数，docstring 3/3=1.0，最长 73 行（`override_sensors`），0 个 >80 行函数 |
| `async_writers.py` | 可接受 | 6 函数，docstring 0.5，最长 70 行（`configure`），7 注释行 |
| `metadata_qos.py` | 可接受 | 1 函数 7 行，docstring 0.0；体量小、逻辑直白，属文档问题 |
| `ready_commit.py` | 可接受 | 6 函数，docstring 0.0，最长 13 行 |
| `self_echo_filter.py` | 可接受 | 6 函数，docstring 0.0，最长 46 行（`receive`），5 注释行；有模块级长 docstring 说明边界 |
| `sensor_relay.py` | 需改进 | 9 函数，docstring 0.111，最长 66 行，2 处 `print`（仅在证据文件写失败时打 stderr，可观测） |
| `feedback_mailbox.py` | 需改进 | 30 函数，docstring 0.433，最长 164 行（`Mailbox.receive`），2 个 >80 行函数，14 注释行 |
| `sensor_gate.py` | 需改进 | 14 函数，docstring 0.214，最长 363 行（`_main`），3 个 >80 行函数；`Gate` 类嵌套在 `_main` 内无法单独导入 |

结构问题集中在 `sensor_gate.py`（单函数 363 行、内嵌类）与 `feedback_mailbox.py`（164 行 `receive`、角色/容量隐式契约）；其余多是文档问题（`self_echo_filter`、`ready_commit`、`metadata_qos`、`sensor_relay` 的 docstring 比 ≤0.11）。`feedback_mailbox.py` 的 `except Exception` 都记录 `evidence_call_errors`/`fault_reason` 后再返回，不是静默吞错。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `sensor_gate.py` 中嵌在 `_main()` 里的 `Gate` 类提到模块级，按"订阅/校验/报告"拆开 `tick`（148 行）与 `receive`（91 行） | 单函数 363 行、类无法单测；健康门是运行必需节点 | 中：涉及 ROS 图与时间语义 | 现有仅 [tests/unit/test_ros_shutdown_lifecycle.py](../../../../tests/unit/test_ros_shutdown_lifecycle.py) 做 AST 检查，需先加节点构造 smoke（不 spin）再拆 |
| P1 | 把 `feedback_mailbox.Mailbox.receive()`（196–359，164 行）按"入队校验 → 去重/计数 → ready 通知"分阶段拆出 | 最长函数 164 行且同时持有锁与证据写入，改一处要读全段 | 中：并发与有界容量语义 | 先补 `take_batch`/`retire_expired` 的纯状态机测试，再拆 |
| P2 | 补函数 docstring：`self_echo_filter.py`(0.0)、`ready_commit.py`(0.0)、`metadata_qos.py`(0.0)、`sensor_relay.py`(0.111)、`sensor_gate.py`(0.214) | 这些文件的契约（返回 None/抛错/写哪个文件）只在代码里 | 低 | 文档改动，人工抽查 |
| P2 | 收敛跨层 import：`feedback_mailbox` 对 control 的依赖改为调用方注入谓词，或把 `check_negative_parking` 移到 control 侧包装 | 感知包反向依赖控制层，分层方向被破坏 | 中：调用方需同步改 | 先记录调用点（`known_scene_inputs.py`），加对照测试再迁 |
| P2 | 与 `common` 一起收敛原子 JSON 写实现（本目录 `sensor_gate.atomic`） | 同一模式 4 份拷贝 | 低 | 逐处替换后跑现有测试 |

本轮只写建议、不改代码；任何跨层迁移都要先有对照测试与调用点清单。

## 关键文件与接口（原"关键文件与接口"）

- `sensor_relay.py`：把 `/livox/imu`、`/demo/camera` 原样转发到 `/demo/teacher/slam/imu`、`/demo/teacher/slam/image`，保留字段与原始时间戳；`--mode sync|async` 只改这两条 writer 的 `publishMode`，并写 `sensor_relay_loaded.json` / `sensor_relay_exit.json`。
- `self_echo_filter.py`：把 `/livox/lidar` 过滤后发到 `/demo/slam/lidar_filtered`，用固定 URDF/scenario 外参和机体包络剔除 Go2 自身点；不读 SLAM 位姿、世界轨迹或真值。
- `sensor_gate.py`：订阅 IMU、原始 LiDAR、图像、CameraInfo、SLAM body odom、配准点云，校验帧、尺寸、K/零 D、图-Info 50 ms 配对、时间戳递增与发布者图，写 `navigation_sensor_gate.json` 并在 `/demo/teacher/navigation/sensor_health` 发布 `ready`/`hold`/`failed`；`WarmupRecovery` 负责初始化与恢复预热。
- `feedback_mailbox.py`：有界不可变 corrected-message 交接（24 slot、16 frame、32 MiB 上限）；intake 组做 join/校验，control 组做反序列化与原始业务回调。
- `ready_commit.py`：环境变量 `R92_READY_COMMIT`（未设置环境变量时默认 `0`；当前普通导航的 `configs/launch/settings.json` 配置为 `1`）控制；用 guard condition 在现有 control 组触发一次 commit，不新增控制 tick。
- `sampling.py`：仅在 fresh、未启动的生成场景上应用已审核采样变体（64 线、480 水平采样、10/20/30 Hz），只改 world 的四个允许叶子；`metadata_qos.py` 决定 corrected metadata 订阅深度。
- `async_writers.py`：`R119_METADATA_ASYNC`（及 body 扩展）在 `rclpy.init` 前预载命名 publisher profile，并用 `writer_qos.so` 读实际 QoS。

## 调用与依赖

[localization_mapping/launch/components.py](../../localization_mapping/launch/components.py) 以子进程启动 relay、self_filter 与 sensor_gate；[launch/prepare.py](../../launch/prepare.py) 调用 `override_sensors`；[localization_mapping/known_scene_inputs.py](../../localization_mapping/known_scene_inputs.py) 与 `navigation/` 使用反馈邮箱、回执和订阅深度。依赖 [common/runtime_io.py](../../common/runtime_io.py)、[common/runtime_telemetry.py](../../common/runtime_telemetry.py)、[common/integration_limits.py](../../common/integration_limits.py)。

## 测试与限制

本目录无功能专用测试；`sensor_gate.py` 仅被 [tests/unit/test_ros_shutdown_lifecycle.py](../../../../tests/unit/test_ros_shutdown_lifecycle.py) 作为 AST 源码检查关停生命周期，其余模块未直接覆盖。运行需要完整 ROS 图与原始 `/livox`、`/demo` 话题；本会话未启动 ROS/仿真。
