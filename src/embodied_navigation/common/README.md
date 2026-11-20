# common：跨层公共基础

`common` 是七层架构中的横切基础设施：提供共享几何/时间工具、有界后台写盘、子进程日志截留、状态来源契约与运行遥测空实现。它不实现业务状态机、控制律或 ROS 节点，也不读 `configs/`。

盘点日期 2026-11-20（分支 `refactor/06-clean-base`，HEAD 5b880b2）。

## 模块定位与架构层级

- 架构层级：第 7 层「基础设施」中的公共库（分层定义见仓库根 `AGENTS.md` 第 2 节），被其余各层与 `scripts/` 单向依赖。
- 输入：上层模块 import 调用；`process_log.read_log_config()` 读取启动脚本冻结在 run 目录的日志预算 JSON。
- 输出：纯函数/类与写盘原语；`process_log.run_logged()` 托管子进程并写 `stdout.head.log`、`stdout.tail.log`、`errors.head.log`、`errors.tail.log`、`summary.json`。
- 上游/下游：调用方分布在 `navigation/`、`robot/`、`simulation/dynanav/`、`scripts/` 与 `tests/`（逐文件表给出具体文件）；本目录不调用任何业务模块。
- 边界：包内只有一条同层依赖 `process_log.py → runtime_telemetry.py`，其余文件只依赖标准库/numpy；没有顶层 ROS import（`runtime_io.latest_sensor_qos()` 在函数内 import rclpy）；不读环境变量、不自己决定输出路径、不提供功能开关。

## 当前文件夹结构

```text
src/embodied_navigation/common/
├── time/
│   └── __init__.py       # 15 行：float32 传感器时钟量化容差
├── README.md             # 本文件
├── __init__.py           # 空包标记（0 行）
├── geometry.py           # 61 行
├── process_log.py        # 227 行
├── runtime_io.py         # 253 行
├── runtime_telemetry.py  # 100 行
└── state_contract.py     # 140 行
```

8 个跟踪文件（7 个 `.py` + 本 README），共 796 行；`time/` 是包内唯一的子包。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：模块定位、逐文件职责、测试与配置注意。 | — |
| [__init__.py](__init__.py) | 空包标记，无重导出。 | 无 |
| [geometry.py](geometry.py) | 共享纯几何：角度归一、四元数→旋转矩阵、位姿校验、AABB–OBB 分离。 | `wrap_angle`、`quaternion_matrix`（已归一化 w,x,y,z）、`pose_transform`（校验有限 XYZ 与单位 WXYZ 四元数，返回 4×4）、`aabb_obb_disjoint`（分离轴测试）；被 8 个 `src/` 文件与 1 个测试导入 |
| [process_log.py](process_log.py) | 有界截留子进程 stdout/stderr，不改子进程寿命与退出码。 | `read_log_config`（校验 `head_bytes`/`tail_bytes`/`error_head_bytes`/`error_tail_bytes` 为正整数）、`run_logged`（`start_new_session=True` 拥有进程组、转发 SIGINT/SIGTERM、`selectors` 排水、约 1 s 落一次 checkpoint）、`exit_like_child`（负返回码恢复为信号退出）；调用方 `scripts/run_with_bounded_log.py`、`simulation/dynanav/experiment.py` |
| [runtime_io.py](runtime_io.py) | 有界后台写盘原语与关停辅助；传感器回调不等待磁盘。 | `atomic_write_json`（同目录 `mkstemp` + `os.replace`，当前 HEAD 唯一有仓库内调用方的函数：`scripts/explore_waypoint_client.py` 6 处）、`EvidenceWriter`（4096 有界队列，满则 `RuntimeError`）、`LatestCommandWriter`（一条在途 + 可替换 pending，不排队）、`BackgroundCheck`（周期完整性检查）、`latest_sensor_qos`、`bounded_shutdown_signals` |
| [runtime_telemetry.py](runtime_telemetry.py) | 默认关闭的诊断记录空实现，只有 `atomic` 真正写 live status。 | `RuntimeTelemetry`（`recording_enabled=False`，`enqueue/append/close` 为 no-op，`atomic` 写 `<path>.tmp` 后 replace）、`RuntimeEvents`（只计数）、`NullHistory`、`NullStream`、`NoRecordingContext`；调用方 `common/process_log.py`、`simulation/dynanav/capture_go2_rgbd.py`、`simulation/dynanav/native_waypoints.py` |
| [state_contract.py](state_contract.py) | 进程内 typed 来源/时间契约：谁产出的状态可以用于什么用途。 | `StateOrigin`（`simulation_truth`/`slam_estimate`/`real_sensor`）、`StatePurpose`（`simulation_safety`/`evaluation`/`navigation`）、`StateProvenance`、`validate_state_provenance`（拒绝缺身份、跨 run/时钟域、真值用于导航，以及把 `GAZEBO_NATIVE64_SOURCE` 重标为非 `SIMULATION_TRUTH`）、`PhysicalSafetyState`、`FORBIDDEN_POSE_SOURCES`；仓库内实际只用到 `FORBIDDEN_POSE_SOURCES`（`simulation/dynanav/operator_map.py`、`scripts/explore_waypoint_client.py`、测试） |
| [time/__init__.py](time/__init__.py) | 传感器/仿真时钟的 float32 量化容差。 | `float32_time_half_ulp(time_s)`（半 ULP）、`float32_sensor_time_tolerance(duration_s)`（`max(2e-6, 半 ULP) + 1e-7`，拒绝非正/非有限时长）；调用方 `simulation/dynanav/capture/physics.py`、`simulation/dynanav/experiment.py`、`simulation/dynanav/native_waypoints.py` |

`process_log` 的额外行为（读代码确认）：stdout 命中 `[Error]`/`[Fatal]` 或 `Traceback (most recent call last):` 的行归入 errors 桶，stderr 一律归入 errors 桶；只精确抑制 Installed SDK 的 PhysX `PxShape::getMaterialFromInternalFaceIndex` 警告（首条保留、后续计数），其他警告原样保留；`summary.json` 记录子进程 pid/返回码、转发过的信号与计数、四个字节预算、head/tail/overwritten 字节数、`target_warning` 计数与 `recording_error`。写盘失败时只打印一次告警到 stderr，不改变导航行为，下一次 checkpoint 重试。

`git grep` 负向结论（2026-11-20 实测）：`runtime_io` 的 `EvidenceWriter`/`LatestCommandWriter`/`BackgroundCheck`/`latest_sensor_qos`/`bounded_shutdown_signals` 与 `state_contract` 的 `StateProvenance`/`validate_state_provenance`/`PhysicalSafetyState` 只命中定义本身，当前 HEAD 没有仓库内调用方；`StateOrigin.REAL_SENSOR` 是保留枚举，没有实机后端。

## 调用与依赖（`git grep` 实测，2026-11-20）

| 文件 | 仓库内调用方 |
| --- | --- |
| `geometry.py` | 8 个 `src/` 文件：`navigation/robogauge/route_tracker.py`、`navigation/yaw_action.py`、`robot/unitree/policy/native/observation.py`、`simulation/dynanav/{experiment,native_waypoints,operator_map,rtx_deskewed_perception,truth_deskew}.py`；外加 1 个测试 |
| `process_log.py` | `scripts/run_with_bounded_log.py`（`read_log_config`/`run_logged`/`exit_like_child`）、`simulation/dynanav/experiment.py`（只用 `read_log_config`） |
| `runtime_io.py` | `scripts/explore_waypoint_client.py`（只用 `atomic_write_json`，6 处） |
| `runtime_telemetry.py` | `common/process_log.py`、`simulation/dynanav/capture_go2_rgbd.py`、`simulation/dynanav/native_waypoints.py` |
| `state_contract.py` | `simulation/dynanav/operator_map.py` 与 `scripts/explore_waypoint_client.py`（`FORBIDDEN_POSE_SOURCES`）、1 个测试 |
| `time/__init__.py` | `simulation/dynanav/capture/physics.py`、`simulation/dynanav/experiment.py`、`simulation/dynanav/native_waypoints.py` |

## 验证与测试入口 / 配置注意

- 直接覆盖本目录的测试：[tests/unit/test_truth_deskew_and_mode.py](../../../tests/unit/test_truth_deskew_and_mode.py)（`geometry.quaternion_matrix` 与 `FORBIDDEN_POSE_SOURCES`）、[tests/unit/test_dynanav_sensor_clock.py](../../../tests/unit/test_dynanav_sensor_clock.py)（经 `simulation/dynanav/experiment` 再导出的 `float32_sensor_time_tolerance`）。
- `process_log`、`runtime_io`、`runtime_telemetry` 没有本目录专用单测，只经 `scripts/run_with_bounded_log.py` 与 `simulation/dynanav/` 的路径间接使用；[tests/unit/test_process_log_routing.py](../../../tests/unit/test_process_log_routing.py) 测的是 Isaac Kit 日志路由，不导入本目录。
- 全量 CPU 测试：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`（文档盘点时基线 646 条 0 失败，见 `AGENTS.md` 第 4 节）。
- 配置：本目录没有 `configs/` 读取点，也没有环境变量开关。唯一外部输入是 `process_log` 的日志预算 JSON，由 `scripts/run_dynanav_native_exploration.sh` 冻结写入 `$RUN/configuration/capture_log.json`（四个键各 4–28 MiB），再经 `--capture-log-config` 传入。
- 产物：本目录不决定输出路径。`process_log` 的 head/tail/summary 写在调用方给的 `--output-dir`（启动脚本用 `$RUN/capture_logs`），`runtime_io`/`runtime_telemetry` 的路径全部由调用方传入；运行产物不入库（`AGENTS.md` 第 6 节）。
