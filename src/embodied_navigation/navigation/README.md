# navigation（控制跟踪层）

> 盘点日期：2026-11-20 · 分支 `refactor/06-clean-base` · HEAD `5b880b2`

本目录是控制跟踪层里与平台无关的纯策略库：显式 yaw 动作的控制/完成判决（`yaw_action.py`）与 RoboGauge native-pose 路线跟踪器（`robogauge/`）。两者都只把“已测量的位姿样本”变成有界 body 命令，不接触 ROS、Isaac、策略或执行器。

## 模块定位与架构层级

- 架构层级：第 5 层 —— 控制跟踪层（规划 → 跟踪 → 机器人执行中的“跟踪”环节）。
- 输入：调用方每次 tick 提供的实测位姿/速度/IMU 样本与已准入的动作上下文。`yaw_action` 的输入是 `yaw` 请求参数（相对 `delta_yaw_rad` 或 map-frame 绝对 `target_yaw_rad`）加 profile 阈值；`RouteTracker` 的输入是实测 3D 航点列、初始 XY、speed/预算与每个 tick 的 `(t, p, yaw, speed, omega, ready)`。
- 输出：有界 body-frame 命令。`RouteTracker.tick()` 返回 `[vx, vy, wz]` 加 phase/failure/events；`yaw_action_step()` 返回 `[0, 0, wz]`（本模块只控制 yaw 轴）加完成证据。命令到速度话题的发布、准入、回执、取消与过期都由调用方负责。
- 上游 / 下游：真实装配在仿真侧 —— [`simulation/dynanav/native_waypoints.py`](../simulation/dynanav/native_waypoints.py) 把 mailbox 的 `goto`/`yaw`/`maneuver` 请求接到 `RouteTracker` 与 `yaw_action`；[`simulation/dynanav/pct_scan_waypoints.py`](../simulation/dynanav/pct_scan_waypoints.py) 另复用 `clip`/`wrap` 并继承 `NativeWaypointTracker`；[`simulation/dynanav/capture/control.py`](../simulation/dynanav/capture/control.py) 在实验内直接使用 `RouteTracker`；[`simulation/dynanav/experiment.py`](../simulation/dynanav/experiment.py) 在 interactive profile 里固化 `yaw_action` 配置并把这些源文件哈希进 source snapshot。命令行侧，[`scripts/explore_waypoint_client.py`](../../../scripts/explore_waypoint_client.py) 用 `yaw_request_parameters` 在写 mailbox 前预校验 `yaw` 请求。
- 边界：不 import `rclpy`/Isaac（本目录 3 个 `.py` 都只依赖标准库与 `common/geometry.py`）；不做路径规划、goto 准入、仲裁、任务状态机或停车终态策略；不写运行产物；`yaw_action_step` 明确把 pose/clearance 准入、receipt、cancel 与 expiry 留给调用方。

## 当前文件夹结构

```text
src/embodied_navigation/navigation/
├── README.md          # 本文件
├── __init__.py        # 1 行包 docstring（无重导出）
├── yaw_action.py      # 显式 yaw 动作：配置校验、请求校验、target 冻结与 dwell/instant 判决
└── robogauge/         # RoboGauge native-pose 路线跟踪，见 robogauge/README.md
    ├── README.md
    ├── __init__.py
    └── route_tracker.py
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `README.md` | 本文件：控制跟踪层定位、yaw 契约、测试入口与配置注意。 | — |
| `__init__.py` | 只有包 docstring `Navigation runtime and RoboGauge route tracking.`；无重导出，调用方必须写完整模块路径。 | — |
| `yaw_action.py` | 纯 yaw 动作模块：校验 profile 阈值与外部请求，把相对/绝对目标在首个被准入位姿上冻结，然后按 `dwell`（默认）或 `instant` 模式判定完成并输出有界 `wz`。 | 常量 `LOCAL_ACTION_BOUNDS`（`lateral_mps=0.05`、`yaw_rate_rps=0.15`、`maneuver_max_duration_s=2.0`、`yaw_max_delta_rad=3.15`、`yaw_tolerance_rad=0.05`、`yaw_gain=1.2`）；`finite_number(value, name)`、`yaw_config(profile)`、`yaw_request_parameters(payload)`、`resolve_yaw_target(action, yaw)`、`yaw_action_step(action, config, context, window, *, yaw, now_wall, future_tolerance_s)`。 |
| `robogauge/` | RoboGauge 的 native-pose 路线跟踪子包（`RouteTracker` 纯控制器）。 | 见 [`robogauge/README.md`](robogauge/README.md)。 |

## yaw 动作契约（读码核实）

- **请求二选一**：`yaw_request_parameters()` 要求 `delta_yaw_rad` 与 `target_yaw_rad` 恰好给一个；相对增量 `|delta| <= 3.15 rad`；绝对目标必须带非空 `frame_id`，且调用方要自己保证它是当前估计 map frame。
- **目标冻结**：`resolve_yaw_target()` 只在动作第一次拿到被准入位姿时把相对/绝对请求折算成绝对目标并写回 `action['target_yaw_rad']`，之后不再漂移。
- **profile 阈值**（`yaw_config()`，键都是 `interactive_*`）：`interactive_yaw_completion_mode` 只能是 `dwell`/`instant`；`interactive_yaw_completion_tolerance_rad` 在 `(0, 0.10]`；`interactive_yaw_control_tolerance_rad` 在 `[0, 0.10]` 且不得超过 completion 容差；`interactive_yaw_settle_rate_radps` 在 `(0, 0.15]`；`interactive_yaw_dwell_sim_s` 在 `[0.1, 5.0]`；另有 `interactive_yaw_eval_max_gap_sim_s`（默认 `0.020001`）、`interactive_imu_max_sim_age_s`（0.10）、`interactive_imu_max_wall_age_s`（0.25）、`interactive_imu_future_tolerance_s`（`2e-6`）。废弃键 `interactive_yaw_tolerance_rad` 只映射到 completion 容差，且不能与新键同时出现。
- **完成判据**：`instant` 模式只要 `|error| <= completion` 立即完成；`dwell` 模式要求连续的“有效 sim 时间窗”：pose 必须 distinct 或在 `max_sim_age` 内且 wall age 在预算内，IMU 静止（`|rate| <= settle`）且 gyro 时间戳不早于 `-future_tolerance`、不旧于 `imu_max_sim_age`，窗口累计 `valid_sim_s >= dwell`、有 distinct stamp、source span `>= dwell - 0.05`、gyro span `>= 0.25`；评估间隔超过 `eval_max_gap` 会重开窗口。
- **输出**：只给 yaw 轴。控制带内（`|error| <= control_tolerance`）命令恒为 `0.0`；否则 `wz = clip(1.2 * error, ±0.15)`。完成时返回 `[0.0, 0.0, 0.0]`，并在 completion 里记录 `released_wz=0.0`、`pre_release_wz`、IMU/pose 来源 stamp 与各容差。
- `LOCAL_ACTION_BOUNDS` 里的 `lateral_mps` 与 `maneuver_max_duration_s` 在本文件内没有读取点（`git grep` 只命中定义处）；maneuver 的 1.0 m/s、0.05 m/s、0.15 rad/s、2.0 s 上限由客户端在 `explore_waypoint_client.py` 校验，本模块只负责 yaw。

## 与调用方的职责边界

| 事项 | 归属 |
| --- | --- |
| 目标/动作准入、TTL、heartbeat、receipt、cancel/stop 终态 | 调用方（[`simulation/dynanav/native_waypoints.py`](../simulation/dynanav/native_waypoints.py)、[`simulation/dynanav/pct_scan_waypoints.py`](../simulation/dynanav/pct_scan_waypoints.py)） |
| pose 来源与坐标系一致性、后向 clearance、命令发布与执行链 | 调用方与机器人执行层 |
| 连续 SCAN 曲线跟踪（`ContinuousScanFollower`） | 不在本目录，在 [`simulation/dynanav/pct_scan_waypoints.py`](../simulation/dynanav/pct_scan_waypoints.py) |
| 纯 yaw 判决、纯路线跟踪、有界命令与 phase/failure/events | 本目录 |

## 验证与测试入口 / 配置注意

- [`tests/unit/test_yaw_shared_control.py`](../../../tests/unit/test_yaw_shared_control.py)：相对/绝对请求选择同一最短纯 yaw、增益上限与控制带独立于 completion、相对目标在首个位姿冻结不漂移、source+gyro 静止窗口 0.5 s 完成、重复位姿/未推进 gyro 不能靠墙钟完成、高 yaw rate/无效 gyro/评估间隔重开窗口。
- worker 侧 yaw 集成（使用真实 worker 回调整套 fixture）：[`tests/unit/test_yaw_worker_bridge.py`](../../../tests/unit/test_yaw_worker_bridge.py)、[`tests/unit/test_yaw_blocked_latch.py`](../../../tests/unit/test_yaw_blocked_latch.py)、[`tests/unit/test_yaw_delayed_cancel.py`](../../../tests/unit/test_yaw_delayed_cancel.py)（fixture [`tests/unit/yaw_acceptance_fixtures.py`](../../../tests/unit/yaw_acceptance_fixtures.py)）。
- `RouteTracker` 的回归与集成测试见 [`robogauge/README.md`](robogauge/README.md)。
- 命令（仓库根执行）：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 配置注意：本目录不读 `configs/`；所有阈值经实验 profile 传入 —— `experiment.py` 在 interactive 模式调用 `yaw_config(profile)` 得到 `profile['yaw_action']` 写进 run 的 `experiment.json`，并把 `navigation/yaw_action.py`、`navigation/robogauge/route_tracker.py` 与 `common/geometry.py` 一起做 sha256 记入 source snapshot；运行时以 run 目录冻结的那份为准。
- 限制：`yaw_action` 与 `RouteTracker` 都只做纯计算，行为级验证依赖单测/回归与调用方回放；完整 Isaac 闭环与实机未在 CPU 测试中验证。
