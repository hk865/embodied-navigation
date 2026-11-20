# navigation/robogauge（native-pose 路线跟踪器）

> 盘点日期：2026-11-20 · 分支 `refactor/06-clean-base` · HEAD `5b880b2`

本目录只剩 RoboGauge 的纯 measured-pose 路线跟踪器：`RouteTracker` 每个 `tick()` 吃一个已测量位姿/速度样本，回一个有界 body 命令；它不做策略、不做 ROS 传输，也不拥有准入或终态停车。

## 模块定位与架构层级

- 架构层级：第 5 层 —— 控制跟踪层内的路线跟踪子模块。
- 输入：`points`（实测 3D 航点，米；z 单独校验，楼层不同不能伪造到达）、`start_xy`（stage 0 的起点 XY）、`speed`、`motion_end`（总时间预算，`None` 表示不限）、`stop_on_stall`、`reverse`、capture 边界；每个 tick 的 `t`（仿真秒）、`p`（路线坐标系下的实测 XYZ）、`yaw`、`speed`（调用方速度标量）、`omega`（实测 yaw rate）与 `ready`。
- 输出：`[vx, vy, wz]`：前进速度不超过 `self.speed`，侧向速度绝对值 <= 0.05 m/s，yaw rate 绝对值 <= 0.15 rad/s；另外维护 `phase`、`failure`、`arrived` 与 `events`（corner capture / 停车 / 转向完成 / 低进展 stall）。
- 上游 / 下游：被 [`simulation/dynanav/native_waypoints.py`](../../simulation/dynanav/native_waypoints.py)、[`simulation/dynanav/pct_scan_waypoints.py`](../../simulation/dynanav/pct_scan_waypoints.py)（复用 `clip`/`wrap` 并继承其 adapter）、[`simulation/dynanav/capture/control.py`](../../simulation/dynanav/capture/control.py) 与单测/回归使用；坐标旋转复用 [`common/geometry.py`](../../common/geometry.py) 的 `wrap_angle`。
- 边界：`RouteTracker` 不识别位姿来源与坐标系（调用方负责提供一致的路线 frame 契约），不做路径规划、准入、heartbeat/TTL、任务状态机或终态停车；`reverse=True` 时后向 clearance 由调用方保证。它不产生副作用（无 ROS、仿真、模型、文件 I/O）。

## 当前文件夹结构

```text
src/embodied_navigation/navigation/robogauge/
├── README.md        # 本文件
├── __init__.py      # 1 行包 docstring（无重导出）
└── route_tracker.py # RouteTracker 类，以及 clip / wrap 两个共享别名
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `README.md` | 本文件：控制器定位、相变/失败条件、测试入口与限制。 | — |
| `__init__.py` | 只有包 docstring `RoboGauge native-pose route tracking.`；无重导出，调用方必须写完整模块路径（例如 `from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker`）。 | — |
| `route_tracker.py` | 单实例、单路线的有界航点跟踪控制律：`waiting → cruise → brake → turn → …（下一段）→ arrived`，含 fail-closed 的横向偏离/过冲/漂移/超时/低进展判定。 | `RouteTracker(points, start_xy, speed=0.15, motion_end=230.0, stop_on_stall=True, *, reverse=False, capture_remaining_m=0.20, capture_distance_m=0.30, final_capture_remaining_m=None, final_capture_distance_m=None)`；`invalidate_measurement()`；`tick(t, p, yaw, speed, omega, ready=True)`；模块级别名 `clip`、`wrap`（后者是 `common.geometry.wrap_angle`）。 |

## 控制律与相变（读码核实）

- **构造期校验**：`reverse` 必须是 bool；capture 边界要求 `0 < remaining <= distance <= 0.40 m`；final-only 覆盖值必须是有限数或 `None`，并把有效值再校验一遍（默认值保持原始 0.20/0.30 m）。
- **tick 入口门**：已 `failure` 或 `arrived` 恒返回零；`t >= motion_end` 置 `route_motion_budget` 并返回零；`ready=False` 或 `t < 7 s` 直接返回零（不改 phase）。
- **段几何**：stage 0 的 origin 是 `start_xy`，之后是上一航点的 XY；`u` 为单位方向、`normal` 为左法向；`reverse=True` 时目标 heading 加 π；`error` 是包裹后的朝向误差，`remaining` 是沿 `u` 的有符号剩余距离，`cross` 是带符号横向偏移，`distance` 是到航点的 2D 距离，`z_error` 是垂直距离。
- **cruise**：`|cross| > 0.6 m` → `route_cross_track_bound`；`remaining < -0.35 m` → `route_waypoint_overshoot`（都是 fail-closed，不做纠偏尝试）。三者同时满足才进入 capture：`remaining <= capture_remaining`、`distance <= capture_distance`、`z_error <= 0.18 m`，并记录 `entered_corner_capture`。否则前进命令为 `sign * min(speed, max(0.04, 0.5*max(remaining,0)))`，但仅当 `|error| < 0.25`，否则前进为 0；侧向为 `-0.5*cross` 投到 body 左轴后截断到 `±0.05`；`wz = clip(0.8*error, ±0.15)`。
- **stall**：有前进命令时开 8 s 窗口，若前向推进 `< 0.02 m` 就记录 `observed_low_progress_stall`；`stop_on_stall=True` 才升级为 `route_low_progress_stall` 失败，否则只记录、继续跑到总预算。
- **brake**：`distance > 0.40 m` → `corner_braking_drift`；`wz = clip(0.8*error, ±0.15)`；只有 `speed < 0.04` 且 `z_error <= 0.18 m` 持续 0.5 s 才认定停稳（记录 `corner_stopped`）：最后一段 → `arrived=True`、`phase='arrived'` 并返回零；否则进入 `turn`。brake 超 12 s → `corner_brake_timeout`。
- **turn**：`distance > 0.45 m` → `corner_turn_drift`；`wz` 指向下一段 heading；完成要求 `|next_error| < 0.10`、`|omega| < 0.06`、`speed < 0.04` 持续 0.5 s（记录 `turn_completed`，`stage += 1` 回 `cruise`）；turn 超 40 s → `corner_turn_timeout`。
- `invalidate_measurement()` 在测量间隔过期时清掉 dwell/stall 窗口（不改 phase/failure），让下一帧新鲜样本重新累计。

## 失败码速查（读码核实）

| 失败码 | 触发条件 |
| --- | --- |
| `route_motion_budget` | tick 的 `t >= motion_end`，总运动预算用尽 |
| `route_cross_track_bound` | cruise 中 `abs(cross) > 0.6 m` |
| `route_waypoint_overshoot` | cruise 中沿段越过航点 > 0.35 m |
| `route_low_progress_stall` | 8 s 窗口内前进 < 0.02 m，且 `stop_on_stall=True` |
| `corner_braking_drift` | brake 中距航点 > 0.40 m |
| `corner_brake_timeout` | brake 相持续超过 12 s |
| `corner_turn_drift` | turn 中距航点 > 0.45 m |
| `corner_turn_timeout` | turn 相持续超过 40 s |

## 验证与测试入口 / 配置注意

- [`tests/regression/test_robogauge_route_tracker.py`](../../../../tests/regression/test_robogauge_route_tracker.py)：14 条回归覆盖等待、前向/侧向符号、yaw 包裹、三段完整序列、横向偏离、过冲、stall、report-only stall、总超时、转向超时、坐标 z 不伪造到达、输出边界、过期间隔清窗、延长预算不停在旧 deadline。命令：`PYTHONPATH=src python3 -B -m unittest discover -s tests/regression`。
- 上层集成单测：[`tests/unit/test_dynanav_native_waypoints.py`](../../../../tests/unit/test_dynanav_native_waypoints.py)（mailbox adapter 用 `RouteTracker`）、[`tests/unit/test_native_reverse_waypoint.py`](../../../../tests/unit/test_native_reverse_waypoint.py)（`reverse=True` 与 `PctScanWaypointTracker`）、[`tests/unit/test_dynanav_unlimited_acceptance.py`](../../../../tests/unit/test_dynanav_unlimited_acceptance.py)（长预算/有限预算下 adapter 仍持续跟踪）。命令：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 配置注意：本目录不读 `configs/`，也不写文件；所有参数由调用方构造 `RouteTracker` 时给出，源文件哈希由 [`simulation/dynanav/experiment.py`](../../simulation/dynanav/experiment.py) 的 source snapshot 固定。
- 限制：
  - 一个实例只跟踪一条路线；重规划需要新实例，单目标完成/失败后回到零命令 idle 由 adapter 负责。
  - 模块 docstring 仍提到本检出已不存在的 `SlamRouteTracker`；实际调用方为上列 `native_waypoints.py`、`pct_scan_waypoints.py` 与 `capture/control.py`。
  - 行为验证来自合成位姿回归；完整 Isaac 运行、实机与真实斜坡/楼梯未在 CPU 测试中验证。
