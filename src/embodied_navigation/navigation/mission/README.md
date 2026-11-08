# navigation/mission

本目录是任务的目标与到达层：把路线定义变成带区域、到达窗口与通过证据的任务，并把 anchor、request 与 global reference 发布给控制链路；它不产生速度命令。

## 边界

- 做：目标几何与到达判定（`goal_regions.py`）、三段平台与 46 区域定义（`route_regions.py`）、连续路线进度（`continuous_route.py`）、与传输无关的任务迁移（`state_machine.py`）、任务运行时节点（`mission_runtime.py`）、协调门与 stop 事件证据（`mission46_guard.py`、`dynamic_stop_event.py`）。
- 不做：控制、规划与物理安全；到达证据不等于运动许可。

## 关键文件

- `mission_runtime.py`：ROS 节点，由 `../../control/launch/components.py` 以 `mission` 角色启动。等定位、epoch 与健康都 ready 后冻结 `navigation_anchor.json` 和 `navigation_request.json`，周期性发布 `/demo/navigation/request` 与 `/demo/teacher/navigation/anchor`，退出时向 stop 话题发 `True`；读 run 目录的 `global_route.json`，生成 schema 为 r85_pct_scan_global_reference/v1 的 `global_reference.json`。
- `goal_regions.py`：`Goal`、`RigidTransform`、`ArrivalWindow`。目标几何不可变，到达用 measured stamp 的 dwell 判定而非 wall timer；本文件不建 ROS 节点、不接受真值控制输入。
- `continuous_route.py`：`RouteProgress`、`ContinuousMission`、`CausalHandoffProjectionController`，把普通区域完成改为实测通过证据；是否启用由 `../../../../configs/planning/settings.json` 的 `continuous_route_contract.enabled`（当前 `false`）决定。
- `route_regions.py`：`add_route_regions`、`validate_route_regions`、`transformed_route_goals`，在准备阶段校验区域歧义。
- `state_machine.py`、`mission46_guard.py`、`dynamic_stop_event.py`：与 ROS 时间解耦的任务迁移、保持时间戳的 controller 协调门、以及把已发布 stop 事件绑定到 guard 证据（无控制权）。

## 测试

本子包无专用测试；其行为通过 `../../navigation/pid_navigation.py` 的运行链路使用，未在单测中覆盖。

## 限制

`mission_runtime.py` 写出的 anchor 字段集与 `../../planning/admission/path_admission.py` 中 `FrozenFence` 的期望并不一致（见 `../../planning/admission/README.md`），说明该准入路径在本检出中未闭合。连续路线契约默认关闭，完整 46 区域未验证。
