# planning/reference

本目录是运行时的参考引导层：把注册路线或 PCT 折线转成方向、速度预算、投影与路线后缀，供控制与准入使用；它只提供先验，不授予运动或到达。

## 边界

- 做：规划 guide（`global_route_reference.py`）、几何方向与速度预算（`spatial_reference.py`）、一次性全局参考与局部重规划 wire（`automatic_reference_runtime.py`）、从实测位姿选不可变路线后缀（`route_suffix.py`）、冻结先验的 3D 走廊（`route_fence.py`）。
- 不做：不做碰撞/支撑证明，不替代 SCAN 实测样本、native guard、区域与截止时间契约。

## 关键文件

- `global_route_reference.py`：`SegmentGuides` 按目标身份冻结 measured activation→original goal 的规划 guide（间距 1.0 m、最多 64 点）；`StableProjectionController(Controller)` 让预览与实际控制用同一条 checked-curve 投影；`direction_check` 在 0.8 m 弧长上审核方向一致性（上限 0.2 rad）。
- `spatial_reference.py`：`reference` 给出方向与 normal/cross 误差；`speed_budget` 用 `|k|·v ≤ yaw 余量`、`|k'|·v²+|k|·|dv/dt| ≤ slew 余量` 收紧速度。参数在 `../../../../configs/control/settings.json` 的 `cascade.spatial_reference`：弧长 0.2 m、yaw 余量 0.12 rad/s、yaw 加速度余量 0.2 rad/s²。
- `automatic_reference_runtime.py`：`AutomaticReferenceRuntime` 按 profile 的 `r85_auto_reference_contract.reference_file` 读取 run 内文档；类的缺省名是 `r85_global_reference.json`，当前 `launch/prepare.py` 明确设置为 `global_reference.json`，校验 epoch/PID/generation、body-ground 0.32 m 与 SCAN transport 0.4 m 偏移、terminal heading；`AutomaticRouteFence` 供 `pid_navigation` 使用。一次发送全局参考后只请求 scoped local replan，不持运动权。
- `route_suffix.py`：`select_route_suffix` 用最近 3D 段投影选不可变路线后缀（resume 用），不搜索新路线、不改起点与终点。
- `route_fence.py`：`tube_evidence` 给出相对冻结的 sensor-registered 先验的水平 0.45 m、高度 0.3 m 走廊证据。

## 输入输出与调用

输入是 `global_reference.json`、anchor、checked SCAN 曲线与实测位姿；输出参考文档、guides、方向审核与速度预算。主要调用方是 `../../navigation/pid_navigation.py` 与 `../../control/tracking/cascade_core.py`。

## 测试

本子包无专用测试，逻辑通过导航链路使用；改动需自行补测。

## 限制

guide 只是规划先验；`global_reference.json` 由 `../../navigation/mission/mission_runtime.py` 生成并依赖仓库根数据入口 assets/geometry 的 `route.json` 数据，完整路线未验证。
