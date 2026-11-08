# planning/admission

本目录是路径准入与静态几何证明层：判断 SCAN 轨迹能否进入控制回路，并对 PCT 折线做静态支撑/碰撞证明；所有输出都只是必要条件，不是运动、到达或执行 ACK。

## 边界

- 做：route-fence 包含性证明（`path_admission.py`）；typed ground/body 静态证书与起点连接器（`static_geometry.py`）；把几何证明绑定到真实准入（`physical_gate.py`）；异步单 owner 证书（`async_admission.py`、`geometry_worker.py`）；替换预览、因果提交与私有绑定（`path_handoff.py`、`pending_path_commit.py`、`path_commit.py`）；固定 FIFO 预算与默认 OFF 开关（`queue_budget.py`、`exhausted_replacement.py`、`replan_ack.py`）。
- 不做：不替代来源新鲜度、航向、停车、native guard 等独立门禁；不新增规划或控制数学。

## 关键文件

- `path_admission.py`：`certify_curve` 用 de Casteljau 分区与 Bézier 凸包球，证明连续 cubic 曲线落在冻结 3D 走廊内；超预算或不确定即拒绝。`FrozenFence.load` 校验 `navigation_anchor.json` 与 scene axis 注册文件。
- `static_geometry.py`：`certify_start_connector`、`certify_physical_curve`；命令行 `python3 -m embodied_navigation.planning.admission.static_geometry {prepare,curve}` 输出连续 receipt，`allowed` 只代表静态几何，不授控制权。
- `validate_route_support.py`：独立 CLI，对 PCT polyline 与端点连接器做有限面/OBB 静态校验，receipt 明写 `ready_for_control=false`、`navigation_PASS=false`；独立 CLI 的默认 `--geometry-metadata` 与 `--config` 仍指向本目录下的 `data/known_scene43_camera_init_h005/metadata.json` 和 `support_validation_config.json`，这些文件未随本检出提供，需显式传参。运行时 `static_geometry.py` 则通过 `common/paths.py` 使用仓库根 assets/geometry 与 `../../../../configs/planning/static_support_limits.json`（采样间距 0.025 m、body 半径 0.25 m、高度带 ±0.12 m 等）。
- `async_admission.py`：`Admission` 通过 `geometry_worker.Worker` 在独立 Python 子进程完成证书，主线程只在原事务中复用结果；由 `R88_ASYNC_PHYSICAL` 启用，有界队列 32 事件 / 2 MiB。

## 输入输出与调用

输入是 SCAN 候选/BSpline、冻结 anchor 与路线走廊、PCT 折线与 `../../../../configs/planning/static_support_limits.json`；输出 allow/reject 证明与证据字典；通过当前默认 `RuntimeTelemetry` 的历史 JSONL 写入为空操作。主要调用方是 `../../navigation/pid_navigation.py`。

## 测试

`../../../../tests/regression/test_navigation_safety.py` 覆盖 `static_geometry.certify_physical_curve` 的允许/拒绝结果（需要私有测量图，缺失时该用例 skip）；该文件虽导入 `certify_start_connector`，没有直接调用它的测试；其余准入文件没有专用测试。

## 限制

静态证明不覆盖跟踪误差、摩擦、步态与实机；私有测量图与几何元数据不在 Git。`FrozenFence` 期望的 `frozen_once`、`navigation_scene_axis_registration.json` 等字段没有对应的写入方——`../../navigation/mission/mission_runtime.py` 写出的 anchor 字段集不同，该路径在本检出中未闭合、未验证。
