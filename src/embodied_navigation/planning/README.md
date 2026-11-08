# planning

本目录是规划侧：SCAN 规划器子进程的拥有与重启、SCAN 输出与所发参考之间的 provenance 契约，以及路径准入、物理几何证明与 PCT 参考路线生成。

## 边界

- 做：拥有一个 frozen SCAN planner 子进程（`scan_epoch_supervisor.py`）；校验子进程启动、真实消费与发布 ACK（`child_startup.py`、`scan_child_consumption.py`、`scan_reference_protocol.py`）；把 SCAN Bspline 与当前参考绑定（`trajectory_contract.py`、`reference_protocol.py`）；生成 bounded SCAN 参数（`scan_runtime_parameters.py`）；纯 polyline 前瞻（`route.py`）。
- 不做：不执行控制、不判定到达；规划输出只是候选，运动许可仍由控制/导航持门禁。

## 关键文件

- `scan_epoch_supervisor.py`：唯一拥有并只 signal 自己保存的 SCAN 子进程；navigation frame 事务时重建 grid，发布 `/demo/navigation/scan_epoch_ready` 与 `scan_epoch_history.jsonl`，并检查两路输入 header 新鲜度（<0.3 s）。
- `scan_runtime_parameters.py`：固定 `fsm.planning_horizon=0.65 m`、grid 分辨率 `0.08 m`、滑动地图 10×10×5 m 等参数；R85 只追加 external mode3 protocol 开关。
- `trajectory_contract.py`：`TrajectoryAssociation` 要求 Bspline payload 与 metadata 完全一致、traj id 前进；只证 provenance，不证碰撞自由或路线进度。
- `reference_protocol.py`：`R85TrajectoryAssociation` 把 SCAN 候选与 exact sent reference、optimizer receipt 连接，不是执行 ACK。
- `admission/`、`reference/`、`pct/`、`launch/` 见各自 README。

## 输入输出与调用

输入是 canonical SLAM 位姿/点云、route/request 与既有 SCAN 配置；输出是 SCAN 轨迹（Bspline/metadata）、准入证明与参考 JSON。SCAN 算法本体在 `../../../third_party/README.md` 所列的包中，由 `launch/scan.launch.py` 启动的 supervisor 拉起仓库根 install/ros 中的 `scan_planner_node`，当前 launch 传入的配置是 `<ros_prefix>/share/scan_planner/config/planner.yaml`，默认位于仓库根 `install/ros`；`../../../third_party/scan/plan_manage/config/planner.yaml` 是源码树中的配置。

## 测试

`route.py` 由 `../../../tests/unit/test_control.py` 覆盖（退化段显式报错，正常段参与 PID 等价对比）；其余顶层文件没有专用单测。

## 限制

仓库根的 install 缺 SCAN 二进制只表示本机未构建；完整路线、跨楼层与实机未验证。
