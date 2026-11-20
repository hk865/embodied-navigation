# Native 单航点失败恢复，2026-11-11

## 结论

静态 native 探索现在把单航点失败作为该目标的反馈：立即零命令，清除该目标和已排队目标，继续仿真、RGB、RTX 与 SLAM，等待模型观察后明确提交新目标。底层仍是原有 `RouteTracker → RoboGauge`，没有增加航点门禁。显式 stop、整个有限运行的预算及原有物理/时钟检查仍保持各自语义。

实际回归已经完成“失败 → 零命令持续等待 → 同场新目标到达 → 显式 stop → 停车退出”，源 USD 未变，所有本次拥有的进程已释放，没有提交或推送。

## 历史问题的具体证据

目录：`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T102621Z`。

- seq 27：目标 `[14.0, 1.5, 4.61]`，`livo_post_vio` 坐标。
- 第一次失败控制记录：sim **1297.135 s**，source **1297.0 s**，`route_low_progress_stall`。原生 8 s 窗口前向进度只有 **0.0002594206 m**，小于原来的 0.02 m；当时 sim age=0.135 s、wall age≈0.002355 s，并非里程计陈旧。
- 随后 254 条控制和 policy 记录全部零命令，sim 1297.135→1302.195。1302.215 s 满足旧的终止后至少 5 s、停稳至少 1 s 条件并退出，早于 1795 s motion budget。
- `result.json` 保存 `navigation_terminal_request.failure=route_low_progress_stall`、`end_reason=navigation_terminal_and_parked`。没有主动 stop。
- `lifecycle.log`：10:53:39.401656227Z capture exit 0，10:53:40.041005686Z cleanup。SLAM 随 launcher 清理终止。

应以该次 `capture/source_snapshot` 解释历史，不能用已修复的工作树反推：

1. 冻结 `native_waypoints.py:284–287` 把 `tracker.failure` 写入 adapter 全局 `_failure` 并返回 terminal。
2. 冻结 `capture/control.py:399–402` 将其提升为 `source_terminal_reason`，471–472 清零，517–547 停车后要求结束。
3. 冻结 `capture_go2_rgbd.py:640–646` 据此 break，随后关闭视频、ROS 和仿真。

## 改动及反馈协议

- `src/embodied_navigation/simulation/dynanav/native_waypoints.py:114–132`：`_fail_waypoint` 保存 seq、目标、原生失败原因、当前 sim/source time、估计位置、零命令和 `run_continues=true`；保留原生事件，清空 active/queued 并回 idle。已排队目标各自收到 `waypoint_dropped`，不会失败后偷偷恢复运动。
- 同文件 observation 增加 `waypoints.failed_goals` 和 `waypoints.last_failure`。单目标失败不写全局 `failure` 或 `terminal_reason`，旧失败回执不会被删除。
- 同文件 `:134–141,251–252,317–320`：sim 或 source time 达到原有 motion budget 仍生成全局 terminal。包括允许的微小未来时间容差内 source 先达预算的边界，不会误作可恢复目标失败。
- `scripts/explore_waypoint_client.py:103`：官方 waiter 识别准确 seq 的 `waypoint_failed`，立即返回 failed/exit 1，不再把失败一直显示为 pending。observe summary 同时显示失败计数和最后失败详情。
- `/home/hyh001/.codex/skills/dynanav-explore/scripts/dynanav_sop.py:114–157`：优先找该 seq 的原始关闭回执；旧目标失败不会关闭新 seq。没有明确 terminal 证据的单独 failure 字段不会被误报为全局退出。skill 入口和接口说明同步了恢复语义。

没有改动 `navigation/robogauge/route_tracker.py` 的控制、到达、低进展或横向偏离判断，也没有改 SCAN/PCT、学习策略或新增准入限制。

## 完整运行验证

目录：`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/waypoint_failure_resume_20261111T110648Z`，domain 227，有限 180 s 预算，1 m/s 命令上限。

这是**控制状态恢复回归，不是自主探索或覆盖成绩**。先查看该次真实前视 RGB，使用当前估计位姿/朝向构造前方 5 cm、故意高 0.5 m 的目标，阻止正常到达，实际触发的是原生低进展失败。未用真值、USD 或原始地图选点。

- seq 1 在 **39.335 s** 失败，source 39.2 s，反馈 `route_low_progress_stall`。
- **39.335→66.595 s，共 27.26 s**：1,364 个控制帧全部 idle/无全局终止/零命令，匹配的 1,364 个 policy request 和 actual command 全零。
- 这一等待期间持续收到 **273 个不同 post-VIO source stamps**，39.2→66.4 s；持续产生 **14 帧前视 RGB**，source 40.4→66.4 s。不是冻结仿真或仅重写旧观测。
- 仅作离线停稳检查：失败后 0.5 s 起，原生 3D 速度峰值 **0.009634 m/s**，到下一次选择位置漂移 **0.003191 m**，body contact=0。原生状态未进入目标选择。
- 重新查看 RGB，再由当前估计位姿/朝向选前方 **0.6 m**、保持当前 pose-origin 高度，**66.595 s** 提交 seq 2；**70.255 s** 收到 `waypoint_reached`。估计位移 0.427156 m，离线原生位移 0.428238 m，符合保留的原生约 0.2 m 到达捕获范围。
- 显式 stop seq 3 被接受，最后 sim **75.315 s**，`end_reason=navigation_terminal_and_parked`，`outcome=completed`，error=null，capture exit **0**。
- 最后四足接触、body 无接触，速度接近零。owned PIDs 1312138/1311809/1313886 均无进程，GPU 无 Isaac；原独立探索对话可以用当前代码和 skill 接手，未自动启动另一场探索。
- 这里只验证短段恢复，实际短段速度峰值约 0.314 m/s；1 m/s 是请求上限，不能把本回归说成持续 1 m/s 巡航。

证据：`recovery_validation.json`、`waypoint_failure_recovery_audit.json`、原始 interactive/control/policy/RGB journals、`result.json`、`lifecycle.log`。视频 `capture/multiwaypoint05_overview_and_rgb.mp4`，752 帧、75.2 s。

源场景 SHA256 在验证后仍为 `26954f819bc4b3441254a3f1e966e5e1abc55092972537f57ab04dff51c1a38f`，复用之前授权的 session 碰撞覆盖修复。

## 保留的其他尝试和测试边界

- `waypoint_failure_recovery_20261111T110129Z`：Isaac 在 asset_loading 阶段原生崩溃，日志 `corrupted double-linked list`，未执行物理步或发航点。只清理已核验属于本次运行的挂住孤儿进程，保留 `startup_failure_cleanup.json`；不能将其视为控制回归成功。
- `waypoint_failure_recovery_retry_20261111T110337Z`：已验证失败后继续 23.68 s 直到该次 100 s 预算结束，但第二点未在预算前提交，不能作为完整接替验证。随后才做上述完整回归。
- 最终 unit **1,053 tests，OK，3 skips**；regression **108 tests，OK，8 skips**；SOP 4 个关闭回执/终态区分情形通过；skill validator 和 `git diff --check` 通过。日志保存在完整运行目录的 `unit_final.log`、`regression_final.log`、`sop_semantics_tests.log`。
- 独立只读复核确认预算边界、失败当 tick 零命令、旧队列清除和旧 seq 不污染新 seq。仍保留一个既存 CLI 边界：取消/stop 后被取消的 goto 缺少单独 closing receipt 时，旧官方 waiter 可能 pending；SOP 已用后续 cancel/stop ack 区分，未把这一旁支改成新的控制机制。
