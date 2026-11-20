# ADR-0001: LIVO 独立 `yaw` 清阻塞与 ≤ 0.2 s 点云融合重叠（暂不重写控制链路）

- 状态：已采纳（2026-11-20）
- 相关分支 / 提交：`fix/livo-yaw-and-imu-patch`（`5049ec7`）
- 实测证据：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261120-yaw-and-imu-fix/livo-yaw-04`（对比基线 `runs/20261120-main-smoke/livo-frontend-smoke-01`）

## 1. 背景

默认 live FAST-LIVO2 模式下，独立原地转向（`explore_waypoint_client.py yaw --delta-deg 180`）在基线中 81% 的 tick 触发 `rotation_space_stale`（仅 19% 有效转向占空比），导致 50 s 内几乎未转动并超时失败。

实测拆解表明，`max_sim_age_s = 0.25 s` 安全门本身并非根源，而是 6 段延迟叠加消耗了全部预算：

1. FAST-LIVO2 以 200 ms 成对突发输出（位姿到达时 `sim_age` 已为 `0.055 s`，老化到下一批前触及 `0.215 s`，天生仅余 `0.035 s` 缝隙）；
2. `pct_scan_worker.pair()` 在主线程先跑 ~35 ms 纯 Python 体素字典循环再转发点云给 SCAN；
3. `pct_scan_worker.poll()` 每 5 s 同步执行 `np.savez_compressed` 冻结主线程 ~208 ms；
4. `pct_scan_worker` 仅在 50 ms `poll()` 定时器发送 `rotation_query`，而非位姿到达即发；
5. SCAN `GridMap::occ_timer_` 为 50 ms，且 5 cm 网格 `raycastProcess()` 对已访问空闲体素重复执行 3D 线段-方块求交（~70 ms/帧）；
6. 每次新点云到达时 `minimum_cloud_stamp_ns` 立即跳到最新未融合点云，迫使新 `body_pose` 死等整帧体素光线投射完成。

## 2. 决策

**保留现有隔离安全架构（不放松 `max_sim_age_s = 0.25 s`，暂不重写文件邮箱与控制主干）**，仅消除沿途阻塞并解耦运行期点云融合等待：

1. `pct_scan_worker`：校验后先转发 `(pose, cloud)` 给 SCAN 再向量化更新 `measured_voxels`；`odometry()` 到达新鲜 VIO 位姿时立即发送 `rotation_query`；每 5 s 地图压缩下放至后台线程并记录异常。
2. `third_party/scan/patches/z_grid_map_latency.patch`：`occ_timer_` 从 50 ms 收紧为 10 ms；`raycastProcess()` 先短路检查 `flag_traverse_ == 0` 再调用 `observedSegmentCrossesVoxel`（占用结果逐比特等价）。
3. 流水线重叠：首次 `clear` 前严格要求当前最新点云入图；进入稳定旋转后，允许 `minimum_cloud_stamp_ns = max(yaw_initial_cloud_ns, body_ns - 200 ms)`，使 `body_pose` 可由 ≤ 0.2 s 内已融合地图立即响应，新点云在后台并行融合（若 LiDAR 或 GridMap 停摆超过 0.2 s 仍自动 fail-closed）。

**放弃的替代方案**：
- 直接放宽 `max_sim_age_s`：破坏安全门，单测与安全契约均不允许。
- 立刻重写 `pct_scan_worker` 文件邮箱为共享内存/直连 ROS，或改造 FAST-LIVO2 200 ms 突发同步节奏：改动面大，会延误 Phase 4 SLAM 后端三路对照，当前性价比低。

## 3. 边界与已知代价

- 实跑 `livo-yaw-04` 验证：独立 `yaw +180°` 稳定返回 `yaw_complete`（误差 `-0.036 rad`，连续驻留 `0.52 s`），有效占空比由 **19% 提升至 76%**，后续 `goto 1.5 m` 正常到达，0 次非足端接触。
- 因 FAST-LIVO2 200 ms 成对突发 + 文件邮箱多跳的固有延迟，仍有约 24% tick 擦边触碰 0.25 s 门限（不影响当前静态场景原地转向完成，但尚未达到 100% 占空比）。
- 首次 `clear` 后 ≤ 0.2 s 的点云融合重叠依赖**静态场景原地旋转**前提（0.2 s 内最大转角 `0.03 rad ≈ 1.7°`，无平移）。

## 4. 重访触发条件

出现以下任一情况时，应重新审视本决策并启动流程级重构：

1. 从 `ref/dynanav-dynamic-pedestrian` 恢复**动态行人/移动障碍避障**（0.2 s 地图滞后不足以拦截近身高速移动障碍）；
2. 控制层需要 >95% `yaw` 占空比或高动态追踪（需消除文件邮箱多跳或引入高频 IMU/VIO 预测）；
3. 修改 FAST-LIVO2 前端发布节奏（消除 200 ms 成对突发）或把 `pct_scan_worker` 热路径迁移至 C++。

## 5. 关联待办

见 [TASKS.md](../TASKS.md)「远期 / 触发式架构优化」中关于文件邮箱重构、FAST-LIVO2 发包节奏排查与动态障碍重审的三项条目。
