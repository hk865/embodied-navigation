# 真实 RTX LiDAR 生产感知/局部地图接线与有限航点控制

> **勘误（2026-11-11）：撤回“世界固定无回波证明合法外侧开口”的解释。** 同出生点 `[-14.53,-7.5,-2.85]`、yaw=π 的历史 USD 图像有完整办公室楼梯、墙与扶手，而当前 RTX/Fabric 图像缺失这些几何，仅见平台与背景。无回波不能证明开口，前述短运动也不构成办公室地图或探索验收。原始运行数据保留为问题证据；新对照与修复状态见 [RTX 场景渲染诊断](dynanav-rtx-scene-rendering-20261111.md)。

日期：2026-11-11（Asia/Shanghai）。本轮把真实 Ouster OS1 RTX LiDAR 经 FAST-LIVO2 的
native IMU deskew 世界点云接入**生产**感知与 operator map，并完成一次真实站立验证与
一次有限前向航点控制。未 commit / 未 push；全部旧实验目录与用户未提交改动保留。

## 1. 关键结论（先说可核事实）

- 生产链路已接通：真实 `/dynanav/rtx_livo/cloud_registered_full`（FAST-LIVO2
  handleLIO 阶段的 native deskew 世界云，`dynanav_lio_map` 系，PointXYZINormal，
  point_step=48）与**同一阶段、同一整数 stamp** 的 `/dynanav/rtx_livo/aft_mapped_to_init`
  （child `go2_imu`）配对后，进入生产 `perception.json` 与 `operator_map`。
- 独立 collector 证据（run G）：208 个 native 完整云 / 227 个 LIO pose，**208/208 云
  stamp 与 pose stamp 精确整数相等**（full-only 0，lio-only 19 为无对应云的 pose）；
  云字段集合为 `x,y,z,normal_x,normal_y,normal_z,intensity,curvature`，point_step=48；
  curvature 跨度 0.072–99.998 ms（非零逐点时间，排除"raw 伪装"）。原始 wire
  （`x,y,z,intensity,t,reflectivity,ring,ambient,range`，point_step=32）在合成测试与
  内容校验中均被拒绝。
- 生产消费者而不是 helper：`scripts/explore_waypoint_client.py map
  --cloud-semantics lio_deskewed_map` 把云以 `frame="map"`（不再二次变换）写入
  OccupancyAccumulator，同时写 `perception.json`；run G 中 map **192/192 帧接受、
  0 拒绝**，floor evidence `established_floor`（support 8894 点，span 22.5×17.9 m），
  coverage 167.74 m²，unknown 估计 52.68 m²。
- 真实控制已发生：一次 2 s 前向 maneuver 与一个 0.30 m 前向航点由确定性 50 Hz
  底层执行，run G 收到 `maneuver_complete` 与 `waypoint_reached`。实际位移（独立测量）：
  **command window 内 native GT（仅离线评分）0.1276 m，post-VIO 0.1389 m**；整段
  post-VIO net 0.1307 m。没有把名义命令积分当位移。
- 仍未达到 1 m/s 巡航，也未完成整层/整栋探索；本轮只有约 13–17 cm 的有限运动。

## 2. 生产改动

- **新增 `src/embodied_navigation/simulation/dynanav/rtx_deskewed_perception.py`**
  （唯一的 deskew 云消费契约，纯 numpy）：
  - topic 角色固定：full 云必须是 `cloud_registered_full`、姿态必须是同阶段
    `aft_mapped_to_init`；raw `/dynanav/lidar_rtx` 与 post-VIO topic 被拒。
  - 内容校验：拒绝 raw wire 字段（t/ring/range/ambient/reflectivity），要求 native
    PointXYZINormal；要求 curvature 时间跨度非零（native deskew 切片证据）。
  - 配对：cloud/pose 必须**整数 ns 完全相等**、同一 map frame、child=`go2_imu`、
    stage=LIO；1 ns 差异视为不同 pair，绝不插值。
  - 几何：`p_imu = R^T(p_map - t_map_imu)`，`p_base = p_imu + imu_mount`（恰一次）；
    operator map 侧以 `frame="map"` 消费、不再变换。
  - 地面/障碍：用同一帧点云拟合**倾斜地面平面**（含机体 6° 级倾斜，run G slope
    6.54°，残差 5.6 mm），floor 不进碰撞带；每方向 90° 走廊要求方位 bin 与径向 bin
    覆盖、地面支持、drop/void 检测；近场地面盲区按测量值披露
    （run G `[0, 1.19] m` 内无地面观测，第一可见径向 bin 前移到 1.675 m 仍记为未知）。
  - 输出含 stage、整数 reference/cloud/pose stamp、topic、binary sha、blind sector、
    floor plane、age 等可审计字段；缺失即 fail-closed。
- **`capture/frames.py`**：RTX 交互模式下 `interactive_perception_owner=rtx_deskewed_cloud`
  时 capture **不再**每帧用 `rtx_lidar_perception_not_connected` 覆盖
  `perception.json`，只读取外部样本的 source/stamp/age 作为收据；无新云时文件自然
  变旧，tracker 原 age 门继续 hold（不续命）。同时新增真实 RGB 截图（source time +
  perception 摘要，不含 pose/GT）。
- **`capture/experiment.py`**：`--interactive-perception-owner {fail_closed|rtx_deskewed_cloud}`
  （默认 `fail_closed` 保持旧行为）与 `--interactive-forward-turn-mode`（默认关）。
- **`exploration_waypoints.py`**：新增地面支持门（front 样本与运动方向；RGB-D 样本无
  该字段→旧路径不变）；显式 forward+turn 生产模式：确定性 follower **只输出 yaw 转向、
  vy 恒 0**，该模式下 lateral maneuver 明确拒绝（不是过门后强压 vy）；默认 holonomic
  follower 与旧侧向门完全不变。
- **`scripts/explore_waypoint_client.py`**：map 新增 `--cloud-semantics`、
  `--perception-out`、`--binary-prefix`；deskew 分支做 content/topic/frame/整数 stamp
  校验、`frame="map"` 入图并写 perception；每个接受帧记录 pair/source binary sha；
  shutdown 与 perception 计数/证据修复。
- 旧 `scripts/dynanav_rtx_deskew_adapter.py` 仅保留为历史 helper 并标注 superseded；
  生产路径不 import 它（其 `/cloud_registered` 是相机 FOV 彩色云，不是导航云）。

## 3. 真实运行

所有 run 使用同一 same-pair build：
`build_cache/dynanav_same_pair_20261110T171010Z`（core sha256 `66724124…f75f6b`，
mapping sha256 `0f47955a…0d4ec79`），LIVO observer namespace `/dynanav/rtx_livo`，
runtime parameter 查询确认 `same_pair_vio_handoff=true`、`lidar_type=3`、`scan_line=32`、
`require_complete_imu=true`、缓存 32/8192/8192。

| run | 用途 | 结果 |
| --- | --- | --- |
| `rtx_prod_stand_20261110T193519Z` (domain 150) | 默认朝向站立链路验证 | capture 30 s/151 扫；map 94/94 接受；perception 正确 fail-closed（正前方是世界固定的无回波开口）；8 张 RGB 截图 |
| `rtx_prod_control_20261110T193724Z` (151) | 站立 + 控制尝试 | 456/456 云-pose 整数 stamp 精确匹配；map 458/458；front measured (1.85 m)；控制请求在收尾期被原有 age 门 hold |
| `rtx_prod_control2_20261110T193912Z` (152) | 60 s holonomic 尝试 | goto 产生 lateral 指令，被侧向 unknown 门**正确拒绝**（`goto_lateral_uncovered`），随后 low-progress stall；证明旧侧门未被绕过 |
| `rtx_prod_fwd_20261110T194145Z` (153) | forward+turn 生产模式 | `maneuver_complete` + `waypoint_reached`；post-VIO net 0.1697 m、native GT(评分) 0.1418 m；map 140/140 |
| `rtx_prod_g_20261110T195201Z` (156) | **最终代码**专项验证 | 见下 |

最终 run `rtx_prod_g_20261110T195201Z`：
- `forward_turn_mode=true`，spawn heading `3π/2`（见 §5 说明）。
- 链路就绪证据：front `measured` clearance 3.0 m、floor `supported`、方位覆盖 1.0、
  径向 bins `[0,0,667,541]`（前两个 bin 在测量近场盲区内并如实记录）；
  pose sim_age 0.135 s、wall_age 0.018 s。
- `maneuver_complete`(seq1) → `waypoint_reached`(seq2, goal `[0.328,-0.034,0.010]`,
  reached position `[0.143,-0.002,0.002]`, remaining 0.1877 m 在 capture 界内）→ 操作端
  `stop` → `interactive_stop`，run 在 26.9 s 自动结束。
- 实际位移（不用命令积分）：command window [10.6,21.5] s 内 native GT 0.1276 m、
  post-VIO 0.1389 m、LIO 0.134 m；整段 post-VIO net 0.1307 m。
- 运行期 hold（未放宽）：`stale_sim_odometry`（真实 post-VIO 图像管线延迟，
  0.25 s 原门不变）、`perception_stale`（源样本 sim age 偶发 >0.30 s）。
- 地图：192/192 帧接受、perception 192 样本；pair 均为整数相等（last 26.9 s）；
  14 张 RGB 截图（source time，real render）。

## 4. 感知几何实测（为什么正前方会 hold）

- 默认朝向（π）下点云在**世界固定**的 ~65–70° 方位扇区无回波；把机体 yaw 旋转 90°
  后该空缺方位同步旋转 −90°，证明它属于场景（外侧开口/无回波方向），不是传感器缺陷，
  也不是本机固定。RGB 截图显示该方向是建筑外侧天空/草地，LiDAR 射线穿出后无回波。
- 因此默认朝向的"正前方"确实是未知空间：生产 perception 输出
  `insufficient_observation`，tracker hold；这是正确的 fail-closed，而非链路不通。
- 站立时机体约有 6–7° 倾斜，单高度地面阈值会把碰撞带整体偏移（早期版本实测会得到
  −0.14 m 或 −0.69 m 的错误地面）。现改为**实测地面平面拟合**（slope 6.5°、残差
  5.6 mm、floor≈−0.38 m），floor 不进碰撞带。
- 旋转 LiDAR 的近场地面盲区是真实的：本姿态下 [0, 1.19] m 内无地面回波；该段如实
  记为未知（`near_field_floor_unobserved_m`），不伪装成"已观测"。
- run G 最终四方向：front/left measured（clear 3.0 m）、rear measured（2.31 m）、
  right insufficient（方位覆盖 0，落在上述世界固定开口内）。

## 5. 有限控制与边界

- 默认朝向正前方为世界固定的未知开口，且未观测时原 front 门也会阻止原地转向，因此
  本轮把**验证用初始朝向**设为 `3π/2`（该朝向来自真实 LiDAR 扇区覆盖实测，不使用
  ground truth，也不改任何安全门）；位置、地图、SLAM、感知、保护全部照旧。
- 旧 holonomic goto 会为纠偏产生侧向 vy，侧向 FOV 未知时被原门拒绝——这是预期行为。
  本轮新增的显式 forward+turn 生产模式只让 follower 用 yaw 纠偏、vy 恒 0，从而在
  不绕过侧门的前提下完成前向航点。
- 未做/未声称：1 m/s 巡航、整层探索、长期定位精度验收、全向覆盖证明。真实 post-VIO
  管线延迟导致的 sim-age hold 仍然存在，本轮没有放宽 0.25 s/0.30 s 门。

## 6. 可核入口

- 生产代码：`src/embodied_navigation/simulation/dynanav/rtx_deskewed_perception.py`、
  `capture/frames.py`、`capture/experiment.py`、`exploration_waypoints.py`、
  `scripts/explore_waypoint_client.py`。
- CPU 测试：`tests/unit/test_dynanav_rtx_deskewed_perception.py`（21）、
  `tests/unit/test_dynanav_rtx_production_bridge.py`（10）、
  `tests/unit/test_dynanav_interactive_waypoints.py`（含新增地面/cached/forward-turn）。
  全量：unit 836 OK（3 skip）、regression 95 OK（5 skip）；`git diff --check` 通过。
- 运行 harness（保留）：`.dsh-subagents/rtx-prod-harness/`；checkpoint：
  `.dsh-subagents/rtx-production-map-control-checkpoint.json`。
- run G 目录：`$PROJECT_DATA_ROOT/embodied-navigation/experiments/rtx_prod_g_20261110T195201Z/`
  （capture/、observer/、collector/、operator_map/、control_actions.jsonl）。
