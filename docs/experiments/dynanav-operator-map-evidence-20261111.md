# Operator map 来源纠正与 unknown 证据审计

日期：2026-11-11（Asia/Shanghai）。仅 CPU、只读历史实验输入；没有启动仿真、修改场景资产、改渲染或提交/推送。保留原有未提交代码。本文不把旧 USD 视频或当前 RTX 缺几何的运行当作正确办公室地图的背书。

## 本次修复

`scripts/explore_waypoint_client.py::_export_operator_grid` 原先把所有地图硬编码标为
“RGB-D depth-backprojection clouds plus post-VIO poses”。run4 实际使用 native deskewed
LIO full cloud 和同阶段 LIO odometry，这个标签确实错误。

现在导出：

- `source_contract` 分别保存配置的 cloud/odometry topic、pose source、实际 snapshot 的 sources、runtime cloud semantics、deskew stage 和最后一个精确配对回执。配置和观测不混为一个字段。snapshot sources 是快照报告的来源，不声称是逐格完整历史谱系。
- 已验证运行状态为 `lio_deskewed_map` / `LIO` 时，文字准确写 native deskewed LIO map-frame clouds / same-stage LIO poses；其余情况只写 measured clouds / estimator poses，不从 sensor/IMU 坐标系猜 RGB-D 或 post-VIO。缺来源时保留空 contract，不编造。
- `free_space_semantics` 明确记录实际 `free_mode`、`free_is_traversability_certificate=false`：floor 命中和 XY ray 投影不等于连续地面支撑、机器人足迹净空或安全路径。
- 新增 `--free-mode auto|floor|rays|both|none`，默认 `auto`：`lio_deskewed_map` 使用 **floor-only**；旧 `sensor_imu` 仍为 **both**。LIO 显式要求 `rays` 或 `both` 会在 ROS 启动前报 `ValueError: ... cannot safely satisfy ... deskewed_cloud_without_physical_lidar_ray_origin_contract`，不静默降级；`floor` 和 `none` 可明确选择。
- `_ingest_deskewed_cloud` 逐帧实施同一策略，即使调用者传入默认 `both` accumulator 也不能绕过。frame receipt、最新 map manifest、grid source/semantics 都记录 `requested`、`effective` 和 `ray_carving_disabled_reason`。禁用不可靠 rays 后，仅实测 floor endpoints 标 free、障碍 endpoints 标 occupied，未观测空间保持 unknown。floor-only **仍不是** traversability 证书。
- 不修改 accumulator 的通用 ray 实现、IMU 外参、门限、goto 或底层执行门。历史 run4 文件没有重写，原始误标与旧 ray 格子仍是可审计历史事实；新运行才应用上述保守策略。

## run4 的实际证据

数据根目录：`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_live_dsh4_20261110T202726Z`。

- `capture/operator_map/latest.json`：cloud `/dynanav/rtx_livo/cloud_registered_full`，pose `/dynanav/rtx_livo/aft_mapped_to_init`，`lio_estimate`，stage `LIO`，exact integer pair，无插值，cloud 以 `frame=map` 入图、不二次变换。ROS map frame 为 `dynanav_lio_map`。
- 最后 pair：cloud / pose / reference stamp 都是 `109900000000 ns`。904 接受、3 拒绝。最后快照因 wall age 过期不可行动，不能拿结束后的快照 freshness 替代运行时判断。
- 保存的 `operator_grid/level_+0.01_operator_map_partial.npz`：75,287 FREE，125 OCCUPIED，resolution 0.05 m。目标 `(0.338, -0.02829)` 属于 `(6,-1)`，**UNKNOWN**。`(0,0)` 是 FREE，`(1,0)` 是 UNKNOWN。
- 75,412 个已标记格子的面积包含 floor 与 ray-derived FREE，不是已验证可行走面积，也不是完整办公室地图。

## 8 帧离线回放

回放实现及 JSON 结果保留在仓库本地 `.dsh-subagents/operator-map-evidence/replay_run4.py` / `replay_run4.json`。

复跑命令（仓库根目录）：

```sh
PYTHONPATH=src:scripts python3 -B .dsh-subagents/operator-map-evidence/replay_run4.py
```

输入是 collector 保存的 `cloud_full_001.npz` 至 `008.npz`，读取 `received.jsonl`
中相同整数 ns 的 LIO pose，验证 cloud/pose 的 `dynanav_lio_map` frame；用 harness 实际
IMU mount `[0.020153064280748367, 0, -0.005108955316245556]` 恰一次转换到 base。
外参 hash 与 live manifest 的 `ccc73e6de6b5bde6ccc383babfa4c9a44e72f7226dd611f47d2877b522037157` 一致。

源时刻分别为 4.4、4.6、4.7、4.8、4.9、5.2、5.3、5.4 秒。全部 8 帧在两种模式下接受。
保留原始到达时间，回放时以样本源时刻评估 age=0；这是几何对比，不是复刻 live scheduler、当时的到达门或 904 帧完整地图。

| 同一输入，仅切换 free_mode | occupied | free | 原点半径 <1.675m 的 free | 原点半径 <0.5m 的 free |
|---|---:|---:|---:|---:|
| floor | 74 | 8,534 | 374 | 0 |
| both | 74 | 8,904 | 463 | 19 |

- **370 个 FREE 仅由 rays 产生**；其中 89 格在半径 1.675m 内，19 格在半径 0.5m 内。半径统计按 map 原点和格子中心计算，不是每一帧移动的 body/front 扇形。
- 目标 cell 在两种回放中均 UNKNOWN，保存的最终 grid 也是 UNKNOWN。不能以附近 FREE 推断目标已观测。
- 原点在这 8 帧两种模式中均 UNKNOWN，而完整 live 最终是 FREE。仅有这 8 帧，**无法归因最终原点 FREE 究竟来自后续 floor 命中还是 ray**。
- 新增合成测试也复现：远处有合法地面 patch、一个高处 obstacle hit，近场查询 cell 没有任何地面点；floor 模式保持 UNKNOWN，both 模式被投影 ray 标为 FREE。该行为是 occupancy 的定义，不能当成地面支持证据。旁边没有 ray 的 unknown 仍是 unknown。

## 目标准入与剩余限制

`build_request/submit` 负责协议、有限坐标和 TTL 等校验，不读取 operator grid；执行侧独立 perception 门负责运动安全。直接把 `FREE` 接成 goal/pass 条件无法修复支撑缺失，反而会把上述 19 个无 floor 的近场格错误当成安全落脚面。新增正确的 operator 目标门至少需要同一时空参考下的独立 floor-support 图层、footprint/路径连续支持，以及 stale/unknown 拒绝；本次不扩大到该设计，也不填 unknown 或降低现有门。

run4 最后 perception.front 同时记录 `floor_observation_state=supported`、radial floor counts `[0,0,458,541]`、`near_field_floor_unobserved_m=[0,1.675]`、第一可见地面距离 1.8372m。这暴露了另一个 production 近场支撑门问题，已交给并行 perception/control 修复，不能用本报告的 metadata 修复宣称它已经解决。

另一个明确的几何约束：harness 传给 operator accumulator 的是 IMU mount；`capture/lidar.py`
的物理 `LIDAR_MOUNT=[0,0,0.25]` 不同。将点坐标静态转换到 IMU frame 并不会把真实射线起点搬到 IMU；当前投影 origin 是 `T_map_base @ T_base_imu`。这会有位置/倾斜相关偏差，旋转扫描还需要考虑 deskew 的参考时刻。可靠修改应显式定义物理 LiDAR 光心及扫描参考契约，而不能偷偷改现有外参造成坐标混用。本次对历史数据保持原样对照回放；生产 LIO 新入口保守禁用这类 rays，不猜光心、不改 IMU 外参。将来若要恢复 rays，需要上述新契约与验证，不声称旧 ray 自由空间是精确体积可达性证明。

## 验证

全部为 CPU、无 ROS graph/仿真：

- `PYTHONPATH=src:scripts python3 -B -m unittest discover -s tests/unit -p 'test_operator_grid_export.py' -q`：11/11。
- 同命令 `test_operator_map.py`：105/105。
- 同命令 `test_explore_waypoint_client.py`：97/97。
- 同命令 `test_dynanav_rtx_production_bridge.py`：14/14。
- 合计 227 项通过，新增 8 项：真实 runtime 来源接线、sensor-frame 不推断模态、缺证据不编来源、ray FREE 不等于 floor 支撑，以及生产 LIO 禁ray/none不填free/显式rays或both拒绝/旧模式与新策略兼容。
- 未跑全仓测试、未运行新的 live simulation，尚无修复后真实场景 operator map 验收。导出仍逐格复制真实 accumulator，已有 cell-copy/hash/unknown-render 测试通过；新 LIO 的 floor-only 策略会保守减少 ray-only FREE，不会增加观测或修复场景缺几何。
