# RTX/Fabric 办公室几何恢复：预挂载会话层展开实例

实验 UTC 时间：2026-11-10 20:41–21:03。结论：**已恢复当前办公室静态场景的真实 RGB 与 RTX LiDAR 楼梯/墙体几何，并完成最终源码短站立全链验证。没有执行导航动作，不是探索、运动精度或通行验收。**

旧 [production map/control 报告](dynanav-rtx-production-map-control-20261111.md) 顶部已加勘误：“世界固定无回波证明合法外侧开口”的解释撤回。原始数据不删除、不重写。原 USD 图像有办公室墙/楼梯，而默认 RTX/Fabric 缺失，故此前稀疏图不能作为正确办公室图。

## 环境、输入与不变项

- 主机 `hyh001-System-Product-Name`，Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`，RTX 5070 Ti，ROS Jazzy。
- 源场景：`$PROJECT_DATA_ROOT/embodied-navigation/dynanav/documents_codex_20261108/codex/2026-11-05/task-2/full_asset_render/bundle/office.usd`。
- 源 SHA256 始终为 `c4a1458847c51c3b3afd65dc6e6d8a50929ad350a135601a67100e35e1f59950`。没有 Save/Export 原资产，没有改原 mesh、法线、材质、碰撞属性。
- 同出生 `[-14.53,-7.5,-2.85]`，yaw=π，OS1 32线10Hz。所有物理 probe 为 interactive 无 waypoint；所有实际 policy 命令为 `[0,0,0]`。图中 `target0.10m/s` 是配置上限，不是实际发出的运动命令。
- 保留 FSD、PhysX Fabric、multi-tick、1/200 history、原生扫描/图像时间、pose/clock/扫描有效性、碰撞、freshness、TTL 和 unknown 门。未用深度云替代 RTX、未合成补图。
- 最初几个隔离 probe 显式带 pose-loss timeout=30s，仅为无动作采样；`usd_cpu` 起以及全部生产全链恢复默认门。没有把早期 probe 当作生产导航验收。

## 根因范围与修复

实测支持的结论是 **当前 RC7 FSD 在初始 stage population 中丢失原场景实例几何的提交**；不是磁盘资产损坏，也不能简单归结为可见性、矩阵或楼梯背面。

默认路径里楼梯的 USD/Fabric `points`、三角索引、worldMatrix 都存在且对应，`omni:rtx:skip=false`、`_worldVisibility=true`。但渲染输出缺失。stage 已挂给 renderer 后再解除 3413 个实例，无效；在挂载前完成同样会话层展开，RGB 与 RTX 均恢复。这个对照定位到 population 时机；没有声称找到了闭源 SDK 内部具体故障行。

`capture/scene.py` 的 RTX 路径现在：

1. 从原 root layer 和**新建独立匿名 session layer**打开 USD stage。
2. 只在 session 中对已加载、active 的原场景 instance roots 设置 `instanceable=false`；按路径重新取 prim，处理嵌套实例，并检测无进展。此时机器人/传感器尚未生成。
3. 放入本进程 `UsdUtils.StageCache`，通过 SDK `attach_stage_with_callback` 挂载，**等待 finish callback 成功后**再核 active stage ID。失败只关闭自己所属的 stage、Erase 自己的 cache entry，不清空全局 cache。
4. 再沿原路径创建 Go2、相机、RTX、物理和原支撑预检。RGB-D 分支仍用原 `open_stage`。

成功 stage 的 cache 生命周期由当前 capture/app 进程持有；所有实验结束后进程已退出。此实现覆盖挂载前已加载的静态办公室实例，不保证后来加载 payload、切换 variant 或动态新增实例自动获得同样处理。

同时更正 `perSensorTickTlas` 参数到官方 `/rtx/rendering/perSensorTickTlas`。此前误写的 `/rtx/hydra/...` 不是本次根因，因为本机 base.kit 已使真正 rendering 键默认为 true；实读两键都为 true。最终回执记录有效 multi-tick 与 per-sensor TLAS 值。

离线 `usd_invariance.json` 比较原场景展开前后 **3785 个 mesh**：points、faceVertexCounts/Indices、normals、orientation、doubleSided、visibility、collision 属性和 computed worldMatrix 全相同；无新增/丢失 mesh。只改变 session 的实例表示。

## 对照记录

所有目录位于 `$PROJECT_DATA_ROOT/embodied-navigation/experiments/`，各目录保留当时的 probe 脚本、原始日志、回执与输出。以下名称时间均为 UTC。

| Run | 唯一主要变化与结果 |
| --- | --- |
| `rtx_scene_baseline_20261110T204109Z` | 原路径，118扫；RGB缺墙/楼梯，9.5s为15002点 |
| `rtx_scene_deinstance_20261110T204148Z` | open_stage之后 session展开3413实例；RGB仍缺失，119扫，9.5s为14858点 |
| `rtx_scene_fsd_off_20261110T204336Z` | FSD=false、PhysX Fabric=true；RGB室内恢复但camera冻结，2.015s原pose门拒绝，RTX未产有效扫描 |
| `rtx_scene_direct_stage_20261110T204451Z` | 去掉启动空stage/提前set_dt；未恢复几何 |
| `rtx_scene_usd_cpu_20261110T204555Z` | FSD=false、PhysX Fabric=false、CPU写USD；118帧RGB，RTX零扫描，原扫描门拒绝，无回退 |
| `rtx_scene_renderer_instances_off_20261110T204637Z` | renderer instancing=false；后证实本来就是默认false，非有效修复 |
| `rtx_scene_inspect_population_20261110T204735Z` / `...inspect_population2_20261110T205036Z` | 只读mesh与SDK设置；证实数据/矩阵/visibility/skip存在，不等于真正提交成功 |
| `rtx_scene_renderer_instances_on_20261110T204808Z` | renderer instancing=true；原支撑preflight拒绝，未产图/扫描；未绕过。此早期run没有另存异常携带的完整ray report，不补写未知miss细节 |
| `rtx_scene_ujitso_20261110T205114Z` | UJITSO.geometry=true；几何未恢复，采集后SDK关闭阶段SIGSEGV，不采纳 |
| `rtx_scene_proxy_matrices_20261110T205214Z` | useMatrixForInstanceProxyTransforms=true；未恢复 |
| `rtx_scene_preattach_deinstance_20261110T205253Z` | **挂载前**session展开3413实例；完整RGB恢复，118扫RTX；原pose/clock/support门通过 |
| `rtx_scene_full_20261110T205538Z` | 第一版全链，asset_loading阶段 materialdb native SIGSEGV，失败；不得当作有效全链 |
| `rtx_scene_full_20261110T205822Z` | 加独立session/等待attach完成后的全链：151扫/151RGB/3060IMU，15.3s原perception失效门停车；不是18s完成 |
| `rtx_scene_final_20261110T210040Z` | 最终scene.py的12s零动作全链，2400步/118扫/RGB，budget结束；当时source manifest尚未覆盖map消费者三文件 |
| `rtx_scene_verified_20261110T210212Z` | **最终验收run**：补齐消费者source manifest后的12s零动作全链；46个源码/配置条目与最终工作树及snapshot逐hash完全相同 |

20:41:01的第一次baseline尝试因interactive不接受speed=0在启动Isaac前拒绝；改为合法速度上限而不发动作。一次inspection进程关闭阶段与下一次启动约1秒交叠，未并行执行物理采样；其后启动脚本增加进程拒绝重叠检查，所有有效A/B物理采样与最终全链串行进行。

## 几何证据：不只看点数

同9.5s，使用原 USD 楼梯2732个三角面，只计算**正面交点**（排除背面），原生scan端点pose插值只供离线评分，绝不输入导航/SLAM。

- baseline保留的969条前向射线中，539条应在1.215–3.155m命中楼梯；**539条全部返回在预期面后超过20cm，0条在5cm内**。这不是合法开口，也不是只看点数的推断。
- 最终verified run有8804条前向保留射线，其中5079条与正面楼梯相交，**4847条返回在原面5cm内，仅7条落面后超过20cm**。未声称每条射线都匹配；遮挡、有限模型范围及原生运动/回波差异仍保留。
- 最终9.5s扫描31250点；其中15067个独立点距离本地34个原始楼梯/墙mesh≤5cm。`SM_StairsMain_2/Section0`有5484点、`SM_StairsCarpet2`有4419点，`SM_WallStairs4`有3467点，其中2818点高于z=-2.8m，不是仅底部floor交线。
- 最终各扫点数约3.1万，仍保留真实32个ring与原生逐点时序，不做预算抽样。

证据：baseline与verified目录的 `ray_mesh_audit.json`、`geometry_audit.json`、`raw_scan_9500000000.npz`；原mesh导出与不变性报告在 `rtx_scene_geometry_20261110T2042/`。前后同11.10s RGB为各run的 `capture/latest_overview.jpg`。中心切片可视化为 `rtx_scene_final_20261110T210040Z/stair_mesh_before_after.png`（图中after明确对应final_210040，非verified_210212）。

## 最终生产链与姿态/时钟

最终run `rtx_scene_verified_20261110T210212Z`：domain173，12s，默认 `--rtf 1.0`，原same-pair build `dynanav_same_pair_20261110T171010Z`。脚本 `run_full.sh` 和只读诊断hook `production_evidence.py` 保存在run目录。hook只归档原生packet及读Fabric时钟，不改变渲染、物理、导航或时间戳。

- 2400个0.005s物理步，600个policy frame全零命令；终态 `interactive_motion_budget`，不是到达目标。
- producer与native diagnostics：IMU 2400、RTX 118、RGB 118；native LiDAR/RGB的每个stamp与producer逐条完全相同。diagnostic attempted=written=4845、dropped=0、writer_io_failed=false。
- 76个LIO与76个post-VIO输出。operator map最后配对cloud/pose/ref为11.8s，stage=LIO；source为真实 `cloud_registered_full` + 同阶段 `aft_mapped_to_init`，不是Laser_map/global map。
- operator map有效free_mode=`floor`，ray carving禁用；近场无地面观测继续unknown/translation failclosed。零动作站立不要求能走，也不授予地图FREE连续支撑含义。
- 旁路best-effort collector有丢包（raw RTX92/118、cloud_full65/76），但native ingress全量接收已由独立持久化diagnostics验证；不把旁路计数包装成全帧无损。
- `/ExternalSimulationTime.omni:time` 2400次读值与SimulationManager同一步时间误差为0；间隔为0.005s（浮点误差约1e-15）。不改stamp、未用当前body时间替代图像/scan时间。
- 按原生传感器时刻做离线pose评分，5s后RGB最大位置差0.955mm、前向差0.296°；LiDAR最大位置差0.418mm、角差0.0883°。含初始化全段RGB最大9.249cm/1.981°，LiDAR7.717mm/0.150°，如实保留。
- 站立期间body自然落地/摇摆位移范围约2.82cm、2.31cm、15.49cm，最大相对初始转角12.58°；传感器跟随而非冻结。这不替代命令运动/去畸变精度验收。

有效运行参数为 FSD=true、physicsFabric=true、updateToUsd=true、suppressReadback=false、supportMultiTickRate=true、rendering/perSensorTickTlas=true。`final_audit.json`、`pose_audit.json`、`fabric_time.jsonl`、native `diagnostics/`、`capture/source_manifest.json`、`owned_pids.txt` 与退出日志保存证据。所有owned实验进程结束，无后台导航命令。

## 测试与剩余边界

本scope最终 focused：`test_dynanav_rtx_scene_preparation`、capture orchestration、RTX source/lidar/writer timing，共56 passed。`py_compile` 和 `git diff --check`通过。父任务另跑全量unit/regression，聚合结果以父任务最终报告为准；本scope不冒称native CTest已重跑（未改native代码）。

没有commit/push，没有改原资产或全局系统配置。局部RTX/RGB几何恢复与静态全链已验证；全办公室覆盖、移动探索、真实动态障碍语义、楼梯可通行性、运动精度、第三方best-effort collector无损接收均未因此验收。
