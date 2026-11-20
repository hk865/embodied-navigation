# Native 探索32航点：现有运行证据整理

核对日期：2026-11-11。只读核查已完成的运行，本次未启动仿真、未发目标、未改控制代码。架构与后续操作入口见 [DynaNav功能架构](../dynanav-exploration.md)。

## 1. 结论与不能推出的结论

run `native_exploration_20261111T111150Z` 保存 **32次waypoint_accepted、32次waypoint_reached、stop seq33**，capture正常停车退出0。它比 [早期五航点原型](dynanav-native-autonomous-exploration-20261111.md) 更新，应作为本机最新执行记录，而不是覆盖或替代旧实验。

同时，run记录 **body_contact_steps=26**，发生在sim1274.015–1274.140秒；本次是report-only/diagnostic-only配置。不能把32次reached写成全程无碰撞、完整楼宇探索或安全能力已验收；26个contact样本本身也不足以判定摔倒。地图倾斜/高度表达仍待单独核因，没有在此次整理中修复。

## 2. 运行身份与文件位置

新卷上的运行根目录：

`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T111150Z`

以下路径均相对于该根目录。大文件仍留新卷，本报告只保存说明和统计，不复制原云/地图/视频/日志进项目。

- `observer/launch.json`：有效ROS_DOMAIN_ID=232；FAST-LIVO complete-scan输入、发布namespace与启动参数。
- `capture/experiment.json`、`capture/result.json`：1800模拟秒请求、motion_end1795、1.0 m/s命令上限；最终显式stop后正常停车，未跑满1800秒。
- `capture/source_manifest.json`和`capture/source_snapshot/`：52个源码/配置快照。此次核对52/52当前文件SHA一致；**这不是Git提交或push证明，也不覆盖整个worktree**。
- `capture/interactive/{request.json,observation.json,receipts.jsonl}`：seq、状态与原始回执。
- `capture/interactive_waypoint_control.jsonl`、`capture/policy_frames.jsonl`：控制与实际策略命令；后者含native诊断状态，不应整份作为在线选点输入。
- `capture/interactive_frames/index.jsonl`及`rgb_*.png`：实际front图像及source时间，保存间隔2模拟秒；原RGB流另为10 Hz。
- `capture/operator_map/latest.json`、`operator_grid/`：实测LIO累积与高度图，最后snapshot是历史产物，不是run结束后仍有效的导航许可。
- `capture/truth_scoring_stream.jsonl`：隔离的原生物理/接触离线诊断，不得回流为模型选点证据。
- `capture_exit.txt`、`lifecycle.log`、`owned_pids.txt`：capture退出与拥有进程的收尾证据。

此前 `/tmp/embodied-navigation-native-exploration-current` 指向本run；pointer会变，只是线索，不代替run身份/所有权核验。

## 3. 回执、终态与视频

| 项目 | 核对结果 | 原始来源 |
| --- | --- | --- |
| 目标 | accepted32、reached32；未见waypoint_failed；最终completed_goals32 | `capture/interactive/receipts.jsonl`、最终observation |
| 停止 | stop_accepted seq33；最终sim1777.695，failure=null，terminal_reason=interactive_stop | 同上 |
| capture | outcome=completed；end_reason=navigation_terminal_and_parked；executed_physics_steps355540 | `capture/result.json` |
| 退出 | capture_exit0；11:49:26.441715580Z退出，11:49:27.664565696Z cleanup | `capture_exit.txt`、`lifecycle.log` |
| 视频 | H264，1024×512，10fps，17777帧，1777.7秒，encoder_exit0；ffprobe元数据通过 | `capture/multiwaypoint05_overview_and_rgb.mp4`及result.video |

本次未逐帧解码全部视频，也没有依据本run原始receipt单独重建完整模型决策审计。run根目录未找到与早期五目标同样的完整`model_decisions.jsonl`，因此**只能据已查证文件确认32个目标执行回执，不能声称本次已独立证明每个目标都完整查看了要求的全部RGB/3D证据**。模型选点来源要求仍见当前skill/SOP。

## 4. 真实接触、速度与地图限制

- result记录body_contact_steps26；离线truth stream中同样有26个接触物理样本，时间1274.015–1274.140秒。并非启动瞬间接触。物理步5 ms，这些样本的首末跨度0.125秒；不把此跨度当作经过连续碰撞接触算法认证的持续时长。
- 最后物理诊断body无接触、四足均有接触；线速度约0.000106 m/s，角速度停车阈值的终态判定来自原capture逻辑。程序终态与原始接触记录分别保留。
- `body_contact_report_only=true`；body speed、姿态、位置包络诊断并不等同于硬性准入/终止门。记录position_bound_out_of_band_steps100270，首次是平面位移越过15 m的旧诊断包络，不应据此单独判定失去支撑。
- `contact_contract_validated=false`、`com_velocity_convention_validated=false`。不能仅凭receipt或退出0认证接触/COM测量约定。
- 请求1.0 m/s只是命令上限。本次没有重做完整run的world-horizontal速度评分；早期五航点的0.854 m/s均值等成绩不能移植到本run。
- 最后operator-map统计frames14696、accepted11568、rejected3128。末端地图reasons含stale墙钟年龄及ambiguous_level，结束后stale不证明运行中所有地图无效，也不证明所有地图可行动。
- 高度图数量不是物理楼层数量。固定绝对z切片可能混入同层不同位置高度、相邻层或倾斜/漂移效应。此次未对地图做新平面拟合、外参修订或重力对齐；地图倾斜的原因和范围仍未验证。
- 轨迹、点云包络、32个目标或地图已观测面积都不能证明所有房间/分支/楼层已遍历。没有已验证的完整覆盖分母、地形可通行性层或自动信息增益规划。

## 5. 时序实测和线程模式

源时基的实际记录：IMU355540条约200 Hz、RTX17776扫10 Hz、RGB17777条10 Hz、policy/control各88885条50 Hz；公式为`(N−1)/(最后stamp−首stamp)`，不使用墙钟Hz替代源频率。

loop wall2231.3484秒，执行1777.7模拟秒，汇总实时因子0.796693。control墙钟平均39.8384 Hz，最大tick墙钟间隔5.4473秒。因此不能称墙钟固定50 Hz。

当前native pipeline是 **serial**。`observer/fastlivo_debug/pipeline_v19_summary.json`记录accepted=delivered=committed899415、peak_pending1、64 MiB/512包容量，normal_completed=true，无capacity rejects/cancel/uncommitted。源码中存在的staged decode线程没有在此run启用。

RTX producer host queue residence中位13.1699 ms、p95 44.7596 ms、最大263.9868 ms、最大depth2；它是host局部排队指标，不是端到端SLAM延迟。完整topic/QoS/线程/cache与测量区间见 [架构时序节](../dynanav-exploration.md#5-关键时序线程与缓存)。

## 6. 测试与本次整理验证的界限

既有完整恢复回归run `waypoint_failure_resume_20261111T110648Z`下的实际日志：

- `unit_final.log`：1053 tests，OK，3 skipped。
- `regression_final.log`：108 tests，OK，8 skipped。
- `sop_semantics_tests.log`：4 tests，OK。

这些是当时保存的测试结果，**不是本次重跑**。此次只改文档/skill项目归档，不重新启动Isaac，不重跑完整仿真/全量suite；只做文档相对链接、归档SHA、Python语法与无动作`--help`检查。最终整理检查记录见 [文档整理说明](../review/dynanav-documentation-20261111.md)。

当前分支`improve/module-docs-readability`，基线HEAD `15d97fb6647c3349a5a02382a9bf96a8494ce7db`。整理前已有30个tracked修改及85个untracked条目，未回滚、覆盖或代为commit。此次不commit、不push。缓存origin也指向该HEAD，但实时远端查询被网络策略阻止，不能保证远端实时状态。
