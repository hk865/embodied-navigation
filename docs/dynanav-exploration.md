# DynaNav 当前探索：按功能阅读架构

后续更新：默认 launcher 已切到 [实测 PCT + 原生 SCAN](dynanav-pct-scan.md)，见 [4 米实时核验](experiments/dynanav-pct-scan-live-20261111.md)。下文保留此前 `native_route` 直线跟随基线，不能当作当前默认规划链。

更新：2026-11-11。本页说明本机工作树的 **static native 探索**，不是生产导航整栈验收。入口代码、配置与可复用 SOP 在项目内；原始传感数据、地图、视频、运行日志和构建产物在 `PROJECT_DATA_ROOT` 的新卷上。历史报告按各自 run 解读，不能把早期 InteractiveWaypointTracker 的限制套在当前 NativeWaypointTracker 上。

## 1. 先看完整工作流程

```text
Isaac 场景/物理 → front RGB、物理 IMU、RTX 完整扫 → ROS 2 / Fast DDS
                                                       ↓
                                                   FAST-LIVO2
                            ┌──────────────────────────┴─────────────────────┐
                     同stamp LIO cloud/pose                          post-VIO pose
                            ↓                                               │
                  operator map / frontier                                  │
                            ↓                                               │
             模型/操作者查看本代RGB、实测点云和地图 ←───────────────────────────┤
                            ↓ official goto mailbox                         │
                     NativeWaypointTracker ←────────────────────────────────┘
                            ↓ 原 RouteTracker → 速度命令
                                                     ↓
                     原策略本体观测 ─────────────→ RoboGauge → 关节/物理
```

这里有两个闭环：FAST-LIVO2 与执行器在各自采样周期工作；模型/操作者较慢地查看 RGB、实测点云和地图，决定有意义的下一目标。模型不直接输出速度，不替换原跟踪律，也没有常驻“下一目标自动规划器”。

**当前状态**：最新保留 run 有 32 次 accepted/32 次 reached、stop seq33、正常停车与 capture exit0；同时记录 26 个 body-contact 物理样本。详见 [32 航点记录](experiments/dynanav-native-exploration-32-waypoints-20261111.md)。回执成功、物理接触、地图质量和探索覆盖是四件不同的事。

## 2. 功能模块、位置和选型

| 功能 | 当前实现/入口 | 选择及边界 |
| --- | --- | --- |
| 启动与资源归属 | `scripts/run_dynanav_native_exploration.sh` | 统一拥有 capture、FAST-LIVO、operator map 的进程；独占锁、新 run、有限预算、正常清理。启动不自带目标。 |
| 仿真与真实传感渲染 | `simulation/dynanav/capture_go2_rgbd.py`、`capture/{scene,physics,sensors,frames,lidar,ros_interface}.py` | Isaac 生成物理 IMU、front RGB 和官方 RTX 返回。当前 RTX 路径禁用 depth 反投影点云；没有将场景几何交给选点器。 |
| SLAM | `third_party/fast_livo2/fast_livo2_core`、`scripts/run_dynanav_lio.sh` | 保留已有 FAST-LIVO2 的 LIO/VIO 与建图，接入 timed Ouster/完整扫及 same-pair handoff 补丁。不是重新实现 SLAM，也未证明是所有候选算法中的最优选择。 |
| 实测地图 | `simulation/dynanav/operator_map.py`、client 的 `map` 子命令 | 同stage、exact source stamp 的 LIO map cloud/pose；保留来源与拒绝原因。LIO cloud 已在 map frame，不能再乘一次 pose。 |
| 看图与决策记录 | `.agents/skills/dynanav-explore/` | 当前代 RGB、点云 local/full XY/XZ/YZ/斜视3D、按高度分组2D辅助图及 gap journal；它是 SOP 和证据工具，不是自动规划器。 |
| 目标接口 | `scripts/explore_waypoint_client.py` | 原子 request.json + 单调 seq；observe/goto/cancel/stop 及回执；exit0/accepted 不等于 reached。 |
| 单目标状态与跟踪 | `simulation/dynanav/native_waypoints.py` → `navigation/robogauge/route_tracker.py` | adapter 只把当前估计 pose 和目标接入原跟踪器；保留原转向、到达、停车和 stall 判断，不新增 map-readiness、heartbeat、TTL 或 floor-clearance 的 goto 门禁。 |
| 足式执行 | `simulation/dynanav/capture/control.py`、RoboGauge policy | 复用既有策略、本体观测和关节契约；没有训练新策略。1 m/s 是命令上限，不是持续实测速度。 |
| 评分与复核 | run 的 control/policy/truth journals、`scripts/audit_dynanav_native_exploration.py` | 真值用于原策略本体观测、物理诊断及明确隔离的离线评分，不能回流为模型选点或导航位姿。CPU检查不代替路线验收。 |

上述 `simulation/...` 等包内路径均以 `src/embodied_navigation/` 为前缀。生产主路径另有 SCAN/PCT、PID、任务/停车模块，见 [全工程架构](architecture.md)；它们不是本次 native 探索链中实际执行的规划器。在线行人/learned semantic 支路也不是本次 `semantic_provider=none` 的 static run。

## 3. 已有 frontier 到底做什么

`operator_map.LevelGrid.frontier(pose_xy)` 找 **已观测 FREE 且8邻域接触未观测格** 的格子，按到当前测量 pose 的平面距离排序，再由 snapshot/export 限制返回数量。

- 它帮助模型/操作者发现地图观测边缘、分支或需要补看的方向。
- 它不是信息增益评分：当前没有对未来视野的可见体积、熵下降、预计新观测面积或旅行代价作联合优化。
- `frontier.actionable/reasons` 反映地图配对、来源、新鲜度、分层和 floor evidence；即使不 actionable，历史格子仍可作为明确标注的地图证据。
- 它不自动选目标，不负责路线搜索/避障，不直接发送 goto。native 执行的 source/frame/freshness 检查独立存在，map.actionable 不是额外 goto 门禁。
- 当前 LIO-deskewed-map 自由空间策略按实测 floor endpoints 判 FREE，不能把默认/其他点云路径的 ray carving 当成本次地图行为。格子未导出才是该 grid 的 UNKNOWN；图中没画到的点、裁剪空白和传感器无返回没有独立的占据语义。

**尚未落地**：多 agent 的常驻分工/互斥仲裁、信息增益目标规划、自动持续补充目标队列、地形可通行性层和完整覆盖判定。本次文档归档不实现这些功能。

## 4. 节点、话题与接口

最新运行实际 ROS domain 为 **232**，不是启动脚本默认值219。进程环境、有效 launch/config 和 run receipt 才是本次实参。

| 生产/消费 | 数据与用途 |
| --- | --- |
| Isaac capture → FAST-LIVO | `/clock`、`/dynanav/imu`、`/dynanav/camera`；以 source header 和原测量时间处理，不拿回调到达时刻冒充采样时间。 |
| Isaac RTX → ROS | `/dynanav/lidar_rtx` 发布 raw PointCloud2；当前 FAST-LIVO 实际订阅 `/dynanav/lidar_rtx/complete_scan`，typed 包络携带完整扫/水位证书和原云。不能将证书误称真实全方向无遗漏或可通行证明。 |
| FAST-LIVO LIO → map | `/dynanav/rtx_livo/cloud_registered_full` + `/dynanav/rtx_livo/aft_mapped_to_init`，同stage精确整数stamp配对；点已是 LIO map frame。 |
| FAST-LIVO VIO → capture/native | `/dynanav/rtx_livo/post_vio_odometry`，使用 `dynanav_lio_map` / `go2_imu`，导航源为 `livo_post_vio`。图像收到或 topic存在本身不证明每帧执行了有效视觉更新。 |
| 模型/操作者 → native adapter | `<RUN>/capture/interactive/request.json`；`goto/cancel/stop`，没有通过 ROS 速度话题直接驱动本控制链。 |
| native adapter → 观测/反馈 | 同目录 `observation.json`、`receipts.jsonl`；另外 `interactive_waypoint_control.jsonl` 记录控制证据。 |
| 地图/图像 → 人或模型 | `capture/operator_map/latest.json`、`operator_grid/*`、`capture/interactive_frames/index.jsonl`及真实front PNG；辅助图不是物理真值地图。 |

详细 QoS、线程、缓存和本次实测频率见下一节；不能从话题名或 nominal 参数推断投递完整性。

## 5. 关键时序、线程与缓存

### 一个传感周期

1. 物理以固定5 ms推进。传感器记录原source时间、RTX逐点相对时间及完整扫区间。
2. RTX writer 回调复制并排队原始扫描，主循环取出、发布 raw cloud 与 typed complete-scan；这段 host queue residence 只是局部排队耗时。
3. FAST-LIVO 同步 IMU/云/图像，完成 LIO deskew/update，再处理绑定的 VIO pair；LIO建图输出与 post-VIO导航输出分开使用。same-pair补丁避免把已就绪VIO不必要地等到下一raw cloud，但不消除所有发布/执行延迟。
4. native 控制以50 Hz读取最近的真实post-VIO测量。新source stamp推进原到达/停车dwell；重复但新鲜的pose不会伪造新测量时间，过期source会零命令hold。
5. 选择新目标是独立、较慢的决策动作；snapshot包括真实front RGB及3D/地图证据，不用第三人称overview代替front输入。

### 一个目标与停止周期

- `goto`写入请求 → `waypoint_accepted` → 等实际新鲜测量激活 → 原RouteTracker输出 → `waypoint_reached` → idle。adapter可以接收队列，但每段保留原到达/停车行为，不是无停顿轨迹拼接器。
- 单目标失败 → `waypoint_failed`、零命令、清除active和旧queued目标（各有dropped反馈）→ 同一仿真/SLAM继续 → 刷新观测后明确选择新目标。
- cancel清空active/queued并idle；stop ack进入终态停车。原停车要求终态至少5秒、线速度<0.02 m/s和角速度<0.05 rad/s持续至少1秒；随后核对capture退出和owned进程清理。
- CLI timeout只说明等待超时，不会自动cancel；先检查同seq回执，不能盲重发。单航点失败不等于capture退出，整个run有限预算和显式stop仍然终止。

### 时钟与速率的阅读规则

- source/simulation秒用于测量同步、source age、控制dwell和有限仿真预算。
- `time.monotonic()`用于原回调arrival、wall age及host队列耗时；不能同source时间相减。UTC用于目录名/生命周期记录。
- 保存front PNG给模型的间隔是2模拟秒，不能说相机只有0.5 Hz；相机原流另有10 Hz记录。
- `ros2 topic hz`测到的是当前墙钟接收频率，会受实时因子和丢包影响；不能据此直接替换源时基频率。
- 下列测量见32航点报告及run原始日志，不等于端到端处理延迟或每帧视觉更新率。

### 本次节点与 QoS（配置事实）

- capture节点 `actual_go2_rtx_source`：IMU、RGB、raw cloud和typed complete-scan发布使用sensor QoS，即BEST_EFFORT / VOLATILE / KEEP_LAST depth5；`/clock`用RELIABLE depth10。导航post-VIO订阅用sensor QoS，最新pose保存在单槽缓存，同时保存原header和独立arrival。
- FAST-LIVO节点 `/dynanav/rtx_livo/dynanav_livo_observer`：输入complete-scan depth4、RGB depth4、IMU depth512，均SensorDataQoS；`cloud_registered_full`、`aft_mapped_to_init`、`post_vio_odometry`与其status输出用默认RELIABLE depth10。可靠发布端兼容best-effort订阅端不等于端到端绝不丢包。
- operator map节点 `dsh_explore_map_accumulator`：cloud/odometry/clock订阅为sensor QoS。snapshot参数为每1墙钟秒检查一次，实际周期受处理工作影响，不是固定1 Hz地图更新保证。
- 实现依据：`capture/ros_interface.py:create_ros_interface`、`LIVMapper.cpp`发布/订阅构造、client的`_cmd_map`；有效输入和domain另见run的`observer/launch.json`。UDP接收缓冲选择与fallback见 [通信说明](dynanav-observer-udp.md)。配置不是DDS投递完整性证明。

### 当前线程与三类不同缓存

| 位置 | 当前实现/容量 | 不能混为一谈的内容 |
| --- | --- | --- |
| capture/RTX host | RGB pending cap4；RTX pending cap4（名义0.4模拟秒扫描），溢出raise | 这是Python/host应用队列，不是DDS history，也不是GPU渲染时延。 |
| FAST-LIVO执行 | `FASTLIVO_PIPELINE_MODE=serial`、`FASTLIVO_SERIAL_IMU_PRIORITY=0`；SingleThreadedExecutor `spin_all`协作预算1 ms，未ready同步sleep200 µs | 源码存在staged decode/receiver线程，但本run未启用，不能描述成当前并行流水线。4 ms wall timer只是名义回调调度，不是250 Hz输出。 |
| native内部buffer | image cap32；IMU与propagation IMU各8192；pipeline cap512包/64 MiB | 与DDS depth4/512是不同层。pipeline summary测得peak_pending1，899415 accepted/delivered/committed，无capacity rejects/cancel/uncommitted，normal_completed=true；不证明所有传感信息都被视觉EKF保留。 |
| operator map配对 | pose cache超过512裁至最新256；pending cloud cap64；wall window0.5 s、sim freshness0.25 s、pose-pair tolerance0.10 s | 等不到/不可信的观测有拒绝原因，不改旧stamp、不外推pose；same-stage LIO-deskewed输入另要求exact source stamp配对。 |
| 可选skill点云collector | DDS BEST_EFFORT depth100；各stamp字典超过300裁至200；0.05 m voxel保留 | 仅显式启动时存在，不属于native launcher必启动进程。不能把collector晚开始的局部区间说成全程SLAM地图。 |

实现入口：`capture/{sensors,lidar}.py`、`scripts/run_dynanav_lio.sh`、`LIVMapper.cpp`、`configs/simulation/dynanav/fastlivo_rtx_os1_32.yaml`、client的`_retry_pending_clouds/_cmd_map`。使用源码与本run receipt区分“配置存在”和“本run测得”。

### 最新32目标run测得的速率与延迟范围

| 流 | 记录数 | 原source/测量stamp区间（模拟秒） | `(N−1)/(末stamp−首stamp)` |
| --- | ---: | --- | ---: |
| IMU | 355540 | 0.01999999955–1777.71496582031 | 约200 Hz |
| RTX扫描 | 17776 | 0.1–1777.6 | 10 Hz |
| RGB原流 | 17777 | published_time 0.1–1777.7 | 10 Hz |
| policy/control | 各88885 | 0.015–1777.695 | 50 Hz |

355540个5 ms物理步对应1777.7模拟秒，loop wall2231.3484秒，汇总实时因子0.796693。control跨2231.112856 monotonic墙钟秒的平均频率39.8384 Hz，墙钟tick间隔中位17.5609 ms、最大5.4473 s，不能承诺墙钟50 Hz。

RTX producer host queue residence中位13.1699 ms、p95 44.7596 ms、最大263.9868 ms，最大记录depth2；raw-copy中位0.2512 ms、p95 0.2837 ms、最大2.2507 ms。这些只是明确测量点之间的host指标，**不是GPU渲染或LiDAR→VIO→执行端到端时延**。本轮没有重新解码native诊断records.bin量化视觉校正保留率或估计器CPU时延。

采样细节：控制每4个物理步，IMU每步，RTX polling每10步（20 Hz轮询，不是20 Hz扫频），RGB处理每20步。50 Hz控制通常复用10 Hz最近测量；复用不等于插入伪造的新source。

## 6. 地图倾斜、分层与物理边界

地图显示中的倾斜/不同位置高度变化仍须单独核因。本轮整理没有做新平面拟合、外参重新标定或重力对齐，也没有把地图“拍平”。不能仅凭截图或z-band把原因归给地形、SLAM漂移、传感外参或投影；这些需要同代测量云、同stage pose、轨迹和RGB联合复核。

`operator_map` 的高度组是估计pose高度分组，**不是物理楼层计数**。固定绝对map-z切片不能可靠区分物理楼层；楼层/楼梯辅助显示须结合本代source-time窗口、实测轨迹和实际RGB，保留完整输入与裁剪/采样来源。地图倾斜和点云裁剪空白不应变成自动goto门禁，也不应被隐式修正为FREE。

当前2D占据层会将floor上方0.15–2.2 m的返回投为OCCUPIED，阶梯也可能占据；这不等于楼梯不可走，也不是已经有2.5D地形规划器。详见 [历史楼梯投影审计](experiments/dynanav-stair-map-projection-audit-20261111.md)，该报告针对旧domain207/旧控制器，不是当前32目标run的完整地形验收。

当前 static-native 模式的body speed/姿态/位置包络有diagnostic-only语义，body contact report-only。必须报告实际接触、失去支撑、摔倒或stall，不能以程序未报failure代替物理成功；反过来，26个contact样本本身也不证明摔倒。电机、时钟、有限状态、原RouteTracker测量与停滞检查仍独立存在。没有新增或放宽任何控制代码。

## 7. 可复用操作与数据归属

项目内快照：[`../.agents/skills/dynanav-explore/SKILL.md`](../.agents/skills/dynanav-explore/SKILL.md)，完整步骤见 [native-interface.md](../.agents/skills/dynanav-explore/references/native-interface.md)。六个入口/参考/脚本从本机已用skill归档，`archive-manifest.json`保留来源与SHA；个人已安装skill未改。项目副本与个人副本此后不会自动同步。

```sh
REPO=/path/to/embodied-navigation
SKILL="$REPO/.agents/skills/dynanav-explore"
python3 -B "$SKILL/scripts/dynanav_sop.py" --help
# RUN / EVIDENCE / CLOUD必须指向本代外部数据目录；不要用仓库保存大数据。
```

`--help`不会发目标或启动仿真。SOP的`status/snapshot`会在指定EVIDENCE写证据；`launch/goto/cancel/stop`有实际操作效果。启动新仿真与选择目标必须属于当前明确任务，不能把阅读命令当作启动授权。

新运行需新目录、空闲明确domain和前台持久会话；启动器接受8..1800模拟秒，默认300，wall上限为max(900, ceil(3×duration))秒。保留原场景/标定/BUILD依赖并核对有效值；默认路径是本机数据产物而非公开下载，单克隆仓库不等于获得Isaac、资产和构建二进制。没有checkpoint/resume，不能把新spawn说成接着旧代继续。

## 8. 当前证据与后续工作

- [32航点回执、接触、来源和视频](experiments/dynanav-native-exploration-32-waypoints-20261111.md)。
- [五航点早期原型](experiments/dynanav-native-autonomous-exploration-20261111.md)：含当时速度评分，不能移植为32航点run速度成绩。
- [单目标失败恢复](experiments/dynanav-native-waypoint-failure-recovery-20261111.md)：同场恢复回归和1053/108测试日志。
- [完整扫运行契约](experiments/dynanav-complete-scan-runtime-contract-20261111.md)、[same-pair时序](experiments/dynanav-same-pair-handoff-20261110.md)：历史测量按对应run阅读。

下一步功能仍需单独设计/验证：解释和量化地图倾斜、测量地形层、信息增益与旅行代价、连续目标调度、多agent读写责任/仲裁、覆盖gap关闭标准。本页只记录现有能力与边界，不以写出方案代替实现。

补充：[今日归档中的实际探索工具与调用证据](experiments/dynanav-exploration-tools-20261111.md)，包括历史观察入口、覆盖/ROI/分层渲染原件及SHA索引；当前skill快照保持不变。

项目内可执行统一入口：`python3 -B scripts/dynanav_exploration.py --help`。`sop/collect/view`连接当前项目skill，`review/roi`是从原工具提取并适配当前SOP快照格式的可复用版本；无须运行旧task目录下的Python文件。完整命令、依赖、输入绑定和历史专用脚本边界见[探索工具入口](../scripts/dynanav_exploration/README.md)。原件归档和个人skill保持不变。
