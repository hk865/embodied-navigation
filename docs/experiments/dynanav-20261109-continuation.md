# DynaNav 后续：完整视觉对照、闭环实跑与 learned 感知

本页继续记录本地提交 `bffa2b6` 之后的工作。此前结果和失败保持在[历史实验报告](dynanav-20261109.md)，本页不把新诊断倒填成旧运行已知事实。各次实际 `source_manifest` / `source_snapshot`、二进制哈希及收据决定运行身份，后续提交不追溯替代它们。

本轮已实测完成以live最终post-VIO为唯一高层路线位姿的双楼梯、三阶段/两次停车转向并到达停车，13项独立检查通过。首轮stall记录保留；该成功限于同spawn实验、无行人路线，在线static cloud仍有simulator oracle来源。

| 项目 | 本轮已测结果 | 当前边界 |
| --- | --- | --- |
| SDK 故障定位 | GDB 压力例在 libpython traceback watchdog 线程真实 SIGSEGV | 压力周期 0.25 s；旧例为 25 s，没有旧例原生栈，只有相关证据 |
| 去 periodic dump 的采集 | 80 s / 800 RGB 稳定；随后双楼梯 GT 路线 235.415 s 完整同 capture 对照结束，监督进程全退出 0 | 证明这些运行完成，不证明所有 SDK 崩溃原因都相同 |
| 完整旁路定位对照 | 201 共同 exact KF：ONLY_LIO RMSE 1.919313 m；post-VIO 0.083742 m | GT 驱动机器人，两个估计器旁路；不是视觉导航闭环 |
| live post-VIO 接口 | 2,196 条真实 pose + 2,196 status 与 diagnostics 逐条核对 | pose-only；twist/covariance unavailable，零占位不能作为停车速度 |
| 三目标 post-VIO 闭环 | 首轮stall保留；新运行完成三阶段/两次转向及5.1s零命令停车，13项独立检查通过 | repeat-spawn、无行人、oracle static-cloud输入仍保留 |
| 地图 / RViz | 新闭环地图92,846点、207exact KF，已实际在RViz展示并QA | SLAM pose+oracle mask；局部路线图，轴只是展示坐标 |
| Mask R-CNN / learned场景 | 74帧RGB-only推理、缓存接入分层对象历史、两场景score审查 | 离线；背景UNKNOWN、STATIC为0；上游LIO仍依赖oracle static cloud |

## Traceback watchdog：直接证据与有限外推

[GDB 压力实验](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-traceback-gdb-stress-20261109T051701Z/fault-evidence.json) 和 [debugger-console.log](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-traceback-gdb-stress-20261109T051701Z/debugger-console.log) 记录 Thread 244 在 `libpython3.12` 的 `_Py_DumpTracebackThreads` / watchdog 线程 SIGSEGV。主线程当时处于 USD 容器代码，**不是实际 fault 线程**。未更改系统 ptrace 设置。

该例仅把 periodic dump 周期从 25 s 压力化为 0.25 s；旧两个中止例没有原生 fault 栈。它复现了 traceback worker 内崩溃，与旧例截断的定时输出相关，不能证明旧例也在相同原生函数崩溃。

capture 已移除无条件 `faulthandler.dump_traceback_later`，保留标量 JSON/JSONL 遥测和外部 GDB 诊断。之后的 [80 s 稳定性收据](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-stack-timer-20261109T052102Z/stability-receipt.json) 记录退出 0，800 个采集/发布/NPZ/视频帧全部一致；该例没有 native estimator，也未完成全路线，所以单独限于采集稳定性。

## 完整同 capture 的 LIO / LIVO 旁路对照

实际运行目录：[dynanav-no-timer-paired-full-20261109T052700Z](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z)。GT 地下室到地面层路线于 235.415 s 结束，[supervision.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/supervision.json) 记录全部自启进程退出 0，无此次 SDK 崩溃。原 RGB 采集/发布/归档/视频均为 2,354 帧，`capture_completed=true`。

两 observer 使用相同新 core、同一 capture 的 static cloud 和 SDK IMU，LIVO 另外接收 RGB。实际消息核对：原始归档 2,354 帧、recorder bag 的 RGB 2,176 帧、LIVO image callback 2,234 帧；不能混为完全相同的 DDS 投递覆盖。两路各接收 47,080 条 IMU、2,354 帧 cloud，原字段核对一致。

[完整评分与来源证据](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/scores-evidence.json) 取共同 exact 关键帧 **201/236**，源时间 5.215000153–235.315002441 s；不插值、重采样或整段轨迹拟合。GT 仅作评分，未进入估计器。

| 对齐协议 | 来源 | 3D RMSE / 最大 / 最后匹配误差（m） |
| --- | --- | --- |
| 每源在同一首帧各一次固定 SE(3)，主协议 | ONLY_LIO | 1.919313 / 2.565622 / 2.545833 |
| 同上 | post-VIO | 0.083742 / 0.156613 / 0.155007 |
| 共享 ONLY_LIO 首帧对齐 | post-VIO | 0.074619 / 0.147819 / 0.146496 |

这次指定场景的完整旁路对照中，视觉辅助的误差明显更小；它不是所有场景、参数或轨迹的普遍结论。机器人控制仍为 GT，不能据此称视觉闭环完成或全办公室覆盖。

可查看[主协议轨迹图](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/trajectory-per_run_single_initial.png)、[共享初始对齐图](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/trajectory-shared_initial_from_ONLY_LIO.png) 和 [independent-checks.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/independent-checks.json)。

## 完整旁路对照的资源采样

[full-run-final-audit.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/full-run-final-audit.json) 的 `active_resource_sampling` / `resource_method` 记录同一次完整运行。按两 mapper 共同活动包络选取完整采样区间：221 段、448.525 s wall time，覆盖共同包络的 99.247%；没有对边缘 3.402 s 插值。

| mapper | CPU 时长加权均值 / p95 / max（单核 100%） | RSS 样本均值 / p95 / 活动样本 max（MiB） |
| --- | --- | --- |
| ONLY_LIO | 9.968 / 11.824 / 12.794% | 134.049 / 162.738 / 162.746 |
| LIVO | 14.800 / 17.239 / 17.758% | 508.082 / 856.301 / 897.176 |

CPU 来自相邻进程 CPU 时间除以实际 monotonic wall interval，p95 为样本 nearest rank；RSS 为这些区间末端的 resident-page 采样。它们不包含 simulator、recorder、GPU 模型或整栈，不是全办公室/其他路线的算力承诺。活动采样峰、全部进程采样峰与 GNU time 的 whole-job maxRSS 统计口径不同，不能混用。原始资源 JSONL、边界流、time 输出与脚本 SHA 均保留在审计中；独立文件为 [resource-audit.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/resource-audit.json) 与 [resource-analysis-used.py](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/comparison-full/resource-analysis-used.py)。

## 真实 post-VIO 发布接口

新 core [build-identity.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/build-identity.json) 对应 `dynanav_post_vio_20261109T052245Z`，已成为 launcher 默认。两个 native CTest **2/2 通过，0.51 s**；分别覆盖原 LIO/lifecycle fixture 和新 post-VIO 输出契约，见 [test-result.txt](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/test-result.txt)。旧构建和旧运行身份保留。

`--mode livo` 显式开启 `/post_vio_odometry` 与 `/post_vio_odometry_status`，按 observer namespace 重映射。普通 `/aft_mapped_to_init` 仍在同帧 LIO 阶段先发布。新 live 接口不同于旧报告中离线派生的 Odometry bag。

[live-post-vio-integrity.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/post-vio-live-audit/live-post-vio-integrity.json) 已逐条核对：

- 2,196 pose / 2,196 status / 2,196 post-processFrame state 的 stamp 集合相同，每个源时间只发布一次。
- 位置差 0，rotation 转换最大差约 6.72e-14；map/child frame 均正确。
- status 记录 2,195 次 `visual_update_applied=true`、1 次 false。125 个稀疏 native snapshots 的 flags 均一致（124 个 `livo`、1 个 skip）；kind12 的 processed flag 本身仍不能独立证明视觉更新。
- writer 和 bridge 的已记录 drop / I/O error 均为 0。
- twist 和 pose covariance 均不可用，1e12 对角表示 unavailable；零 twist 是占位，不是测得静止。消费者须采用 pose-window 与真实 SDK gyro，不能用占位值确认停车。

这个接口审计没有启动新的 estimator/simulator，也不等于闭环验收。

## 三目标 measured post-VIO 闭环：首轮失败与终点专用实验

新参考 [reference.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-post-vio-three-goal-reference-v1/reference.json) （SHA256 `91175b334e62bce55d178398f8a8e054870a7b78da1666200e0c289625660930`）从实际 post-VIO 记录构造三个目标，是 pose-only 参考，不使用 GT 目标位姿；repeat-spawn/no global registration 假设保留。

实际 [closed-loop result.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-closed-loop-20261109T054100Z/capture/result.json) 属于 domain110，仅消费 `/dynanav/livo_navigation/post_vio_odometry` 作为 route pose；`ground_truth_navigation=false`、`SLAM_control_enabled=true`、`navigation_command_source=measured_livo_post_vio_route`。GT 仍用于 policy 本体观测、物理 guards 和评分。此次 capture 条件为 Remaining 0.05 m / Distance 0.10 m，不套用此前较宽目标捕获条件。

首轮实测已经结束：两个转角分别在 source85.214996338 /175.414993286 s 完成，第三段于 source268.714996338 s 触发 `route_low_progress_stall`，8 s 窗口内前向进展只有0.003692836 m。最终 `arrived=false`；控制器终末命令为零，result终末policy tick为273.895 s，result的error为null也不能当到达成功。[terminal-capture-diagnosis.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-closed-loop-20261109T054100Z/terminal-capture-diagnosis.json) 进一步读取native260.715–268.715 s的1,601个样本：XY净位移0.003691722 m、平均XY速度0.003639331 m/s、终末native位置约`[-13.639011,1.504275,0.348654]` m；估计距参考0.098317 m、前向请求0.048884 m/s。机器人本身也几乎没动，不能把它描述为仅估计冻结。统一5cm终点捕获比此前完整VIO约8cm RMSE/16cm最大评分误差更紧，这支持另做捕获范围实验，不等于5cm几何定位已经可保证。

### 仅修改终点捕获范围的新连续实跑

新增 `--slam-final-capture-remaining-m` / `--slam-final-capture-distance-m` 默认 `None`，继承原统一条件，不改变原行为。新实验明确保持中间目标0.05/0.10 m，仅最终目标设0.15/0.20 m；stall、速度、dwell和z条件保留。离线把同一实测状态从旧cruise门变为新brake门，只是gate测试，不能替代闭环。

[新运行supervision.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/supervision.json) 记录domain111、success=true，livo/recorder/resource-monitor/capture四进程全退出0。[实际result.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/capture/result.json) 报告 `arrived=true`、failure=null、`end_reason=navigation_terminal_and_parked`；route到达判定source249.214996338 s，最后policy tick为254.395 s，native流末样本为254.415 s。最终目标捕获门于source248.414993286 s进入、249.214996338 s确认停止；控制器记录与独立逐流到达、停车审计均已核对，见下文。

该运行导航来源仍为 `measured_livo_post_vio_route`，GT导航为false、SLAM控制为true；GT用于policy本体、物理guards、评分和旁路诊断图。capture callback只订post-VIO Odometry，没有订status；按配置不用twist，实际门禁为pose/frame/freshness及pose-window+SDK gyro，不宣称controller把status或covariance作为门禁。bag事后status/native一致性另作审计。两次视觉闭环均无行人，定位输入仍为simulator oracle分类的static-cloud分支，learned mask未在线替换；它们不复用早前ONLY_LIO行人hold结果来证明视觉动态闭环。2,544帧RGB采集/发布/NPZ/视频一致，255KF、0空/低点数RGB-D；92,465点即时图仍为GT/oracle诊断；另已生成使用实测post-VIO位姿的92,846点图，二者不能混称。完整办公室覆盖、learned动态控制及任意spawn定位仍未验收。

新运行的 [native-post-VIO审计](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/native-post-vio/live-post-vio-integrity.json) 已通过：2,367 pose /2,367 status /2,367 post-processFrame状态按原image stamp精确配对，位置差0、rotation差约2.31e-14；status中2,366 true/1 false，135个稀疏快照为134 livo/1 skip并全部一致，writer/bridge已记录drop0。kind12 flag1只证明处理，status的视觉更新标记与稀疏快照证据仍分别保留。该topic契约审计本身不验收导航到达/停车。

[route journal](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/capture/slam_route_control.jsonl) 有580条hold：300条not_ready、280条stale_sim_odometry；后者对应约5.6 s策略时段的零命令hold，不能据result中的`freshness_hold_events=[]`称全程没有stale。重复采样hold次数、累计时长与连续停顿不是同一统计；精确区间和命令对应保留于独立gate报告。

[最终independent-review.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/independent-review.json) / [可读报告](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/independent-review.txt) 已对本次实际运行的13项检查全部通过：

- [controller-source-integrity.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/controller-source-integrity.json) 将12,500条policy位姿输入与2,366个独立live post-VIO pose逐原stamp核对，位置/quaternion差均0；没有非零命令缺失估计位姿。prior anchor与3目标原stamp/位置匹配旧估计bag，固定首测量对齐差约2.22e-16，不输入GT路线位姿。
- [route-gates.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/route-gates.json) 核对12,720条policy/control时间戳和request/control/actual command全一致；580条hold和255条terminal命令全部为零，tracking11,885条。5个stop/turn完成窗口各含6个distinct source测量、跨度0.5s，重复policy tick不会推进dwell。
- tracking的sim/wall age最大约0.240007/0.327153s，分别低于0.25/0.5s门；其间真实stale hold仍按上述记录，不称全程无stale。
- terminal零命令窗为249.315–254.415s、5.1s；native连续停车核对为5.0s，最后1s的201个200Hz样本全部满足停车条件。停车3D速度均值约0.000989、p95约0.004025m/s。

[metrics.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/metrics.json) 只用GT/native作离线物理评分：最终base-world位置`[-13.632945,1.499765,0.348012]` m，相对配置base目标`[-13.5,1.5,0.35]`的XY误差0.132945m、Z误差−0.001988m。该base/world误差没有拿估计sensor/map目标直接相减；没有向控制器回灌GT或整段拟合。

| 实际命令选择 / 覆盖时间 | native XY均值 / p95 / max（m/s） | native 3D均值 / p95 / max（m/s） |
| --- | --- | --- |
| 全部非零actual command /236.60s | 0.106295 /0.177146 /0.293267 | 0.120655 /0.215940 /0.439521 |
| `actual_command[0]>=0.1425`巡航子集 /191.98s | 0.125682 /0.183444 /0.293267 | 0.142280 /0.220529 /0.439521 |

速度按实际命令区间与native物理步相交计算，p95为时长加权nearest rank；0.15m/s是命令上界，不是实测全程均速，更不是1m/s验证。通用summarizer中的保守`mission_success_verified=false`不能替代上述13项实际独立检查。接触计数/事件截断没有覆盖所有碰撞，`contact_contract_validated=false`、`com_velocity_convention_validated=false`边界保留；本次完成不等于接触模型、所有速度参考点或泛化导航均已校准。

真实交付视频为 [multiwaypoint05_overview_and_rgb.mp4](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/capture/multiwaypoint05_overview_and_rgb.mp4)：254.4 s、2,544帧、10 fps、1024×512 H.264，58,037,148 bytes；SHA256 `5bbeff3b2271eabc8f0c3c4d2c30294a848479029374692ca2b74e846b354267`。它并排显示第三人称视图和机器人RGB，叠字速度是目标命令，不是实测速度，也不是原始RGB归档。已查看的[50 s真实视频帧](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/first-flight-video-frame.png)及[交付receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/video-delivery-receipt.json)保留在数据卷。Library附件尝试在tools/list阶段HTTP403，未创建文件或上传链接；本地视频存在，不能写成已上传成功。

从工程根目录使用仓库入口复跑该配置，不需要临时 `/tmp` supervisor。两个终端先设置ROS、PROJECT_DATA_ROOT和PYTHONPATH，并使用同一个新的`DYNANAV_RUN`路径；先启动observer，再在同domain启动capture。完整录包和监督细节保留于上述实际收据。

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=111  # 两个终端一致；改用自己空闲的隔离domain。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
DYNANAV_DATA="$PROJECT_DATA_ROOT/embodied-navigation"
DYNANAV_RUN="$DYNANAV_DATA/experiments/post-vio-final-repeat-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$DYNANAV_RUN"
cat > "$DYNANAV_RUN/snapshots.yaml" <<EOF
/**:
  ros__parameters:
    backend.snapshot_directory: "$DYNANAV_RUN/livo/native_keyframes"
    backend.max_ready_directories: 256
    backend.min_interval_s: 1.0
    backend.max_interval_s: 2.0
EOF
scripts/run_dynanav_lio.sh --mode livo \
  --binary-prefix "$DYNANAV_DATA/build_cache/dynanav_post_vio_20261109T052245Z" \
  --observer-namespace /dynanav/livo_navigation \
  --capture-result "$DYNANAV_DATA/experiments/dynanav-20261109-livo-dual-stairs/capture/result.json" \
  --parameter-override "$DYNANAV_RUN/snapshots.yaml" --output-dir "$DYNANAV_RUN/livo"
```

第二终端复制上面已选定的`DYNANAV_RUN`绝对路径，使用相同环境与domain：

```sh
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$DYNANAV_RUN/capture" --route basement-to-ground --speed 0.15 --duration 300 \
  --render-backend usd --imu-hz 200 --archive-rgb \
  --slam-reference "$DYNANAV_DATA/experiments/dynanav-20261109-post-vio-three-goal-reference-v1/reference.json" \
  --odometry-topic /dynanav/livo_navigation/post_vio_odometry \
  --slam-capture-remaining-m 0.05 --slam-capture-distance-m 0.10 \
  --slam-final-capture-remaining-m 0.15 --slam-final-capture-distance-m 0.20
```

运行前可给capture命令加`--check`作文件/工厂检查；它不启动仿真。结束后停止自己启动的observer；不要把其他domain或进程一并清理。

## RViz 实际查看与展示坐标

[review-receipt.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-rviz-review-20261109T053029Z/review-receipt.json) 记录 RViz 已在 domain 109 实际打开，[截图](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-rviz-review-20261109T053029Z/rviz-map-registered-frame.png) 已查看。它显示此前 91,449 点估计地图；`dynanav_lio_map -> dynanav_review_axes` 是 identity 展示轴，不是机器人 pose、GT/SLAM TF 或定位验收。

上述91,449点图和截图是历史单组VIO结果。本次闭环实际地图另在 [post-vio-map/receipt.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/post-vio-map/receipt.json)：从本轮live post-VIO bag按原header精确配对207/255 KF，跳过48（5初始化前、43缺exact post-pose），没有GT位姿、插值或fallback。输出[static_map.npz](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/post-vio-map/static_map.npz)为92,846点；mask仍sim_oracle、complete_full_scene=false。[独立坐标审计](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/post-vio-map/independent-audit.json)通过，坐标最大差约1.42e-14；这是重建对应/运算证据，不替代定位精度或内部动态污染验收；[overview.png](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/post-vio-map/overview.png)已实际生成并QA。

本轮[RViz截图](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/rviz-closed-loop-map.png)已真实显示该92,846点图，Global Status为Ok。[新review receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/rviz-review-receipt.json)记录map SHA、domain109和展示轴范围；只显示静态review cloud，不发控制/机器人TF，不用GT读数，不能用截图证明完整办公室覆盖。

实际查看入口为 [view_dynanav_map.py](../../scripts/view_dynanav_map.py)，说明见 [visualization README](../../configs/visualization/README.md)。viewer 只发布静态 review 点云，不发控制或机器人 TF。原图、XWD 与 RViz 日志保留数据卷。

## 原始 RGB replay 的真实 ROS 入口验证

[runtime-audit.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rgb-replay-runtime-20261109T055456Z/runtime-audit.json) 记录 domain112 的独立实际 ROS 接口检查：读取此前已结束重试的 627 个原始 RGB NPZ，时钟只来自该 finalized bag 的 `/clock`（wall playback rate4，源时间不改）。[replay_dynanav_rgb.py](../../scripts/replay_dynanav_rgb.py) 实际发布 627 帧，独立可靠 subscriber 收到 627 个唯一原 header，frame/layout/像素 SHA 均匹配，无缺帧或 audit error。

adapter、subscriber、clock player 均退出0且已停止；没有启动SDK/native estimator，未触碰其他domain或RViz。它证明这一前缀的原始RGB replay接口，不是完整路线重估计、native视觉处理覆盖或导航闭环验证。

## 真实 Mask R-CNN：离线 learned 基线

[评测收据](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/evaluation/receipt.json) 记录本机 RTX 5070 Ti / CUDA 的实际 `torchvision.maskrcnn_resnet50_fpn_v2` 推理。模型只读取 RGB，不接收 depth、pose、oracle labels 或提示文本；74 个输入是先前 landing 实验保留的关键帧，不是实时视频服务。

| 取样 | 新推理/评分范围 | Precision / Recall / IoU |
| --- | --- | --- |
| 全部关键帧 | 74 帧实际推理，known sampled cloud pixels 的 micro 统计 | 0.923885 / 0.824131 / 0.771733 |
| 2 s 子集 | 从相同预测缓存按 source_ns 选择 37 帧，无第二次推理 | 0.905887 / 0.868562 / 0.796674 |

2 s 表是调度/缓存子集，选中样本不同，不能据子集分数宣称周期法更准确或已达成 live deadline。37 个未排程帧中，32 帧有 oracle person，6,593 个对应 person samples 没有新预测；这些帧保持 UNKNOWN，不能写入 STATIC。即使在已推理帧，模型背景也不被认证为静态。低于 1 的召回率和误报继续保留，不能把 learned mask 当完美 oracle。

当前 score/mask 阈值均为 0.5，临时二维 mask-IoU 关联门限为 0.25；它们是实验值，不是历史约定，也不是通用三维 re-ID。纯 NumPy 对应/评分工具在 [person_segmentation.py](../../src/embodied_navigation/perception/person_segmentation.py)，GPU 入口是 [evaluate_dynanav_person_segmentation.py](../../scripts/evaluate_dynanav_person_segmentation.py)。已有离线 learned-dynamic LayeredMap 产物，详见下一节；它没有替换 live oracle，也没有生成 learned 静态导航图。

权重仅在数据卷：[下载与哈希 receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/models/perception/maskrcnn_resnet50_fpn_v2_coco-73cbd019.receipt.json) 记录官方 URL、185,828,065 bytes 和 SHA256 `73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e`。本轮没有图片/仓库上传，权重不进入 Git。

## Learned对象历史：真实离线接入与上游边界

[all receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scene-all/receipt.json) 与 [periodic2s receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scene-periodic2s/receipt.json) 读取同一74帧预测缓存，无重复模型推理。重建只读geometry allowlist和exact recorded LIO pose，不读归档GT变换、oracle states或instance IDs来生成新mask。

| 离线取样 | exact位姿 / 初始化前未匹配 | DYNAMIC / UNKNOWN观测点 | STATIC持久点 | 分配person IDs |
| --- | --- | --- | --- | --- |
| all74 | 69 / 5 | 12,376 / 535,760 | 0 | 2 |
| 2s子集37 | 34 / 3 | 6,981 / 263,115 | 0 | 1 |

**这里不是端到端oracle-free结果**：新mask来源是learned、直接建图位姿来自SLAM，但旧landing LIO位姿上游消费了simulator oracle过滤的static cloud。局部重建不读oracle标签，不会消除上游依赖。RGB-only模型输入和该重建步骤排除了直接oracle标签，整条定位链仍有上述上游依赖；离线评分另用oracle作对照。

为使context保留传递来源，另有 [pose-provenance addendum](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scenes-pose-provenance-addendum.json) 和新的 [all v2 context](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scene-all-provenance-v2/person_context.txt) / [2s v2 context](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scene-periodic2s-provenance-v2/person_context.txt)。它们显式记 `direct_observation_sim_oracle_used=false`、`pose_preprocessing_mask_source=sim_oracle`、传递来源 `sim_oracle_used=true`、`end_to_end_oracle_free=false`。v2只是metadata视图，原as-run产物及16文件哈希保留，未重新推理/建图，点数和ID不改；旧semantic_oracle=false只表示直接mask范围，不能解释成全链无oracle。

已有[learned_person_mapping.py](../../src/embodied_navigation/perception/learned_person_mapping.py) 与 [reconstruct_dynanav_learned_person_map.py](../../scripts/reconstruct_dynanav_learned_person_map.py) 接入LayeredMap的label+3D可见点中心关联；门限1m、TTL5s、历史32条为当前参数，不借用oracle或二维tracker ID。2/1个ID只表示分配数量，不是re-ID准确率或IDF1。最终snapshot中轨迹过TTL，fresh查询0不证明场景无人。UNKNOWN背景没有被写成STATIC，因此static_map为空是保守边界，不是完整建图成功。

产物包含scene_map、person_history、query及结构化context；源时间离线同步，不测live模型延迟或掩码新鲜度。2s子集没有插值/传播未调度帧，不是实时事件触发系统。复跑入口及参数见[perception README](../../src/embodied_navigation/perception/README.md)。

## Score阈值审查：改善与不可补救的漏检

[operating-point-review.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-threshold-review-20261109T054739Z/threshold-sweep/operating-point-review.json) 和[逐阈值receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-threshold-review-20261109T054739Z/threshold-sweep/receipt.json) 在landing74缓存及新增crossing75上审查score0.1/0.2/0.3/0.5/0.7，mask固定0.5。旧74帧没有再次推理，新75帧只推理一次；缓存保留模型返回的score≥0.05候选，所以score筛选可重算，已二值化mask不支持mask概率阈值回放。

| 场景 | score0.5实际问题 | score0.1的P / R / IoU | score0.1仍全漏的阳性帧 |
| --- | --- | --- | --- |
| landing74 | 65阳性帧中38帧全漏 | 0.922311 / 0.901903 / 0.838223 | 31/65 |
| crossing75 | 3阳性帧全部漏，recall0、precision未定义 | 0.973635 / 0.820201 / 0.802380 | 1/3 |

较低score恢复了部分真实候选，也增加误报；landing相对0.5新增1,079 TP和112 FP，crossing新增3,102 TP和84 FP。0.1只是**下一隔离实验**的operating point，默认仍0.5，原预测/地图未改，也没有据此替换live oracle。数据来自相同模拟场景/角色，两组都参与阈值审查，不是盲测泛化结论。

远而窄的小目标仍有明显漏检；landing的31个全漏帧中，26帧没有模型返回的person候选。crossing的0.864m近距离、图像边缘裁切样本也没有score≥0.05候选。可见范围/裁切是已测上下文，尚不能单独证明失败机制；单纯降低筛选阈值不能生成缺失候选。下一步还需几何障碍证据、时序/可见性处理和更多数据，未识别点继续UNKNOWN，不能认证静态或安全通行。

## 本轮代码检查

[最终代码检查收据](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/final-verification/result.json) 记录完整套件unit217项（3 skip）、regression86项（5 skip），其余通过；native2/2通过。20个本轮Python文件Ruff check和format、bash语法、git diff空白检查通过；vendor manifest304文件大小及SHA全部匹配。RGB adapter另有627帧真实ROS逐字节入口检查；这些证据分别记录，不能代替完整地图覆盖、learned动态导航或实机验收。本次repeat-spawn视觉估计闭环及物理停车已独立核查；全办公室覆盖、实时learned动静分离和视觉行人闭环仍未完成。已提交历史报告和历史原话不被本轮新结果改写。
