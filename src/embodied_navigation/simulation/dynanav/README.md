# simulation/dynanav

本目录负责办公室 DynaNav 场景接入、Go2 RGB-D/IMU 采集，以及显式选择的楼梯物理诊断和模拟器 oracle 分层地图适配。完整任务、SLAM、规划与部署职责仍在各自模块中。

## 入口与文件

- [capture_go2_rgbd.py](capture_go2_rgbd.py)：Isaac 无头采集循环，200 Hz 物理、50 Hz 策略，挂载真实 articulation 相机和 IMU，逐流写 JSONL。该文件有启动副作用，不能作为普通工具模块导入。
- [experiment.py](experiment.py)、[routes.py](routes.py)：显式 RoboGauge 楼梯 profile、首段或地下室到地面层路线、资产/模型检查与新输出目录管理。入口为 [scripts/run_dynanav_experiment.py](../../../../scripts/run_dynanav_experiment.py)。
- [slam_route.py](slam_route.py)：可选 live LIO 路线控制；参考生成入口为 [build_dynanav_slam_route.py](../../../../scripts/build_dynanav_slam_route.py)。这是 repeat-spawn 诊断，不做全局配准。
- [session_collision.py](session_collision.py)：地下室碰撞会话修复和实际 PhysX 支撑射线 preflight，不改原场景资产。
- [oracle_mapping.py](oracle_mapping.py)：同步深度点、像素实例分类与 GT 位姿接入 [semantic_mapping](../../localization_mapping/semantic_mapping/README.md)，导出局部静态图、对象历史及结构化快照；这是 oracle 诊断，不是学习模型或 SLAM 地图。
- [rgb_archive.py](rgb_archive.py)：`--archive-rgb` 逐帧保存视频叠字前的原始 RGB 和源时间元数据，ZIP_STORED NPZ；最终核对采集/发布/归档/视频计数。
- [collision_bounds.py](collision_bounds.py)：导出非机器人碰撞体世界包围盒，供几何核对。
- [pedestrian.py](pedestrian.py)、[dynamic_hold.py](dynamic_hold.py)：可选本机 skeletal human 与 oracle 点云 hold/resume 切片；已测一次单向穿越延后起步并到达，重复穿越失败仍保留。
- [replay_dynanav_imu.py](../../../../scripts/replay_dynanav_imu.py)：按原时刻/原序回放实际 SDK time/accel/gyro，只依赖收到的 `/clock`，不合成或使用 GT/SDK orientation。
- [reconstruct_dynanav_slam_map.py](../../../../scripts/reconstruct_dynanav_slam_map.py)：按实际 LIO 位姿与精确关键帧时间戳重建分层图，掩码仍为 oracle。已生成早期全路线 130,423 点地图和 landing LIO 单段 19,812 点图；全多层定位漂移未通过验收，内部动态污染需独立审计。
- [scripts/summarize_dynanav_experiment.py](../../../../scripts/summarize_dynanav_experiment.py)：只读统计实际命令时段的 native 速度、z 差、RGB-D 质量和独立到达/停车状态；不启动仿真、不改原始结果。

## 三种控制路径与来源

旧 [capture 入口](../../../../scripts/capture_dynanav.py) 保留 Student/COM 速度 PID 行为，接收 `/dynanav/nav/approved_cmd`，并检查 `/aft_mapped_to_init` 新鲜度。默认 4400 个 0.005 s 步，约 22 s；0.5 m/s 策略档位、0.65 m/s 物理速度断言和 `MAX_DISPLACEMENT=0.25` m XY 位移终止条件保留。它是采集探针，不是默认长路线导航。

显式 experiment 使用 RoboGauge 45D 本体策略和 Go2HV effort，以 native pose RouteTracker 发命令，无 COM 速度 PID、无 slew，`ground_truth_navigation=true`。目标命令上界可配置到 1 m/s，没有按楼梯位置自动降速；倾倒、时钟与电机等保护仍会终止运行。新 experiment 的关节速度读回数值容差为 0.001 rad/s，未更改引擎 30 rad/s 上限或 Go2HV torque，也未截断原始观测状态；旧 capture 默认比较保留。其 GT 控制与 oracle 地图不能标成 SLAM。实际速度须从流记录计算，不能用目标速度代替。

`--slam-reference` 另选 live LIO 控制，`ground_truth_navigation=false`，路线位姿反馈只用 `/dynanav/lio_observer/aft_mapped_to_init`；停止角速度用 SDK gyro，动态 hold 独立使用 oracle 云。它假设相同 spawn，以先前和当前测量锚点对齐，不做全局配准或任意初始定位。0.5 s pose window 的速度与 reported twist 不一致仍是限制；首次运行 `corner_brake_timeout`，后续 `slam-landing-sdk200` 已实际完成首段 LIO 导航及移动中的行人 hold/resume；完整多层精度仍未验收。该模式的即时 `mapping/` 仍是诊断 oracle 图；真正使用 LIO 位姿的地图由单独重建脚本生成。

新 experiment 默认使用 `--render-backend usd`：关闭 Fabric Scene Delegate 与 PhysX Fabric，通过 CPU 物理结果更新 USD。对照已恢复办公室墙体、扶手、楼梯，并消除该次路线的深度空帧；renderer CameraParams 与 native 相机位置一致。旧 capture 默认 Fabric 保留；每次 `result.json` 的 `runtime_render_path` 记录实际选择。

各路径中的 native 状态都用于策略本体观测和物理检查；Student 路径还用于 COM PID。LIO 控制模式将导航位姿与其他 GT 诊断用途明确分开。`truth_only_for_scoring=false` 反映这一事实。结果中的 `navigation_running=false` 不作为整栈导航已完成的声明。

## 传感、地图与对象流

新 experiment 默认 `--imu-hz 200`，实际每个物理步发布 SDK IMU；可选 100 重现旧抽样，旧 capture 默认仍是 legacy 100 Hz。新实验把 ROS IMU 的 SDK orientation 标为 unavailable（covariance[0]=-1），不输入模拟器姿态。

采集发布 `/dynanav/imu`、`/dynanav/camera`、`/dynanav/depth`、`/dynanav/camera_info`、`/dynanav/lidar_filtered`、`/dynanav/lidar_static` 与 `/clock`。点云来自 RGB-D 反投影，不是旋转 LiDAR。深度、标定和最终采样点保留像素对应关系；同一观测的分类才可应用于该批点。

- 完整导航云保留动态和 UNKNOWN 点，供障碍感知；静态云另分支提供给 opt-in LIO 观察器。
- oracle 掩码每帧分类；语义标签当前按 2 s 周期及最短 0.5 s 新实例事件间隔刷新，地图关键帧为 1 s。它们是本轮实验参数，不是过去已确定的频率。
- 缺失或不匹配掩码按 UNKNOWN 隔离，不作为静态点持久化。当前没有学习模型调用，`learned_model_calls=0`。
- 即时 `mapping/static_map.npz`、`scene_map.json` 与 `keyframes/` 使用 oracle 位姿，保留为局部诊断。离线重建另选实际 LIO 位姿，`pose_source=slam`、掩码仍 `sim_oracle`；两类图不能混称，也不能用点数证明全场景/多层覆盖。对象 ID 与查询边界见 [semantic_mapping](../../localization_mapping/semantic_mapping/README.md)；[query_scene_map.py](../../../../scripts/query_scene_map.py) 输出 JSON 或确定性结构化 context，不补造房间、楼层和关系。

过滤归档不等于 FAST-LIVO 内部图已免受动态污染。首段 live LIO 与单人 oracle-mask hold/resume 已实测；当前视觉前端未实现动态图像掩码，学习行人分割、通用动态避障和完整办公室多层地图仍待验收。

## 已测结果与启动

完整数字、产物、命令和未完成项见 [2026-11-09 实验记录](../../../../docs/experiments/dynanav-20261109.md)：

- 首段低速 GT 运行 67.815 s 到达、72.915 s 停车；非零命令时段 XY 平均 0.125494 m/s。USD 运行 729 帧/73 关键帧、0 空/低点数帧、17,812 点局部 oracle 图。旧 Fabric 3,954 点图保留作问题对照。
- 地下室→地面层 GT 路线 30 个支撑检查通过，230.335 s 到达、235.415 s 停车；2,354 帧/236 关键帧、0 空帧、91,865 GT 静态点。实际早期 LIO 重建 231 exact + 5 跳过、130,423 点，无 GT pose；100 Hz/g=false 的全路 RMSE 6.376 m、末端 10.545 m。SDK 200 Hz/g=false A/B 改善至 RMSE 1.851 m、末端 2.441 m，仍未通过完整多层精度；100 Hz/g=true 更差不采纳，g=false 保留。
- 新 `--speed 1 --duration 40` GT 首段运行 18.595 s 到达、23.615 s 停车；`actual_command[0]>=0.95` 的 6.18 s 巡航段 XY 平均 0.902891 m/s；全部非零命令 11.58 s 平均 0.629510 m/s。FR calf 最大读回 30.000137329 rad/s；新 experiment 仅给读回比较加 0.001 容差，引擎 max 30、Go2HV torque 与原始状态保留。早期 exact 30 断言过严；旧中止记录未被改写。
- 单向行人 GT 切片 69.255 s 到达、74.315 s 停车；hold 6.715、resume 8.215 s，累计 39,971 动态点观测排除、17,899 静态体素点、0 动态接触。hold 发生在默认行走窗口前，证明延后起步，不是行进中制动标定；重复 sine 穿越于 17.180 s 触发速度保护失败。
- 预修复 LIO 站立实测有 77 odom/77 registered/76 TF，4.315 s 以 304 样本初始化，随后约 0.677 m 漂移。`ONLY_LIO` 时间原点修复的数值回归从约 0.284937 m 错移降至小于 6e-16 m，默认 LIVO 保留；见 [0003 patch](../../../../third_party/fast_livo2/patches/0003-lio-instantaneous-time-origin.patch) 与 [LIO README](../../../../configs/simulation/dynanav/README_LIO.md)。数值修复不等于完整路线定位可靠。
- `slam-landing-sdk200` 已使用 live LIO 唯一 route pose 到达（source stamp 68.614997864 s）、停车（physics 73.715 s）。移动中 hold/resume 为 63.315/63.515 和 64.315/64.815 s；各 hold 前 0.5 s native XY 平均速度约 0.1283/0.107 m/s，仅作评分证据。Go2 与行人 capsule 代理已记录接触 0，累计 139,109 动态点观测排除；737 RGB-D/74KF、14,740 IMU。17,852 点即时图仍是旁路 GT/oracle 图；独立 LIO 位姿地图已生成 19,812 点（69 exact + 5 初始化前跳过），掩码仍 oracle。69帧单初始SE(3)离线评分 RMSE 0.035572 m、最大0.142490 m、末端0.107903 m；GT对齐未输入估计/route/重建。独立审计核对737 raw/static、74逐点mask、620控制odom与3685零差命令；663帧缺mask数组且无native完整点payload，不证明内部完全无污染。
- 首次 LIO 路线 `corner_brake_timeout` 和 clock-fixed 构造器 2 μs 失败均保留。构造器已改为 `future_tolerance_s < max_sim_age_s`，默认 2 μs 不变；factory 推导的 3.914697 μs 已实际传入成功。`--check` 纯实例化 live loop 共用控制器，避免只查配置文件漏掉真实构造约束。
- 完整路线全部 47,080 raw SDK 样本已原序回放、无 GT、publisher acknowledgment 完成；200 Hz 改善没有证明全路线已可可靠导航。gyro 四组合独立审查已完成：当前/右端样本逐步姿态增量 RMS 优于中值，但四组合全段误差仍约 0.01051 rad；仅改 dt 影响至多约 6.85e-5 rad。没有仅据此采纳积分 phase 修复或修改运行时。详见实验报告的审查摘要。
- 前述路线估计为 `ONLY_LIO`；新增约12 s LIVO smoke 的4个选定native快照有3个 `final_stage=livo`、1个 `lio_vio_skipped`，已证明所选帧有限视觉EKF更新实际保留。首轮双楼梯物理235.415s完成，但活SQLite查询锁冲突令bag止于约67.6s，仅675帧点云/CameraInfo和644帧RGB；旧失败保留。首个paired出现SDK SIGSEGV，最后保存的原生truth约36.11s，observer/recorder正常保存前缀但归档未完整；干净实例domain105重试也在最后原生truth约62.71s后SDK SIGSEGV（精确崩溃步未知），supervisor正常停止剩余进程，已停止新SDK重试，仅保存前缀对比，完整同输入路线未完成。post-VIO评分取processFrame后的kind12，不能用同帧先发布的aft_mapped；kind12 flag1本身也不证明视觉保留。
- 完整GT驱动双楼梯旁路live VIO的单组评分已完成：185 exact配对、RMSE 0.079707 m/max 0.160615/最后匹配0.152466，单次初始SE3仅评分；123个稀疏快照证实保留视觉更新。残bag不妨碍此单组评分，但它不是同输入A/B或视觉导航闭环。
- 已用2037个实际post-VIO记录导出明确派生的离线Odometry bag（未实发ROS，非旧残bag），生成91,449点静态图、185exact/51unmatched，无GT建图位姿、mask仍oracle；仅路线观察到的局部多层office。共同前缀27KF的LIO/VIO RMSE为.015871/.015430 m（VIO最大更高），重试49KF为.020962/.021943 m，不能宣称视觉一致更优。
- 派生导出/建图独立审计已逐条验证2037位置bit-exact、R往返约2.07e-14、185KF/91449体素bit-exact，无GT字段读取；历史reader路径漂移通过原receipt同SHA的as-run副本解析。前缀mapper资源仅有观测范围：一核CPU均值LIO/VIO约9.70/14.08%及9.56/13.72%，active RSS约123/235及126/321MiB，不能外推整栈/全路线峰值。
- 新 `--mode livo --capture-result RESULT_JSON` 启动与真实安装标定见 [LIO/LIVO README](../../../../configs/simulation/dynanav/README_LIO.md)。旧bag无原始RGB，不能从有损叠字视频伪造原始图像；两次paired已保留原始camera/camera_info/depth等前缀，`--observer-namespace` 隔离两种输出；归档标记为非完整路线，录制时不读取活DB。
- 终末native复核发现 `ONLY_LIO` 跳过视觉初始化后的未初始化指针析构bug，已有 [0004最小修复](../../../../third_party/fast_livo2/patches/0004-safe-uninitialized-vio-lifecycle.patch) 与生命周期回归；独立新 `dynanav_lio_lifetime_20261109T043801Z` 已通过1/1 native CTest（0.28s）并成为launcher默认。旧prefix和live运行身份保留，完整路线视觉精度仍待比较。


在工程根目录设置 ROS 与数据环境后启动，输出必须为新目录：

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=91
NEW_DATA="$PROJECT_DATA_ROOT/embodied-navigation/experiments/dynanav-$(date -u +%Y%m%dT%H%M%SZ)"
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$NEW_DATA/capture" --speed 0.15 --duration 90 --check
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$NEW_DATA/capture" --speed 0.15 --duration 90
```

`--check` 核对资产文件、策略哈希与参数；选择 `--slam-reference` 时还经同一工厂纯实例化控制器。它不启动 Isaac/ROS，不证明场景碰撞、传感输入或策略可运行。实际运行仍需本机 Isaac 6.0.1 RC 与有权使用的场景/机器人/策略资产。异常与 traceback 写入输出 `result.json` 并向调用方传播；新采集器也记录可用的故障执行器状态。数据、地图、NPZ 与视频只保存在外部项目数据目录。

本轮已使用真实 Isaac 环境运行上述诊断。新归档/namespace 后的完整代码报告为 unit 170 项（3 skip）、regression 83 项（5 skip），其余通过；native 1/1 PASS。36 个本轮 Python 文件 Ruff check/format --check、vendor manifest SHA、bash -n 与 diff 检查通过。CPU 检查包括深度对应、分层地图、IMU 回放、控制器工厂及汇总器；汇总器自身 8 项通过。这些测试不替代 ROS/Isaac 物理运行、SLAM 或实机验收。
