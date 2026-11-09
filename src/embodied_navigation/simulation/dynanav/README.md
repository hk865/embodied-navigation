# simulation/dynanav

本目录负责办公室 DynaNav 场景、Go2 RGB-D/IMU 采集、显式楼梯实验与 oracle 分层地图适配。定位算法、语义地图数据结构和控制器仍属于各自模块。已完成的完整 GT 路线旁路 LIO/LIVO 对照与实际视觉闭环须分别看待，最新结果见[实验续篇](../../../../docs/experiments/dynanav-20261109-continuation.md)；先前失败保留于[历史报告](../../../../docs/experiments/dynanav-20261109.md)。

## 入口与文件

- [capture_go2_rgbd.py](capture_go2_rgbd.py)：Isaac 无头采集循环，200 Hz 物理、50 Hz 策略，真实 articulation 相机/IMU，独立流 JSONL。它有启动副作用，不能作为普通工具模块导入；已移除无条件 periodic Python traceback dump，保留标量遥测。
- [experiment.py](experiment.py)、[routes.py](routes.py)：RoboGauge 楼梯 profile、首段或地下室到地面层路线、资产/模型检查与新输出管理。入口为 [run_dynanav_experiment.py](../../../../scripts/run_dynanav_experiment.py)。
- [slam_route.py](slam_route.py)：显式 live LIO 或最终 post-VIO 路线控制；[build_dynanav_slam_route.py](../../../../scripts/build_dynanav_slam_route.py) 从实际估计位姿构造参考。repeat-spawn、无全局配准假设须保留。
- [session_collision.py](session_collision.py)：地下室碰撞会话修复与实际 PhysX 支撑射线 preflight，不改原场景资产；[collision_bounds.py](collision_bounds.py) 导出非机器人碰撞体包围盒。
- [oracle_mapping.py](oracle_mapping.py)：同步深度点、像素实例分类与 GT 位姿接入 [semantic_mapping](../../localization_mapping/semantic_mapping/README.md)，输出即时局部诊断图、对象历史和结构化快照。它不是 learned 或 SLAM 图。
- [rgb_archive.py](rgb_archive.py)：`--archive-rgb` 保存叠字前原始 RGB 与源时间，ZIP_STORED NPZ；完成时核对 acquired/published/archive/video 计数。
- [pedestrian.py](pedestrian.py)、[dynamic_hold.py](dynamic_hold.py)：可选 skeletal human 和 oracle cloud hold/resume；首段 live LIO 行进中单次穿越已测，重复穿越失败保留。
- [replay_dynanav_imu.py](../../../../scripts/replay_dynanav_imu.py)、[replay_dynanav_rgb.py](../../../../scripts/replay_dynanav_rgb.py)：只按收到的 `/clock` 原序发布真实 source stamp 的 SDK IMU 或原始 RGB；不生成时间、不从视频伪造图像、不读取 GT 位姿。
- [reconstruct_dynanav_slam_map.py](../../../../scripts/reconstruct_dynanav_slam_map.py)：用实际估计位姿与 exact 关键帧时间重建图；mask 仍 oracle。历史独立 LIO 图为 19,812 点、post-VIO 派生地图为 91,449 点；本轮live post-VIO地图92,846点（207exactKF、48跳过），均不代表完整办公室覆盖。
- [view_dynanav_map.py](../../../../scripts/view_dynanav_map.py)：只发布归档静态点云供 RViz，`--check` 不启动 ROS；无机器人 TF 或控制，见 [visualization README](../../../../configs/visualization/README.md)。
- [summarize_dynanav_experiment.py](../../../../scripts/summarize_dynanav_experiment.py)：只读计算实际非零命令时段的 native 速度、z 差、RGB-D 质量、到达与停车，不把进程退出 0 当导航成功。

## 控制路径与位姿来源

旧 [capture 入口](../../../../scripts/capture_dynanav.py) 保留 Student/COM 速度 PID、`/dynanav/nav/approved_cmd` 和 `/aft_mapped_to_init` 新鲜度门。默认 4400×0.005 s（约22 s），0.5 m/s 策略档位、0.65 m/s 物理速度断言、`MAX_DISPLACEMENT=0.25` m XY 终止条件保留；它是采集探针。

新 experiment 默认 RoboGauge 45D 本体策略和 Go2HV effort，GT native pose 驱动 RouteTracker，无 COM 速度 PID/slew，`ground_truth_navigation=true`。可给目标命令上界 1 m/s，没有按楼梯位置自动限速；倾倒、电机和时钟等物理保护仍会终止。新实验仅给关节读回比较加入 0.001 rad/s 数值容差，没有更改引擎 30 rad/s 上限、torque 或原始观测。实测速度从流记录计算，不能把命令上界写成全程实际速度。

`--slam-reference` 按参考的 `pose_source` 选择 `lio_estimate` 或 `livo_post_vio`，并设置 `ground_truth_navigation=false`。LIO 默认消费 `/dynanav/lio_observer/aft_mapped_to_init`；post-VIO 必须消费 dedicated `/post_vio_odometry`，可用 `--odometry-topic` 选择独立 namespace，不能用同帧 pre-VIO `aft_mapped_to_init` 冒充最终视觉位姿。新 post-VIO 输出不提供测得 twist/covariance；控制停车用 0.5 s pose window 与实际 SDK gyro，零 twist 占位不能表示静止。当前capture只订Odometry，不订status；实际门禁为pose/frame/freshness、pose-window及SDK gyro，没有使用status/covariance门禁。LIO reported-twist 与 pose-window 速度差异仍须记录。

所有路径中的 native 状态仍用于 policy 本体观测和物理 guards；GT 还用于离线评分及即时 oracle 诊断图。动态 hold 使用 oracle cloud。因此 `truth_only_for_scoring=false`，但 live 模式的 route pose 不用 GT。参考是 repeat-spawn 的先验测量目标，不是全局定位；无任意初始姿态/地图配准保证。`navigation_running=false` 不作为整栈完成声明。

## 传感与地图

新实验默认 `--render-backend usd`，关闭 Fabric Scene Delegate/PhysX Fabric 并用 CPU 更新 USD，已恢复办公室墙、扶手和楼梯深度；CameraParams 与 native 相机一致。旧 capture 默认 Fabric 保留，实际路径记录于 result。新实验默认 SDK IMU 200 Hz（可选100复现旧抽样），ROS SDK orientation 标 unavailable；旧 capture 仍 legacy100 Hz。点云由 RGB-D 反投影，不是独立旋转 LiDAR。

发布 `/dynanav/{imu,camera,depth,camera_info,lidar_filtered,lidar_static}` 与 `/clock`。深度/标定/最终点保存像素对应，同次观测分类才能应用于该批点：

- 完整障碍云保留 DYNAMIC 和 UNKNOWN；另一个静态分支供 opt-in 估计器，缺失/不匹配 mask 按 UNKNOWN 隔离。
- live oracle mask 每帧计算；标签调度为 2 s 周期 + 最短 0.5 s 新实例冷却，地图 KF 为 1 s。这是本轮实验参数，不是历史已决定协议。
- 当前 capture 的 `learned_model_calls=0`；实际 Mask R-CNN 与缓存接入learned对象历史是另一个离线流程（上游LIO pose仍用oracle-static cloud），见 [perception README](../../perception/README.md)，不能改写成 live oracle 已被替换。
- 即时 `mapping/` 使用 GT pose/oracle mask，保留作诊断。离线重建显式 `pose_source=slam`、mask 仍 `sim_oracle`；旧 post-VIO 派生 Odometry bag 也不同于新 live final-pose topic。
- oracle 过滤归档不能证明 native 内部图完全无动态污染，视觉前端仍无图像动态 mask。对象 ID/context 查询边界见 [semantic_mapping](../../localization_mapping/semantic_mapping/README.md) 与 [query_scene_map.py](../../../../scripts/query_scene_map.py)。

## 已测范围

历史报告包含 USD 首段67.815 s到达/72.915 s停车、完整GT双楼梯230.335/235.415 s、1m/s命令上界首段18.595/23.615 s，以及移动中 oracle 行人 hold/resume 的首段 ONLY_LIO闭环。高速度段 `actual_command[0]>=0.95` 的6.18 s内XY均速0.902891 m/s，全部非零命令11.58 s内0.629510 m/s，不能称全程1m/s。独立landing LIO69KF评分RMSE0.035572 m；早期完整多层LIO漂移未验收。

续篇新增：去 periodic dump 后完整同 capture 的GT路线旁路对照正常完成，201共同exact KF的ONLY_LIO/post-VIO RMSE为1.919313/0.083742 m（各源首帧固定SE3仅评分）；这是旁路定位，非导航反馈。2,196条live final pose/status与诊断逐条匹配，当前 `dynanav_post_vio_20261109T052245Z` native prefix的2/2 CTest通过。三目标 measured post-VIO首轮第三段stall保留；新终点专用0.15/0.20m捕获运行报告`arrived=true`、source249.214996s到达、native流至254.415s，13项独立检查确认三阶段/两转向及5.1s零命令停车；2,367条final pose/status/native一致性已核对；journal有280条stale零命令hold，不能称全程无停顿。中间0.05/0.10m及stall/dwell等保护未改。两次视觉闭环都无行人，learned mask仍离线。离线74帧learned mask评测和RViz已实际执行。新结果不倒填旧运行身份，二进制/source manifest以各次收据为准。

## 启动与检查

在工程根目录设置 ROS 与数据环境，输出必须为新目录：

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=91
DYNANAV_RUN="$PROJECT_DATA_ROOT/embodied-navigation/experiments/dynanav-$(date -u +%Y%m%dT%H%M%SZ)"
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$DYNANAV_RUN/capture" --speed 0.15 --duration 90 --check
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$DYNANAV_RUN/capture" --speed 0.15 --duration 90
```

`--check` 核对资产/哈希和参数；选择 live reference 时用同一工厂纯实例化控制器，不启动 ROS/Isaac，不证明场景物理或输入质量。实际需要本机 Isaac6.0.1 RC和有权使用的资产。估计器独立启动、标定与模式说明见 [LIO/LIVO README](../../../../configs/simulation/dynanav/README_LIO.md)。所有地图、视频、RGB、日志和模型放外部数据卷，异常保留 result 和可用执行器故障状态。

历史提交 `bffa2b6` 验证为unit170（skip3）、regression83（skip5）、native1/1及其lint；它不是新增post-VIO/learned代码的最终测试计数。当前新增代码完整套件为unit217（skip3）、regression86（skip5），其余通过；native2/2，20个Python文件Ruff check/format、bash语法及vendor304文件大小/SHA通过。CPU fixture或文件检查不替代ROS/Isaac运行、完整地图或实机验收。
