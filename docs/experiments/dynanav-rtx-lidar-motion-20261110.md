# Isaac RTX LiDAR 有限运动诊断（2026-11-10）

## 结论

在官方 OS1 32线/10Hz RTX LiDAR + 原始RGB + 物理IMU + 新FAST-LIVO2 Ouster/LIVO构建上，完成一次30仿真秒的有限运动诊断。传感通路和视觉更新保持运行，但**未通过预定运动幅度、运动定位精度或逐点去畸变效果验收**。

实际转向约1.91°、前进约2.87cm。控制指令被原有0.25秒里程计源时刻新鲜度保护频繁截停。没有扩大运动窗口，没有放宽保护，没有启用旧探索/全楼路线或raw LiDAR operator map。

## 实验与失败证据

数据位于 `$PROJECT_DATA_ROOT/embodied-navigation/experiments/`：

- 首轮 `rtx_lidar_motion_20261110T153413Z`：约8.485秒，转向刚开始，generic ROS command分支的NumPy bool新鲜度记录不能JSON序列化，进程退出1。保留`capture.log`与`failure_status.json`；其中`result.json`的running只是最后一次成功周期保存，不表示仍在运行。
- 重试 `rtx_lidar_motion_20261110T153800Z`：`capture/observer/collector`，进程退出0。执行6000个严格0.005秒物理步，30仿真秒、35.3216墙秒，RTF约0.8493。
- 生产代码相对站立run14仅新增`fresh = bool(...)`，保持阈值及条件不变，差异保存在本次`freshness_only.patch`。没有用JSON通用fallback掩盖未知数据类型错误。

本次测试harness在实验数据目录：`capture_entry.py`、`command_pulses.py`、`collect_rtx.py`、`run_motion.sh`。使用已有ROS approved command分支，以post-VIO只作新鲜度门；不使用GT作导航或估计输入，不调用waypoint探索。声明的脉冲为：8..12秒wz=0.12rad/s，15..19秒vx=0.08m/s，其余零。硬位移上限1m，保留原倾斜、物理速度、电机、时钟及里程计新鲜度保护。

不能把脉冲窗口乘最大速度当作实际运动：

- 转向窗口200个50Hz控制帧，仅37帧非零（18.5%），实际指令积分0.0888rad（5.09°），GT净yaw变化1.9064°。
- 前进窗口200帧，仅36帧非零（18%），指令积分0.0576m，GT净位移0.0287m。指令积分本身也不是物理位移。
- 19秒后停车至结束。所有相关Isaac/native observer/collector/命令发布进程均已退出。

## 传感器与IMU因果链

- producer发布LiDAR298扫、RGB298帧、IMU6000条；native分别完整接收298/298/6000。
- 独立collector的298点云均与producer原始wire SHA256匹配；native的点数/layout/stamp逐条一致。298帧RGB的native stamp一致。每扫14861..15105有效点、全部32个ring；297个scan-start间隔均100000000ns。
- 独立collector的IMU为5950、clock5941，存在best-effort观测丢包，不能声称它们也完整。native诊断6000条IMU完整，诊断writer 14005/14005、dropped=0、io_failed=false。
- 255条post-VIO与原生状态对应，254次visual_update_applied=true，1次false。post-VIO header保留图像源时刻，没有改为发布时刻以骗过新鲜度门。
- 点云全为SENSOR/CARTESIAN/NONCOMPENSATED，保留逐点ns与ring，仅固定安装外参变换，没有GT或前置去畸变。
- 256组原生传播记录证明：上一完成时间段的VIO状态逐分量等于本段先验，再本段IMU预测/deskew，后LIO、后VIO。完整IMU对最大间隔5.001069ms，最后样本至目标的外推最大5.000687ms；积分目标区间一致。
- 源码仅在LIO阶段deskew，VIO不再deskew；没有当前扫描后验倒灌。**运行时没有逐点补偿前后坐标或访问次数记录，因此不能声称已逐点证明补偿收益或exact-once访问。** 离线评分脚本不做任何deskew或估计输入变换。

## 新鲜度问题与延迟拆分

46次hold、46次恢复，全部因sim_age超过0.25秒触发，未因wall_age超过0.5秒触发。hold启动源年龄0.255..0.315秒，墙钟年龄最大121.22ms；最长连续hold为0.180秒仿真/0.21409秒墙钟，未触2秒持续失效终止。

时钟类型没有混减：源/物理时间使用仿真秒，跨进程耗时使用同Linux主机的monotonic/steady/perf_counter。RGB原生事件与RP ReferenceTime全部一致；图像原生时刻到采集为15..110ms，图像到发布上下文为15..115ms，中间最多5ms。没有把该延迟称作stamp错误。

通信接收很短：LiDAR publish-call→native callback中位0.49ms，RGB中位4.38ms。图像callback→post-VIO中位约212ms包含等待，不是纯计算。

两个等待来源：

1. LIVO要求`lid_newest_time >= img_capture_time`。8.1秒图像时，scan8.0最后有效点为8.095616148秒，即使该扫描native pose end为8.1，也不能将其冒充有点覆盖到8.1。需等待scan8.1，而该包在物理8.315秒采到。不能伪造点、抹平逐点时间或改图像stamp绕过。
2. `LIVMapper.cpp:1553`的入口在区分LIO/VIO阶段之前，无条件要求新raw LiDAR队列非空。源码结合时序**强支持**它使已经完成LIO的同图像时刻VIO也等下一云：139/255帧LIO→VIO空档超过50ms，全部在下一LiDAR callback后0.389..1.351ms启动VIO。8.0秒帧空档200.281ms，15.1秒帧202.483ms。没有分支探针，不能写成直接量到了queue为空。

运动窗口82个估计帧的计算中位：IMU/deskew约0.698ms，LIO更新5.166ms，VIO传播/更新1.605ms；LIO后至VIO前空档中位93.436ms。不能用延长命令窗口掩盖同步问题，也不能把全部延迟归咎算力。

## 位姿评分和未通过项

评分仅离线读取GT，以5秒首个估计姿态作固定SE(3)初始对齐，scale=1，不拟合时间偏移；将GT base按真实IMU杆臂转至go2_imu，按Odometry源stamp插值，不用received_wall配GT，也不将对齐结果回馈估计器。

5..29.7秒248帧：位置RMSE1.542cm、最大6.525cm；姿态RMSE0.1008°、最大0.1902°。1秒RPE最大平移8.408cm，区间15.8→16.8秒。此误差相对仅2.87cm实际前进量不可忽略，不能称运动平移精度通过。

16.8秒最大误差主要为侧向位置差。传播后误差6.756cm，LIO后6.782cm，VIO后6.525cm；16.6秒VIO会增大误差，17.0秒又作约5.17cm校正。不能归咎单一阶段或凭轨迹平滑证明deskew收益。

原生renderer对物理装配的离线评分（5秒后）：LiDAR最大0.889mm/0.1753°；RGB最大5.66mm/前向0.5333°。RGB含初始化全段仍有9.25cm/1.98°，不冒充高精度动态标定通过。

## 产物与测试

本run：`sensor_stream_audit.json`、`build_identity_check.json`、`harness_manifest.json`、`command_publish.jsonl`、`capture/policy_frames.jsonl`、`capture/source_manifest.json`、`collector/cloud_*.npz`。60份采样NPZ含原始wire，均核hash；没有用NumPy结构体padding重建字节假充原始wire。

离线分析位于同experiments下`rtx_motion_offline_audit_20261110/`：`audit_motion.py`、`audit_freshness.py`、`motion_153800_v3/audit.json`、`freshness_153800_v3/{freshness.json,pipeline_by_image.csv,hold_intervals.csv}`。脚本有输入hash，旧版输出保留。

同次RGB/overview视频：`capture/multiwaypoint05_overview_and_rgb.mp4`，H.264 1024×512、10Hz、298帧、29.8秒，完整解码通过。是仿真视频，不是真机。

最终生产代码43份source_manifest全部匹配；native core/mapping仍匹配已验收新build SHA，未换构建。791个unit（3 skip）、93个regression（5 skip）通过；新增freshness测试修复前失败、修复后通过，相关204项通过。native源码/二进制未变，之前3/3 CTest通过。diff检查通过。未提交、未推送，原探索改动和旧实验均保留。

设备总GPU采样47次（含桌面，不是纯算法或保证峰值）：利用率中位12%/采样最大15%，显存中位5207MiB/最大5419MiB。

## 建议下一步（尚未执行）

先单独修正LIVO阶段的输入可用性门：已完成同时间LIO后，VIO不应仅因下一raw LiDAR尚未到达而饥饿；保持真实stamp、完整IMU与安全保护，加native阶段推进/空队列回归测试。然后用同一有限脉冲协议复测时延和实际运动，再补逐点deskew输入/输出及调用次数证据。不要先增加前置补偿或引入新的IMU反馈链。
