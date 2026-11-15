# Isaac RTX LiDAR 接入与站立验证（2026-11-10）

## 当前结论和边界

真实 RTX LiDAR + 原始 RGB + 物理 IMU 已接入新的 FAST-LIVO2 Ouster/LIVO 路径，完成一次 25 仿真秒的站立通路验证。**这不是运动精度、探索地图或导航路线验收。** 没有发出任何机体运动指令。

使用显式 `--point-cloud-source rtx_lidar`。该模式完全关闭 depth annotator、深度反投影点云及依赖深度像素的 oracle/learned 语义映射。RGB 仍是真实渲染图像，IMU 仍来自物理传感器，ROS IMU 不向估计器提供完美姿态（orientation 不可用）。旧 `rgbd` 入口保留兼容回退，但本轮及后续该 LiDAR 工作流必须显式选择 RTX，不能省略参数又回到深度云。

当前模型是官方 NVIDIA Ouster OS1 `OS1_REV6_32ch10hz1024res`：32 线、10 Hz、1024 水平方位采样，maxReturns=2。不是 Livox。也不等价于旧 Gazebo 的 64×480、30 Hz；本机官方注册表没有64线30Hz旋转 variant，定制匹配尚未实现或验收。

## 传感器、坐标和时钟

- LiDAR 位于真实 Go2 base 子树 `RtxLidar/sensor`，安装目标为 base 轴向一致、位置 `[0,0,0.25] m`；沿 USD 局部祖先链校验完整安装矩阵，拒绝 resetXformStack 和随时间变化的装配。
- ROS 发布 `/dynanav/lidar_rtx`，frame=`go2_imu`。点只做固定平移 `p_imu = p_lidar + lidar_mount - imu_mount`。没有 world 真值位姿变换或真值去畸变。
- 输出必须是 `SENSOR/CARTESIAN/NONCOMPENSATED`。Motion BVH 用于模拟真实射线发射时刻，不把输出变成完美去畸变点云。
- 原生 GMO `timestampNs` 是扫描起点，`timeOffsetNs` 是逐点 ns 偏移。ROS header 保留原生起点，t保留偏移，不按render帧或点序伪造时间。GMO 实测 channelId 已是0..31，不对 USD authoring 中的1..32再减一。
- Ouster wire layout：x/y/z/intensity float32，t uint32，ring uint8，point_step=32。reflectivity/ambient 未提供，明确置零；range为原生传感器距离的毫米值。有效回波必须有 VALID bit。仅过滤无效/机身回波，不做预算抽样。
- Native ingress 上限为65536点、2MiB data，producer同样硬限制65536，超限拒绝而非截断。operator通用解码上限100k，但拒绝把raw旋转RTX扫描直接用单一header位姿写图。
- RGB 按该 render-product 的原生 NEW_FRAME 完成事件采集，同时复制 RGB/CameraParams。事件的 rational time 与本RP ReferenceTime 交叉核对，再经 SDK 转为仿真时间；不使用回调时刻或当前body时间补stamp。拒绝缺失/非正 denominator，避免 SDK 对 `(0,0)` 的“当前时间”fallback。
- RGB辅助 `IsaacReadSimulationTime` 在此多RP图中可能execOut=0，因此不拿其未执行的零当时间。原生事件及ReferenceTime保留在逐帧证据中。
- LiDAR用官方GenericModelOutput Writer复制每次完整扫描；RGB用SDK Camera同样的完成事件途径，避免未触发的RGB Writer调度及轮询覆盖。两路有界队列、原生stamp去重，倒退/溢出报错。初始化及停止边界不完整帧不冒充完整帧。

RTX要求FSD、multi-tick、perSensorTickTlas和物理Fabric更新，history period在stage/render product建立前设为1/200。相机注册OmniSensorAPI。原RGB-D后端保持不变；RTX profile记录原请求并明确选Fabric。物理仍为每次恰好0.005s、step_count+1；render-only flush不额外推进物理。

## FAST-LIVO2 不是只换配置

实际默认旧prefix的构造门硬限制generic/type0。`observer1/runner.log`实跑拒绝type3，故不能声称该旧可执行文件仅换YAML即可使用。此结论仅针对核过的该旧构建，不泛指上游所有FAST-LIVO2二进制。

新配置：`configs/simulation/dynanav/fastlivo_rtx_os1_32.yaml`，type3、32线、非feature、point_filter_num=1，明确 `common.allow_ouster_lidar=true` 和完整IMU。点已在IMU系，因此native LiDAR extrinsic为I/0；RGB Rcl/Pcl仍用实际相机静态标定。

新源码补丁：`third_party/fast_livo2/patches/0006-opt-in-timed-ouster.patch`。默认不开Ouster，只显式放行type3，保留GPS/HILTI/IMU等门；校验字段/layout，修正Ouster初始化传播原点、工作副本时间基准、LIVO点序及deskew首点边界。不覆盖旧库。

新buildroot：`$PROJECT_DATA_ROOT/embodied-navigation/build_cache/dynanav_rtx_ouster_20261110T134330Z`。传给launcher的 `--binary-prefix` 是这个含install/ros的根，不是install/ros本身。旧prefix的RTX preflight现在明确失败；新prefix通过。

Core SHA256：`991840a86e89cb3606284728fef7ae31b167c12b0bc2a06e56e822b996315ba2`。
Mapping SHA256：`07765ef0b3f066161160e68353e4b99bbbaa266193931b2e605d6f620b9342d9`。

## IMU因果链，没有新增前置补偿

本轮仅用native内部deskew。阶段是：同步 → 上一已完成处理时间段的状态/偏置 + 本段IMU预测 → 本段deskew一次 → 本段LIO更新 → 同图像时刻VIO更新 → post-VIO输出。当前LIO/VIO后验不会倒回作为当前deskew的必需输入，VIO分支不再deskew。

LIVO会按图像时间切分原始扫描，因此“上一已完成状态”指上一时间段/切片，不必是上一整圈雷达。已完成前缀用于后续时间段是因果的。LIO/VIO共用状态和协方差，不应包装成两份独立位姿观测再融合。本轮没有实现任何前置补偿、反馈支路或重复补偿。

源码线索：LIVMapper.cpp的主循环sync_packages→processImu→stateEstimationAndMapping；IMU_Processing.cpp的last_imu/last_prop_end_time与bias积分、仅LIO条件下的deskew；具体行号见同次新build的source snapshot。

## 真实run14证据

外部数据目录：`embodied-navigation/experiments/rtx_lidar_integration_20261110T213000Z/`。
使用 `capture14/observer14/collector14`，不是此前失败轮。

- 5000个0.005s物理步，25仿真秒；采集循环29.8513墙秒，RTF=0.8375。
- producer：248扫LiDAR、248帧RGB、5000条IMU。独立collector及native接收诊断数量完全相同；native的LiDAR stamp/point_count/layout、RGB stamp逐条匹配。
- 248个collector点云字节SHA256全部与producer匹配。诊断written=attempted=11505、dropped=0，writer_io_failed=false。
- 扫描起点0.1..24.8秒，247个间隔全部100000000ns。RGB原生时间0.2..24.9秒，连续10Hz；启动/停止边界没有补假帧。
- 每扫14875..15098个有效点，每扫保留全部32个ring；开放场景有效点数不等于发射射线总数。
- 205条post-VIO状态，204次visual_update_applied=true，1次false如实保留。状态共享同一估计器，不是独立的第二路定位。
- 1250条50Hz策略记录，actual_command全部 `[0,0,0]`。终态是interactive_motion_budget，仅表示站立预算结束，不是到达导航目标。
- GPU总占用采样39次：显存最大5515MiB，GPU利用率最大22%；这是该次桌面总设备采样，不是保证峰值、30Hz预算或纯算法开销。

离线姿态评分只读truth/scoring记录，未参与点变换或估计：
- LiDAR native pose在5秒后相对物理安装的最大位置差0.66mm、最大角差0.110°。
- RGB renderer在5秒后相对原生图像时间的安装期望最大位置差4.30mm、最大前向差0.662°；全段含初始化时最大9.25cm/1.98°。不能把站立通路验证写成高精度运动标定通过。
- 图像发布相对其原生时刻延迟15..115ms，保留原stamp不伪造当前时刻。

主要证据：`run14_audit.json`、`run14_pose_audit.json`、`capture14/{result.json,rtx_lidar_frames.jsonl,camera_render_pose.jsonl,sensor_publish_trace.jsonl,source_manifest.json}`、`collector14/received.jsonl`、`observer14/diagnostics/`、`resources14.*`。视频是同次真实仿真RGB/overview，非物理真机。

## 反例和未验收项

run3/4的视觉update虽出现，相机存在初始姿态冻结，不能作完成证据；run9轮询覆盖、run11只有LiDAR/RGB=0也不是融合通过。保留这些产物，不改写历史身份。

当前raw旋转扫描不直接送operator map；LiDAR去畸变后的局部感知/建图尚未接好，互动运动保持fail-closed。没有恢复旧探索，没有前进/转向/全楼路线验收。64×480@30Hz、运动中精度和动态障碍语义也未验收。

官方OS1资产已从真实加载结果冻结到本机数据卷的`runtime_assets/sensors/isaacsim_6_0_1_rc7/`，校验provenance和SHA后复用，避免每次实验检查远端资产服务器。该资产来自官方USD，不是自造扫描参数。

## 测试与复现入口

同一代码的最终测试：790个unit测试（3 skip）、93个regression测试（5 skip）通过；native CTest 3/3通过，包含Ouster模式/字段/完整IMU区间与deskew边界测试。`git diff --check`通过。站立验收当时run14的source_manifest全部与生产代码一致；后续有限运动仅修复control.py中fresh的Python bool类型，见[运动诊断](dynanav-rtx-lidar-motion-20261110.md)。

本次实际运行脚本保存在同一外部实验目录的`run_standing14.sh`，包含独立observer、collector、资源采样和全零命令心跳；它引用新buildroot并显式选择RTX。复现实验必须另建输出目录及专用ROS domain，不能重用原证据目录。RGB标定来源需指向本次`capture14/result.json`中真实的相机外参。

本轮结束后无Isaac/native observer/collector/命令心跳继续运行。源码与测试均未提交或推送，原有未提交探索改动保留。
