# fast_livo2_core

FAST-LIVO2的滤波和局部建图库。默认构造链使用 Generic LiDAR+IMU+RGB，不开启 GPS。原生 Ouster type=3 需显式 `common.allow_ouster_lidar=true` 和 `common.require_complete_imu=true`，且关闭 feature extraction；image-free LIO 仍另需 `common.allow_lio_only=true`。不是滑窗因子图，也不把LIO/VIO当作两个独立位姿源。

## 核心入口与数据结构

| 文件/符号 | 职责 |
| --- | --- |
| `src/runner.cpp::run_mapping` | 节点/mapper生命周期入口 |
| `src/LIVMapper.cpp::run/sync_packages` | 回调入队、时刻切分及状态owner |
| `include/fast_livo2_core/core/common_lib.h::StatesGroup` | R,p,逆曝光,v,bg,ba,g，共19维误差；R右乘Exp增量 |
| `LidarMeasureGroup/MeasureGroup/PairedCloud` | 当前/下一切分点云与IMU/图像及源关联 |
| `src/IMU_Processing.cpp::IMU_init/UndistortPcl/Process2` | 静止初始化、积分传播、按点时刻补偿 |
| `src/voxel_map.cpp::StateEstimation/BuildJacobianRows` | 不确定性加权点到平面ESIKF |
| `src/voxel_map.cpp::UpdateVoxelMap` | voxel hash/八叉树与平面统计增量更新 |
| `src/vio.cpp::processFrame/updateState` | LiDAR几何支持的patch直接光度更新 |

主线为IMU传播→LIO→VIO，共享同一StatesGroup及协方差。导航odom/dense在post-LIO同源提交；彩云可在post-VIO生成。不能把同名world frame下的两种云当成完全相同阶段。

## 实际启用的算法

IMU以中值样本传播R/p/v及P，保存Pose6D供去畸变；不是为因子图构造IMU预积分因子。静止初始化用加速度估计重力，初始gyro bias为0，不是直接把gyro均值赋给bg。随后在线bias/gravity开关由生成配置开启。

LiDAR用0.5m根voxel/自适应八叉分层平面，点/平面协方差参与权重；0.15m leaf降采样供匹配。VIO有独立0.5m视觉hash，点附着LiDAR几何，8×8patch/4层forward直接法；曝光、on-demand raycast关闭。`updateReferencePatch`有定义但当前`processFrame`未调用，不能据函数存在宣称法向反复细化已启用。

## 参数、线程与调试

base参数位于根`config/slam/fastlivo.yaml`，普通入口生成覆盖：IMU200Hz→600样本/2.9s、img_point_cov1000，重力/bias在线估计true。当前require_complete_imu=false、map_sliding=false、surface-validity=0。

serial模式两个单线程executor在owner内优先服务IMU，再处理普通callback；1ms intake预算不能抢占callback。队列有512项/64MiB边界，通用点云另限8帧/65536点。LIO残差行可4线程，VIO patch线程配置意图1而构建宏元数据4，不能仅凭环境变量确认实际团队。状态更新与矩阵聚合仍有序执行。

调试先看源时间、初始化是否静止、量测残差关联、当前地图支撑与协方差，再看显示。小P、低残差或好看的RGB地图不独自认证绝对定位精度。使用/编译边界回到[SLAM模块](../../README.md)；公式、上下游及限制见[完整技术报告](../../../../docs/technical_report.zh-CN.md)。

## 显式同时间 pair 交接（2026-11-10）

`common.same_pair_vio_handoff` 默认 `false`，保留旧 raw/IMU 全局门和初始化时序；RTX override 显式设为 `true`。开启后，当前图像先经过原有逐点末时刻与 IMU 端点覆盖检查，再选择 LIO。pending pair 保存原始 header 整数 ns、校准队首时间、目标 t 和图像身份。只有当前非初始化 IMU 输出和正常 LIO handler 真正完成，才允许同一图像进入 VIO，不等待无关的新 raw/IMU。初始化、空点、身份变化、异常和停止均不能复用旧缓存；不是以 residual>0 定义成功，也不把成功等同于定位准确。

VIO 仍执行原零时长 `Process2` 和 `state_propagat=_state`，使用当前 LIO posterior 作为先验；没有回放 IMU、再次 deskew、改写源 stamp、改动逐点时间或并行更新状态。`require_complete_imu` 现有语义只检查最新端点达到目标，不证明成熟区间没有内部采样缺口。

`common.max_buffered_images/max_buffered_imu/max_buffered_prop_imu` 默认 0（旧配置不变），RTX 分别显式设 32/8192/8192。达到上限时在入队/时间戳更新前报错并结束当前运行，不删除未消费数据。它们是保守资源硬界，不是延迟或新鲜度目标；原源端 4 帧和 native raw 8 帧界限不变。启动 `[PAIR_HANDOFF]` 日志显示实际 flag 和三个上限。

`FASTLIVO_SYNC_OBSERVE=1` 配合现有 boundary 文件可记录阶段、消费/skip 和三个缓存的实际首尾时间/计数；默认关闭，观测值不驱动估计。实现/5 个 native fixtures 与限制见 [同 pair 验证](../../../../docs/experiments/dynanav-same-pair-handoff-20261110.md)。

## Explicit RTX complete-scan coverage (default off)

`common.complete_scan_watermark=true` requires the versioned atomic
`dynanav_sensor_msgs/CompleteLidarScan` source, the fixed audited OS1 config,
complete IMU, same-pair LIVO and original LiDAR timestamps. The default remains
legacy PointCloud2. Certified mode never falls back to a later raw tail after a
metadata/continuity failure. It preserves real pending points across an otherwise
unnecessary future-scan wait; completion is not free-space evidence. See
`docs/experiments/dynanav-complete-scan-watermark-20261111.md` for the exact
15ms coordinate-frame phase, empty-packet lifecycle difference and replay limits.
