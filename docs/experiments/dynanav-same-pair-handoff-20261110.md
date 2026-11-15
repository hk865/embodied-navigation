# FAST-LIVO2 同时间 pair 的阶段化交接

日期：2026-11-10 UTC。仓库已有 RTX/Ouster 和探索改动保留，无 commit/push。

## 动机与限定

旧 serial_observed 轨迹存在当前 LIO 已正常完成、图像仍为相同目标时间、raw 新队列为空时，入口 RawEmpty 门阻止 VIO 的实证。修复只解除已经完成的同 pair 对无关未来 raw/IMU 的依赖。新 LIO 的点时间覆盖和 IMU 端点检查没有放宽。`require_complete_imu` 不是内部缺口检测器：成熟 IMU 前跳仍沿用原 WARN/接纳策略，本轮没有改写积分缺口策略。

## 实现

- `common.same_pair_vio_handoff=false` 默认保留原时序；RTX YAML 明确 true。同一最终 binary 可只覆盖此参数作 false/true 对照。
- `PendingLivoPair` 是 owner 单写状态，不依赖可选观测。选择时保存原始图像 header 整数 ns、校准队首时间、目标 t、图像 data 身份及 sequence。原整数 stamp 直接取自 callback 的 header，不从 double 反推。
- 已选择不代表成功。Process2 开始时仍初始化的 pair，即使本次刚完成初始化，也不成为 ready。初始化可能不重写输出，因此仅在新路径清除旧输出，禁止旧点云充数。空点/空降采样也不能进入 ready。
- 正常 LIO handler 到尾部，拥有当前发布点、有限 state/cov，且传播目标等于该 pair，才设 LioReady。不要求残差数大于零，正常建图/启动完成也可交接；这不是导航 readiness 或精度认证。
- ready pair 必须精确匹配 source stamp/校准队首时间/目标/图像身份，方可绕过 raw/IMU 新队列门。图像只消费一次。VIO 继续调用原 Process2，零次新 IMU 积分，不重复 deskew，并保留 LIO posterior→state_propagat 的先验关系。
- skip 只退役自己的图像；身份变化不偷取另一图像。异常/停止使 token 失效。VIO 未执行完不开始下一个 LIO；不并行改共享状态/地图。原 `pcl_proc_next` 所有权保留。

## 缓存硬界与观测

显式参数 `common.max_buffered_images/max_buffered_imu/max_buffered_prop_imu` 默认 0，保留旧配置；RTX YAML 为32/8192/8192。界限前拒绝新样本并报出队列名、计数/上限，终止当前 owned run，不静默删除未消费数据或替换时间戳。源端4帧、native raw8帧不变。这是保守资源界限，不是降低延迟的办法，也不放宽 freshness。

旧串行观察样本的 raw/image/primary-IMU 计数峰值为3/3/64；初始化期间2/2/64。旧 probe 没有 prop-IMU 大小，不能声称已验证其正常峰值。8192高于该30 s实验全部6000条IMU，是初始保守硬界，需实跑新观测后收紧，而不是以初始化4.2s臆测缓存需要。

默认关闭的 `sync_observation.h` 沿用 v1 schema 和 reason0..15，新增16 IdentityChanged、17 PairSkipped、18 LioUnfinished、19 VioUnfinished。pending_pair 含 enabled、sequence、phase、outcome、source_stamp_ns/valid、image_time/target_time、retired/skipped；phase0..7和outcome0..7见源码枚举。`pair_retired` 区分正常VIO完成与skip。View 新增 cache_counts（primary IMU/image/prop IMU）、cache_endpoints（各 oldest/newest）、cache_valid；这是实际 O(1) 队列首尾读取，缺失用valid区分，不按采样率猜跨度。

## 构建与 native 验证

BUILD_ROOT：`/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_same_pair_20261110T171010Z`

- `install/ros/lib/libfast_livo2_core.so` SHA256：`667241240081236481f7d04f756aae148fda79962af96f4fba68ff3827f75f6b`
- `install/ros/lib/fast_livo2_ros/fastlivo_mapping` SHA256：`0f47955abaee9d2666be294c42853f343ee87e6af63eadd1895456aaa0d4ec79`
- 使用独立 source/build/install，`--parallel 2`。原 Ouster/observation prefixes 和 source/hash 保留。
- CTest：5/5通过，1.29s。包括原 LIO、post-VIO、Ouster；新增 observation 与 same-pair。
- 真正调用正常 LIO→VIO 的 fixture 中，新 raw/IMU 队列均空仍能交接。VIO零时长准备后 state/cov 与前一 LIO posterior 逐元素一致，先验/last IMU/传播目标/点时间与slice所有权不变。
- 初始化、空点、未完成、不同 stamp/目标/图像、缺 stamp、停止/异常、重复消费、前段 point/IMU端点不足和三个缓存界限覆盖通过。
- 同 binary feature OFF 保留旧门/初始化时序，feature ON 正常结果 state/cov/prior逐元素相同。两个新增fixture的 observation OFF/ON 签名相同；OFF不写文件，ON分别7/91行且row序列连续。

可核文件：`test-result.txt`、`build/test/Testing/Temporary/LastTest.log`、`artifact/observation-result.json`、`artifact/*-on.jsonl`、`source-manifest.json`、`build-identity.json`。启动日志 `[PAIR_HANDOFF] enabled=... cache_caps image=... imu=... prop_imu=...` 可核实际配置。

## 同 binary 的 Isaac 实跑对照

实验根：`$PROJECT_DATA_ROOT/embodied-navigation/experiments/rtx_pair_handoff_20261110T171223Z/`，`baseline`与`fixed`均真实完成30仿真秒、6000个5ms步并退出0。两者为同一上述binary，serial输入模式、同32/8192/8192硬界、同新Writer观测、同原生观测和同短运动协议；仅common.same_pair_vio_handoff为false/true。`runtime_parameters.json`由运行中parameter service读取，验证实际开关、界限、complete IMU、type3/32线，不只读取YAML。

8..12秒原地yaw命令上限0.12rad/s，15..19秒前向命令上限0.08m/s，其余零；原0.25秒源年龄、0.5秒墙钟年龄及物理/电机/倾斜保护未变。是同协议实时闭环，不是同raw数据重放，放行变化导致真实运动/输入不同。

- 缓存LIO已ready→选择同t VIO：中位93.691→0.407ms，P95 202.022→0.831ms，超过50ms的pair 143→0。
- 另一原生切点（LIO更新后kind13→VIO Process2开始）包含LIO尾部发布和调度：中位95.475→2.067ms，不与上一个指标混用。
- 图像publish-return→post-VIO：中位216.524→114.510ms，P95 315.767→224.700ms。整体源交付仍成批，端到端延迟标准差47.38→95.83ms，不能宣称全链路抖动改善。
- 原生输出墙钟相邻间隔P95 212.76→213.89ms、标准差90.31→93.46ms，基本未改善；成熟源stamp始终10Hz，无重复/回退，不是均匀墙钟10Hz保证。
- 固定版257次VIO均有同target此前真实LIO完成，未见重复或错stamp，41个初始化pair明确skip，298个pair全部退休。138次在新raw队列0时仍成功使用同帧缓存进入VIO。这是pair消费证据，不等于逐点deskew访问次数证明。
- 第一个正常LIO（4.3s）没有此前VIO，先验与4.2s初始化后状态相同；随后256组均接更早VIO，逐分量状态差0。不能把正常bootstrap标成循环依赖失败。
- 转向/前进命令放行15%/16.5%→69.5%/69.5%，400个运动tick非零63→278。GT实际yaw 1.213→13.830°、前向窗口净位移1.194→5.409cm，不能用命令积分代替实际位移。
- 5秒后的初始SE(3)对齐位置RMSE 1.413→2.536cm，最大3.724→7.957cm；1秒RPE最大4.879→9.930cm。激励变化，不能单因素判精度回归，也不能说运动定位精度通过。
- baseline/fixed分别原生完整收到6000 IMU、298/299云和RGB，全部点云原始wire SHA/布局/stamp匹配。额外collector的IMU5941/5939、clock5933/5934有best-effort观测丢失，未冒充完整。
- RTF约0.85077/0.85078。视频298/299帧、10Hz、29.8/29.9秒，完整解码通过。结束边界多交付1完整帧如实保留，不补假帧。

## Writer callback 与缓存驻留

只在`capture/lidar.py`增加标量单调时钟、callback/read序号和入出队深度，原wire/header/t/ring不变。既有4扫FIFO不变，字段进入原有rtx_lidar_frames.jsonl，不增加回调内大文件I/O。

- Writer队列观测峰值2/容量4，至少2扫余量，未满。队列驻留P95约33.7/33.4ms；callback→发布中位9.97/10.79ms。
- raw物化/复制中位约0.25ms（含raw.numpy()，不是纯memcpy），其后解析/pack/hash约1.6ms。
- 同一次drain的两扫来自不同callback，典型相隔约25ms，之后dequeue只隔约0.5ms；callback端本身已有长短交替，drain又将其压到一起。不能把全部批次归于后端门，也不是队列容量耗尽。
- acquisition_physics_ns仍是Python消费/发布物理上下文，不是原生射线采样或GPU完成时刻；scan-start到该上下文115/215ms不能当GPU耗时。GPU完成→callback仍未测，保留边界。

## 有界余量的实跑覆盖

两轮缓存观测primary IMU峰值64/8192、image 3/32，未触硬界。prop快照为0，与uav.imu_rate_odom=false配置一致，只能说明该旁路未积压，不能作为8192容量压力验收。合成fixture覆盖硬界exact-boundary和越界无部分入队；源端4扫是独立层，不能与DDS/ingress/native容量直接相加成“可承受延迟”。

缓存保留未来未消费数据及一个last_imu前缀，不把已消费IMU放回主队列或重复积分。沿用原迟到/旧帧处理；没有增加跨已提交epoch的重排或状态回滚。缓存也不修复实际缺样。本轮6000条native IMU完整，不等于best-effort链路今后不会丢样。

## 最终验证与限制

797项unit（3 skip）、95项regression（5 skip）通过，diff检查通过。Writer相关68项focused通过；native 5/5及观察OFF/ON通过。曾有1项旧YAML精确字典期望需同步新增开关/界限，原失败log保留在unit_before_profile_expectation_update.log，最终全量重新通过。旧0006/构建/证据保留，0007已验证应用及文件hash，未提交或推送。

逐点deskew前后/visit-count、运动定位精度、源回调之前的GPU时序、均匀墙钟输出和完整导航仍未验收。未恢复探索，raw旋转点云仍不能直接写operator map。当前可确认的是同帧交接时序改善与有界缓存落实，不是整个感知/控制问题已解决。

最终分析入口为实验根的`analysis_CANONICAL.json`，总报告`analysis_comparison_final/README.md`和`comparison.json`，两轮独立canonical为`analysis_baseline_final/`与`analysis_fixed_final/`。24项离线CPU测试通过，输入和分析脚本SHA逐项核对。命令积分按逐记录left-held时间算，不以非零条数乘固定20ms冒充实测时长。
