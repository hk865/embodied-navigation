# RTX LiDAR 缓存与阶段时序实测（2026-11-10）

## 范围

沿用OS1 32线10Hz RTX LiDAR、原始RGB、物理IMU和30秒短运动协议：8..12秒wz=0.12rad/s、15..19秒vx=0.08m/s，其余零；原0.25秒源年龄、0.5秒接收年龄及物理保护不变。未启探索，未修改估计器同步条件、时间戳、点云消费、IMU因果顺序或控制逻辑。

实验根为 `$PROJECT_DATA_ROOT/embodied-navigation/experiments/rtx_pipeline_compare_20261110T162648Z/`：
- `serial`、`staged`：同一原Ouster二进制、相同现成线程交接/边界计时开关，均已完成并退出。
- `serial_observed`：独立构建仅加默认关闭、有界的同步分支/pair观测，已完成并退出。
- `rx_decode`和`staged_observed`仅准备了脚本，**未运行**。

这是同协议实时闭环对照，不是同一份raw数据重放；模式和观测开销可改变调度、保护放行及实际运动，不能把误差差异作单因素因果结论。

## 多线程对照结果

实际trace确认serial接收/解码/owner为同TID；staged有独立接收、LiDAR解码、图像解码和owner四个TID。输入解耦真实存在，但LIO/VIO仍顺序更新同一滤波状态和地图。

serial→staged：
- 图像commit→post-VIO中位210.06→211.15ms。
- LIO结束→VIO开始中位89.55→92.73ms；超过50ms的等待137→136帧。
- LiDAR receive→commit中位0.397→0.706ms；图像0.167→3.275ms。
- 转向保护放行16.5%→15.5%，前进20.5%→22%。GT转1.560→1.304°、前进2.994→1.838cm。
- 位置RMSE1.555→1.679cm、最大3.907→7.301cm。运动量很小且不同，不能宣称运动精度通过或staged普遍更差。

两轮均原生完整接收6000 IMU、298云、298 RGB，298云wire hash/布局/stamp逐条匹配；255个post-VIO状态含254次真实视觉更新、1次false。独立best-effort collector有IMU/clock丢包，分别在各run的`sensor_stream_audit.json`中披露；不影响已核实的native完整计数。

## 不是5Hz

两模式post-VIO源stamp均为4.3..29.7秒连续255帧，间隔100ms±1ns，稳态源频率10Hz。41个早期空云/初始化VIO记录不发布位姿；结束时还有待处理帧，不是隔帧删除。

墙钟输出存在长短交替：serial有114个间隔>150ms、115个<50ms；staged为116/116。首末输出间实测墙钟平均约9.35/9.27Hz，不是5Hz。210ms是单帧延迟，不能取倒数当输出频率；10Hz源吞吐也不等于位姿足够新鲜。原生源时刻、物理上下文和monotonic墙钟分别分析。

## 两层成批/等待

1. **源发布已经成批。** serial的298扫由32次单扫和133次双扫Python发布上下文组成；staged为30次单扫和134次双扫。批内发布相邻常<5ms，批间常>150ms，scan-start仍连续10Hz。

`rtx_lidar_frames.jsonl`里的`acquisition_physics_ns`实际上由`publish_rtx_pending()`写入，是Writer缓存被Python取出/发布的物理上下文，不是原生射线采样或GPU完成时刻。射线时刻用header+t；Writer callback→Python drain尚无独立时间观测，不能把115/215ms统称GPU延迟。

2. **同帧已完成LIO后，确有额外队列门等待。** 独立探针直接记录了37条`RawEmpty/false`快照，覆盖26个已完成有效LIO且目标仍为同图像t的pair。例：row568、pair91、t=9.2秒，last_lio=9.2、LIO正常完成且有有效残差/输出，图像队列2、IMU队列42且最新9.409999847秒；raw云队列0，入口实际返回RawEmpty，仍未进入同t VIO。这已是分支实测，不再只是下一云到达的相关性。

前段必要等待不能取消：LIO仍需真实逐点时间覆盖图像t及完整IMU，旧缓存或伪stamp不能代替。不能把所有RawEmpty都判为多余，也不能把没有记录的pending点云时间范围当成覆盖证明。

## 直接探针证据及边界

观测构建：`build_cache/dynanav_sync_observe_20261110T162901Z`，core SHA256 `4e60b65fc3da087e9b5fc8a61426b00ca731be316bf235a4d4a5a2f1f4a8400a`。仅开启`FASTLIVO_SYNC_OBSERVE=1`，复用现有排他创建、无fsync、共享8MiB上限的边界日志。旧构建与生产源码未变。

- 原3项native测试＋新增off/on边界fixture 4/4通过，状态签名相同；原3项观测ON复测也通过。
- 去掉观测hook、还原包装return后源码与baseline逐字一致；原条件/返回值/消费未混改。观测I/O仍可能影响墙钟调度，不能声称绝对零开销。
- 实测1843条观测row连续且有final；167104次返回原因累计相等；文件约1.24MB，未触8MiB上限。计数是poll次数，含启动/收尾，**不是等待秒数或时间占比**。
- 298次LIO selection：257正常完成、41初始化/空云skip；297次VIO selection：256 processed，其中255次实际EKF视觉更新。末pair298仍pending。selection与观测`lio_effective`都不冒充算法原有ready或精度证明。
- 本轮6000 IMU、299云、299 RGB均原生接收，299云hash逐条匹配；结束边界比前两轮多交付1完整帧，如实记录。独立collector IMU5945、clock5938，存在观测丢包。
- 现成边界文件仅产生聚合行，没有逐帧R73 schema，虽设了相关环境变量也不冒称逐帧计时已获得。精确交接来自23列pipeline trace及原生状态记录。
- 仍无逐点deskew前后/visit-count证据，不能宣称补偿收益或逐点exactly-once通过。

## 产物与后续

各run保留`capture/observer/collector`、视频、命令、原始wire采样、source manifest及`sensor_stream_audit.json`。对照在`analysis_comparison/`；直接分支在`analysis_serial_observed_sync_v2/`；源发布批次在`analysis_source_batches_v2/`。观测构建的`artifact/`含schema、补丁、验证和限制。离线解析13项CPU测试通过。原有未提交探索工作保留，未提交或推送。

接下来可设计带真实stamp与成功/skip状态的轻量同帧pair缓存，复用已保留的LIO点/地图/状态；只在依赖已满足的同t VIO路径调整队列门，并验证唯一消费、初始化、空云、先验/协方差语义。另需补Writer callback→drain时间，定位源发布成批，不能用增加线程或扩大队列掩盖。以上逻辑调整尚未实施，不能把本次诊断当作已修复。
