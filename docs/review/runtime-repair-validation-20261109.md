# 运行故障修复与真实重验（2026-11-09）

本次修复针对 capture 收据/生命周期、observer 就绪与 supervisor 回收；同时重新执行真实 Isaac 在线行人和 Gazebo PID 路线。**离线测试通过不代表路线通过；本次两条路线均没有取得验收通过。** 没有放宽 8 帧队列容量、停车条件、物理或来源门限。

## 如何阅读结果

| 层次 | 当前结论 | 不代表什么 |
| --- | --- | --- |
| Python 故障路径与离线回归 | 见下方最终测试记录 | 不证明真实传输、定位、感知质量或路线成功 |
| Isaac 在线行人 | 新 ready 握手成功；原配置再次触发 LiDAR buffered source capacity | 不把子进程退出 0 或收尾完成当导航通过 |
| RGB 发布/接收边界 | 取消前 174 次 RGB 发布返回，native 仅收到 70 帧 | publish 返回不是 DDS 送达；尚未确定内核/传输内部根因 |
| Gazebo PID | 实际 SLAM 前进，但在 R85 路线证书门保持，没有到达 | 零命令保持不是终态停车，也不能算固定 5 秒完成 |
| 收尾 | 所有本次拥有的任务已按各自监督记录收尾 | 不清理用户其他容器/服务，不宣称整个 GPU 或系统空闲 |

## 修复内容与契约

### capture 收据与原始异常

- journal 逐个关闭，一个流失败仍尝试其他流。
- 异常 `repr`、traceback、`report` 属性获取/序列化和运行证据采集分别保护；二次失败不能替换首错或跳过后续资源回收。
- 初始写收据、配置解析和取消纳入生命周期。现有 `stage` 名保留，新增 `outcome` 区分 running/completed/failed/cancelled。
- 成功释放过的视频和 ROS 资源不会因后续错误再次释放。
- 真实 SDK 会在构造时替换 SIGINT，并在 fast shutdown 中直接结束进程。capture 在 SDK 构造返回后重新拥有 SIGINT/SIGTERM，先写终态再调用 native close。
- `app_close_requested` 与 `app_close_returned` 分开；completed 表示采集工作完成，不表示 native close 一定返回或外部进程清理已独立验证。
- SDK 尚未返回的构造资源、SIGKILL/native crash 和持续磁盘故障仍不能由 Python handler 保证回收/落盘。

### observer 与 supervisor

- `launch.json` 继续只记录来源。独立的原子 `ready.json` 在所拥有 mapper 暴露相机/IMU/点云订阅后写入；不声称已经收到帧或估计器已初始化。
- observer 启动时 worker 死亡会阻止 capture；mapper 或 camera blackboard 意外退出（包括 0）均失败。
- 回收进程组时，不因组长已退出而跳过还存活的后代。
- 每个子进程独立 best-effort 回收；重复取消不打断剩余回收；终态监督收据记录首错、cleanup_errors 和进程退出。
- launcher 在有界 TERM/KILL 后移除 ready 并写 `shutdown.json`；capture 即使退出 0 也必须有自身 `outcome=completed` 终态收据，否则监督失败。监督只记录白名单 RMW/传输/trace 环境，不记录密钥或完整环境。没有更改 native 生命周期门、消息时间戳、QoS 或队列容量。

### 可选发布边界诊断

设置 `DYNANAV_PUBLISH_TRACE=1` 后，`sensor_publish_trace.jsonl` 对 RGB 和选中的 frontend cloud 记录 `publish_call`/`publish_returned`、原 source stamp、monotonic 时间和数据字段字节数。数据量为每帧两个标量记录，不保存图像内容。

- 文件仍由 Receipts 拥有并逐条 flush；flush 不等于 fsync。
- `delivery_confirmed=false` 始终显式保留。
- 时间括号含前一条 journal 的开销，字节数不是完整 DDS 序列化长度。
- 可与现有 `FASTLIVO_INGRESS_TRACE=1` 的接收/commit 记录对齐；不修改发布频率或阈值。

## 真实 Isaac 运行

证据目录标识：`repair-revalidation-20261109T130512Z/online-person`，保存在用户 `PROJECT_DATA_ROOT/embodied-navigation/experiments`，不上传原始资产/日志。

- ROS domain 126；basement-to-ground；0.30 m/s 命令上限；210 s 预算；200 Hz IMU；tile1。
- 相同模型 SHA `73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e`、native prefix、测量参考、标定与原物理/停止门。
- 使用原 `rmw_fastrtps_cpp`、强制 `UDPv4`，仅增加两侧本地 trace 与本次生命周期/就绪修复。
- source 17.514999390 s 的云触发 `cloud_points=7944 lidar_cloud_queue=8 image_queue=0 imu_queue=180`。7944 小于原 65536 点上限，触发项是 8 帧缓存。
- observer 自行退出 2，supervisor 返回失败并回收 worker/capture，`cleanup_errors=[]`。没有完整路线/终态地图验收。

### 缺图与队列的因果边界

已结束文件 `audit/new-boundary-review.json` 保存逐帧配对、完整 missing stamps、时延、取消边界和 16 个证据/源码文件 SHA。

- native 取消开始之前，174 个 RGB 的调用/返回全部成对；其中 70 个到达并 commit，104 个无 native 接收回调。
- 最后连续缺失的 8 个取消前 RGB source 为 16.715–17.415 s。17.515 s 同帧 RGB 和 17.615 s 尾帧是在 native 取消后发布，不能计为传输损失。
- 同期云取消前 175/175 到 callback；前 174 commit，最后一帧在容量异常处退出，故无 commit CSV。
- 已收到 RGB 的 publish-return→native-receive 中位约 0.316 ms、最大约 1.640 ms；receive→commit 最大约 3.836 ms。
- 最后有效 image 到致错 cloud 相隔 0.9 **仿真秒**、约 2.711 **墙钟秒**。
- 源码 `sync_packages` 在 `img_en` 且 image buffer 为空时停止消费云；后续云/IMU继续入队，最终容量守卫中止。这解释了本次积压机制，但未确定 publish-return 至 native-receive 之间的内部损失原因。

这不证明 capture 拆分引入丢图，也没有排除 DDS、分片、socket、订阅调度或主机状态。不得直接扩大队列容量换通过。

### 合成传输反例

另用独立进程、50 张 512×512 rgb8 合成图像、10 Hz **墙钟**、best-effort/depth4 reader 对照。UDPv4 收到 11/50，DEFAULT 24/50，SHM 22/50，LARGE_DATA 22/50。它们均没有给出可靠修复，因此没有据此改变默认传输。该探针无 native/Isaac/模型负载，不是路线验收，不能将不同 run 的计数直接当成协议优劣排名。

### 真实取消重验

首次在线重跑在 SDK 原信号路径中仍留下 running 收据；第一次受控取消（`cancellation-check`，domain132）虽取得 failed 终态，但 SDK 将 SystemExit 吞在 async watcher 中，ffmpeg 已因组信号退出，随后记录 BrokenPipeError。两份原始记录保留，不追溯改为通过。信号处理据此改为闩锁请求、在 Python 安全边界处理；最终重验 `cancellation-check2`（domain133）在首个真实 sensor 发布后向 supervisor 发送 SIGTERM：supervisor 退出143，capture 收据 `outcome=cancelled` / `CaptureCancellation(130)`，`app_close_requested=true`、`app_close_returned=false`，worker/observer/capture 均由 supervisor 回收，`cleanup_errors=[]`。capture 经过 SDK wrapper 的进程退出为1，未误报0或 completed。原操作异常若出现仍单独保留。这验证真实SDK取消/收据/回收，不是路线验收；最后信号修复后只做此短补验，没有重新跑完整路线。

## 真实 Gazebo PID 运行

证据目录标识：`pid-live-hold-20261109T130410Z`。实际 run 为 `20261109T130410Z_pid-live-20261109T130410Z_fe0d`。

- 新 domain 227、380 s 有界预算；当前源码/config/scripts/start.py 冻结并只读挂载，SHA 清单保留。
- native 依赖来自既有镜像 `sha256:219f864fe3bda7625472e70fc63f1895e2153b844716e72f34a1596010f6d50b`，没有声称重建了全部依赖。
- 360 个可读文件与镜像相同，5 个差异均属于 DynaNav；PID/controller、物理门、可读配置和 start.py 哈希相同。17 个镜像文件不可读，明确未比较。
- 实测 SLAM 净前进 15.388 m，末位姿约 `[13.729448, 6.896231, 1.239191]`。
- `r85_actual_start_physical_connector_not_certified` 在 sim 214.535–347.035 s 持续；嵌套原因为 `actual_curve_physical_space_not_certified` / `continuous_outer_bound_not_certified`，位置 near ramp_23。
- 此段约 156.36 wall /132.50 sim 秒，仅 1.47 mm 净位移，但 command 为 untyped_zero/revoked，不是 ordinary terminal zero_hold。`region_arrivals=[]`，有效停车样本为 0。
- launcher 和 15 个组件退出 0、error=null，仅说明进程退出，并不推翻路线未完成。自己的容器移除、无仿真残留，原有 napcat 未改动。

### 5 秒停车判据

只累计健康 ready、ordinary_parking/ordinary_zero_hold、同一非空 terminal event 绑定且零命令的 live 样本；命令 sequence、控制/SLAM 时钟和源测量须推进且 age 合规。gate_hold、failed、revoked、缺 event、冻结、超时均断开窗口。按末戳减首戳计算，不能按样本数替代时长，并另核对实际速度/位移/yaw。

原 330 s 测试最后有效控制戳距 handoff 只有 4.495 sim 秒，后续是关机快照，不能称“6 秒停车通过”。新 run 没有到达停车阶段，仍未补成固定 5 秒停车证明。没有为了补证明放宽路径证书。

## 最终测试与可复现入口

从根目录执行（测试数字是该主机环境的快照，skip 不等于通过）：

```sh
source /opt/ros/jazzy/setup.bash
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -q
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression -q
bash -n scripts/run_dynanav_lio.sh
git diff --check
```

最终全部代码（包括合作式信号处理和监督终态校验）重新执行：unit **351 项，18.131 s，OK (skipped=3)**；regression **86 项，7.603 s，OK (skipped=5)**。3项unit skip需要USD运行时/本地人形资产，5项regression skip因宿主可选CPU Torch缺失。57个变更Python文件AST解析、shell语法和 `git diff --check` 通过；变更Markdown的596个相对链接在检查时均存在。未把fake故障注入、合成传输探针或历史运行加算进整套数字。发布前另去掉新提取 pid_support.py 文件末尾多余空行，AST前后相同；此纯空白收尾不作为真实PID运行源码哈希相同的证明。

## 后续工作与发布范围

1. 对大 RGB 消息在 publish-return 至 native receive 之间继续做可复算的传输诊断；必要时再提出经过验证的配置/接收策略变化，先不改 guard。
2. 定位 R85 路径证书为何在当前实测起点拒绝，保留物理证书与来源门；取得完整到达后再验 5 秒 live 停车。
3. Person 漏过、历史地图污染、图像动态 mask 和原有 repeat-spawn/oracle-reference 边界没有因本次生命周期修复获得质量验收。
4. 只发布本项目源码/测试/配置/文档，排除数据、模型、USD、原始日志、构建产物、`.env.local` 和 DSH 邮箱。既有第三方许可/来源声明保留，不新增 blanket 许可，不宣称既有许可缺口已清除。
