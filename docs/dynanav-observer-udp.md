# DynaNav observer UDP 接收缓冲

## 选择与回退

`run_dynanav_lio.sh --observer-transport auto` 为默认策略；也可用环境变量
`DYNANAV_OBSERVER_TRANSPORT=auto` 选择，CLI 优先于环境变量。只有 LIVO 模式、
显式 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 且
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 时，才考虑选用
`configs/simulation/dynanav/observer_udp_rx4m.xml`。online-person supervisor
已提供这两个默认值；直接启动时若未明确中间件或传输方式，则保留原行为，不猜测安装环境。

已存在的 `FASTRTPS_DEFAULT_PROFILES_FILE`、`FASTDDS_DEFAULT_PROFILES_FILE`、
`FASTDDS_ENVIRONMENT_FILE`、`SKIP_DEFAULT_XML`（包括空值），以及启动工作目录内的
Fast DDS 默认 XML 均优先于自动选择。启动器不合并、替换、验证或修复用户自己的 DDS
配置；非 FastRTPS、非 UDP 和 LIO 模式保持不变。无效策略名会在启动节点前报错。

无需撤销代码即可回退：observer 使用 `--observer-transport baseline`，或启动 online
supervisor 时设置 `DYNANAV_OBSERVER_TRANSPORT=baseline`。baseline 只表示“不新增
profile”，不会清除调用方的自定义配置。重现原始 UDP 基线时，仍需使用原始启动环境，
不要遗留实验用 profile 环境变量。

profile 只在 observer shell 内、`--check` 返回之后导出，作用于它启动的 mapper、
blackboard 和就绪检查客户端，不传回父 supervisor 或 Isaac publisher。本改动不改变
QoS、发布频率、图像处理、8 帧守卫、估计器设置或阈值。

## 参数与生效边界

`receiveBufferSize=4194304` 请求 4 MiB（4 × 1024 × 1024 字节），来自已验证的
observer 单侧实验。XML 与该实验候选完全一致：禁用内建传输，使用自定义 UDPv4
描述符；其他描述符字段使用库默认值，不增加发送缓冲、接口白名单、消息尺寸或 QoS 设置。

`preflight.json` 与 `launch.json` 都记录策略来源（默认值、环境变量或 CLI）、选择原因、
相关继承环境、选定 XML 路径和 SHA-256、请求字节数，以及只读取得的
`/proc/sys/net/core/rmem_max`。`--check` 只打印这些选择信息，不创建运行目录或 ROS
participant；这些收据不是内核已应用配置的证据，`socket_buffer_verified` 保持 false。

运行时必须将 UDP socket 归属到实际 native mapper 的 PID/inode，保存 `ss -u -m -p`
等 socket 缓冲和丢包证据，再与下述原始 trace 对账。Linux 接收缓冲采用加倍计账：
已验证主机的上限是 4194304，实际 native 数据 socket 的 `rb` 是 8388608。主机上限、
中间件版本或 XML 加载结果变化后，必须重新核验。较小上限只记录，不自动修复；不写
sysctl，也不调用 `SO_RCVBUFFORCE`。

## 原始追踪、双时钟与逐行校验

通信调查可显式启用已有的两个开关，本补丁不默认开启它们，不增加高频 stdout，也不增加
额外数据订阅：

- capture 进程设置 `DYNANAV_PUBLISH_TRACE=1`，由现有 journal 写入 capture 输出目录的
  `sensor_publish_trace.jsonl`。范围为 RGB 与选定的前端点云，每条保留 topic、
  `source_ns`、`monotonic_ns`、数据字节数，以及 `publish_call` / `publish_returned`。
  返回行另有 `before_publish_monotonic_ns`。`source_ns` 是原始消息的仿真时间戳，
  `monotonic_ns` 是主机单调时钟；不得用其中一个替代另一个。
- observer 进程设置 `FASTLIVO_INGRESS_TRACE=1`。启动器已有的
  `DEMO_RUN_DIR=<observer output>` 对应的 `fastlivo_debug/` 保存
  `pipeline_v19_events.csv`、`pipeline_v19_lifecycle.csv` 和停止时生成的
  `pipeline_v19_summary.json`；另一个 `FASTLIVO_DIAGNOSTIC_DIR=<observer output>/diagnostics`
  保存 `records.bin` / `index.csv`，不要混淆两个目录。事件行包含 native 本地 `sequence`、`kind`、
  `source_ns`、`receipt_wall_ns`、各处理阶段单调时钟与 `commit_end_wall_ns`。
  `wall_ns` 在这里来自 `CLOCK_MONOTONIC`，不是日历时间；只在确认同一主机时钟域后，
  才可与 publisher 的单调时钟直接相减。native sequence 是接收管线内部序号，
  不是 publisher 序号；跨两侧按传感器类型/topic 与原始 `source_ns` 对齐。

`publish_returned` 仅证明 publish 调用正常返回，所有发布 trace 的
`delivery_confirmed` 仍为 false；publish 不等于 delivery。native 事件 CSV 在 owner
回调正常完成并 acknowledge 后才写一行，所以它记录的是 committed，不是全部接收。
committed 也不等于图像已产生保留的视觉更新，后者需要独立的 native 状态证据。
停止摘要里的 `uncommitted_packets` 保存已进入管线但未 commit 的 `seq`、`kind`、
`source_ns`、阶段及失败原因；`rejected_packet` 是单独的拒收记录，不能并入 committed。
异常退出时，应先联合 committed 与 uncommitted/rejected 记录检查，不能把 CSV 缺行
直接当作网络丢包。

逐轮保留原始文件、启动/退出收据、runner 日志、socket 证据和配置 hash，再离线逐行校验：

1. 对 JSONL 的每个非空行严格解析，核对所需字段、整数时间戳、事件类型和
   `delivery_confirmed=false`；按 topic/source stamp 对齐 call/returned，报告重复、
   缺配对和截断尾行，不静默跳过坏行。异常退出留下的未返回 call 必须单列。
2. 对 native CSV 校验表头、每行 23 列、字段类型、sequence 唯一性与顺序；对停止摘要
   做严格 JSON 解析。完整停止时对照 CSV 行数与 `committed`，再核对
   `accepted` / `delivered` / `committed`、未提交清单和正常完成标记。不要只看总行数，
   不要为了计数一致而补行、截断或重编号。
3. 分别统计发布返回、native committed、uncommitted 和 rejected，再按类型/source stamp
   找差集；保留原始 native sequence 和两套时钟。将“无 native 记录”与“已接收未提交”
   分开报告，并结合 socket drops 判断传输证据，不把单一差集等同于因果证明。
4. 记录非零退出、终止信号、日志截断、缺失摘要或校验失败；这些都意味着证据不完整。
   publisher journal 每行 flush 到操作系统但不保证 fsync；native CSV 存在缓冲，正常
   停止会 flush。硬退出或主机故障可能丢失尾部记录，不能据此宣称完整接收或完整丢包统计。

## 2026-11-09 验证记录与未解决项

本机 Fast DDS 2.14.6、ROS Jazzy、同一 post-VIO 二进制/路线/传感器配置下，
原 UDP 基线的无行人三轮均到达，但 RGB 发布返回与 native 记录分别差 112、117、150 帧；
有行人三轮均未到达，分别是转弯漂移、8 帧云队列保护、第二转弯超时。
原始两侧 trace 与正常停止摘要逐行校验后，才统计接收差集。socket datagram drops
不是图像帧数，也不足以单独证明导航失败的全部原因。

保持 QoS、算法、8 帧守卫和路线门限不变，仅 observer 请求 4 MiB 的三轮候选实验，
RGB 分别为 1518/1518、1479/1479、571/571 进入 native 并 commit，实际接收 socket
`rb=8388608`，观察到的 native UDP drops 增量均为 0。前两轮到达，第三轮在第一目标前
低进展停车：8 秒内实际 base 净位移约 0.36 mm，策略输入约 0.04 m/s。
这是小样本记录，不是故障概率或整栈稳定性结论。

随后通过官方 `run_dynanav_online_person.py` 直接启动、未使用候选 wrapper 的
`comment-formal-auto-person-20261109T164141Z`，preflight/launch 自动选中同一 SHA-256
的 XML；实际 native RTPS 接收 socket 核验为 `rb=8388608`。1070/1070 RGB 和
1070/1070 前端云均接收并 commit，mapper UDP drops 增量为 0；97957 条事件与停止摘要
accepted/delivered/committed 一致，正常 drain，无坏行或未提交 packet。
但路线在第二段 101.915 秒再次低进展停车，未到达：对应 8 秒内真实 base 净位移约
0.43 mm，策略输入约 0.052–0.054 m/s。因此本改动针对已观察到的接收缓冲丢失，
不声称解决低速策略响应、接触/支撑、动态物体过滤质量或导航整体可靠性。

离线检查：358 个单元测试通过（跳过 3），87 个回归测试通过（跳过 5），
另通过 shell 语法、Python 编译、XML 解析及 diff 空白检查。未运行的项不能以跳过代替通过。
