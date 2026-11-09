# DynaNav RGB-D capture 实现

本包实现 [capture_go2_rgbd.py](../capture_go2_rgbd.py) 的采集运行过程。入口保留实验脚本的 `runpy` 调用，使用显式 `run/main` 开始执行；普通 import 不创建运行目录或启动 Isaac、ROS 节点、模型。

本文行数与可读性表保留结构核查时的快照，不是随代码更新的精确统计；当前故障语义与验证入口见下节。

## 生命周期与故障收据

- **入口与初始化**：普通 import 无副作用；`run` 解析位置参数和 experiment，配置错误在可用的输出目录写 `stage=configuration`。缺失输出参数、目录不可写等情况下不能保证落盘，但始终重新抛出原错误。首个 `Receipts.save` 已纳入运行失败处理。
- **阶段与终态分开**：`stage` 保持原有阶段名，便于知道失败发生在哪里；新增 `outcome` 为 `running/completed/failed/cancelled`。采集与 Python 产物/ROS 收尾完成后、调用 native app close **之前**写 `completed`，不把它当成 native 清理完成证明。`KeyboardInterrupt/SystemExit` 写 `cancelled`，仍重新抛出；正常关闭产生的 `SystemExit(0)` 保留已写的 `completed`。
- **逐项清理**：每个 journal 都尝试关闭，即使前一个 flush/close 失败；再分别尝试 archive 诊断、执行器快照、mapping、错误附加报告、video、ROS、运行证据、收据及 app 关闭。已正常关闭的 video/ROS 不会在后续失败时重复销毁。
- **二次异常隔离**：错误 `repr`、traceback 格式化、`error.report` 属性访问或 JSON 序列化、运行证据采集、stdout 和收据写入都不能阻止后续资源清理。可写时在 `failure_cleanup_errors` 记录失败步骤；文件系统持续失败时只能保留原异常和内存诊断。
- **边界**：只能清理已返回并登记的资源。SDK 构造函数内部尚未返回的资源、进程被 SIGKILL/native crash 杀死、磁盘完全不可写，不能由 Python 异常处理保证收尾。SimulationApp 返回后入口接管 SIGINT/SIGTERM，只锁存首个信号，在编排安全边界转成 `CaptureCancellation(128 + signum)` 并走取消收据；Python 能返回时恢复启动 SDK 之前的信号处理器。不把 CPU fake 通过当成真实仿真验收。

### Native fast shutdown 与进程退出

安装的 Isaac SimulationApp 会覆盖 SIGINT，SDK 默认处理器可能关闭 native 应用并以 0 退出；fast shutdown 的 `app.close` 也可能直接 `os._exit`。因此不能依赖 `close` 后面的 Python 或 `finally` 写终态。

入口在 SDK 构造返回后立即接管 SIGINT/SIGTERM。信号处理器只锁存首个信号，不在 SDK/asyncio 回调栈内抛异常；主循环开始、SDK step 返回、frame 处理后、等待循环与初始化阶段边界主动检查。若同一进程组的 ffmpeg 已退出，后续操作先抛 BrokenPipe，则以已锁存的取消为终态，并将原操作错误及 traceback 保留到 `interrupted_operation_error`/`interrupted_operation_traceback`。没有锁存取消时继续保留原失败；清理期间收到重复信号也不重入抛异常。失败/取消路径先完成可执行的诊断与 Python 清理，写终态收据及已有的二次错误，然后请求 native close。成功路径也在请求 native close 前写采集完成终态。收据的 `app_close_requested=true` 表示已决定请求关闭，`app_close_returned=false` 表示尚无 Python 返回证据；仅在调用返回后才尝试更新为 true。两者都不等于外部已确认所有子进程退出，最终进程清理仍需 supervisor/外部检查。

针对真实信号路径的 subprocess 测试模拟 SDK 覆盖 SIGINT，再向自身发送 SIGINT/SIGTERM，并令 fake native close 调用 `os._exit`，检查硬退出前已写出取消收据；正常 fast shutdown、`SystemExit(0)`、返回路径恢复信号处理器也有独立用例。

### 发布边界诊断（可选）

在线排查时设置 `DYNANAV_PUBLISH_TRACE=1`，入口将 `Receipts.journal` 回调交给 ROS 接口，写 `sensor_publish_trace.jsonl`。默认关闭，不新增发布或改变消息 stamp、QoS、节拍及安全检查。

- 范围只含 RGB `/dynanav/camera` 与 experiment 的 `lidar_frontend_topic`；未显式配置时 learned 模式选 `/dynanav/lidar_perception_filtered`，否则选 `/dynanav/lidar_static`。其他点云、IMU、depth、clock 不写该诊断。
- 每帧每选定 topic 最多两条标量行（当前传感循环 10 Hz，即最多 40 行/秒），不复制图像或点云 payload、不在内存积累完整轨迹。文件随运行时长增长。
- `publish_call` 在调用发布器前写出；`publish_returned` 只在 `publisher.publish` 正常返回后写出。两行均含 `source_ns`、`monotonic_ns`、`topic`、`data_bytes`（data 字段字节数，不是 DDS 序列化总字节数）、`point_count`（RGB 为 null）与 `delivery_confirmed=false`。返回行还含 `before_publish_monotonic_ns`。
- 每行由 receipts flush 到操作系统，未承诺 fsync 持久化。前后间隔包含调用前诊断写入开销，不能直接当成纯 DDS publish 耗时。日志失败向上传播到运行失败收据，不静默忽略。
- 发布返回不代表订阅端已收到。需与 native `FASTLIVO_INGRESS_TRACE=1` 的接收侧时间和相同 source stamp 对照，才能定位发布端、传输/队列或回调边界。

### 可复跑的验证

```bash
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -p test_dynanav_capture_orchestration.py
```

该套测试通过替身执行真实编排入口，不启动 Isaac/ROS/GPU/ffmpeg。除零步、一步、首 tick 失败与常规清理失败外，还覆盖首个收据写失败、坏配置、早期取消、journal 关闭失败后继续、错误报告属性抛错、不可序列化报告、异常 repr 抛错以及运行证据采集失败。真实传感器时序和资源进程是否退出必须另用在线仿真验证。

## 目录结构

```text
src/embodied_navigation/simulation/dynanav/capture/
├── README.md           # 本文件
├── __init__.py         # 14 行，只重导出 CaptureContext/Receipts/record_failure
├── control.py          # 472 行，50 Hz 控制 tick、路线/命令新鲜度、动态 hold、停车与终止
├── frames.py           # 251 行，10 Hz RGB-D 帧处理、点云过滤/发布、视频面板与帧证据
├── mapping.py          # 277 行，oracle/learned 建图、独立评分与 raw RGB 归档接线
├── physics.py          # 215 行，0.005 s 物理步 + SDK 时钟/IMU 读回 + 关节/速度/姿态断言
├── policy.py           # 199 行，Student/RoboGauge 策略加载、动作与力矩计算
├── receipts.py         # 177 行，result.json、JSONL journal、故障上下文与尽力清理
├── ros_interface.py    # 270 行，ROS 节点、话题编码、订阅回调与实测里程计缓存
├── scene.py            # 507 行，SimulationApp/USD 场景、Go2 与行人、接触回调、物理准备与预检
├── sensors.py          # 185 行，相机安装位姿、IMU、RGB-D annotator、overview 与标定记录
└── video.py            # 94 行，overview/RGB 视频的 ffmpeg 编码器生命周期
```

11 个文件、合计 2661 行（最大 `scene.py` 507 行）；无子目录。本包不包含入口，入口是同级的 `../capture_go2_rgbd.py`（485 行，只做 argv/experiment 配置、编排与失败收据）。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `__init__.py` | 活代码但很小：只 `from .receipts import CaptureContext, Receipts, record_failure` 并写 `__all__`；导入本包会连带导入 `receipts`（标准库 + json），不会把 Isaac 带进来。 |
| 代码 | `scene.py`/`sensors.py`/`physics.py` | Isaac 会话层：`start_simulation_app`、`open_stage`、`spawn_articulation_asset`、`setup_physics`、`initialize_articulation`、`run_preflights`、`play`/`stop`；相机/IMU/annotator 安装与 `sensor_installation_receipt`；`PhysicsStepper.step` 做一步 0.005 s 并校验时钟、力矩、关节读回。 |
| 代码 | `ros_interface.py`/`frames.py` | ROS 与发布层：节点创建、`/dynanav/*` 消息编码/订阅回调、实测里程计缓存；`FrameStream.process` 处理一帧 RGB-D、发布点云并写帧证据。 |
| 代码 | `control.py`/`policy.py`/`mapping.py`/`video.py` | 决策与产物层：控制 tick 与终止/停车；策略加载与力矩；oracle/learned/评分/归档接线；ffmpeg 编码器。 |
| 代码 | `receipts.py` | 证据层：`Receipts`（result.json + journal）、`CaptureContext`（本次资源与故障现场）、`record_failure`（逐项尽力清理）。 |
| 日志/输出 | `<output>/result.json` | 每次运行的主收据。`Receipts.save()` 节流 0.25 s（避免 200 Hz 物理率打数据卷），先写 `result.json.tmp` 再 `replace`，崩溃不会留截断文件。 |
| 日志/输出 | `<output>/<name>.jsonl` | 追加式 journal，名字与写入方：`camera_frames.jsonl`/`camera_render_pose.jsonl`/`pedestrian_native.jsonl`（frames）、`slam_route_control.jsonl`/`policy_frames.jsonl`/`dynamic_hold_control.jsonl`（control）、`dynamic_hold_frames.jsonl`（mapping）、`imu_stream.jsonl`（入口 `run_capture`）、`truth_scoring_stream.jsonl`（入口的 truth 评分支路）。 |
| 日志/输出 | `<output>/mapping/…`、`<output>/learned_scoring/…`、`<output>/rgb_raw/…` | 建图 sink 子目录：oracle/learned 写 `frames.jsonl`、`static_map.npz`、`receipt.json`（二选一，由 `semantic_provider` 决定）；`oracle_evaluation` 打开时另写 `learned_scoring/{frontend,missing_labels,persistent_admissions}.jsonl` + `receipt.json`；`archive_rgb` 时 `rgb_raw/` 存叠字前原始 RGB。 |
| 日志/输出 | `<output>/multiwaypoint05_overview_and_rgb.mp4` | `video.py` 的 ffmpeg 产物，文件名固定；编码器在失败路径由 receipts 尽力关闭。 |
| 输入（非日志） | `DYNANAV_EXPERIMENT`、`argv[1]`/`argv[2]` | 配置来源：`capture_go2_rgbd.parse_config` 读 `argv[1]=output`、`argv[2]=scene`，`DYNANAV_EXPERIMENT` 指向 experiment JSON（由 `../experiment.py` 写出）。 |
| 配置 | 无 `configs/` 直接读取点 | `grep -rn "configs/" capture/` 为空；本包只消费 experiment 字典与 argv/env。`output` 必须位于 `$PROJECT_DATA_ROOT/embodied-navigation` 下，`mapping.open_rgb_archive` 对归档路径再校验一次。 |

## 模块内文件耦合

- 依赖方向：入口 `../capture_go2_rgbd.py → 本包 10 个模块`（全部显式构造）；包内是"编排层 → 领域层"：`mapping → top-level 的 oracle_mapping/learned_mapping/learned_scoring/rgb_archive`；`control → dynamic_hold`、`policy → 策略/契约模块`、`scene → 行人/session_collision/collision_bounds`、`frames → render_diagnostics`、`ros_interface → semantic_mapping`、`physics → policy.native.observation`。
- **向上依赖**（叶子依赖父级）：`control.py:60` 与 `policy.py:170` 在函数内部 `from embodied_navigation.simulation.dynanav.experiment import make_slam_controller / joint_speed_observation`。因为是函数内 import，普通 `import` 不会成环；但分层上是"包内模块反向依赖同级入口模块"。
- 无共享可变状态：`Receipts` 的节流计时与 journal、`ContactMonitor` 的计数与事件列表都按运行实例持有；`CaptureContext` 显式保存资源，替代跨作用域的 `locals()` 探测。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `scene.py` | 507 | `project`、`collision_bounds`、`pedestrian`、`session_collision`、Isaac/Omni | 入口 | 偏重：507 行、21 个函数、13 个有 docstring；`on_contact` 85 行无 docstring |
| `control.py` | 472 | `route_tracker`、`policy.native.observation`、`dynamic_hold`、`experiment`（函数内） | 入口 | 偏重：`ControlLoop.tick` 282 行 |
| `mapping.py` | 277 | `project`、`semantic_mapping`、4 个 top-level dynanav 模块（函数内） | 入口 | 清楚：7 个函数 6 个有 docstring，模式分支集中 |
| `ros_interface.py` | 270 | `semantic_mapping`、rclpy | 入口 | 偏重：11 个函数只有 1 个 docstring，`create_ros_interface` 235 行 |
| `frames.py` | 251 | `perception.depth`、`render_diagnostics` | 入口 | 偏重：`FrameStream.process` 194 行 |
| `physics.py` | 215 | `policy.native.observation` | 入口 | 可接受：`step` 153 行但有 docstring |
| `policy.py` | 199 | `project`、策略/契约模块、`experiment`（函数内） | 入口 | 清楚：4 个函数 3 个有 docstring |
| `sensors.py` | 185 | 无内部依赖 | 入口 | 清楚：5 个函数 docstring 覆盖率 1.0 |
| `receipts.py` | 177 | 无内部依赖 | 入口、`__init__` | 可接受：9 个函数 2 个 docstring，但核心 `save/journal` 语义清楚 |
| `video.py` | 94 | 无内部依赖 | 入口 | 清楚：ffmpeg 生命周期集中在一个类 |
| `__init__.py` | 14 | `receipts` | 无 | 清楚 |

四类问题：**上帝文件无**（507 < 800）；**重复实现无**（各文件职责不交叉，唯一相近的是 oracle/learned 两条建图分支，但它们本就应该分开）；**跨层直达有**：`control`/`policy` 直接 import 父级 `experiment` 的私有助手，`mapping` 直接 import 4 个 top-level 模块；**隐式契约有**：journal 文件名是字符串、`experiment` 字典的键（`semantic_provider`/`oracle_evaluation`/`archive_rgb`/`imu_publish_stride`/`imu_orientation_available`/`dynamic_hold`/`speed_mps`）散落在各模块，`Receipts.journal(name, payload)` 的 payload 结构无类型。

## 可读性评估

- `sensors.py`、`mapping.py`、`policy.py`：**良好**。docstring 覆盖率 1.0/0.857/0.75，函数短（最长 63 行），每个函数只做一件事；`mapping.py` 的模块 docstring 说清两类 sink 与"导入时不拉 Isaac"。
- `scene.py`、`frames.py`、`physics.py`、`receipts.py`、`video.py`：**可接受**。`scene.py` 21 个函数里 13 个有 docstring、最长 85 行（`on_contact` 无 docstring 是为性能回调）；`receipts.py` 的 `save` 把"0.25 s 节流 + 原子替换"写成注释，`record_failure` 有"保留原 error/traceback，再逐项尝试清理"的 docstring。
- `control.py`、`ros_interface.py`：**需改进**。证据：`ControlLoop.tick` 282 行（docstring 有，但把状态组装、停车、路线、动态 hold、动作调用都压在一个方法里）；`ros_interface.py` 11 个函数只有 1 个 docstring（0.091），`create_ros_interface` 235 行把 10+ 个 topic 的 QoS/编码一次性建完；两者都是"一帧/一次初始化里步骤太多"，不是命名含糊。
- `__init__.py`：14 行、docstring 说清"导入无副作用、按 config→session→ROS→mapping→loop→receipts 顺序构造"，**良好**。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `ControlLoop.tick`（282 行）按阶段拆成 `_assemble_state`/`_resolve_command`/`_apply_dynamic_hold`/`_tick_action` 四个方法，`create_ros_interface`（235 行）按"node/订阅/发布"拆三段 | 这两个函数是采集探针里最容易改错的点（命令新鲜度、QoS、时钟），现在只能整段读 | 中：控制顺序与 journal 内容都是证据，拆错会改变行为 | 先跑 `tests/unit/test_dynanav_capture_orchestration.py`（覆盖零/一步/首 tick 失败/清理二次失败等编排）与 `test_dynanav_odometry_source.py`；拆分只允许移动代码，不允许改变调用顺序，逐项比对 journal 名与 payload 键 |
| P2 | 把 `experiment.make_slam_controller`/`joint_speed_observation` 移到更低的共享模块（或在本包内复制薄封装），消除"叶子 import 父级入口" | 现在 `control`/`policy` 反向依赖 `../experiment.py`，单独复用本包会连带拉起入口模块的依赖 | 低：只换 import 位置，不改实现 | 用 `python -c "import embodied_navigation.simulation.dynanav.capture.control"` 检查不拉起 Isaac；`tests/unit/test_dynanav_*` 全通过 |
| P2 | 给 `ros_interface.py` 的 topic/QoS/编码函数补 docstring，并把 topic 名字面量与消息类型集中成常量表 | 0.091 的 docstring 覆盖率 + 10 个订阅/发布点散落，核对话题契约只能靠 grep | 无（文档/常量） | 人工复核；对比 `ros_interface.py` 里所有 topic 字面量与 `docs/glossary.md` 的 `/dynanav/*` 列表 |
| P2 | 把 `Receipts.journal(name, payload)` 的 journal 名收敛成枚举/常量，并在 `receipts.py` 列出"谁写哪个文件" | 实测 16 个不同 journal 名散落在 7 个文件（本包 3 个 + 入口 + oracle/learned/scoring 三个 sibling 模块），离线脚本只能靠字符串匹配 | 低：常量等价替换 | grep 确认无遗漏；跑一次 CPU fake 编排测试比对文件名集合 |

结构核查时未列 P0；后续故障注入补齐了本节上方记录的初始化与二次异常边界。本包的失败路径是显式的（先保留原异常，再尽力写收据与清理，二次错误记录在 `failure_cleanup_errors`），且 `tests/README.md` 记录的 CPU fixture 覆盖了"零/一步/首 tick 失败"等编排边界。P1 是"超长函数可审阅性"，且已被测试覆盖，可以安全推进。本轮只写建议、不改代码；任何拆分都要先跑编排测试，因为测试直接断言 receipt/journal 内容。

## 模块分工

| 文件 | 负责什么 |
| --- | --- |
| `scene.py` | SimulationApp、USD 场景、Go2 与行人、接触回调、物理准备和场景预检 |
| `sensors.py` | 相机安装位姿、IMU、RGB-D annotator、overview 与安装标定记录 |
| `physics.py` | 0.005 s 物理步、SDK 时钟与 IMU 读回、关节/速度/姿态检查 |
| `ros_interface.py` | ROS 节点、话题消息编码、订阅回调和实测里程计缓存 |
| `control.py` | 路线控制、命令新鲜度、动态 hold、停车与终止条件 |
| `frames.py` | RGB-D 帧处理、点云过滤与发布、视图面板和帧证据 |
| `mapping.py` | oracle/learned 建图、独立评分与 raw RGB 归档接线 |
| `policy.py` | Student/RoboGauge 策略加载、动作、力矩计算 |
| `video.py` | overview/RGB 视频的 ffmpeg 编码器生命周期 |
| `receipts.py` | 每次运行的收据、JSONL journal、显式故障上下文与尽力完成的清理 |

## 调用与状态

`capture_go2_rgbd.parse_config` 读取旧位置参数 `[程序名, output, scene]` 与 `DYNANAV_EXPERIMENT`；`run_capture` 按顺序建立场景、传感器、建图、ROS、策略和主循环。Isaac/Omni 依赖保持在应用启动后的实际函数里加载。

`Receipts` 的节流计时与 journal、`ContactMonitor` 的计数和事件列表均按运行实例持有。`CaptureContext` 显式保存本次创建的资源与故障现场，代替跨作用域的 `locals()` 探测。失败时先保留原 error/traceback，再分别尝试收据与资源清理；二次错误记录在 `failure_cleanup_errors`。

传感节拍、坐标系、位姿来源与算法模式仍来自原实验配置；拆分没有增加楼梯限速，也没有把 oracle 评分输入接入 learned 控制/建图。结构核查阶段没有运行真实仿真；本次生命周期修复已复跑上述 CPU fake 编排测试。真实仿真的最新运行结果应查看对应实验报告，不由此处的替身测试推断。

原有入口、实验条件与结果见 [DynaNav 模块说明](../README.md)；本轮修改与验证见 [独立核查](../../../../../docs/review/dsh-structural-independent-review-20261109.md)。CPU fake 验证通过不等于真实 SDK 或在线路线验收。
