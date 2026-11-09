# Embodied Navigation

一个按系统职责组织的导航工程。开发源码、配置、文档都在本仓库内，不从 Documents 或其他本机自建工程加载代码。视觉策略训练独立位于 RL_for_unitree；这里只保留导航部署所需策略推理和机器人契约。

## 系统结构

各自有模块的职责、入口和边界见 [源码模块索引](src/embodied_navigation/README.md)，src 下 47 个模块 README（另有包索引，共 48 个 README 文件）均从该索引进入。易混术语（四条执行路径、provenance、raw/corrected 位姿、45D/247D、时钟域）见 [术语对照](docs/glossary.md)。

**本轮起每个模块 README 都包含**：当前目录结构、代码/日志/配置的含义、模块内文件耦合评估、可读性评估、按优先级排序的重构建议。跨模块汇总与全部重构建议见 [模块结构与耦合总评](docs/review/module-structure-coupling-20261109.md)。此前 589 个代码/配置目标的可读性报告见 [全量可读性总入口](docs/review/full-readability/README.md)（DSH 全量实现，Codex 独立事实复核，DSH 串行合入）。此前的 [本轮验证记录](docs/review/readability-verification.md)、[审计明细](docs/review/validation-and-threshold-audit.md) 与各模块 README 索引保留为历史记录。

- src/embodied_navigation/simulation：Gazebo/Isaac 场景与物理生命周期。simulation/dynanav 负责 DynaNav/Isaac 场景接入、真实 Go2 RGB-D capture，以及该实验路径的 oracle/learned 建图、实测位姿路线控制和动态行人模块；它不是整个导航系统，也不能替代生产导航整栈。
- perception：深度反投影、自身点过滤、传感流适配。
- localization_mapping：FAST-LIVO2 接入、相对里程计、地图配准和后端。实际 FAST-LIVO2 源码在 third_party/fast_livo2，不依赖另一个本机工程。
- planning：路线、SCAN 调度、PCT 以及路径交付。实际 SCAN/PCT 源码在 third_party。
- control：单步跟踪/控制律、碰撞与支撑检查、parking停车执行、safety统一仿真门禁和定位反馈来源检查。
- robot/unitree：Go2 观测、关节顺序、策略推理和执行桥。没有训练流程。
- navigation：完整任务状态和模块协作。默认主路径的任务状态机是 base_node.py 的 `Navigation.control()`（ros_node.py 包装、pid_navigation.py 装配为现役 controller，准入/路由/控制拆在同目录 `pid_*.py`，进程入口 `pid_entrypoint.py`）；task.py 是独立 `relative-slam-navigation` 入口的状态机，不在 demo.launch.py 中；mission/state_machine.py 在本检出没有实例化点，默认 mission 角色是 mission_runtime.py；Building 任务在 building_navigation.py；ROS 适配位于 ros_node.py、relative_slam_node.py。
- evaluation：轨迹长度/误差等离线指标及独立进程资源采样。仿真真值还用于现有策略本体观测和 COM 速度内环，不作为生产 SLAM 定位输入；显式 DynaNav 真值控制诊断另行记录来源。
- launch：模块装配、环境、进程生命周期。业务任务状态不放在 launch。
- configs、scripts、tests、docs：功能配置、少数可发现入口、回归测试和说明。配置读取顺序见 [配置入口](configs/README.md)，测试命令与边界见 [测试入口](tests/README.md)，实验与审查记录见 [文档索引](docs/README.md)。

## 目录结构

```text
embodied-navigation/
├── README.md                   # 本文档：结构、耦合、可读性、重构建议、验证状态
├── README_zh.md                # 中文简版入口
├── start.py                    # 11 行；唯一原生启动入口（检查/prepare/实际启动）
├── pyproject.toml              # 包元数据与 console script `relative-slam-navigation`
├── requirements-cpu.lock       # CPU 依赖锁
├── compose.yaml                # Docker Compose 服务定义；默认命令是 --check，不是启动系统
├── Dockerfile                  # Native 镜像：ROS Jazzy + Gazebo Harmonic + CPU Torch 2.7.1
├── src/                        # 自有源码（embodied_navigation 包）：259 个 py、60,546 行（文本口径，见"可读性评估"）
├── configs/                    # base 配置与契约数据；一次运行的有效值另写到 run 目录
├── scripts/                    # 少数可发现入口 + DynaNav 实验/回放/重建/评测脚本（16 py + 1 sh，共 17 文件）
├── tests/                      # 离线 CPU 测试：unit / regression / native（数量与 skip 见 tests/README）
├── docs/                       # 架构、术语、实验记录、依赖许可、审查与验证记录
├── docker/                     # 构建、源码 pin、私有 DDS、原生源码校验脚本
├── third_party/                # vendored 算法与驱动（fast_livo2、scan、pct、vikit、livox、fastdds）
├── install/                    # 本机构建产物；Git 忽略，缺文件≠缺源码
├── assets/                     # 外部数据卷软链：场景资产；Git 与 Docker 输入排除
├── models/                     # 外部数据卷软链：策略权重；同上
├── runs/                       # 外部数据卷软链：运行产物；同上
├── dynanav/                    # 迁移前历史副本（327M）；Git 与 Docker 均排除
├── history/                    # 历史副本（190M）；同上
└── perception_history/         # 历史副本（35M）；同上
```

782 个文件在 Git 跟踪内（2026-11-09 `git ls-files`）。`assets/`、`models/`、`runs/` 是软链，指向 `PROJECT_DATA_ROOT` 下的运行资产，内容不进 Git。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `src/embodied_navigation/` | 自有实现与 ROS 2 装配代码（259 py / 60,546 行），按 simulation → perception → localization_mapping → planning → navigation/control → robot 的数据流组织；每个子目录有自己的 README（结构 + 代码/日志/配置 + 耦合 + 可读性 + 重构建议）。 |
| 代码 | `third_party/` | vendored 开源算法与驱动，本仓库只做接入与构建，不是自有实现；版本、补丁与许可证见 `docs/dependencies/`。 |
| 代码 | `docker/`、`Dockerfile`、`compose.yaml` | 构建与启动装配；`docker/verify_native_sources.py` 校验原生源码 pin，`docker/build_*.py` 生成本地 `install/`。 |
| 日志/运行产物 | `runs/<UTC>_<label>_<hex>/` | 一次原生运行的**全部**输入与输出：`runtime_context.json`、`navigation_profile.json`、`processes.json`、`ros_logs/`、`*.log`、`component_exit.jsonl`、`exit.json`、`navigation_command.json`、`navigation_status.json`、`navigation_anchor.json`、生成的 `world.sdf`/`navigation_*.yaml` 等。`runs/` 是外部数据卷软链，Git 忽略。 |
| 日志/运行产物 | `$PROJECT_DATA_ROOT/embodied-navigation/experiments/<run>/` | DynaNav/Isaac 实验产物：`capture/`、LIO observer、person-worker、RGB/bag 与各 `*.json` 回执；由 `scripts/run_dynanav_*` 写入，代码强制位于数据根目录下。 |
| 日志/运行产物 | 进程级遥测与证据 | **默认不落盘**：`common/runtime_telemetry.py` 的 `RuntimeTelemetry.append/enqueue` 是空操作（只有 `.atomic()` 真写实时状态），所以各模块里 `evidence.append(...)` 的历史 `.jsonl` 默认不产生；例外是显式同步写盘（如 `control/parking/causal_join.finish()` 的 `r104_parking_causal_join.json`）与 `evaluation/process_resources.py` 的 `process_resources.csv`。判断某个产物是否真的存在，以 run 目录实际文件为准，不看代码里的 `append` 调用。 |
| 配置 | `configs/` | base 配置与契约数据，**不是**每次运行的有效值：`launch/navigation.json` 列出六个模块配置，`launch/prepare.py` 生成本次参数并 pin 若干键，运行期消费者分"读 `runtime_context`"与"直接重读 configs"两类。完整读取顺序、pin 与无读取点配置见 [配置入口](configs/README.md)。 |

"日志"在本仓库统一指**运行产物**（run 目录、实验目录、遥测/证据 JSON、ROS 日志），不是指仓库内某个日志文件夹；仓库不提交任何日志。各模块 README 只写自己产出的那部分，全局布局以 [配置入口](configs/README.md) 与 [数据与产物布局](docs/review/dsh-workflow-and-readability-20261109.md) 为准。

## 模块耦合

按一级模块统计（AST 解析 2026-11-09），包内依赖**不是严格 DAG**：11 个一级模块之间有 9 组 A↔B 双向依赖。大多数是 1–2 条窄依赖（共享小工具/数据结构），真正构成层次倒置、值得优先处理的是下面 5 条（均可 grep 复查）：

| 层次倒置 | 证据 | 为什么值得处理 |
| --- | --- | --- |
| 规划持有控制器子类 | `planning/reference/global_route_reference.py:11` import `control.tracking.cascade_core.Controller`；反向 `control/tracking/cascade_core.py:16` import `planning.reference.spatial_reference` | 规划与控制互相依赖，二者无法单独测试或替换 |
| 定位使用导航层数据类 | `localization_mapping/relative_odometry.py:8` import `..navigation.task.Observation` | 定位层反向依赖导航层；`Observation` 是数据契约，应在 common/sensors 层 |
| 传感适配层依赖控制内部 | `perception/sensors/feedback_mailbox.py` import `control.tracking.cascade_core`、`control.parking.parking_pose10hz` | 感知/传感层反向拉入控制实现，破坏"感知不依赖控制"的意图 |
| 策略 ACK 与停车控制双向依赖 | `robot/unitree/policy/lower_loop_ack.py` ↔ `control/parking/integral_unload.py` | 执行桥与控制律互相引用，任一侧改动都要双侧回归 |
| 安全门禁跨层直达 | `control/safety/duplicate_source.py` import `robot.unitree.bridge`、`planning.admission.physical_gate` | 安全模块同时依赖机器人和规划，边界难判定 |

此外还有 `navigation↔simulation`、`robot↔simulation`、`launch↔planning`、`launch↔localization_mapping`、`localization_mapping↔perception` 等窄环，多数是"共享小工具 + 装配 helper"性质，可以接受，但应在对应模块 README 里标注方向与原因，避免继续扩大。依赖密度最高的是 navigation：它 import control 39 处、common 35 处、planning 16 处；navigation 也是最大的模块（11,634 行 / 27 文件）。

## 可读性评估

> 行数口径：本表是 2026-11-09 修复前的结构快照，后续修复会改变文件/行数；本表与包索引用 facts 包文本口径（`read_text().count("\n")+1`），**每文件比 `wc -l` 多 1**，空文件记 1 行；模块 README 多用 `wc -l`，同文件两处数字差 1 属口径差异，不是笔误。

| 一级模块 | 文件 | 行数 | 函数数 | 函数 docstring 覆盖（AST 粗计） | >800 行文件 |
| --- | ---: | ---: | ---: | ---: | --- |
| navigation | 27 | 11,634 | 279 | 0.27 | base_node(1618)、pid_control(1174)、pid_support(1031)、pid_admission_apply(892)、ros_node(875) |
| control | 46 | 9,718 | 282 | 0.33 | cascade_core(1851)、transition_gate(1731) |
| planning | 32 | 9,361 | 263 | 0.34 | async_admission(1200)、static_geometry(1010)、validate_route_support(1003)、reference_protocol(891) |
| simulation | 43 | 9,095 | 215 | 0.40 | 无 |
| localization_mapping | 40 | 8,429 | 241 | 0.25 | known_scene_inputs(1139) |
| perception | 18 | 4,631 | 151 | 0.29 | person_geometry_tracking(823) |
| robot | 17 | 4,591 | 129 | 0.39 | bridge(1069)、policy/student(838) |
| common | 20 | 1,616 | 100 | 0.27 | 无 |
| launch | 9 | 1,314 | 23 | 0.57 | 无 |
| evaluation | 3 | 95 | 4 | 0.50 | 无 |
| sensors | 2 | 37 | 0 | — | 无 |

结论：**模块级文档可读性好，代码级可读性不均**。边界、禁入项、未验证声明在每个模块 README 里写得比代码本身清楚；代码侧的主要负债是 15 个 >800 行文件（集中在 navigation/control/planning/robot/localization_mapping/perception）和 0.25–0.57 的函数 docstring 覆盖。"文档说清楚、代码靠单文件承载"是当前的主要可读性风险，而不是命名混乱。docstring 覆盖是 AST 粗计（含嵌套函数），不作为质量评分，只用于横向比较。

## 重构建议

按“收益/风险”排序；本节是 2026-11-09 文档审计提出的建议，不表示本轮没有修复代码。进一步拆分前须先补齐控制、阶段切换和故障生命周期行为测试。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 拆 `control/tracking/cascade_core.py`(1851) 与 `control/parking/transition_gate.py`(1731) | 两个最大文件同时是 control↔planning、control↔robot 循环依赖的中心；拆开才能打断层次倒置 | 高：控制律数值等价必须逐样本对照 | 现有 1000 组只覆盖 relative PID 公式，来源契约测试也不能锁定这两个模块；先补 cascade 控制与阶段切换行为基线，再按职责提取纯函数 |
| P1 | 先修上表 5 条中的 3 条低风险项：`Observation` 下沉出 navigation；规划不再 import `Controller`（改为协议/回调注入）；`feedback_mailbox` 只依赖消息契约 | 改 import 与构造参数即可，行为不变，是三条低风险项 | 中：需同步改构造点 | Python 规范化 AST 对照 + 相关模块单测；import graph 复查环消失 |
| P1 | 拆 `navigation/base_node.py`(1618) 与 `pid_*` mixin 家族 | 现役 controller 由 6 个 mixin 组合，隐式契约是 `self` 状态；单类难读、难单测 | 高：默认主路径 | `test_ros_shutdown_lifecycle`、`test_pid_navigation_compat` 与运行链路对照 |
| P2 | 给 15 个 >800 行文件补模块级/函数级 docstring 与内部分段 | docstring 覆盖 0.25–0.57 与文件规模不匹配 | 低（纯注释） | 人工复核 diff；AST 行为不变 |
| P2 | 统一"运行产物"口径：root/configs/docs 三处只保留一份权威描述，其余引用 | 当前 run 目录与 experiments 产物的说明分散在多处，容易不同步 | 低 | 交叉引用检查（本轮已做） |
| P2 | 标注 4 个无 Python 读取点的配置（`control/relative_pid.yaml`、`navigation/relative_shuttle.yaml`、`robot/unitree_usd_manifest.json`、`runtime_assets_manifest.json`）为保留/归档 | 避免"改了没生效"被误判为 bug | 低 | `configs/README.md` 已列出，补删除/保留决策 |

各项的模块级细节（含 P0/P1/P2 分文件建议）见对应模块 README 的"重构建议"节，以及 [模块结构与耦合总评](docs/review/module-structure-coupling-20261109.md)。

## 两个运行边界

1. Native Gazebo 系统：start.py 装配已有导航模块。Dockerfile 从公开 ROS Jazzy、Gazebo Harmonic、CPU Torch 2.7.1 构建，必要自建源码均包含，FAST-LIVO2/SCAN/PCT 与补丁也在仓库中。
2. Isaac / DynaNav：当前入口是 scripts/run_dynanav_experiment.py（实验编排，在本进程内用 runpy 执行 simulation/dynanav/capture_go2_rgbd.py），配合 scripts/run_dynanav_lio.sh（独立观测器）与 scripts/run_dynanav_person_worker.py（独立行人推理）；observer 的 UDP 接收缓冲选择与回退见 [通信说明](docs/dynanav-observer-udp.md)；三者的在线行人组合顺序由 scripts/run_dynanav_online_person.py 监督（`--print-commands`/`--check` 可只打印或只核对；实跑时逐进程日志写到 run 的 `logs/`，失败/超时/SIGINT/SIGTERM 会先停 capture 再停 observer/worker）。navigation/relative_slam_node.py 是分离后的任务节点。它们仍需匹配的 Isaac 安装、场景与机器人资产。scripts/capture_dynanav.py 是旧的采集探针：`--check` 只检查文件，普通参数（`--scene/--output/--steps`）仍会启动采集。原机使用 Isaac Sim 6.0.1 RC；该版本的公开可获取性尚未核实，不能声称一份 Native Docker 镜像就能完整复现 Isaac。

最新 DynaNav 在线感知、速度阶梯与完整路线状态见 [在线行人实验记录](docs/experiments/dynanav-online-person-20261109.md)；此前的楼梯通行、分层地图、动态行人与 LIO/LIVO 实测见 [实验记录](docs/experiments/dynanav-20261109.md)。历史原话及尚未确定的研究参数见 [动态语义建图约定](docs/research/dynamic-semantic-mapping-agreement.md)，其他文档见 [文档索引](docs/README.md)。这些隔离实验不等于生产导航整栈验收。

## 快速检查

新克隆先复制 `cp .env.example .env.local`，编辑其中的 `PROJECT_DATA_ROOT`、`PUID` 和 `PGID`，再按下节准备有权使用的运行资产。`.env.local` 仅留本机，不提交 Git；下列 Compose 命令默认它已经准备好。CPU 单元检查不需要运行资产或 Docker。

- CPU单元测试：PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v
- 既有源码回归：PYTHONPATH=src python3 -B -m unittest discover -s tests/regression -v
- 干净公共镜像CPU验证：docker build -f docker/Dockerfile.cpu -t embodied-navigation-cpu .
- 原生源码验证：python3 docker/verify_native_sources.py --compile-pct
- Native完整镜像：docker compose --env-file .env.local build
- 依赖和资产检查：python3 start.py --check，或先设置 PROJECT_DATA_ROOT 再运行 docker compose --env-file .env.local run --rm navigation

Docker/Compose默认命令是检查。`python3 start.py --check` 解析配置、校验 ROS domain 的数值范围，并检查所需解释器、资产和构建产物是否为文件；它不验证 ROS 域占用、资产内容或系统可运行性。显式加 `--resource-check` 才检查本机 CPU 负载和可用内存；`--check --resource-check` 也会执行该资源检查。`install/` 缺失通常表示本机尚未构建，应按 [原生构建说明](docs/dependencies/BUILD.md) 或根 Dockerfile 生成，不把二进制加入 Git。直接运行 `python3 start.py` 会实际启动系统；可用有限正数 `--duration` 限制墙钟运行时间。

## 数据、权重和资产

Git不包含运行数据、原始日志、地图、权重、USD/纹理、图片、视频、build/install产物或本机 `.env.local`。部分历史证据引用和 Isaac 安装默认值仍含原机绝对路径；它们不是可移植下载链接，新机器须按配置提供自己的路径。设置 PROJECT_DATA_ROOT 指向 6.project_data；原生运行可先用 python3 scripts/configure_data.py --data-root /path/to/6.project_data --link 建立三个数据入口。Compose将 embodied-navigation/runtime_assets/{assets,models,runs} 显式挂载。场景和DynaNav实验数据位于该项目的数据目录，清单见 configs 中的资产manifest。assets/models/runs 是指向 `PROJECT_DATA_ROOT` 的软链（数据入口），内容不进 Git；代码本身不随这三个入口进入仓库。

这些资产不是从其他本机自建工程临时借用：必要的部署USD、已有Student权重和测量地图已另存导航数据目录并逐个校验。换机器仍需提供有权使用的相同数据，或按配置换成公开资产。仓库不伪造这些自有权重/地图的公开下载地址。

## 验证状态与限制

2026-11-09 最新生命周期修复与真实重验见 [运行故障修复记录](docs/review/runtime-repair-validation-20261109.md)。本轮真实 Isaac 在线行人仍因缺 RGB 后的原 8 帧缓存守卫中止；Gazebo PID 在 R85 路径证书门保持，未取得到达/5 秒终态停车证明。离线通过、进程退出与路线验收分开记录，未放宽安全门。

2026-11-09 较早阶段的独立复核、两项启动参数修复及当时 54 项单元/62 项回归结果见 [独立复核记录](docs/verification/independent-review-2026-11-09.md)。该阶段 5 项可选 Torch 测试跳过，未重跑仿真。后续 DynaNav 实际运行与最新测试另见上方实验记录。

2026-11-08 历史验证（针对当时的源码快照，不能代替后续修改的运行验证）：

- 当时源码48项单元测试通过，包括1000组原PID公式精确对比、导航任务、停车来源契约、生产SLAM入口及仿真安全门禁。官方ROS Jazzy纯CPU镜像的当时结果见docs/verification/cpu-container.json。
- 无数据的干净Git候选导出：62项回归成功结束；7项因未提供Torch或私有测量地图而明确跳过。双后端门禁集成和原native fixture另在装有CPU Torch的公共依赖容器实测通过；RoboGauge真实权重30帧输出与原JIT一致。
- FAST-LIVO2/SCAN等11个ROS包、PCT、4个自有原生模块、私有DDS补丁测试已在隔离公共依赖环境中从本仓库源码编译。完整Docker镜像已从公开ROS基础镜像构建成功，实际资源preflight通过；最终源码刷新和验证结果记录于docs/verification。
- 最终自足镜像45秒隔离软件栈通过：只挂本项目assets、models和新runs，不挂源码或其它工程的install；全部关键源码SHA匹配。1330次CPU策略推理、798帧相机、797帧LiDAR、57580个地图点；SCAN接受2条轨迹，22个健康非零命令样本及25个native执行确认样本。103条仿真安全门禁来源遥测全部有效，无物理fault；末段原有命令新鲜度门禁因来源超时拒绝并置零。15个组件全部退出0，日志无Traceback或未取回异常。详见docs/verification/software-startup.json。
- 此前发现的SIGINT关停竞态已修复：共用信号生命周期先退出spin和排空回调，最后关闭ROS context；真实callback异常仍传播，子组件非零退出会使主入口失败。48项单元包含6项生命周期测试、4项退出汇总测试及7项Building入口测试。
- 真实记录驾驶层回放1052/1052个样本通过：按目标点、航向误差和距航点距离独立唯一选帧，不用待检命令选帧，最大命令差异1.91e-14。它不是任务状态机或整栈证明。

可选Building Student入口的资源记录脚本与SCAN配置路径已实际补齐，仅允许SLAM模式；真实prepare-only通过，尚未运行Building分支，详见docs/dependencies/BUILDING-ENTRYPOINT.md。

仍未验收：生产导航整栈的完整路线、实机、策略训练和完整 Isaac 生产链路。docs/robogauge/isaac_closure_audit.json 记录较早的策略闭包审计；本轮 DynaNav 已补齐显式会话碰撞、RoboGauge 执行与采集诊断，并分别记录真值控制路线、首段 LIO 动态闭环和视觉旁路定位证据，不能据此声称生产链路已完整复现。

原测试3个停车样例缺少当前要求的pose_producer字段，在原工程也失败。本仓库更新fixture并纠正一处原源码要求真值标志为true的冲突：现在只接受显式非真值且生产者与时钟有效的SLAM反馈，没有放松阈值。运行源码和来源契约另见docs/architecture.md。

## 许可证和来源

已保留第三方源码、版本、许可证声明和补丁来源，详见 docs/dependencies。尚有Vikit授权来源、组合分发、自有源码/模型与部分资产来源问题需要确认，因此不能宣称公开发布许可已齐备。分发前请核对这些许可边界。

## 历史副本

dynanav/、history/、perception_history/ 是保留的原始/阶段副本，已从Git和Docker输入排除，不是本工程开发入口。这些本地目录不属于运行依赖，也不会进入Git或Docker输入。
