# Embodied Navigation · 具身导航

具身导航连接传感器、定位建图、路线规划和 Go2 执行。DynaNav / Isaac Sim 是开发主线，Gazebo 保留历史入口。机器狗策略训练在独立工程 `RL_for_unitree`，本仓库维护部署推理和机器人接口。

## 从哪里开始

- **DynaNav / Isaac Sim** 从 [探索启动器](scripts/run_dynanav_native_exploration.sh) 进入。前向 RGB、RTX LiDAR 和 IMU 输入 FAST-LIVO2，生成 post-VIO 里程计和实测点云。人或 Agent 通过 [航点客户端](scripts/explore_waypoint_client.py) 发送目标。默认链路是实测 PCT 全局规划 → 原生 SCAN 局部规划 → 连续路线跟踪 → RoboGauge。`native_route` 保留用于历史跟随器对照。参见 [PCT/SCAN 时序](docs/dynanav-pct-scan.md) 和 [探索 SOP](.agents/skills/dynanav-explore/SKILL.md)。
- **Gazebo / ROS 2** 从 [start.py](start.py) 进入，走 `launch`、`navigation`、`control` 和 Go2 执行桥。参见 [配置入口](configs/README.md) 和 [原生构建说明](docs/dependencies/BUILD.md)。
- **复查实验** 从 [experiments](experiments/) 查定义和结论，到报告对应的数据卷 run 核对冻结配置、真值、回执与评分。传感驱动探索和真值辅助录包按各自协议验证。

## 顶层目录

```text
embodied-navigation/
├── README.md · README_zh.md · AGENTS.md · CLAUDE.md  # 总览和协作规则
├── start.py                                         # Gazebo / ROS 2 入口
├── Dockerfile · compose.yaml · docker/               # 构建与运行装配
├── pyproject.toml · requirements-*.lock/.txt          # 包和依赖
├── .agents/skills/                                  # 协作 SOP
├── configs/                                         # 默认参数和资产入口
├── scripts/                                         # 启动、观测、定位与评测入口
├── src/embodied_navigation/                         # 自有代码
├── third_party/                                     # FAST-LIVO2、SCAN、PCT、Vikit、Livox、Fast DDS
├── experiments/                                     # 实验定义和结论
├── tests/                                           # unit、regression、native
└── docs/                                            # 架构、术语、验证和历史审查
```

Linux 检出还包含数据卷 `runtime_assets/{assets,models,runs}` 的三个链接 `assets/`、`models/`、`runs/`。`install/` 是本地构建输出。`dynanav/`、`history/`、`perception_history/` 和 `.dsh-subagents/` 存放本地历史或协作材料。新实验从下文的项目数据根分配目录。

| 模块 | 职责 |
| --- | --- |
| [simulation](src/embodied_navigation/simulation/README.md) / [perception](src/embodied_navigation/perception/README.md) | Gazebo / Isaac 生命周期及 DynaNav 传感采集、仿真侧接口、点云和图像适配 |
| [localization_mapping](src/embodied_navigation/localization_mapping/README.md) | FAST-LIVO2、里程计、地图、关键帧、回环、位姿图和先验重定位 |
| [planning](src/embodied_navigation/planning/README.md) | PCT 路线、SCAN 局部轨迹及向控制侧的交付 |
| [navigation](src/embodied_navigation/navigation/README.md) / [control](src/embodied_navigation/control/README.md) | 航点、任务状态、跟踪、转向与停车。DynaNav 和 Gazebo 各有装配入口 |
| [robot](src/embodied_navigation/robot/README.md) | Go2 观测、关节契约、策略推理和执行 |
| [launch](src/embodied_navigation/launch/README.md) / [common](src/embodied_navigation/common/README.md) / `evaluation/` | 进程装配与生命周期、共享类型和工具、资源与轨迹评价 |

更多入口见 [源码模块索引](src/embodied_navigation/README.md)、[术语对照](docs/glossary.md) 和 [测试入口](tests/README.md)。

## 分支与实验基线

2026-11-20 在 Linux 核对本地检出和 GitHub 分支。提交号描述对应的分支头。当前实验工作树还有未提交改动，复现时以各次运行记录的源码为准。

| 分支或基线 | 用途 |
| --- | --- |
| `main`、`origin/main`（`51209ba`） | 集成与独立验收入口 |
| 本地 `exp/two-floor-demo`（`e378b8d`） | 当前两层 DynaNav、PCT/SCAN、转向和控制联调。GitHub 未见同名分支 |
| `refactor/00-plan-docs` 至 `refactor/05-agents-rules` | 分包推进重构与协作规则。`05-agents-rules` 尚未进入 `main` |
| `refactor/05-loop-evidence-base`（`d582b28`） | 回环共同比较口径与证据 |
| `exp/loop-measured-submaps`、`exp/loop-surface-gate`、`exp/loop-visual-retrieval` | 多帧实测子图、表面门控、视觉候选检索，详见 [分支协作规则](AGENTS.md#相关分支为何保留) |
| `improve/module-docs-readability`、本地 `feat/navigation-review-web`（`15d97fb`） | 模块文档与阅读入口 |
| `archive/pre-review-20261109`、`wip/navigation-local-snapshot-20261115`、`backup/slam-controlled-experiment-20261116` | 迁移前档案、在途源码快照、受控 SLAM 追踪和评价实现 |
| 官方 `hku-mars/HBA` | 外部离线全局优化对照，产物在数据卷 `runs/20261118-hba-offline/` |

`runs/20261117-loop-comparison/`、`runs/20261119-hba-project-comparison/` 和 `runs/20261119-rtx-pitch-slam/` 是数据卷的评测目录。各次实验的输入和算法以运行配置及来源记录为准。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `src/embodied_navigation/` | 自有实现与 ROS 2 装配代码（259 py / 60,546 行），按 simulation → perception → localization_mapping → planning → navigation/control → robot 的数据流组织；每个子目录有自己的 README（结构 + 代码/日志/配置 + 耦合 + 可读性 + 重构建议）。 |
| 代码 | `third_party/` | vendored 开源算法与驱动，本仓库只做接入与构建，不是自有实现；版本、补丁与许可证见 `docs/dependencies/`。 |
| 代码 | `docker/`、`Dockerfile`、`compose.yaml` | 构建与启动装配；`docker/verify_native_sources.py` 校验原生源码 pin，`docker/build_*.py` 生成本地 `install/`。 |
| 日志/运行产物 | `$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>/` | 一次原生运行的**全部**输入与输出：`runtime_context.json`、`navigation_profile.json`、`processes.json`、`ros_logs/`、`*.log`、`component_exit.jsonl`、`exit.json`、`navigation_command.json`、`navigation_status.json`、`navigation_anchor.json`、生成的 `world.sdf`/`navigation_*.yaml` 等。仓库的本地 `runs/` 链接指向 `runtime_assets/runs/`。 |
| 日志/运行产物 | `$PROJECT_DATA_ROOT/embodied-navigation/experiments/<run>/` | DynaNav/Isaac 实验产物：`capture/`、LIO observer、person-worker、RGB/bag 与各 `*.json` 回执；由 `scripts/run_dynanav_*` 写入，代码强制位于数据根目录下。 |
| 日志/运行产物 | 进程级遥测与证据 | **默认不落盘**：`common/runtime_telemetry.py` 的 `RuntimeTelemetry.append/enqueue` 是空操作（只有 `.atomic()` 真写实时状态），所以各模块里 `evidence.append(...)` 的历史 `.jsonl` 默认不产生；例外是显式同步写盘（如 `control/parking/causal_join.finish()` 的 `r104_parking_causal_join.json`）。判断某个产物是否真的存在，以 run 目录实际文件为准，不看代码里的 `append` 调用。 |
| 配置 | `configs/` | base 配置与契约数据，**不是**每次运行的有效值：`launch/navigation.json` 列出六个模块配置，`launch/prepare.py` 生成本次参数并 pin 若干键，运行期消费者分"读 `runtime_context`"与"直接重读 configs"两类。完整读取顺序、pin 与无读取点配置见 [配置入口](configs/README.md)。 |

"日志"在本仓库统一指**运行产物**（run 目录、实验目录、遥测/证据 JSON、ROS 日志），不是指仓库内某个日志文件夹；仓库不提交任何日志。各模块 README 只写自己产出的那部分，全局布局以 [配置入口](configs/README.md) 与 [数据与产物布局](docs/review/dsh-workflow-and-readability-20261109.md) 为准。

## 历史结构审查 · 模块耦合（2026-11-09）

按一级模块统计（AST 解析 2026-11-09），包内依赖**不是严格 DAG**：11 个一级模块之间有 9 组 A↔B 双向依赖。大多数是 1–2 条窄依赖（共享小工具/数据结构），真正构成层次倒置、值得优先处理的是下面 5 条（均可 grep 复查）：

| 层次倒置 | 证据 | 为什么值得处理 |
| --- | --- | --- |
| 规划持有控制器子类 | `planning/reference/global_route_reference.py:11` import `control.tracking.cascade_core.Controller`；反向 `control/tracking/cascade_core.py:16` import `planning.reference.spatial_reference` | 规划与控制互相依赖，二者无法单独测试或替换 |
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

各项的模块级细节（含 P0/P1/P2 分文件建议）见对应模块 README 的"重构建议"节，以及 [模块结构与耦合总评](docs/review/module-structure-coupling-20261109.md)。

## 两个运行边界

1. Native Gazebo 系统：start.py 装配已有导航模块。Dockerfile 从公开 ROS Jazzy、Gazebo Harmonic、CPU Torch 2.7.1 构建，必要自建源码均包含，FAST-LIVO2/SCAN/PCT 与补丁也在仓库中。
2. Isaac / DynaNav：当前 static native 探索使用 `scripts/run_dynanav_native_exploration.sh`（见上方专项架构）；其他隔离实验的入口是 scripts/run_dynanav_experiment.py（实验编排，在本进程内用 runpy 执行 simulation/dynanav/capture_go2_rgbd.py），配合 scripts/run_dynanav_lio.sh（独立观测器）与 scripts/run_dynanav_person_worker.py（独立行人推理）；observer 的 UDP 接收缓冲选择与回退见 [通信说明](docs/dynanav-observer-udp.md)；三者的在线行人组合顺序由 scripts/run_dynanav_online_person.py 监督（`--print-commands`/`--check` 可只打印或只核对；实跑时逐进程日志写到 run 的 `logs/`，失败/超时/SIGINT/SIGTERM 会先停 capture 再停 observer/worker）。它们仍需匹配的 Isaac 安装、场景与机器人资产。原机使用 Isaac Sim 6.0.1 RC；该版本的公开可获取性尚未核实，不能声称一份 Native Docker 镜像就能完整复现 Isaac。

2026-11-09 阶段的 DynaNav 在线感知、速度阶梯与路线状态见 [在线行人实验记录](docs/experiments/dynanav-online-person-20261109.md)；此前的楼梯通行、分层地图、动态行人与 LIO/LIVO 实测见 [实验记录](docs/experiments/dynanav-20261109.md)。历史原话及尚未确定的研究参数见 [动态语义建图约定](docs/research/dynamic-semantic-mapping-agreement.md)，其他文档见 [文档索引](docs/README.md)。这些隔离实验不等于生产导航整栈验收。

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

Linux 通过 `.env.local` 将 `PROJECT_DATA_ROOT` 指向新加卷的 `6.project_data`。项目数据根为 `$PROJECT_DATA_ROOT/embodied-navigation/`。启动前核对解析路径、挂载设备和剩余容量。大录包、点云、帧图像、视频、长日志统一写到项目数据卷。

- `runs/<批次>/<run-id>/` 存新运行和回放。`experiments/` 保留既有 Isaac 运行及部分旧入口。`runtime_assets/runs/` 对应既有原生 Compose 数据入口。
- `runtime_assets/` 存部署资产和模型，`models/` 存独立模型，`build_cache/` 存构建产物，`reports/`、`deliverables/` 存整理后的交付。
- 在项目数据根打开 `AGENTS.md` 查看放置和清理规则。仓库侧入口见 [协作规则](AGENTS.md#6-数据在项目外)。Git 保存源码、配置、实验定义和结论。换机器时，按配置提供有权使用的部署权重和场景。

新克隆需要原生数据链接时，使用 `python3 scripts/configure_data.py --data-root <数据根目录> --link`。正式运行应保留本次有效配置和源数据引用。

## 验证状态与限制

2026-11-09 阶段的生命周期修复与真实重验见 [运行故障修复记录](docs/review/runtime-repair-validation-20261109.md)。本轮真实 Isaac 在线行人仍因缺 RGB 后的原 8 帧缓存守卫中止；Gazebo PID 在 R85 路径证书门保持，未取得到达/5 秒终态停车证明。离线通过、进程退出与路线验收分开记录，未放宽安全门。

2026-11-09 较早阶段的独立复核、两项启动参数修复及当时 54 项单元/62 项回归结果见 [独立复核记录](docs/verification/independent-review-2026-11-09.md)。该阶段 5 项可选 Torch 测试跳过，未重跑仿真。后续 DynaNav 实际运行与最新测试另见上方实验记录。

2026-11-08 历史验证（针对当时的源码快照，不能代替后续修改的运行验证）：

- 当时源码48项单元测试通过，包括1000组原PID公式精确对比、导航任务、停车来源契约、生产SLAM入口及仿真安全门禁。官方ROS Jazzy纯CPU镜像的当时结果见docs/verification/cpu-container.json。
- 无数据的干净Git候选导出：62项回归成功结束；7项因未提供Torch或私有测量地图而明确跳过。双后端门禁集成和原native fixture另在装有CPU Torch的公共依赖容器实测通过；RoboGauge真实权重30帧输出与原JIT一致。
- FAST-LIVO2/SCAN等11个ROS包、PCT、4个自有原生模块、私有DDS补丁测试已在隔离公共依赖环境中从本仓库源码编译。完整Docker镜像已从公开ROS基础镜像构建成功，实际资源preflight通过；最终源码刷新和验证结果记录于docs/verification。
- 最终自足镜像45秒隔离软件栈通过：只挂本项目assets、models和新runs，不挂源码或其它工程的install；全部关键源码SHA匹配。1330次CPU策略推理、798帧相机、797帧LiDAR、57580个地图点；SCAN接受2条轨迹，22个健康非零命令样本及25个native执行确认样本。103条仿真安全门禁来源遥测全部有效，无物理fault；末段原有命令新鲜度门禁因来源超时拒绝并置零。15个组件全部退出0，日志无Traceback或未取回异常。详见docs/verification/software-startup.json。
- 此前发现的SIGINT关停竞态已修复：共用信号生命周期先退出spin和排空回调，最后关闭ROS context；真实callback异常仍传播，子组件非零退出会使主入口失败。48项单元包含6项生命周期测试、4项退出汇总测试及7项Building入口测试。
- 真实记录驾驶层回放1052/1052个样本通过：按目标点、航向误差和距航点距离独立唯一选帧，不用待检命令选帧，最大命令差异1.91e-14。它不是任务状态机或整栈证明。

仍未验收：生产导航整栈的完整路线、实机、策略训练和完整 Isaac 生产链路。docs/robogauge/isaac_closure_audit.json 记录较早的策略闭包审计；本轮 DynaNav 已补齐显式会话碰撞、RoboGauge 执行与采集诊断，并分别记录真值控制路线、首段 LIO 动态闭环和视觉旁路定位证据，不能据此声称生产链路已完整复现。

原测试3个停车样例缺少当前要求的pose_producer字段，在原工程也失败。本仓库更新fixture并纠正一处原源码要求真值标志为true的冲突：现在只接受显式非真值且生产者与时钟有效的SLAM反馈，没有放松阈值。运行源码和来源契约另见docs/architecture.md。

## 许可证和来源

已保留第三方源码、版本、许可证声明和补丁来源，详见 docs/dependencies。尚有Vikit授权来源、组合分发、自有源码/模型与部分资产来源问题需要确认，因此不能宣称公开发布许可已齐备。分发前请核对这些许可边界。

## 历史副本

dynanav/、history/、perception_history/ 是保留的原始/阶段副本，已从Git和Docker输入排除，不是本工程开发入口。这些本地目录不属于运行依赖，也不会进入Git或Docker输入。
