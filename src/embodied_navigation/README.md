# embodied_navigation 源码包

本包按“仿真/传感器 → 感知 → 定位建图 → 规划 → 任务与跟踪控制 → 机器人执行”的数据流组织导航系统，包内是自有 Python/C++ 源码与 ROS 2 装配代码，不含第三方算法实现、运行数据或构建产物。

## 目录结构

```text
src/embodied_navigation/
├── common/                 # 跨模块基础：配置、状态/时间契约、I/O、路径、关停（20 文件 / 1616 行）
├── control/                # 跟踪律、停车契约、仿真安全门禁与进程入口（46 文件 / 9718 行）
├── evaluation/             # 离线轨迹指标与独立进程资源采样（3 文件 / 95 行）
├── launch/                 # 系统入口与装配层：prepare、环境构造、ros2 launch、关停（9 文件 / 1314 行）
├── localization_mapping/   # 定位建图层：前端接入、已知场景、地图归档、后端配准（40 文件 / 8429 行）
├── navigation/             # 导航任务层：状态机、航点、停车、RoboGauge 编排（27 文件 / 11634 行）
├── perception/             # 图像/深度/行人分割与传感适配（18 文件 / 4631 行）
├── planning/               # SCAN 拥有与重启、准入/几何证明、PCT 参考路线（32 文件 / 9361 行）
├── robot/                  # Unitree Go2 观测、关节契约、策略推理与执行桥（17 文件 / 4591 行）
├── sensors/                # 跨层轻量数据契约（2 文件 / 37 行，当前无调用者）
├── simulation/             # Gazebo/Isaac 场景、物理/传感器、安全状态与真值流（43 文件 / 9095 行）
├── README.md               # 本文件：包索引、包级三类与模块间耦合
├── __init__.py             # 1 行：包 docstring
└── project.py              # 22 行：仓库/数据根发现（PROJECT_DATA_ROOT、EMBODIED_NAV_ROOT）
```

行数为 facts 包文本口径（`read_text().count("\n")+1`，2026-11-09；每文件比 `wc -l` 多 1，空文件记 1 行，模块 README 多用 `wc -l`）；全包 **259 个 Python 文件 / 60,546 行**（`wc -l` 为 60,287）。11 个一级子模块各自有 README，细节以子 README 为准，本文件不重复。

## 代码、日志与配置

包级三问（本包是库 + 可执行入口，运行产物主要落在包外）：

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | 11 个一级子模块 | 自有实现：仿真/传感适配、定位建图接入、规划/准入、导航任务、跟踪/停车/安全、机器人策略与执行、离线评测。第三方算法（FAST-LIVO2、SCAN、PCT、Vikit、Livox 驱动、FastDDS 补丁）在 `third_party`，本包只做接入与装配。 |
| 代码（包根） | [`project.py`](project.py)、[`__init__.py`](__init__.py) | `project.py` 只解析仓库根与外部数据根（`EMBODIED_NAV_ROOT`、`PROJECT_DATA_ROOT`），被 25 个模块/脚本导入；`__init__.py` 仅包 docstring。 |
| 代码（入口） | [`launch/main.py`](launch/main.py)、`start.py`、[`launch/building.py`](launch/building.py)、[`navigation/robogauge/benchmark.py`](navigation/robogauge/benchmark.py)、[`evaluation/process_resources.py`](evaluation/process_resources.py) | 四条执行路径的入口（见 [architecture.md](../../docs/architecture.md)）；具体进程图不在本文件展开。 |
| 日志/输出（默认主路径） | `runs/<UTC>_<label>_<4hex>/`（软链到外部数据卷） | `launch/main.py` 创建 run 目录；[`launch/prepare.py`](launch/prepare.py) 写入 `runtime_context.json`、`navigation_profile.json`、`navigation_fastlivo.yaml`、`v34_scan_overrides.yaml`、`world.sdf` 等；受管进程把日志/收据写在同一 run 目录。`runs` 是外部数据软链，内容不入 Git。 |
| 日志/输出（DynaNav） | 外部 `experiments/<name>/` | DynaNav 实验路径由 `scripts/` 与 [`simulation/dynanav/`](simulation/dynanav/README.md) 写入外部数据卷；`evaluation/process_resources.py` 的 `process_resources.csv` 也是 run 目录内的人工证据。 |
| 日志/输出（构建） | `install/` | 原生库/ROS 包安装目录（本机构建产物，不入 Git）；缺文件通常表示尚未构建，不代表缺源码。 |
| 配置 | `configs/`（包外） | 本包不内嵌配置：入口经 [`common/config.py`](common/config.py) 的 `read()` 合并 `configs/launch/navigation.json` 的六个模块文件，运行期部分进程重读 configs 或读 run 内 `runtime_context.json`；读取顺序、pin 与无读取点数据见 [configs/README.md](../../configs/README.md)。 |
| 配置（包内例外） | [`common/communication/dds_transport.xml`](common/communication/dds_transport.xml) | 唯一放在 `src/` 下的运行配置文件（DDS loopback UDP+SHM）；由 `launch/main.py`、`launch/building.py`、`navigation/robogauge/benchmark.py` 通过环境变量 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向。 |

本包根目录本身**不写日志、不建线程、不启动进程**：副作用集中在各 `main()`/入口模块；导入 `embodied_navigation` 只执行 `__init__.py` 的 docstring。

## 模块内文件耦合（包级模块间）

**数据流方向（意图）**：`simulation`（世界/真值/传感）→ `perception`（图像/深度/行人）→ `localization_mapping`（定位建图）→ `planning`（参考/准入）→ `navigation`（任务状态）→ `control`（跟踪/停车/安全）→ `robot`（策略/执行）；`common` 是所有模块的基础，`evaluation`/`sensors` 是离线/契约旁支。

**实际 import 方向（AST 解析，repo_stats.json）**：这是“意图”不是强制分层——包级有 **9 组 A↔B 双向依赖**，不是严格 DAG：

| 模块 | 出边（import 谁） | 入边（被谁 import） | 判断 |
| --- | --- | --- | --- |
| `common` | 仅包根 `project`（2） | 8 个模块（control/launch/loc/nav/perception/planning/robot/simulation） | 清楚：基础层，不反向依赖业务 |
| `sensors` | 无 | 无 | 孤立：预留契约，当前无调用者 |
| `evaluation` | 无 | 无（仅 `launch/building.py` 以 `-m` 子进程启动） | 孤立：离线工具，符合定位 |
| `navigation` | 7 个（common/control/loc/perception/planning/robot/simulation） | `localization_mapping`、`simulation` | 偏重：任务层聚合了几乎所有下层 |
| `simulation` | 7 个（common/control/launch/loc/navigation/perception/robot） | `navigation`、`robot` | 偏重：仿真同时依赖导航/机器人（DynaNav/RoboGauge 集成） |
| `control` | 5 个（common/launch/loc/planning/robot） | 6 个 | 偏重：控制层被多数层反向引用 |
| `localization_mapping` | 5 个（common/control/launch/navigation/perception） | 6 个 | 偏重：定位依赖控制/导航（见下） |
| `planning` | 4 个（common/control/launch/loc） | 4 个 | 清楚但含层次倒置 |
| `robot` | 4 个（common/control/planning/simulation） | 3 个（control/navigation/simulation） | 清楚：执行层 |
| `perception` | 3 个（common/control/loc） | 4 个 | 可接受：`perception→control` 是单向反向依赖（见下） |
| `launch` | 4 个（common/loc/perception/planning） | 4 个（control/loc/planning/simulation） | 可接受但角色重叠：装配层同时被当作运行期 helper |

**谁不依赖谁（重要负向结论）**：`common` 不 import 任何业务模块；`control`/`planning`/`robot`/`perception` 不 import `navigation`；`navigation` 不 import `launch`；`evaluation` 与 `sensors` 不被任何一级模块 import。

**9 组双向依赖（repo_stats.json `cycles2`）**：control↔planning、control↔robot、control↔localization_mapping、localization_mapping↔navigation、localization_mapping↔perception、navigation↔simulation、robot↔simulation、launch↔planning、launch↔localization_mapping。多数是 1–2 条窄依赖（共享小工具/数据结构），**不应直接当作架构缺陷**；但下列几处是真正的“层次倒置”，建议列入允许清单并加注释/测试（本轮不改代码）：

| 反向依赖（已 grep 核实） | 事实 | 判断 |
| --- | --- | --- |
| [`planning/reference/global_route_reference.py:11`](planning/reference/global_route_reference.py) import `control.tracking.cascade_core.Controller`；反向 [`control/tracking/cascade_core.py:16`](control/tracking/cascade_core.py) import `planning.reference.spatial_reference` | 规划侧定义控制器子类（`StableProjectionController`），控制侧又用规划的参考工具 | **层次倒置**：规划不应以控制器子类为实现细节 |
| [`localization_mapping/relative_odometry.py:8`](localization_mapping/relative_odometry.py) import `..navigation.task.Observation`；反向 navigation 有 6 处 import localization_mapping | 定位层使用导航层的数据类 | **层次倒置**：契约数据类应下沉到 `sensors`/`common` |
| [`perception/sensors/feedback_mailbox.py:595,618`](perception/sensors/feedback_mailbox.py) 延迟 import `control.tracking.cascade_core`、`control.parking.parking_pose10hz` | 传感适配层解释控制层内部反馈格式 | 单向反向依赖（非环）；应把反馈格式契约下沉 |
| [`planning/launch/components.py:4`](planning/launch/components.py)、[`localization_mapping/launch/components.py`](localization_mapping/launch/components.py) import `launch.launching`；反向 `launch/prepare.py:189,192` 延迟 import 这两个模块 | `launch` 既是装配层又是运行期 helper（`python()`/`owned()`） | 合法窄依赖，但需要注释说明分层 |

**4 类结构问题**：
- 上帝文件：**15 个文件 >800 行**（最大 `control/tracking/cascade_core.py` 1851、`control/parking/transition_gate.py` 1731、`navigation/base_node.py` 1618、`planning/admission/async_admission.py` 1200、`navigation/pid_control.py` 1174；完整列表见 repo_stats.json）。
- 重复实现：本轮未做全量重复检测（**未核实**）；已知 `localization_mapping/known_scene_*` 系列是同一功能的职责拆分，不是重复实现（见该目录 README）。
- 跨层直达：见上表 2 处层次倒置 + 1 处单向反向依赖；其余双向边多为窄依赖。
- 隐式契约：`runtime_context.json` 的 `profile` 字典（`load()` 只校验是 dict，无 schema）、ROS 话题名、launch 组件的 role 名、`processes.json` 的 `{role,pid}` 结构，都靠字符串键跨进程传递；改名不会在包级报错。

结论：包级耦合**总体清楚但非严格分层**——“数据流方向”成立，但存在 9 组双向窄依赖和 2 处真实层次倒置。适合把它们登记为“允许的窄依赖 + 2 个待重构项”，而不是宣称架构是干净 DAG。

## 可读性评估

按一级模块的实测指标（repo_stats.json，AST 粗计，2026-11-09）：

| 模块 | 文件/行数 | 函数 docstring 覆盖 | 无模块 docstring 文件 | 分级与证据 |
| --- | --- | ---: | ---: | --- |
| `launch` | 9 / 1314 | 0.57 | 2 | **良好**：小模块、覆盖率最高；入口/装配职责集中 |
| `evaluation` | 3 / 95 | 0.50 | 1 | **良好**：见其 README，两个纯函数文档完整 |
| `simulation` | 43 / 9095 | 0.40 | 3 | **可接受**：模块 docstring 覆盖较好，但 43 文件/9k 行、含 Isaac/Gazebo/DynaNav 三类路径 |
| `robot` | 17 / 4591 | 0.39 | 2 | **可接受**：契约/策略分层清楚，`bridge.py` 1069 行偏大 |
| `planning` | 32 / 9361 | 0.34 | 4 | **可接受**：准入/参考/PCT 拆分合理，但 `async_admission.py` 1200 行 |
| `control` | 46 / 9718 | 0.33 | 5 | **可接受偏低**：`cascade_core.py` 1851 行 + `transition_gate.py` 1731 行，两个“上帝文件”占模块近 37% |
| `perception` | 18 / 4631 | 0.29 | 1 | **可接受**：文件数少、职责清楚 |
| `navigation` | 27 / 11634 | 0.27 | 2 | **需改进（文档）**：模块最大、docstring 覆盖率低；`base_node.py` 1618、`pid_control.py` 1174 |
| `localization_mapping` | 40 / 8429 | 0.25 | 7 | **需改进（文档）**：覆盖率最低、7 个文件无模块 docstring；`known_scene_inputs.py` 1139 行 |
| `common` | 20 / 1616 | 0.27 | 3 | **可接受**：小而基础，关键契约有 docstring（如 `config.read()`、`runtime_context.load()`） |
| `sensors` | 2 / 37 | 无函数 | 1 | 不评价：3 个 dataclass，已有字段级 docstring |

- **结构问题**：集中在 `control`/`navigation` 的大文件（15 个 >800 行文件里 control 2 个、navigation 5 个、planning 4 个、loc 1 个、robot 2 个、perception 1 个、simulation 0），以及包级 9 组双向依赖；这些会提高改动成本。
- **文档问题**：`navigation`/`localization_mapping` 的函数 docstring 覆盖最低（0.27/0.25），且无模块 docstring 的文件最多（2/7）；这不是结构缺陷，补 docstring 即可。
- 命名与注释：模块 README 质量普遍较高（11 个一级模块都有 README，多数写了“不做什么”的边界）；公共入口（`config.read()`、`project.project_root()`、`launch/prepare.prepare()`）有完整 docstring。
- 未发现无证据的“结构清晰”类空话：本文件所有结论都以 import 图/行数/覆盖率/文件行号为据。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P0 | 无 | 包级没有发现已知的正确性/安全缺陷（安全门禁有单测与原生集成测试；`common/state_contract.py` 拒绝非法 provenance） | — | — |
| P1 | 拆分 2–3 个最大“上帝文件”：`control/tracking/cascade_core.py`（1851）、`control/parking/transition_gate.py`（1731）、`navigation/base_node.py`（1618） | 单文件过大，review/测试定位成本高；三者都在高频改动路径 | 高：涉及控制/状态机热路径，拆分可能改变 import 关系与测试 AST 断言 | 先补行为对照（现有 `tests/unit` + `tests/regression`），按“纯移动 + 重导出”分步拆；每步跑全量 CPU 测试并检查入口闭包测试 |
| P1 | 消除 2 处真实层次倒置：① 把 `Controller` 基类/协议抽到 `control/tracking/controller_protocol.py`，规划只依赖协议；② 把 `navigation/task.Observation` 下沉到 [`sensors/frames.py`](sensors/frames.py)（该模块已是跨层契约且当前无调用者）或 `common` 契约模块 | 规划→控制、定位→导航的反向依赖让“数据流方向”失真，且会阻碍独立演进 | 中：涉及 import 面与兼容重导出；`sensors` 当前无调用者，需先确认契约归属 | 移动后全量 CPU 测试；`grep` 确认无新的反向边；保留旧路径重导出并加弃用注释 |
| P1 | 加一个“包级 import 环/允许清单”检查脚本或单测（允许 9 组窄依赖白名单，新增环即失败） | 现在 9 组双向依赖只存在于报告里，新增反向依赖无人拦截 | 低：只加检查，不改业务代码 | 脚本对当前树输出 9 组白名单通过；人为加一条 import 验证会失败 |
| P2 | 提高 `navigation`（0.27）/`localization_mapping`（0.25）的函数 docstring 覆盖，优先公共入口/状态机方法；补 9 个无模块 docstring 文件的模块头 | 文档缺口集中在这两个最大模块，补文档风险最低、收益直接 | 无（纯注释） | 重跑 repo_stats 的 docstring 统计，覆盖率上升；行为测试不变 |
| P2 | 在 [architecture.md](../../docs/architecture.md) 或本文件登记“允许的窄依赖清单”（9 组 + 2 处待重构），并在代码处加一行注释 | 避免后来者把合法窄依赖当缺陷反复“修” | 无 | 人工对照 repo_stats.json 的 `cycles2` |
| P2 | 明确 `sensors` 的定位：要么在 P1 中承接 `Observation` 等契约，要么在包索引声明“预留、未接入” | 当前既无调用者又有“跨层契约”名称，容易被误用 | 低 | `grep -rn "embodied_navigation.sensors"` 仍为空或按新契约更新 |

没有 P0：包级正确性风险主要由仿真/实机未验证承担，而不是代码结构；P1 的两类动作都需要“先有测试/对照再动”，本轮只写建议、不改代码。

## 按数据流阅读

1. **启动与装配**：仓库根 `start.py` 调用 [launch](launch/README.md) 的 `main.py`，校验配置与依赖、生成单次运行目录，再由 [launch/launch](launch/launch/README.md) 的 `demo.launch.py` 包含定位、规划、控制、仿真四个模块 launch。
2. **世界与传感器**：[simulation](simulation/README.md) 生成 Gazebo 世界，启动策略 actor、传感器桥与仿真进程，把相机/LiDAR/IMU 数据送入 ROS。仿真原生状态还服务于策略本体观测、COM 速度内环及安全/评测；生产 SLAM 和上层导航位姿不以真值替代。
3. **传感流适配**：[perception](perception/README.md) 与 [perception/sensors](perception/sensors/README.md) 负责中继、自身回波过滤、采样和传感门禁；跨边界的数据类型定义在顶层 [sensors](sensors/README.md)。
4. **定位建图**：[localization_mapping](localization_mapping/README.md) 接入 FAST-LIVO2 前端（真实算法在 `third_party/fast_livo2`，本包只是接入与包装），后端负责注册与图优化，并输出带生产者和时间信息的定位反馈。 [semantic_mapping](localization_mapping/semantic_mapping/README.md) 提供观测分层、对象历史和只读结构化查询，实验接入范围单独记录。
5. **规划**：[planning](planning/README.md) 提供参考路线、SCAN 调度和 PCT 路线生成（PCT 真实算法在 `third_party/pct`）；admission 负责路径准入与交付。
6. **任务与跟踪控制**：[navigation](navigation/README.md) 维护任务状态与既有 PID 导航；[control](control/README.md) 计算跟踪/停车命令，并包含仿真安全门禁与定位反馈来源检查。
7. **机器人执行**：[robot](robot/README.md) 下的 [robot/unitree](robot/unitree/README.md) 负责 Go2 观测、策略推理和执行桥。当前 Student 用 exact45 推理，历史 Teacher 247D 观测保留作诊断；RoboGauge 使用另一套独立 45D 接口，观测与 actuator 契约不能混用。
8. **离线与横切**：[evaluation](evaluation/README.md) 提供离线轨迹指标及独立进程资源采样；[common](common/README.md) 提供配置、路径、时钟、通信与运行遥测等公共件。

## 包边界与实现归属

- 自有实现在本包；仓库 [configs](../../configs/README.md)、`scripts`、[tests](../../tests/README.md)、`docs` 分别放配置、脚本、测试与构建/来源说明。
- 第三方实现（FAST-LIVO2、SCAN、PCT、Vikit、Livox 驱动、FastDDS 补丁）在 `third_party`，本包只做接入与装配。
- `install/` 只是本机构建产物，缺文件通常表示尚未构建，不代表缺源码。
- native64 的本体状态用途与导航定位来源分开：前者用于现有仿真策略/控制内环及安全/评测；上层定位与导航位姿按实际生产者、标志和时钟契约接收 SLAM 反馈。

## 入口与验证

- 以下命令从仓库根目录执行；源码布局的 Python 模块入口需设置 `PYTHONPATH=src`，或先安装本包。
- 运行入口：`python3 start.py --check` 解析配置并检查依赖与资产；`--prepare-only` 准备运行文件而不启动 ROS/仿真；普通启动会实际运行系统，可用有限正数 `--duration` 限制墙钟时间。
- 测试入口：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v` 与 `... -s tests/regression -v`，均为离线 CPU 测试，不启动仿真。
- 原生构建、依赖与许可边界见 [构建说明](../../docs/dependencies/BUILD.md) 与 [依赖总览](../../docs/dependencies/README.md)。

## 未验证边界

Isaac/DynaNav 端到端、实机、完整路线与 GPU 训练尚未验证；prepare 成功、测试通过或 `install/` 有文件都不等于完整系统可运行，运行仍需匹配的 ROS/Gazebo 环境、资产与权重。
