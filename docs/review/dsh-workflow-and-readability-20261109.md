# embodied-navigation 工程流程导读与可读性整理（2026-11-09）

本文面向**刚加入本工程的开发者**，回答两个问题：

1. 这个仓库到底有哪几条流程、它们如何启动、数据怎么在模块之间流动；
2. 哪些地方的写法/命名/文档会让人（尤其是新人）读错，为什么，以及怎么改。

**后续进展（2026-11-09 同日）**：本报告的部分建议已在独立的实施轮中落地（入口/状态机文档、查询逻辑归位、prepare pin 显式化、在线流程 supervisor、若干文档事实修正）；实际改动、被更正的判断和未实施项见 [工程流程修复与重构实施记录](dsh-workflow-fixes-20261109.md)。该轮仍未提交 Git。

**再后续（同日第四轮）**：导读报告 §8 中"`pid_navigation.py` 按职责拆分"与"`capture_go2_rgbd.py` 去掉 import 副作用、包出 `main()`"两项已实际实施，`capture_go2_rgbd.py` 的 `RuntimeError` 导入守卫已被移除；实现、兼容入口、行为验证与剩余限制见 [现役 controller 与 DynaNav 采集入口结构重构](dsh-structural-refactor-20261109.md)。

**本文的边界（重要）**

- 本报告这一轮只做阅读与文档整理，**没有修改任何既有实现、配置、测试或权限**，也没有提交 Git（实施轮的改动见上面的链接，不并入本文结论）。
- **本次会话没有启动仿真、没有运行测试，也没有重跑任何一条路径**（用户已明确要求继续暂停）。所有"进程会这样启动""顺序是这样"的结论来自**读代码**，不是本次实测。`python3 start.py --check` 在本次执行环境返回 2（13 个固定路径中 6 个不存在）——这只说明**该环境**尚未构建 `install/` 等产物（`docs/verification/independent-review-2026-11-09.md:7` 同样把它们解释为"正常的未构建输出"），**不能据此推断 Linux 宿主机/镜像里四条路径都跑不了**。历史运行状态见 §2 的三层表。
- 依据是**当前工作树**（含 28 项既有未提交改动：9 modified + 19 untracked；本文自身新增的 1 个文件不计入这 28 项），不是 HEAD `43cba33`。未提交的新模块（`person_worker.py`、`learned_mapping.py` 等）在本文中按"已存在于磁盘"处理。
- 行号是本次阅读时的行号，后续改动会漂移；引用时优先看"文件 + 符号名"。

---

## 0. 先记住这 8 件事

1. **仓库里有 4 条互不共享启动器的执行路径**，`start.py` 只覆盖其中一条。另外三条（Building、RoboGauge、DynaNav/Isaac）都是独立程序，只有 `start.py` 那条用 `demo.launch.py`。
2. **默认主路径** = `start.py` → `launch/main.py` → `prepare()` 生成一个全新 run 目录 → 启动**唯一子进程** `ros2 launch demo.launch.py`，由它拉起 **15 个受管进程**（+1 个不受管的就绪屏障）。
3. **真正的任务状态机在 `navigation/base_node.py` 的 `Navigation.control()`**（1611 行里的 `:1071-1488`），由 `ros_node.TeacherNavigation`、`pid_navigation.PIDNavigation`、`pid_navigation.IntegratedNavigation` 逐层覆盖。`navigation/task.py` 与 `navigation/mission/state_machine.py` **都不在主路径上**（前者只服务独立的 `relative-slam-navigation` 入口，后者在本检出中没有任何实例化点）。
4. **命令传递是"ROS 话题 + run 目录 JSON 文件"两级结构**：控制器发 `/demo/cmd_vel` → `robot/unitree/bridge.py` 校验后原子写 `navigation_command.json` → actor 进程读该文件 → 走 Unix socket 写关节目标。**不存在"命令话题直达执行器"的捷径**。
5. **时间有 4 个基准**（ROS/sim 时间、steady-wall 时间、native 物理时钟、DynaNav source 时间），**来源有 2 条硬边界**（仿真真值 vs SLAM 估计，由 `common/state_contract.py` 的 typed provenance 强制）。几乎所有"这个 0.3 s 是什么意思"的困惑都来自这两件事。
6. **配置不是单一事实源**：`prepare()` 会冻结一部分配置进 run 目录，但 actor 侧**运行时仍重读** `configs/`；`configs/launch/settings.json` 里还有 2 个开关被启动器强制改成 `0`。
7. **文档是"审计账本"风格**：事实准确、边界声明充分，但按"证据/不能证明什么"组织，不按"新人要做什么"组织。想跑一次 DynaNav，从根 README 出发会走到 55KB 实验日志里。
8. **目录里有 3 个历史副本**（`dynanav/` 327MB、`history/` 190MB、`perception_history/` 35MB），已被 git/Docker 排除、**不是开发入口**；但 `dynanav/README.md` 自己仍给了一套完整运行步骤，容易误入。

---

## 1. 阅读地图：按"我要回答什么问题"找文档

| 你想知道 | 读这里 | 说明 |
| --- | --- | --- |
| 工程是什么、验证到哪一步 | `README.md`（根，69 行） | 结构/边界准确；但"最新实验"指针和测试计数已过期（见 §11 C-1/C-2） |
| 模块如何串起来、数据边界、安全门禁 | `docs/architecture.md`（34 行） | 全仓性价比最高的一页；不含命令、不含新的在线 learned 分支 |
| 某目录做什么、入口是谁 | `src/embodied_navigation/README.md` → 目标目录的 `README.md` | 46 个模块 README 齐全，相对链接基本零断链 |
| 某行为被什么钉住 | 测试文件首行 docstring | 本仓最好的文档资产之一；但没有 `tests/README.md` 索引 |
| 配置链和默认值 | `configs/launch/navigation.json` + `configs/scenarios/README.md` | 入口只列 6 个 `settings.json`，合并规则在 `common/config.py` |
| 第三方源码、构建、许可 | `docs/dependencies/README.md` → `BUILD.md` / `PYTHON-AND-DEPLOYMENT.md` / `LICENSE-STATUS.md` | 2026-11-08 快照 |
| "到底实测过什么" | `docs/experiments/`：`dynanav-online-person-20261109.md`（最新）→ `-continuation.md` → `dynanav-20261109.md` | 是证据日志，不是操作手册 |
| 阈值/校验的原始审计 | `docs/review/validation-and-threshold-audit.md` | 行号绑定旧 HEAD `b44f748d`，已漂移 |
| 可读性改造账本（589 个目标） | `docs/review/full-readability/` | 机器可读、可核对 |

**建议阅读顺序**：根 README → `docs/architecture.md` → 你负责模块的目录 README → 对应测试 docstring → 需要动手时再看实验日志。

**任何文档都没有写的（新人会撞墙）**：

1. 没有术语表。`VIO` / `LIO` / `LIVO` / `PCT` / `SCAN` / `GT` / `COM` / `45D` / `247D` / `native64` / `exact45` / `post-VIO` 全部首次出现即使用（术语表见附录 A）。
2. 没有测试索引（40 unit + 16 regression 文件与模块的对应关系只能靠读 import；本文 §10 给了一份）。
3. 没有配置清单：43 个 config 文件里哪些是运行必需、哪些没人读，全仓无文档（本文 §10 / 附录 D）。
4. 没有术语一致的中英约定：同一目录里 `configs/simulation/dynanav/README.md` 是中文、`README_LIO.md` 是英文。
5. 没有把"当前处于暂停状态"写进任何已提交文档（只在未提交的最新实验报告里，见 §11 C-5）。
6. 没有可移植的环境搭建说明：命令里写死 `/home/hyh001/isaacsim/python.sh`，Isaac 6.0.1 RC 的获取方式未核实。

---

## 2. 四条执行路径总览

只记住这张表，就不会把"某个命令"套到错误的场景上。

| | **A. Native 导航主路径** | **B. Building（SLAM-only）** | **C. RoboGauge benchmark** | **D. DynaNav / Isaac 实验** |
| --- | --- | --- | --- | --- |
| 入口 | `start.py` → `launch/main.py:136` | `launch/building.py:496`（`main`） | `navigation/robogauge/benchmark.py:381` | `scripts/run_dynanav_experiment.py:4` + `scripts/run_dynanav_lio.sh` + `scripts/run_dynanav_person_worker.py` |
| 准备 | `launch/prepare.py:20` | `building.py:25`（自带 prepare，只接受 `--mode slam`） | `simulation/gazebo/robogauge/world.py:31` | `experiment.py:322-365`（profile + 源码快照） |
| 进程管理 | 1 个 `ros2 launch` 子进程 + 4 个模块 launch → 15 进程 | 自带 `Popen` 管理 | 自带 `Popen` 列表 | 无进程管理：`runpy` 在本进程内跑 capture；observer/worker 是另外两个终端 |
| 仿真 | Gazebo Harmonic (`gz sim`) | Gazebo | Gazebo | Isaac Sim（USD 场景） |
| SLAM | `install/ros/.../fastlivo_mapping` + run 目录 yaml | `building_fastlivo.json` + backend | FAST-LIVO2 + backend + `building_runtime` | 独立 observer 进程（可选 ONLY_LIO / LIVO），或默认 GT 诊断 |
| 控制 | `pid_navigation` + `bridge` + `mission_runtime` | `building_navigation` | RoboGauge worker（native64 循环） | 进程内 `RouteTracker` / `SlamRouteTracker` |
| 输出 | `runs/<UTC>_<label>_<hex>/` | 数据卷 `runs/` | 数据卷 `runs/robogauge/<label>` | `$PROJECT_DATA_ROOT/embodied-navigation/experiments/<run>/` |
| ROS_DOMAIN_ID 默认 | 231 | 226 | 226 | 91（常规）；116（45 s 在线片段）；117（完整路线）。注意 person worker 走**文件 IPC、不依赖 ROS domain**，只有 observer 与 capture 必须同域 |

**四条路径之间共享什么、不共享什么**

- **共享代码，不共享进程图**：`perception/depth.py`、`localization_mapping/semantic_mapping/`、`robot/unitree/robogauge/`、`control/safety/simulation_gate.py`、`simulation/safety_state.py` 被多条路径复用。
- **不共享话题**：主路径用 `/demo/*`，DynaNav 用 `/dynanav/*`。**不能靠"两条都启动"把它们接起来**。
- **不共享锁**：`start.py` 取 3 个 `/tmp` flock（`launch/main.py:167-171`），`building.py:576` 只取 `building_navigation.lock`，benchmark 取 4 个。**后果：`start.py` 与 `building.py` 可以同时跑在同一台机器/同一个 ROS/Gazebo 主机上**（见 §11 B-14）。
- **域号是纯魔法数**：231 / 226 / 91 / 119（RViz 审查）互不相关，没有命名常量。

**运行状态分三层，本文全篇按此口径**

| 层次 | 主路径 A | Building | RoboGauge | DynaNav |
| --- | --- | --- | --- | --- |
| 本次会话重跑 | 否 | 否 | 否 | 否（用户明确暂停） |
| 历史已记录 | Docker 镜像内 45 s 默认 smoke：15 组件退出 0（`docs/verification/software-startup.json`）；完整任务闭环未列为通过项 | 入口/构建文档齐全（`docs/dependencies/BUILDING-ENTRYPOINT.md`），未见运行归档 | `docs/robogauge/` 有归档数值（Isaac 首飞 68.815/73.815 s）；Gazebo benchmark 本轮未复跑 | **真实成功运行**：无行人四档（各轮四进程退出 0）+ 0.3 m/s 在线行人完整路线（domain117，五进程退出 0，`mission_completed=true`）+ 45 s 片段（`mission_completed=false`）；感知质量未通过（person 漏过 39.65%） |
| 尚未验证 | 生产整栈的完整任务闭环 | 端到端运行 | 本轮复跑 | 行人质量指标（报告明确"不算动态感知验收"） |

关键区别：**"本次没有重跑" ≠ "跑不了" ≠ "历史没跑过"**。`start.py --check` 返回 2 只说明当前环境没构建 `install/`；DynaNav 走数据卷的显式 `--binary-prefix` + 宿主 Isaac，已有成功运行记录。

---

## 3. 主路径 A：Native Gazebo 全栈

### 3.1 启动时序（`start.py` → 15 个进程）

| # | 阶段 | 位置 | 关键点 |
| --- | --- | --- | --- |
| S1 | 参数校验 | `launch/main.py:105-133` | `--domain` 必须 1..232（默认 231）；`--duration` 必须有限正数；非法直接 `parser.error`（退出码 2），**在任何配置读取/建目录之前** |
| S2 | 读配置 | `launch/main.py:147` → `common/config.py:42-73` | `configs/launch/navigation.json` 的 `modules` 顺序合并；**跨模块重复 key 直接报错**；入口文件自己的 key 最后覆盖（静默）；`read()` 无缓存，每次重读 |
| S3 | 文件级预检 | `launch/main.py:150` → `launch/preflight.py:6-47` | 13 个固定文件是否存在（Student 权重、actor 解释器、Go2 URDF/SDF、world、route、L1 mesh、actuator、`writer_qos.so`、patched DDS、ROS install、SLAM、SCAN）。`ready = 全部 is_file` |
| S4 | 依赖门 | `launch/main.py:151-152` | 普通启动缺依赖直接 `RuntimeError`；`--check` / `--prepare-only` 允许缺依赖 |
| S5 | 资源门（可选） | `launch/main.py:153-154, 86-102` | `--resource-check`：load1 ≤ 0.9×CPU 核数，`MemAvailable` ≥ 4 GiB |
| S6 | `--check` 出口 | `launch/main.py:155-157` | 打印 JSON，返回 0（ready）/ 2（不 ready）。**本次执行环境实测 exit=2，13 项里 6 项缺失**；缺的是未构建产物而非源码（`docs/verification/independent-review-2026-11-09.md:7,27` 同一解释），在完成构建的环境里不成立 |
| S7 | 串行锁 | `launch/main.py:167-174` | 3 个非阻塞 flock（`go2_pid_physics_serial` / `go2_model_pid_simulation` / `go2_navigation_dispatch`），失败抛 `BlockingIOError` |
| S8 | 建 run 目录 | `launch/main.py:176-182` | `ROOT/runs/<UTC>_<label>_<4hex>`；`runs` 是指向外部数据卷的软链，未配置数据链接会在这里失败 |
| S9 | **prepare()** | `launch/main.py:183` → `launch/prepare.py:20-149` | 阻塞子进程：生成 world/sensor/桥/FASTLIVO/相机/SCAN/route/anchor 等（产物见 §3.3 与 §8） |
| S10 | `--prepare-only` 出口 | `launch/main.py:188-190` | 只准备不启动；锁仍持有到 `finally` |
| S11 | 构造子进程环境 | `launch/main.py:193, 19-83` | 阻塞 bash 子进程 source `/opt/ros/jazzy` + 项目 `install/ros/local_setup.bash`，取回 `env -0`；再设 DDS profile、domain、`GZ_PARTITION`、`TEACHER_SOCKET`、run 目录、线程数、FASTLIVO 开关；最后注入 `features` 并把 7 个探针强制 `"0"` |
| S12 | 信号处理 | `launch/main.py:195-204` | SIGINT/TERM/HUP 只置 `stop` 标志；旧 handler 保存但不恢复 |
| S13 | 启动唯一子进程 | `launch/main.py:240-248` | `ros2 launch <...>/demo.launch.py run_dir:=<dir>`，`start_new_session=True`，日志 `runs/<dir>/launch.log`，记录 `/proc/<pid>/stat` 起始 tick 写入 `processes.json` |
| S14 | 监控循环 | `launch/main.py:260-269` | 每 0.2 s 只 poll 直接子进程；`--duration` 是**墙钟**秒 |
| S15 | 关停 | `launch/main.py:275-308` | 只对 tick 匹配的直接子进程发 SIGINT；共享 15 s 等待；超时再对进程组 SIGTERM + 5 s |
| S16 | 退出记录 | `launch/main.py:310-353` | 汇总 `component_exit.jsonl` + 直接子进程返回码 → 写 `exit.json`，打印 `{"stage":"stopped"}`；**若在第一个子进程被记录前就失败（prepare/环境），不会写 `exit.json`** |

### 3.2 15 个受管进程（+1 个屏障）

`demo.launch.py:15-20` 依次 include 四个模块 launch，每个模块 launch 只是 `DeclareLaunchArgument("run_dir")` + `OpaqueFunction`，真正内容在各自 `launch/components.py`。

| 组 | 角色（日志名） | 启动的模块/二进制 | 作用 |
| --- | --- | --- | --- |
| localization (8) | `sensor_relay` | `perception.sensors.sensor_relay` | Gazebo 相机/IMU → `/demo/teacher/slam/{image,imu}`，**保持源时间戳不修复**；为本进程单独改 publisher QoS（sync/async 由 profile 的 `sensor_relay_publish_mode` 决定） |
| | `self_filter` | `perception.sensors.self_echo_filter` | `/livox/lidar`（remap 自 `/demo/teacher/raw_lidar`）→ 去自车回波 → `/demo/slam/lidar_filtered`（另发 `raw_point_identity`、`lidar_filter_status`） |
| | `camera_parameters` | `demo_nodes_cpp/parameter_blackboard` | 把 `navigation_camera.yaml` 发布成可查询参数服务 |
| | `slam` | `install/ros/lib/fast_livo2_ros/fastlivo_mapping` | **真正的 SLAM 算法二进制**（源码在 `third_party/fast_livo2`），读 run 目录两份 yaml |
| | `odom_adapter` | `localization_mapping.odom_adapter` | `/aft_mapped_to_init` → `/demo/slam/raw_body_odom` + `/demo/slam/raw_lidar_odom`（含有效 twist） |
| | `map_archive` | `localization_mapping.mapping.map_archive` | 运行期观测地图归档 |
| | `global_matching` | `localization_mapping.localization_runtime` | 已知场景辅助定位；输出 `/demo/slam/{body,lidar}_odom`、`/cloud_registered_full`、`/demo/navigation/localization_bundle`、`localization_status` |
| | `sensor_gate` | `perception.sensors.sensor_gate` | 传感器/SLAM 预热门，发布 `/demo/teacher/navigation/sensor_health`（`ready` 才允许发起路线请求） |
| planning (1) | `scan_supervisor` | `planning.scan_epoch_supervisor` | **唯一进程**，它自己再 spawn `scan_planner_node`（C++，不受 launch 追踪）；参数来自 run 目录 `v34_scan_overrides.yaml` |
| control (3) | `controller` | `navigation.pid_navigation` | 现役控制器（实际节点类 `IntegratedNavigation`，4702 行的文件） |
| | `command_bridge` | `robot.unitree.bridge` | 订阅 `/demo/cmd_vel`，做 15 类否决码校验，原子写 `navigation_command.json` |
| | `mission` | `navigation.mission.mission_runtime` | 只在测得静止后**冻结一个目标**，1 Hz 发 `/demo/navigation/request` 与 `/demo/teacher/navigation/anchor` |
| simulation (3) | `actor` | `robot.unitree.policy.worker` | 读 `navigation_command.json` → Student 策略推理 → Unix socket 写关节目标；`CUDA_VISIBLE_DEVICES=""`（CPU 推理） |
| | `sensor_bridge` | `ros_gz_bridge/parameter_bridge` | Gazebo 话题 ↔ ROS 话题，配置 `run/sensor_bridge.yaml` |
| | `gazebo` | `gz sim -s -r --headless-rendering world.sdf --seed 42` | 物理+渲染服务端 |
| （不受管） | `actor_ready` | `simulation.wait_actor` | 30 s 就绪屏障；**不是 `owned()`**，失败只体现在 launch 日志，不进 `component_exit.jsonl` |

**启动顺序是真的有依赖的**：actor 与 sensor_bridge 先起 → `actor_ready` 等 `<run>/worker_ready` + socket + PID 存活 → 屏障退出 0 才启动 Gazebo（`simulation/launch/components.py:44-62`）。其余进程并发启动，**没有互相等待**，"谁先就绪"由各节点自己的门（`sensor_gate`、`worker_ready`、`processes.json`）处理。

**任何一个受管组件退出** → `launching.owned()` 的 `OnProcessExit` 写 `component_exit.jsonl` 并 `EmitEvent(Shutdown)` → 整个 launch 树收缩 → `start.py` 看到直接子进程非 0 → `RuntimeError`，`exit.json` 记录原因。

### 3.3 一次完整任务的数据流

```
                    ┌──────────────────────── configs/ (6 个 settings + yaml) ───────────────────────┐
                    │                          common/config.py read() / path()                     │
                    └───────────────────────────────────┬──────────────────────────────────────────┘
                                                        ▼
 start.py ──► launch/main.py ──prepare()──► runs/<stamp>/   (world.sdf, sensor_contract.json,
                                             navigation_fastlivo.yaml, navigation_camera.yaml,
                                             v34_scan_overrides.yaml, global_route.json,
                                             runtime_context.json, sensor_bridge.yaml, ...)
                                                        │
                                          ros2 launch demo.launch.py run_dir:=...
                                                        ▼
 ┌─────────────── simulation 组 ───────────────┐     ┌──────────── localization 组 ───────────────┐
 │ gz sim (物理/渲染) ──► sensor_bridge ──► ROS 话题 │     │ sensor_relay ─► /demo/teacher/slam/{image,imu} │
 │        ▲                     │              │     │ self_filter  ─► /demo/slam/lidar_filtered       │
 │        │ 关节力矩            ▼              │     │            fastlivo_mapping (C++)               │
 │ libteacher_actuator.so  /demo/teacher/...   │     │ odom_adapter ─► /demo/slam/raw_{body,lidar}_odom│
 │        ▲                                    │     │ localization_runtime ─► /demo/slam/body_odom    │
 │        │ Unix socket (64 float↔16 float)    │     │ sensor_gate ─► /demo/teacher/navigation/sensor_health
 │ actor: policy.worker ◄── navigation_command.json  └───────────────────────┬────────────────────────┘
 └────────────────────────────────────────────┘                             │
        ▲                                                                    ▼
        │                          ┌──────────── control 组 ────────────────────────────┐
        │                          │ mission_runtime ──/demo/navigation/request─────────►│
        │                          │                  ──/demo/teacher/navigation/anchor─►│
        │                          │ pid_navigation (base_node.control)                 │
        │                          │   ├─ /demo/navigation/scan_reference ──► SCAN      │
        │                          │   ├─ ◄── /demo/navigation/bspline + metadata       │
        │                          │   ├─ /demo/cmd_vel ──► bridge.CommandGate          │
        │                          │   └─ /demo/navigation/status ──► run 目录 json      │
        │                          │ bridge ──/demo/control/safety──────────────────►    │
        │                          └───────────────────────────────────────────────────┘
        │                          planning 组：scan_supervisor → scan_planner_node (C++)
        └──────────────────────────────────────────────────────────────────────────────
```

**关键接口的真实方向**（易错点：不是所有下游都"订阅上游话题"，有一半是读文件）：

| 边 | 载体 | 生产者 | 消费者 |
| --- | --- | --- | --- |
| 目标/请求 | 话题 `String`(JSON) `/demo/navigation/request` + 文件 `navigation_request.json` / `navigation_anchor.json` | `mission_runtime` | `base_node.on_request`、admission |
| 路线参考 | 话题 `Path` `/demo/navigation/scan_reference` + 文件 `global_reference.json` | `base_node.request_plan` / `mission_runtime` | SCAN planner |
| SCAN 轨迹 | 话题 `Bspline` `/demo/navigation/bspline` + `/trajectory_metadata` 等 | `scan_planner_node`（经 supervisor remap） | `base_node.on_spline` |
| 定位 | 话题 `/demo/slam/body_odom`、`/cloud_registered_full`、`localization_bundle`、`localization_status` | `localization_runtime` | 控制器、admission、bridge |
| 速度命令 | 话题 `Twist` `/demo/cmd_vel` | `base_node.publish_command` | `bridge` |
| 执行命令 | **文件** `navigation_command.json`（原子替换 + flock） | `bridge.CommandFile.write` | `policy/worker.py` 读文件 → `control/executor_command.py` 二次校验 |
| 安全反馈 | 话题 `String` `/demo/control/safety` | `bridge` | `base_node.on_bridge_safety` / `ros_node.apply_tilt_guard` |
| 关节目标 | **Unix socket**（`TEACHER_SOCKET`），64 float 状态 → 16 float 回复 | `policy/worker.py` | Gazebo actuator 插件 `libteacher_actuator.so` |
| 真值执行命令（仅 Building/RoboGauge） | 文件 `truth_navigation_command.json` | `building_navigation.py:623` / robogauge worker | `simulation/truth_worker.py:77`（Student 仿真适配）、robogauge worker |

### 3.4 任务状态机（真正的那个）

`base_node.Navigation.control()`（`base_node.py:1071-1488`）是每个控制周期都跑一遍的"状态机 + 门禁链"。它的状态与迁移：

| 状态 | 进入位置 | 退出/迁移 | 触发条件 | 该状态下发出的命令 |
| --- | --- | --- | --- | --- |
| `idle` | `base_node.py:149-150` | → `running` | 收到合法 request | 精确零 |
| `running` + `pre_turn` | `base_node.py:586`（request_start）等 | → `align` | 测得"已停止"合格（`transition_gate.py:1330-1338`） | 精确零 |
| `running` + `align` | `transition_gate.py:1336` | → `settle` / `drive` | 航向误差 ≤ `turn_complete_rad`，再 1 s 零沉降 | 仅偏航 |
| `running` + `drive` | `transition_gate.py:1328-1330` | → `pre_turn` | 航向误差超门限，需换向 | 跟踪命令 |
| `running`（到达） | `base_node.py:1144-1183` | → 下一目标 / `succeeded` | 旧式：距离 < `arrival_radius` 保持 0.4 s；新式：`ArrivalWindow` 按 measured stamp 的 dwell | 零 |
| `succeeded` | `base_node.py:1179-1180` | → 停车流程 | 所有 waypoint 消费完 | `pid_navigation` 的停车控制律 |
| `failed` | `base_node.py:375/1007/1318/1086/1203/1112` 等 | 终态（latch） | 倾覆、交接失败、超时、连续 8 s 源过期、控制器异常 | 零（锁存） |
| `stopped` | `base_node.py:596` | 只能由新 request 恢复 | `/demo/navigation/stop` 收到 True | 零 |

`control()` 内部的门按固定顺序求值（这才是新人要读的"状态机"）：

1. 声明的区域 deadline 到 → `failed`
2. 倾覆/桥安全（`apply_tilt_guard`，0.50 rad 锁存 / 0.30 hold / 0.18 释放 + 0.8 s）→ 零
3. 源过期（pose 0.8 s、cloud 1.0 s、raw IMU）→ 零；连续 8.0 s → `failed`
4. protected（tilt/障碍/桥未就绪）→ 零
5. 无可用 SCAN 路径 → 最多每 3 s 重发一次 plan 请求
6. 航向门要求"不前进" → 平移零、允许偏航
7. 障碍守卫（held 时 0.05 s / 平时 0.10 s 周期）→ `obstacle_hold` + 零；释放需 1.0 s 无碍 **且** `ClearGuardWindow`（10 个新鲜样本、跨度 ≥1 s、间隔 ≤0.3 s）
8. 路径耗尽 / 低命令无几何进展 → 零 + 重规划

### 3.5 命令与安全门禁（14 层，按顺序）

一条命令要穿过全部这些层才会变成关节电流。读日志时"命令为什么是 0"必须逐层排查。

| # | 门 | 位置 | 失败动作 |
| --- | --- | --- | --- |
| 1 | 源新鲜度/过期 | `base_node.py:1096-1113` | 零；8 s 后 `failed` |
| 2 | 倾覆/桥安全 | `ros_node.py:327-397` | 零；严重时 `failed` |
| 3 | 控制时钟保持（重复/倒退时钟） | `control/safety/clock_hold.py`，`pid_navigation.py:3813-3818` | 在 PI/航向/几何之前精确零 |
| 4 | mission46 保持 | `mission/mission46_guard.py`，`pid_navigation.py:3804-3830` | 零速度请求，状态仍 `running` |
| 5 | 路径准入 | `planning/admission/path_admission.py:191`、`R85PhysicalPathGate` | 拒绝候选、保持 + 重规划 |
| 6 | 跟踪限幅 | `control/tracking/control_core.py:215-229`、`pid_core.py:11-26` | **静默裁剪**（不报错） |
| 7 | 航向门 | `control/parking/transition_gate.py:1313-1565` | 平移零 |
| 8 | 障碍保持 | `base_node.py:1376-1425` | 零 + 证据记录 |
| 9 | cascade native guard | `control/tracking/cascade_core.py:798+` | 零，`mode` 字段解释原因 |
| 10 | 命令文件门 | `robot/unitree/bridge.py:343-616`（15 类否决码在 `control/safety/gate_reasons.py:17-36`） | `healthy=false`，文件里写零 |
| 11 | 执行器门 | `control/executor_command.py:50-212` | 零 + `rejected=True`（**从不缓存非零命令**） |
| 12 | benchmark 门 | `navigation/robogauge/commands.py:9-32` | 抛错 / 零（仅 RoboGauge） |
| 13 | 策略物理门 | `control/safety/simulation_gate.py:37-119` | fault damping / stop |
| 14 | 停车零契约 | `control/parking/parking_zero_contract.py:122-286` | 拒绝该帧 → 零 |

**同一组 (vx, vy, wz) 限值在不同层不一样**（读任何单个文件都会得到错误结论）：

| 层 | vx | vy | wz |
| --- | --- | --- | --- |
| `configs/control/settings.json` `cascade.command_limits`（控制器权威值） | 0.6 | 0.2 | 0.6 |
| `bridge.py:50` 无 profile 时的 fallback | 0.3 | 0.2 | 0.3 |
| `student.CorrectionOnlyPID`（策略侧） | 0.3 | 0.2 | 0.3 |
| `navigation/robogauge/commands.py:26`（benchmark 门） | 0.6 | 0.2 | 0.3 |
| `base_node` 发布前加速限（`max_speed`，可被 `profile["max_speed_mps"]` 覆盖） | 0.12 默认 | | |

### 3.6 三种策略/观测接口（绝对不要混用）

|  | **Student（现役推理）** | **Teacher 247D（仅诊断）** | **RoboGauge 45D（另一套部署）** |
| --- | --- | --- | --- |
| 入口 | `student.exact_student45(state64, command3, prev_action12)`（`robot/unitree/policy/student.py:26-61`） | `native/observation.build_observation(...)` | `robogauge/contract.observation(...)`（`:36-63`） |
| 观测 | float32[45]：gyro×0.2 + 投影重力 + 原始 command + (q−q0) + qd×0.05 + 上一动作 | float32[247]：含 187 条垂直 SDF 射线 | float32[45]：gyro×0.25、command×[0.5,1,1]、关节按**官方顺序**重排 |
| 关节顺序 | native：**FR, FL, RR, RL** | 同 native | native → 官方顺序映射（`joint_indices()`） |
| 谁调用 | `robot/unitree/policy/worker.py`（actor 进程） | 同一 worker 顺带构造，仅写诊断，`teacher247_used_for_student_inference=False` | `simulation/gazebo/robogauge/native_loop.py` + `navigation/robogauge/benchmark.py` |
| 备注 | 无裁剪/无噪声；COM 速度 PID 在外层 `CorrectionOnlyPID` | 射线是静态 SDF，不是在线估计 | 45D **不是** 247D 的裁剪 |

机器人本体动作契约：`q_target = q_default + 0.25 × raw_action`，50 Hz（物理 0.005 s × decimation 4），DCMotor kp 25 / kd 0.5，effort 23.5（`robot/unitree/policy/native/contract.json`）。**训练不在本仓库**（`RL_for_unitree` 外部工程），这里只有推理与部署。

### 3.7 配置分层：prepare 冻结 vs 运行时重读

这是最容易误判的一节。`prepare()` 会把配置**复制/派生**进 run 目录，但并非所有消费者都读冻结副本：

| 配置 | 冻结方式 | 谁读冻结副本 | 谁仍读原配置 |
| --- | --- | --- | --- |
| `configs/localization_mapping/fastlivo.yaml` | 派生 → `run/navigation_fastlivo.yaml` | SLAM 二进制（`--params-file`） | — |
| `camera.yaml` | 复制 → `run/navigation_camera.yaml` | SLAM、parameter_blackboard | — |
| SCAN 参数 | 生成 → `run/v34_scan_overrides.yaml` | `scan_supervisor --params` | — |
| route | 抽取 → `run/global_route.json` | `mission_runtime` | — |
| mission 目标 | 写进 `run/runtime_context.json` | `mission_runtime` | — |
| `student_runtime` | 写进 profile | — | **actor 运行时 `read()` 重读 `configs/`**（`student.py:334`） |
| `runtime_paths` | — | — | `path()` 每次实时解析 |
| `features` | 注入环境变量 | 各子进程 `os.environ` | — |

**结论**：改 `configs/control/settings.json` 里的 `student_runtime` 会影响一个已准备好的 actor；改 `configs/launch/settings.json` 的 `mission` 不会。配置文件里没有任何标记说明这一点。

`prepare()` 还有 4 处**静默覆盖**（配置里写了但会被改掉）：`experiment`、`cascade.daily_navigation`、`r85_auto_reference_contract.reference_file`、`engineering_recording.*`。

### 3.8 时间域与来源契约

四个时间基准：

| 基准 | 产生者 | 用在哪 | 误读风险 |
| --- | --- | --- | --- |
| **ROS/sim 时间** | Gazebo `/clock`；所有节点 `use_sim_time=true` | 消息 header、sim 时间 freshness、日志时间戳 | 与墙钟不等；`--duration` 是墙钟，`duration_s` 是 sim 秒 |
| **steady/wall 单调时间** | `Clock(ClockType.STEADY_TIME)` 定时器、`time.monotonic_ns()` | 大多数看门狗、bridge 的 wall receipt、`*_wall_s` 字段 | 与 ROS 时间不同域，**不能直接相减** |
| **native 物理时钟** | 64 float 状态里的 `physics_clock_ns` | 策略证据、执行器 ACK | 与 ROS sim 时间近似但独立 |
| **source/received 对** | 消息原始 stamp + 接收墙钟 | 过期判断、回放对齐 | "stale" 在不同层含义不同（0.3 s 命令源 / 0.8 s pose / 8 s 故障） |

来源契约（`common/state_contract.py`）：

- `StateOrigin`：`simulation_truth` / `slam_estimate` / `real_sensor`（仅保留枚举，无实现）。
- `validate_state_provenance()`：必须给出**精确**的 source、clock_domain、run_id；`NAVIGATION` 用途**只接受 SLAM**；`SIMULATION_SAFETY` **只接受**仿真真值；native64 帧不允许被重标为其它来源。
- 运行目录里到处是 `navigation_ground_truth_used=false`、`pose_producer="fast_livo2_async_backend"`、`robot_truth_pose_used=false` 这类字段——**它们是程序内数据契约，不是密码学认证**。
- 历史命名仍带 `truth`（如 `/demo/teacher/*`、`truth_worker.py`、`truth_navigation_command.json`）。**允许导航的判据是实际生产者+标志+时钟，不是名字**。

### 3.9 关停与常见失败点

- 组件异常退出 → 整个 launch 树关停（`owned()` 写 `component_exit.jsonl` + `EmitEvent(Shutdown)`）。
- `start.py` 只对"起始 tick 仍匹配"的直接子进程发 SIGINT（避免 PID 复用误杀），共享 15 s，超时再对进程组 SIGTERM + 5 s。
- **prepare 或环境构造阶段失败不会写 `exit.json`**：`finally` 里 `if children:` 守卫导致只有 traceback。新人会以为"还在启动"。
- `actor_ready` 屏障失败只体现在 launch 日志（它不是受管组件）。
- 锁被占用 → `BlockingIOError`，没有友好提示（不知道是哪个进程在跑）。

---

## 4. 路径 B：Building（SLAM-only）

- 入口 `launch/building.py:496`，默认 domain 226，自带 `prepare()`（`building.py:25`）与 `Popen` 管理，**不是 `demo.launch.py` 的一部分**。
- 在读取任何资产前就拒绝非 SLAM 模式（`building.py:44-46`），这是有意的来源门。
- 流程：route JSON（`assets/building/routes/<case>.json`）→ `PCT_route.json` → `building_navigation.py` 节点（读 `PCT_route.json`、SCAN spline、自带 PD/停车）→ 关节。
- 会启动 `evaluation/process_resources.py` 采样资源（`building.py:271`），是唯一带资源采样的路径。
- 已知校验缺口（历史审计已记录，本轮未改）：`--duration` 缺 `math.isfinite`；`--check` 提前 return 跳过 domain/duration/label 校验。
- 7 个 case：`stairs_short`、`single_floor_turn`、`combined_floor1_inside`、`multi_floor2`、`single_floor_loop`、`second_stair_short`、`third_stair_short`。

## 5. 路径 C：RoboGauge benchmark

- 入口 `navigation/robogauge/benchmark.py:381`（文件名叫 benchmark，实际是**进程编排器**：`run()` 在 `:251-380`，`--check` 只做文件/哈希预检）。
- 拉起 actor / sensor bridge / FAST-LIVO2 / backend / localization / `building_navigation` / SCAN / Gazebo，写 `processes.json`，12 s/5 s SIGTERM/SIGKILL 关停。
- 45D 观测 + 官方关节顺序 + 专用 actuator 插件（`install/robogauge/lib/librobogauge_actuator.so`）；指标分析在 `analyze.py`。
- 与 Native 路径的限值/契约不同（见 §3.5 表、§3.6 表），**不能把 RoboGauge 的 45D 当成 Student 的 exact45**。

## 6. 路径 D：DynaNav / Isaac（当前实验主线）

三个**独立终端**（没有统一 launcher，没有进程管理）：

| 顺序 | 进程 | 入口 | 作用 |
| --- | --- | --- | --- |
| 1 | 行人推理 worker（可选） | `scripts/run_dynanav_person_worker.py` | 持久加载 Mask R-CNN，写 `state.json`（含模型哈希）与本地 IPC channel |
| 2 | 估计器 observer（可选） | `scripts/run_dynanav_lio.sh --mode lio\|livo` | 隔离 namespace 下的 ONLY_LIO / LIVO；写 `launch.json` 收据（含输入话题、post-VIO 输出、domain、二进制/配置哈希） |
| 3 | capture / 实验 | `scripts/run_dynanav_experiment.py` → `experiment.main` → `runpy` 在**本进程内**执行 `capture_go2_rgbd.py` | Isaac 场景 + 物理循环 + 传感器发布 + `RouteTracker`/`SlamRouteTracker` 控制 + 产物落盘 |

- 前置：Isaac（原机 `/home/hyh001/isaacsim/python.sh`，6.0.1 RC，公开可获取性未核实）、`PROJECT_DATA_ROOT`、数据卷 `build_cache/` 下的 native 构建前缀（`run_dynanav_lio.sh` 默认用固定目录 `build_cache/dynanav_post_vio_20261109T052245Z`，可用 `--binary-prefix` 指定其它前缀）、模型权重与 SHA。
- 节奏（DynaNav graph）：物理步 0.005 s；IMU 按 `experiment.py --imu-hz` 发布，**默认 200 Hz**（`choices=(100,200)`，`imu_publish_stride = 200 // args.imu_hz`：200→stride 1、100→stride 2）；直接跑 `capture_go2_rgbd.py` 且无 experiment profile 时才落 legacy `stride=2`（≈100 Hz）。最近实验实际用 200 Hz（报告记录 `--imu-hz 200`、SDK IMU 200 Hz）。RGB-D/点云 10 Hz（每 20 tick）；所有时间戳 = IMU source time `round(it*1e9)`（并断言与物理时间在 float32 半 ULP 内一致）。learned 推理是**同步阻塞**的：**代码设计**上同步等待期间物理暂停，已有运行报告记录了物理步/墙钟证据（45 s 物理 → loop wall 121.6 s），本轮未重测。
- 产物：`$PROJECT_DATA_ROOT/embodied-navigation/experiments/<run>/`，含 `capture/result.json`、`experiment.json`、`source_manifest.json`、`launch.json`、RGB 归档（`--archive-rgb`，约 0.75 MiB/帧）、可选 bag。
- 与主路径的关系：**共享代码、不共享进程图和话题**（`/dynanav/*` vs `/demo/*`）。rejoin 点是 `perception/depth.py`、`localization_mapping/semantic_mapping/`、`robot/unitree/robogauge/`、`control/safety/simulation_gate.py`。
- ROS 耦合度不同：**person worker 通过文件 IPC 通道交互，不 import rclpy、不受 `ROS_DOMAIN_ID` 影响**；observer 与 capture 则必须处于同一 ROS domain（报告里三终端统一 export 是操作便利，不是 worker 的依赖）。
- **当前状态：用户已要求暂停**（这是用户决定，不是能力结论）；只做已有结果分析，不再启动仿真、推理或测试。历史已有成功运行记录（见 §2 三层表），最新报告见 `docs/experiments/dynanav-online-person-20261109.md`。

**DynaNav 的三种"语义来源"（容易混）**：

| 模式 | 位姿来源 | mask/标签来源 | 是否 oracle-free |
| --- | --- | --- | --- |
| 默认 oracle 诊断 | GT（`sim_oracle`） | 模拟器实例 ID | 否（明确标注） |
| LIO/LIVO 旁路 | 估计器（`lio_estimate` / `livo_post_vio`） | 仍是 oracle mask | 否（旧 landing 上游仍用 oracle static cloud） |
| 显式在线 learned | exact 实测 post-VIO | 独立 RGB worker + 几何候选 | 部分（历史路线参考仍带 oracle 前处理来源） |

---

## 7. 感知与定位建图

### 7.1 两个 `sensors` 包（先分清）

| 包 | 性质 | 现状 |
| --- | --- | --- |
| `src/embodied_navigation/sensors/` | 跨层轻量契约：`SensorFrame`、`PoseEstimate`、`VelocityCommand` 三个 frozen dataclass | **当前源码无任何导入者**（README 自己写了），是预留接口 |
| `src/embodied_navigation/perception/sensors/` | ROS 传感流适配：中继、自车回波过滤、采样、健康门、反馈邮箱 | 主路径的 localization 组里 4 个进程来自这里 |

新人在 `import` 报错时先确认自己指哪个 `sensors`。

### 7.2 传感流（Native 路径）

```
Gazebo 相机/IMU ──sensor_bridge──► /demo/camera, /demo/teacher/imu...
                                   ├─ sensor_relay ──► /demo/teacher/slam/image, /demo/teacher/slam/imu（源戳）
                                   └─ self_filter ──► /demo/slam/lidar_filtered（去自车点）
                                                        │
                             fastlivo_mapping ◄──────────┘ + 相机参数服务
                                   │ /aft_mapped_to_init, /cloud_registered_full
                                   ▼
                             odom_adapter ──► /demo/slam/raw_body_odom / raw_lidar_odom
                                   ▼
                             localization_runtime（known-scene 辅助）──► /demo/slam/body_odom 等
```

`prepare()` 会把 SLAM 的输入话题**固定**成 `/demo/teacher/slam/image`、`/demo/slam/lidar_filtered`、`/demo/teacher/slam/imu`（`prepare.py:85-89`）。名字里的 `teacher` 是历史前缀，不代表加载了 Teacher 策略。

### 7.3 深度与行人（在线 learned 分支）

**先纠正一个最容易踩的命名**：`perception/person_worker.py` **不是 worker**——它明确"不 import torch/SDK/ROS"，只做 IPC 客户端 + 触发策略 + 二维证据分档；**真正的模型加载与推理在 `scripts/run_dynanav_person_worker.py:56-62,271`（`model_load_count=1`）**。`perception/README.md:12` 把"模型一次加载"写在 `person_worker.py` 名下，是错的（见 §12 D19）。

**真实调用顺序**（`capture_go2_rgbd.py` 的物理循环里每 20 tick 一次，`:1302-1303`）：

| # | 阶段 | 输入 → 输出 | 关键约束 |
| --- | --- | --- | --- |
| 1 | `perception/depth.py:81` `depth_cloud_packet` | 深度(m) + 内参/安装 → `points (N,3)m` + 原始像素索引 + keep mask + self mask | 深度 0.05–20 m；点预算 ≤8000；**原点在 IMU、坐标轴是 base**（两者差一个 COM 偏置） |
| 2 | `learned_mapping.py:99` `process()` | 触发策略（`initial/max_interval/low_confidence_confirmation/motion_new_view`）→ 可选**同步** IPC 推理 | 运动量必须来自 `lio_estimate/livo_estimate/livo_post_vio/raw_imu`，GT 字符串被拒 |
| 3 | IPC 目录 | 请求 NPZ（只允许 9 个字段）→ worker → 结果 NPZ | 客户端 3 s 超时；结果做 exact 身份复核（uuid+source_ns+frame） |
| 4 | `person_tiling.py` | grid=1 只跑 full；grid=2 加 4 个象限 crop 后按 IoU NMS 合回 | 在线仍用 tile1（tile2 是已记录的反例） |
| 5 | `learned_person_mapping.py:41` | mask/score/像素 → 每点状态 `{0=UNKNOWN, 2=DYNAMIC}` + 单帧实例 ID | 0.1 分阈；一个点只被一个 mask 拥有 |
| 6 | `person_geometry_tracking.py:361` | 点 + 像素 + mask + `T_map_sensor`（只接受 SLAM）→ 内部 track ID、quarantine AABB、2D motion | 单函数 461 行；姿态年龄门 250 ms；无 GT 回退 |
| 7 | `temporal_geometry.py:147` | 重复观测投票 → 准入的原始点索引 + ledger | 3 distinct 帧 / ≥0.5 s，retention 300 s / 200k voxel / 8 票 |
| 8 | `semantic_mapping` `LayeredMap.integrate` | → STATIC/DYNAMIC/UNKNOWN 分层 + 对象历史 | 缺 mask 时整帧按 UNKNOWN 隔离，但导航云仍是完整点云 |
| 9 | `learned_mapping.py:397` `finish()` | `static_map.npz`、`static_support_ledger.json`、`scene_map.json`、`person_query.json`、`person_context.txt`、`receipt.json`、`keyframes/*.npz` | 显式 `semantic_oracle=False`、`pose_oracle=False`、`complete_full_scene=False` |

- 位姿来自 `/dynanav/<ns>/post_vio_odometry`（精确帧匹配，`learned_mapping.offer_pose`），**不是** GT；learned 模式要求 `frame_id="dynanav_lio_map"`、`child_frame_id="go2_imu"`。
- `person_mask_propagation.py` 是暂停前新增的**未接入**光流 helper：只有 5 项注入 flow 的纯测试，没有 importer（除测试），**不要当成在线能力**。
- **编排缺口（工程可读性，不是"无法复现"）**：最新报告已给出三条仓库入口命令及先后顺序（worker ready → LIVO observer + `launch.json` 收据 → capture，含 `--pose-launch-receipt` 要求），**基础配置可据此复跑**；但把 worker/observer/recorder/resource-monitor/capture 五进程统一拉起、统一收集退出码与录包的**精确监督流程**只存在于数据卷的 `supervisor.py`（其 `supervision.json` 记录五进程退出 0）。`experiment.py:350-402` 只**校验** worker `state.json` 与 observer `launch.json` 已存在，不启动它们。缺的是一个可复用的监督层。
- 实测结论（来自实验报告，不是本文验证）：45 s 片段漏过 person 点 36.26%（主要来自未推理帧），历史 geometry 准入 344 个 person 点；"最终 ledger 里没有 person"**不能**抹掉历史污染。

### 7.4 SLAM 接入：前端 / 后端 / 定位 runtime

- **前端**：`third_party/fast_livo2` 是真正的算法与 ROS 包；`localization_mapping/frontend/fast_livo2_ros/` 只是接入资料/launch/脚本，`third_party` 里的补丁（`0004` 生命周期、`0005` post-VIO 输出）是本仓库自有。
- **后端**：`localization_mapping/backend/` 做注册、关键帧历史、回环验证与 SE(3) 图优化；队列/历史/地图写运行目录，不依赖 ROS。**注意：主路径（`slam.launch.py`）里没有这个进程**——它只由 `launch/building.py:326` 与 `navigation/robogauge/benchmark.py:100` 启动。`feedback_state` 里 `pose_producer="fast_livo2_async_backend"` 是**标签**；仓库内无法确认它指的是 C++ 内部异步后端还是这个 Python backend（B 调查列为未解项），不要凭标签推断进程存在。
- **定位反馈来源**：`feedback_state.accepted_slam_feedback()` 构造 canonical state（`frame_id="camera_init"`、`pose_producer="fast_livo2_async_backend"`、`navigation_ground_truth_used=False`）。它**不是** truth→SLAM 转换器：调用方必须已经拥有已验证的 SLAM commit。
- **同名话题陷阱**：`odom_adapter` 默认发布 `/demo/slam/{body,lidar}_odom`，但生产 launch 把它 remap 成 `/demo/slam/raw_*`；真正的 corrected 同名话题由 `localization_runtime` 发出。**同一个话题名在不同 launch 图里可能是 raw 也可能是 corrected**（安全相关，见 §12 D20）。
- **已知场景辅助**：`known_scene_*.py`（参考构建、匹配、输入、保持门）在 `localization_runtime` 里做实时校正；`prepare()` 会生成 `known_scene_reference.json`。

### 7.5 语义分层地图 / 对象历史 / 查询

`localization_mapping/semantic_mapping/`：纯 CPU、无 ROS/模拟器/检测器依赖。

- `LayeredMap.integrate()` 永远返回**完整**导航点云（缺 mask 时按 UNKNOWN 隔离），STATIC/DYNAMIC/UNKNOWN 分开；`static_points()` 才是持久静态图。
- `STATIC=1, DYNAMIC=2, UNKNOWN=0`；含 dynamic 点或 person 标签的组整体排除。
- 对象历史保留原始 instance ID、epoch/frame/calibration、source stamp、位姿与 mask 来源；`revise_mask()` 可撤销历史贡献；`update_anchor()` 需递增 pose revision。
- 只读查询入口：`scripts/query_scene_map.py <scene_map.json> --label person --near ... --radius ...`（默认输出确定性 JSON，`--context` 输出结构化文本；**不给导航授权**）。
- 边界：这是"归档后过滤"，**不证明 FAST-LIVO 内部几何/视觉图已排除动态物体**。

---

## 8. 规划与路径交付

**产物链**

| 产物 | 生产者 | 消费者 | 坐标系/格式 |
| --- | --- | --- | --- |
| `assets/geometry/route.json` | 仓库数据（不在 Git） | `prepare.py:133` | body/ground 点、`T_navigation_world` |
| `run/global_route.json` | `prepare.py` | `mission_runtime` | 同上子集，校验 `(N,3)` 有限、≥2 点 |
| `run/v34_scan_overrides.yaml` | `prepare.py` ← `planning/scan_runtime_parameters.py` | `scan_supervisor --params` | horizon **强制 0.65 m**（不等就报错）、grid 0.08 m、滑窗 10×10×5 m、`frame_id=camera_init` |
| `run/navigation_anchor.json` / `navigation_request.json` | `mission_runtime` | 控制器、admission | anchor schema 1；request schema 2（一个 disc_prism 目标） |
| `run/global_reference.json` | `mission_runtime` | `planning/reference/automatic_reference_runtime.py` | `r85_pct_scan_global_reference/v1` |
| SCAN 轨迹 | `scan_planner_node` → 话题 | `base_node.on_spline` / `TrajectoryAssociation` | `scan_planner_msgs/Bspline` |
| `PCT_route.json`（仅 Building） | `planning/pct/generate_routes.py` | `building_navigation.py` | `native_PCT_Astar_physical_reference/v1` |

**third_party 边界**：仓库**不 import** SCAN/PCT 的算法，只 import 生成的 `scan_planner_msgs` 消息类型，并用二进制/子进程交互。SCAN 的子话题是相对名（`planning/bspline` 等），由 supervisor 统一 remap 到 `/demo/navigation/*`。`fsm.navi_mode=3` 让 SCAN 自己的控制器不参与闭环。

**准入（admission）**：两条分支，读文档时容易混——
- `FrozenFence`（`path_admission.py:92-182`）要求"原始 46 区域"anchor 字段，`mission_runtime` **写不出**这些字段 ⇒ 该 fallback 分支在本检出不可用。
- 默认 profile 启用 `r85_auto_reference_contract`，因此实际走 `AutomaticRouteFence` + `R85PhysicalPathGate`（`pid_navigation.py:1097-1110`）。

**PCT 的坑**：`planning/pct/generate_routes.py` 是**模块级脚本，import 即执行**（会调用外部 CLI）。不要 import 它。

---

## 9. 数据与产物布局

| 位置 | 内容 | 说明 |
| --- | --- | --- |
| `runs/<UTC>_<label>_<hex>/` | 一次运行的**全部**输入/输出：`world.sdf`、`sensor_contract.json`、`navigation_fastlivo.yaml`、`navigation_camera.yaml`、`v34_scan_overrides.yaml`、`global_route.json`、`known_scene_reference.json`、`runtime_context.json`、`processes.json`、各 `*.log`、`component_exit.jsonl`、`exit.json`、`navigation_command.json`、`navigation_status.json`、`navigation_anchor.json`… | `runs/` 是指向外部数据卷的软链；Git 忽略 |
| `assets/` `models/` | 场景/网格/URDF/SDF、Student 权重 | 软链到 `PROJECT_DATA_ROOT/embodied-navigation/runtime_assets/`；`scripts/configure_data.py` 建立 |
| `install/` | 构建产物（ROS 前缀、SLAM/SCAN 二进制、actuator/DDS/writer 库），**由各环境自行构建**：镜像里由 `docker/build_owned_native.py` / `build_native.py` 生成 | Git 忽略；**缺文件≠缺源码**，但"有几个文件"也≠ready；本次执行环境未构建 |
| `$PROJECT_DATA_ROOT/embodied-navigation/experiments/<run>/` | DynaNav 每次实验：`capture/`、`livo/observer/`、`person-worker/`、`*.json` 收据、RGB/bag | 必须位于数据根目录下（代码强制） |
| `dynanav/` `history/` `perception_history/` | 迁移前历史副本（327MB / 190MB / 35MB） | Git 与 Docker **均排除**；不是开发入口 |

---

## 10. 测试地图（"改 X 跑 Y"）

统计（静态 AST 计数，**本次未运行**）：`tests/unit` 40 文件 / 289 个 `test*` 方法（已提交 31 文件 217 个 + 未提交 9 文件 72 个）；`tests/regression` 16 文件 / 86 个；`tests/native/dynanav_lio` 2 个 CTest（需已构建的 ROS 版 core，非离线）。

| 改动点 | 要跑的测试 |
| --- | --- |
| `perception/depth.py` | `test_depth_packet.py`、`test_perception.py` |
| `perception/person_*.py`、`temporal_geometry.py` | `test_person_segmentation.py`、`test_person_worker.py`、`test_person_tiling.py`、`test_person_geometry_tracking.py`、`test_person_mask_propagation.py`、`test_temporal_geometry.py`、`test_learned_person_mapping.py` |
| `simulation/dynanav/*` | `test_dynanav_*.py`（13 个）+ `test_learned_person_mapping.py` |
| `localization_mapping/backend/*` | `test_backend_flow.py`、`test_backend_registration.py`、`test_backend_auto_loops.py` |
| `localization_mapping/semantic_mapping/*` | `test_layered_mapping.py` |
| `control/safety/*`、`simulation/safety_state.py` | `test_simulation_safety.py`、`test_native_safety_integration.py`（需 Torch）、`test_navigation_safety.py`（部分需私有地图） |
| `control/relative_pid.py`、`control/collision` | `test_control.py` |
| `control/tracking/*` | `test_planned_step_filter.py`、`test_route_progress.py`、`test_terrain_speed.py` |
| `control/parking/*` | `test_parking_source_contract.py` |
| `navigation/task.py` | `test_navigation.py` |
| `navigation/robogauge/*` | `test_robogauge_contract.py`、`_evidence`、`_benchmark`、`_route_tracker`、`test_dynanav_slam_route.py` |
| `launch/*` | `test_launch_cli.py`、`test_launch_exit_status.py`、`test_building_entry_closure.py`、`test_ros_shutdown_lifecycle.py` |
| `common/config.py` / `paths.py` | `test_data_paths.py`、`test_launch_cli.py` |
| `evaluation/trajectory.py` | `test_metrics.py` |
| `planning/admission` | `test_navigation_safety.py`（部分跳过） |

**跳过项**：缺 Torch 5 项（`test_native_safety_integration` 4 + `test_robogauge_native_loop` 1）；缺私有测量地图 2 项（`test_navigation_safety`）；缺 USD/pxr 运行时 3 项（`test_dynanav_pedestrian`）。这与文档里"5 项 Torch 跳过""7 项因 Torch 或私有地图跳过"是吻合的——**这两处文档是准确的**。

**脆弱点**：7 个测试文件直接 `ast.parse` / `read_text` / `exec` 源码做断言（`test_building_entry_closure.py`、`test_production_slam_mode.py`、`test_ros_shutdown_lifecycle.py`、`test_dynanav_learned_wiring.py`、`test_dynanav_odometry_source.py`、`test_dynanav_post_vio_output.py`、`test_native_safety_integration.py`）。它们不是行为测试，**重命名/拆函数会误伤**，而没有文档提示这一点（`docs/architecture.md` 完全没提）。

---

## 11. 可读性问题清单

分三级：**A = 直接阻断理解**（不查清就无法正确改代码），**B = 容易误读**（会让人得出错误结论），**C = 噪声**（不影响正确性但增加成本）。每条给"位置 / 为什么难读 / 建议"。

### A 级：直接阻断理解

**A-1 三套导航实现在抢"我是谁"，而且文档排序是错的。**
`navigation/base_node.py:102 class Navigation`、`navigation/building_navigation.py:68 class Navigation`、`pid_navigation.py:4480 class IntegratedNavigation`——**两个文件里各有一个类字面就叫 `Navigation`**。`navigation/README.md:13-18` 列了五个文件，却没给"入口 → 节点类 → 进程角色 → 是否在默认 launch 里"的映射。
→ 新人打开 `navigation/` 看到 4702 行的 `pid_navigation.py`、1611 行的 `base_node.py`、341 行的 `task.py`、351 行的 `mission/state_machine.py` 都叫"导航"，无法判断哪个在跑。
**建议**：在 `navigation/README.md` 顶部加一张四列表（可直接用本文 §0/§3.4 的结论）。

**A-2 `navigation/task.py` 的状态机不在主路径上，但三处文档把它当主路径。**
根 `README.md:15`（"task.py 是...的真实...状态机"）、`navigation/README.md:16`（"现有 shuttle 任务状态机"）、`control/README.md:8`（"任务级状态机在 `../navigation/task.py`"）。
事实：`task.py` 只被 `localization_mapping/relative_odometry.py:8`（取 `Observation`）和 `tests/unit/test_navigation.py` 引用；它的真实消费者 `relative_slam_node.py` 是一个**独立 console script**（`pyproject.toml:17` `relative-slam-navigation`），不在 `demo.launch.py` 里。
**建议**：文档统一表述为"`task.py` = 独立 relative-SLAM shuttle 入口的状态机；默认主路径的状态机是 `base_node.control()`"。

**A-3 `navigation/mission/state_machine.py` 是 351 行的死代码，README 说它是活的。**
`mission/README.md:7` 把它列为"与传输无关的任务迁移"层、`:20` 说"行为通过 pid_navigation 的运行链路使用"。
事实：唯一 importer 是 `mission/continuous_route.py:19`，而 `ContinuousMission`（`:414`）**在整个仓库没有任何构造点**；`continuous_route_contract.enabled=false`（`configs/planning/settings.json:2-5`）；实际运行的 `mission` 角色是 `mission_runtime.py` 里自己定义的 `class Mission(Node)`（`:40`），两者同名不同物。
**建议**：要么删/标 legacy，要么在 README 里写"未实例化；live 的 mission 角色是 `mission_runtime`"。

**A-4 两个 `HeadingGate` 用同一套 phase 词，语义不同，且被静默替换。**
`control/tracking/control_core.py:253`（`drive/pre_turn/settle`，硬编码 1.0 s settle）与 `control/parking/transition_gate.py:57`（`pre_turn/align/settle`，实测停止资格、受控刹车、交接 deadline）。`base_node.py:219` 建第一个，`ros_node.py:152` **静默替换**成第二个，`pid_navigation.py:2830` 还会再换一个深拷贝变体。
后果：`publish_status` 里 `alignment_phase` 是同一个字符串，**日志分不出是哪台机器**。
**建议**：类名改成 `SimpleHeadingGate` / `TeacherHeadingGate`，并在 status 里记录类名。

**A-5 `pid_navigation.py` 4702 行，混合 5 类职责。**
ROS 接线（`:1036` 类、`:4463 _main`）、准入事务（`:2013`）、停车 owner（`:4038`）、证据/JSON 写入（模块级 70+ 个 `_r47_*`/`_r56_*`/`_r67_*` 探针 shim，`:114-935`）、进程 bootstrap（`:4696 main`）。
→ 想找"命令是怎么算出来的"，得先跳过近千行探针桩。
**建议**：至少把 admission / parking / evidence 拆出去；模块 docstring 写清楚入口顺序。

**A-6 命令链的关键一跳是文件，不是话题，而且有两套命令文件。**
`bridge` 写 `navigation_command.json`，`policy/worker.py:257` 读它；同时 Building/诊断路径写 `truth_navigation_command.json`（`building_navigation.py:623`、`simulation/truth_worker.py:77`）。新人沿着 `/demo/cmd_vel` 往下找会发现"话题到 bridge 就断了"。
**建议**：在 `docs/architecture.md` 的命令链图里明确标"文件 IPC + socket"，并把两个命令文件并列说明。

**A-7 `features` 是 45 个开关（其中 39 个叫 `R编号`）的环境变量，没有注册表。**
`configs/launch/settings.json:9-55` 注入，消费方散落各处 `os.environ.get("R88_ASYNC_PHYSICAL")` 之类（`planning/admission/async_admission.py`、`navigation/ros_node.py:191`、`perception/sensors/ready_commit.py:11`、`control/parking/*.py`）。`src/` 里只有 **17 个**读取点，**多个 flag 没有任何读取者**（如 `R90_SPINNER_POSITIVE_YIELD`、`R97` 之外的多数探针）。
→ 名字像代码标识符，但没有任何地方解释 `R88`/`R110`/`R124` 是什么；唯一的解释在 62KB 的历史审计 `docs/review/validation-and-threshold-audit.md` 里，且行号已漂移。
**建议**：一张 `flag → 模块 → 默认 → 生产值` 表（放 `common/` 或 docs），或让 flag 名带模块前缀。

**A-8 同一组限值在 5 层不一样（0.3/0.6 混用）。**
见 §3.5 表。`robot/unitree/README.md:7` 把 `LIMITS=(.3,.2,.3)` 写成契约，但那只是 `bridge.py:50` 的 fallback；有 profile 时是 `[0.6,0.2,0.6]`，策略层又压到 `[0.3,0.2,0.3]`，benchmark 门是 `[0.6,0.2,0.3]`。
**建议**：把 (vx,vy,wz) 三元组收进 `common/integration_limits.py` 风格的常量（该文件已经存在），每个消费者注明"我执行的是哪一层的哪一档"。

**A-9 `capture_go2_rgbd.py` 1778 行、**import 时有副作用**、没有 `main()`。**
`:23-24` 在模块导入时 `Path(sys.argv[1]).mkdir(...)`；`:25-36` 读 experiment JSON；整个文件是一个顶层 `try/except`。
→ 任何 import（pytest 收集、复用某个常量）要么 `IndexError` 要么在磁盘上建目录。
**建议**：包成 `run(out, scene)` + `if __name__ == "__main__":`。

**A-10 "Teacher" 是历史命名，但仍是线协议/契约名。**
`install/actuators/lib/libteacher_actuator.so`、`TEACHER_SOCKET`、`TEACHER_ACTUATOR_LOG`、话题 `/demo/teacher/slam/image`、`/demo/teacher/navigation/anchor`、`robot/unitree/policy/native/worker.py` 的 `teacher_sim::TeacherActuator`。实际加载的是 **Student** 策略（`student.py:334-350`）。
→ 新人会以为有 Teacher 策略在跑。
**建议**：在 `simulation/actuators/README.md` 加一张命名表（"Teacher = 历史插件 ABI，不是被加载的策略"），并从 `launch/main.py` 的 docstring 链过去。

**A-11 `perception/person_worker.py` 不是 worker，模型在 `scripts/` 里。**
`person_worker.py:1-4` 明确"不 import torch/SDK/ROS"，它做的是 IPC 客户端 + 触发策略 + 二维证据分档；真正的 Mask R-CNN 加载与推理在 `scripts/run_dynanav_person_worker.py:56-62,271`。
→ 想读"检测模型怎么跑"的人打开 perception 包，会找不到 torch，从而误判"感知还没实现"。
**建议**：重命名为 `person_worker_client.py`（或 `person_inference_ipc.py`），并在两个 README 首行写"谁拥有模型"。

**A-12 在线 learned 管线的"统一监督层"不在仓库里（复跑靠文档补齐）。**
最新报告已给出三终端命令与顺序（worker ready → observer 收据 → capture），**基础配置能照此复跑**；但统一拉起 worker/observer/recorder/resource-monitor/capture、采集退出码与录包的 `supervisor.py` 只在数据卷 run 目录（`…/dynanav-online-person-prefix-20261109T065302Z/supervisor.py`），`experiment.py:350-402` 只校验收据已存在。仓库里的 `scripts/run_dynanav_*.py` 是三个各管一段的入口。
→ 复跑者要在三个终端手工维持顺序、环境与收据一致，成本高且容易漏 `--pose-launch-receipt`；这是**编排/可读性缺口**，不是不可复现。
**建议**：把 `supervisor.py` 收敛为 `scripts/run_dynanav_online_person.py`（带 `--check` 与 `supervision.json`），报告中的命令作为它的 `--print-commands`。

### B 级：容易误读

**B-1 `mission/README.md:24` 说准入"未闭合"，但默认走的是另一条已闭合的 R85 分支。**
`FrozenFence` 确实没有 writer，但默认 profile 启用 `r85_auto_reference_contract`，实际走 `AutomaticRouteFence` + `R85PhysicalPathGate`（`pid_navigation.py:1097-1110`）。
**建议**：区分"fallback 分支（未闭合）"与"live 分支（R85）"。

**B-2 `duration` 在四处的含义不同。**
`--duration` = 墙钟秒（`main.py:122-127`）；`duration_s=800` = actor/policy 的 sim 秒（`policy/worker.py:322-324`）；`mission.timeout_sim_s=770`；`max_simulation_duration_s=800`（**无人读**）。
**建议**：把 CLI 改为 `--wall-timeout`，或在这四处旁列一张对照表。

**B-3 `terrain` / `spawn` 有 3~4 个来源，其中 2 个被覆盖、1 个无效。**
`expected_asset.spawn`（无人读）、`original_scenario.spawn`（dict，写进 scenario）、顶层 `spawn`（真正生效）；`simulation/world/prepare.py:152-160` 还硬编码 spawn 后被 `launch/prepare.py:66-74` 覆盖。`expected_asset.terrain="ramp_up"` 只影响这个被覆盖的 spawn，**改它不会改变世界几何**（只有 `step05/step10` 会加几何）。
**建议**：只保留一个 `spawn`；把 terrain 语义改成"影响世界生成"或明确文档化为"ramp 已静态存在于 `assets/world/three_floors.sdf`"。

**B-4 配置写 `1` 但启动器强制 `0`。**
`configs/launch/settings.json:32` `R107_SCAN_TRANSPORT_PROBE="1"`、`:54` `MATCH_QUERY_CAPTURE="1"`，`launch/main.py:73-82` 无条件改 `"0"`（这是"防仓库外实验污染生产"的防线，**不要删**）。但配置值会让人以为探针已开。
**建议**：把这两项在配置里改成 `"0"`，或加注释"launch 强制关闭"。

**B-5 `prepare()` 静默覆盖 4 个配置值。**
`experiment`、`cascade.daily_navigation`、`r85_auto_reference_contract.reference_file`（配置写 `r85_global_reference.json`，实际改成 `global_reference.json`）、`engineering_recording.*`。
→ 改配置的人以为生效，实际被丢掉。

**B-6 死配置让人误以为能调。**
`configs/control/settings.json:223 max_simulation_duration_s`；`configs/simulation/settings.json:3-8 expected_asset.spawn`、`:3269 dynamic_scene`；整文件 `configs/navigation/relative_shuttle.yaml`、`configs/control/relative_pid.yaml`、`configs/robot/unitree_usd_manifest.json`、`configs/runtime_assets_manifest.json`、`configs/simulation/dynanav/office.yaml` 均**无代码读取者**；`configs/robogauge/contract.json` 也没有读取者，但 `docs/robogauge/README.md:12` 说它是权威契约（真正权威是 `robot/unitree/robogauge/contract.py:11-20` 的硬编码）。
**建议**：删除或标"归档，代码不读"。

**B-7 `simulation/terrain/mission46.py:28-29` 指向不存在的路径。**
它硬编码 `tests/terrain_targets/{lower12,upper23}.json`，而仓库里只有 `configs/scenarios/terrain_targets/lower12.json`，且该文件**无任何代码读取**；`mission46_required=false`，provider 在正常启动中不会被构造。这是一个诚实的死分支，但 `configs/scenarios/README.md:12` 仍把它描述成"provider 信息"。

**B-8 `common/observation.py` 是一个 88 行的"全空端口"。**
约 40 个 no-op 方法名（`producer_begin`、`guard_window_begin`、`parking_protection`…）只作为历史 hook 位点保留；`api(name)` 恒返回同一个 `DisabledObservation`。
→ 新人会以为这些是活的诊断钩子。
**建议**：README 已说明，但建议在文件顶部加一行"这些方法名是历史 hook 位点，全部 no-op"。

**B-9 两个 `sensors` 包、两层"perception"，消费者完全不相交。**
`src/embodied_navigation/sensors/` 是**无导入者**的预留契约（README 自己写了）；`perception/sensors/` 是主路径 4 个进程。同时 `perception/` 里纯 NumPy 的模块（`depth`、`person_segmentation`、`learned_person_mapping`）**只被 DynaNav 的 capture/脚本导入**，而 `perception/sensors/` **只被生产导航/定位导入**——两者共享一个包名却服务两条互不相干的路径，README 把它们平铺成一个列表。
**建议**：拆成 `perception/offline/`（纯 NumPy）、`perception/online/`（worker/tiling/tracking）、`perception/ros_adapters/`（sensors），每个子 README 首行写"谁导入我"。

**B-10 两个 `sensors/` 之外，`worker.py` 也有两个。**
`robot/unitree/policy/worker.py`（actor 入口，读命令文件、跑 Student）与 `robot/unitree/policy/native/worker.py`（native64 socket 循环，16 float 回复）。文件名在 traceback 和日志里无法区分。

**B-11 `navigation/robogauge/benchmark.py` 不 benchmark 任何东西。**
它 `:251-380` 做进程编排，指标在 `analyze.py`；`--check` 只做文件/哈希预检。名字会让人以为它产出分数。

**B-12 `policy_selection.py` 的 "snapshot"/"archive" 并不快照。**
`binding()` 里 `*_snapshot = str(origin)` 只是路径；`archive_relative()` 指向树里不存在的 `HERE/"policy_candidate"`。

**B-13 长时间用"中文状态消息"当机器状态。**
`base_node.py:558,1110,1434,1447-1451`、`state_machine.py:63,94,260`、`pid_navigation.py:3775` 把中文自由文本塞进 `message` 并被记录进证据；改一个词就改了记录数据。
**建议**：旁边加稳定的 `message_code`。

**B-14 三个 launcher 的锁集合不一致 ⇒ 可以并发跑两个入口。**
`main.py` 3 把锁、`benchmark.py` 4 把（多 `building_navigation`）、`building.py` 1 把（只有 `building_navigation`）。`start.py` 与 `building.py` 会在同一台机上同时操作 ROS/Gazebo。
**建议**：抽 `common` 的 `acquire_run_locks(kind)`，并写明哪些组合互斥。

**B-15 同一事实（测试数）在 6 个地方有 6 个数字。**
48 / 54 / 62 / unit170 / unit217 / unit283 / 工作树 unit289。每个都像"当前值"。
**建议**：一张由脚本生成的"提交 → 实测计数 → 命令 → 日期"表，其余文档只引用它。

**B-16 `docs/review/full-readability/README.md:64` 的"唯一的测试 AST 例外"极易误读。**
它指"本轮改造范围内唯一被改的测试断言"，但读者会读成"仓库里只有这一处源码断言测试"；实际有 7 个文件（§10）。

**B-17 本机 README 权限不一致（14 个 600、32 个 664）。**
600 的包括 `launch/README.md`、`perception/README.md`、`localization_mapping/README.md` 等，只有属主可读，与其它文档不同。
→ 这是**本地文件系统现状**，不是 git 可见差异：Git 通常只跟踪可执行位，读写位不随提交传播，因此既不能说"这些差异都是未提交变更"，也不能假定他人检出后同样如此。
**建议**：本地统一为 664/644（**本次未改权限**）；评审时不要把它当 diff。

**B-18 `perception/depth.py:30-65` 一个 30 行函数里有两种坐标帧。**
docstring 说"反投影到 Go2 IMU 帧"，返回的点其实是相机射线旋转后**减去 `imu_mount`**（原点 IMU），但自车包围盒检查又把 `imu_mount` 加回去（在 **base** 帧）；两者相差一个 COM 偏置（厘米级）。发布的 ROS header 又写 `go2_imu`，强化了"IMU 帧"的读法。
**建议**：返回带名字的两套点（`points_imu` / `points_base`），或让 dataclass 明确"origin=IMU, axes=base"。

**B-19 同一物理量有多份副本，且单位混用。**
内参 `512*24/42` 在 `depth.py:35,86`、`capture_go2_rgbd.py:519,1521`、`camera_rgbd.yaml:12-13`（又写成十进制 `292.5714…`）、`capture_go2_rgbd.py:401-403,419` 反复出现；同一角度在代码里是 `camera_yaw_rad`、在产物里是 `custom_sensor_yaw_degrees`；同一时间在 NPZ 里是 int ns、在 `imu_stream.jsonl` 里是 float 秒、在 ROS 里是 `stamp.sec/nanosec`。
→ 改一处内参不影响其它副本；"这个 0.1 是米还是毫米"无法从代码判断。

**B-20 在线与离线两条近重复的 learned 管线，产物同名但 metadata key 不同。**
`learned_mapping.finish()`、`reconstruct_dynanav_learned_person_map.py`、`reconstruct_dynanav_slam_map.py`、`oracle_mapping.py` 都写 `static_map.npz`，但 key 集合不同（有的有 `voxel_size`，有的没有；**没有任何生产者的写 `provenance`**，而 `view_dynanav_map.py:22-30` 声明它会读 `provenance`），且 `load_map:90-92` 对缺失 key **静默跳过**。
→ 同一个查看器在不同产物上"看起来正常"，缺字段时没有任何提示。

**B-21 阈值重复且不一致。**
姿态年龄门在 `learned_mapping.py:170` 是 `250_020_000` ns，在 `person_geometry_tracking.py:127` 是 `250_000_000`；多出的 20 µs 无解释。触发策略默认值在 `person_worker.py:306-323` 与 `learned_mapping.py:63-67` 各存一份。分阈 0.1 出现在 4 个文件；`--mask-threshold 0.5` 与 `--score-threshold 0.1` 的关系无人说明。
→ "这里 0.1 那里 0.25"无法从代码回答，改一处会悄悄让另一处失效。

**B-22 provenance 靠**字面字符串**在 5 处校验，没有枚举。**
`"fast_livo2_async_backend"` 出现在 `feedback_state.py:32`、`building_navigation.py:230`、`feedback_source.py:14`、`parking_pose10hz.py:58,79`；`"camera_init"` 出现在约 30 个文件；`"slam"/"sim_oracle"/"livo_post_vio"` 也被多处字符串比对。打错一个字母 → 位姿被静默拒绝（fail-closed），现象看起来像"SLAM 坏了"。
**建议**：`common/provenance.py` 里定义枚举常量，生产方与门禁共用。

**B-23 用 `importlib` 在运行时加载 CLI 脚本取函数。**
`learned_mapping.py:453-461` 用 `spec_from_file_location` 加载 `scripts/query_scene_map.py` 再调 `select`/`context_text`；`reconstruct_dynanav_learned_person_map.py:379-385` 同样手法加载另一个脚本。
→ grep/IDE 找不到调用关系；脚本的模块级副作用会在映射线程里执行。
**建议**：把 `select`/`context_text` 移进 `semantic_mapping/query.py`，CLI 保持薄层。

**B-24 文件名的"意图"与实现不符。**
`learned_person_mapping.py` 不做地图，只把单帧转成每点状态 + 离线 provenance 注释器（真正的地图在 `simulation/dynanav/learned_mapping.py`）；`known_scene_inputs.py`（1138 行）不是"inputs"，是位姿/点云/元数据的 join 消费节点；`temporal_geometry.py` 不是通用时序几何，是重复观测体素投票缓存；`mapping/map_archive.py` 同时是实时状态发布器 + save 服务。
→ 按文件名找模块会先找到错的。

**B-25 静默回退让"没观测到"和"观测被丢掉"无法区分。**
`person_worker.py:163-166` 把任何不可读/损坏的 `state.json` 都返回 `{"state": "not_started"}`（损坏的 worker 看起来像没启动）；`view_dynanav_map.py:90-92` 缺 metadata 静默跳过；`learned_mapping.py:168` 从 `source_ns` 回退到 `round(latest_odom["time"]*1e9)`。
**建议**：统一带 `reason` 码的状态枚举并计数（`unmatched.jsonl` 是好的范式）。

**B-26 "原子写"helper 在至少 5 处被复制，保证强度不同。**
`person_worker.py:40-63`（`atomic_npz/atomic_json`）、`view_dynanav_map.py:112-115`、`reconstruct_dynanav_learned_person_map.py:43-55`、`backend/worker.py:20-26`、`reconstruct_dynanav_slam_map.py` 各有一份，`O_EXCL link` / `replace` / 普通写混用。
**建议**：`common/artifacts.py` 统一并注明保证。

**B-27 文件名写 `truth`，内容却声明不是真值。**
`truth_navigation_command.json` 由 `building_navigation.py:623` 写入，字段里是 `"navigation_ground_truth_used": false`、`"source": "PCT->native SCAN->FAST_LIVO2 spatial feedback"`；`truth_worker.py:77` 与 robogauge worker 读它。同理 `/demo/teacher/*` 话题、`TEACHER_SOCKET`、`libteacher_actuator.so` 全是历史命名。
**建议**：至少在这三处加一行"truth/teacher 是历史命名，不代表来源"的注释；长期改成 `command_stream.json` 之类。

### C 级：噪声（不影响正确性，但拖慢阅读）

- **C-1** 超长单行段落：`configs/simulation/dynanav/README_LIO.md:197`（2167 字符）、多篇实验报告单行 800~1000 字符。
- **C-2** 用实验日志当操作手册：根 `README.md:25` 指向 421 行日志来回答"启动命令"。
- **C-3** 159 条 `/media/hyh001/新加卷/...` 绝对路径内联在文档里（本机可解析、不可移植）。
- **C-4** 免责声明密度过高：`perception/README.md:23` 一段含 3 个数字 + 2 条"不等于验收"；`simulation/dynanav/README.md:47` 一段 10+ 数字 + 3 条边界。建议事实/边界分两列。
- **C-5** 术语中英混排无约定：同目录中文 README + 英文 README_LIO；测试 docstring 全英文。
- **C-6** 历史 commit 哈希散落正文（b44f748d / 5e7982e / 5e32eb9 / bffa2b6 / 43cba33）且无含义说明。
- **C-7** `docs/architecture.md` 只描述主路径，另三条路径完全没有架构位置。
- **C-8** `README.md:1` 的 "Development acceptance is a separate tool" 与 `acceptance_required=False` 字段——仓库里**没有这个工具**，字段无人消费。
- **C-9** `base_node.py` 有 3 处重复的 `from ...common.observation import api as _observer_api`（`:32`、`:82`、`:97`），像两个模块拼在一起。

---

## 12. 文档与实现不一致清单（已核对）

| # | 文档说法 | 代码事实 | 证据 |
| --- | --- | --- | --- |
| D1 | `navigation/mission/README.md:7,20`：`state_machine.py` 是活的、经 pid_navigation 使用 | `Mission` 从未实例化；只有 `ContinuousMission` 继承它，而后者也无构造点；live 的 mission 角色是 `mission_runtime` | `mission/continuous_route.py:19,414`；`pid_navigation.py:1200-1206`；`control/launch/components.py:30` |
| D2 | `navigation/README.md:13-16`：`pid_navigation` 与 `task.py` 都算"现役" | `task.py` 只在独立入口 `relative-slam-navigation` 上；不在 `demo.launch.py` | `pyproject.toml:17`；`launch/launch/demo.launch.py:15-20` |
| D3 | `control/README.md:8`：任务级状态机在 `navigation/task.py` + `navigation/mission/` | 默认运行的任务状态机是 `base_node.control()` | `base_node.py:1071-1488` |
| D4 | `robot/unitree/README.md:7`：`LIMITS=(.3,.2,.3)` 是契约 | 只是 fallback；有 profile 时是 `[0.6,0.2,0.6]` | `bridge.py:49-50,123-133`；`configs/control/settings.json` |
| D5 | `planning/admission/README.md:27`：准入未闭合 | 只对 `FrozenFence` fallback 成立；默认走 R85 分支 | `pid_navigation.py:1097-1110`；`configs/planning/settings.json:14-24` |
| D6 | `configs/launch/settings.json:32,54`：两个探针 = `"1"` | `launch/main.py:72-84` 无条件改 `"0"` | 同上 |
| D7 | `simulation/README.md:8`（+根 `README.md:9,23`）：`dynanav` "只做 capture" | `dynanav/` 还含 oracle/learned 在线建图、实测位姿路线控制、行人/动态 hold、会话碰撞修复 | `simulation/dynanav/README.md:3,12-13,21-29`；`oracle_mapping.py`、`learned_mapping.py`、`slam_route.py`、`dynamic_hold.py`、`session_collision.py` |
| D8 | 根 `README.md:23`：DynaNav 入口是 `capture_go2_rgbd.py` / `scripts/capture_dynanav.py` | 当前入口是 `scripts/run_dynanav_experiment.py`；`capture_dynanav.py` 是旧采集探针 | `simulation/dynanav/README.md:8,23,59-62` |
| D9 | `docs/robogauge/README.md:12`：`configs/robogauge/contract.json` 是冻结契约 | 无代码读取；权威是 `robogauge/contract.py:11-20` 硬编码 | grep 全仓 |
| D10 | `configs/scenarios/README.md:12`：`terrain_targets/lower12.json` 提供 provider 信息 | 无代码读取；`mission46.py:28-29` 指向不存在的 `tests/terrain_targets/` | grep + `ls` |
| D11 | `robot/unitree/policy/worker.py:396`：`privileged_actor_observation_dimensions=232` | 文档/契约/代码都是 **247D** | `native/README.md:1,7-8`；`observation.py:237,291` |
| D12 | `docs/review/validation-and-threshold-audit.md:4-6`：行号以 `b44f748d` 为准 | 当前 HEAD `43cba33`，行号已漂移（如 `input_freshness.py` 的 `enabled()` 从 `:2-6` 移到 `:11`） | 抽查 |
| D13 | `README.md:46`：最新测试见"上方实验记录" | 上方链接的报告自报 unit170；最新是未提交的 online-person 报告（unit283） | `dynanav-20261109.md:198`；`dynanav-online-person-20261109.md:189` |
| D14 | `docs/architecture.md:28`：tests/unit 是"纯 CPU" | `test_dynanav_pedestrian` 需 USD/pxr 运行时，缺则 3 项跳过 | `test_dynanav_pedestrian.py:105-110` |
| D15 | 已提交文档把运行命令当常规操作，未记录"当前暂停" | 暂停是**用户决定**，只写在未提交报告里；历史运行（DynaNav 四档、在线行人完整路线与 45 s 片段、镜像 smoke）均有成功记录 | `dynanav-online-person-20261109.md:115-121` |
| D16 | `docs/robogauge/README.md:43`：68.815/73.815 s | 实验文档写 67.815/72.915 s（差 1.000 s，无交叉说明） | `dynanav-20261109.md:38-39` |
| D17 | `localization_mapping/frontend/fast_livo2_ros/README.md:5,8`：两个副本与上游**逐字节相同** | `scripts/decompress_images.py` 相同；`launch/mapping_aviz.launch.py` **不同**（多了 6 行 docstring，126 vs 120 行，第 3 行起就 diff），且该差异已提交 | `cmp` / `diff`（B 调查已实测） |
| D18 | `simulation/dynanav/README.md:35`：发布的 `/dynanav/*` 话题清单 | 漏了 `/dynanav/lidar_perception_filtered`（learned 模式的 FAST-LIVO2 输入），只在 `:39` 后文出现；`lidar_static` 只在 oracle 模式发布 | `capture_go2_rgbd.py:545-551,1485-1511` |
| D19 | `perception/README.md:12`：`person_worker.py` 做"持久 RGB-only 推理、模型一次加载" | 该文件明确不 import torch；模型加载在 `scripts/run_dynanav_person_worker.py:56-62,271` | `person_worker.py:1-4` |
| D20 | `localization_mapping/README.md:18`：`odom_adapter` 输出 `/demo/slam/{body,lidar}_odom` | 生产 launch 把这两个名字 remap 成 `/demo/slam/raw_*`；真正的 corrected `/demo/slam/*_odom` 由 `localization_runtime` 发布 ⇒ **同名话题在不同图里是不同东西** | `launch/components.py:59-73`；`localization_runtime.py:205-208` |
| D21 | `localization_mapping/backend/README.md` | 承认 `backend/run.py:3` 里 `-m localization.backend.run` 是旧路径，但没改代码（一行的事，留着继续误导） | `backend/run.py:3` |
| D22 | `scripts/run_dynanav_lio.sh` usage 文本：`--observer-namespace /dynanav/lio_comparison` | 默认值已是 `/dynanav/lio_observer` | `run_dynanav_lio.sh:36` vs usage 行 |

**核对为准确、无需改的文档**（可作为正面例子）：`launch/README.md`、`launch/launch/README.md`、`simulation/launch/README.md`（屏障 30 s 语义）、`simulation/world/README.md`、`planning/launch/README.md`、`planning/pct/README.md`（import 即执行）、`control/launch/README.md`、`robot/unitree/policy/README.md:8`（247D 仅诊断）、`evaluation/README.md:17`、`control/safety/README.md` 的阈值、`docs/verification/*` 的数字（54/62、exit 2、15 组件 exit 0、1330/798/797/57580 等逐项一致）；`perception/README.md:16` 的重复几何参数（3/0.5 s/300 s/200k/8）、`mapping/README.md` 的 0.08 m/8M、`semantic_mapping/README.md` 的容量界、`sensors/README.md` 的 24 slot/16 frame/32 MiB、`person_mask_propagation` 的"5 项注入 flow 测试"均与代码一致。链接层面：367 条相对链接**无真实断链**，159 条 `/media/...` 绝对链接在本机全部可解析（但不可移植）。

---

## 13. 建议的最小改造（按性价比排序，**本轮未实施**）

1. **加一页 `docs/architecture.md` 级别的"路径索引"**：四条路径 × 入口/前置/输出/domain（本文 §2 可直接用）。这是投入最小、收益最大的一项。
2. **在 `navigation/README.md` 顶部加"入口 → 节点类 → 进程角色 → 是否在默认 launch"三行表**，并在 `mission/README.md` 标注 `state_machine.py` 未实例化。（A-1/A-2/A-3）
3. **把 (vx,vy,wz) 限值收成一处常量**，每个消费者注明执行哪一档。（A-8）
4. **统一 `features` 与探针语义**：`features`（行为）与 `probes`（永不因配置开启）分开，配一张 flag→模块→默认表。（A-7/B-4）
5. **给 `prepare()` 的 4 处静默覆盖加显式提示**（值不同就报错或打印），并删除死配置。（B-5/B-6）
6. **`capture_go2_rgbd.py` 去掉 import 副作用**（包成函数）；`generate_routes.py` 同理。（A-9/§8）
7. **加 `docs/glossary.md` + `tests/README.md` + `configs/README.md`**：三份索引，内容可直接从本文 §10/附录 A/附录 D 生成。
8. **在根 README 顶部加一行状态横幅**："2026-11-09 起暂停仿真/测试；当前仅阅读与文档"。（D15）
9. **给状态类文档加 front-matter**（`snapshot_commit / generated_at / 适用树`），把"当前"改为"该快照"。（D12/D13）
10. **抽 `common/topics.py` 或 docs 里的 topic 表**：52 个 `/demo/*` 话题现在只能靠 grep 找生产/消费者（附录 B 是初版）。

---

## 附录 A：术语表（新人第一次遇到时对照）

| 缩写 | 含义 | 本仓库里具体指 |
| --- | --- | --- |
| **FAST-LIVO2** | 激光-惯性-视觉里程计（LiDAR-Inertial-Visual Odometry） | `third_party/fast_livo2`，二进制 `fastlivo_mapping` |
| **LIO** | LiDAR-Inertial Odometry | DynaNav observer 的 `--mode lio`（图像关闭，`ONLY_LIO`） |
| **LIVO** | LiDAR-Inertial-Visual Odometry | `--mode livo`，RGB 辅助，禁止回退成 image-free |
| **VIO** | Visual-Inertial Odometry | 本仓库里常指 FAST-LIVO2 的视觉部分；`post-VIO` = 同帧视觉更新后的最终位姿 |
| **SCAN** | 本仓库使用的局部规划器 | `third_party/scan`，`scan_planner_node`，经 `scan_epoch_supervisor` 管理 |
| **PCT** | 路径/轨迹生成工具（Building 分支） | `third_party/pct`，`install/pct/pct_astar` |
| **GT** | Ground Truth（仿真真值） | 只用于离线评分、策略本体观测、物理门禁；**不作导航定位输入** |
| **COM** | Center of Mass | `cascade.base_com_offset`，COM 速度内环 |
| **native64** | 仿真原生 64 float 状态帧 | `gazebo.native64.preupdate`，驱动策略/物理门禁 |
| **45D / 247D** | 观测维度 | 45D：Student `exact45` 与 RoboGauge（两者语义不同）；247D：Teacher 特权观测（含 187 条 SDF 射线） |
| **canonical** | "被接受的、带来源与时钟的" | canonical SLAM pose = 已接受并标注来源的 SLAM commit |
| **admission** | 路径准入（必要不充分） | Bézier 曲线是否留在冻结 3D 走廊内 |
| **measured / diagnostic / oracle** | 来源标签 | measured=来自真实测量/估计；diagnostic=仅诊断；oracle=模拟器真值 |

## 附录 B：关键话题速查（52 个 `/demo/*` 话题中的主干）

| 话题 | 类型 | 生产 | 消费 |
| --- | --- | --- | --- |
| `/demo/teacher/slam/image`、`/imu` | Image / Imu | `sensor_relay` | `fastlivo_mapping` |
| `/demo/slam/lidar_filtered` | PointCloud2 | `self_filter` | `fastlivo_mapping` |
| `/demo/slam/raw_body_odom`、`raw_lidar_odom`、`raw_registered_full` | Odometry / PC2 | `odom_adapter` / `fastlivo_mapping` | `localization_runtime`、`map_archive` |
| `/demo/slam/body_odom`、`lidar_odom` | Odometry | `localization_runtime` | 控制器、admission、bridge |
| `/demo/navigation/localization_bundle`、`localization_status` | String | `localization_runtime` | 控制器、mission_runtime |
| `/demo/teacher/navigation/sensor_health` | String | `sensor_gate` | `bridge`（`sensor_health.ready` 门） |
| `/demo/navigation/request` | String(JSON) | `mission_runtime` | `base_node.on_request` |
| `/demo/teacher/navigation/anchor` | String | `mission_runtime` | `bridge` / admission |
| `/demo/navigation/scan_reference` | Path | `base_node` | SCAN |
| `/demo/navigation/bspline`、`trajectory_metadata`、`reference_attempt`、`route_definition`、`local_replan_request`、`execution_frozen`、`scan_epoch_ready` | Bspline / String / Path | `scan_planner_node`（remap） | `base_node` / `scan_supervisor` |
| `/demo/cmd_vel` | Twist | `base_node` | `bridge` |
| `/demo/control/safety` | String | `bridge` | `base_node` / `ros_node` |
| `/demo/navigation/status`、`/stop` | String / Bool | `base_node` / `mission_runtime` | 记录 / 控制器 |
| `/demo/teacher/navigation_bridge/status`、`/stop` | String / Bool | `bridge` | （树内无 Python 订阅者） |
| `/clock` | Clock | `gz sim` | 所有节点（`use_sim_time=true`） |
| DynaNav：`/dynanav/{imu,camera,depth,camera_info,lidar_filtered,lidar_static,lidar_perception_filtered}` + `/clock` | — | `capture_go2_rgbd.py` | observer / 下游 |

## 附录 C：命令速查（**当前均已暂停，仅供恢复后参考**）

```sh
# 只读依赖检查（本机实测 exit=2：13 项中 6 项构建产物缺失）
python3 start.py --check

# 只准备 run 目录，不启动
python3 start.py --prepare-only

# 普通启动（会真的跑仿真；当前暂停）
python3 start.py --duration 60

# 离线单元测试 / 回归（当前暂停）
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression -v

# 数据链接
python3 scripts/configure_data.py --data-root /path/to/6.project_data --link

# DynaNav（三终端；worker 走文件 IPC 不依赖 ROS domain，observer 与 capture 必须同域）
scripts/run_dynanav_lio.sh --check
python3 scripts/run_dynanav_experiment.py --check ...
python3 scripts/run_dynanav_person_worker.py --channel ... --weights ... --device cuda:0
```

## 附录 D：配置读取关系（精简）

- 入口：`configs/launch/navigation.json` → 6 个 `settings.json`（`common/config.py:42-73` 合并 + `${PROJECT_ROOT}` 展开）。
- 必需且被 prepare 冻结：`localization_mapping/{fastlivo,camera}.yaml`、`planning/settings.json` 的 horizon、`simulation/settings.json` 的 spawn/overview/scenario。
- 运行时仍读原文件：`configs/control/settings.json` 的 `student_runtime`、`runtime_paths`。
- 无读取者（建议删或标归档）：`control/relative_pid.yaml`、`navigation/relative_shuttle.yaml`、`robogauge/contract.json`、`robot/unitree_usd_manifest.json`、`runtime_assets_manifest.json`、`scenarios/terrain_targets/lower12.json`、`simulation/dynanav/office.yaml`。

---

## 附录 E：本次工作的证据边界

- **做了什么**：只读阅读 `src/`（58373 行 Python）、`scripts/`、`configs/`、`tests/`、`docs/`、`third_party/` 的接口面、`start.py`、`pyproject.toml`、Docker/compose；执行了 `python3 start.py --check`（只读文件检查，未启动任何进程）与纯静态 grep/ast 统计。
- **没做什么**：未修改任何源码/配置/测试/文档权限；未运行测试套件；未启动 ROS/Gazebo/Isaac/机器人；未 git add/commit/push；未访问网络。
- **不能据此声称**：本次会话跑通或跑不通任何路径、任何测试"当前全绿"、任何精度/性能结论。文中"已运行/退出 0/指标"数字均转引自 `docs/experiments/` 与 `docs/verification/` 的既有记录，未独立复现。
- **历史 vs 本次（口径）**：DynaNav 无行人四档、0.3 m/s 在线行人**完整路线**（`mission_completed=true`）与 45 s 片段（`mission_completed=false`）都是**历史已运行**记录（进程退出 0、有独立审计；感知质量指标未通过）；主路径 A 的 45 s 镜像 smoke 是**历史启动记录**（15 组件退出 0），不等同完整任务验收；本次会话**没有重跑**任何路径。
- **分工**：本文由 Lead 汇总 4 个只读调查（启动/仿真、感知/SLAM、规划/导航/控制/机器人、文档/测试/配置）与 Lead 自己的入口/公共层阅读；各分组的原始证据笔记保存在任务目录 `.dsh-subagents/2026-11-09T07-25-26-735Z-81d359de/notes/`（不属于交付物，未提交）。
