# 模块结构与耦合总评（2026-11-09）

本轮把仓库内 **62 个 README**（61 个原有 + 新建 `scripts/README.md`）统一改写为同一结构：`目录结构` / `代码、日志与配置` / `模块内文件耦合` / `可读性评估` / `重构建议`，并在此基础上给出跨模块结论。本文是**索引与汇总**，细节以各模块 README 为准。

## 1. 范围与方法

| 项 | 内容 |
| --- | --- |
| 覆盖 | `src/embodied_navigation/` 48 个、`configs/` 5 个、`docs/` 4 个、`tests/` 2 个、`scripts/` 1 个（新建）、仓库根 2 个（`README.md`、`README_zh.md`） |
| 事实来源 | 磁盘实测（`ls`、`grep -n`、AST 解析、git）+ `/tmp/readme_audit/facts/` 生成的模块事实包。**行数口径**：本文与根 README/包索引用 facts 包文本口径（`read_text().count("\n")+1`，每文件比 `wc -l` 多 1，空文件记 1 行；`wc -l` 全包 60,287），各模块 README 的逐文件行数多用 `wc -l`——引用时差 1 属口径差异 |
| 自检 | `check_readmes.py` 逐 README 校验：5 节非空、目录树与磁盘逐项一致、相对链接有效、原有"限制/未验证"事实保留、无空话模板句。**62/62 通过（0 error）** |
| 边界 | 本轮**只改 `.md`**，未改任何 `.py`/`.json`/`.yaml`/SDF/C++；未 commit、未回滚工作树既有改动。行号均为 2026-11-09 当前 checkout 实测，源码变动后会漂移 |
| 未验证 | 未启动 ROS/Gazebo/Isaac，未跑完整测试套件（最后统一跑 CPU 单元/回归，结果见 §8）。所有"运行时行为"结论来自源码与产物写入点，不是实跑证据 |

## 2. 全局现状（AST 粗计）

包内 **259 个 Python 文件 / 60,546 行**；11 个一级模块：

| 一级模块 | 文件 | 行数 | >800 行文件 |
| --- | ---: | ---: | --- |
| navigation | 27 | 11,634 | base_node(1618)、pid_control(1174)、pid_support(1031)、pid_admission_apply(892)、ros_node(875) |
| control | 46 | 9,718 | cascade_core(1851)、transition_gate(1731) |
| planning | 32 | 9,361 | async_admission(1200)、static_geometry(1010)、validate_route_support(1003)、reference_protocol(891) |
| simulation | 43 | 9,095 | 无 |
| localization_mapping | 40 | 8,429 | known_scene_inputs(1139) |
| perception | 18 | 4,631 | person_geometry_tracking(823) |
| robot | 17 | 4,591 | bridge(1069)、policy/student(838) |
| common | 20 | 1,616 | 无 |
| launch | 9 | 1,314 | 无 |
| evaluation | 3 | 95 | 无 |
| sensors | 2 | 37 | 无 |

函数 docstring 覆盖率约 0.25–0.57（launch 0.57、simulation 0.40、robot 0.39、planning 0.34、control 0.33、navigation 0.27、common 0.27、localization 0.25），与文件规模不匹配：**最大的 10 个文件里 9 个在 docstring 覆盖最低的 navigation/control/planning/localization_mapping**（唯一例外是 robot 的 `bridge.py`）。

## 3. 跨模块耦合结论

模块级依赖**不是严格 DAG**：11 个一级模块间有 9 组 A↔B 双向依赖（AST 解析）。多数是 1–2 条窄依赖（共享小工具/数据结构/装配 helper），跨层直达、值得优先处理的是下面 6 条，均已 grep 复核并在对应模块 README 记录。其中第 1/2/4/5/6 条列入根 README 的"层次倒置"表；第 3 条（定位↔停车契约互引）是双向契约依赖，只在模块级记录——`docs/architecture.md` 的"数据流分层"是功能叙述，不是严格依赖 DAG：

| # | 倒置 | 证据 | 影响 |
| --- | --- | --- | --- |
| 1 | 规划持有控制器子类 | `planning/reference/global_route_reference.py:11` → `control.tracking.cascade_core.Controller`；反向 `control/tracking/cascade_core.py:16` → `planning.reference.spatial_reference` | planning 与 control 无法独立测试/替换；`StableProjectionController._project` 与 `Controller._project` 近乎逐行重复 |
| 2 | 定位使用导航层数据类 | `localization_mapping/relative_odometry.py:8` → `..navigation.task.Observation` | 定位层反向依赖导航层；`Observation` 是数据契约，应在 common/sensors |
| 3 | 定位↔停车契约互引 | `localization_mapping/known_scene_inputs.py:72-77` → `control.parking.{causal_join,terminal_owner}`；`control/parking/terminal_owner.py:6` → `known_scene_hold_guard._frame`；`known_scene_hold_guard.py:252,263` 延迟 import `transition_gate` | 文件级因延迟 import 不成环，所以 `import` 体检发现不了 |
| 4 | 传感适配层依赖控制内部 | `perception/sensors/feedback_mailbox.py` → `control.tracking.cascade_core`、`control.parking.parking_pose10hz`；`self_echo_filter.py` → `localization_mapping.*` | "感知不依赖控制"的分层叙述与实际不符 |
| 5 | 策略 ACK ↔ 停车控制双向 | `robot/unitree/policy/lower_loop_ack.py` ↔ `control/parking/integral_unload.py` | 执行桥与控制律任一侧改动都要双侧回归 |
| 6 | 安全门禁跨层直达 | `control/safety/duplicate_source.py` → `robot.unitree.bridge`、`planning.admission.physical_gate` | 安全模块同时依赖机器人/规划，边界难判定；`validate_controller_source` 住在 ROS 节点里是根因 |

其余窄环（`navigation↔simulation`、`robot↔simulation`、`launch↔planning`、`launch↔localization_mapping`、`localization_mapping↔perception`）多为装配 helper 或共享消息契约，可在对应 README 里找到逐条说明；不建议为了"消除环"强行搬动，但新增依赖应先看该模块 README 的"模块内文件耦合"节。

依赖密度最高的是 navigation（import control 39 处、common 35 处、planning 16 处），也是最大模块；`navigation/pid_support.py` 是 7 个文件的导入枢纽，5 个 `*Mixin` 经 `pid_navigation` 组合、`pid_entrypoint` 延迟导入破环。

## 4. 跨模块重复实现（可收敛）

| 重复项 | 位置 | 建议 |
| --- | --- | --- |
| 原子 JSON 写 | `common/runtime_io.py:51`、`common/runtime_telemetry.py:24`、`perception/sensors/sensor_gate.py:136`、`perception/person_worker.py:54` | 收敛到 `common` 单一 helper（临时后缀/`allow_nan`/`ensure_ascii` 已不一致） |
| actor 适配脚手架 | `simulation/truth_worker.py` ↔ `simulation/gazebo/robogauge/worker.py` 逐行匹配 **81 行非空（94 行含空行）**（同名 JSONL 字段、SIGUSR1 停止、`ContactPublisher` 生命周期；`difflib` 可复现） | 抽共享 helper；影响 Building 与 RoboGauge 两条路径 |
| rpy 公式 | `simulation/gazebo/robogauge/native_loop.py:51` ↔ `simulation/safety_state.py:23` | `native_loop` 已 import `safety_state`，改用 `native_rpy` + 数值逐位断言 |
| 投影实现 | `planning/reference/global_route_reference.StableProjectionController._project` ↔ `control/tracking/cascade_core.Controller._project` | 只差弧长下界 `max(0, progress-0.15)`；明确哪份是权威 |
| PointCloud2 XYZ 解码 | localization_mapping 内 3 份 | 统一到 `geometry` 一处，附等价性对照 |
| `payload()`/schema | `planning/reference_protocol.py`(schema 3)、`planning/trajectory_contract.py`(schema 1)、`planning/admission/path_admission.py` | 明确唯一权威，其余标历史 |
| scripts 横切工具 | 4 份 `digest`、3 份 `write_json`、9 份 `ROOT` | 抽 `scripts/_support.py`（脚本不是包，可按相对路径 import） |
| 策略输入契约 | exact45（Student）/ 247D（Teacher，仅诊断）/ 45D（RoboGauge）三套并存 | 不是重复而是**必须区分**；已在 `docs/glossary.md` 与 robot README 对齐 |

## 5. "声明了但默认没有生效"清单（写文档/排障时最容易误判）

| 项 | 事实 | 文档处理 |
| --- | --- | --- |
| 历史证据 JSONL | `common/runtime_telemetry.py` 的 `RuntimeTelemetry.append/enqueue` 是空操作，只有 `.atomic()` 写实时状态；`navigation/ros_node.make_evidence_writer` 恒返回该类型 | 所有模块 README 统一写"声明但默认不落盘"；**不要**把代码里的 `evidence.append` 当成产物存在 |
| `navigation_command_history.jsonl` | `bridge.py` 计算路径但 `control/io.py:12-14` 固定传 `history=None` | 标注"不产出"（P2 决定接线或删除） |
| `safety_configuration.clock()` / `clock_hold.CONTRACT` | 同一组常量的两份副本，全仓 0 调用点，`configs/` 无 `clock_protection` 键 | 标注"看起来在校验、实际没接线" |
| `sensors/frames.py` | 全仓零导入者 | 标注"预留契约，未接线" |
| `perception/person_mask_propagation.py` | 无生产导入者（仅测试） | 同上 |
| DDS 预检 | `dds_environment.DOMAIN_QUERY_DIRECTORY` 无读取点；`native_environment()`/`verify_environment()` 无外部调用点 | 同上 |
| 4 个动态 helper | `student.py` 通过 `Path(__file__).with_name(...)` 加载 `pre_zero_integral_unload.py` 等 4 个文件，本检出不存在对应源码；其中 2 个有配置键（`r85_pre_zero_integral_unload`、`r85_student_yaw_pd_fade`，默认 `enabled:false`），另 2 个无独立配置键、只在特定分支按 `with_name` 尝试加载 | 写"仅有部署接线，helper 源码不在本检出，且不是全部由配置开关控制"，不要说"已实现" |
| `reference_version_paths()` | 全仓只有定义 | 标注死代码（可能有外部使用，未核实） |
| `go2hv_actuator.cpp` 的 `saturation_effort` | 解析/校验/写日志但不参与钳位，与 Teacher 插件语义相反 | 标注"调它没有效果" |
| 无 Python 读取点的配置 | `control/relative_pid.yaml`、`navigation/relative_shuttle.yaml`、`robot/unitree_usd_manifest.json`、`runtime_assets_manifest.json`，另有 `configs/control/settings.json` 的 3 个 `cascade.*` 键 | 统一写"无 Python 读取点/保留归档"，避免"改了没生效"被当 bug |

## 6. 测试缺口（按模块 README 的"测试"节归纳）

现有套件：unit 323 项、regression 86 项（含 Torch/私有资产 skip），全部离线 CPU。本轮没有新增测试，只记录缺口：

- `navigation/`：`base_node`/`ros_node`/`pid_*` 控制器行为**零专用单测**（`tests/unit/test_navigation.py` 只覆盖 `task.py`），这是全仓最大的验收缺口。
- `control/`：两个最大文件 `cascade_core.py`(1851)、`transition_gate.py`(1731) 在 `tests/` 零直接断言；`executor_command`、`publication_ledger` 同样无测试。
- `localization_mapping/`：`known_scene_*`、`odom_adapter`、`localization_runtime`、`building_runtime`、`building_sensors`、`mapping/map_archive`、`launch/components` **零直接断言**；现有测试地图（`tests/README.md`）未列这条缺口。
- `perception/sensors/`：无功能测试（仅 `test_ros_shutdown_lifecycle.py` 对 `sensor_gate.py` 做 AST 检查）。
- `scripts/`：17 个文件里 11 个有覆盖；`replay_dynanav_rgb.py`、`run_dynanav_online_person.py` 的进程编排/超时分支未测。
- `simulation/dynanav`：源码快照测试只守护 `capture/*.py`，新增顶层 `dynanav/*.py` 不会被抓，运行身份可能缺文件。
- 反向问题：部分测试直接读源码文本/AST（入口闭包、原生安全接线），**改代码结构会先影响这些测试**而不是行为——重构前先看 `tests/README.md` 的提示。

## 7. 各 scope 结论汇总

| scope（README 数） | 耦合结论 | 可读性结论 | P0 | P1 |
| --- | --- | --- | ---: | ---: |
| perception + sensors + common + launch（8） | 清楚；`launch/` 偏重（入口→库单向无环，但 `main.main()` 218 行、与 `building.run()` 重复进程监督） | 3 良好 / 3 可接受 / 2 需改进 | 0 | 7 |
| localization_mapping（10） | 偏重（包级双向 + 1138 行 `known_scene_inputs` mixin；backend 单向分层清楚） | 需改进集中在根与 `known_scene_inputs`；其余可接受 | 0 | 3 |
| navigation + scripts（4） | 单向无环但偏重（`pid_support` 枢纽、851 行单函数）；scripts 横向重复 | navigation 需改进（5 个 >800 行文件）；scripts 4 个需改进 | 0 | 7 |
| control（5） | 偏重（`cascade_core`/`transition_gate` 同时是双向依赖端点；契约层清楚） | 契约层良好，两个最大文件需改进 | 0 | 11 |
| planning（5） | 偏重（三个千行文件 + `static_geometry ↔ validate_route_support` 的 importlib+内容哈希互锁）；叶子模块清楚 | 可接受→需改进（`validate_route_support.run()` 360 行、`scan_epoch_supervisor.main()` 597 行） | 0 | 6 |
| robot（5） | 清楚（三层单向，除 `bridge ↔ control` 与 `lower_loop_ack ↔ integral_unload`） | `bridge.py`/`worker.main()` 需改进；native/robogauge 可接受 | 0 | 4 |
| simulation（10） | 清楚但有重复脚手架与跨层 monkeypatch | 3/5 良好、其余可接受；`truth_worker`/`native_loop`/`world.prepare` 需改进 | 0 | 7 |
| 包索引 + configs + tests + docs（12） | 索引类文档；包级结论见 `src/embodied_navigation/README.md` | 文档结构统一，配置说明补全读取点 | 0 | 3 |

**全部 scope 的 P0 都是 0**：没有发现"必须立即修、否则会算错/会动安全边界"的结构问题；两处安全相关判定都是 fail-closed（`pid_navigation.py:188-192` 拒绝未验证走廊控制权、`building_navigation.py:42-45` 拒绝非 SLAM），`FrozenFence` 缺字段也是抛 `ValueError` 而不是静默通过。这是本轮最重要的"没有坏消息"。

## 8. 全局重构优先级（跨模块）

| 优先级 | 动作 | 理由 | 先决条件 |
| --- | --- | --- | --- |
| P1 | 拆 `cascade_core.py`(1851)、`transition_gate.py`(1731)、`base_node.py`(1618) | 三个文件同时是双向依赖端点与 >400 行单函数聚集地 | 先给控制律录逐帧命令对照（现有 1000 组 PID 对照可扩展） |
| P1 | 修 §3 的 6 条层次倒置（`Observation` 下沉、规划不 import `Controller`、`validate_controller_source` 移到 `common`、`feedback_mailbox` 只依赖消息契约、断开 `localization↔parking`、断开 `safety→robot/planning`） | 让 perception/localization/planning 可独立单测；import 体检能发现新环 | 先补对应模块的单测（当前多处为零） |
| P1 | 补齐 3 个最大模块的单测缺口（navigation controller、control tracking/parking、localization known_scene_*） | 这些文件最大、改动风险最高，却几乎没有行为测试 | 用现有 run 产物做 golden 样本 |
| P2 | 收敛 §4 的重复实现（原子写、actor 脚手架、rpy、投影、PointCloud2、scripts 工具） | 重复实现已出现行为差异（临时后缀/`allow_nan`/弧长下界） | 每项先写等价性断言 |
| P2 | 统一 §5 的"声明但未生效"口径（README 已统一），并逐个决定接线或删除 | 排障时最容易误判"没有产物=程序坏了" | 需产品/契约所有者决定 |
| P2 | 补 `configs/README.md` 的"无读取点/部分读取点"表（3 个 cascade 键、`terrain_layer_switch`、三处反直觉读取点） | 配置与代码耦合的可见性 | 无 |
| P3 | 把 `check_readmes.py` 的 5 节/树一致性检查纳入 CI 或 pre-commit | 防止本轮成果随代码漂移 | 脚本需移入仓库（当前在 `/tmp/readme_audit/`） |

## 9. 验证与一致性

- **结构自检**：62 个 README 全部 5 节齐全、目录树与磁盘一致、相对链接有效、无模板空话（`check_readmes.py --all` = 0 error / 0 warning）。
- **越界检查**：`git status` 与任务开始时基线逐项比对，工作树中非 `.md` 改动与改动前完全一致（15 个既有 M + 既有 untracked），本轮未新增任何代码/配置改动。
- **口径统一**（本轮新增的一致性修正）：证据默认不落盘、`runs/` 是外部数据卷软链、experiments 产物由 scripts 写、simulation 真值不进入 SLAM、exact45/247D/45D 三套契约区分、`FrozenFence` 是未闭合 fallback、`pid_navigation` 是 facade（实现在 `pid_*`）、`state_machine.py` 无实例化点。
- **旧文档更正**（以磁盘为准）：`backend/run.py` docstring 已更名；`frontend/fast_livo2_ros/launch/README.md` 原称"与上游一致"实测不一致；`robot/unitree/policy/native/` 无 CMake/C++（C++ 在 `simulation/gazebo/robogauge/motor_src`）；`docs/architecture.md` 的 `pid_navigation.IntegratedNavigation` 与现名 `PIDNavigation` 的差异已在模块 README 按实际写。
- **独立复核**：两名复核员交叉复核全部 62 个 README 与本文，抽查约 180 条硬断言，报出 ERROR 11 条（根 README 数字、`configs/README.md` 的 dynanav 计数、`tests/README.md` 的 skip 归属、`planning/README.md` 函数名、`navigation/README.md` 证据口径、本文 §3 编号等，均已在本轮内修复）与 WARN 19 条（行数口径、空 `__init__.py`、函数长度口径、快照计数等，已统一口径或标注）。复核报告作为会话内产物保存在 `/tmp/readme_audit/reviews/`，不入库。
- **测试结果**（本轮实跑，仅确认文档任务未影响测试基线）：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit` → `Ran 321 tests ... OK (skipped=4)`（skip 因本机缺 `rclpy`/USD 运行时）；`tests/regression` → `Ran 86 tests ... OK (skipped=5)`（skip 因可选 CPU Torch 未装）。与 `tests/README.md` 记录的"本轮复跑 321/86"一致；历史记录的 323 是 `rclpy` 可导入环境下的数字，差异来自环境而非删测试。

## 10. 未核实与限制

- 全程未启动 ROS/Gazebo/Isaac，未加载任何原生 `.so`；所有进程装配、时序、日志字段均按源码/字符串核对。
- 行号是当前 checkout 实测，源码后续改动会漂移；重构前请重新 `grep -n`。
- 部分结论依赖"字符串契约"而非运行证据（例如 `evidence.append` 是否被自定义 writer 覆盖、外部数据卷里是否已有历史 `.jsonl`）。
- `install/` 本机只有 `backend/`，Teacher/RoboGauge 两个 C++ 插件与 ROS 前端未构建；`docs/robogauge/` 的构建步骤未在本轮执行。
- 4 个动态 helper 是否存在于外部数据卷/私有部署目录，本检出不可见。
- `docs/review/full-readability/verification.json` 的 504/85 与三份 coverage 按 kind 合计 494/89/4/2 差 4 条；来源未解释，已在 full-readability README 标"未核实"，本轮未改动该目录 JSON。
- `docs/review/` 下历史报告记录的是"当时 HEAD 的行号"，当前工作树已有结构重构（`pid_navigation.py` 拆为 `pid_*.py`、`capture_go2_rgbd.py` 拆出 `capture/`）；这些报告按历史快照保留，不回填，查代码请按符号 grep 而不是按旧行号。
- 本文档与各模块 README 的"重构建议"都**没有实施**；验证方式只写到步骤层面。

## 11. 入口

- 包索引与包级三类说明：[src/embodied_navigation/README.md](../../src/embodied_navigation/README.md)
- 配置读取顺序：[configs/README.md](../../configs/README.md)
- 测试命令与边界：[tests/README.md](../../tests/README.md)
- 文档索引：[docs/README.md](../README.md)
- 模块级 5 节（结构/代码日志配置/耦合/可读性/重构建议）：各模块目录下 `README.md`
- 上一轮逐文件可读性审查（历史记录）：[docs/review/full-readability/README.md](full-readability/README.md)
