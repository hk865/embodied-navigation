# navigation

本目录是导航任务层：把 canonical SLAM 位姿、IMU、注册点云与 SCAN 轨迹组织成导航任务，决定航点等待、跟随、到点稳定、转向、停车与失败。

## 边界

- 做：任务状态与节点适配（`task.py`、`pid_navigation.py` 及其 `pid_*` 职责模块、`ros_node.py`、`base_node.py`、`building_navigation.py`），以及到达区域与任务运行时（`mission/`、`robogauge/`）。
- 不做：控制律（在 `../control/tracking/`）、参考路线与准入（在 `../planning/`）、进程装配（在 `../control/launch/` 等 launch 组）。
- 不把仿真真值当作 SLAM 输入；真值只出现在明确标注的评测与仿真安全边界。

## 目录结构

```text
src/embodied_navigation/navigation/
├── mission/                            # 子模块，见 mission/README.md
├── robogauge/                          # 子模块，见 robogauge/README.md
├── README.md                           # 本文件（模块说明；行数随改写变化，不标注）
├── __init__.py                         # 1 行，只重导出 task 的 NavigationTask/Observation/TaskConfig
├── base_node.py                        # 1617 行，measured-pose SCAN 基类；默认任务状态机 Navigation.control() 在此
├── building_navigation.py              # 650 行，Building SLAM-only 分支节点；import 即读 run 目录并 rclpy.init()
├── pid_admission.py                    # 616 行，准入生命周期 mixin（AdmissionMixin）
├── pid_admission_apply.py              # 891 行，准入决策应用 mixin（单个方法 _admit_matched_spline 852 行）
├── pid_control.py                      # 1173 行，cascade/PID 速度选择、ACK、停车与命令发布 mixin
├── pid_entrypoint.py                   # 303 行，rclpy 初始化、executor drain 与进程 main
├── pid_ingress.py                      # 454 行，请求/走廊/IMU/odom/cloud 订阅回调 mixin
├── pid_navigation.py                   # 249 行，现役 controller 兼容门面：类组合 + 旧导入路径重导出
├── pid_route.py                        # 301 行，checked trajectory 执行、连续进度与规划适配 mixin
├── pid_support.py                      # 1030 行，共享导入枢纽 + 证据/探针助手
├── relative_slam_node.py               # 172 行，relative-SLAM 任务的 ROS 适配器（独立入口）
├── ros_node.py                         # 874 行，TeacherNavigation 有限包装 + RecordedPublisher 状态发布/证据写入器
└── task.py                             # 346 行，shuttle 任务状态机，与 ROS/仿真解耦（独立入口使用）
```

## 代码、日志与配置

### 代码

| 文件 | 性质 | 判断依据 |
| --- | --- | --- |
| `pid_navigation.py` | 兼容门面（活代码） | 除基类组合（`class PIDNavigation(...)`，L88–95）、`__init__`（L98–231，57 处 `self.* =` 赋值）与 `_main`/`main` 薄包装（L234–249）外，其余是旧导入路径的重导出 import 块；方法体全在同目录 `pid_*.py` |
| `pid_ingress.py` / `pid_admission.py` / `pid_admission_apply.py` / `pid_route.py` / `pid_control.py` | 活代码 | 各定义一个 `*Mixin` 类（`pid_ingress.py:24`、`pid_admission.py:37`、`pid_admission_apply.py:35`、`pid_route.py:22`、`pid_control.py:50`），由 `pid_navigation` 按 MRO 组合 |
| `pid_support.py` | 活代码 + 导入枢纽 | 25 个顶层函数 + 2 个辅助类（`_TeacherSubstageTiming`、`ResetBookkeeping`）；`pid_entrypoint.py:19-35` 连 `argparse`/`json`/`os`/`Path` 都从这里取 |
| `pid_entrypoint.py` | 活代码（进程入口） | `_main()`（L59–291）做 rclpy init、`IntegratedNavigation` 组合、drain 与收尾收据 |
| `base_node.py` | 活代码 | `class Navigation(Node)`（L108）+ `control()`（L1077–1493，417 行）；`main()` 在 L1594 |
| `ros_node.py` | 活代码 | `TeacherNavigation(base.Navigation)`（L134）+ `RecordedPublisher`（L58）；`control()` 在 L600 |
| `task.py` + `relative_slam_node.py` | 活代码（独立入口） | 只有显式 `main()` 建节点；console script `relative-slam-navigation` 指向 `relative_slam_node.py` |
| `building_navigation.py` | 活代码（仅子进程） | L28 起模块级 `run = Path(os.environ["DEMO_RUN_DIR"])`、读 `experiment_setup.json`/`PCT_route.json`/`sensor_contract.json`，L47 `rclpy.init()`；只由 `../../launch/building.py:354` 与 `robogauge/benchmark.py:118` 以 `-m` 启动 |
| `__init__.py` | 重导出 | 58 字节，只有 `from .task import NavigationTask, Observation, TaskConfig` |

### 日志与输出（默认大多不落盘）

本包不写 `experiments/`；默认主路径的 run 目录是 `runs/<UTC>_<label>_<hex>/`（`runs/` 是外部数据卷软链，不入 Git）。**证据分两类，默认行为相反**：

- **实时状态**：`evidence.atomic(path, value)` → `RuntimeTelemetry.atomic()`（`common/runtime_telemetry.py:24-29`，临时文件 + `replace`），**默认写盘**。
- **历史证据（JSONL/归档）**：`evidence.append()` / `evidence.enqueue()` → `RuntimeTelemetry.append()` 直接 `return None`（`common/runtime_telemetry.py:18-19`），`enqueue()` 同理（`:13-16`）；`enqueue` 里包着的归档闭包（点云数组、轨迹 NPZ）**默认不会执行**，所以默认 run 里不产生这些文件。

依据：`ros_node.py:148` 用 `make_evidence_writer()` 创建 writer，而 `control/io.py:7-9` 恒返回 `RuntimeTelemetry()`；本检出里 `make_evidence_writer` 只有这一个调用点，因此**没有"打开记录开关"的运行路径**——接入记录型 writer 是另一个改动。所以下表"默认写盘"列标 ✅ 的才是默认 run 里能看到的文件，标 — 的需要替换 writer 才会出现；不要因为表里列了 40 多个 JSONL 就推断默认会落盘。

下表按写入方分组、非穷举（全局布局与开关语义见 `docs/review/dsh-workflow-and-readability-20261109.md` §9）：

| 写入方 | 文件 | 默认写盘 | 机制 / 含义 |
| --- | --- | --- | --- |
| `ros_node.py` | `navigation_status.json` | ✅ | `atomic()`：状态快照，每次 publish 覆盖（`ros_node.py:131`） |
| `ros_node.py` | `navigation_status.jsonl` | — | `append()`：逐条状态历史（`ros_node.py:123-124`） |
| `ros_node.py` | `navigation_guard_history.jsonl`、`navigation_guard_continuity_history.jsonl` | — | `append()`：native guard 审计/连续性证据（`ros_node.py:673`、`:809`） |
| `ros_node.py` | `navigation_guard_writer_receipt.json` | ✅（仅 `dynamic_fixture` profile） | `write_text()`（`ros_node.py:852`） |
| `ros_node.py` | `navigation_slam_poses.jsonl`、`navigation_cloud_history.jsonl` | — | `append()`：实测位姿/点云历史（`:486`、`:536`） |
| `ros_node.py` | `navigation_trajectories/` | — | `enqueue(archive)`：压缩轨迹 NPZ + 同名 JSON（`:575`） |
| `pid_route.py` | `navigation_region_passes.jsonl`、`navigation_segment_activation.jsonl`、`navigation_heading_projection.jsonl`、`navigation_coverage_replans.jsonl`、`navigation_global_reference_requests.jsonl` | — | `append()`：区域通过、段激活、航向投影与重规划证据 |
| `pid_admission.py` / `pid_admission_apply.py` | `path_admission_decisions.jsonl`、`reference_replan_events.jsonl`、`navigation_cascade_paths.jsonl` | — | `append()`：准入决策与"候选 spline vs 冻结路线"证据 |
| `pid_control.py` | `navigation_controller_errors.jsonl`、`navigation_fixed_goal_references.jsonl`、`ordinary_preT0_attempts.jsonl`、`ordinary_terminal_failure_history.jsonl` | — | `append()`：控制器错误、固定目标参考与普通停车证据 |
| `pid_control.py` | `cascade_command_source.json` | ✅ | `tmp.write_text()` + `tmp.replace()`：命令源原子快照（`pid_control.py:1019-1022`） |
| `pid_ingress.py` | `navigation_cloud_xyz.jsonl`、`navigation_cloud_arrays/` | — | 点云归档写在 `enqueue(archive)` 闭包里（`pid_ingress.py:448-455`） |
| `pid_ingress.py` | `navigation_feedback_history.jsonl`、`navigation_cloud_callbacks.jsonl`、`corridor_inputs.jsonl`、`corridor_certificates.jsonl`、`corridor_input_rejections.jsonl` | — | `append()`：点云反馈、回调与走廊输入证据 |
| `pid_support.py` | `navigation_imu_history.jsonl`、`navigation_pid_history.jsonl` | — | `append()`/`append_pid_snapshot()`/`enqueue(write)`，两条路径都不落盘 |
| `pid_support.py` | `teacher_control_substage_timing.json` | ✅（仅 profile 选择 `r74_teacher_substage_timing`） | `write_exclusive()`（`pid_support.py:498-502`）；失败时写 `.flush_error.json` |
| `pid_entrypoint.py` | `navigation_pid_writer_receipt.json` | ✅ | `write_text()`（finally，`pid_entrypoint.py:236`） |
| `pid_entrypoint.py` | `r88_async_physical_exit.json` | ✅（仅收据非 None） | `write_text()`（`pid_entrypoint.py:225`） |
| `pid_entrypoint.py` | `corridor_worker_receipt.jsonl`、`corridor_certificates.jsonl`、`corridor_input_rejections.jsonl` | — | `append()`：exit 前 flush 的走廊收据 |
| `base_node.py` | `scan_route_definitions.jsonl`、`r85_reference_attempts.jsonl`、`r85_reference_holds.jsonl` | — | `append()`：路线定义与 r85 参考尝试/保持证据 |
| `building_navigation.py`（Building 路径） | `navigation_history.jsonl`、`navigation_slam_poses.jsonl`、`accepted_SCAN_curves.jsonl`、`actual_command_ack.jsonl`、`native_pose_source.jsonl` | ✅ | 真实文件句柄（`open("w")`/`open()`），只在 Building run 目录出现 |
| `building_navigation.py`（Building 路径） | `parking_10Hz_result.json`、`truth_navigation_status.json` | ✅ | 模块内 `atomic()`（临时文件 + `replace`） |
| `relative_slam_node.py`（独立入口） | `result.json`、`stop_request.json` | ✅ | `write_text()` 到 `--output`，不一定在 run 目录 |

两份常见但**不是本目录写**的文件：`mission46_status.json` 与 `closed_loop_executor_ack.json` 在本包只有读点（`pid_control.py:571`、`:209`）；前者在本检出的 Python 源码里没有写入点（推测由原生 SCAN 侧产生，未核实），后者由 `robot/unitree/policy/worker.py:250` 的 `atomic_ack_rows` 写。`navigation_anchor.json` / `navigation_request.json` / `global_reference.json` 也不是本目录写的：由 `mission/mission_runtime.py` 冻结/发布（见 `mission/README.md`）。

### 配置

本包**没有直接读 `configs/` 的代码**（`grep -rn "configs/" src/embodied_navigation/navigation/*.py` 无命中）。实际读取点是：

| 来源 | 读取点 | 说明 |
| --- | --- | --- |
| run 内 `runtime_context.json` 的 `profile` | `ros_node.py:145`（`load_context`）、`mission/mission_runtime.py:34` | 业务阈值/开关都经 prepare 生成后 pin 进 profile；读取顺序与 pin 规则见 [`../../../configs/README.md`](../../../configs/README.md) |
| ROS 参数 | `base_node.py:133-140`：`max_speed`、`arrival_radius`、`pose_timeout`、`cloud_timeout`、`scenario_path` | 启动时 `declare_parameter` 读一次；`scenario_path` 默认指向 `../simulation/scenario.json`，该文件只取 IMU 固定旋转 |
| run 内其它 JSON | `global_route.json`、`navigation_request.json`、`experiment_setup.json`、`PCT_route.json`、`sensor_contract.json` | 分别是路线/请求/场景/路线/传感器契约；**跨进程文件契约**，不是配置 |
| profile 中的契约键 | `continuous_route_contract.enabled`（`mission/continuous_route.py:42`，当前 `false`）、`r85_auto_reference_contract`、`corridor.mode` | `prepare.py:72-76` 把 `r85_auto_reference_contract.reference_file` pin 成 `global_reference.json`；`cascade.daily_navigation` 也被 pin |

没有读取点的常见误判：改 `configs/control/settings.json` 的 cascade 值不会经过本包直接生效，它同样经 prepare 进 profile。

## 模块内文件耦合

依赖方向总体是单向的：`pid_support` ← 5 个 `*Mixin`（ingress/admission/admission_apply/route/control）← `pid_navigation` ← `pid_entrypoint`；节点层 `base_node` ← `ros_node` ← `pid_support`。**没有 import 环**，但有两处需要说明：

- `pid_navigation.py` 顶层 import `pid_entrypoint._main/main`，而 `pid_entrypoint.main()` 在函数体内 `from .pid_navigation import PIDNavigation`（`pid_entrypoint.py:295-297`，docstring 明写 "import the class lazily to avoid an import cycle"）——环靠延迟 import 规避，不是天然无环。
- `pid_support.py` import `ros_node.TeacherNavigation`（`pid_support.py:46`），即"助手模块"反过来依赖节点类，只为让 `pid_navigation` 从一处取基类。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 1 | `task` | `ros_node`（包级导入） | 清楚 |
| `task.py` | 346 | `control.collision`、`control.relative_pid` | `navigation/__init__`、`relative_slam_node`、`localization_mapping.relative_odometry` | 清楚（纯状态机，无 ROS） |
| `relative_slam_node.py` | 172 | `task`、`control.collision`、`localization_mapping.relative_odometry` | 无（console script 入口） | 清楚 |
| `base_node.py` | 1617 | 14 个跨层模块（common/control/localization_mapping/perception/planning）+ `mission.goal_regions` | `ros_node` | 偏重（上帝类） |
| `ros_node.py` | 874 | `base_node`、`mission.dynamic_stop_event`、control/common 多模块 | `pid_support` | 偏重（节点包装承担 12 处证据写点，默认只有状态快照真落盘） |
| `pid_support.py` | 1030 | `ros_node` + 24 个内部模块 | 5 个 `*Mixin` + `pid_navigation` + `pid_entrypoint` | 可疑（导入枢纽：7 个文件依赖它，且自身 42 个函数（含嵌套）混杂） |
| `pid_ingress.py` | 454 | `pid_support`、`common.observation`、`localization_mapping.feedback_state`、`perception.sensors.feedback_mailbox`、`planning.reference.spatial_reference` | `pid_navigation` | 清楚 |
| `pid_admission.py` | 616 | `pid_support` | `pid_admission_apply`、`pid_navigation` | 清楚 |
| `pid_admission_apply.py` | 891 | `pid_admission`、`pid_support`、`planning.reference*` | `pid_navigation` | 可疑（852 行单函数，见下） |
| `pid_route.py` | 301 | `pid_support` | `pid_navigation` | 清楚 |
| `pid_control.py` | 1173 | `pid_support`、`control.parking.*`、`control.safety.*`、`control.publication_ledger`、`mission.mission46_guard` | `pid_navigation` | 偏重 |
| `pid_navigation.py` | 249 | 5 个 `*Mixin` + `pid_support` + `pid_entrypoint` + `planning.admission.physical_gate` | `pid_entrypoint`（延迟） | 清楚（门面职责单一） |
| `building_navigation.py` | 650 | `control.parking.*`、`control.safety.contact_mailbox`、`control.tracking.*`、`common.project` | 无（子进程 `-m`） | 可疑（import 即执行） |

四类问题：

1. **上帝文件（>800 行）**：本目录 5 个 —— `base_node.py` 1617、`pid_control.py` 1173、`pid_support.py` 1030、`pid_admission_apply.py` 891、`ros_node.py` 874；另有 `building_navigation.py` 650 行，虽未过 800 线，但它是单文件单类的模块级脚本（见第 3 类）。行数来自 `wc -l`。
2. **重复实现**：两处同名 `class Navigation`（`base_node.py:108` 与 `building_navigation.py:68`）——同名不同类，靠文档区分，容易误读；`main()` 有 4 份（`base_node.py:1594`、`ros_node.py:829`、`pid_entrypoint.py:294`、`pid_navigation.py:243`），但语义不同（前两个是独立历史入口）。
3. **跨层直达**：`pid_support` 直接 import 节点层 `ros_node`（向上依赖）；`pid_ingress` 直接读 `localization_mapping.feedback_state`（本地化内部状态）而不是走公共 observation 契约；`base_node.py:139` 直接按路径读 `src/embodied_navigation/simulation/scenario.json`。
4. **隐式契约**：run 目录文件名/JSON 键是主要跨进程契约（`navigation_request.json` 被 `ros_node.py:581` 与 `simulation/terrain/*` 读，`navigation_anchor.json` 被 `planning/admission/path_admission.py:93` 读），没有 schema 版本常量集中定义；`base_node` 与 `ros_node` 各自写 `navigation_guard_history.jsonl`（同名文件两个写入方）。

结论：**耦合方向清楚但分层偏浅**——mixin 拆分确实把 1173/1030/891 行的实现从门面里搬出来了，`pid_navigation.py` 只剩 249 行组合代码；但没有真正解耦职责，`pid_support` 变成"什么都依赖、什么都被依赖"的枢纽，`pid_admission_apply` 把 852 行压进一个函数。适合当前单人维护，不适合多人并行改 controller。

## 可读性评估

分级依据为实测 AST 指标（函数总数 / 有 docstring 函数比 / 最长函数 / >80 行函数数 / 注释行），并抽查了 `control()`（L1077–1493）与 `publish_command`（L852–1173）的实际长度复核，指标可信。

**良好**

- `pid_route.py`：301 行 / 12 函数 / 0 个 >80 行函数，12 个函数都不超过 65 行（最长 `follow_checked_trajectory` L25–89，65 行），结构清晰；扣分项在文档（docstring 1/12），不影响读结构。
- `__init__.py`：1 行重导出，无歧义。

**可接受**

- `task.py`：346 行 / 3 函数 / docstring 3/3，命名与中文注释明确；唯一问题是 `step()`（L136–346，211 行）过长。
- `pid_navigation.py`：门面本身仅 249 行，docstring 2/3；`__init__` 134 行含 57 处 `self.* =` 字段赋值，属"组装代码"可接受，但 L203–L221 把走廊观察者 19 个几何/阈值写成字面量，未进 profile。
- `ros_node.py`：22 函数 / docstring 7/22，最长 `record_native_guard` 101 行（L686–786），3 个 >80 行函数；状态与证据逻辑集中在 `RecordedPublisher`，可读。
- `pid_ingress.py`：10 函数 / 最长 `on_odom` 88 行（L241–328）；无 docstring，但回调命名直白。
- `pid_admission.py`：19 函数 / docstring 3/19，最长 `control_admission_hold` 126 行（L491–616）。

**需改进**

- `base_node.py`：39 函数只有 6 个 docstring（15%），`control()` 417 行（L1077–1493）一个函数承担 stale 判定、tilt guard、到点、转向、停车与失败；文件 1617 行同时做状态机、传感器超时、证据记录与命令发布。
- `pid_admission_apply.py`：`_admit_matched_spline`（L40–891）**单函数 852 行**、0 docstring；这是全目录最长的单个可执行单元，任何准入改动都要在一个函数里定位。
- `pid_control.py`：12 函数 0 docstring，`publish_command`（L852–1173）322 行、`select_pid_velocity`（L237–515）279 行；4 个 >80 行函数。
- `pid_support.py`：导入枢纽，且**文件中部还有 import**——import 分成三段：L9–108（主体）、L255（单独一条 `import sys`）、L1011–1029（`goal_regions`/`continuous_route`/`path_commit`/`pending_path_commit`/`navigation_stop_window`）；读者很难一眼判断完整依赖面。
- `relative_slam_node.py`：7 函数 0 docstring，`main()` 153 行（L16–168）把 argparse、建节点、循环、收尾写在一起。
- `building_navigation.py`：`import` 即执行（模块级读 4 个 run 文件、`rclpy.init()`、`os.environ["DEMO_RUN_DIR"]` 无兜底），因此无法在无 run 目录的进程里导入做静态检查或单测；`tick()`（L435–635）201 行。

区分说明：`base_node.py`/`pid_admission_apply.py`/`pid_control.py`/`pid_support.py` 的问题是**结构**（长函数、导入枢纽、门面之外没有稳定边界）；`task.py`、`pid_ingress.py`、`pid_admission.py` 的问题是**文档**（函数规模可接受但缺 docstring），不要混为一谈。

## 重构建议

本轮只写建议、不改代码；任何一条都要"先有对照/回归再动"。**没有 P0**：默认主路径的关键开关是 fail-closed（例如 `pid_navigation.py:188-192` 对未验证走廊控制权直接 `ValueError`，`building_navigation.py:42-45` 拒绝非 SLAM 模式），且未发现 `except: pass` 静默吞异常（grep 无命中）。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `_admit_matched_spline`（`pid_admission_apply.py` L40–891）按"候选评估 / 准入决策 / 收据写入"拆成 3–5 个方法 | 852 行单函数不可局部修改，是准入逻辑的主要维护成本 | 中：准入顺序与证据行序敏感，拆错会改行为 | 先跑 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`；再用已有 run 目录做一次离线回放对照 `path_admission_decisions.jsonl` |
| P1 | 把 `base_node.Navigation.control()`（417 行）按相位拆成 `_control_stale/_control_arrival/_control_heading` 等 | 417 行一个函数同时管超时、tilt、到点、转向、停车、失败，改一处要读全函数 | 高：这是默认任务状态机，`tests/unit/test_navigation.py` 只覆盖 `task.py`，没有对应行为单测 | 拆分前先补 `base_node` 状态转移的单测或基于 run 目录 replay 的对照 |
| P1 | 给 `pid_control.publish_command`（322 行）/`select_pid_velocity`（279 行）补 docstring 并抽出停车/终端分支 | 0 docstring + 322 行发布函数，命令发布是安全相关路径 | 中 | 先补文档再动结构；改结构后跑 unit/regression 全量 |
| P1 | 把 `pid_support.py` 中部的 import（L255、L1011–1029）移到文件头 | 两段 import 区让模块依赖面不可一眼确认，也会放大循环 import 风险 | 低（纯移动） | `python3 -c "import embodied_navigation.navigation.pid_navigation"` + `tests/unit/test_pid_navigation_compat.py` |
| P2 | 给 `pid_ingress`/`pid_admission`/`pid_control`/`pid_route` 的公开方法补一行 docstring | 目前 0–26% docstring 比；方法名已较直白，补文档性价比高 | 无 | 只改文档，不跑行为测试 |
| P2 | 把走廊观察者字面量（`pid_navigation.py` L203–L221）移进 profile 或常量块 | 19 个魔法数散在 `__init__`；当前仅 `shadow` 模式可用，风险有限 | 低（需确认无 pin 冲突） | `grep` 确认唯一读取点 + 启动一次 shadow 模式对照 |
| P2 | 把两处同名 `class Navigation` 之一改名（`building_navigation.Navigation` → `BuildingNavigation`） | 同名不同类是两个入口最易混的点，改名零行为风险 | 低 | 同步 `launch/building.py:354`、`robogauge/benchmark.py:118` 的 `-m` 路径（仅注释/文档）+ 全仓 grep |

## 入口、节点类与进程角色

| 入口/进程 | 节点类 | 启动者 | 在默认 `demo.launch.py` 中？ |
| --- | --- | --- | --- |
| `controller` 角色 | `pid_navigation.PIDNavigation`（由 `pid_ingress`/`pid_admission`/`pid_admission_apply`/`pid_route`/`pid_control` 组合）→ `TeacherNavigation` → `base_node.Navigation`；进程入口 `pid_entrypoint` | `../control/launch/components.py` | 是；任务状态机是 `base_node.Navigation.control()` |
| `mission` 角色 | `mission.mission_runtime.Mission`（文件内定义，非 `mission/state_machine.py`） | `../control/launch/components.py` | 是；只冻结/发布目标与参考，不产生速度 |
| Building 节点 | `building_navigation.Navigation`（同名不同类） | `../launch/building.py`、`robogauge/benchmark.py` | 否 |
| relative-SLAM 入口 | `task.py` 状态机 + `relative_slam_node.py` | console script `relative-slam-navigation`（`pyproject.toml`） | 否，独立入口 |

注意 `state_machine.py`（`mission/`）在本检出中没有实例化点；`continuous_route_contract.enabled=false`。四条路径的总览见 [`../../../docs/architecture.md`](../../../docs/architecture.md)，术语见 [`../../../docs/glossary.md`](../../../docs/glossary.md)。

## 关键文件

- `pid_navigation.py`：现役 controller 入口（兼容导入/`-m` 入口）。`PIDNavigation` 的 `__init__` 与类组合在此；方法体按职责拆到同目录模块：
  - `pid_ingress.py`：请求/走廊/IMU/odom/cloud 订阅回调与归档；
  - `pid_admission.py` + `pid_admission_apply.py`：准入生命周期与"候选 spline vs 冻结路线"的大决策体；
  - `pid_route.py`：checked trajectory 执行、连续进度、规划与 bundle 参考；
  - `pid_control.py`：cascade/PID 速度选择、ACK、停车/终端保护与命令发布；
  - `pid_support.py`：共享证据/探针助手与导入；
  - `pid_entrypoint.py`：rclpy 初始化、executor drain、进程 `main`。
  该 facade 还按旧导入路径重导出原模块的 27 个 helper 以及 `_drain_executor`/`_main`（`_main` 保留 `(shutdown_signals)` 原签名），并使用 absolute imports，因此旧的 `python .../pid_navigation.py --run ...` 直接脚本入口和 `-m` 入口都保留；实现仍在上面的职责模块。`../control/launch/components.py` 仍以 `--run <run_dir>` 启动 `embodied_navigation.navigation.pid_navigation`。
- `ros_node.py`：`TeacherNavigation(base.Navigation)`，Teacher 的有限包装，拒绝重复或倒退的来源 header，并要求实测 SLAM/点云。
- `base_node.py`：`Navigation` 基类，做 measured-pose SCAN 执行、freshness watchdog 与 LiDAR 停/恢复；自身不设仿真位姿、不订阅真值。
- `task.py`：现有 shuttle 任务状态机（等待、跟随、settle、转向、停止/失败），与 ROS、仿真解耦；**只在独立 `relative-slam-navigation` 入口使用，不在 `demo.launch.py` 中**；驾驶指令来自 `../control/relative_pid.py`。
- `relative_slam_node.py`：相对 SLAM 任务的 ROS 适配器，也是 pyproject 的 `relative-slam-navigation` 入口；只有显式 `main()` 才创建节点。
- `building_navigation.py`：Building 分支节点（canonical FAST-LIVO2，只允许 SLAM），读 run 目录的 `PCT_route.json`，由 `../launch/building.py` 与 `robogauge/benchmark.py` 启动。
- `mission/` 见 `mission/README.md`；`robogauge/` 见 `robogauge/README.md`。

## 测试

`../../../tests/unit/test_navigation.py` 只覆盖 `task.py` 的状态与失败条件；`../../../tests/unit/test_building_entry_closure.py` 覆盖 Building 入口只接受 SLAM 的准备逻辑。facade 的导入/脚本入口兼容由 `../../../tests/unit/test_pid_navigation_compat.py` 覆盖（27 个 helper 可导入 + 直接脚本与 `-m` 两种 `--help`）；`pid_entrypoint.py` 的关停路径由 `../../../tests/unit/test_ros_shutdown_lifecycle.py` 解析源码验证；`building_navigation.py` 的 SLAM 模式门由 `../../../tests/unit/test_production_slam_mode.py` 用 AST 抽片段 `exec` 验证（**改这个文件的结构可能先影响测试而不是行为**）。`base_node.py`、`ros_node.py`、`pid_*.py` 的控制器行为没有专用单测，只随运行链路使用。测试入口见 [`../../../tests/README.md`](../../../tests/README.md)。

## 限制

完整路线、Building 实际运行与实机均未验证。完整原生闭环依赖 SCAN/FAST-LIVO2 构建产物；仓库根 install 缺产物时仍可阅读源码并运行不依赖这些产物的单元测试，不能据此验证完整闭环。此外：

- `base_node.Navigation.control()` 是默认任务状态机，但本目录没有它的行为级单测（见上节），改动它只能靠运行链路观察。
- `building_navigation.py` 无法被安全导入（模块级 `rclpy.init()` + 必需 `DEMO_RUN_DIR`），其行为只能通过 `-m` 子进程或源码级测试验证。
- 走廊控制权当前只有 `off`/`shadow` 两种模式可用，`retain_valid_plan` 会被拒绝（`pid_navigation.py:188-192`）；不要把它当作已验证的控制通道。
