# navigation/robogauge

本目录是 RoboGauge 仿真部署的导航侧：fail-closed 的命令准入、bounded Building 基准编排与离线证据审计；它不是主相对 SLAM 导航任务。

## 边界

- 做：把 run 内导航命令按同 run、边界与新鲜度准入（`commands.py`）；准备、预检与显式运行 Building 基准（`benchmark.py`）；离线审计同 stamp 的执行器证据（`analyze.py`）；独立的 native-pose 路线跟踪辅助类（`route_tracker.py`，有回归用例；当前 Building 基准启动 `building_navigation`）。
- 不做：策略训练、45D 观测构造与执行器 socket。它们分别在 `../../robot/unitree/robogauge/` 与 `../../simulation/gazebo/robogauge/`。

## 目录结构

```text
src/embodied_navigation/navigation/robogauge/
├── README.md          # 本文件（模块说明；行数随改写变化，不标注）
├── __init__.py        # 1 行，只有包 docstring（命令准入/路线跟踪/离线证据审计），无重导出
├── analyze.py         # 172 行，离线同 stamp 执行器证据审计 CLI
├── benchmark.py       # 462 行，bounded Building 基准编排（--check / --prepare-only / --run）
├── commands.py        # 47 行，fail-closed 的同 run 导航命令准入
└── route_tracker.py   # 329 行，纯 measured-pose 路线跟踪器（无 ROS/仿真副作用）
```

## 代码、日志与配置

### 代码

| 文件 | 性质 | 判断依据 |
| --- | --- | --- |
| `commands.py` | 活代码（库） | `admit_command`/`read_command`（`commands.py:9`、`:35`）；被 `simulation/gazebo/robogauge/worker.py` 使用 |
| `route_tracker.py` | 活代码（库） | `class RouteTracker`（`route_tracker.py:18`）；被 `simulation/dynanav/capture/control.py` 与 `simulation/dynanav/slam_route.py` 使用，也是回归用例的对象 |
| `benchmark.py` | 活代码（进程入口） | `main()`（L381）按 `--check`/`--prepare-only`/`--run` 三段式执行；自管 `Popen`；无其它模块 import 它 |
| `analyze.py` | 活代码（离线 CLI） | `main()`（L157）读 run 里的 JSONL 并输出收据；返回码 0=全通过、2=失败 |
| `__init__.py` | 只有 docstring | 88 字节，无 re-export；调用方必须写完整模块路径 |

### 日志与输出

本目录**不写自己的日志**：5 个文件里 `getLogger()` 为 0；`print` 只有 `analyze.py` 1 处（CLI 收据）与 `benchmark.py` 3 处（准备/运行收据）。数据面都在**运行产物**里，且都属于路径 C（RoboGauge benchmark），不写进 `experiments/`：

| 位置 | 写入方 / 消费方 | 含义 |
| --- | --- | --- |
| `<run>/telemetry.jsonl`、`<run>/actuator_physics.jsonl` | 由 actor 侧写；`analyze.py:56-57` 消费 | 每行带 `kind`、`t`（秒）与 `q_target`/`qtarget`/`pd_raw`/`qd`/`tau`；`_index()` 按四舍五入的纳秒 stamp 建索引 |
| `<run>/truth_navigation_status.json`、`<run>/worker_result.json` | actor/worker 侧写；`analyze.py:107-108` 消费（文件不存在则为 `None`） | 导航状态与 worker 结果，只作审计输入 |
| `<run>/launch_plan.json` | `benchmark.py:430` 写 | 准备阶段落盘的 argv 计划，`scope="Prepared only; not execution evidence"` |
| `<run>/` 下 `sensor_bridge.yaml`、`building_fastlivo.json`、`building_camera.json`、`scan_params.json`、`world.sdf`、`backend_*` | `benchmark.py:process_commands()` 引用；由 `simulation/gazebo/robogauge/world.prepare()` 生成 | 基准的进程图输入；**准备文件不是执行证据** |
| `/tmp/{building_navigation,go2_pid_physics_serial,go2_model_pid_simulation,go2_navigation_dispatch}.lock` | `benchmark.py:415-422` 获取 `flock` | 与 Building/native 运行互斥的 advisory lock |
| 分析器输出 | `analyze.py --output` | JSON 收据；判定阈值：同 stamp `q_target` 最大绝对差 ≤ 1e-12、Go2HV 包络内 torque 差 ≤ 1e-9（包络 20.2/23.4 Nm，13.5–30 rad/s 线性降额，见 L96-99） |

`commands.py` 本身不写文件：它只读一份命令 JSON（`read_command`），失败时返回零命令 + 原因字符串。

### 配置

本目录**没有直接读 `configs/` 的代码**（`grep -rn "configs/" src/embodied_navigation/navigation/robogauge/*.py` 无命中）。间接读取点只有一处：

| 配置 | 读取点（不在本目录） | 说明 |
| --- | --- | --- |
| `configs/robogauge/`（`CONFIG`）、`configs/scenarios/building/scan.json`、`configs/localization_mapping/{fastlivo,camera}.yaml` | `simulation/gazebo/robogauge/world.py:9,93,99,124` | `benchmark.py:23-29` 从该模块 import `prepare`/`CONFIG`/`write`，因此**基准配置的读取点属于 simulation 层**，改这些配置的影响面也在那边 |
| 环境变量 | `benchmark.py:38` | `NAVIGATION_ACTOR_PYTHON` 覆盖 actor 解释器；缺省用当前解释器 |
| run 内 `experiment_setup.json` | `benchmark.py:41` | 取 `spawn_xyz_yaw` 前三维作 actor 初始位姿 |

`commands.py` 的限值不是配置：形状 `(3,)`、上限 `[0.6, 0.2, 0.3]`、sim 龄 `[-0.05, 0.3)` s、墙钟龄 `[0, 0.3)` s 都是代码常量（`commands.py:12-30`）。

## 模块内文件耦合

依赖方向：`commands.py` 与 `route_tracker.py` 是**零内部依赖的纯库**（只依赖标准库/NumPy），`analyze.py` 也几乎如此（只依赖 NumPy）；`benchmark.py` 是唯一的重编排模块，向上依赖 `robot/unitree/robogauge/runtime` 与 `simulation/gazebo/robogauge/world`。四个文件之间**互不 import**——耦合全部通过 run 目录文件与进程发生。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 1 | 无 | 无 | 清楚 |
| `commands.py` | 47 | 标准库 + NumPy | `simulation/gazebo/robogauge/worker.py` | 清楚（最小 fail-closed 契约） |
| `route_tracker.py` | 329 | 标准库（`math`）+ `wrap`/`clip` lambda | `simulation/dynanav/capture/control.py`、`simulation/dynanav/slam_route.py` | 清楚（纯函数式，无副作用） |
| `analyze.py` | 172 | 标准库 + NumPy | 无（离线 CLI） | 清楚 |
| `benchmark.py` | 462 | `common.communication.dds_environment`、`common.config`、`common.runtime_environment`、`project`、`robot.unitree.robogauge.runtime`、`simulation.gazebo.robogauge.world` | 无（子进程编排入口） | 偏重（462 行里 `run()` 128 行管理 8 个 Popen 与关停；也是本目录唯一跨层模块） |

四类问题：

1. **上帝文件（>800 行）**：无，最大 `benchmark.py` 462 行。
2. **重复实现**：无。四个文件职责不重叠；`route_tracker.py` 的 `wrap`/`clip` 是两行 lambda，与 `control/tracking` 的同名助手无共享，但只在本文件内使用，不构成跨层重复。
3. **跨层直达**：`benchmark.py` 直接 import `simulation.gazebo.robogauge.world` 的 `prepare`/`CONFIG` 并 `Popen` 一长串原生二进制（`install/ros/...`、`fastlivo_mapping`、`scan_planner_node`），把"基准编排"和"仿真世界准备"绑在一起；这是路径 C 的既有设计，不是意外。
4. **隐式契约**：`commands.py` 的命令 JSON 键（`record["command"]`、`["stamp_ns"]`、`["monotonic_wall"]`、`["run_id"]`）由 worker 侧写、本模块读，没有共享 dataclass；`analyze.py` 依赖 JSONL 行里的 `kind` 字符串与 `t` 秒值约定（`_index` 用 `round(float(row["t"]) * 1e9)`），这些约定只写在 docstring 里。

结论：**耦合清楚且方向合理**——三个纯库不 import 任何内部模块，唯一的重模块 `benchmark.py` 是显式编排入口、不被别人 import，所以本目录的可读性问题主要落在 `benchmark.py` 单文件内部，而不是文件之间。反向依赖（`simulation/` import 本目录的 `commands`/`route_tracker`）说明这两个库已是跨路径共享契约，改动它们的影响面比目录结构看起来大。

## 可读性评估

指标为实测 AST 值（函数总数 / docstring 比 / 最长函数 / >80 行函数数），已抽查 `commands.admit_command`（24 行，含完整门禁 docstring）与 `route_tracker.RouteTracker.__init__`（带参数语义 docstring）确认可信。

**良好**

- `commands.py`：47 行 / 2 函数 / docstring 1/2 / 最长 24 行（`admit_command`，L9–32）；`admit_command` 的 docstring 把四类门禁与上限写清楚了，是"小文件写清楚"的范例。
- `analyze.py`：172 行 / 5 函数 / docstring 4/5 / 最长 108 行（`audit`，L47–154）；`audit()` 的 docstring 给出比较对象与容差，阈值以注释说明来源（Go2HV 包络）。

**可接受**

- `route_tracker.py`：3 函数 / docstring 3/3，但 `tick()` 212 行（L118–329）是单一路径控制律，线性分支多；类与模块 docstring 已明确"无副作用、调用方负责位姿来源"。
- `benchmark.py`：8 函数 / docstring 1/8，`run()` 128 行（L251–378）、`process_commands()` 负责 8 个进程 argv；结构是"一屏命令表 + 一个等待循环"，读起来长但不绕。

**需改进**

- 无。本目录没有需要重构的坏味道；`benchmark.py` 的长函数属于编排代码的固有形态，且它不被其他模块 import，爆炸半径限于本入口。

## 重构建议

本轮只写建议、不改代码。**没有 P0/P1**：三个库都是 fail-closed（`commands.admit_command` 失败抛 `ValueError`，`read_command` 返回零命令+原因；`analyze.py` 对非法证据抛错且返回码非 0），`benchmark.py` 把准备与运行分开（`--prepare-only` 与 `--run` 互斥，见 `benchmark.py:401-402`），且准备文件明确标注不是执行证据。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `benchmark.py` 的 `run()`/`process_commands()` 补 docstring，写明每个角色的进程与退出清理顺序 | docstring 1/8，读者只能从 argv 列表反推进程图 | 无（纯文档） | 只改文档 |
| P2 | 把 `benchmark.py:23-29` 从 `simulation.gazebo.robogauge.world` import 的 `prepare`/`CONFIG` 依赖在本 README 与源码注释里显式标注为"配置读取点在 simulation 层"（本 README 已写） | 否则会有人以为改 `configs/robogauge/` 的读取点在本目录 | 无 | 交叉阅读 `configs/README.md` 的配置消费者表 |
| P2 | 给 `analyze.py` 的 `main()` 补 docstring，说明返回码语义（0=全通过，2=失败）与 `--output` 的用途 | 返回码决定脚本是否被当作证据，CLI 契约应写明 | 无 | 只改文档 |

## 45D 与 247D 是两条链

RoboGauge 的 45D 是本部署的直接观测基准接口（`../../robot/unitree/robogauge/contract.py` 的 `observation()`，输入 state64/command3/last_action12）；既有 native 适配链还会构造 Teacher 的 247D 观测。当前默认 Student 的实际推理输入也是 45D（`../../robot/unitree/policy/student.py`），247D 在该 Student 路径仅作诊断，不能把 247D 称作 Student 的实际输入。两者共享同一仿真安全门禁，但不是同一策略输入或 actuator 契约，不能互相替代。

## 关键文件

- `commands.py`：`admit_command`/`read_command` 校验 `run_id`、命令形状与界（`[0.6, 0.2, 0.3]`）、源龄 `< 0.3 s`、接收墙钟龄 `< 0.3 s`；`admit_command` 对非法记录抛异常；`read_command` 捕获列出的读取/校验异常，返回零命令并附原因。
- `benchmark.py`：`--check` 只做文件/hash 预检，`--prepare-only` 只写 CPU 准备文件，`--run` 才启动 ROS/Gazebo；编排 actor、sensor bridge、FAST-LIVO2、backend、localization、`building_navigation`、SCAN 与 Gazebo。
- `analyze.py`：离线审计 `telemetry.jsonl` 与 `actuator_physics.jsonl` 的 policy exchange/physics step；推理 receipt 不等于实际运动。
- `route_tracker.py`：`RouteTracker` 纯控制器，没有仿真或策略副作用。

## 测试

`../../../../tests/regression/test_robogauge_contract.py`、`test_robogauge_evidence.py`、`test_robogauge_route_tracker.py`、`test_robogauge_benchmark.py` 分别覆盖命令准入/native 契约、证据审计、路线跟踪与基准准备；`test_robogauge_native_loop.py` 覆盖 robot/simulation 侧的 native 循环。

## 限制

仓库根的 install 构建产物缺失只表示未构建，不代表缺源码。CPU 预检不是执行证据；完整 Isaac 运行、GPU 与实机均未验证，历史 N=1 结果不是可靠性统计。详见 `../../../../docs/robogauge/README.md`。另外：`benchmark.py` 的 `process_commands()` 在准备阶段把 socket 路径写成占位的 `/tmp/robogauge_<allocated>.sock`（L438），实际运行时的分配在 `run()` 内完成；不要把 `launch_plan.json` 里的占位路径当作可执行计划。
