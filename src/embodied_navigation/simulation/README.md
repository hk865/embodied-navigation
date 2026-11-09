# simulation

本目录是仿真与场景生命周期层：生成 Gazebo/Isaac 场景、启动物理与传感器、维护仿真安全状态与真值流，并承载 RoboGauge/DynaNav 两类仿真部署；它不实现导航算法。

## 边界

- 仿真真值日志和 ground truth 路径用于离线评测；native64 帧还用于现有特权步态观测、COM 速度内环与仿真安全门禁，**不作为 SLAM/导航定位输入**。[safety_state.py](safety_state.py) 产出的安全状态明确标为 `simulation_truth` + `simulation_safety`（`StatePurpose.SIMULATION_SAFETY`）。
- 不承载整个导航系统；`dynanav` 负责 DynaNav/Isaac 场景接入、真实 Go2 RGB-D capture，以及该实验路径的 oracle/learned 建图、实测位姿路线控制和动态行人模块（见 `dynanav/README.md`）。
- Isaac/DynaNav 已有分段与部分完整路线实验记录（见 `docs/experiments/`），但**生产整栈与实机未验收**；不能把仿真准备成功当作整栈完成。

## 目录结构

```text
src/embodied_navigation/simulation/
├── actuators/          # 子模块，见 actuators/README.md（Gazebo 关节力矩插件 C++）
├── dynanav/            # 子模块，见 dynanav/README.md（Isaac/DynaNav 实验路径）
├── gazebo/             # 子模块，见 gazebo/README.md（Gazebo 部署归置，含 robogauge/）
├── launch/             # 子模块，见 launch/README.md（仿真进程装配）
├── terrain/            # 子模块，见 terrain/README.md（受控 Actor 地形切换，默认关闭）
├── world/              # 子模块，见 world/README.md（世界/传感器契约生成）
├── README.md           # 本文件
├── __init__.py         # 1 行，包 docstring（"Simulation and scene lifecycles, independent of navigation algorithms."）
├── mesh_terrain.py     # 72 行，Building.dae 三角网格的批量垂直射线（仅特权观测用，非定位参考）
├── safety_state.py     # 73 行，Gazebo native64 → PhysicalSafetyState（simulation_truth / simulation_safety）
├── truth_worker.py     # 208 行，SLAM-only Student 仿真 actor 适配器（沿用 truth 历史 wire 名）
└── wait_actor.py       # 24 行，actor 就绪屏障：worker_ready 的 PID 存活 + Unix socket 已监听
```

6 个子目录 + 5 个顶层 Python 文件（合计 378 行，最大 `truth_worker.py` 208 行）。子目录各自的目录树与耦合见对应 README，此处不展开。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `__init__.py` | 包 docstring 一行，无 import、无副作用；不重导出任何符号。 |
| 代码 | `safety_state.py` | 活代码：`from_gazebo_native64(native_state, *, first_world_time_s, ground_height_m, run_id)` 把 64 维帧校验成 `PhysicalSafetyState`（roll/pitch、`body_clearance_m`、5 组接触、执行器 fault code），`sample_stamp_ns = (state[0] - 0.005)·1e9` 明确"读上一物理步"。消费者：`robot/unitree/policy/native/worker.py` 与 `gazebo/robogauge/native_loop.py`。 |
| 代码 | `mesh_terrain.py` | 活代码（实验用）：`MeshSurface` 预索引 `assets/building/building_triangles.npz` 到 0.5 m XY 栅格，`intersect(starts)` 批量返回离地间隙；只被 `robot/unitree/policy/native/observation.py` 的 Building 网格分支调用。 |
| 代码 | `truth_worker.py` | 活代码：`main()` 读 `runtime_context.json`，非 `localization_mode == "fast_livo2_async"` 直接 `ValueError`；随后包装 `robot/unitree/policy/native/worker` 的 `legacy.main`，注入 observation/command/publish/finalize 钩子。由 `launch/building.py:247` 以 `-m` 启动。 |
| 代码 | `wait_actor.py` | 活代码：`main()` 最多等 30 s，要求 `<run>/worker_ready` 的 PID 可 `os.kill(pid,0)` 且 socket 已是 Unix socket；超时抛 `TimeoutError`。由 `simulation/launch/components.py` 作为 `actor_ready` 进程启动。 |
| 日志/输出 | `<run>/native_pose_source.jsonl` | `truth_worker.receive()` 写：`stamp_ns`、`world_sim_s`、`position`、`quaternion_wxyz`、`velocity_body_com`、`angular_velocity_body`、`contacts`（5 组）、`source`（SLAM 模式下固定为 "Gazebo native state for offline scoring only"）。是离线评分证据，不被导航消费。 |
| 日志/输出 | `<run>/actual_command_ack.jsonl` | `truth_worker.finalize()` 写：`stamp_ns`、`command`、`requested`、`goal_index`、`command_sequence`、`actor_inferred`、`fault`、`native_effective_source_ns`、`navigation_ground_truth_used`。命令执行证据。 |
| 日志/输出 | `<run>/policy_manifest.json`、`<run>/worker_result.json` | 由 legacy worker 循环写出；本模块的 `publish` 钩子只补 `navigation_ground_truth_used`、`navigation_truth_used`、`Student_input_dimension=45`、`Student_terrain_input=False`、`existing_native_COM_velocity_PID=True` 等来源字段。 |
| 输入（非日志） | `<run>/runtime_context.json`、`<run>/truth_navigation_command.json`、`<run>/terrain_target_manifest.json` | `truth_worker` 读前者取 profile，读后者作命令来源（带 `run_id`、`stamp_ns`、`monotonic_wall` 新鲜度与 `±[0.6,0.2,0.3]` 界校验），manifest 只作为 legacy 循环参数透传。 |
| 配置 | 无直接配置文件读取点 | 本层 5 个文件都不读 `configs/`（`grep -rn "configs/" src/embodied_navigation/simulation/*.py` 为空）。`configs/simulation/settings.json` 由启动器 `common.config.read` 按 `configs/launch/navigation.json` 合并，再由 `launch/prepare.py`（`expected_asset.terrain`）与 `planning/admission/path_admission.py`（`original_scenario`）消费——**不是** simulation 目录自己读的。 |

DynaNav 的产物目录是外部数据卷的 `experiments/<run>/`（不是 `runs/`），全局布局见 [configs/README.md](../../../configs/README.md) 与 [docs/review/dsh-workflow-and-readability-20261109.md](../../../docs/review/dsh-workflow-and-readability-20261109.md) §9；本层只写上面两个 JSONL。

## 模块内文件耦合

- 依赖方向（单向，无环）：`safety_state.py → common.state_contract`；`mesh_terrain.py → project`；`truth_worker.py → robot.unitree.policy.native.worker` + `robot.unitree.policy.student` + `control.tracking.policy_selection` + `control.safety.contact_mailbox`；`wait_actor.py` 不 import 任何内部模块（纯标准库）。
- 被依赖方向：`safety_state` ← native worker / robogauge native_loop；`mesh_terrain` ← native observation（Building 网格分支）；`truth_worker` 无 import 方（只被当进程启动）；`wait_actor` 无 import 方（只被 launch 当进程启动）。
- 文件之间是"库函数 + 进程入口"两种关系：`safety_state`/`mesh_terrain` 是纯函数/纯类，`truth_worker`/`wait_actor` 是带 argparse 的 `main()`，没有跨文件共享可变状态。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `safety_state.py` | 73 | `common.state_contract` | `robot/unitree/policy/native/worker.py`、`gazebo/robogauge/native_loop.py`、单测 | 清楚：纯工厂函数，两个消费者各一行调用 |
| `mesh_terrain.py` | 72 | `project`（`project_root`/`data_root`） | `robot/unitree/policy/native/observation.py` | 清楚：只服务 Building 网格，docstring 明说不是定位参考 |
| `truth_worker.py` | 208 | legacy native worker、StudentRuntime、policy_selection、contact_mailbox | 无 import 方；`launch/building.py` 以 `-m` 启动 | 偏重：188 行 `main()` + 多层嵌套闭包 + 改 `sys.argv` 再调 `legacy.main` |
| `wait_actor.py` | 24 | 仅标准库 | 无 import 方；`simulation/launch/components.py` 当进程启动 | 清楚：24 行、单一职责 |
| `__init__.py` | 1 | 无 | 无 | 清楚 |

四类问题：**上帝文件无**（无 >800 行文件）；**重复实现有**：`truth_worker.py` 与 `gazebo/robogauge/worker.py` 是同一套 actor 适配脚手架的两个副本（`difflib.SequenceMatcher(None, truth_worker 行列表, robogauge worker 行列表, autojunk=False)` 逐行比对：相同行含空行 94 行、非空 81 行；`native_pose_source.jsonl`/`actual_command_ack.jsonl` 字段同名、同样用 SIGUSR1 停止、同样 `ContactPublisher` 生命周期），差异只在命令来源与是否包装 `StudentRuntime`；`gazebo/robogauge/native_loop.py:51` 的 `rpy()` 与 `safety_state.py:23` 的 `native_rpy()` 是同一公式的两份实现（后者 docstring 自述 "Exact original native-loop formula"）；`wait_actor.py` 的就绪屏障在 `launch/building.py:379` 与 `navigation/robogauge/benchmark.py:305` 各有一份内联轮询（文件存在 vs 文件+PID+socket，30 s vs 60 s，语义并不一致）。**跨层直达有**：`truth_worker` 直接 import 策略层 legacy worker 并 monkeypatch `sys.argv`/模块函数（`legacy.recv_exact`/`atomic_json`/`build_observation`），跳过了显式接口。**隐式契约有**：`worker_ready` 文件内容必须是可 `int()` 的 PID、`truth_navigation_command.json` 的键名与 `run_id == run.name`、两个 JSONL 的字段名被离线脚本消费。

## 可读性评估

- `safety_state.py`：**良好**。函数 docstring 覆盖率 1.0（2/2）；`from_gazebo_native64` 的 docstring 说清前置条件（"Caller supplies the first frame of this run, never an unrelated wall clock"）与反例（"A real sensor cannot use this factory to acquire simulated-state identity"）；错误用固定字符串码（`invalid_gazebo_native64`、`invalid_simulation_start_or_ground_height`）；常量 `PHYSICS_STEP_S = 0.005`。
- `mesh_terrain.py`：**良好**。72 行、2 个方法，`intersect` 的 docstring 写清分箱、取"≤探测高度的最高命中"、无命中返回 `+inf`、结果 clamp 到 ≥0；`MeshSurface.__init__` 强制 `samefile` 校验 Building 资产归属。
- `truth_worker.py`：**需改进**。证据：9 个函数 **0 个 docstring**（只有模块级一句）；`main()` 188 行、内部定义 7 个闭包（`request_stop`/`receive`/`request`/`prepare`（内嵌 `zero_adapter`）/`observation`/`finalize`/`publish`）；`command` 校验把 6 类条件（run_id/形状/有限性/速度界/源龄/墙钟龄）塞进一个 `if`；`sys.argv = [...]` 后调 `legacy.main` 属非常规控制流，只有 `launch/building.py` 的调用点能解释。没有发现命名含糊，问题集中在"单函数过长 + 间接控制流"。
- `wait_actor.py`：**可接受**。24 行、意图清楚；不足：`main()` 无 docstring，`30`（秒）与 `0.05`（轮询）是魔法值，`TimeoutError` 的消息是唯一说明。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `truth_worker.py` 的 `main()` 拆成"读 profile/建 runtime/定义 7 个内联闭包/调用 legacy 循环"四个具名函数，钩子改成模块级小类或 `dataclass` | 188 行 + 7 个内联闭包让唯一的 SLAM-only Building actor 难以审阅，改一个新鲜度界要读完整函数 | 中：`tests/unit/test_building_entry_closure.py` 用 AST 直接解析 `truth_worker.py` 的 `main()`（找 `not slam` 守卫与 `StudentRuntime(` 顺序），拆分必然要同步改测试 | 先改测试（锁行为而非结构）→ 再拆；`PYTHONPATH=src python3 -B -m unittest tests.unit.test_building_entry_closure`；`test_native_safety_integration` 仍通过 |
| P2 | 把 `truth_worker.py` 与 `gazebo/robogauge/worker.py` 的公共脚手架（两个 JSONL 的写出、SIGUSR1 停止、`ContactPublisher` 生命周期）抽到本目录一个共享 helper | 两份相同部分为 94 行（含空行）/81 行（非空）（`difflib` 逐行比对，`autojunk=False`），字段名一旦漂移会让离线脚本读不到 | 中：动 actor 主循环外围；两个调用方分别在 Building 与 RoboGauge 路径 | 先抽纯写日志函数并用单测固定行格式，再替换两处；两个 fake-socket 回归通过 |
| P2 | `gazebo/robogauge/native_loop.py` 改用 `safety_state.native_rpy`，删掉本地 `rpy()` | 同一公式两份实现，且已经存在 `native_loop → safety_state` 的 import 关系 | 低：需断言数值逐位一致（都用 `np.clip` 的 asin） | 加一个随机四元数对比测试；`test_native_safety_integration` 与 `test_robogauge_native_loop` 通过 |
| P2 | 统一"等 actor 就绪"的语义：`wait_actor.py` 的文件+PID+socket 与 `launch/building.py`/`benchmark.py` 的内联轮询择一 | 三处判据不同（socket 检查只有主路径有），排障时容易误判"ready" | 中：跨模块（`launch/`、`navigation/`），需与对应负责人一起改 | 用伪造的 `worker_ready`/socket 做三个入口的对照测试；先只统一文档说明再改代码 |

没有 P0：真值边界与安全状态契约已经由 `StateProvenance` 强校验，`truth_worker` 也在模型/文件之前拒绝非 SLAM；本次没有发现会把仿真真值写成导航定位输入或在无日志时静默运行的路径。P1 是"可审阅性"，不是正确性缺陷。本轮只写建议、不改代码；任何 actor 脚手架改动都要先跑两个 fake-socket 回归与 Building 入口闭包测试。

## 组成

- [world/README.md](world/README.md)：从仓库资产生成隔离的 SDF、传感器契约与桥配置。
- [actuators/README.md](actuators/README.md)：当前 Gazebo Student/native 部署链的唯一关节力矩写入插件（保留 Teacher 历史命名）；RoboGauge 部署链另有专用插件。
- [gazebo/README.md](gazebo/README.md)：Gazebo 仿真部署，其中 `robogauge/` 是真实 SLAM 的 RoboGauge benchmark。
- [launch/README.md](launch/README.md)：仿真进程装配。
- [terrain/README.md](terrain/README.md)：受控 Actor 地形切换。
- [dynanav/README.md](dynanav/README.md)：Isaac/DynaNav RGB-D capture。

## 测试与限制

- [safety_state.py](safety_state.py) 有 [tests/unit/test_simulation_safety.py](../../../tests/unit/test_simulation_safety.py) 与 [tests/regression/test_native_safety_integration.py](../../../tests/regression/test_native_safety_integration.py) 覆盖；`mesh_terrain` 有 [tests/unit/test_data_paths.py](../../../tests/unit/test_data_paths.py)（资产归属与路径）；`truth_worker` 有 [tests/unit/test_building_entry_closure.py](../../../tests/unit/test_building_entry_closure.py)（AST 检查"非 SLAM 在模型加载前拒绝"）。
- 其余仿真启动没有端到端测试。本次核对没有启动 Gazebo、Isaac 或 ROS；`wait_actor.py` 与 `truth_worker.py` 的实际启动路径未验证。
- `runs/`、`assets/`、`models/` 是指向外部数据卷的软链/外部数据，不入 Git；Isaac 场景、模型权重与策略资产须另行提供。
