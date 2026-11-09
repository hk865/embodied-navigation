# simulation/gazebo/robogauge

本目录是 RoboGauge MoE-CTS 的 Gazebo 仿真部署：准备真实 SLAM benchmark 的输入、用 native request64/response16 回路驱动机器人，并记录接触与命令证据。

## 目录结构

```text
src/embodied_navigation/simulation/gazebo/robogauge/
├── motor_src/          # 子模块，见 motor_src/README.md（Go2HV 原生执行器源码与 CMake）
├── README.md           # 本文件
├── __init__.py         # 1 行，包 docstring（native request64/response16 Gazebo loop, worker and world preparation）
├── native_loop.py      # 371 行，RoboGauge worker 使用的无 ROS socket 主循环（与 legacy policy/native/worker.py 的循环同构）
├── worker.py           # 139 行，独立 actor 进程：真实 SLAM 命令 → 推理 → 力矩回路
└── world.py            # 159 行，一次 case 的世界/配置落盘（不启动模拟器）
```

3 个实现文件、合计 670 行；`motor_src/` 另有 430 行 C++。无 `runs/` 软链，产物写外部数据卷。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `world.py` | 活代码：`prepare(case, label, mode='slam', backend_capture=False)` 只接受真实 SLAM（truth/capture 一律 `ValueError`），`CASES` 7 个（`stairs_short`/`single_floor_turn`/`combined_floor1_inside`/`multi_floor2`/`single_floor_loop`/`second_stair_short`/`third_stair_short`）；`actuator_path()` 返回 `install/robogauge/lib/librobogauge_actuator.so`。 |
| 代码 | `worker.py` | 活代码（进程入口）：读 `runtime_context.json`，要求 `localization_mode == "fast_livo2_async"`，`Runtime`（RoboGauge MoE-CTS）加载模型，读 `<run>/navigation_command.json`（经 `navigation/robogauge/commands.read_command`）并驱动 `native_loop.main`。docstring 明写 "Never imports StudentRuntime"。 |
| 代码 | `native_loop.py` | 活代码：`main(*, actor_loader, observation_builder, command_source, receive, publish, finalize, prepare, runtime_duration, termination_requested, argv)` 是依赖注入式主循环，自建 Unix socket server、`TerrainHeightMap.from_sdf`、`SimulationSafetyGate`，逐帧调用 `from_gazebo_native64` 并回 16 维 response。worker 把 `prepare_velocity_input`/`recv_exact`/`atomic_json` 注入进去。 |
| 日志/输出 | `<run>/runtime_context.json`、`PCT_route.json`、`world.sdf`、`sensor_bridge.yaml`、`sensor_contract.json`、`terrain_target_manifest.json`、`scan_params.json`、`building_fastlivo.json`、`building_camera.json`、`experiment_setup.json` | `world.prepare()` 一次写出的 10 个文件（另有目录本身）。`experiment_setup.json` 记录 case、spawn、goal、timeout、`navigation_mode="slam"`、`navigation_ground_truth_used=False`、`stair_speed_cap_mps=0.25`、terminal PD、`motor_envelope="Go2HV 20.2 driving / 23.4 braking…"` 等；`scan_params.json`/`building_fastlivo.json` 只覆盖 frame id 与 topics/backend 参数。 |
| 日志/输出 | `<run>/native_pose_source.jsonl`、`<run>/actual_command_ack.jsonl` | `worker.py` 写：前者每帧记录 `stamp_ns`/`world_sim_s`/`position`/`quaternion_wxyz`/`velocity_body_com`/`angular_velocity_body`/`contacts`/`source`（固定 "offline scoring only; no navigation or COM policy feedback"）；后者是命令 ACK。 |
| 日志/输出 | `<run>/policy_manifest.json`、`worker_ready`、`state.json`、`worker_result.json` | `native_loop.main` 写：manifest（含 `observation_source`、`navigation_truth_used=False`、`command_slew_acceleration`、safety 来源/目的、seed、torch 线程等）；`worker_ready` 是 PID 文本；`state.json` 每 10 帧原子写一次；`worker_result.json` 在收尾写（`completed`/`fault`/`samples`/`physical_test_completion_time_s`/`last_sim_time`/`cpu_forward_ms` 的 p50/p95/max）。 |
| 日志/输出 | `<run>/telemetry.jsonl`、`<run>/height_scan_sources.jsonl`、`<run>/observations_actions.npz` | 仅在 `NAVIGATION_RECORD_RAW=1` 时写：逐帧观测/动作、地形扫描来源；npz 存 `observations`/`actions`。默认关闭（telemetry 指向 `/dev/null`），所以"没有 telemetry"是默认状态，不是丢失。 |
| 配置 | `configs/robogauge/` | `prepare()` 读 `deployment_profile.json`、`routes/<case>.json`、`world.sdf.in`、`sensor_bridge.yaml`、`sensor_contract.json`、`terrain_targets.json`；`world.sdf.in` 的 `${ASSET_ROOT}`/`${ROBOGAUGE_ACTUATOR}` 在 prepare 时展开。 |
| 配置 | `configs/scenarios/building/scan.json`、`configs/localization_mapping/fastlivo.yaml`、`configs/localization_mapping/camera.yaml` | prepare 时读入并写入 run 目录：scan 的 `grid_map.frame_id=map`、`sliding_map_frame_id=building_scan_sliding_map`；FAST-LIVO2 的 topics（`/demo/camera`、`/building/lidar_filtered`、`/livox/imu`）+ `backend_spool` 目录与队列参数；camera 配置原样复制。运行期不再读 `configs/`。 |

`prepare()` 要求 label 目录不存在（`exist_ok=False`），目录是 `$PROJECT_DATA_ROOT/runtime_assets/runs/robogauge/<label>`；短 case（`stairs_short`/`single_floor_turn`）超时 160 s、其余 600 s，profile 的 `duration_s = timeout + 15`。模式只允许真实 SLAM。

## 模块内文件耦合

- 依赖方向：`worker.py → native_loop.py`（import 并注入回调）；`worker.py` 另 import `robot/unitree/robogauge/{runtime,contract}`、`navigation/robogauge/commands`、`control/safety/contact_mailbox`；`world.py` 只 import `project`。**无环**；`native_loop` 不 import worker，靠参数注入避免反向依赖。
- 被依赖方向：`world.py` ← `navigation/robogauge/benchmark.py`（`prepare`/`actuator_path`/`CASES`）；`worker.py` 无 import 方（`benchmark.py:54` 以 `-m` 启动）；`native_loop.py` ← worker，也被 `tests/regression/test_robogauge_native_loop.py` 直接 import。
- 文件之间是"库函数 + 进程入口 + 回调注入"：`native_loop.main` 用 10 个关键字参数（legacy 版 9 个，多一个 `argv`）把 actor/observation/command/publish/finalize 全部注入，因此它和 legacy `policy/native/worker.py` 的循环共享同一套回调形状（隐式契约）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `native_loop.py` | 371 | `policy.native.observation`、`common.state_contract`、`safety_state`、`control.safety.simulation_gate`、torch | `worker.py`、回归测试 | 偏重：`main` 单函数 312 行，10 个注入回调 |
| `worker.py` | 139 | `native_loop`、robogauge runtime/contract、commands、contact_mailbox | 无 import 方；`benchmark.py` 启动 | 可接受但重复：脚手架与 `simulation/truth_worker.py` 有 94 行（含空行）/81 行（非空）逐行相同（`difflib`，`autojunk=False`） |
| `world.py` | 159 | `project`、`configs/robogauge/*`、`configs/scenarios/building/scan.json`、`configs/localization_mapping/{fastlivo,camera}.yaml` | `navigation/robogauge/benchmark.py`、回归测试 | 清楚：单函数写 10 个文件，docstring 完整 |
| `__init__.py` | 1 | 无 | `worker.py`（`from . import native_loop as loop` 会先执行包 `__init__`） | 清楚 |

四类问题：**上帝文件无**（371 < 800，但 `native_loop.main` 312 行已接近"事实上帝函数"）；**重复实现有**（worker 与 truth_worker 同构；`native_loop.main` 与 `robot/unitree/policy/native/worker.py:main` 是几乎相同的依赖注入循环（同一组回调名，前者多一个 `argv`），RoboGauge 选择自带一份而不是复用 legacy）；跨层直达有（`world.py` 直接读 5 个 `configs/` 路径并改写参数，不经过 `common.config`，所以 `configs/README.md` 的"prepare pin"规则管不到这里的覆盖）；隐式契约有：`NAVIGATION_RECORD_RAW`、`BUILDING_SAFETY_SOCKET` 环境变量，`worker_ready` 只要存在（benchmark 不校验 PID/socket），10 个注入回调的名字与顺序。

## 可读性评估

- `world.py`：**良好**。3 个函数 docstring 覆盖率 1.0；`prepare` 的 docstring 写清"只接受 SLAM 模式""展开资产根与插件路径""设置 case spawn""写 setup/reference 文档"四步；`CASES` 与超时规则集中可见；`write()` 助手固定 `allow_nan=False`。不足：`prepare` 129 行依次改 3 份配置字典，读起来要靠 `parameters["common"]`/`["backend"]`/`["publish"]` 的键名。
- `native_loop.py`：**需改进**。5 个函数只有 2 个 docstring（0.4）；`main` 312 行，混合了"解析参数/搭 socket/加载地形/构造 metadata/逐帧安全门禁/写 8 类文件/收尾"七件事；`metadata` 字典 20+ 键把契约写在数据里，没有类型或常量表；好在 `prepare_velocity_input`/`recv_exact`/`atomic_json` 三个助手有 docstring 或语义清楚。
- `worker.py`：**可接受**。`main` 123 行、docstring 只有 1 个（7 个函数里 0.143），但结构是线性的"解析→检查模式→Runtime→hooks→loop"，与 `truth_worker.py` 同构所以读一份即可；魔法值（`stamp_ns = world_sim_s - 0.005` 的 `0.005`）靠 `safety_state.PHYSICS_STEP_S` 的语义解释，本文件内无注释。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `native_loop.main` 拆成 `_open_socket`/`_load_terrain`/`_build_metadata`/`_run_frames`/`_write_result` 五个函数，保留现有注入签名不变 | 312 行单函数是 RoboGauge 唯一执行回路，改安全门禁或写文件顺序时容易碰坏别的段 | 中：这是两条部署路径共用的循环，时序与文件写出顺序都是证据 | 先补/复用 `tests/regression/test_robogauge_native_loop.py` 的假 socket 用例（已有）覆盖每个产物，拆分后逐项比对 `state.json`/`worker_result.json` 键与 `observations_actions.npz` 形状 |
| P2 | 把 `worker.py` 与 `simulation/truth_worker.py` 的公共脚手架抽到 `simulation/` 共享 helper | `native_pose_source.jsonl` JSON 写出与 SIGUSR1/ContactPublisher 生命周期两份逐行相同（`difflib` 比对：含空行 94 行、非空 81 行） | 中：影响两条 actor 路径 | 先抽纯写日志函数并单测固定字段顺序，再替换两处；两个 fake-socket 回归通过 |
| P2 | 在 README/注释标注 `NAVIGATION_RECORD_RAW=1` 才会写 `telemetry.jsonl`/`height_scan_sources.jsonl`/`observations_actions.npz` | 默认 run 里没有这些文件，容易被误判为"记录缺失" | 无（文档） | 人工复核；与 `native_loop.py` 的 3 处 `recording` 分支对照 |
| P2 | 记录 `world.py` 绕过 `common.config`、直接改写 5 个 configs 参数的事实（或在 `configs/README.md` 的读取点表里补一行） | 现有配置文档只覆盖启动器路径，读者会以为 `configs/robogauge/*` 原样生效 | 无（文档） | 对照 `world.py` 的 3 处 `parameters[...]` 改写 |

没有 P0：模式检查（只接受真实 SLAM）、`navigation_ground_truth_used=False` 与安全门禁逐帧调用都在代码里显式存在；本次没有发现把真值当导航输入的路径。P1 是唯一的大函数拆分，且已有假 socket 回归可依托。本轮只写建议、不改代码；拆分前后都必须跑 `test_robogauge_benchmark.py`、`test_robogauge_native_loop.py` 与 `test_native_safety_integration.py`。

## 调用与测试

`navigation/robogauge/benchmark.py` 的 `process_commands` 把本目录的 `worker` 与 Gazebo、FAST-LIVO2、backend、Building 导航装配成完整进程链；`preflight` 只查文件与哈希，不代表可运行。

专用回归：[tests/regression/test_robogauge_benchmark.py](../../../../../tests/regression/test_robogauge_benchmark.py)、[test_robogauge_native_loop.py](../../../../../tests/regression/test_robogauge_native_loop.py) 与 [test_native_safety_integration.py](../../../../../tests/regression/test_native_safety_integration.py)，都使用 fixture 或假 socket，不启动 Gazebo。

Gazebo 原生 state64 中的姿态、角速度和关节量用于策略观测；COM 线速度与地形不进入 MoE-CTS 观测，位置、速度和接触等还用于评分、安全与诊断。导航定位仍使用 SLAM 反馈，`navigation_ground_truth_used` 固定为 false。本次核对没有启动 Gazebo 或 ROS，端到端 benchmark 未验证；`world.prepare()` 的 10 个产物只按源码核对，未实际生成。
