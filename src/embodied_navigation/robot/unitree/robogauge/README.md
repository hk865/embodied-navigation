# robot/unitree/robogauge：RoboGauge 45D 部署

本目录提供 RoboGauge MoE-CTS 的纯数值契约、45D 直接观测、CPU TorchScript 推理与可选真实权重 replay，不启动 ROS、Gazebo、Isaac 或硬件。

## 目录结构

```text
src/embodied_navigation/robot/unitree/robogauge/
├── README.md       # 本文件：45D 契约、运行时与权重校验
├── __init__.py     # 1 行，包 docstring：Go2 关节契约/观测映射/actor 运行时
├── contract.py     # 76 行，45D 观测数值契约 + Go2HV 力矩包络
├── cpu_fixture.py  # 77 行，可选真实权重 CPU replay（不启动仿真）
└── runtime.py      # 99 行，校验 SHA256 的 CPU TorchScript actor
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `contract.py`（76 行） | 活代码/纯 NumPy：`observation(state64, command3, last_action12)` 返回 `(float32[45], provenance)`；`joint_indices()` 在 official（FL/FR/RL/RR）与 native（FR/FL/RR/RL）顺序间映射；`go2hv_torque()` 复刻归档 Go2HV PD 与速度包络；`MODEL_SHA256` 与 `Q0` 冻结 |
| 代码 | `runtime.py`（99 行） | 活代码：`verify_model()` 流式 SHA256 校验；`Runtime` 加载 CPU TorchScript、`torch.set_num_threads(1)`、预热 10 帧后 `reset()`；`infer()` 返回 12 维动作（模型 official 序 → 夹到 ±100 → 映射回 native 序）；`metadata()` 记录契约 |
| 代码 | `cpu_fixture.py`（77 行） | 活代码/可选 fixture：需要 public PyTorch；对 30 帧与独立 JIT 实例逐帧对比、验证 reset 可复现、统计 p50/p95 延迟 |
| 代码 | `__init__.py`（1 行） | 包 docstring "Go2 joint contract, policy observation mapping and verified actor runtime." |
| 日志/输出 | 无日志、无日志文件 | 本目录 4 个 py 文件 `getLogger` 均为 0（事实包 grep）；没有 ROS/仿真日志 |
| 日志/输出 | `cpu_fixture.py --output <path>` | 唯一写文件点：把 fixture 结果 JSON 写入显式给定路径；不传 `--output` 时只 `print` 到 stdout。产物是本机 CPU 复算收据，不是物理/导航证据 |
| 日志/输出 | `<PROJECT_DATA_ROOT>/embodied-navigation/runtime_assets/models/robogauge_policy.pt` | 只读：`Runtime` 启动时做 SHA256 校验并加载；权重不在 Git |
| 配置 | 无 `configs/` 读取点 | 本目录不 import `common.config`、不读 `configs/`；`configs/robogauge/contract.json` 是**归档副本、无 Python 读取者**（见 [docs/robogauge/README.md](../../../../../docs/robogauge/README.md)），权威常量在 `contract.py` 里 |
| 配置 | `PROJECT_DATA_ROOT` | 经 `project.data_root()` 解析权重路径；`--model` 可指定别处同一份导出，但 SHA256 必须等于 `MODEL_SHA256`，否则 `verify_model()` 拒绝加载 |

补充：`configs/robogauge/` 目录里其它文件（`sensor_contract.json`、`world.sdf.in` 等）由 `simulation/gazebo/robogauge/world.py` 读取，**不属于本目录的配置读取点**，不要混为一谈。

## 模块内文件耦合

- 依赖方向：`contract.py` → [policy/native/observation.py](../policy/native/observation.py) 的 `quaternion_rotation()`（唯一跨子目录依赖）；`runtime.py` → `contract.py` + `project.data_root()`；`cpu_fixture.py` → `runtime.py` + `contract.py`。三层单向，**无环**。
- 反向依赖：`runtime.py` 被 `navigation/robogauge/benchmark.py`、`simulation/gazebo/robogauge/worker.py`、`simulation/dynanav/capture/policy.py`、`simulation/dynanav/experiment.py` 复用；`contract.py` 被 `simulation/gazebo/robogauge/worker.py`、`simulation/dynanav/capture/policy.py` 复用。本目录是被依赖的库，不 import 上层。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `contract.py` | 76 | `policy.native.observation.quaternion_rotation` | `cpu_fixture`、`runtime`、`navigation.robogauge.benchmark`（间接）、`simulation.gazebo.robogauge.worker`、`dynanav.capture.policy` | 清楚：纯函数 + 常量 |
| `runtime.py` | 99 | `project.data_root`、`contract` | `cpu_fixture`、`navigation.robogauge.benchmark`、`simulation.gazebo.robogauge.worker`、`dynanav.capture.policy`、`dynanav.experiment` | 清楚 |
| `cpu_fixture.py` | 77 | `runtime`、`contract` | 无（可选脚本，`__main__` 入口） | 清楚 |
| `__init__.py` | 1 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：没有（最大文件 99 行）。
- **重复实现**：有（跨目录、有测试守护）——Go2 关节名与默认姿态在 `contract.py`（`OFFICIAL_NAMES`/`NATIVE_NAMES`/`Q0`）与 [policy/native/contract.json](../policy/native/contract.json)（`joint_names`/`default_joint_positions`）各存一份；`tests/regression/test_robogauge_contract.py` 断言 `NATIVE_NAMES == CONTRACT["joint_names"]` 且 `QDEFAULT == CONTRACT["default_joint_positions"]`。另外 `simulation/gazebo/robogauge/native_loop.py` 也自带一份 `QDEFAULT`，不在本目录守护范围内。
- **跨层直达**：轻微——`contract.py` 直接从 `policy/native/observation.py` 取 `quaternion_rotation`，两个子目录共用同一旋转实现（好处是不重复实现、坏处是 robogauge 依赖 policy 内部文件）。
- **隐式契约**：有——`MODEL_SHA256` 必须与权重文件匹配（不匹配直接拒绝，不静默接受）；official/native 两套关节顺序靠 `joint_indices()` 显式映射；模型输入固定 `(1, 45)` float32；内部历史帧固定 10 帧（`runtime.py:45-46` 预热 10 次，`metadata()` 里 `internal_history_frames=10` 是同一数字的第二处出现）；`action_scale=0.25` 只写在 metadata，真正的 `QDEFAULT + 0.25*action` 在 `simulation/gazebo/robogauge/native_loop.py` 里执行。

结论：**耦合清楚**，是本层里最小、最内聚的目录；唯一可议的是 `contract.py → policy/native/observation.py` 的跨子目录依赖与 Go2 常量的两处保存。

## 可读性评估

| 对象 | 分级 | 证据 |
| --- | --- | --- |
| `contract.py` | **良好** | 76 行 / 3 函数；docstring 0.667；最长函数 28 行；无 >80 行函数；3 行注释；模块 docstring 说明 wxyz/body-frame 帧约定，`OFFICIAL_NAMES` 上方注释解释了两套顺序 |
| `runtime.py` | **可接受** | 99 行 / 8 函数；docstring 0.25（2/8：`default_model_path`、`verify_model`）；最长函数 28 行；无 >80 行；0 行注释。`metadata()` 返回的 20+ 个字段本身就是契约文档 |
| `cpu_fixture.py` | **可接受** | 77 行 / 2 函数；函数 docstring 0.0，但模块 docstring 写明“never launches ROS, Gazebo, Isaac or hardware”，断言信息可读（"Sequential actor replay differs" / "Reset is not reproducible"）；最长 49 行 |
| `__init__.py` | **良好** | 1 行 docstring，无代码 |

区分结构与文档：本目录没有结构问题（无环、无静默 except、失败即抛错），文档缺口集中在 `cpu_fixture.run_fixture()` 与 `runtime.Runtime.__call__/reset/metadata`——它们行为清楚但没写 docstring。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把 `runtime.py` 里的 10 帧预热提为命名常量，并复用到 `metadata()` 的 `internal_history_frames` | 同一魔法数出现两处，改一处漏一处会让 metadata 与实际行为不一致 | 无（纯常量） | `tests/regression/test_robogauge_contract.py`、`test_robogauge_native_loop.py` |
| P2 | 给 `Runtime.__call__/reset/metadata` 与 `cpu_fixture.run_fixture()` 补 docstring | 运行时与 fixture 是外部使用者最先读的入口 | 无 | 静态检查 |
| P2 | 在 `contract.py` 顶部注释说明“Go2 常量与 `policy/native/contract.json` 必须一致，由 `test_robogauge_contract` 守护”；`contract.py:9` 的跨子目录 import 也加一句理由 | 现在防漂移只靠测试，代码里没有提示 | 无 | 已有测试断言 |
| P2 | `QDEFAULT` 在 `simulation/gazebo/robogauge/native_loop.py` 的第三份副本：改为 import 本目录常量（跨模块改动，需与该目录负责人协调） | 三份 Go2 默认姿态里只有两份被测试守护 | 低—中：涉及 simulation 目录，超出本 README scope | `grep -rn "QDEFAULT" src` + `test_robogauge_native_loop` |

没有 P0/P1：本目录最大文件 99 行、无配置读取、无日志写入、依赖单向、权重有 SHA256 硬校验（fail closed）。本轮只写建议、不改代码；涉及 simulation 的建议需要先有测试与对齐。

## 关键文件与接口

- `contract.py`：`observation(state64, command3, last_action12)` 返回 `(float32[45], provenance_dict)` 二元组（body 角速度 ×0.25、投影重力、command ×[0.5, 1, 1]、关节位置相对默认、关节速度 ×0.05、上一原始动作；策略侧 official FL/FR/RL/RR 顺序）；`go2hv_torque()` 复刻归档 Go2HV PD 与速度包络（20.2 / 23.4 Nm，13.5 / 30 rad/s）；`MODEL_SHA256` 与默认关节顺序冻结。
- `runtime.py`：`verify_model()` 校验模型 SHA256；`Runtime` 从 `data_root()/runtime_assets/models/robogauge_policy.pt` 加载 CPU TorchScript，单线程，预热 10 个内部历史帧后 `reset()`；`infer(obs)` 返回 12 维动作，`metadata()` 记录 45D、proprioception-only、无 terrain/COM 速度输入、无 COM 速度 PID。
- `cpu_fixture.py`：可选真实权重 CPU replay，对 30 帧与独立 JIT 对比并验证 reset 可复现；需要 torch，不启动仿真。

## 调用与依赖

实际 45D native 循环在 [simulation/gazebo/robogauge/native_loop.py](../../../simulation/gazebo/robogauge/native_loop.py)，基准编排在 [navigation/robogauge/benchmark.py](../../../navigation/robogauge/benchmark.py)；本目录只提供契约与运行时。权重不在 Git，由 `PROJECT_DATA_ROOT` 指向的数据目录提供；更完整的部署与权重说明见 [docs/robogauge/README.md](../../../../../docs/robogauge/README.md)。

## 与 247D 的区别

45D 是 RoboGauge 自己的直接观测接口（独立 scale 与关节顺序），不是 [policy/native](../policy/native/README.md) 247D 中间观测链的裁剪，也不含 Student 的 COM 速度 PID 或 terrain/camera 输入；两者不能当作同一策略输入或 actuator 契约。

## 测试与限制

[tests/regression/test_robogauge_contract.py](../../../../../tests/regression/test_robogauge_contract.py)（NumPy 契约）、[test_robogauge_native_loop.py](../../../../../tests/regression/test_robogauge_native_loop.py)（可选 torch + 假 socket）、[test_robogauge_benchmark.py](../../../../../tests/regression/test_robogauge_benchmark.py)、[test_native_safety_integration.py](../../../../../tests/regression/test_native_safety_integration.py) 覆盖。CPU fixture 不是物理或导航测试；仿真部署与训练环境不宣称等价。`test_robogauge_native_loop.py` 在未安装可选 CPU Torch 时会被 skip（见 [tests/README.md](../../../../../tests/README.md) 的 skip 记录），skip 不等于通过。
