# robot/unitree：Go2 部署子包

盘点日期 2026-11-20。`robot/unitree` 是 Go2 的唯一实现子包：两套策略契约（45D RoboGauge 生产路径、247D 冻结 Teacher 观测）加一个共享四元数工具，全部是纯 CPU 数值代码。

## 模块定位与架构层级

- 架构层级：第 6 层「机器人执行」的实现子包，承上（`robot/README.md` 的层边界）启下（`policy/native`、`robogauge`）。
- 输入：`state64`（wxyz 姿态 + body 速度 + 12 关节状态）、`command3`、`last_action12`；以及被仿真侧读取的关节契约数据（`contract.json` 的 `joint_names`、`default_joint_positions`）。
- 输出：45D 观测与 12 维动作（RoboGauge）、247D 观测与 height scan 诊断（native）、Go2HV PD 力矩包络、canonical 关节顺序映射。
- 上游 / 下游：调用方全部在 [simulation/dynanav/](../../simulation/dynanav/)：`capture/policy.py` 加载 [robogauge/runtime.py](robogauge/runtime.py) 与 [contract.py](robogauge/contract.py)，`capture/scene.py` 读 [policy/native/contract.json](policy/native/contract.json) 校验 articulation 关节并写入初始姿态，`capture/{control,physics,ros_interface}.py` 用 `quaternion_rotation` 做世界/机体旋转。本子包不 import 仿真或 ROS。
- 边界：不训练、不联网、不注册 ROS 节点、不写文件；`Runtime` 只读已冻结的 TorchScript 权重。路线跟踪、停车、安全门都在 `navigation/` 与仿真采集侧，不在本子包。

## 当前文件夹结构

```text
src/embodied_navigation/robot/unitree/
├── README.md       # 本文件：子包索引、两套契约关系与共享依赖
├── __init__.py     # 1 行：包 docstring
├── policy/
│   ├── __init__.py # 0 行占位（policy/ 层无独立 README，故在此表覆盖）
│   └── native/     # 247D 冻结观测契约，见 policy/native/README.md
│       ├── __init__.py
│       ├── contract.json
│       └── observation.py
└── robogauge/      # 45D MoE-CTS 契约与运行时，见 robogauge/README.md
    ├── __init__.py
    ├── contract.py
    └── runtime.py
```

## 逐文件职责

本表覆盖本目录直接文件与两处子目录索引；子目录内每个文件的行级职责见各自 README，不在此重复。

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 包标记，1 行 docstring "Unitree deployment: observations, joint order and inference."；无符号、无副作用 | 无 |
| `README.md` | 本文件：子包职责、目录归属与跨子包依赖 | 索引页 |
| `policy/__init__.py` | 0 行占位文件，仅使 `policy` 成为包；`policy/` 自身无 README | 无 |
| `policy/native/` | 247D 冻结 Teacher 观测契约（`observation.py` 312 行 + `contract.json` 414 行）；逐文件职责见 [policy/native/README.md](policy/native/README.md) | `build_observation()`、`TerrainHeightMap`、`quaternion_rotation()` |
| `robogauge/` | 45D MoE-CTS 契约与 SHA256 校验的 CPU TorchScript 运行时；逐文件职责见 [robogauge/README.md](robogauge/README.md) | `observation()`、`go2hv_torque()`、`Runtime`、`verify_model()` |

## 两套契约与唯一的跨子包依赖

- **生产契约（45D）**：[robogauge/contract.py](robogauge/contract.py) 的 `observation()` 产出 45D 观测，`go2hv_torque()` 产出 Go2HV PD 力矩；生产消费方是 `capture/policy.py` 的 `PolicyDriver`。
- **冻结契约（247D）**：[policy/native/observation.py](policy/native/observation.py) 的 `build_observation()` 按 [contract.json](policy/native/contract.json) 的 9 段拼接顺序、噪声、clip、scale 产出 247D；在本检出中**没有生产调用方**（`git grep "build_observation|TerrainHeightMap" -- src` 只命中该文件），保留用于诊断与适配。
- **关节契约是生产数据**：即使 247D builder 未接入，[policy/native/contract.json](policy/native/contract.json) 仍被 `capture/scene.py::initialize_articulation` 直接读取，用于校验 Isaac articulation 的 12 个 DOF 名称并按 `default_joint_positions` 设定初始姿态；改这个文件会影响仿真初始化，不只是文档。
- **唯一跨子包依赖**：[robogauge/contract.py](robogauge/contract.py) 第 9 行 `from ...policy.native.observation import quaternion_rotation`，两套契约共用同一 wxyz 旋转实现；依赖方向 `robogauge → policy/native` 单向，无环。
- **四元数约定**：本子包全部使用 `[w, x, y, z]`，`quaternion_rotation()` 返回 world-from-body；ROS `geometry_msgs/Quaternion` 序列化时按字段名赋值，不靠元组顺序。

## 验证与测试入口 / 配置注意

- [tests/regression/test_robogauge_contract.py](../../../../tests/regression/test_robogauge_contract.py)：关节顺序 `ORDER`、默认姿态 `QDEFAULT` 与 `contract.json` 一致性断言，错误权重在导入 torch 前被拒绝。
- [tests/unit/test_dynanav_joint_readback.py](../../../../tests/unit/test_dynanav_joint_readback.py)、[tests/unit/test_truth_deskew_and_mode.py](../../../../tests/unit/test_truth_deskew_and_mode.py) 分别覆盖 `go2hv_torque` 与 `quaternion_rotation` 约定。
- 命令：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`、`PYTHONPATH=src python3 -B -m unittest discover -s tests/regression`；纯 CPU，无需 Isaac/ROS/torch（`Runtime` 相关路径需要 torch，但上述契约测试不导入它）。
- 权重路径由 `PROJECT_DATA_ROOT` 决定，见 [robogauge/README.md](robogauge/README.md)；仓库内不保存 `.pt` 或场景资产。
