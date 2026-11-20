# robot：机器人执行层的观测与策略契约

盘点日期 2026-11-20。`robot` 是「机器人执行」层的顶层命名空间：只定义 Unitree Go2 的观测向量、关节顺序、默认姿态与策略推理入口，不含训练、场景、ROS 节点或实机驱动。

## 模块定位与架构层级

- 架构层级：七层中的第 6 层 —— 机器人执行（body 命令 → 策略观测 → 关节目标 → Go2HV 力矩）。
- 输入：`state64`（`state[4:8]` 为 wxyz 四元数、`state[8:14]` 为 body COM 线速度/角速度、`state[14:50]` 为 12 关节 q/qd/力矩）、`command3=[vx, vy, wz]`、`last_action12`。真实生产者是 [simulation/dynanav/capture/policy.py](../simulation/dynanav/capture/policy.py)、[control.py](../simulation/dynanav/capture/control.py)、[physics.py](../simulation/dynanav/capture/physics.py)。
- 输出：12 维原始动作、关节目标 `q_target = q_default + 0.25 * raw_action`、Go2HV PD 力矩包络（`20.2/23.4 Nm`、`13.5/30 rad/s`）。本层不发布 ROS 消息、不写执行器、不接触 `cmd_vel`。
- 上游 / 下游：被 `simulation/dynanav/` 采集链路调用（策略加载、关节契约、姿态旋转）；反向不依赖任何上层。唯一的外部依赖是 `common/geometry.py`。
- 边界：不训练、不采集数据、不导出模型；没有实机后端。路线跟踪与停车属于 `navigation/`（`RouteTracker` 输出 body 命令，本层只负责把命令变成关节动作），感知与 SLAM 不经过本层。

## 当前文件夹结构

```text
src/embodied_navigation/robot/
├── README.md          # 本文件：层职责、两套观测契约与边界
├── __init__.py        # 1 行：包 docstring（"No policy training code"）
└── unitree/           # 唯一实现子包，见 unitree/README.md
    ├── __init__.py    # 1 行：包 docstring
    ├── policy/
    │   ├── __init__.py          # 0 行占位
    │   └── native/              # 247D 冻结观测契约，见 policy/native/README.md
    │       ├── __init__.py
    │       ├── contract.json
    │       └── observation.py
    └── robogauge/     # 45D MoE-CTS 契约与运行时，见 robogauge/README.md
        ├── __init__.py
        ├── contract.py
        └── runtime.py
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 包标记，仅 1 行 docstring "Robot observation and actuation contracts. No policy training code."；无 import、无符号、无运行时副作用 | 无 |
| `README.md` | 本文件：层级定位、两套观测契约的关系与边界 | 索引页 |
| `unitree/` | 唯一实现子包：承载全部关节契约与策略代码；子目录内逐文件职责见 [unitree/README.md](unitree/README.md)，本表不重复 | 子包入口 |

## 两套观测契约（不要混用）

| 契约 | 维度 | 位置 | 当前角色 |
| --- | ---: | --- | --- |
| RoboGauge MoE-CTS 直接观测 | 45 | [unitree/robogauge/contract.py](unitree/robogauge/contract.py) 的 `observation()` | **生产路径**：DynaNav 采集默认策略输入，独立 scale 与关节顺序（策略侧 FL FR RL RR） |
| 冻结 Teacher native 中间观测 | 247 | [unitree/policy/native/observation.py](unitree/policy/native/observation.py) 的 `build_observation()` | **仅诊断/适配**：`git grep "build_observation|TerrainHeightMap" -- src` 只命中本文件（与文档），生产链路不加载 Teacher actor |

两套契约的观测项、缩放、裁剪、关节顺序和 actuator 参数都不同，不能互相裁剪或替换。45D 观测的 `provenance` 明确 `terrain_input=False`、`COM_linear_velocity_input=False`；247D 契约的 height scan 与 `base_com_offset` 语义只属于冻结 Teacher 契约。

## 验证与测试入口 / 配置注意

- 纯 CPU 契约测试：[tests/regression/test_robogauge_contract.py](../../../tests/regression/test_robogauge_contract.py)（关节顺序、默认姿态、力矩包络、错误权重拒绝，不导入 torch/Isaac）；[tests/unit/test_dynanav_joint_readback.py](../../../tests/unit/test_dynanav_joint_readback.py) 复用 `go2hv_torque`；[tests/unit/test_truth_deskew_and_mode.py](../../../tests/unit/test_truth_deskew_and_mode.py) 用 `quaternion_rotation` 交叉校验四元数约定。
- 全量入口：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit` 与 `... -s tests/regression`；本层测试不需要 Isaac、ROS 或机器人。
- 策略权重是外部资产：`$PROJECT_DATA_ROOT/embodied-navigation/runtime_assets/models/robogauge_policy.pt`（SHA256 硬校验，见 robogauge README）；场景与机器人 USD 同样在数据卷，见 [docs/dependencies/BUILD.md](../../../docs/dependencies/BUILD.md)。
- 本目录没有独立 ROS 节点；DynaNav 主线的策略执行链只有 `robot/unitree/robogauge`，`policy/native` 只提供冻结观测契约与共享四元数工具。