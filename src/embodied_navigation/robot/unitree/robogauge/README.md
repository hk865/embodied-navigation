# robot/unitree/robogauge：RoboGauge MoE-CTS 契约与运行时

盘点日期 2026-11-20。本目录是 DynaNav 主线的生产策略层：45D 直接观测的纯 NumPy 契约、Go2HV 力矩包络，以及经过 SHA256 校验的 CPU TorchScript actor；不启动 ROS/Isaac，也不训练。

## 模块定位与架构层级

- 架构层级：第 6 层「机器人执行」下的策略契约与推理运行时。
- 输入：`state64`（wxyz 姿态、body 角速度、12 关节 q/qd/力矩等，由仿真采集侧组装）、`command3=[vx, vy, wz]`、`last_action12`；以及数据卷里的冻结 TorchScript 权重 `runtime_assets/models/robogauge_policy.pt`。
- 输出：`float32[45]` 观测 + provenance；`(1, 12)` 动作（official 顺序，夹到 ±100 后映射回 native 顺序）；Go2HV PD 力矩（含速度相关包络）；`Runtime.metadata()` 契约字典。
- 上游 / 下游：被 [simulation/dynanav/capture/policy.py](../../../simulation/dynanav/capture/policy.py)（`load_policy`/`compute_torque`）和 [simulation/dynanav/experiment.py](../../../simulation/dynanav/experiment.py)（启动前 `verify_model`）调用；本目录不依赖仿真层。
- 边界：不做路线跟踪、不做停车、不发布 ROS 消息、不写执行器；动作目标 `q_target = q_default + 0.25 * action` 由采集侧 `PolicyDriver.step` 计算，本目录只负责观测、推理与力矩包络。

## 当前文件夹结构

```text
src/embodied_navigation/robot/unitree/robogauge/
├── README.md        # 本文件
├── __init__.py      # 1 行：包 docstring
├── contract.py      # 76 行：45D 观测契约、关节顺序映射、Go2HV 力矩
└── runtime.py       # 99 行：SHA256 校验 + CPU TorchScript actor
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 包标记，1 行 docstring "Go2 joint contract, policy observation mapping and verified actor runtime." | 无 |
| `README.md` | 本文件：45D 契约、权重校验、生产消费关系与测试 | 索引页 |
| `contract.py` | 常量 `MODEL_SHA256`、`OFFICIAL_NAMES`（策略侧 FL FR RL RR）、`NATIVE_NAMES`（仿真 body 侧 FR FL RR RL）、`Q0`、`ORDER`/`QDEFAULT`；`joint_indices(canonical)` 双向映射；`observation(state, command, last_action)` 返回 45D + provenance；`go2hv_torque(target, q, qd)` 返回 `(raw, lower, upper, clipped)` | `observation()`、`go2hv_torque()`、`joint_indices()` |
| `runtime.py` | `default_model_path()` 指向 `data_root()/runtime_assets/models/robogauge_policy.pt`；`verify_model(path)` 流式 SHA256 与 `MODEL_SHA256` 对比，不符即 `ValueError`；`Runtime` 单线程加载 CPU TorchScript、`reset()` 后以 10 帧零输入预热，再 `reset()`；`observation()` 组 45D；`__call__(x)` 校验 `(1,45)` 有限值、推理并返回 native 顺序 12 动作；`infer(obs45)` 返回 NumPy 12 维；`metadata()` 记录全部契约字段 | `verify_model()`、`Runtime(path=None, canonical=NATIVE_NAMES)`、`infer()`、`metadata()` |

## 45D 观测与 Go2HV 包络

- **45D 组成**（`contract.observation` 的拼接顺序）：`state[11:14] × 0.25`（3）+ 投影重力 `R^T·[0,0,-1]`（3）+ `command × [0.5, 1.0, 1.0]`（3）+ 关节位置相对默认姿态（12，official 顺序）+ 关节速度 `× 0.05`（12）+ 上一原始动作（12）。`provenance` 明确 `terrain_input=False`、`COM_linear_velocity_input=False`、`joint_order="FL FR RL RR; hip thigh calf"`。
- **关节顺序**：`NATIVE_NAMES` 与 `policy/native/contract.json` 的 `joint_names` 完全一致；`OFFICIAL_NAMES` 是归档策略顺序，`joint_indices()` 用名字映射而不是硬编码下标，`Runtime.__call__` 返回前用 `to_native` 换回仿真顺序。
- **Go2HV 公式**（`go2hv_torque`）：`raw = 25·(target−q) − 0.5·qd`；同向运动（`qd·raw>0`）时 base 为 20.2 Nm，否则 23.4 Nm；`|qd| ≥ 13.5 rad/s` 时按 `base·(30−|qd|)/16.5` 线性收敛到 0（下限 0），最后 `clip(raw, −limit, limit)`。这是关节 PD + 速度包络，不是 COM 速度 PID。
- **权重不可替换**：`MODEL_SHA256="bde5210a..."` 固定；换权重必须先改常量并重跑契约测试，否则加载即失败（fail closed）。

## 与其它策略契约的边界

- 45D 是 RoboGauge 自己的直接观测，不含 height scan/地形输入、不含 COM 线速度，也不是 247D 冻结观测的裁剪；两套契约的 clip/scale/关节顺序不能互换。
- 数据流：采集侧组装 `state64` → `contract.observation()` → `Runtime.infer()` → 采集侧算 `q_target` 与 Go2HV 力矩。本目录不持有指令平滑、停车或安全门。
- `metadata()` 显式声明 `proprioception_only=True`、`physics_asset_equivalence=False`、`existing_native_COM_velocity_PID=False`：CPU 推理与训练环境不宣称物理等价。

## 验证与测试入口 / 配置注意

- [tests/regression/test_robogauge_contract.py](../../../../../tests/regression/test_robogauge_contract.py)：`ORDER`/`QDEFAULT` 与 `contract.json` 一致性、45D 帧与关节顺序、错误权重在导入 torch 前被拒绝。
- [tests/unit/test_dynanav_joint_readback.py](../../../../../tests/unit/test_dynanav_joint_readback.py) 覆盖 `go2hv_torque` 的包络方向；[tests/unit/test_dynanav_rtx_source.py](../../../../../tests/unit/test_dynanav_rtx_source.py)、[test_dynanav_unlimited_acceptance.py](../../../../../tests/unit/test_dynanav_unlimited_acceptance.py)、[test_truth_deskew_and_mode.py](../../../../../tests/unit/test_truth_deskew_and_mode.py) 以 monkeypatch 替换 `default_model_path`/`verify_model`，不加载真实权重。
- 命令：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`、`... -s tests/regression`；契约测试不需要 torch，实例化 `Runtime` 需要 `torch` 与真实权重。
- 权重与模型布局见 [docs/robogauge/README.md](../../../../../docs/robogauge/README.md)；45D 与 247D 是不同契约，见 [policy/native/README.md](../policy/native/README.md)。
