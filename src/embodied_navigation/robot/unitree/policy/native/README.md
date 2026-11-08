# robot/unitree/policy/native：247D 中间观测与 native64 循环

本目录保留历史 Teacher 的 native 中间观测（247D），并提供当前 Student 复用的 socket 驱动 native64 关节控制循环。普通导航不加载 Teacher actor，247D 仅用于诊断，实际 Student 推理使用 exact45；本目录不直接面向实机。

## 关键文件与接口

- `observation.py`：`build_observation(state, command, last_action, terrain, ...)` 用静态 SDF 碰撞做 187 条竖直向 raycast，按契约生成 float32[247] 观测及高度扫描诊断（raw/hit、clip 比例等）；`quaternion_rotation()` 按 wxyz 计算 world-from-body 旋转。
- `contract.json`：冻结 247D 拼接顺序、各段 scale/clip/noise、关节名与默认位置、动作公式、DCMotor actuator 参数、50 Hz / 0.005 s 时序，是数值事实来源。
- `worker.py`：通过 Unix socket 接收 64 个 float64 state，`prepare_velocity_input()` 做指令 slew，构造 247D 观测、调用 actor、输出 16 个 float64（前 12 为关节目标，`[12]` 为 fault，`[13]` 为 done，`[14:16]` 保留为 0）；每帧调用 `SimulationSafetyGate` 与 `from_gazebo_native64`，并写 `worker_result.json`、`state.json` 等。

## 输入输出

`state[0]` 为世界时间，`[8:11]` 是 body-frame COM 速度，`[11:14]` 角速度，`[14:26]` / `[26:38]` 关节位置/速度，`[50:55]` 接触；对端是 Gazebo actuator 的 native64 socket。

## 与 45D 的关系

247D 是历史 Teacher 的中间观测适配链：[../worker.py](../worker.py) 复用它；Student 实际推理在 [../student.py](../student.py) 用 exact45，247D 仅供诊断。[robogauge/README.md](../../robogauge/README.md) 的 45D 是完全独立的直接观测接口，两者不是同一个策略输入或 actuator 契约。

## 测试与限制

[tests/regression/test_native_safety_integration.py](../../../../../../tests/regression/test_native_safety_integration.py) 用真实 247D 观测构造与假 socket 执行本循环并检查共享门禁；[tests/regression/test_robogauge_contract.py](../../../../../../tests/regression/test_robogauge_contract.py) 校验 `quaternion_rotation` 与 `contract.json` 字段。地形来自 SDF 静态碰撞而非在线估计，仅用于仿真。
