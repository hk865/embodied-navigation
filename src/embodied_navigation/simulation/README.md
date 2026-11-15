# simulation

本目录是仿真与场景生命周期的公共层：世界/传感器契约生成、DynaNav 场景接入、Gazebo 关节力矩插件，以及仿真安全状态与 actor 就绪屏障。具体世界与实验编排在子目录，本文件只做索引。

## 目录结构

```text
src/embodied_navigation/simulation/
├── actuators/          # 子模块，见 actuators/README.md（Gazebo 关节力矩插件 C++）
├── dynanav/            # 子模块，见 dynanav/README.md（Isaac/DynaNav 实验路径）
├── launch/             # 子模块，见 launch/README.md（仿真进程装配）
├── world/              # 子模块，见 world/README.md（世界/传感器契约生成）
├── README.md           # 本文件
├── __init__.py         # 1 行，包 docstring（"Simulation and scene lifecycles, independent of navigation algorithms."）
├── safety_state.py     # 73 行，Gazebo native64 → PhysicalSafetyState（simulation_truth / simulation_safety）
└── wait_actor.py       # 24 行，actor 就绪屏障：worker_ready 的 PID 存活 + Unix socket 已监听
```

4 个子目录 + 3 个顶层 Python 文件（合计 98 行）。子目录各自的目录树与耦合见对应 README，此处不展开。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `safety_state.py` | 活代码：`from_gazebo_native64(native_state, *, first_world_time_s, ground_height_m, run_id)` 把 64 维帧校验成 `PhysicalSafetyState`（roll/pitch、`body_clearance_m`、5 组接触、执行器 fault code），`sample_stamp_ns = (state[0] - 0.005)·1e9` 明确"读上一物理步"。消费者：`robot/unitree/policy/native/worker.py`。 |
| 代码 | `wait_actor.py` | 活代码：等 `worker_ready` 的 PID 存活与 Unix socket 监听，供 launch 当进程使用；不 import 内部模块。 |
| 日志/输出 | 无 | 本层不写 run 目录；世界/实验产物由 `world/`、`dynanav/` 与入口脚本负责。 |
| 配置 | 本目录无读取点 | 世界/传感器参数由 `configs/simulation/` 与 run 内生成物提供，见 [configs/README.md](../../../configs/README.md)。 |

## 耦合与可读性

- 依赖方向（单向，无环）：`safety_state.py → common.state_contract`；`wait_actor.py` 不 import 任何内部模块（纯标准库）。
- 被依赖方向：`safety_state` 被 native worker 与 `tests/` 使用；`wait_actor` 只被 launch 当进程启动。
- `safety_state.py`：**清楚**。73 行纯工厂函数，消费者一行调用；真值/安全边界由 `StateProvenance` 强校验，不把仿真真值写成导航定位输入。
- `wait_actor.py`：**可接受**。24 行，就绪判据（文件 + PID + socket）集中在一处。

## 测试

`safety_state.py` 有 [tests/unit/test_simulation_safety.py](../../../tests/unit/test_simulation_safety.py) 与 [tests/regression/test_native_safety_integration.py](../../../tests/regression/test_native_safety_integration.py) 覆盖；`wait_actor.py` 没有专用单测，只随 launch 使用。

## 限制

本次核对没有启动 Gazebo、Isaac 或 ROS；`wait_actor.py` 的实际启动路径未验证。仿真层通过不等于完整系统或路线验收。
