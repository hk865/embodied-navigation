# simulation/gazebo

本目录是 Gazebo 仿真部署的归置目录：当前只有 `robogauge` 一个子目录，本层没有独立可执行文件。

## 目录结构

```text
src/embodied_navigation/simulation/gazebo/
├── robogauge/          # 子模块，见 robogauge/README.md
├── README.md           # 本文件
└── __init__.py         # 1 行，仅包 docstring（"Portable RoboGauge simulation deployment components."）
```

本层只有 1 个 Python 文件、1 行；没有配置、日志或运行入口。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `__init__.py` | 不是活代码：只有模块 docstring，无函数、无 import、无副作用。它让 `embodied_navigation.simulation.gazebo.robogauge` 成为可导入的包路径。 |
| 日志/输出 | 无 | 本模块不直接写日志。证据：事实包 grep 命中全 0（env/config/output_dir/file_write/telemetry/logging/print/process/ipc/ros 均为 0），且目录内只有 1 行文件。 |
| 配置 | 无配置读取点 | 本层不读 `configs/`；RoboGauge 部署的配置读取点全在 `robogauge/world.py`（见 [robogauge/README.md](robogauge/README.md)）。 |

## 模块内文件耦合

- 依赖方向：本层无内部依赖。`__init__.py` 不 import 子模块；子模块 `robogauge` 在需要时才被 `navigation/robogauge/benchmark.py` 直接导入（按完整包路径），不经过本层的重导出。
- 结论：耦合为零，不存在环、共享状态或隐式契约。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 1 | 无 | 无（无人 `import embodied_navigation.simulation.gazebo` 取符号） | 清楚：纯包标记 |
| `robogauge/` | 见子 README | 见子 README | `navigation/robogauge/benchmark.py`、`tests/regression/test_robogauge_*.py` | 清楚：唯一子模块 |
| `README.md` | 本文件 | — | 人 | — |

四类问题均不存在（无上帝文件、无重复实现、无跨层直达、无隐式契约）。

## 可读性评估

- `__init__.py`：**良好**。1 行 docstring 与目录实际内容（RoboGauge 部署组件）一致，没有夸大成"通用 Gazebo 层"。
- 本 README 的职责就是归置说明，不需要按代码模块评级。

## 重构建议

没有 P0/P1/P2 建议：本层只有 1 行包标记，删掉或合并它只会改动 `robogauge` 的导入路径（`navigation/robogauge/benchmark.py` 按完整包路径导入 `...gazebo.robogauge.worker`），收益为零、风险非零。保持现状即可。

规则层面只有一条约定（非重构项）：`robogauge` 之外若还要放别的 Gazebo 部署，按同样约定新增子目录并配自己的 README，不要在本层写进程逻辑。

## 测试与限制

- [robogauge/README.md](robogauge/README.md) 承载 RoboGauge MoE-CTS 的 Gazebo 部署。
- 通用 Teacher/Student Gazebo 链不在这里：世界生成在 [world](../world/README.md)，进程装配在 [launch](../launch/README.md)，关节力矩插件在 [actuators](../actuators/README.md)。
- 上层模块说明见 [simulation/README.md](../README.md)。
- 本目录无专用测试，也未在本次核对中启动 Gazebo；`robogauge` 的部署行为未在本层验证。
