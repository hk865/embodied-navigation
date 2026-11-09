# control/launch

本目录只是控制进程组的 ROS launch 装配，不含任何控制算法。

## 边界

- 做：声明 `run_dir` 参数，按固定顺序启动 `controller`、`command_bridge`、`mission` 三个受管进程。
- 不做：不实现控制律、不读业务配置、不判断任务状态；真实控制实现在 [tracking/README.md](../tracking/README.md)、[parking/README.md](../parking/README.md)、[safety/README.md](../safety/README.md)，任务状态机在 [navigation/README.md](../../navigation/README.md)。
- 本目录没有可单独执行的入口：`run_dir` 由 prepare 阶段生成，见 [configs/README.md](../../../../configs/README.md) 的读取顺序。

## 目录结构

```text
src/embodied_navigation/control/launch/
├── README.md               # 本文件
├── __init__.py             # 0 行，空包声明
├── components.py           # 27 行，返回三个受管 LaunchAction 的有序列表
└── control.launch.py       # 14 行，ROS launch 入口，只声明 run_dir
```

实测合计：3 个 Python 文件 41 行。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `components.py` | 活代码；`components(run)` 用 `launch.launching.python` 与 `owned` 组装 controller / command_bridge / mission |
| 代码 | `control.launch.py` | 活代码；`generate_launch_description()` 用 `OpaqueFunction` 把 `run_dir` 解析成绝对路径后交给 `components` |
| 代码 | `__init__.py` | 空文件，无逻辑、无副作用 |

三个组件命令（`components.py` 原文）：

- `controller`：`embodied_navigation.navigation.pid_navigation --run <run_dir>`
- `command_bridge`：`embodied_navigation.robot.unitree.bridge --runtime-config <run_dir>/runtime_context.json --command-file <run_dir>/navigation_command.json`，读 `runtime_context.json`，把 `/demo/cmd_vel` 写成 `navigation_command.json`
- `mission`：`embodied_navigation.navigation.mission.mission_runtime --run <run_dir>`

进程生命周期与退出汇总由 [../../launch/main.py](../../launch/main.py) 统一管理（`owned(role, command, run)`）。该组被 `../../launch/launch/demo.launch.py` 以 `('control','control')` 引入，与 localization、planning、simulation 三个同构组并列。

**日志/输出**：本目录不写任何文件、不创建目录、不打印；进程日志与退出码归 `../../launch/main.py` 的受管进程框架所有。`run_dir` 下的 `navigation_command.json` 由 `command_bridge` 进程写，不是本目录写。

**配置**：本目录没有 `configs/` 读取点。`run_dir` 是 launch 参数（prepare 生成）；`runtime_context.json` 由 `command_bridge` 进程自己按 `--runtime-config` 读取，本目录只传路径。

## 模块内文件耦合

依赖方向单向：`control.launch.py → components.py → ../../launch/launching.py`，无环、无重复实现。41 行里 `components()` 是唯一组合点。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `components.py` | 27 | `embodied_navigation.launch.launching`（`python`、`owned`） | `control.launch.py` | 清楚 |
| `control.launch.py` | 14 | `control.launch.components`、ROS `launch` | 无 import 方（被 `demo.launch.py` 以文件路径引入） | 清楚 |
| `__init__.py` | 0 | 无 | 无 | 清楚 |

四类问题：

- **上帝文件**：无（最大 27 行）。
- **重复实现**：无；三组组件都走同一个 `python()` 包装。
- **跨层直达**：无；只依赖本仓库自己的 launch 工具层，不 import 任何控制/导航模块。
- **隐式契约**：只有一条 —— role 名 `controller` / `command_bridge` / `mission` 会被 `owned()` 用于进程标识与退出汇总，改名会改变日志与监督记录里的角色字符串。

结论：耦合**清楚且合适**。41 行、单向依赖、无状态，没有可拆的结构。

## 可读性评估

- `components.py`：**良好**。模块 docstring 说明三个进程分工（controller 发布 `/demo/cmd_vel`、bridge 校验并落盘、mission 协调请求/状态），`components()` 1 个函数 17 行、docstring 比 1.0。
- `control.launch.py`：**可接受**。14 行、2 个函数、docstring 比 0.0；`OpaqueFunction` + `DeclareLaunchArgument` 的写法是本仓库 launch 的固定范式，短到没有结构问题。
- `__init__.py`：空文件，无需评估。
- 这是"小模块只缺 docstring"的典型：没有结构问题，唯一缺口是 `run_dir` 的含义与"必须已由 prepare 生成"这一约束只写在 README 里，代码里没有。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `control.launch.py` 的 `start()` 补一句 docstring，写明 `run_dir` 由 prepare 生成且必须存在 | 唯一文档缺口 | 无 | 自检脚本 |
| P2 | 把三个 role 名提成模块级常量（如 `ROLES = ("controller","command_bridge","mission")`），`components()` 与日志引用同一来源 | 隐式字符串契约 | 无 | `../../../tests/unit/test_launch_cli.py` |

没有 P0/P1：42 行、单向依赖、无状态、无可拆结构。本轮只写建议，未改代码。

## 测试

本目录无专用测试；通用启动 CLI 由 `../../../tests/unit/test_launch_cli.py` 覆盖，不验证本组三个组件实际可启动。

## 限制与未验证

- 三个组件"能被 `owned()` 起起来"这件事**未被测试覆盖**，也未在实机/完整路线上验证；测试只覆盖 launch 命令行本身。
- 本目录不含任何控制或安全逻辑，不能据此推断控制行为；控制实现在同级的三个子模块。
