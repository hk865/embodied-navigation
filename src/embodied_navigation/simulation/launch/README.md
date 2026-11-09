# simulation/launch

本目录装配通用仿真进程：actor 就绪后再启动 Gazebo，由顶层 `demo.launch.py` 包含。

## 目录结构

```text
src/embodied_navigation/simulation/launch/
├── README.md               # 本文件
├── __init__.py             # 0 行（空文件，仅包标记，无 docstring、无 import）
├── components.py           # 63 行，构造 actor → sensor_bridge → actor_ready → gz sim 的受管 launch actions
└── simulation.launch.py    # 14 行，launch 入口：声明 run_dir，经 OpaqueFunction 调 components(run)
```

本层只有 launch action 构造，没有节点实现；无配置、无日志。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `simulation.launch.py` | 活代码：launch 入口。`generate_launch_description()` 声明必填 `run_dir`，用 `OpaqueFunction` 把解析后的目录交给 `components`。 |
| 代码 | `components.py` | 活代码：唯一导出 `components(run)`（55 行，内部含 `ready` 闭包）。按顺序组装 4 段 action：`actor`（`embodied_navigation.robot.unitree.policy.worker`，`ros=False`，`CUDA_VISIBLE_DEVICES=""`）、`sensor_bridge`（`ros_gz_bridge/parameter_bridge` 加载 `<run>/sensor_bridge.yaml`）、`actor_ready` 屏障（`python -m embodied_navigation.simulation.wait_actor`）、以及屏障成功退出后才创建的 `gz sim -s -r --headless-rendering <run>/world.sdf --seed 42`。 |
| 代码 | `__init__.py` | 空文件，不是活代码；没有重导出，`from embodied_navigation.simulation.launch import components` 会失败（要按模块路径 import）。 |
| 日志/输出 | 无自有文件 | 本目录不写日志、不创建目录。它只消费 run 目录里已由 prepare 生成的 `world.sdf`、`sensor_bridge.yaml`，以及 actor 进程写出的 `worker_ready`（PID 文本）+ Unix socket；日志是各子进程自己的 stdout/stderr。 |
| 配置 | `common.config.path("actor_python")` | 唯一配置读取点：actor 解释器路径来自启动器配置（`configs/launch/settings.json` 经 `common.config` 读入）。 |
| 配置 | 环境变量 `TEACHER_SOCKET` | `components()` 用 `os.environ["TEACHER_SOCKET"]` 直接取，缺失即 `KeyError`；由调用方 `launch/main.py` 提前注入。本层不读 `configs/` 文件本身。 |

硬编码常量（未改代码，仅记录）：`sensor_bridge` 的可执行文件路径写死为 `/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge`；`gz` 从 PATH 取；Gazebo 种子写死 42；就绪屏障上限 30 s、轮询 50 ms 由 `../wait_actor.py` 内部决定，launch 侧不可配。

## 模块内文件耦合

- 依赖方向：`simulation.launch.py → components.py → {launch.launching.owned/python, common.config.path}`，单向向下、无环。对 actor、屏障、Gazebo 不 import，而是按进程路径启动（`python -m` 与可执行文件），耦合面是**命令行参数与环境变量**。
- 本层没有共享可变状态；`run` 目录是唯一共享句柄，四个子进程都以它为工作上下文。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `components.py` | 63 | `common.config`、`launch.launching`，以及外部进程 `policy.worker`/`wait_actor`/`parameter_bridge`/`gz` | `simulation.launch.py` | 清楚：一个函数就是一条装配顺序 |
| `simulation.launch.py` | 14 | `components.py`、`launch` | 顶层 `demo.launch.py`（按 `simulation` 模块包含） | 清楚：只做参数解包 |
| `__init__.py` | 0 | 无 | 无 | 清楚：空包标记 |

四类问题：上帝文件无；重复实现无；**跨层直达有（轻微）**：`components.py` 直接写死 ROS 发行版路径并直接生成 `gz sim` 命令，绕过了配置层；隐式契约有：`TEACHER_SOCKET` 必须在包含本 launch 之前存在、`<run>/sensor_bridge.yaml` 与 `<run>/world.sdf` 必须已由 prepare 生成、`worker_ready` 的内容必须是可 `int()` 的 PID —— 三者都靠调用顺序而非校验。

## 可读性评估

- `components.py`：**可接受**。优点：`components` 有 docstring 写明"actor → sensor bridge → Gazebo after actor readiness"；顺序通过线性 `actions +=` 可读；`ready` 闭包里失败即 `EmitEvent(Shutdown(...))`，没有静默失败。不足：模块本身**无 module docstring**，`owned()`/`python()` 的语义（受管进程、退出即整体关停）只在函数体里体现；`parameter_bridge` 绝对路径、`--seed 42`、`-s -r --headless-rendering` 三处魔法值没有注释；`components` 55 行 + 闭包，靠缩进表达阶段。
- `simulation.launch.py`：**良好**，14 行，模块 docstring 说清"resolve the required run_dir and start owned components"。
- `__init__.py`：空文件，无评价意义；但"import 本包拿不到符号"这一点未写进任何注释（见重构建议）。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `components.py` 补 module docstring，写清四段进程的职责、`owned()` 的关停语义、`TEACHER_SOCKET` 的前置条件 | 现在读 launch 文件要靠反推 `launching.owned` | 无（文档） | 人工复核 |
| P2 | 把 `parameter_bridge` 绝对路径与 `--seed` 提成模块常量（或走 `common.config.path`） | 换 ROS 发行版/要复现实验时只能改代码 | 低：行为等价 | `start.py --prepare-only` 后检查生成的 launch 参数；`tests/unit/test_launch_cli.py` 仍通过 |
| P2 | 在 `__init__.py` 加一行注释或 docstring 说明"本包不重导出，请按模块路径 import" | 空文件容易让人以为能 `from ...launch import components` | 无 | 人工复核 |
| P2 | 把"屏障 30 s + socket/文件双条件"写进本 README 的显式契约（现已写）并考虑参数化 | 超时写死在 `wait_actor.py`，与 launch 侧无连接 | 低：新增 launch 参数需同步默认值 | 用假 `worker_ready`/socket 手工验证超时路径 |

没有 P0/P1：本目录 77 行、单向依赖、失败会显式关停，结构问题很小；缺口在"跨层硬编码路径"和"前置条件只在代码里"，属 P2。本轮只写建议、不改代码；改启动顺序或用例前先确认 `tests/unit/test_launch_cli.py` 与 `tests/unit/test_ros_shutdown_lifecycle.py` 不依赖这些文本。

## 测试与限制

- 注意：这里启动的是当前普通 Student/native 链（协议仍有 Teacher 历史命名）；Building 的 `truth_worker` 由 `launch/building.py` 单独启动，RoboGauge 的 `worker` 由 `navigation/robogauge/benchmark.py` 启动。
- 屏障自身不 import ROS（`ros=False`），最多等 30 秒；它检查运行目录下 `worker_ready` 文件中的 PID 是否仍存活，以及 socket 是否已是 Unix socket，两者都满足才放行 Gazebo。
- 本目录无直接验证组件编排或就绪屏障的专用测试。上层 CLI 参数检查见 [tests/unit/test_launch_cli.py](../../../../tests/unit/test_launch_cli.py)；`sensor_gate`、`pid_entrypoint` 节点的信号与 executor/context 关停测试见 [tests/unit/test_ros_shutdown_lifecycle.py](../../../../tests/unit/test_ros_shutdown_lifecycle.py)。这些测试不验证本目录组件实际可启动；本次核对没有启动 launch、Gazebo 或 ROS，端到端启动未验证。
