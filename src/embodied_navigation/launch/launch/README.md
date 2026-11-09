# launch/launch：ROS2 launch 文件目录

本目录不是独立执行入口，只存放被 `launch/main.py` 调用的 ROS2 launch 描述文件。

## 边界

- 只声明"包含哪些模块 launch 文件"与 `run_dir` 参数，不启动进程、不读写 run 文件、不构造环境。
- 真正的进程列表与开关在四个模块各自的 `launch/components.py` 里；本目录改不动组件行为。
- 不提供 `main` 或命令行入口；直接 `python -m` 运行没有意义。

## 目录结构

```text
src/embodied_navigation/launch/launch/
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），包标记
└── demo.launch.py  # 40 行，include 四个模块 launch 文件并共享 run_dir
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `demo.launch.py` 40 行；`__init__.py` 空文件（0 行） | `generate_launch_description()` 是唯一函数：声明 `run_dir` 参数，然后依次 include `localization_mapping/launch/slam.launch.py`、`planning/launch/scan.launch.py`、`control/launch/control.launch.py`、`simulation/launch/simulation.launch.py`。`__init__.py` 只是包标记。 |
| 日志/输出 | 无 | 本目录不写任何文件、不打印：`demo.launch.py` 里没有 `open(`/`write`/`print`，进程日志由被 include 的组件自己写。 |
| 配置 | 无配置文件读取点 | 只从 `common.config` 取 `ROOT` 拼绝对路径；唯一"配置"是 launch 参数 `run_dir`（由 `launch/main.py` 以 `run_dir:=<run目录>` 传入）与写死的模块映射表。 |

## 模块内文件耦合

依赖方向单一：`demo.launch.py → common.config`（只要 `ROOT`）+ 四个外部 launch 文件；本目录无反向依赖（`launch/main.py` 是用 `ros2 launch` 子进程启动它，不是 import）。无环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `demo.launch.py` | 40 | `common.config.ROOT`、4 个模块 launch 文件 | `launch/main.py`（`ros2 launch` 命令行） | 清楚（纯装配清单） |
| `__init__.py` | 0 | 无 | 无 | 清楚 |
| `README.md` | — | — | — | — |

四类问题核查：没有 >800 行文件；没有重复实现；没有跨层直达（不 import 任何业务模块）；有一处**隐式契约**——`modules` 列表里 `("localization", "slam")` 的逻辑名到包名映射（`localization` → `localization_mapping`）写在条件表达式里，新增模块要同时改映射与目录名；另外四个子 launch 文件都必须接受同名 `run_dir` 参数，否则 include 成功但参数落不到组件。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `demo.launch.py` | 良好 | 40 行、1 个函数 27 行、0 个 >80 行函数、无分支嵌套；模块 docstring 明确写出"逻辑名到包路径"的映射；唯一缺口是 `generate_launch_description()` 本身没有 docstring（docstring 比 0.0） |
| `__init__.py` | 可接受 | 空文件（0 行）包标记，无 docstring |

结构问题：无（整个目录 41 行）。文档问题：函数级 docstring 缺失；映射表是列表字面量，新增模块时需要读代码才知道规则。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `generate_launch_description()` 补 docstring，写明"四个 include 必须都接受 `run_dir`" | 当前只靠模块 docstring 的一半信息 | 低 | 纯文档 |
| P2 | 把 `modules` 映射表改成显式 `(package_dir, launch_name)` 二元组列表 | 现在的条件表达式把 `localization→localization_mapping` 特例藏在列表推导里 | 低 | 对照四个文件路径存在即可；无测试覆盖 |

本目录规模小、职责单一：**没有 P0/P1，只有 P2 文档/小整理项**。本轮只写建议、不改代码。

## 关键文件与接口（原内容）

- `demo.launch.py`：`generate_launch_description()` 声明 `run_dir` 参数，并依次 include 四个模块的 launch 文件：[slam.launch.py](../../localization_mapping/launch/slam.launch.py)、[scan.launch.py](../../planning/launch/scan.launch.py)、[control.launch.py](../../control/launch/control.launch.py)、[simulation.launch.py](../../simulation/launch/simulation.launch.py)。
- 各子 launch 文件本身很薄，真正的进程列表在其模块的 `launch/components.py` 中。
- `__init__.py` 只是包标记；本目录没有 `main` 或命令行入口。

使用方式是 `launch/main.py` 以 `ros2 launch .../demo.launch.py run_dir:=<run目录>` 启动；它负责准备 run 文件、构造环境并监控退出。包代码与使用说明见 [../README.md](../README.md)。本目录无专用测试。
