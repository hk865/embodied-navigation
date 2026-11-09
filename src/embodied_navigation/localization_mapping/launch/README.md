# localization_mapping/launch

本目录装配 `localization_mapping` 的 ROS 进程：一个 launch 文件加一个组件清单，由顶层 `demo.launch.py` 包含。这里没有算法，只有"起哪些进程、传什么参数、重映射哪些话题"。

## 边界

- 不实现任何定位/建图逻辑：8 个角色里只有 3 个属于本模块（`odom_adapter`、`map_archive`、`global_matching`），`slam` 是第三方二进制，`sensor_relay`/`self_filter`/`sensor_gate` 来自 `perception.sensors`，`camera_parameters` 是 `/opt/ros/jazzy` 的 demo 节点。
- 不设进程环境：ROS domain、DDS、`DEMO_RUN_DIR` 由 `launch/main.py` 注入，本目录不设环境变量。
- 不做启动前检查：缺失的 `install/ros`/`install/actuators` 由 `launch/preflight.py` 拦截，本目录直接 `ExecuteProcess` 启动。

## 目录结构

```text
src/embodied_navigation/localization_mapping/launch/
├── README.md         # 本文件
├── __init__.py       # 空占位（0 行，空文件）
├── components.py     # 8 个角色的有序组件清单，含重映射与环境（94 行）
└── slam.launch.py    # 只声明 run_dir，用 OpaqueFunction 调 components(run)（14 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `components.py` | 单函数 `components(run)` 83 行，docstring 覆盖 1.0；用 `common.config.path`、`common.runtime_context.load`、`launch.launching.{python,owned,ROS}` 构造 8 个 `ExecuteProcess` 并展平返回。 |
| 代码 | `slam.launch.py` | `generate_launch_description()` 只做两件事：`DeclareLaunchArgument("run_dir")` + `OpaqueFunction(start)`，`start()` 里 `components(Path(run_dir).resolve())`。 |
| 代码 | `__init__.py` | 空文件（0 行）。 |
| 日志/输出 | `<run>/component_exit.jsonl` | **不是本目录写的**：由 `launch/launching.py` 的 `owned()` 在任一子进程退出时追加 `{"role", "pid", "returncode"}`，并触发 `Shutdown`。本目录只是调用 `owned()`。 |
| 日志/输出 | 子进程 stdout/stderr | `ExecuteProcess(output="screen")` 直接进 launch 终端；本目录不落盘日志。 |
| 配置 | `<run>/runtime_context.json` | `components()` 读 `profile`；`profile` 决定 `sensor_relay` 的 `--mode`（默认 `"async"`）。读取顺序见 [configs/README.md](../../../../configs/README.md)。 |
| 配置 | `<run>/navigation_scenario.json`、`navigation_camera.yaml`、`navigation_fastlivo.yaml` | 分别作为 `--scenario`/`--params-file` 传给 `self_filter`、`slam`、`odom_adapter`、`camera_parameters`；都是 `launch/prepare.py` 生成的 run 目录文件。 |
| 配置 | `path("ros_prefix")` | 本目录唯一重读 `configs/` 资产路径的地方（`configs/launch/settings.json` 的 `ros_prefix` = `install/ros`），用于拼 `install/ros/lib/fast_livo2_ros/fastlivo_mapping`。 |
| 配置 | 无其它 `configs/` 读取点 | 门限/阈值不在本目录，任何参数改动都应改 `configs/` 或 `prepare.py`，不是改这里。 |

## 模块内文件耦合

依赖方向：`slam.launch.py → components.py → {common.config, common.runtime_context, launch.launching}`，单向、无环。`components.py` 不 import 它启动的那些节点模块（只用模块名字符串），因此"改节点文件"不会牵动本目录，反之亦然。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `components.py` | 94 | `common/config`、`common/runtime_context`、`launch/launching` | `slam.launch.py` | 清楚：单函数、docstring 完整；但 83 行把 8 条命令全铺在一个函数里 |
| `slam.launch.py` | 14 | `components.py`、ROS `launch` | 顶层 `demo.launch.py`（通过文件路径 Include） | 清楚：标准 launch 壳 |
| `__init__.py` | 0 | 无 | 无 | 清楚：占位 |

四类问题：

- **上帝文件**：无（94 行）。
- **重复实现**：无。与 `control/launch/components.py` 等兄弟模块同构，但各自维护自己的角色清单，属于合理重复。
- **跨层直达**：有一处需要注意——`components.py` 直接按 `install/ros/lib/fast_livo2_ros/fastlivo_mapping` 路径拼第三方二进制，绕过了 ROS 包解析（`ros2 launch`/`package.xml`）。这是本仓库的既定做法（`install/` 是构建产物目录），但换构建布局时必须同步改这里与 `preflight.py`。
- **隐式契约**：8 个角色的**顺序**是隐式的（sensor_relay 必须先于 self_filter/slam，`map_archive` 依赖 slam 的话题），代码只靠 list 顺序表达，没有注释或断言；另外 `profile.get("sensor_relay_publish_mode", "async")` 和话题 remap 字符串（`-r /x:=/y`）是字符串契约，与各节点的硬编码话题名两处需同步。

结论：耦合**清楚**。launch 层天生是"清单"，用单函数铺开 8 条命令是可接受的写法；唯一值得改的是顺序契约没有文档化。

## 可读性评估

- **良好**：`slam.launch.py`（14 行、2 函数、`generate_launch_description` 有 docstring，职责单一）；`components.py`（函数 docstring 1.0，模块 docstring 说明"每个角色一个自有子进程、话题重映射让前端只看到项目自有的源"）。
- **可接受**：`components.py` 的 83 行单函数——8 个角色各自是 5–10 行的元组，结构重复但有规律；`ROS` 常量（`--ros-args -p use_sim_time:=true`）与 `python()` 包装来自 `launch/launching.py`，不是本目录的魔法值。
- **文档缺口**：角色顺序与依赖关系只体现在 list 位置；`camera_parameters` 依赖 `/opt/ros/jazzy` 的 `demo_nodes_cpp` 这一点只在 README 里，代码里没有注释。
- 这是**文档问题**而非结构问题：94 行、单函数、无分支嵌套。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `commands` 列表每项旁加一行注释说明"依赖前一个角色的哪些话题" | 现在顺序即依赖，但没有任何提示；重排会静默坏掉 | 无（纯注释） | 人工复核 |
| P2 | 把 8 个角色的命令构造抽成 8 个小函数（`_sensor_relay(run, profile)` …） | 83 行单函数里 8 段结构高度相似，抽出来后 diff 更小、单测可逐角色断言 | 低：纯搬移，顺序不变 | 用 `test_launch_cli` 之外的静态检查：`python3 -c "from embodied_navigation.localization_mapping.launch.components import components"` + 传入假 run 目录打印角色名序列 |
| P3 | 在 `components.py` 顶部注明 `install/ros` 布局与 `preflight.py` 的对应关系 | 换构建布局时两处需同步 | 无 | 人工复核 |

没有 P0/P1：本目录不参与算法正确性，8 个进程的启动参数与重映射在 README 里已可逐条核对。

## 8 个角色与话题

- `sensor_relay`、`self_filter`、`camera_parameters`、`slam`、`odom_adapter`、`map_archive`、`global_matching`、`sensor_gate`（代码顺序即启动顺序）。
- `sensor_relay`、`self_filter`、`sensor_gate` 来自 `perception.sensors`，不属于本模块；`self_filter` 把 `/livox/lidar` 重映射成 `/demo/teacher/raw_lidar`。
- `slam` 直接执行 `install/ros/lib/fast_livo2_ros/fastlivo_mapping`，带两个 `--params-file`，并把 `/cloud_registered_full` 改到原始名 `/demo/slam/raw_registered_full`。
- `odom_adapter` 对应 [../odom_adapter.py](../odom_adapter.py)（`-r /demo/slam/{body,lidar}_odom:=/demo/slam/raw_*`）；`map_archive` 对应 [../mapping/map_archive.py](../mapping/map_archive.py)；`global_matching` 对应 [../localization_runtime.py](../localization_runtime.py)。

## 测试与限制

本目录无专用测试；[tests/unit/test_launch_cli.py](../../../../tests/unit/test_launch_cli.py) 覆盖的是通用启动 CLI，不验证这八个组件是否真能启动。

普通 `start.py` 启动会先由全局文件前检（[launch/preflight.py](../../launch/preflight.py)）拦截缺失的 `install/ros/lib_setup.bash`、`install/ros/lib/fast_livo2_ros/fastlivo_mapping`、`install/actuators` 等产物；本检出 `install/ros` 不存在，因此本目录内容只能静态阅读。若直接调用本 launch，`camera_parameters` 另依赖 `/opt/ros/jazzy` 下的 `demo_nodes_cpp` 节点，本机是否安装属**未核实**。
