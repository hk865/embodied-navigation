# planning/launch

本目录只是 planning 的 SCAN 进程组 launch 装配，不含规划实现。

## 目录结构

```text
src/embodied_navigation/planning/launch/
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），包不导出 API
├── components.py  # 28 行，声明唯一的 scan_supervisor 进程
└── scan.launch.py  # 20 行，ROS launch 入口（run_dir 是唯一参数）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（活库） | `components.py` | 单函数 `components(run)`，用 `launching.owned()` 生成唯一 `scan_supervisor` 进程：`python -m embodied_navigation.planning.scan_epoch_supervisor --run <run> --planner <ros_prefix>/lib/scan_planner/scan_planner_node --config <ros_prefix>/share/scan_planner/config/planner.yaml --params <run>/v34_scan_overrides.yaml` |
| 代码（launch 入口） | `scan.launch.py` | `generate_launch_description()` 只声明 `run_dir` 参数，用 `OpaqueFunction` 在 launch 期解析并调用 `components()` |
| 代码（空） | `__init__.py` | 0 字节；无重导出 |
| 日志/输出 | 本目录**不直接写日志** | 进程退出记录由上层 `../../launch/launching.py` 的 `owned()` 写到 `runs/<run_id>/component_exit.jsonl`；子进程 stdio 走 launch 的 `output="screen"`，日志落盘在启动器侧 |
| 配置 | `ros_prefix` 一处 | `components.py` 调 `common.config.path("ros_prefix")`，最终值来自 `configs/launch/settings.json` 的 `runtime_paths.ros_prefix`（默认 `install/ros`，相对仓库根）；其余参数全部来自 `run_dir`（prepare 产物 `v34_scan_overrides.yaml`） |

## 模块内文件耦合

**方向结论**：单向两层，`scan.launch.py → components.py → common.config / launch.launching`，无环、无共享状态；`run_dir` 是唯一的跨文件契约（字符串路径）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `components.py` | 28 | `embodied_navigation.common.config.path`、`embodied_navigation.launch.launching.python/owned` | `scan.launch.py` | 清楚（28 行、docstring 1.0） |
| `scan.launch.py` | 20 | `components`、ROS `launch` | `../../launch/launch/demo.launch.py`（按 `("planning","scan")` include） | 清楚 |
| `__init__.py` | 0 | 无 | 无 | 清楚 |

四类问题：上帝文件无（最大 28 行）；重复实现无；跨层直达无（只依赖 `common.config.path` 与 `launch.launching` 这两个既定入口）；隐式契约一项——`components()` 返回的是 launch 动作列表（`RegisterEventHandler` + `ExecuteProcess`），"进程角色名 `scan_supervisor`、参数顺序、`v34_scan_overrides.yaml` 文件名"都是字符串约定，改了不会在 Python 层报错，只会在运行时找不到文件。

结论：耦合**清楚且合适**——本目录就是一层薄装配，职责边界与 `scan_epoch_supervisor.py` 的实现完全分离。

## 可读性评估

分级：**良好**。

- `components.py`：28 行 1 函数、docstring 比 1.0、最长函数 22 行；函数 docstring 明确写了"planner binary 与默认 config 来自 ROS prefix，run 目录提供 v34 overrides"。
- `scan.launch.py`：20 行、docstring 比 0.5，模块 docstring 说明 `run_dir` 是唯一参数；内部 `start(context)` 2 行、外层 `generate_launch_description` 7 行。
- 唯一"文档缺口"是 `__init__.py` 为空、没有导览，但本目录只有两个文件，不构成阅读障碍。

## 重构建议

没有 P0/P1，只有 P2 文档项：本目录代码不到 50 行、单向依赖、无写盘，结构上已是最小形态。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `components.py` docstring 里列出四个参数的来源表（`ros_prefix` ← configs、其余 ← run 目录） | 现在来源信息只藏在代码里 | 低 | 与 `../../../../configs/README.md` 的读取顺序对照 |
| P2 | 在 `scan.launch.py` docstring 里写明"本文件不能单独运行，只由 `demo.launch.py` include" | 旧 README 已说明，但代码里没写，直接 `ros2 launch scan.launch.py` 会缺 `run_dir` | 低 | 无代码改动，仅注释 |

## 运行与装配

`scan.launch.py` 只声明 `run_dir` 参数，`components.py` 用 `owned()` 启动唯一的 `scan_supervisor` 进程，参数为 `--run <run_dir>`、`--planner <ros_prefix>/lib/scan_planner/scan_planner_node`、`--config <ros_prefix>/share/scan_planner/config/planner.yaml`、`--params <run_dir>/v34_scan_overrides.yaml`。`ros_prefix` 来自 `../../../../configs/launch/settings.json` 的 `runtime_paths.ros_prefix`，默认是仓库根下的 `install/ros`。

SCAN planner 二进制与配置随 `install/ros` 构建产物提供（由 `../../../../docker/build_native.py` 构建），源码在 `../../../../third_party/README.md` 所列的 SCAN 包中；本仓库不携带构建产物，install 缺失只表示本机未构建。该组被 `../../launch/launch/demo.launch.py` 以 `('planning','scan')` 引入，没有可单独执行的入口。真实实现在 `../scan_epoch_supervisor.py`，见 `../README.md`。

## 测试

本目录无专用测试；通用启动 CLI 由 `../../../../tests/unit/test_launch_cli.py` 覆盖，不验证 `scan_supervisor` 实际可启动。测试分层见 `../../../../tests/README.md`。
