# embodied_navigation 源码包

`src/embodied_navigation/` 是仓库全部自有导航实现的根：7 个一层子包按「仿真 → 感知 → 定位建图 → 规划 → 导航跟踪 → 机器人执行」分层，公共基础设施单列。第三方算法（FAST-LIVO2、SCAN、PCT、vikit、Livox 驱动）在 `third_party/`，本包只做接入与装配。

盘点日期 2026-11-20（分支 `refactor/06-clean-base`，HEAD 5b880b2）。

## 模块定位与架构层级

- 架构层级：七层架构的源码根，本身不属于某一层；包根只放包 docstring 与仓库/数据根发现，各层职责由子包承担。git 跟踪的目录节点共 20 个（包根 + 19 个子目录，其中 7 个一层子包）。
- 输入：ROS 2 话题（仿真侧 `/dynanav/lidar_rtx`、`/dynanav/camera`、`/dynanav/imu`、`/dynanav/nav/approved_cmd`；前端 `/dynanav/rtx_livo/post_vio_odometry`、`/dynanav/rtx_livo/cloud_registered_full`；真值 `/dynanav/truth/*`）、run 目录里冻结的配置、`$PROJECT_DATA_ROOT` 下的资产与模型。
- 输出：体坐标系速度指令与关节目标（仿真执行）、TF 与修正里程计（`/dynanav/backend/corrected_odometry`）、run 目录内的证据/状态文件；产物全部落在数据卷，不进仓库。
- 上游/下游：上游是 `scripts/` 的薄入口与 Isaac Sim 宿主进程；下游是各子包之间的 data flow，见「端到端数据流」与「各层入口」。
- 边界：不内置第三方算法；不写仓库内运行产物；`import embodied_navigation` 只执行 [__init__.py](__init__.py) 的 docstring，不启动进程、不建线程、不读配置。

## 当前文件夹结构

```text
src/embodied_navigation/
├── agent_runtime/             # Agent 运行时：Prefix-Stable 200K 上下文、流式 Tool-Calling、Hooks 与压缩
├── common/                    # 横切基础设施：几何/时间、有界写盘、子进程日志、状态契约
│   └── time/
├── localization_mapping/      # 定位建图：FAST-LIVO2 接入、点云 SLAM 后端、语义点层
│   ├── backend/
│   └── semantic_mapping/
├── navigation/                # 导航跟踪：RouteTracker、RoboGauge 路线跟踪、yaw 动作
│   └── robogauge/
├── perception/                # 感知适配：RGB-D 反投影、源时间速度窗
├── planning/                  # 规划：PCT 全局、SCAN 局部、交接与探索候选
│   ├── exploration/
│   └── pct/
├── robot/                     # 机器人执行：Go2 观测契约、RoboGauge MoE-CTS 运行时
│   └── unitree/
│       ├── policy/native/
│       └── robogauge/
├── simulation/                # 仿真：DynaNav/Isaac 采集、传感器、真值、operator 地图
│   └── dynanav/
│       └── capture/
├── README.md                  # 本文件
├── __init__.py                # 1 行：包 docstring
└── project.py                 # 22 行：仓库根/数据根发现
```

规模（`git ls-files` 实测）：**70 个 `.py` / 16,993 行**，88 个跟踪文件。一层子包的文件数（含各自 README）：`common` 8、`localization_mapping` 17、`navigation` 6、`perception` 4、`planning` 10、`robot` 12、`simulation` 28。

## 端到端数据流（D 路径唯一主线）

1. **启动**：`scripts/run_dynanav_native_exploration.sh [NEW_RUN_DIRECTORY]` 先占锁、冻结 run 目录与日志预算 `$RUN/configuration/capture_log.json`，再按模式组装后续进程。
2. **仿真与传感**：[simulation](simulation/README.md) 经 `scripts/run_dynanav_experiment.py` 进入 `simulation/dynanav/capture/`，在 Isaac Sim 中生成场景/物理/传感器，发布 `/dynanav/lidar_rtx`、`/dynanav/camera`、`/dynanav/imu`、`/dynanav/truth/*`，并订阅 `/dynanav/nav/approved_cmd`（`geometry_msgs/TwistStamped`）作为速度请求交给 `simulation/dynanav/capture/control.py` 的控制器换算关节目标。
3. **前端**：`scripts/run_dynanav_lio.sh` 启动 `third_party/fast_livo2`，发布 `/dynanav/rtx_livo/post_vio_odometry`、`/dynanav/rtx_livo/cloud_registered_full`、`/dynanav/rtx_livo/aft_mapped_to_init`。
4. **定位建图**：[localization_mapping](localization_mapping/README.md) 的 [backend](localization_mapping/backend/README.md) 消费 FAST-LIVO2 导出的关键帧 spool（`$RUN/observer/native_keyframes`），做 ScanContext + ICP 回环、SE(3) 位姿图、地图保存/先验地图重定位，发布 TF `map → camera_init` 与 `/dynanav/backend/corrected_odometry`；[semantic_mapping](localization_mapping/semantic_mapping/README.md) 是纯 CPU 的语义点层与只读查询。
5. **规划**：[planning](planning/README.md) 由 `scripts/run_dynanav_pct_scan.py` → `simulation/dynanav/pct_scan_worker.py` 驱动：PCT 生成全局路线（`third_party/pct`），SCAN 做局部规划（`third_party/scan`），`planning/scan_handoff.py` 与 `planning/scan_source.py` 负责交接与点云输入。
6. **跟踪与执行**：[navigation](navigation/README.md) 的 `robogauge/route_tracker.py` 把实测位姿变成有界体速指令，`yaw_action.py` 提供共享 yaw 动作；[robot](robot/README.md) 的 `unitree/robogauge/` 用 RoboGauge MoE-CTS TorchScript 策略把指令变成关节目标。
7. **Agent 感知与控制介入层**：`simulation/dynanav/operator_map.py`、`pct_scan_waypoints.py`、`native_waypoints.py`、`scripts/explore_waypoint_client.py` 与 `.agents/skills/dynanav-explore/` 提供目标下发、运行中干预与状态查询。
8. **`--truth-pose` 旁路**：跳过 FAST-LIVO2 与后端，用 `/dynanav/truth/control_odometry`、`/dynanav/truth/imu_odometry`、`/dynanav/truth/cloud_deskewed` 驱动 PCT + SCAN。

进程图与话题契约的总览见 [docs/architecture.md](../../docs/architecture.md)。

## 各层入口

| 层 | 子包 | 规模（`.py` / 行） | 关键入口 | 模块 README |
| --- | --- | --- | --- | --- |
| 基础设施 | `common/` | 7 / 796 | 库：`common.geometry`、`common.runtime_io`、`common.process_log`、`common.time` | [common/README.md](common/README.md) |
| 仿真 | `simulation/` | 25 / 11,563 | `scripts/run_dynanav_experiment.py` → `simulation/dynanav/capture_go2_rgbd.py` | [simulation/README.md](simulation/README.md)、[dynanav/README.md](simulation/dynanav/README.md) |
| 感知 | `perception/` | 3 / 142 | 库：`perception.depth.depth_cloud_packet`、`perception.pose_velocity.update_pose_velocity` | [perception/README.md](perception/README.md) |
| 定位建图 | `localization_mapping/` | 14 / 2,211 | `python -m embodied_navigation.localization_mapping.backend.node --spool … --namespace … --odometry-topic …` | [localization_mapping/README.md](localization_mapping/README.md) |
| 规划 | `planning/` | 7 / 1,086 | `scripts/run_dynanav_pct_scan.py` → `simulation/dynanav/pct_scan_worker.py` | [planning/README.md](planning/README.md) |
| Agent 运行时 | `agent_runtime/` | 6 / ~650 | `EmbodiedAgentRuntime`、`PrefixStableContextManager`、`OpenCodeToolChatClient` | [agent_runtime/README.md](agent_runtime/README.md) |
| 导航跟踪 | `navigation/` | 4 / 682 | `navigation.robogauge.route_tracker.RouteTracker`、`navigation.yaw_action` | [navigation/README.md](navigation/README.md) |
| 机器人执行 | `robot/` | 8 / 490 | `robot.unitree.robogauge.runtime`（TorchScript 策略） | [robot/README.md](robot/README.md) |

更细的 README 还有：`planning/pct/README.md`、`navigation/robogauge/README.md`、`robot/unitree/README.md`、`robot/unitree/robogauge/README.md`、`simulation/dynanav/README.md`、`simulation/dynanav/capture/README.md`；`planning/exploration/` 的文件清单见[规划页](planning/README.md)。脚本入口清单见 [scripts/README.md](../../scripts/README.md)。

## 逐文件职责（包根）

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：包总索引、端到端数据流、各层入口。 | — |
| [__init__.py](__init__.py) | 包 docstring（1 行），无重导出、无副作用。 | 无 |
| [project.py](project.py) | 仓库根与工程数据根发现，不写死个人路径。 | `project_root()`（`EMBODIED_NAV_ROOT` 覆盖，否则向上找 `pyproject.toml`）、`data_root()`（要求 `PROJECT_DATA_ROOT`，返回 `$PROJECT_DATA_ROOT/embodied-navigation`）；被 `robot/unitree/robogauge/runtime.py` 等模块与脚本导入 |

## 验证与测试入口 / 配置注意

- CPU 测试（仓库根执行，不启 ISAAC/ROS）：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit` 与 `… -s tests/regression`；文档盘点时基线为 646 条单测 + 60 条回归 0 失败（见 `AGENTS.md` 第 4 节）。测试索引见 [tests/README.md](../../tests/README.md)。
- 编译检查：`PYTHONPATH=src python3 -B -m compileall -q src scripts`。
- 配置全部在仓库 [configs](../../configs/README.md)，运行时冻结进 `$RUN/configuration/`；本包不含 `src/` 下的配置文件。DynaNav 场景、资产、模型与权重在 `$PROJECT_DATA_ROOT/embodied-navigation/`，缺失时入口会显式报错而不是回退到仓库路径。
- 运行产物（日志、点云、地图、逐帧 JSON）写 `$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>/` 或 `experiments/<run>/`，不进仓库（`AGENTS.md` 第 6 节）。
- 原生依赖（`third_party/fast_livo2`、`third_party/scan`、`third_party/pct` 等）的构建前置与许可边界见 [docs/dependencies/BUILD.md](../../docs/dependencies/BUILD.md)。

## 未验证边界

端到端 Isaac/DynaNav 运行、真实 FAST-LIVO2 吞吐、回环正确性与实机执行都需要 Linux + Isaac/ROS 环境；本文件只依据 2026-11-20 的代码、`git ls-files` 与 CPU 测试结论，未运行仿真。
