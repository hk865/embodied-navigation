# scripts（DynaNav 入口脚本与离线工具）

盘点日期 2026-11-20。本目录是 D 路径（DynaNav / Isaac Sim）的薄 CLI 入口与离线工具集：解析参数、编排进程、调用 `src/` 里的包；业务逻辑一律在 `src/`，脚本互相之间不 `import`，也不是 Python 包或 `pyproject.toml` 的 console script。

## 模块定位与架构层级

- 架构层级：不属于七层中的任何一层，是跨层的**编排/工具层**。主入口 [`run_dynanav_native_exploration.sh`](run_dynanav_native_exploration.sh) 串起仿真（`src/embodied_navigation/simulation/dynanav/`）、估计器（`third_party/fast_livo2` 的 `fastlivo_mapping` + `scripts/run_dynanav_lio.sh`）、SLAM 后端（`localization_mapping/backend/node.py`）、规划（`run_dynanav_pct_scan.py` → `pct_scan_worker.main`）与 Agent 介入层（`explore_waypoint_client.py`）。
- 输入：人的命令行参数、`.env.local` 提供的 `PROJECT_DATA_ROOT`、`configs/simulation/dynanav/` 下的估计器 YAML、外部数据卷上的 build prefix / SCAN install / 场景 USD、run 目录里的收据与点云。
- 输出：数据卷 `$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>/` 下的 run 目录（`capture/`、`observer/`、`backend.log`、`planner*.log`、`map.log`、`operator_grid/`、`lifecycle.log`、`capture_exit.txt` 等）；仓库内**不写**运行产物。
- 上游 / 下游：上游是人、`.agents/skills/dynanav-explore/` 的 SOP 与 `run_dynanav_native_exploration.sh`；下游是 `simulation/dynanav/experiment.py`、`simulation/dynanav/pct_scan_worker.main`、`localization_mapping/semantic_mapping`、`common/process_log`。
- 边界：不实现控制律、不修改源码、不写仓库内产物、不做验收。`run_dynanav_native_exploration.sh` 只负责进程编排与清理；导航与感知是否成功由 run 目录收据与离线工具判定。

## 当前文件夹结构

```text
scripts/
├── README.md                                 # 本文件
├── build_dynanav_fast_livo2.py               # 107 行，暂存并编译带构建期补丁的 FAST-LIVO2（含可选核心 CTest）
├── configure_data.py                         # 51 行，检查/创建 assets、models、runs 三个数据卷软链
├── dynanav_exploration.py                    # 61 行，探索工具统一入口（sop/collect/view/review/roi/candidates）
├── dynanav_exploration/
│   ├── README.md                             # 114 行，探索工具说明（历史版本，见逐文件表备注）
│   ├── review_generation.py                  # 375 行，只读同代覆盖审计与多视角渲染
│   └── review_roi.py                         # 44 行，显式 XY ROI 的 YZ/斜视检查
├── dynanav_observer_transport.py             # 60 行，observer UDP 接收配置选择（纯函数）
├── explore_waypoint_client.py                # 2302 行，算子侧交互航点/观察/占用图客户端
├── prepare_dynanav_waypoint_candidates.py    # 59 行，PCT 路线候选建议（只读、不下发动作）
├── query_scene_map.py                        # 76 行，查询已落盘地图快照的 CLI
├── reconstruct_dynanav_slam_map.py           # 602 行，用记录 LIO 位姿离线重建观测点层
├── render_dynanav_operator_grid.py           # 145 行，把导出的 operator grid NPZ 渲染成 PNG
├── run_dynanav_experiment.py                 # 7 行，experiment.main 的薄入口
├── run_dynanav_lio.sh                        # 684 行，隔离的 LIO/LIVO observer 启动器
├── run_dynanav_native_exploration.sh         # 305 行，D 路径唯一主入口（PCT+SCAN 探索）
├── run_dynanav_pct_scan.py                   # 10 行，pct_scan_worker.main 的薄入口（ROS system python）
├── run_with_bounded_log.py                   # 27 行，带冻结日志预算的子进程包装器
└── summarize_dynanav_experiment.py           # 414 行，汇总 run 目录收据与 journal
```

18 个 git 跟踪文件（14 个 Python 脚本、2 个 shell、2 个 README），14 个 Python 合计 4233 行、2 个 shell 合计 989 行（`wc -l`，2026-11-20）。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `README.md` | 本文件：入口清单、主链路、CLI、配置读取点与验证方式。 | — |
| `run_dynanav_native_exploration.sh` | D 路径主入口：解析参数与环境、加模拟器独占锁、冻结日志配置与 YAML、按顺序拉起观察器/SLAM 后端/采集/规划/占用图，并在退出时按「先停交互、再 TERM、最后 KILL」清理自有子进程。 | `[NEW_RUN_DIRECTORY] [--until-stopped] [--truth-pose] [--scan-prefix DIR] [--isaac-root DIR] [--pct-max-grid-cells N] [--overview-eye-base X Y Z] [--overview-focus-base X Y Z]`；见下文「主入口」与「配置读取点」 |
| `run_dynanav_lio.sh` | 在隔离 namespace 下启动 ONLY_LIO 或 LIVO observer，永不启动模拟器；`--check` 只做文件/hash/动态依赖 preflight，不建 run、不起 ROS；livo 另起 `parameter_blackboard` 并用 `--capture-result` 的安装标定校验相机参数。 | `--output-dir`、`--mode lio\|livo`、`--binary-prefix`、`--dependency-prefix`、`--parameter-override`、`--config-dir`、`--capture-result`、`--observer-namespace`、`--lidar-topic`、`--observer-transport auto\|baseline`、`--check`；产物 `preflight.json`、`launch.json`、`ready.json`、`shutdown.json`、`runner.log`、`ros_logs/`、`camera_parameter_ready.json` |
| `run_dynanav_experiment.py` | 纯委托：`from embodied_navigation.simulation.dynanav.experiment import main`，`raise SystemExit(main())`；参数全在被委托模块。 | `experiment.main`（`--output`、`--scene`、`--duration`/`--until-stopped`、`--point-cloud-source`、`--interactive`、`--interactive-pose-source`、`--interactive-controller` 等） |
| `run_dynanav_pct_scan.py` | 用 ROS system python 的薄入口：`sys.path` 插入 `<repo>/src` 后调用 `pct_scan_worker.main`；不启动模拟器。 | `pct_scan_worker.main`（`--run-dir`、`--pose-source`、`--cloud-topic`、`--lio-topic`、`--vio-topic`、`--build-scan`、`--build-only`、`--body-support-offset`、`--pct-start-blind-radius`、`--pct-max-grid-cells`） |
| `run_with_bounded_log.py` | 以冻结的日志预算运行子命令，并保留子进程原始退出码/信号语义；配置来自 `common.process_log.read_log_config`。 | `--config FILE --output-dir DIR -- COMMAND...` |
| `explore_waypoint_client.py` | 算子侧客户端：往 run mailbox 写请求、维持 heartbeat、读 observation 与 receipts、按实测话题构建粗粒度占用快照；不发速度命令、不 import Isaac、不读真值。 | 9 个子命令（见下节）；退出码 0 完成、1 拒绝/失败、2 超时、3 用法错误 |
| `dynanav_exploration.py` | 探索工具统一分发入口：`sop`、`collect`、`view` 转给 `.agents/skills/dynanav-explore/scripts/`，`review`/`roi` 转给本目录 `dynanav_exploration/`，`candidates` 转给 `prepare_dynanav_waypoint_candidates.py`；`sop` 自动补 `--repo`。 | `python3 scripts/dynanav_exploration.py COMMAND [ARGS]`；未知命令退出 2 |
| `dynanav_exploration/review_generation.py` | 只读、显式绑定同一代数据的覆盖审计与渲染：读 binding/snapshot，校验 run_id/cloud_stage/map_frame，绘制 XY/XZ/YZ/3D 视图与 height grid；输出必须是外部新目录，拒绝覆盖。 | `--binding FILE --snapshot NAME [--regions FILE] [--reference-snapshot] [--marker-from-postvio] [--cloud-dir DIR]` |
| `dynanav_exploration/review_roi.py` | 在显式 XY 裁剪内做 YZ/斜视检查，叠加 post-VIO 轨迹与位姿标记；只写 `<label>_yz_oblique.png` 与 `<label>_record.json`，不下发目标。 | `--binding FILE --snapshot NAME --xy X_MIN X_MAX Y_MIN Y_MAX --label LABEL` |
| `dynanav_exploration/README.md` | 探索工具的子目录说明（历史版本）：命令表仍列 `multicamera`、`shadow-audit` 等当前未在 `dynanav_exploration.py` 的 `COMMANDS` 中注册的子命令，阅读时以本文件与 `dynanav_exploration.py` 为准。 | — |
| `prepare_dynanav_waypoint_candidates.py` | 只读的 PCT 路线候选建议工具：结合 run 绑定、扫描快照/观测与显式 LiDAR 偏移，输出候选可行性建议；输出必须位于 action mailbox 之外。 | `--run-dir DIR`（必须）+ `--candidates JSON` 和/或 `--frontier-json`；`--lidar-offset DX DY DZ`（必须）；`--frontier-z`、`--prepare-next`、`--offline`、`--snapshot`、`--observation-json`、`--output`、`--voxel-m`、`--max-range-m`、`--route-spacing-m`、`--max-route-samples`、`--azimuth-samples`、`--body-support-offset`、`--pct-start-blind-radius`、`--pct-binary` |
| `query_scene_map.py` | 查询一份已落盘 `scene_map.json` 快照；过滤逻辑在 `localization_mapping.semantic_mapping.query`，本文件只做参数解析、快照读取与输出格式化。 | `MAP --label CLASS [--near X Y Z --radius R] [--include-expired] [--context]` |
| `reconstruct_dynanav_slam_map.py` | 用记录的关键帧与 SQLite bag 里的 LIO 估计离线重建观测点层，每个关键帧必须匹配同帧实测里程计；不读归档真值位姿。 | `--keyframes DIR --bag DIR\|.db3 --output NEW_DIR --observer-label LABEL [--observer-metadata FILE]... [--odom-topic] [--map-frame] [--sensor-frame] [--calibration-id] [--stamp-tolerance-ns] [--epoch] [--map-id] [--voxel-size] [--max-points] [--max-objects]` |
| `render_dynanav_operator_grid.py` | 把 `operator_grid_export/v1` 的 `.npz` + manifest 渲染成 PNG；occupied 黑、free 浅灰、未知灰、ROI 外白，未知格绝不画成 free；不连 ROS、不改地图。 | `render_dynanav_operator_grid.py <grid.npz> <manifest.json> <out.png>` |
| `summarize_dynanav_experiment.py` | 汇总 run 目录的 `result.json` 与 journal（速度、位移、建图、导航、执行器指标）；缺证据写 `null`，不把「进程完成」当「导航成功」，不覆盖已存在的输出文件。 | `RUN_DIRECTORY [--output FILE] [--low-count-threshold N]`；schema `dynanav-experiment-summary/v1` |
| `configure_data.py` | 检查或创建 `assets`、`models`、`runs` 三个数据卷软链；已存在真实内容或指向别处时报错不改。 | `--data-root DIR`（或 `PROJECT_DATA_ROOT`）、`--link`；输出 JSON，缺数据退出 2 |
| `build_dynanav_fast_livo2.py` | 调用 `localization_mapping.fast_livo2_source.stage_fast_livo2_source` 在外部目录暂存并编译带 `0011` 等构建期补丁的 `dynanav_sensor_msgs` / `fast_livo2_core` / `fast_livo2_ros`，生成 `capture_message_env.sh` 并支持 `--with-tests` 跑核心 CTest。 | `--build-root DIR --dependency-prefix DIR [--jobs N] [--with-tests]` |
| `dynanav_observer_transport.py` | 只读选择 observer 的有界 UDP 接收配置：仅在显式 LIVO/RTX + `rmw_fastrtps_cpp` + `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 且无既有 DDS 配置时启用 4 MiB 接收 profile，否则保留继承环境；不修改调用方环境。 | `select_transport(mode, policy, environment, repo, cwd, *, lidar_topic=None)`；由 `run_dynanav_lio.sh` 的 preflight 内联导入；profile `configs/simulation/dynanav/observer_udp_rx4m.xml` |

## 主入口：`run_dynanav_native_exploration.sh`

前置条件（缺一即退出）：`source /opt/ros/jazzy/setup.bash` 可用、仓库根有 `.env.local`（提供 `PROJECT_DATA_ROOT`）、Isaac 的 `python.sh` 可执行（默认 `$HOME/isaacsim`，用 `--isaac-root` 覆盖）、`--scan-prefix` 指向已构建的 SCAN install（默认 `$PROJECT_DATA_ROOT/embodied-navigation/build_cache/dynanav_scan_native/install`）、build prefix 提供 `dynanav_sensor_msgs`（默认 `.../build_cache/dynanav_complete_scan_*/`，用 `BUILD_PREFIX` 覆盖）。

命令形态：

```bash
scripts/run_dynanav_native_exploration.sh [NEW_RUN_DIRECTORY] [--until-stopped] [--truth-pose] \
  [--scan-prefix DIR] [--isaac-root DIR] [--pct-max-grid-cells N] \
  [--overview-eye-base X Y Z] [--overview-focus-base X Y Z]
```

固定行为：

- run 目录必须不存在；缺省为 `$PROJECT_DATA_ROOT/embodied-navigation/runs/native-exploration/<UTC 时间戳>`，并在 `/tmp/embodied-navigation-native-exploration-current` 写一份当前路径。
- `ROS_DOMAIN_ID` 缺省 219 且必须在 0..232（Fast DDS 端口映射上限）；固定 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`、`DYNANAV_RTX_COMPLETE_SCAN=1`。
- 独占锁 `/tmp/embodied-navigation-isaac-exclusive.lock`（`flock -n`）；若已有 kit / `fastlivo_mapping` 进程则不动它并退出 4。
- 冻结 `<run>/configuration/capture_log.json`；SLAM 模式另把 `configs/simulation/dynanav/fastlivo_livo.yaml`、`camera_rgbd.yaml`、`fastlivo_rtx_os1_32.yaml` 复制进 `<run>/configuration/`。
- 环境变量：`DURATION_SIM_S`（默认 300，必须在 8..1800；配合 `timeout` 墙钟预算 `max(900, 3×duration)`）、`SPEED_MPS`（1.0）、`NAVIGATION_BACKEND`（`pct_scan`，可设 `native_route`）、`CAPTURE_CALIBRATION`、`SIMULATION_SCENE`、`PCT_BODY_SUPPORT_OFFSET_M`、`PCT_START_BLIND_RADIUS_M`、`SPAWN_YAW_RAD`、`PRIOR_MAP`。
- 退出码：2 用法/前置条件、3 锁被占、4 已有模拟器/估计器、5 观察器提前退出、6 观察器启动超时、7 自有依赖在采集期间退出。

**默认模式（SLAM）**：先 `run_dynanav_pct_scan.py --build-scan --build-only` 构建 SCAN，再启动 `run_dynanav_lio.sh --mode livo`（等 `<run>/observer/ready.json`）与 `localization_mapping.backend.node`（`--spool <run>/observer/native_keyframes --namespace /dynanav/backend`，可选 `--prior-map`），最后用 `run_with_bounded_log.py` 包住 `run_dynanav_experiment.py`（`--point-cloud-source rtx_lidar --semantic-provider none --interactive --interactive-pose-source livo_post_vio --interactive-controller pct_scan --odometry-topic /dynanav/rtx_livo/post_vio_odometry --render-backend fabric`）。等 `<run>/capture/interactive` 出现后，再起 PCT+SCAN 规划进程与 `explore_waypoint_client.py map` 占用图进程。

**`--truth-pose` 模式**：跳过观察器与 SLAM 后端，采集改为 `--interactive-pose-source simulation_truth --odometry-topic /dynanav/truth/control_odometry`；规划进程加 `--pose-source simulation_truth --cloud-topic /dynanav/truth/cloud_deskewed --lio-topic /dynanav/truth/imu_odometry --vio-topic /dynanav/truth/control_odometry`；占用图也改读同一组真值话题。该模式不再复制 SLAM YAML，但仍需要 build prefix 提供 `dynanav_sensor_msgs`。真值链路的内容见 [capture README](../src/embodied_navigation/simulation/dynanav/capture/README.md) 的「200 Hz 真值去畸变接入」。

收尾：`trap cleanup EXIT` 先在 `<run>/capture/interactive` 存在时停掉交互（`explore_waypoint_client.py stop`，最多等 20 s），再对自有进程组发 TERM、等 10 s、必要时 KILL，并把采集退出码写 `<run>/capture_exit.txt`、时序写 `<run>/lifecycle.log`。

## `explore_waypoint_client.py` 子命令

| 子命令 | 作用 | 关键参数 |
| --- | --- | --- |
| `heartbeat` | 刷新算子存活心跳（`write_heartbeat`），`--loop` 时按 `--interval` 持续写 `--duration` 秒 | `--interval`（1.0）、`--duration`（600） |
| `goto` | 提交一个航点，本地跟踪器做 50 Hz 执行 | `--x --y --z`、`--reverse`（仅 `native_route`）、`--ttl`（30）、`--pose-source`（`lio_estimate`）、`--wait/--timeout` |
| `yaw` | 原地转向，需要独占的 SCAN 旋转空间回执 | `--delta-deg` 或 `--target-deg`（配 `--frame-id`）、`--ttl`、`--pose-source`、`--wait/--timeout` |
| `maneuver` | 短时有界本体系移动 | `--forward --lateral --yaw-rate --duration`、`--ttl`（15） |
| `scan` | 原地逐步转向并抓取 observation 快照 | `--steps`（4）、`--step-deg`（90）、`--timeout-per-step`（20）、`--settle-s`（1.0） |
| `cancel` / `stop` | 取消当前航点 / 停住并结束交互 | 仅公共选择器参数 |
| `observe` | 读 observation bundle 与最近回执 | `--receipts`（8）、`--full` |
| `map` | 订阅实测点云/里程计/时钟，累积算子侧占用快照并可导出 grid | `--cloud-topic`、`--odometry-topic`（必填）、`--clock-topic`、`--pose-source`、`--sensor-frame`、`--base-frame`、`--imu-mount`、`--extrinsic-translation`/`--extrinsic-quaternion`、`--free-mode`、`--resolution`、`--duration`、`--snapshot-interval`、`--frontier-limit`、`--export-grid`、`--cloud-semantics sensor_imu\|lio_deskewed_map`、`--binary-prefix` |

公共选择器 `--run-dir` / `--mailbox` / `--run-id` 可放在子命令前后；`--run-dir` 与 `--mailbox` 至少给一个。客户端只写 mail 目录（`requests/` 等），从不发 `cmd_vel`。

## 配置读取点

Python 脚本自身没有「启动时扫 configs 目录」的逻辑；真实读取点只有下面三处（其中 `dynanav_observer_transport.py` 由 `run_dynanav_lio.sh` 内联导入调用）：

| 读取方 | 读取什么 | 时机 |
| --- | --- | --- |
| [`run_dynanav_lio.sh`](run_dynanav_lio.sh) | `configs/simulation/dynanav/fastlivo_lio.yaml` 或 `fastlivo_livo.yaml`、`camera_rgbd.yaml`；`--lidar-topic /dynanav/lidar_rtx` 时再追加 `fastlivo_rtx_os1_32.yaml` | 拼 `--params-file` 并做模式/post-VIO 一致性 preflight；`--config-dir` 可指向 run 内冻结副本 |
| [`dynanav_observer_transport.py`](dynanav_observer_transport.py) | `configs/simulation/dynanav/observer_udp_rx4m.xml` | `run_dynanav_lio.sh` preflight 内联导入 `select_transport`，仅在 auto 且满足 FastRTPS/UDPv4/无既有 DDS 配置时选中 |
| `run_dynanav_experiment.py` 委托的 `simulation/dynanav/experiment.py` | `configs/simulation/dynanav_basement_collision.json`（运行期经 `session_collision.load_config` 读，取 `provenance` 并修复会话碰撞）、`configs/simulation/dynanav_basement_to_ground_collision.json`（只进 source snapshot） | 启动时 |

目录与配置语义见 [configs README](../configs/README.md)。

## 验证与测试入口

```bash
PYTHONPATH=src python3 -B -m compileall -q src scripts
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

| 脚本 | 直接覆盖它的测试 |
| --- | --- |
| `run_dynanav_lio.sh` | `tests/regression/test_dynanav_lio_launcher.py` |
| `run_dynanav_native_exploration.sh` | `tests/unit/test_pct_grid_budget.py`（经 `pct_budget_acceptance_fixtures.py` 以替身重放启动命令） |
| `run_dynanav_pct_scan.py` | `tests/unit/test_pct_grid_budget.py`、`tests/native/dynanav_pct_scan/ros_smoke.py` |
| `explore_waypoint_client.py` | `tests/unit/test_explore_waypoint_client.py`、`test_native_reverse_waypoint.py`、`test_operator_grid_export.py`、`test_truth_deskew_and_mode.py` |
| `render_dynanav_operator_grid.py` | `tests/unit/test_operator_grid_export.py` |
| `dynanav_exploration.py` / `dynanav_exploration/*.py` | `tests/unit/test_dynanav_exploration_tools.py`（含各子命令 `--help` 无动作检查） |
| `prepare_dynanav_waypoint_candidates.py` | `tests/unit/test_dynanav_waypoint_candidates.py` |
| `query_scene_map.py` | `tests/unit/test_dynanav_source_snapshot.py`、`test_layered_mapping.py` |
| `reconstruct_dynanav_slam_map.py` | `tests/unit/test_dynanav_slam_reconstruction.py` |
| `summarize_dynanav_experiment.py` | `tests/unit/test_dynanav_experiment_summary.py` |
| `dynanav_observer_transport.py` | `tests/unit/test_dynanav_observer_transport.py` |
| `configure_data.py`、`run_dynanav_experiment.py`、`run_with_bounded_log.py`、`dynanav_exploration/review_roi.py` | 无独立测试文件；`run_with_bounded_log.py` 经启动器集成用例间接覆盖 |

这些测试不启动 Isaac、GPU 或真实 ROS 图；真实仿真、原生 SCAN build 与 ROS 话题时序仍需在 Linux 主机执行主入口验证。

## 相关入口

- 主链路说明与模块分层：[DynaNav README](../src/embodied_navigation/simulation/dynanav/README.md)
- 采集实现与 200 Hz 真值去畸变：[capture README](../src/embodied_navigation/simulation/dynanav/capture/README.md)
- 配置读取语义：[configs README](../configs/README.md)
- Agent 探索 skill：`.agents/skills/dynanav-explore/`
