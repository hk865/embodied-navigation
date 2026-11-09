# launch：装配与进程生命周期

`launch` 是普通导航运行的系统入口与装配层：`start.py` 调用 `main.py`，完成配置准备、环境构造、`ros2 launch` 子进程拉起、运行监控与有序关停，业务任务状态不在这里实现。

## 边界

- 只做模块装配、环境与进程管理；不写控制律、停车判断、任务状态机或安全阈值。
- `--check` 只是文件级依赖检查，通过不代表 ROS/仿真可运行。
- `building.py` 是独立的 Building SLAM 生产入口，不是普通导航主入口的一部分。
- 不生成业务状态证据：它写 run 目录的"启动/退出/配置快照"，业务状态由各模块自己持有。

## 目录结构

```text
src/embodied_navigation/launch/
├── launch/  # 子模块，见 launch/README.md
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），空包标记
├── building.py  # 603 行，Building SLAM 独立入口（prepare/run/--check）
├── exit_status.py  # 13 行，父进程与子组件真实退出汇总
├── launching.py  # 34 行，共享 launch 机制（python/owned）
├── main.py  # 353 行，普通导航生命周期（check/lock/prepare/launch/monitor/cleanup）
├── preflight.py  # 47 行，文件级依赖检查
└── prepare.py  # 215 行，从 configs/assets 生成 run 配置
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | 7 个 `.py` 共 1266 行（最大 `building.py` 603 行） | 全部是活代码，无兼容门面/生成物。`main.py` 是普通导航入口（由 [start.py](../../../start.py) 调用）；`building.py` 是独立生产入口；`prepare/preflight/launching/exit_status` 是库；`__init__.py` 空包标记。 |
| 日志/输出 | `runs/<UTCstamp>_<label>_<4hex>/`（`launch/main.py` 生成） | 每个子进程一个 `<role>.log`（`open("x")`）；`processes.json` 记录 `role/pid/start_ticks`；`component_exit.jsonl` 由 [launching.py](launching.py) 的 `owned()` 在组件退出时追加；`exit.json` 在 cleanup 时写 `error`/`process_exit_codes`/`component_exits`。`prepare()` 另写：`world.sdf`、`asset_manifest.json`、`sensor_contract.json`、`sensor_sampling_contract.json`、`world_before_sensor_override.sdf`、`navigation_fastlivo.yaml`、`navigation_camera.yaml`、`navigation_profile.json`、`navigation_scenario.json`、`terrain_target_manifest.json`、`known_scene_reference.json`、`v34_scan_overrides.yaml`、`global_route.json`、`runtime_context.json`。`building.py` 的 run 目录另写 `PCT_route.json`、`scan_params.json`、`experiment_setup.json`、`building_fastlivo.json`、`building_camera.json`、`transport_settings.json`、`experiment_exit.json`，并有 `ros_logs/`、`actuator_physics.jsonl`、`backend_spool/`、`backend_history/`、`backend_result.json`。启动/停止阶段还向 stdout 打一行 JSON（`stage=prepared/running/stopped`）。 |
| 配置 | `common.config.read()` + `runtime_paths` | 读 [configs/launch/navigation.json](../../../configs/launch/navigation.json) 列出的模块配置；`preflight.check()` 共查 13 个条目：6 个来自 `path()`（`student_model`、`actor_python`、`actuator`，以及 `ros_prefix` 下的 ROS install/SLAM/SCAN），其余 7 个是固定工程相对路径（Go2 URDF/SDF、world、route、L1 mesh、`writer_qos.so`、patched DDS）。`prepare()` 读 `configs/localization_mapping/fastlivo.yaml`、`camera.yaml`，把 profile 中 `daily_navigation`、`experiment=navigation`、`r85_auto_reference_contract.reference_file`、`engineering_recording.raw_source_capture`/`registered_cloud_archive` 强制 pin。`environment()` 固定 `FASTRTPS_DEFAULT_PROFILES_FILE`、`ROS_DOMAIN_ID`、`GZ_PARTITION`、`DEMO_RUN_DIR`、`ROS_LOG_DIR`，并把 7 个探针开关强制为 `"0"`。`building.py` 读 `configs/scenarios/building/student_profile.json`、`configs/scenarios/building/scan.json` 与 `configs/localization_mapping/camera.yaml`。权威读取顺序见 [configs/README.md](../../../configs/README.md)。 |

## 模块内文件耦合

依赖方向是"入口 → 库"的单向树，无环：`main.py → {prepare.py, preflight.py, exit_status.py}`，`prepare.py → {common.config, common.runtime_context, localization_mapping.known_scene_reference_builder, perception.sensors.sampling, planning.scan_runtime_parameters}` 并以子进程调用 `simulation/world/prepare.py`；`launching.py` 被 4 个模块的 `launch/components.py` 反向复用；`exit_status.py` 只被 `main.py` 使用；`building.py` 与其它 5 个 Python 文件**零 import 关系**，是并列的第二入口。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `main.py` | 353 | `common.config`/`runtime_environment`/`communication.dds_environment`、`prepare`、`preflight`、`exit_status`、`subprocess` | `start.py` | 偏重（`main()` 218 行，中枢） |
| `prepare.py` | 215 | `common.config`/`runtime_context`、`localization_mapping`、`perception.sensors.sampling`、`planning` + 子进程 | `main.py` | 偏重（`prepare()` 172 行、跨 4 个模块） |
| `building.py` | 603 | `common.config`/`runtime_environment`/`communication`、`subprocess` | 无（`-m` 独立入口） | 偏重（`run()` 292 行；与 `main.py` 重复进程监督逻辑） |
| `preflight.py` | 47 | `common.config` | `main.py` | 清楚（只读文件检查） |
| `launching.py` | 34 | `launch` 库 | `control/localization_mapping/planning/simulation` 的 `launch/components.py` | 清楚（共享机制） |
| `exit_status.py` | 13 | 无 | `main.py` | 清楚（纯函数） |
| `__init__.py` | 0 | 无 | 无 | 清楚 |

四类问题核查：

- 上帝文件：没有 >800 行文件（`building.py` 603 行），但有 3 个 >80 行函数：`main.main()` 218 行、`building.run()` 292 行、`prepare.prepare()` 172 行。
- 重复实现：有且明显。`main.py` 与 `building.py` 各自实现一遍"起子进程组 → 记录 `/proc` start ticks → 发 SIGINT → 15 s 等待 → SIGTERM 组 → 汇总退出"；`prepare()` 与 `building.prepare()` 也各自生成 run 文件。两处细节已经分叉（`main` 用 `bounded_shutdown` 式信号标志 + `exit.json`，`building` 用 `experiment_exit.json`）。
- 跨层直达：有，但属于装配层职责。`prepare()` 直接 import `localization_mapping`/`planning` 的构建器并 `subprocess.run` 世界生成脚本；`main.environment()` 直接 pin FastRTPS profile 文件路径与探针开关（绕过 `configs` 层）。
- 隐式契约：有。`/tmp` 下 3 个固定锁文件名（`go2_pid_physics_serial.lock` 等）、`building` 的 `/tmp/building_navigation.lock`、run 目录内 `runtime_context.json`/`sensor_contract.json`/`component_exit.jsonl` 的文件名与字段、`--domain` 1..232 与 `--duration` 有限正数、`-m` 模块名（`embodied_navigation.*`）都是跨进程契约。

结论：作为装配层，耦合面大但方向清楚（入口依赖库、库之间不互调）；真正的问题是 `main.py` 与 `building.py` 的**监督循环重复实现**，以及 `prepare()` 把 4 个模块的构建逻辑串在一个 172 行函数里。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `exit_status.py` | 良好 | 13 行，1 函数 10 行，docstring 0.0 但函数名与 4 行实现自解释（父进程零退出不能掩盖子组件失败） |
| `preflight.py` | 良好 | 1 函数 42 行，docstring 1.0，明确"只检查、不启动"与 `is_file` 语义 |
| `launching.py` | 可接受 | 3 函数，docstring 0.0，最长 18 行；`owned()` 用 `sigterm_timeout="8"/sigkill_timeout="5"` |
| `main.py` | 需改进 | 6 函数，docstring 1.0（每个函数都有），但 `main()` 218 行（136–353）同时做锁/prepare/启动/监控/cleanup，32 注释行 |
| `prepare.py` | 需改进 | 3 函数，docstring 0.667，`prepare()` 172 行（44–215）串起世界生成、SLAM 外参、SCAN、路线、reference 与 context |
| `building.py` | 需改进 | 8 函数，docstring 0.5，`run()` 292 行（202–493）；`prepare()` 125 行、`main()` 104 行 |

区分两类问题：**结构问题**是 3 个超长函数与两处重复监督逻辑；**文档问题**较轻——`main.py`/`preflight.py` 的 docstring 反而最完整，`launching.py`/`exit_status.py` 缺函数 docstring 但函数很短。静默行为有一处刻意设计：cleanup 中 `except (OSError, ProcessLookupError)` 忽略信号错误，随后由退出码汇总重新暴露。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `main.py` 与 `building.py` 重复的"子进程组启动 + `/proc` start ticks 校验 + 信号 + 超时 + 退出汇总"抽到 `launching.py`（或新 `supervision.py`） | 同一生命周期逻辑两套实现已分叉（`exit.json` vs `experiment_exit.json`），修一处必须记得另一处 | 中：进程/信号语义，改错会留下僵尸进程 | 先跑 `tests/unit/test_launch_exit_status.py`、`tests/unit/test_launch_cli.py`、`tests/unit/test_building_entry_closure.py`，再加一个 fake `Popen` 的监督循环测试 |
| P1 | 拆 `main.main()`（218 行）为 `acquire_locks` / `prepare_run` / `launch_tree` / `monitor` / `cleanup` 五个阶段函数 | 单函数承担 5 个阶段，异常路径与 finally 交叉 | 中：cleanup 顺序敏感 | 保持 `exit.json` 字段与退出码不变；用 `--check`/`--prepare-only` 与现有 CLI 测试对照 |
| P1 | 拆 `building.run()`（292 行） | 同型问题，且 Building 是生产入口 | 中 | `tests/unit/test_building_entry_closure.py` 覆盖 prepare/CLI，run 仍需人工对照 |
| P2 | 拆 `prepare.prepare()`（172 行）为 `generate_world` / `write_slam_extrinsics` / `write_scan_params` / `write_route` / `write_context`，每步只接收目录与 profile | 便于单测与定位"哪个产物生成失败" | 中：产物顺序与 pin 报告 | `tests/unit/test_launch_prepare_pins.py` 覆盖 `_pin`；拆分后逐产物比对文件名/内容 |
| P2 | 补 `launching.py`/`exit_status.py` 的函数 docstring | docstring 0.0，虽短但 `owned()` 的 timeout 语义重要 | 低 | 纯文档 |

本轮只写建议、不改代码。`launch/` 是启动路径，任何重构都必须"先有可对照的 run 产物/退出码"，不能只靠单元测试。

## 关键文件与接口（原"关键文件与接口"）

- `main.py`：`parse_arguments()` 在读取配置和建目录前校验参数（`--domain` 限 1..232，`--duration` 必须是有限正数）。`--check` 输出 JSON 并返回 0/2；`--prepare-only` 只生成 run 文件；普通启动会 `prepare()`、构造环境、启动 `ros2 launch demo.launch.py run_dir:=...`、监控子进程，并在退出时写 `exit.json`。`--resource-check` 是显式 CPU/内存预检。
- `prepare.py`：从 `configs` 与 `assets` 生成 world、sensor、SLAM/SCAN 配置、route 与 runtime context；会调用 [perception/sensors/sampling.py](../perception/sensors/sampling.py) 的 `override_sensors`。
- `preflight.py`：只读检查 Student 模型、actor Python、Go2 URDF/SDF、world、route、L1 mesh、actuator、`writer_qos.so`、patched DDS、ROS install、SLAM、SCAN 等文件是否存在。
- `launching.py`：`python()` 生成 `-m` 启动命令；`owned()` 创建带 SIGTERM/SIGKILL 超时的 `ExecuteProcess`，子组件退出写 `component_exit.jsonl` 并触发 Shutdown。
- `exit_status.py`：`failed_exits()` 汇总父进程与子组件的真实非零退出。
- `building.py`：Building 场景入口，只接受 `--mode slam`，在读取资产前拒绝 truth/capture 模式，自带 `prepare`/`run`/`--check`。
- 子目录 [launch/README.md](launch/README.md) 是 ROS2 launch 文件目录；普通主流程经 [start.py](../../../start.py) 进入。

## 调用与依赖

依赖 [common/config.py](../common/config.py)、[common/runtime_environment.py](../common/runtime_environment.py)、[common/communication/dds_environment.py](../common/communication/dds_environment.py) 与 `simulation/world/prepare.py`；各模块 `launch/components.py` 复用 `launching.py`。

## 测试与限制

[tests/unit/test_launch_cli.py](../../../tests/unit/test_launch_cli.py) 覆盖参数校验与 `--check` 分支，[tests/unit/test_launch_exit_status.py](../../../tests/unit/test_launch_exit_status.py) 覆盖退出汇总，[tests/unit/test_launch_prepare_pins.py](../../../tests/unit/test_launch_prepare_pins.py) 覆盖 prepare 的 pin 报告，[tests/unit/test_building_entry_closure.py](../../../tests/unit/test_building_entry_closure.py) 覆盖 Building prepare/CLI。`--check` 不验证 ROS 域占用、资产内容或系统可运行性；当前 `install/` 缺构建产物只说明本机未构建。本会话未实际启动 ROS/仿真，`main.py` 的监控/关停路径只经单测与静态阅读核对。
