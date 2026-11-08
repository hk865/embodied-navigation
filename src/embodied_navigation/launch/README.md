# launch：装配与进程生命周期

`launch` 是普通导航运行的系统入口与装配层：`start.py` 调用 `main.py`，完成配置准备、环境构造、`ros2 launch` 子进程拉起、运行监控与有序关停，业务任务状态不在这里实现。

## 边界

- 只做模块装配、环境与进程管理；不写控制律、停车判断、任务状态机或安全阈值。
- `--check` 只是文件级依赖检查，通过不代表 ROS/仿真可运行。
- `building.py` 是独立的 Building SLAM 生产入口，不是普通导航主入口的一部分。

## 关键文件与接口

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

`tests/unit/test_launch_cli.py` 覆盖参数校验与 `--check` 分支，`tests/unit/test_launch_exit_status.py` 覆盖退出汇总，`tests/unit/test_building_entry_closure.py` 覆盖 Building prepare/CLI。`--check` 不验证 ROS 域占用、资产内容或系统可运行性；当前 `install/` 缺构建产物只说明本机未构建。
