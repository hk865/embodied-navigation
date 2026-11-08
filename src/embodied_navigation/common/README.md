# common：跨模块公共基础

`common` 提供导航各模块共享的配置入口、状态来源与时间契约、后台 I/O、路径和进程关停语义，自身不实现业务状态机、控制律或 ROS 节点。

## 边界

- `observation.py` 是正常运行时默认关闭的诊断观测空端口：不创建线程、定时器或文件，历史 observer 不在 `src` 内；该空端口不记录诊断数据。
- ROS 相关导入都放在函数内，本目录可被纯 CPU 测试直接导入。
- 没有实机安全后端；`state_contract.py` 中 `REAL_SENSOR` 只是保留枚举。

## 关键文件与接口

- `config.py`：`read()` 合并 [configs/launch/navigation.json](../../../configs/launch/navigation.json) 列出的模块配置并展开 `${PROJECT_ROOT}`；`path(name)` 解析 `runtime_paths`，`actor_python=auto` 时读 `NAVIGATION_ACTOR_PYTHON`。
- `state_contract.py`：`StateProvenance`、`StateOrigin`、`StatePurpose` 与 `validate_state_provenance()`。该契约的 `StatePurpose.NAVIGATION` 位姿用途只接受 `SLAM_ESTIMATE`；仿真真值可用于 `SIMULATION_SAFETY` 和 `EVALUATION`，不能重标为导航定位；`PhysicalSafetyState` 是物理安全事实，不是位姿提供者。
- `runtime_io.py`：`EvidenceWriter`、`LatestCommandWriter`（一条在途 + 可替换 pending，不排队）、`BackgroundCheck`、`bounded_shutdown_signals`；传感器回调不等待磁盘落盘。
- `integration_limits.py`：冻结常量，按量纲与时间基准命名（3 s 匹配/拼接老化、300 ms 命令与 bundle 新鲜度、0.35 m 射线 p95、LiDAR 机体平移等），只被消费，不是新配置项。
- `input_freshness.py`：正常源新鲜度适配器，`enabled(profile)` 拒绝把诊断放宽带进正常导航。
- `shutdown.py`：`ShutdownSignals` 先通过信号请求退出，让调用方在销毁 ROS context 前排空回调；`ShutdownRequested` 只由本进程信号触发。
- 其余：`runtime_context.py`（run 配置读写）、`runtime_environment.py`（source 公共 ROS 前缀）、`paths.py`（工程内规范路径）、`runtime_telemetry.py`（当前关闭历史归档，保留 live status 的同步原子文件写入）。
- 通信与时钟子目录见 [communication/README.md](communication/README.md) 与 [time/README.md](time/README.md)。

## 调用与依赖

被 `launch` 与多数模块导入；`runtime_io`、`integration_limits`、`input_freshness` 出现在热路径，`state_contract` 由 [simulation/safety_state.py](../simulation/safety_state.py) 等产出方与门禁共同消费。

## 测试与限制

[tests/unit/test_simulation_safety.py](../../../tests/unit/test_simulation_safety.py) 覆盖 `state_contract`；[tests/unit/test_ros_shutdown_lifecycle.py](../../../tests/unit/test_ros_shutdown_lifecycle.py) 覆盖 `shutdown`；[tests/regression/test_navigation_safety.py](../../../tests/regression/test_navigation_safety.py) 使用 `config.read`。本目录无独立功能测试文件；CPU 入口为 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v`。来源契约是程序内数据契约而非密码学认证；`install/` 缺失只代表本机未构建，不代表缺源码。
