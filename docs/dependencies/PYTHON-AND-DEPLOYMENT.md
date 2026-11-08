# 项目自有 Python 与部署材料

以下是迁移前模块路径，用于核对实际依赖闭包。迁移后的 `embodied_navigation` 命名空间由主工程维护，不需要其它本地定制工程加入 `PYTHONPATH`。

## Isaac Go2 最小 Python 闭包

定位后端共 34,710 字节：

- `localization/__init__.py`
- `localization/backend/__init__.py`
- `localization/backend/external_map.py`
- `localization/backend/tracking.py`
- `localization/backend/geometry.py`
- `localization/backend/registration.py`

部署控制共 61,910 字节：

- `control/__init__.py`
- `control/policy/__init__.py`
- `control/policy/student.py`
- `control/policy/speed_tiers.py`
- `control/policy/native/__init__.py`
- `control/policy/native/observation.py`
- `control/policy/native/contract.json`
- `common/__init__.py`
- `common/input_freshness.py`

`observation.py` 导入时读取相邻 `contract.json`，因此 JSON 不是可忽略的历史记录。恢复的 Isaac 脚本使用 `exact_student45`、`CorrectionOnlyPID`、`PID_SPEC` 和 `quaternion_rotation`，不会执行训练。`TerrainHeightMap` 才会延迟导入 `simulation.mesh_terrain`；完整 Gazebo/替代策略流程还需要各自的调用闭包，不能只沿用以上最小清单。

系统 Python 依赖包括 NumPy、SciPy、PyYAML、Pillow、Torch 和 ROS Jazzy 的 rclpy、sensor_msgs_py、nav_msgs、geometry_msgs、sensor_msgs、std_msgs、rosgraph_msgs。`scan_planner_msgs` 必须从本工程 ROS 包生成，不能把旧安装目录当作源码。

## 完整 Building 流程的其它自有 C++

这些是主工程功能模块，不属于第三方库；应保留实际源文件和 CMake：

- `simulation/actuators/{CMakeLists.txt,teacher_actuator.cpp}`：22,055 字节；依赖 `gz-sim8` 和 `gz-plugin2::register`，生成 `install/actuators/lib/libteacher_actuator.so`。
- `localization/backend/native/{CMakeLists.txt,graph_solver.cpp}`：6,082 字节；依赖 Eigen3，生成 `install/backend/lib/liblivo_graph.so`。
- `common/communication/{CMakeLists.txt,writer_qos.cpp}`：依赖 `rcl`、`rmw_fastrtps_shared_cpp` 和本工程 FastDDS，生成 `writer_qos.so`。

命名空间/功能目录迁移必须同时修复 `common.config.ROOT/CONFIG`、`common.paths`、`native_graph.LIBRARY` 等基于 `parents[n]` 的路径，以及 JSON 内原始路径。

## 数据与部署模型

- Student 部署 JIT：`student_model_4999_deterministic_jit.pt`，1,342,573 字节。部署需要真实模型；RL 训练代码、训练日志、原始 checkpoint 不属于导航源码依赖。保留原 `MODEL_NOTICE.md`，其授权边界见许可说明。
- Go2 部署 USD：原 `assets/usd/go2_training_layered/go2_description/` 共 9 个文件、5,824,674 字节，所有文本层引用都在该目录内。应作为部署资产独立提供或按已授权数据清单取得，不依赖整个 RL 工程。无需复制重复的 `go2_description_1`。二进制 USD 内的外部资产引用尚未用 USD 组合器进一步验证。
- DynaNav `office.usd` 及其场景依赖、碰撞几何、`points_with_plane.npy`、PCT 网格/路线是具体运行需要的数据，不应误删为日志。大数据应通过明确的数据包位置/获取清单供运行入口使用，而不是硬编码本机挂载目录。
- 可选视觉模型脚本仍可能依赖 `embodied-agent/workspace/vla_baselines` 的模型环境。选择这类入口时必须改为明确公开依赖+独立模型清单，或者明确标记该可选模式未恢复，不能称主工程已独立支持它。

原 `guarded_scene_probe.py` 会把脚本复制到运行目录执行。新包路径不应由复制后脚本的 `parents[2]` 推导；应使用已安装包或显式本工程 `PYTHONPATH`，否则干净环境中仍会找不到路径模块。
