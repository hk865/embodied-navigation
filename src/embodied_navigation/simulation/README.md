# simulation

本目录是仿真与场景生命周期层：生成 Gazebo/Isaac 场景、启动物理与传感器、维护仿真安全状态与真值流，并承载 RoboGauge/DynaNav 两类仿真部署；它不实现导航算法。

## 边界

- 仿真真值日志和 ground truth 路径用于离线评测；native64 帧还用于现有特权步态观测、COM 速度内环与仿真安全门禁，不作为 SLAM/导航定位输入。[safety_state.py](safety_state.py) 产出的安全状态明确标为 `simulation_truth` + `simulation_safety`。
- 不承载整个导航系统；`dynanav` 只做 DynaNav 场景接入与真实 Go2 RGB-D capture。
- 未验证 Isaac/实机/完整路线可运行，不能把仿真准备成功当作整栈完成。

## 组成

- [world/README.md](world/README.md)：从仓库资产生成隔离的 SDF、传感器契约与桥配置。
- [actuators/README.md](actuators/README.md)：当前 Gazebo Student/native 部署链的唯一关节力矩写入插件（保留 Teacher 历史命名）；RoboGauge 部署链另有专用插件。
- [gazebo/README.md](gazebo/README.md)：Gazebo 仿真部署，其中 `robogauge/` 是真实 SLAM 的 RoboGauge benchmark。
- [launch/README.md](launch/README.md)：仿真进程装配。
- [terrain/README.md](terrain/README.md)：受控 Actor 地形切换。
- [dynanav/README.md](dynanav/README.md)：Isaac/DynaNav RGB-D capture。
- 顶层：[truth_worker.py](truth_worker.py)（Student 仿真适配，仅 SLAM 模式）、[safety_state.py](safety_state.py)（native64 到仿真安全状态）、[mesh_terrain.py](mesh_terrain.py)（Building 网格垂直射线）、[wait_actor.py](wait_actor.py)（actor 就绪屏障）。

## 测试与限制

[safety_state.py](safety_state.py) 有 [tests/unit/test_simulation_safety.py](../../../tests/unit/test_simulation_safety.py) 与 [tests/regression/test_native_safety_integration.py](../../../tests/regression/test_native_safety_integration.py) 覆盖；其余仿真启动没有端到端测试。本次核对没有启动 Gazebo、Isaac 或 ROS。
