# simulation/gazebo

本目录是 Gazebo 仿真部署的归置目录：当前只有 `robogauge` 一个子目录，本层没有独立可执行文件。

- [robogauge/README.md](robogauge/README.md) 承载 RoboGauge MoE-CTS 的 Gazebo 部署。
- 通用 Teacher/Student Gazebo 链不在这里：世界生成在 [world](../world/README.md)，进程装配在 [launch](../launch/README.md)，关节力矩插件在 [actuators](../actuators/README.md)。
- 上层模块说明见 [simulation/README.md](../README.md)。

本目录无专用测试，也未在本次核对中启动 Gazebo。`robogauge` 之外若还要放别的 Gazebo 部署，按同样约定新增子目录并配自己的 README，不要在本层写进程逻辑。
