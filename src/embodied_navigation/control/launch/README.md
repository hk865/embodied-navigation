# control/launch

本目录只是控制进程组的 ROS launch 装配，不含任何控制算法。

`control.launch.py` 只声明 `run_dir` 参数，再交给 `components.py` 启动三个受管进程：

- `controller`：`embodied_navigation.navigation.pid_navigation --run <run_dir>`
- `command_bridge`：`embodied_navigation.robot.unitree.bridge`，读 `runtime_context.json`，把 `/demo/cmd_vel` 写成 `navigation_command.json`
- `mission`：`embodied_navigation.navigation.mission.mission_runtime --run <run_dir>`

`components.py` 使用 `launch.launching.python` 与 `owned`，进程生命周期和退出汇总由 `../../launch/main.py` 统一管理。该组被 `../../launch/launch/demo.launch.py` 以 `('control','control')` 引入，与 localization、planning、simulation 三个同构组并列；`run_dir` 由 prepare 阶段生成，所以本目录没有可单独执行的入口。

真实实现在 `../tracking/`、`../parking/`、`../safety/`（见 `../README.md`）与 `../../navigation/`；控制组自身不实现任务状态机。

## 测试

本目录无专用测试；通用启动 CLI 由 `../../../../tests/unit/test_launch_cli.py` 覆盖，不验证本组三个组件实际可启动。
