# launch/launch：ROS2 launch 文件目录

本目录不是独立执行入口，只存放被 `launch/main.py` 调用的 ROS2 launch 描述文件。

- `demo.launch.py`：`generate_launch_description()` 声明 `run_dir` 参数，并依次 include 四个模块的 launch 文件：[slam.launch.py](../../localization_mapping/launch/slam.launch.py)、[scan.launch.py](../../planning/launch/scan.launch.py)、[control.launch.py](../../control/launch/control.launch.py)、[simulation.launch.py](../../simulation/launch/simulation.launch.py)。
- 各子 launch 文件本身很薄，真正的进程列表在其模块的 `launch/components.py` 中。
- `__init__.py` 只是包标记；本目录没有 `main` 或命令行入口。

使用方式是 `launch/main.py` 以 `ros2 launch .../demo.launch.py run_dir:=<run目录>` 启动；它负责准备 run 文件、构造环境并监控退出。包代码与使用说明见 [../README.md](../README.md)。本目录无专用测试。
