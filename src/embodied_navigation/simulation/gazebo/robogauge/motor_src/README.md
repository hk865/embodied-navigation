# simulation/gazebo/robogauge/motor_src

本目录是 RoboGauge 专用的 Go2HV 关节执行器原生源码，构建为 Gazebo 系统插件后由 benchmark 的世界文件加载。

- [go2hv_actuator.cpp](go2hv_actuator.cpp)：与 `simulation/actuators/teacher_actuator.cpp` 结构相同，但力矩包络换成 Go2HV：加速 20.2 N·m、制动 23.4 N·m，13.5 rad/s 以上线性降到 30 rad/s 归零；仍保留 `teacher_sim::TeacherActuator` 插件 ABI 名，以及 `TEACHER_SOCKET` 和 `TEACHER_ACTUATOR_LOG` 环境键（历史 wire 兼容）；插件要求 socket 路径非空、日志路径可打开。
- [CMakeLists.txt](CMakeLists.txt)：依赖 `gz-sim8`、`gz-plugin2`，产出 `robogauge_actuator` 共享库。
- 构建入口是仓库根的 `docker/build_owned_native.py` 的 `robogauge` 模块，安装到 `install/robogauge/lib/`；使用方是 [../world.py](../world.py) 的 `actuator_path()`，背景见 `docs/robogauge/README.md`。

无 C++ 单测；[tests/regression/test_robogauge_benchmark.py](../../../../../../tests/regression/test_robogauge_benchmark.py) 只断言准备出的世界引用了正确 `.so` 路径，不加载或运行它。`install/` 缺该产物只表示本机未构建，源码完整。
