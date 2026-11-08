# simulation/actuators

本目录是 Gazebo 仿真中唯一的关节力矩写入插件（Teacher/Student 链），以源码形式随仓库提供，构建后由世界文件加载。

- [teacher_actuator.cpp](teacher_actuator.cpp)：实现 `gz::sim::System` 的 Configure/PreUpdate，插件名为 `teacher_sim::TeacherActuator`；默认 `kp=25`、`kd=0.5`、力矩上限 23.5、速度上限 30，每 4 步取一次目标，并通过 Unix socket 与策略进程交换 request64/response16。注释明确它不写 base wrench，也不在启动后重置位姿或速度。
- [CMakeLists.txt](CMakeLists.txt)：依赖 `gz-sim8` 与 `gz-plugin2`，构建 `teacher_actuator` 共享库并安装到 `lib/`；`docker/build_owned_native.py` 的 `actuators` 模块把它装到 `install/actuators/lib/libteacher_actuator.so`。
- 使用方：[../world/prepare.py](../world/prepare.py) 在生成的 `world.sdf` 中写入插件元素 `libteacher_actuator.so`，`launch/prepare.py` 再按配置改写成 install 下的绝对路径；`configs/launch/settings.json` 的 `runtime_paths.actuator` 指向同一 `.so`；仿真 actor 进程通过 `TEACHER_SOCKET` 与它通信。插件还要求非空且可打开的 `TEACHER_ACTUATOR_LOG`；普通启动由 `launch/main.py` 同时注入两个变量，并将日志路径设为 `/dev/null`。
- 原生源码只在本仓库编译，不从其他工程链接；`install/` 缺产物只表示本机未构建。

测试：C++ 侧无专用单测；socket 协议、目标保持与安全门禁调用由 [tests/regression/test_native_safety_integration.py](../../../../tests/regression/test_native_safety_integration.py) 用假 socket 间接覆盖（不加载本 `.so`）。
