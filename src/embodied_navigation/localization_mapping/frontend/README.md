# localization_mapping/frontend

本目录归置 FAST-LIVO2 的 ROS 接入资料；它本身不是可独立运行的算法包，真正的 FAST-LIVO2 实现在 `third_party/fast_livo2`。

- 目前只有 [fast_livo2_ros/README.md](fast_livo2_ros/README.md) 一个子目录，内容是上游 ROS 包的部分文件副本，不是在本仓库重新编译的包。
- 生产链路里的前端适配节点在上一层：[../odom_adapter.py](../odom_adapter.py) 负责把 FAST-LIVO2 的 IMU 状态换算成 body/lidar 里程计，由 [../launch/components.py](../launch/components.py) 启动；SLAM 可执行文件 `fastlivo_mapping` 从 `install/ros` 加载。
- 与上游的关系见 [third_party/fast_livo2/fast_livo2_ros/README.md](../../../../third_party/fast_livo2/fast_livo2_ros/README.md)。
- 本目录没有 `package.xml`、`CMakeLists.txt` 或可导入的业务代码；仓库内也没有模块导入 `embodied_navigation.localization_mapping.frontend`。保留副本的目的只是让接手者在源码树里看到接入点对应的上游文件。

本目录无专用测试。不要把这里的 `__init__.py` 或 launch 副本当成算法本体，也不要把副本路径当作 ROS 包安装路径。
