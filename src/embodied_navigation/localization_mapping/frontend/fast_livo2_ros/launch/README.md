# localization_mapping/frontend/fast_livo2_ros/launch

本目录只保留上游 FAST-LIVO2 ROS 包的示例 launch 副本，不是本工程的启动入口。

- [mapping_aviz.launch.py](mapping_aviz.launch.py) 是上游 bag/AViz 演示：从 `fast_livo2_ros` 包的 share 目录取 `config/avia.yaml`、`config/camera_pinhole.yaml` 和 rviz 配置，启动 `fastlivo_mapping`、图像解压与可选 rviz。本仓库这一层没有对应的 `package.xml` 和 CMake 安装，因此不能当成可直接 `ros2 launch` 的包。
- 本工程实际启动 SLAM 的位置是 [localization_mapping/launch/slam.launch.py](../../../launch/slam.launch.py) 及其 `components.py`，它们直接使用 `install/ros` 下的 `fastlivo_mapping` 和运行目录内生成的两个 yaml。
- 上游同名文件见 [third_party/fast_livo2/fast_livo2_ros/launch/mapping_aviz.launch.py](../../../../../../third_party/fast_livo2/fast_livo2_ros/launch/mapping_aviz.launch.py)，内容与本副本一致。

无专用测试；不要把示例 launch 当作导航链路已验证的启动方式。
