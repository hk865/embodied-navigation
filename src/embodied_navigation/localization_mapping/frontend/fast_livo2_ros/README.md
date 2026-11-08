# localization_mapping/frontend/fast_livo2_ros

本目录是 `third_party/fast_livo2` 中真实 FAST-LIVO2 ROS 包的接入/包装副本，只保留 `launch/` 与 `scripts/` 两个参考文件，不是算法本体，也不是本工程编译出的 ROS 包。

- [launch/mapping_aviz.launch.py](launch/mapping_aviz.launch.py) 与 [scripts/decompress_images.py](scripts/decompress_images.py) 与上游同名文件逐字节相同（核对于当前 checkout），用于理解上游的 bag 回放接口；本工程导航不通过它们启动 SLAM。
- 真正的 FAST-LIVO2 源码、`CMakeLists.txt`、`package.xml` 与配置在 [third_party/fast_livo2/fast_livo2_ros/README.md](../../../../../third_party/fast_livo2/fast_livo2_ros/README.md)；实际运行的可执行文件是 `install/ros/lib/fast_livo2_ros/fastlivo_mapping`。
- 本工程的接入点在 [../../launch/components.py](../../launch/components.py) 与 [../../odom_adapter.py](../../odom_adapter.py)：前者启动 `fastlivo_mapping`，后者把 `/aft_mapped_to_init` 适配成 `/demo/slam/{body,lidar}_odom`。上游 launch 依赖的 `config/avia.yaml`、`config/camera_pinhole.yaml`、`rviz/` 在本副本里并不存在，运行参数改由运行目录里的两个 yaml 提供。
- 本副本与上游当前文件一致这一结论只针对上述两个文件做了逐字节核对，其余上游文件（`src/main.cpp`、配置等）没有复制进来。

本目录无专用测试。图像解压脚本保留 `main()` 入口，示例 launch 保留 `generate_launch_description()`；它们需要已安装的 ROS 依赖与包资源，不是本工程导航入口。不要把这里的副本当作可构建包或算法实现。
