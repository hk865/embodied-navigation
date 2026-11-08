# fast_livo2_ros

本包提供`fastlivo_mapping`可执行入口与上游launch/config/rviz资源。滤波、同步和建图实现在兄弟[fast_livo2_core](../fast_livo2_core/README.md)。

## 入口与接口

`src/main.cpp`进入`fast_livo2_core::run_mapping()`，main负责ROS初始化，runner负责mapper生命周期和主循环。普通工程入口直接调用项目安装内的可执行文件，传入本次生成的两个参数文件；不是Include上游bag演示launch。

输入为当前Generic LiDAR PointCloud2、relay IMU与RGB；主要原始输出为`/aft_mapped_to_init`和remap后的`/demo/slam/raw_registered_full`。body/LiDAR外参适配和全局C纠偏均在Python层，详细topics见[系统数据流](../../../../docs/source_architecture.zh-CN.md)。

## 配置和使用

根`config/slam/fastlivo.yaml`和`camera.yaml`是base，`runtime_inputs.prepare()`依据传感器安装生成外参并覆盖实际参数。`common`指定topics，`imu`控制初始化/估计噪声，`lio`控制平面地图，`vio`控制patch，`extrin_calib`绑定传感器。不要只修改上游`config/avia.yaml`后预期日常运行生效。

从仓库根使用`python3 -B start.py`。上游`launch/mapping_aviz.launch.py`可作为bag接口参考，传感器、frame和依赖与当前链不同，未经本工程等价验证。编译/安装顺序见[恢复说明](../../../../docs/restoration.md)。

调试需同时检查实际选中的可执行、生成参数和topic remap。ROS context关停期间native非零退出仍有已知边界；外层entry退出0不能代替每个子进程退出码。原理和当前开关见[SLAM模块](../../README.md)。
