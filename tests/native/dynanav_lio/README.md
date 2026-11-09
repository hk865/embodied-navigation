# DynaNav native LIO / post-VIO fixtures

本目录直接测试真实 C++ `LIVMapper` / `ImuProcess`，使用小型合成点云与 IMU fixture。需要 ROS Jazzy 的 `rclcpp` 和已编译的本仓库 `fast_livo2_core` 及公共依赖；不启动 Isaac、相机参数服务或外部传感发布进程。测试会创建本地 ROS node，不是无 ROS 依赖的 NumPy 测试。

[CMakeLists.txt](CMakeLists.txt) 注册 `dynanav_lio_native` 与 `dynanav_post_vio_native` 两个 CTest。前者的 [test_lio_mode.cpp](test_lio_mode.cpp) 包含以下断言组：

- 非零 `0xA5` storage 上 placement-construct `VIOManager`，要求拥有的 `visual_submap` 为 null，且未经 camera 初始化即可析构。
- 默认配置拒绝未显式 opt-in 的 image-free LIO；允许模式不订阅图像/GPS，保留 cloud/IMU 输入。
- sparse cloud 的成对队列丢弃、buffer mutex 互斥及随后有效点/IMU 所有权不变。
- 实际 IMU 非零加速度/角速度传播中，generic 瞬时点的参考时间正确、原始点序/强度/offset 不被修改。
- 非 generic 点云保持原有 timed offsets 与所有权路径。

[test_post_vio_output.cpp](test_post_vio_output.cpp) 在本地 pub/sub fixture 中注入已结束估计的状态，检查显式 opt-in、原 image stamp、map/child frame、同帧 status、有效 no-op 的 false 标记、不可用 covariance/twist、重复/倒序/非有限/LIO-stage 拒绝，以及发布不改估计器状态。它不运行图像前端，不证明视觉精度或控制闭环。

## 对已安装的新 core 构建/运行 fixture

从仓库根目录运行。以下使用已验证的新 core prefix；fixture 写入另一个新外部构建目录，不覆盖原 core。

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
CORE_PREFIX="$PROJECT_DATA_ROOT/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/install/ros"
DEPENDENCY_PREFIX="$PROJECT_DATA_ROOT/embodied-navigation/build_cache/native_validation_20261108T124116Z/project/install/ros"
TEST_BUILD="$PROJECT_DATA_ROOT/embodied-navigation/build_cache/native-lio-check-$(date -u +%Y%m%dT%H%M%SZ)"
export LD_LIBRARY_PATH="$CORE_PREFIX/lib:$DEPENDENCY_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cmake -S tests/native/dynanav_lio -B "$TEST_BUILD" \
  -DCMAKE_PREFIX_PATH="$CORE_PREFIX;$DEPENDENCY_PREFIX;/opt/ros/jazzy" \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$TEST_BUILD" --parallel 2
ctest --test-dir "$TEST_BUILD" --output-on-failure
```

`fast_livo2_core` 必须是已从本仓库源码构建并安装的相应版本；本目录只构建 fixture，不下载依赖或重建整个 core。其它机器可显式替换两个 prefix。完整原生依赖构建见 [BUILD.md](../../../docs/dependencies/BUILD.md)，估计器入口见 [LIO/LIVO README](../../../configs/simulation/dynanav/README_LIO.md)。

## 实测与旧 core 反例

当前 post-VIO core 的 [build-identity.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/build-identity.json) 和 [test-result.txt](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/test-result.txt) 记录 **2/2 CTest 通过，0.51 s**。它包括已修复的 LIO/lifecycle 和新 final-pose 发布契约，不是物理楼梯、视觉定位精度或完整导航验收。先前 lifetime prefix 的 1/1、0.28 s 是历史阶段结果，原二进制未被覆盖。

旧 offset core SHA256 `9f13c15ad9f326c55772066c12adc6378ddd463a96bff6b487c417c3c4913c51` 的构造器没有初始化 submap 指针；`ONLY_LIO` 又跳过 `initializeVIO`，析构会 delete 不确定指针。真实 gdb 曾定位析构 SIGSEGV，切换 DDS transport 没有消除根因。相同 poisoned-storage fixture 对旧 core 确定性失败，输出 `VIOManager did not initialize visual_submap ownership`，而不是让偶然清零的 allocator 隐藏错误。

证据见[原始生命周期诊断](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-slam-landing-sdk200/verification/native-lifecycle-diagnosis/lifecycle-diagnosis.json)及同目录反例日志。[0004 patch](../../../third_party/fast_livo2/patches/0004-safe-uninitialized-vio-lifecycle.patch) 仅建立 null ownership，正常 LIVO 仍按原初始化分配 submap，没有改变估计数学。旧 core/prefix 与旧运行哈希保留，未被新构建覆盖。
