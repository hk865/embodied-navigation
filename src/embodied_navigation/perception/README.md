# perception：图像/深度处理

`perception` 负责图像与深度的直接处理，把传感器数据转换成下游可用的几何结果，不做 SLAM、定位或控制。

## 边界

- 不放检测器/训练代码；`perception/__init__.py` 只说明实际检测后端应归入本包，而不是 planning。
- 不订阅 ROS、不读仿真真值；输入是数组与标定参数。

## 关键文件与接口

- `depth.py`：`depth_to_cloud(depth, sensor_yaw, camera_mount, imu_mount, focal_px=512*24/42, center_px=(256,256))` 把深度图反投影到 Go2 本体 IMU 坐标系；过滤非有限与 0.05–20 m 之外的点，用固定机体包围盒剔除自车点，并按 8000 点上限等步长降采样。返回 `(points, keep, self_mask)`。
- 输入还包括相机安装与 IMU 安装的相对位姿；默认焦距/中心值来自当前源码内置相机参数，单位是像素。

## 调用与依赖

[simulation/dynanav/capture_go2_rgbd.py](../simulation/dynanav/capture_go2_rgbd.py) 调用 `depth_to_cloud` 做 DynaNav 场景的真实 Go2 RGB-D capture。`perception/sensors/` 是感知侧的 ROS 传感流适配（转发、自车点过滤、健康门、反馈邮箱），与顶层 [sensors/](../sensors/README.md) 的跨层数据契约不是同一层，见 [sensors/README.md](sensors/README.md)。

## 测试与限制

[tests/unit/test_perception.py](../../../tests/unit/test_perception.py) 用固定随机样例逐点比对原投影/自过滤公式。本目录没有其他功能测试；DynaNav capture 需要匹配的 Isaac 安装与场景资产，本会话未运行。
