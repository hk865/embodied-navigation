# vikit_common / vikit_ros

来自 SVO (Semi-Direct Visual Odometry) 项目的视觉工具库，提供相机模型抽象、数学工具、鲁棒代价函数和 ROS2 参数加载功能。

当前导航链的配置/使用入口见[SDK说明](../../README.md)，有效相机内参由根`config/slam/camera.yaml`与生成参数决定。下文参数示例用于接口说明，不作为当前数值。

## 包概览

| 包名 | 类型 | 职责 |
|------|------|------|
| `vikit_common` | 共享库 | 相机模型 + 数学工具 + Robust Cost + 图像对齐 |
| `vikit_ros` | 共享库 | ROS2 参数加载 + 可视化辅助 |

## 代码架构

### vikit_common — 相机模型与视觉工具

**相机模型继承树：**

```
vk::AbstractCamera (抽象基类)
├── vk::PinholeCamera        # 针孔相机模型 (含径向畸变 d0~d4)
├── vk::ATANCamera           # FOV/ATAN 畸变模型 (Deverneay & Faugeras 2001)
├── vk::OmniCamera           # 全向相机模型 (omnidirectional)
├── vk::EquidistantCamera    # 等距投影相机模型 (fisheye)
└── vk::PolynomialCamera      # 多项式投影模型
```

| 文件 | 类/函数 | 功能 |
|------|--------|------|
| [abstract_camera.h](include/vikit/abstract_camera.h) | `AbstractCamera` | 相机模型抽象基类。定义 `cam2world()`, `world2cam()`, `isInFrame()` 等核心接口。 |
| [pinhole_camera.h](include/vikit/pinhole_camera.h) | `PinholeCamera` | 针孔相机模型。支持径向畸变 (k1~k3 + p1~p2)，内建畸变校正映射 `undistortImage()`。 |
| [atan_camera.h](include/vikit/atan_camera.h) | `ATANCamera` | FOV 畸变模型，源自 PTAM 中的 ATAN 实现。适用于广角相机。 |
| [omni_camera.h](include/vikit/omni_camera.h) | `OmniCamera` | 全向/全景相机模型。参数包括 xi (镜面参数)、焦距、畸变等。 |
| [equidistant_camera.h](include/vikit/equidistant_camera.h) | `EquidistantCamera` | 鱼眼等距投影模型。常用于 180° 以上 FOV。 |
| [polynomial_camera.h](include/vikit/polynomial_camera.h) | `PolynomialCamera` | 多项式投影模型。 |
| **数学工具** |||
| [math_utils.h](include/vikit/math_utils.h) | `shuffle()`, `triangulateFeature()` 等 | 数学辅助：矩阵洗牌、三角化、非线性最小二乘等。 |
| [nlls_solver.h](include/vikit/nlls_solver.h) | `NllsSolver` | 非线性最小二乘求解器模板。 |
| [robust_cost.h](include/vikit/robust_cost.h) | `TukeyWeightFunction`, `HuberWeightFunction` | Tukey/Huber 鲁棒核函数，用于 M-estimator 外点抑制。 |
| [homography.h](include/vikit/homography.h) | 单应矩阵计算 | 从匹配点对计算单应矩阵。 |
| [img_align.h](include/vikit/img_align.h) | 图像对齐 | 直接法图像对齐工具。 |
| [vision.h](include/vikit/vision.h) | 视觉工具 | 视觉相关辅助函数。 |
| **性能工具** |||
| [performance_monitor.h](include/vikit/performance_monitor.h) | `PerformanceMonitor` | 性能监控/计时器。 |
| [timer.h](include/vikit/timer.h) | `Timer` | 高精度计时器。 |
| [ringbuffer.h](include/vikit/ringbuffer.h) | `RingBuffer` | 环形缓冲区容器。 |
| [aligned_mem.h](include/vikit/aligned_mem.h) | `AlignedMem` | 对齐内存分配。 |
| [patch_score.h](include/vikit/patch_score.h) | Patch 评分 | 图像 patch 的评分（如 ZMSSD）。 |

### vikit_ros — ROS2 集成

| 文件 | 功能 |
|------|------|
| [camera_loader.h](../vikit_ros/include/vikit/camera_loader.h) | 从 ROS2 参数服务器加载相机模型：`loadFromRosNs()` 根据 `cam_model` 参数（"Pinhole", "ATAN", "Omni", "Equidistant", "Polynomial"）创建对应的 `AbstractCamera` 子类实例。 |
| [output_helper.h](../vikit_ros/include/vikit/output_helper.h) | ROS2 可视化辅助函数（发布点云、路径、T_W_C 姿态等）。 |
| [params_helper.h](../vikit_ros/include/vikit/params_helper.h) | ROS2 参数获取辅助宏/函数。 |

## 数据流分析

### 在本项目中的使用

`vikit_common` 和 `vikit_ros` 被 [fast_livo2_core](../../localization/frontend/fast_livo2_core/README.md) 作为核心依赖使用：

```
vikit_ros::camera_loader::loadFromRosNs()
    │  从 ROS2 Parameter Server 读取 cam_model + 内参
    ▼
vk::PinholeCamera (或 ATANCamera 等)
    │  传入 VIOManager + Frame
    ▼
VIOManager::cam  →  用于:
    ├── cam2world(): 像素 → Bearing Vector (VIO 初始化)
    ├── world2cam(): 3D点 → 像素 (投影与优化)
    ├── isInFrame(): 检查点是否在视野内
    └── getWarpMatrixAffine(): Patch 仿射 Warp
```

### vikit_ros 参数格式

在 `camera_pinhole.yaml` 等配置文件中：

```yaml
/**:
  ros__parameters:
    cam_model: Pinhole               # 相机模型类型
    cam_width: 640                   # 图像宽度
    cam_height: 480                  # 图像高度
    scale: 1.0                       # 缩放因子 (0.5 = half resolution)
    cam_fx: 349.2                    # 焦距 x
    cam_fy: 349.2                    # 焦距 y
    cam_cx: 320.0                    # 光心 x
    cam_cy: 240.0                    # 光心 y
    cam_d0: 0.0                      # 畸变系数 k1
    cam_d1: 0.0                      # 畸变系数 k2
    cam_d2: 0.0                      # 畸变系数 k3
    cam_d3: 0.0                      # 畸变系数 p1
```

### 构建关系

```
vikit_common ──────► vikit_ros ──────► fast_livo2_core ──────► fast_livo2_ros
(相机模型+数学)      (ROS2集成)        (SLAM算法库)            (ROS2可执行)
```
