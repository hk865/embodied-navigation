# localization_mapping/frontend/fast_livo2_ros/scripts

本目录存放上游 FAST-LIVO2 示例脚本的副本，用于理解 bag 回放时的图像解压；不是本工程运行链路的一部分。

- [decompress_images.py](decompress_images.py)：一个 rclpy 节点，订阅 `/camera/image_color/compressed`，用 `cv_bridge` 解成 `bgr8` 后以原始 header 时间戳发布到 `/camera/image_color`。
- 上游 launch 自己改用 `image_transport republish` 完成同样的事，因此这个脚本是参考实现，不是必需节点。
- 上游原文件在 [third_party/fast_livo2/fast_livo2_ros/scripts/decompress_images.py](../../../../../../third_party/fast_livo2/fast_livo2_ros/scripts/decompress_images.py)，与本副本一致；本工程的启动配置没有调用它。

无专用测试。本工程启动链不调用该脚本；脚本保留独立 `main()` 入口，需要 ROS 2 Python 环境、`sensor_msgs`、`cv_bridge` 与 OpenCV。本次仅静态核对，未运行节点。
