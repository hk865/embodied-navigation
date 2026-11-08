# localization_mapping/mapping

本目录负责本次运行的地图归档：把 FAST-LIVO2 配准后的彩色点云累积成体素地图，并原子地发布给浏览器或离线消费方。

- 只有 [map_archive.py](map_archive.py) 一个实现，入口是 `python3 -m embodied_navigation.localization_mapping.mapping.map_archive --run-dir <run>`。
- 订阅 `/cloud_registered`（彩色点云，frame 必须是 `camera_init`），同时观察 `/livox/lidar`、`/livox/imu`、`/camera/image_color`、`/demo/slam/body_odom`、`/cloud_registered_full` 的新鲜度；写出 `map_metadata.json`、不可变的 `colored_map.NNNNNN.bin`（每点 16 字节：XYZ float32 + RGB uint8 + padding）与 `/demo/slam/map_status`。
- 保存服务是 `/demo/slam/save_map`（`std_srvs/Trigger`）：只有在 SLAM、相机与彩色点云都健康且没有容量拒绝时才写 `colored_map.pcd`；已存在 `map_metadata.json` 的运行目录会被拒绝复用。
- 元数据包含 `revision`、`point_count`、`bounds`、各 topic 的最后 stamp 与年龄、`slam_healthy`/`camera_healthy`、`save` 状态和错误；二进制修订至少保留 3 份，供 HTTP 读取方在原子替换期间继续读旧文件。
- 由 [../launch/components.py](../launch/components.py) 以 `map_archive` 角色启动，并把输入 topic 重映射到运行目录的原始流；`--run-dir` 也可用 `DEMO_RUN_DIR`，没有对应变量会直接报错，避免覆盖旧地图。

本模块无专用测试，也没有在本次核对中运行 ROS 验证地图保存。体素大小默认 0.08 m、容量上限 8,000,000 点，都是代码内现有常量。
