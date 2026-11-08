# simulation/dynanav

本目录只负责 DynaNav 场景接入与真实 Go2 的 RGB-D capture，不承载整个导航系统。

## 关键文件

- [capture_go2_rgbd.py](capture_go2_rgbd.py)：Isaac Sim 无头脚本，打开场景 USD、挂载 `go2_description`、在真实 articulation base 上装相机和 IMU，跑 200 Hz 物理与 50 Hz 策略，并发布 `/dynanav/imu`、`/dynanav/camera`、`/dynanav/lidar_filtered`、`/clock`；订阅 `/aft_mapped_to_init` 与 `/dynanav/nav/approved_cmd`。
- [collision_bounds.py](collision_bounds.py)：导出非机器人碰撞体的世界包围盒到 `complete_collision_bounds.json`，用于核对场景几何。
- 显式入口：[scripts/capture_dynanav.py](../../../../scripts/capture_dynanav.py)，支持 `--check`（只查文件、不启动 Isaac）与显式运行；场景默认取 `data_root()/dynanav/.../office.usd`，另需 Isaac Sim 6.0.1 RC 与 Student 权重。

## 边界与限制

- native 姿态、速度还用于现有仿真步态观测、COM 速度 PID 与安全断言；它们不作为 SLAM/导航定位输入。世界坐标点用于评分和渲染，RGB-D 与 truth 流分开记录，`result.json` 的 `navigation_running` 保留为 false。
- `/dynanav/lidar_filtered` 是分离后的相对 SLAM 任务节点 `navigation/relative_slam_node.py` 的输入之一，该节点向 `/dynanav/nav/approved_cmd` 回发已批准命令；capture 本身不判断路线是否到达。
- 运行参数来自环境变量（`GO2_SPAWN_X/Y/YAW`、`PROBE_STEPS`、`NAV_END_SIM`、`PROBE_RTF` 等），physics dt 固定 0.005 s，策略 50 Hz；脚本内有站立、倾倒、速度、位移等多重断言，失败会终止采集。其中 `MAX_DISPLACEMENT` 默认 0.25 m，限制相对初始 spawn 的 XY 位移；达到边界会终止采集，不能据此宣称默认入口完成长路线导航。本次未验证或调整该阈值。
- 本次没有 Isaac 环境，未启动验证；失败分支会把 traceback 写进输出目录的 `result.json`。当前仓库没有针对本目录的 unittest。
