# localization_mapping

本目录是导航系统的定位与建图层：把 FAST-LIVO2 的里程计和点云接入导航坐标系，提供已知场景/地图辅助定位，归档本次运行观测到的地图，并给出可选的异步后端配准。

## 边界

- 不包含 FAST-LIVO2 算法本体。真实算法与 ROS 包在 `third_party/fast_livo2`，本目录只做接入、包装、校正与状态发布。
- 不把仿真真值当定位输入；bundle 和状态都显式携带 `robot_truth_pose_used=false` 或 `navigation_ground_truth_used=false`。
- 不负责规划、控制和任务状态机，也不直接生成导航命令。

## 组成

- [backend/README.md](backend/README.md)：异步配准、关键帧历史、回环验证与 SE(3) 图优化。
- [frontend/README.md](frontend/README.md)：FAST-LIVO2 的 ROS 接入资料。
- [launch/README.md](launch/README.md)：本模块的 ROS 进程装配。
- [mapping/README.md](mapping/README.md)：运行期观测地图归档。
- 顶层节点：[odom_adapter.py](odom_adapter.py) 把 `/aft_mapped_to_init` 的 IMU 状态换算成 body/lidar 里程计与有效 twist；[localization_runtime.py](localization_runtime.py) 做已知场景地图辅助的实时定位与 bundle 发布；[building_runtime.py](building_runtime.py) 是 Building 唯一的校正 owner；[building_sensors.py](building_sensors.py) 过滤 LiDAR 自回波。
- 支撑模块：`geometry.py`（坐标代数）、`sensor_snapshot.py`（同源时间配对）、`point_provenance.py`（点与 commit 身份校验）、`known_scene_*.py`（场景参考、匹配、输入与保持门）、`relative_odometry.py`、`feedback_state.py`。

## 输入输出

前端输入是 `/aft_mapped_to_init` 与注册点云；输出 `/demo/slam/{body,lidar}_odom`、`/cloud_registered_full`、`/demo/navigation/localization_bundle`、`/demo/navigation/localization_status`。Building 路径改读 `/building/raw_body_odom`、`/building/raw_cloud`，并把状态写到运行目录的 `slam_status.json`、`slam_poses.jsonl`、`correction_applications.jsonl`。

## 测试与限制

后端有专用用例 [tests/regression/test_backend_flow.py](../../../tests/regression/test_backend_flow.py) 等；顶层 ROS 节点没有专用 unittest，[tests/unit/test_building_entry_closure.py](../../../tests/unit/test_building_entry_closure.py) 与 [tests/unit/test_production_slam_mode.py](../../../tests/unit/test_production_slam_mode.py) 覆盖的是启动/导航侧的模式与入口契约，不是本层算法。`fastlivo_mapping` 二进制来自 `install/ros`，缺失只表示本机未构建。
