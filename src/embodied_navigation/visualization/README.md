# visualization：DynaNav 多机位防穿模可视化与录制

本包为 D 路径（Isaac Sim / DynaNav）提供 RViz2 实时点云可视化控制与多画面录制逻辑，完全基于纯 NumPy / SciPy 与标准 ROS 2 工具封装，不修改规划或控制决策。

> 盘点日期：2026-11-26 · 规模：4 个 `.py` / 1,260 行

## 当前文件夹结构

```text
src/embodied_navigation/visualization/
├── README.md            # 本文件
├── __init__.py          # 19 行：导出 Spring-Arm 防穿墙相机运动学核心接口
├── spring_arm_camera.py # 410 行：Sphere-Cast 防穿墙悬臂相机、体素障碍栅格与楼层剖切过滤
├── spring_arm_node.py   # 252 行：ROS 2 节点，广播 rviz_cam_{spring,overhead,isometric} TF
└── rviz_recorder.py     # 579 行：多机位 RViz2 录制、防穿模回放与视频合成编排（scripts/record_dynanav_rviz.py 调用）
```

## 依赖边界与测试

- 上游：订阅 `/dynanav/backend/corrected_odometry`（或前端里程计）与去重全局点云（使用 `localization_mapping.backend.node.pack_pointcloud2_xyz` 与 `common.geometry.wrap_angle`）。
- 配置与脚本入口：`configs/visualization/*.rviz` 与 `scripts/record_dynanav_rviz.py`。
- 验证：`tests/unit/` 覆盖 `SpringArmCameraController` 与楼层剖切数学逻辑；完整端到端录制依赖 Linux + ROS 2 RViz 环境。
