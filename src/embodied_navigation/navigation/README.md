# navigation

本目录是导航任务层：把 canonical SLAM 位姿、IMU、注册点云与 SCAN 轨迹组织成导航任务，决定航点等待、跟随、到点稳定、转向、停车与失败。

## 边界

- 做：任务状态与节点适配（`task.py`、`pid_navigation.py`、`ros_node.py`、`base_node.py`、`building_navigation.py`），以及到达区域与任务运行时（`mission/`、`robogauge/`）。
- 不做：控制律（在 `../control/tracking/`）、参考路线与准入（在 `../planning/`）、进程装配（在 `../control/launch/` 等 launch 组）。
- 不把仿真真值当作 SLAM 输入；真值只出现在明确标注的评测与仿真安全边界。

## 关键文件

- `pid_navigation.py`：现役 controller 入口。`PIDNavigation(TeacherNavigation)` 装配 cascade、路径准入、mission 与停车/安全证据；`../control/launch/components.py` 以 `--run <run_dir>` 启动它。
- `ros_node.py`：`TeacherNavigation(base.Navigation)`，Teacher 的有限包装，拒绝重复或倒退的来源 header，并要求实测 SLAM/点云。
- `base_node.py`：`Navigation` 基类，做 measured-pose SCAN 执行、freshness watchdog 与 LiDAR 停/恢复；自身不设仿真位姿、不订阅真值。
- `task.py`：现有 shuttle 任务状态机（等待、跟随、settle、转向、停止/失败），与 ROS、仿真解耦；驾驶指令来自 `../control/relative_pid.py`。
- `relative_slam_node.py`：相对 SLAM 任务的 ROS 适配器，也是 pyproject 的 `relative-slam-navigation` 入口；只有显式 `main()` 才创建节点。
- `building_navigation.py`：Building 分支节点（canonical FAST-LIVO2，只允许 SLAM），读 run 目录的 `PCT_route.json`，由 `../launch/building.py` 与 `robogauge/benchmark.py` 启动。
- `mission/` 见 `mission/README.md`；`robogauge/` 见 `robogauge/README.md`。

## 测试

`../../../tests/unit/test_navigation.py` 只覆盖 `task.py` 的状态与失败条件；`../../../tests/unit/test_building_entry_closure.py` 覆盖 Building 入口只接受 SLAM 的准备逻辑。`pid_navigation.py`、`ros_node.py`、`base_node.py`、`building_navigation.py` 没有专用单测，只随运行链路使用。

## 限制

完整路线、Building 实际运行与实机均未验证。完整原生闭环依赖 SCAN/FAST-LIVO2 构建产物；仓库根 install 缺产物时仍可阅读源码并运行不依赖这些产物的单元测试，不能据此验证完整闭环。
