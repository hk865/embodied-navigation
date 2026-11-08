# evaluation：离线指标与进程资源采样

本目录提供轨迹长度/误差等离线指标，以及独立运行的进程 CPU/RSS 采样入口，不参与导航控制或实时决策。

## 边界

- 指标只做离线计算；真值可以用于评测误差，但真值仍只属于评测/仿真安全边界，不得写成 SLAM/导航定位输入。
- `process_resources.py` 是独立 `python -m` 进程，导入模块本身不产生副作用。

## 关键文件与接口

- `trajectory.py`：`path_length_xy(positions)` 返回 XY 路径长度；`position_error_xy(estimated, reference)` 返回 `rmse_m`、`max_m`、`final_m`。输入是位置数组，只取前两列。
- `process_resources.py`：`main()` 从 `DEMO_RUN_DIR` 读 `processes.json`，每秒采样 `/proc/<pid>/stat` 并写 `process_resources.csv`，列为 `wall_monotonic_s`、`role`、`pid`、`cpu_total_s`、`cpu_cores_interval`、`rss_mib`（RSS 单位 MiB）。

## 调用与依赖

[launch/building.py](../launch/building.py) 以子进程方式启动 `embodied_navigation.evaluation.process_resources`；`trajectory.py` 由 `__init__.py` 重导出，当前主要供离线分析/测试使用，不在主导航流程中调用。

## 测试与限制

[tests/unit/test_metrics.py](../../../tests/unit/test_metrics.py) 覆盖两个轨迹函数；[tests/unit/test_building_entry_closure.py](../../../tests/unit/test_building_entry_closure.py) 验证 `process_resources` 导入无副作用且被 Building 入口引用。本目录不是完整评测框架：没有路线成功判定、汇总报告或与 run 结果绑定的验收流程；资源采样只读 `/proc`，Linux 之外不可用。
