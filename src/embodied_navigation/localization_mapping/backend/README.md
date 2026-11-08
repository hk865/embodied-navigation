# localization_mapping/backend

本目录是可选的后端定位组件：在与前端分离的线程或进程中收集关键帧、对外部地图做配准、验证回环并做 SE(3) 图优化，最后输出可被定位 owner 采纳的校正候选。

## 边界

- 不做前端 IMU/滤波/里程计，也不写前端状态。
- 不直接发布导航姿态。worker 可通过 `poll` 返回候选，也可将候选写入结果文件；当前 Building 路径由 `../building_runtime.py` 读取 `backend_result.json` 并决定是否应用。`../localization_runtime.py` 使用独立的已知场景匹配链路。
- CPU Python 配准是参考实现，不代表 C++ 或实时吞吐指标。

## 关键文件与接口

- [worker.py](worker.py)：`BackendWorker`，有界队列；`try_submit` 只入队，`poll` 取结果，`close(timeout)` 请求关闭并等待排空，返回是否已退出；超时返回 `False`。队列满时丢弃新关键帧，不丢已接受的图边。
- [history.py](history.py)：`Keyframe.owned`、`Edge`、`History.add/verify_and_add_loop/optimize/save/load`。
- [external_map.py](external_map.py)：`read_pcd`、`ExternalMap` 空间分块；[registration.py](registration.py)：`Relocalizer`、`RegistrationConfig/Result`；[tracking.py](tracking.py)：`PriorTracker` 在显式先验盆地内跟踪。
- [correction.py](correction.py)：`TemporalAdmission`（时间确认）、`apply_bundle`（同一校正同时作用于位姿与点云）、`BackendResultReader`（按年龄/epoch/sequence 设门限读取）。
- [native_graph.py](native_graph.py)：加载 `install/backend/lib/liblivo_graph.so`，调用 C++ 图求解器；原生源码见 [native/README.md](native/README.md)。
- [run.py](run.py)：无 ROS 的 worker CLI，参数含 `--spool/--history/--external-map/--result/--initial-world-odom`。

## 调用与依赖

Building 入口 `launch/building.py` 和 RoboGauge benchmark 都用 `python3 -m embodied_navigation.localization_mapping.backend.run` 启动；队列、历史、地图写在运行目录，不依赖 ROS。

## 测试与限制

专用回归用例：[tests/regression/test_backend_flow.py](../../../../tests/regression/test_backend_flow.py)、[test_backend_registration.py](../../../../tests/regression/test_backend_registration.py)、[test_backend_auto_loops.py](../../../../tests/regression/test_backend_auto_loops.py)，均为纯 CPU、不启动 ROS/Gazebo；图优化相关断言要求 `install/backend/lib/liblivo_graph.so` 已构建（见 [native/README.md](native/README.md)）。`run.py` 顶部 docstring 里的 `-m localization.backend.run` 是旧路径，实际以 `embodied_navigation...` 启动。
