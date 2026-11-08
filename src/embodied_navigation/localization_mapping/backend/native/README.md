# localization_mapping/backend/native

本目录是仓库自带的一个 C++ SE(3) 稀疏图求解器，供后台 Python 通过 C ABI 调用。

- [graph_solver.cpp](graph_solver.cpp)：导出 `livo_optimize_graph`，输入位姿数组、边端点、相对测量、平移/旋转 σ 与 loop 标记，在固定窗口内迭代并返回代价与耗时；不含 ROS、前端状态、点云或文件 I/O。
- [CMakeLists.txt](CMakeLists.txt)：依赖 Eigen3，构建共享库 `livo_graph` 并安装到 `lib/`。

真实实现就在本目录，执行入口不在 C++：[../native_graph.py](../native_graph.py) 固定加载 `install/backend/lib/liblivo_graph.so`，由 [../history.py](../history.py) 的 `History.optimize` 间接调用；库缺失时返回 `native_graph_solver_unavailable`，不阻断历史收集。构建走 `docker/build_owned_native.py` 的 `backend` 模块，说明见 `docs/dependencies/BUILD.md`。

`install/` 中缺少该 `.so` 只表示本机未构建，源码完整。求解路径在图优化用例中被间接覆盖：[tests/regression/test_backend_flow.py](../../../../../tests/regression/test_backend_flow.py) 的 `test_verified_geometry_loop_optimizes_and_rebuilds_history` 需要该共享库存在。
