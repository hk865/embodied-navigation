# 导航源码依赖闭包

审计与复制日期：2026-11-08。源仓库为 [hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control](https://github.com/hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control)，提交 `5688f2868e241ca4a7cc80ba9f166f8afa84c727`，分支 `feature/fastlivo2-backend-building-20261030`。原工作树跟踪文件干净，仅未跟踪的 `history/` 未复制。没有 Git 子模块或 gitlink；所有必要包为实体文件。

## 当前已提供

- FAST-LIVO2 core + ROS wrapper，含实际使用的 VIO 四处 floor 修复。
- SCAN `plan_manage`、`plan_env`、`path_searching`、`bspline_opt`、`traj_utils`，以及生成 `Bspline` 的 `scan_planner_msgs`。
- PCT `pct_cli.cpp`、`native/a_star/a_star_search.{cc,h}` 和 `native/common/data_types.h`，加原有 LICENSE/NOTICE 和独立 CMake 构建文件。
- Vikit common/ROS、Livox ROS Driver2 的实际源码和现有声明。
- FastDDS 2.14.6/Asio 固定公开源码来源和三份实际补丁、构建脚本。
- 可从公开依赖构建的 CPU ROS Jazzy Docker 原生层。没有复制旧二进制或使用其它本地定制工程。

## 文件入口

- [构建与运行边界](BUILD.md)
- [Python/部署与额外原生模块](PYTHON-AND-DEPLOYMENT.md)
- [许可状态](LICENSE-STATUS.md)
- [逐文件复制清单](vendored-source-manifest.json)
- [本次验证结果](verification.json)

## 依赖关系

`livox_ros_driver2 + vikit_common + vikit_ros -> fast_livo2_core -> fast_livo2_ros/fastlivo_mapping`。

`plan_env -> path_searching -> bspline_opt -> traj_utils`，这些包与 `scan_planner_msgs` 一同供 `scan_planner` 使用。实际入口启动 `scan_planner_node` 和 `closed_loop_controller`，因此不能只复制单个规划器 `.cpp`。

FAST-LIVO2 的两个包必须维持相邻结构，wrapper 的 CMake 包含 `../fast_livo2_core/include/...`。SCAN `plan_env` 的回归测试源在原 CMake 中无条件构建，已保留，不能仅通过 `BUILD_TESTING=OFF` 排除它们。

原 SCAN `package.xml` 中还保留 `go2_description`、`map_generator`、`mockamap`、`local_sensing_node`、`odom_visualization` 等上游演示依赖。这些不在当前直接启动节点的编译闭包内，本目录没有伪造这些包；上游演示 launch 不因此自动可用。构建脚本显式选择本目录的 11 个包，避免把未使用演示误当作系统入口。

PCT 的 Python 接口、`common/smoothing`、GTSAM、OSQP、pybind11 不在当前 CLI 调用链中，未复制。其原 NOTICE 提及这些完整上游工程组件，已原样保留；这不是声明本工程提供或需要它们。
