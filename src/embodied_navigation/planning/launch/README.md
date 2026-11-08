# planning/launch

本目录只是 planning 的 SCAN 进程组 launch 装配，不含规划实现。

`scan.launch.py` 只声明 `run_dir` 参数，`components.py` 用 `owned()` 启动唯一的 `scan_supervisor` 进程，参数为 `--run <run_dir>`、`--planner <ros_prefix>/lib/scan_planner/scan_planner_node`、`--config <ros_prefix>/share/scan_planner/config/planner.yaml`、`--params <run_dir>/v34_scan_overrides.yaml`。`ros_prefix` 来自 `../../../../configs/launch/settings.json` 的 `runtime_paths.ros_prefix`，默认是仓库根下的 install/ros。

SCAN planner 二进制与配置随 install/ros 构建产物提供（由 `../../../../docker/build_native.py` 构建），源码在 `../../../../third_party/README.md` 所列的 SCAN 包中；本仓库不携带构建产物，install 缺失只表示本机未构建。该组被 `../../launch/launch/demo.launch.py` 以 `('planning','scan')` 引入，没有可单独执行的入口。真实实现在 `../scan_epoch_supervisor.py`，见 `../README.md`。

## 测试

本目录无专用测试；通用启动 CLI 由 `../../../../tests/unit/test_launch_cli.py` 覆盖，不验证 `scan_supervisor` 实际可启动。
