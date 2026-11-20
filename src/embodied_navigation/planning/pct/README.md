# planning/pct（实测支撑 → 原生 PCT A*）

> 盘点日期：2026-11-20 · 分支 `refactor/06-clean-base` · HEAD `5b880b2`

本目录是“当前一代实测支撑网格 → vendored 原生 PCT A*”的适配器：真正的 A* 算法在 [`third_party/pct`](../../../../third_party/pct/CMakeLists.txt)，这里只负责构造有界多层实测支撑网格、编译/定位原生可执行文件并解释其结果。

## 模块定位与架构层级

- 架构层级：第 4 层 —— 规划层内的全局规划子模块（PCT 部分）；SCAN 局部规划与参考交接在上一级 [`planning/`](../README.md)。
- 输入：调用方给出的 **map-frame 实测点云 XYZ**（worker 按同 stage 精确 stamp 配对后累积的实测代表点）、可选的实测 body 轨迹（只作 provenance，不用于补出支撑）、以及 body child-origin 约定的 `start`/`goal` XYZ。
- 输出：`plan()` 返回 `{found, body_path_xyz, reason, provenance}`；`found=False` 时 `reason` 为 `no_measured_support` / `start_support_unobserved` / `goal_support_unobserved` / `native_no_path` 之一，且 `body_path_xyz` 为空——**从不返回部分路线，也不做最近目标替换或直线兜底**。
- 上游 / 下游：被 [`simulation/dynanav/pct_scan_worker.py`](../../simulation/dynanav/pct_scan_worker.py) 的 `plan_snapshot()` 与 [`planning/exploration/waypoint_candidates.py`](../exploration/waypoint_candidates.py) 调用；下游是原生可执行 `pct_astar`。
- 边界：本模块不是 clearance 证书、不是支撑承载证明、也不是 goto 门禁。它不做射线开凿、补洞、场景资产/tomogram 加载；未知支撑默认不可通行（`unknown_support_traversable=False`）。

## 当前文件夹结构

```text
src/embodied_navigation/planning/pct/
├── README.md            # 本文件
└── measured_tomogram.py # MeasuredPctPlanner、native_pct_binary、MeasuredPctError
```

> 本目录**没有** `__init__.py`，`embodied_navigation.planning.pct` 是 PEP 420 隐式命名空间包；调用方必须写完整模块路径（例如 `from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner`）。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `README.md` | 本文件：模块定位、原生可执行文件来源、起点接入规则与测试入口。 | — |
| `measured_tomogram.py` | 只从调用方给的实测 XYZ 构造有界多层支撑网格，序列化后调用原生 PCT A*；`MeasuredPctError` 表示原生构建/序列化/执行失败。 | `MeasuredPctPlanner(points_world, trajectory_body_xyz=None, *, resolution_m=0.15, body_support_offset_m=0.32, native_binary=None, generation=None, source_metadata=None, endpoint_height_tolerance_m=0.22, support_cluster_m=0.06, clearance_m=0.65, max_grid_cells=2_000_000, timeout_s=10.0, start_blind_radius_m=0.0)`；`plan(start_body_xyz, goal_body_xyz)`；`native_pct_binary()`；常量 `DEFAULT_MAX_GRID_CELLS`。 |

## 原生 PCT 可执行文件从哪来

三条来源，按默认路径优先级说明：

1. **自动构建（默认路径）**：`native_pct_binary()` 对 `third_party/pct` 全部文件按“相对路径 + 内容”算 sha256，把可执行文件缓存在 `$PROJECT_DATA_ROOT/embodied-navigation-cache/pct-<摘要前 16 位>/pct_astar`（未设 `PROJECT_DATA_ROOT` 时退到系统临时目录）。缓存缺失时用 `fcntl.flock` 串行化并发构建，执行 `cmake -S third_party/pct -B <cache> -DCMAKE_BUILD_TYPE=Release` 与 `cmake --build <cache> --parallel 2`（每步 120 s 超时），**全程不写 checkout**，失败抛 `MeasuredPctError`。
2. **显式预构建二进制**：`docker/build_native.py` 把 `third_party/pct` 安装到 `install/pct/pct_astar`；调用方可用 `native_binary=<路径>` 直接指定（worker CLI 为 `--pct-binary`），此时不会自动构建。
3. **来源校验冒烟**：`python3 docker/verify_native_sources.py --compile-pct` 在临时目录重新编译并对一个有界自由网格做一次规划冒烟，不联网、不加载模型。

原生协议（[`third_party/pct/pct_cli.cpp`](../../../../third_party/pct/pct_cli.cpp)）：`map.bin` = int32 三维尺寸 + double resolution + 三层 double 平面（cost / height / gateway，本适配器第三层填 0）；`query.txt` = start 与 goal 各一个 `layer col row`；`path.json` = `{"found": bool, "cells_layer_row_col": [[layer,row,col], ...]}`。退出码 0 = 找到，4 = 无路，其他 = 错误。适配器显式做 query `(layer,col,row)` 与结果 `(layer,row,col)` 的坐标顺序转换。

## 网格构造与起点接入（读码核实）

- 每个 cell 只填充实测点：同一 XY cell 内按 `support_cluster_m=0.06` 切垂直簇，丢弃高度跨度大于簇宽的垂直墙柱；再看 foot 以上 `max(0.10, cluster)` 到 `clearance_m=0.65` 之间是否有障碍，有则不作为支撑。
- 支撑 cell 必须在 **XY 两个轴向上都有实测邻居**（同一 grid cell 内、高度差 `<=0.2000001 m`）；这拒绝孤立 return 与细竖柱，但不证明可承载。未知格保持 blocked。
- 网格尺寸与层数都受 `max_grid_cells`（默认 `DEFAULT_MAX_GRID_CELLS = 2_000_000`）约束，超限抛 `MeasuredPctError`；原生整数高度哈希要求 `max(height)/resolution < 210`。
- **起点脚印 4 邻域实测支撑接入**（`_footprint_start_node(start, max_radius_m=0.25)`）：仅在起点自身 cell 已有“足底高度附近实测 return”（例如台阶鼻或坡边导致单 cell 水平聚类失败）且 clearance 带内无实测障碍、无深于容差 0.10 m 的落差时，把起点挂到机器人足迹（水平 ≤0.25 m）内**直接 4 邻域**（`(-1,0)`、`(1,0)`、`(0,-1)`、`(0,1)`）的实测支撑 cell 上；只取 4 邻域，明确拒绝对角切角、未观测洞、墙与悬崖，且**不伪造 cell、不放宽 goal 准入**（goal 仍走严格 `_node()`）。结果写入 provenance `start_footprint_snap`（`applied`、`cell_layer_row_col`、`horizontal_distance_m`、`vertical_delta_m` 或拒绝原因 `start_out_of_bounds` / `start_cell_unobserved` / `start_cell_obstacle` / `start_cell_cliff_drop` / `start_cell_no_foot_elevation` / `no_adjacent_support`）。
- `plan()` 顺序：严格 `_node(start)` → 失败才尝试 4 邻域脚印快照 → 仍失败且 `start_blind_radius_m > 0` 才尝试盲区平面模型（`_start_surface_model`：周边 ≥20 个实测支撑 cell、覆盖四个象限、p90 残差 ≤0.035 m、坡度 ≤0.15；只推断起点半径内与起点连通且未观测的 cell，实测 cell 不覆盖，高度不兼容处保留 1 cell halo，provenance 标 `observed_free=False`、`support_certified=False`）。
- `body_support_offset_m` 是 child-origin 到支撑面的显式配置偏移，默认 `0.32`；默认启动器传 `PCT_BODY_SUPPORT_OFFSET_M=0.3148910446837544`（0.32 标称基座 + 记录到的 IMU 安装 z 的组合），这是机器人高度假设，不是实测地面认证。盲区模型默认关闭，启动器以 `PCT_START_BLIND_RADIUS_M=1.5` 显式开启。

## 验证与测试入口 / 配置注意

- [`tests/unit/test_measured_pct_tomogram.py`](../../../../tests/unit/test_measured_pct_tomogram.py)：`setUpClass` 真实调用 `native_pct_binary()` 跑 vendored C++ A*（需要 cmake）；覆盖矩形轴与端点语义、绕障、未知缺口无兜底、合成楼梯 4 m 高度目标、负坐标与负高度、盲区环形模型不覆盖实测格/不跨越半径外缺口/不覆盖实测障碍、起点脚印跨单个台阶 riser 的 4 邻域接入、以及墙/悬崖/对角切角拒绝。
- [`tests/unit/test_pct_grid_budget.py`](../../../../tests/unit/test_pct_grid_budget.py)：`DEFAULT_MAX_GRID_CELLS` 预算语义、worker CLI 默认值/显式值确实传到构造函数、启动器参数分派与非法值在启动前被拒。
- 命令（仓库根执行）：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`；原生编译冒烟：`python3 docker/verify_native_sources.py --compile-pct`。
- 运行参数：worker 侧 `--pct-binary`、`--pct-max-grid-cells`、`--body-support-offset`、`--pct-start-blind-radius`；构建说明见 [`docs/dependencies/BUILD.md`](../../../../docs/dependencies/BUILD.md)。
- 限制：point-footprint 全局规划器，不认证承载/机身 clearance，也不消除上游对角切角；稀疏实测可能无解。合成单测与 CPU 编译通过不等于 Isaac 闭环或实机可通行。相关系统说明见 [`docs/dynanav-pct-scan.md`](../../../../docs/dynanav-pct-scan.md)。
