# planning（规划层）

> 盘点日期：2026-11-20 · 分支 `refactor/06-clean-base` · HEAD `5b880b2`

本目录是规划层：把 vendored 原生 SCAN 源码 staged 并打补丁到检出外的构建目录、维护一次 PCT→SCAN 参考交接状态机，并把实测分层支撑接入 vendored 原生 PCT A*；Agent 侧只读航点候选在 `exploration/`。

## 模块定位与架构层级

- 架构层级：第 4 层 —— 规划层（PCT 全局 + SCAN 局部）。本目录只做源码来源治理、参考交接与规划器接入，算法本体在 `third_party/`。
- 输入：
  - `third_party/scan/`（vendored 源码与 `patches/*.patch`）：`scan_source.stage_scan_source()` 的输入；
  - 当前 run 的 `capture/interactive/` mailbox：`planner_request.json`、`planner_measured_map.npz`、`planner_global_route.json`、`observation.json`（由 `simulation/dynanav/pct_scan_worker.py` 与采集侧写入）；
  - 实测 map-frame 点云 XYZ：默认 `/dynanav/rtx_livo/cloud_registered_full`，`--truth-pose` 时为 `/dynanav/truth/cloud_deskewed`，由 worker 完成同 stage 精确 stamp 配对后传入；
  - SCAN 子进程回传的 ACK / route definition / reference status，话题在 worker 命名空间下（`.../planning/odometry_consumption_ack`、`.../planning/route_definition`、`.../planning/reference_status`）。
- 输出：外部构建目录中的 staged SCAN 源码树与 `source_stage.json`、`patches.log`；PCT 全局路线 `body_path_xyz` 与 provenance；经 `ReferenceHandoff` 校验的参考投递状态；`exploration/` 的只读候选 JSON（schema `dynanav_waypoint_candidates/v1`）。
- 上游 / 下游：上游是 `scripts/run_dynanav_native_exploration.sh` → [`scripts/run_dynanav_pct_scan.py`](../../../scripts/run_dynanav_pct_scan.py) → [`simulation/dynanav/pct_scan_worker.py`](../simulation/dynanav/pct_scan_worker.py)；下游是原生 `scan_planner_node`（vendored `third_party/scan`）、跟踪层 [`navigation/robogauge/route_tracker.py`](../navigation/robogauge/route_tracker.py) 与 [`simulation/dynanav/pct_scan_waypoints.py`](../simulation/dynanav/pct_scan_waypoints.py)；`exploration/` 只由 [`scripts/prepare_dynanav_waypoint_candidates.py`](../../../scripts/prepare_dynanav_waypoint_candidates.py)（可经 `scripts/dynanav_exploration.py candidates`）调用。
- 边界：本目录不 import `rclpy`/Isaac、不发布 `cmd_vel`、不判到达、不做 goto 准入。参考是否可执行、何时停车由 worker 与跟踪器决定；候选 JSON 是 advisory，绝不自动下发目标。

## 当前文件夹结构

```text
src/embodied_navigation/planning/
├── README.md          # 本文件
├── __init__.py        # 包 docstring（无重导出）
├── scan_handoff.py    # PCT→SCAN 参考交接状态机
├── scan_source.py     # vendored SCAN 源码 staged + 顺序打补丁
├── pct/               # 实测分层支撑 → 原生 PCT A*，见 pct/README.md
│   ├── README.md
│   └── measured_tomogram.py
└── exploration/       # Agent/操作者只读候选准备，见 exploration/README.md
    ├── README.md
    ├── __init__.py
    ├── os1_calibration.json
    ├── route_information.py
    └── waypoint_candidates.py
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `README.md` | 本文件：规划层定位、目录结构、文件职责与测试入口。 | — |
| `__init__.py` | 只有包 docstring `Global route planning (PCT) and autonomous exploration.`；无重导出，调用方必须写完整模块路径。 | — |
| `scan_handoff.py` | 一次 PCT 参考交接的纯状态机：只接受与本请求 `run_id`/`seq`、本子进程 PID、本 `nonce`、已实际发出的 body-source stamp 精确匹配的 ACK，记录 reference 发送、commit、失败原因与超时阶段；失败后不可复活。 | `ReferenceHandoff`（`sent_body_pose` / `consume_ack` / `sent_reference` / `matches_reference` / `accept_commit` / `receive_status` / `fail` / `check_timeout` / `accepts_candidate`）；`reference_waypoints(route, consumed_position)` 丢弃 PCT 起点（旧规划原点）并对重合点去重；ACK schema 常量 `CONSUMPTION_ACK_SCHEMA`/`ODOMETRY_ACK_SCHEMA`。 |
| `scan_source.py` | 在检出外新建构建目录，复制 `third_party/scan`，按文件名顺序用 `git apply --ignore-space-change --whitespace=nowarn` 打补丁；写 `source_stage.json`（输入/输出文件 sha256、补丁顺序、命令）与 `patches.log`；不改 vendored 树，也不复用旧构建目录。 | `stage_scan_source(vendor_root, build_root)`：构建目录必须在检出与 vendor 树之外，且必须是不存在的新目录；补丁失败保留现场并抛错。 |
| `pct/` | 实测分层支撑网格到 vendored 原生 PCT A* 的适配器（起点脚印 4 邻域接入、盲区平面模型、原生二进制缓存构建）。 | 见 [`pct/README.md`](pct/README.md)。 |
| `exploration/` | Agent/操作者的只读候选准备：实测端点快照、建模可见性打分、可选 frontier 聚类；无执行权限。 | 见 [`exploration/README.md`](exploration/README.md)。 |

## 交接状态机要点（读码核实）

- `ReferenceHandoff` 的 deadline 只覆盖子进程启动与 reference 响应，不覆盖路线执行；超时按阶段报 `native_scan_odometry_ack_timeout` / `native_scan_reference_reader_timeout` / `native_scan_reference_response_timeout`。
- `consume_ack()` 要求 schema、`run_id`、`planner_epoch`、`planner_pid`、`localization_generation`、`startup_nonce`、`source_frame_id`、`have_odom` 的类型与取值全部匹配，且 `source_stamp_ns` 必须是真正发出过的 stamp；迟到的更旧 stamp 被拒绝。
- `accept_commit()` 要求 `scan_route_definition/v1`、reference 内容 frame 一致、`committed_reference_revision` 为正整数；`receive_status()` 只接受 `scan_reference_status/v1` 的 `reference_rejected` / `target_abandoned`；`accepts_candidate()` 只放行与已 commit revision 相等的候选。
- 交接只记录状态与失败原因，实际话题发布在 [`simulation/dynanav/pct_scan_worker.py`](../simulation/dynanav/pct_scan_worker.py)。

## 验证与测试入口 / 配置注意

- 单测（离线 CPU，仓库根执行 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`）：
  - [`tests/unit/test_scan_source_staging_acceptance.py`](../../../tests/unit/test_scan_source_staging_acceptance.py)：复制逐字节一致、补丁文件名顺序与仓库相对路径、LF 补丁打到 CRLF 源码、拒绝复用/检出内目录、失败现场保全与 receipt 哈希（fixture [`tests/unit/scan_source_acceptance_fixtures.py`](../../../tests/unit/scan_source_acceptance_fixtures.py)）。
  - [`tests/unit/test_scan_handoff_worker_acceptance.py`](../../../tests/unit/test_scan_handoff_worker_acceptance.py)：通过真实 worker 回调黑盒覆盖 ACK 作用域、route definition 绑定、子进程静默/reference 静默超时、`target_abandoned`、cancel 关闭 owned child 等交接结果（fixture [`tests/unit/scan_worker_acceptance_fixtures.py`](../../../tests/unit/scan_worker_acceptance_fixtures.py)）。
  - PCT 与候选的单测见 [`pct/README.md`](pct/README.md) 与 [`exploration/README.md`](exploration/README.md)。
- 运行注意：
  - 默认入口 `bash scripts/run_dynanav_native_exploration.sh`；启动器先以 `--build-scan --build-only` 把 SCAN 闭包构建到 `--scan-prefix`（默认 `/tmp/dynanav-scan-native/install`）的外部目录，运行期 worker 再用 `source_receipt.json` 的 source sha 与当前 `third_party/scan` 逐文件摘要比对，stale 直接 `RuntimeError`。
  - SCAN 只能从当前 checkout 构建，兄弟目录的旧二进制会被拒绝；构建与运行产物都在外部目录/数据卷，不入 Git。
  - 交接状态机本身不读 `configs/`，阈值由 worker CLI 参数决定（`--reference-response-timeout-s` 默认 10 s）。
- 相关说明：[`docs/dynanav-pct-scan.md`](../../../docs/dynanav-pct-scan.md) 的“Data and execution path / Native limits”章节；vendored 算法与补丁规则见 [`third_party/README.md`](../../../third_party/README.md)。
