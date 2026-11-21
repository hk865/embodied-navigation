# 活跃分支测试与验收计划（TEST_PLANS）

> 盘点日期：2026-11-22 · 活体文档，与 [ROADMAP.md](ROADMAP.md)、[TASKS.md](TASKS.md)、[ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) 同步维护。协作与测试红线见 [AGENTS.md](../../AGENTS.md)。

本文件回答两件事：**每个活跃分支改了什么、合入 `main` 前必须过哪些离线与实机测试**，以及 **Phase 4 SLAM 后端各分支如何用同一份数据、同一时间轴、同一把打分尺横向对照**。

---

## 0. 所有分支通用的合入底线（强制门禁）

任何分支向 `main` 发起审阅/合并前，必须在仓库根通过：

```bash
git diff --check
PYTHONPATH=src python3 -B -m compileall -q src scripts
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

- **单测/回归失败名单相对起点不得新增**（`main` 基线 650 单测 / 60 回归；`exp/startup-walk-imu-bias` 当前 679 单测 / 60 回归，本机 3 条按环境 skip）。
- 严禁通过删除/放松旧断言、加 skip 或调大超时来换取通过。
- CPU 测试通过、进程退出码 0 **不等于**仿真或实机物理到达验收。

---

## 1. 活跃分支测试计划总览

| 分支 | 当前状态 | 已有测试覆盖 | 待执行测试 / 合并前缺口 |
| --- | --- | --- | --- |
| `main`（`2144e5f`） | Phase 3 已收口、与 `origin/main` 一致 | 650 单测 + 60 回归 + Phase 3 实机烟雾（`livo-frontend-smoke-01`、`livo-yaw-04`） | 仅作回归基线；合入新分支后重跑全量 CPU 测试与 1 次默认入口烟雾 |
| `exp/startup-walk-imu-bias` | 当前开发推进分支（领先 `main` 13+ 提交） | 679 单测 + 60 回归；起步稳态门/带噪 IMU/RGB8 图层/滑窗建图已实机跑录（`133734Z`、`142308Z`） | **`clearance_m=0.95 m` 与直通脱困 `maneuver` 尚未在 Isaac Sim 实机闭环验证**；`--backend-pose` 待后端修复后复测 |
| `exp/slam-backend-stabilization`（待切出） | Phase 4 核心修复（`B1/B2/B3`） | 尚未实现 | 先建 ADR-0004 三层评测器并跑出 `B0` 基线，再逐项消融 `B1/B2/B3/B1+B2+B3` + Codex 数学审计 + 实机回放复测 |
| `exp/slam-measured-submaps`（`5b300b0`） | 已过 Codex 审计，待统一评测 | `test_slam_measured_submaps.py`（110 行）+ `slam_measured_submaps.json` | 在 `D1/D2/D3` 统一离线缓存上做单变量对照（`query / history / anchor` 消融），补齐三层指标 |
| `exp/slam-surface-gate`（`958ec01`） | 已过 Codex 审计，待统一评测 | `test_slam_surface_gate.py`（204 行）+ `slam_surface_gate.json` | 在 `D1/D2/D3` 统一离线缓存上分别报告误配拦截率与漏配率，补齐三层指标 |
| `exp/slam-visual-retrieval`（`16883d1`） | 已过 Codex 审计，待统一评测 | `test_slam_visual_retrieval.py`（522 行）+ `slam_visual_retrieval.json` | 在 `D1/D2`（从 MCAP 抽关键帧 RGB）与 `D3/inputs/input_visual.json` 上测候选召回、视角不变特征匹配与几何验证后有效图边 |

---

## 2. `exp/startup-walk-imu-bias` 专项测试计划

本分支包含 5 块功能改动，合并 `main` 前按项核销（不能用其中一项的实机通过替代另一项）：

| # | 改动项（提交） | 离线 CPU 验证 | Isaac Sim 实机验证场景与判据 | 当前状态 |
| --- | --- | --- | --- | --- |
| 1 | 起步预热行走 + 稳态门 + 带噪 IMU（`ce3bac1`） | `tests/unit/test_dynanav_startup_gate_and_imu_noise.py` | 启用 `configs/simulation/dynanav/startup_walk_imu_bias_experiment.json`，确认 `startup_lifecycle.jsonl` 经过 `walk → stop_wait → settled → gate_open`，静止期无前端初始化炸散，且 `imu_bias_truth.jsonl` 落盘 | **已实机验证**（`133734Z`、`142308Z`） |
| 2 | RViz RGB8 彩色点云图层与录制 CLI（`6f79507`） | `compileall` + RViz 配置加载 | 实机运行期间 RViz `dynanav_lio_map` 正常显示彩色重建点云与后端轨迹图层 | **已实机验证**（`133734Z`、`142308Z`） |
| 3 | PCT/SCAN 局部滑窗 + 30 s 近期扫描恢复 + `--backend-pose`（`95fc675`、`62d5015`） | `tests/unit/test_pct_scan_worker_voxels.py` | ① `--truth-pose` + 滑窗不破坏爬楼与多房间；② 后端修复后再跑 `--backend-pose`，验证 `5 cm / 0.5°` 阈值内无高频体素重建抖动，且发生 `start_support_unobserved` 时近期扫描恢复生效 | 滑窗已验证；**`--backend-pose` 待 Phase 4 后端修复后复测** |
| 4 | PCT 默认头顶净空 `0.65 m → 0.95 m`（`04ceede`） | `tests/unit/test_measured_pct_tomogram.py`（49 条） | 在地下室楼梯下方楔形低矮区复现 `135406Z` 场景：PCT 不再规划穿过楼梯底楔形区的路线，且正常上/下楼梯路线不受误杀 | **仅 CPU 通过，待实机回归** |
| 5 | 不依赖 PCT/SCAN 的直通脱困 `maneuver`（`2f64d8d`） | `tests/unit/test_dynanav_pct_scan_waypoints.py` | 实机在窄门角或贴墙静止位下发 `explore_waypoint_client.py maneuver --vx -0.25 --wz 0.6 --duration 1.2 --wait`，确认不再报 `unsupported_action`，收到 `maneuver_accepted` → `maneuver_complete`，车身真实退离障碍且 `stop` 后归零 | **仅 CPU 通过，待实机验证** |

> 注：Phase 4 后端优化优先推进期间，暂不专门重开 Isaac Sim 录制；待后端 `B1+B2+B3` 离线评测收口后，安排一次短时实机回归把第 3/4/5 项一次性核销。

---

## 3. Phase 4 统一离线评测数据集与打分尺（ADR-0004）

所有 Phase 4 后端分支（稳定化修复 + 三条回环对照）**必须**在同一套数据集与同一套评测口径下输出可复核结果，禁止各说各话或换数据集挑优。

### 3.1 统一数据集矩阵（全部位于 `$PROJECT_DATA_ROOT/embodied-navigation/runs/`）

| 编号 | 数据路径 | 规模与传感器 | 关键特性 / 必测 Badcase |
| --- | --- | --- | --- |
| **D1** | `20261121-noisy-imu-rgb-twofloor-explore/20261121T133734Z/` | 414.5 s；`map/` 219 关键帧；真值流 + 带噪 IMU + 13 话题 MCAP（≈9.3 GB） | **双失效黄金回归包**：① `t≈211 s` 无重力约束 6-DoF ICP 将 Roll/Pitch 拉歪 `6°–8.3°`、Z 跳变 `+22 cm`（前端误差接近 0）；② `t=261–362 s` 贴白墙长停滞诱发前端 `-43 cm` Z 发散 |
| **D2** | `20261121-noisy-imu-rgb-twofloor-explore/20261121T142308Z/` | 2770 s（46 min）；双层 16 区域 `70 m × 27 m`；`map/` 1,567 关键帧；真值流 + 带噪 IMU + 13 话题 MCAP（≈80 GB） | **超长双层漂移与过密回环包**：前端自然漂移（`dx≈-0.86 m, dz≈-0.24 m`），旧后端 407+ 过密回环把二楼 Z 拽偏至 **`-0.74 m`**（典型负优化） |
| **D3** | `20261119-hba-project-comparison/same-876/` | `inputs/input.json` + `inputs/input_visual.json` + `inputs/gravity.json` + `inputs/images/`（876 关键帧） | **历史口径对照包**：保留与 2026-11-19/20 旧版及外部离线基线 `hku-mars/HBA` 的同输入对比能力 |

### 3.1.1 `D3`（`same-876`）已有历史基准快照（2026-11-20 实测，仅终图 ATE）

> 数据源：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261120-slam-phase4-comparison/{same-876,same-876-visual-features}/manifest.json` 与 `20261118-hba-offline/REPORT.zh-CN.md`。本轮仅统计终图 ATE（尚未跑 ADR-0004 在线因果曲线与逐秒分布，也未在带噪 IMU 双层包 `D1/D2` 上验证），作为 Phase 4 新评测的回归参照：

| 方案（分支） | 配置摘要 | 回环数 | 终图 ATE RMSE (Max) | 终图 Z RMSE (上层 Z) | 耗时 |
| --- | --- | ---: | ---: | ---: | ---: |
| 原前端（FAST-LIVO2） | 无后端 | 0 | `0.2395 m` | `0.2383 m` (`0.3396 m`) | — |
| 外部基线 HBA | `hku-mars/HBA` 3 层离线 BA | — | `0.2947 m` | `0.2906 m` (`0.4014 m`) | `11.8 s` |
| `main_aniso`（`main` `9816a2b`） | 默认后端 + 各向异性权重（`loop_sigma_rad=0.03 / yaw=0.01 / cauchy=3.0`） | 329 | `0.1692 m` (`0.4724 m`) | `0.1520 m` (`0.1889 m`) | `346.3 s` |
| `measured_aniso`（`exp/slam-measured-submaps` `5b300b0`） | 查询+历史 5 帧实测子图 + 各向异性权重 | 324 | `0.1008 m` (`0.2840 m`) | `0.0930 m` (`0.1149 m`) | `326.0 s` |
| `surface_aniso`（`exp/slam-surface-gate` `958ec01`） | 平面/表面覆盖门控 + 各向异性权重 | 318 | `0.1651 m` (`0.4716 m`) | `0.1545 m` (`0.1918 m`) | `525.1 s` |
| `visual_akaze`（`exp/slam-visual-retrieval` `16883d1`） | ScanContext + AKAZE 视觉补充 | 345 | `0.1899 m` (`0.5361 m`) | `0.1834 m` (`0.2265 m`) | `397.9 s` |
| `visual_akaze_only`（`exp/slam-visual-retrieval` `16883d1`） | 仅 AKAZE 视觉候选（关 ScanContext）+ 各向异性权重 | 107 | `0.0827 m` (`0.2628 m`) | `0.0620 m` (`0.0700 m`) | `93.3 s` |

### 3.2 离线关键帧缓存原则

- 几何后端（`B0–B3`、`measured-submaps`、`surface-gate`）直接读取 `map/map.json` + `map/kf_*.npz` + `capture/truth_scoring_stream.jsonl`（及 `capture/imu_stream.jsonl` / `interactive_waypoint_control.jsonl`），**严禁在调参循环中重复扫描 80 GB 原始 MCAP**。
- 视觉检索分支（`exp/slam-visual-retrieval`）一次性从 `D1/D2` 的 MCAP 按关键帧最近邻时间戳抽取缩略图/灰度帧缓存到 `$PROJECT_DATA_ROOT/embodied-navigation/cache/slam-backend-phase4/`，随后离线复用。

### 3.3 三层评测指标（遵循 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)）

每次评测统一按两种对齐（① 首帧 SE(3) 对齐，反映绝对重力/朝向漂移；② Umeyama SE(3) 对齐，反映纯轨迹形状精度）同时评估 **在线因果轨迹**（每帧刚发布时的 `T_world_body`，决定导航控制）与 **最终优化轨迹**（最后一次图优化后全部关键帧，决定建图质量）：

1. **第一层（标量汇总）**：
   - `ATE`（3D / XY / Z 的 RMSE、Mean、P95、Max）
   - `AOE`（总旋转角及 Roll / Pitch / Yaw 的 RMSE、Mean、P95、Max）
   - `RTE` / `ROE`（滑窗 `5 m / 10 m / 20 m` 及 `1 s / 5 s / 10 s` 的 RMSE、Max）
   - 运行统计：关键帧数、候选数、通过数、接受率、总优化次数、单帧/总耗时。
2. **第二层（逐关键帧演化曲线）**：
   - 随关键帧序号 $k = 1 \dots N$ 的前缀子轨迹 `ATE(k) / AOE(k)`（Mean & Max）及每次回环触发后的即时前后跳变 $\Delta\text{ATE}$。
3. **第三层（逐秒历史位姿修订分布）**：
   - 按 1 s 时间桶统计每个历史位姿在整个回放过程中被后端反复修改的分布（中位数、IQR、P5–P95、Max、累计修订抖动量），直观暴露“反复拉扯/负优化”。
4. **标准导出**：导出 TUM 格式轨迹（真值、前端、在线后端、终图后端），便于第三方 `evo` 复核。

---

## 4. Phase 4 后端稳定化主线测试计划（`exp/slam-backend-stabilization`）

基于统一评测器，先跑出当前代码 `B0`（未改后端）在 `D1/D2/D3` 上的完整基线，再通过显式配置 overlay（**严禁环境变量开关**）做严格消融：

| 配置代号 | 变量定义 | 核心验证目标 | 预期通过判据（相对 `B0`） |
| --- | --- | --- | --- |
| **B0** | 当前后端默认实现（基线） | 量化复现 `D1` 倾角失真与 `D2` Z 轴负优化 | 完整落盘三层指标与曲线，验证 `D1` 出现大 Roll/Pitch 偏斜、`D2` 终图 Z 劣于前端 |
| **B1** | + 原地/近邻过密回环抑制（空间/时间间隔与去重门） | 消除 `D1/D2` 静止或慢速区域重复闭合同一局部区域导致的图边过权重 | `D2` 回环密度显著下降（避免 400+ 同区冗余边），无单区过度拉扯 |
| **B2** | + 重力 Roll/Pitch 强先验（ICP 线性化先验 + 位姿图重力因子） | 锁死水平面，禁止无纹理走廊/窄角 6-DoF ICP 扭歪俯仰/横滚 | `D1` 全程 Roll/Pitch Max 从 `6°–8.3°` 降至 **`< 1.5°`**；`D2` 终图 Z RMSE 优于前端与 `B0` |
| **B3** | + 静止/Agent 思考停驻期 ZUPT 停车锚定约束 | 抑制机器人静止等待期间相对位姿漂移与无效关键帧堆积 | `D1` 静止段（`t=261–362 s`）后端相对位姿增量近零，不产生虚假漂移边 |
| **B1+B2+B3** | 三项组合 | 验证三者叠加无冲突、正向增益稳定 | ① `D1` Roll/Pitch Max `< 1.5°` 且在线/终图 ATE 均优于前端和 `B0`；② `D2` 在线与终图 3D/Z ATE 均显著优于 `B0` 且**不劣于前端（杜绝负优化）**；③ `D3` 不出现回归；④ 经 Codex 独立数学/假设审计并新增对应单测 |

---

## 5. Phase 4 三条独立回环对照分支测试计划

三条分支均基于 `9816a2b` 切出且已完成 Codex 初轮审计，**在 `B1+B2+B3` 稳定基线明确前后**，分别在 `D1/D2/D3` 同一输入上跑出单变量对照，汇总进 `experiments/slam-backend-phase4/REPORT.md`（≤ 300 行）：

| 分支 | 对应配置与单测 | 专项消融项 | 必报指标与淘汰/晋升条件 |
| --- | --- | --- | --- |
| `exp/slam-measured-submaps`（`5b300b0`） | `configs/localization_mapping/slam_measured_submaps.json` + `test_slam_measured_submaps.py` | 仅单帧 vs 历史子图 vs 查询+历史子图（锚点帧对齐策略） | 报告配准内点率、回环接受率、三层 ATE/AOE/RTE/ROE 与单次 ICP 耗时；若仅增耗时无精度收益或增大误配则保留结论不合入 |
| `exp/slam-surface-gate`（`958ec01`） | `configs/localization_mapping/slam_surface_gate.json` + `test_slam_surface_gate.py` | 关门控 vs 开平面/表面覆盖门控 | **必须分开报告误配拦截率（FP 减少）与好回环漏配率（FN 增加）**，核验窄角白墙（`D1`）是否被正确拒收且二楼大厅/楼梯口（`D2`）未被误杀 |
| `exp/slam-visual-retrieval`（`16883d1`） | `configs/localization_mapping/slam_visual_retrieval.json` + `test_slam_visual_retrieval.py` | 仅 ScanContext vs 视觉检索补充候选（ORB/AKAZE/BRISK/SIFT + 几何验证） | 报告实际进入几何验证的视觉候选数、真正成边的数量、反向/跨视角回环召回增益（`D2` 双层折返与 `D3/input_visual.json`）及帧耗时 |

---

## 6. 产物归档与晋升决策规则

- **评测产物位置**：所有离线评测 JSON、NPZ、TUM 轨迹与图表一律写入 `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261122-slam-backend-phase4/<run-id>/`，附 `manifest.json`（记录分支、commit、脏树差异、冻结配置、数据集 SHA/路径）。
- **仓库结论报告**：横向评测完成后，在仓库内写不超过 300 行的结论报告 `experiments/slam-backend-phase4/REPORT.md`，只写可复核数字、收益、失败案例与适用场景。
- **合并原则**：出现回归、缺少同输入可比证据、或收益不足以覆盖复杂度时，**保留分支与结论并停止推广**；达到判据且通过 Codex 审计后，由用户审阅 diff 并执行合并与推送。
