# Phase 4 SLAM 后端统一三层评测、新基线确立与三路正交叠加报告（2026-11-22）

- **新基线分支**：`exp/slam-backend-stabilization`（核心代码提交 `9ec9837`，基于 `main` `2144e5f` 干净切出）
- **三路正交叠加分支**（均从 `9ec9837` 独立切出）：
  - `exp/slam-v2-measured-submaps`（`57f8d68`）
  - `exp/slam-v2-surface-gate`（`35db0de`）
  - `exp/slam-v2-visual-retrieval`（`6b600dc`）
- **评测器**：ADR-0004 三层离线评测器（`src/embodied_navigation/localization_mapping/backend/evaluation.py` + `scripts/eval_slam_backend.py`）
- **数据盘产物根**：
  - 基线与历史分支：`runs_base = $PROJECT_DATA_ROOT/embodied-navigation/runs/20261122-slam-backend-phase4/`
  - 正交叠加与重力消融：`runs_v2 = $PROJECT_DATA_ROOT/embodied-navigation/runs/20261122-slam-backend-phase4-v2/`
- **验证边界**：本机 CPU 离线回放与全量测试通过（`exp/slam-backend-stabilization` 688 单测 / 60 回归 0 失败；三路 v2 各自 93 / 95 / 97 条 SLAM 单测全部通过）；**未运行 Isaac Sim / ROS 2 实时联调**。

---

## 1. 新统一评测基线（3 数据集 × 3 层指标）

| 代号 | 场景 | KFs / 行程 / 时长 | 原始前端（`Init-SE3`） | 数据源（`$PROJECT_DATA_ROOT/embodied-navigation/` 下） |
| --- | --- | --- | --- | --- |
| **D1** | 一楼窄转角双故障（长停+原地转向+垂向漂移） | `219` / `16.3 m` / `401 s` | ATE `0.1950` (max `0.460`), Z `0.1917`, Tilt max `3.50°` | `runs/20261121-noisy-imu-rgb-twofloor-explore/20261121T133734Z/` |
| **D2** | 46 分钟带噪 IMU 两层 16 房间长程探索 | `1567` / `313.6 m` / `2757 s` | ATE `0.0876` (max `0.251`), Z `0.0708`, Tilt max `1.48°` | `runs/20261121-noisy-imu-rgb-twofloor-explore/20261121T142308Z/` |
| **D3** | 两层返航闭环 `same-876`（离线 HBA `0.2947 m`） | `876` / `422.7 m` / `1318 s` | ATE `0.2106` (max `0.650`), Z `0.2073`, Tilt max `1.48°` | `runs/20261119-hba-project-comparison/same-876/inputs/` |

配置说明（全部通过 `configs/localization_mapping/*.json` 控制，零环境变量开关）：
- `slam_backend_stabilized.json`（**推荐新默认基线**）：`B0_aniso` + `B1 (travel=5 m, spacing=0.5 m)` + `B2 (icp_gravity=0.02, max_loop_tilt=0.06, graph_gravity=null)` + `B3 (zupt_enabled=true, full_stop=0)`。
- `slam_backend_stabilized_graph_grav.json`：同上但额外开启逐节点位姿图一元重力先验（`graph_gravity=0.02`），用于对比逐节点重力锁的影响。
- `slam_backend_stabilized_sparse.json`：稀疏回环探索配置（`spacing=1.5 m, interval=4.0 s, graph_gravity=0.02`）。

---

## 2. 全量对照结果（统一 `Init-SE3` 主口径；每行绑定确切 run 目录）

### 2.1 D1（一楼窄转角双故障，前端 ATE `0.1950 m` / Z `0.1917 m` / Tilt max `3.50°`）

| 方案 / 分支 | 归档目录 | 回环 | 耗时 | Online ATE (max) | Final ATE (max) | Final Z | Tilt max | L3 `jmax` |
| --- | --- | ---: | ---: | --- | --- | ---: | ---: | ---: |
| `B0_default`（旧默认） | `runs_base/D1_B0_default` | 161 | 27.8 s | `0.2195 (0.373)` | `0.2143 (0.320)` | `0.1731` | **`8.40°`** | `0.136` |
| `B0_aniso` | `runs_base/D1_B0_aniso` | 162 | 39.4 s | `0.1832 (0.323)` | `0.1788 (0.282)` | `0.1329` | `4.00°` | `0.151` |
| 旧 `exp/slam-measured-submaps` | `runs_base/D1_measured_submaps` | 163 | 31.8 s | `0.1185 (0.193)` | `0.1197 (0.179)` | `0.0572` | `3.51°` | `0.108` |
| 旧 `exp/slam-surface-gate` | `runs_base/D1_surface_gate` | 78 | 29.7 s | `0.1413 (0.253)` | `0.1475 (0.214)` | `0.0241` | `2.24°` | `0.147` |
| 旧 `exp/slam-visual-retrieval` (`akaze_only`) | `runs_base/D1_visual_akaze_only` | 14 | 6.9 s | `0.1974 (0.465)` | `0.1974 (0.465)` | `0.1947` | `3.52°` | `0.005` |
| **新基线 `stabilized`（`9ec9837` / `icp_grav`）** | `runs_v2/D1_stabilized_icpgrav_only` | **0** | **3.6 s** | **`0.1011 (0.315)`** | **`0.0985 (0.194)`** | **`0.0633`** | **`3.50°`** | **`0.000`** |
| **全部三路 `v2` 分支（均继承 B1+B3）** | `runs_v2/D1_v2_*` | **0** | **3.0–5.8 s** | **`0.1011 (0.315)`** | **`0.0985 (0.194)`** | **`0.0633`** | **`3.50°`** | **`0.000`** |

### 2.2 D2（46 分钟 2F 16 房间带噪 IMU，前端 ATE `0.0876 m` / Z `0.0708 m` / Umeyama `0.0479 m` / Tilt `1.48°`）

| 方案 / 分支 | 归档目录 | 回环 | 耗时 | Online ATE | Final ATE (max) | Final Z | Umeyama (Z) | Tilt RMSE/max | L3 `IQR95 / jmax` |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- | --- | --- |
| `B0_default`（旧默认） | `runs_base/D2_B0_default` | 713 | 584.1 s | `0.2826` | `0.2142 (0.447)` | `0.2100` | `0.0605 (0.0472)` | `— / 1.82°` | `0.260 / 0.605` |
| `B0_aniso` | `runs_base/D2_B0_aniso` | 709 | 624.4 s | `0.1462` | `0.0890 (0.200)` | `0.0787` | `0.0446 (0.0250)` | `— / 1.78°` | `0.211 / 0.434` |
| 旧 `exp/slam-measured-submaps` | `runs_base/D2_measured_submaps` | 717 | 642.3 s | `0.2827` | `0.2206 (0.530)` | `0.1459` | `0.0652 (0.0415)` | `— / 1.47°` | `0.065 / 0.694` |
| 旧 `exp/slam-surface-gate` | `runs_base/D2_surface_gate` | 237 | 488.3 s | `0.1577` | `0.1565 (0.386)` | `0.1191` | `0.0622 (0.0392)` | `— / 1.60°` | `0.001 / 0.357` |
| 旧 `exp/slam-visual-retrieval` (`akaze_only`) | `runs_base/D2_visual_akaze_only` | 160 | 116.1 s | `0.2025` | `0.2060 (0.470)` | `0.1974` | `0.0796 (0.0707)` | `— / 1.40°` | `0.000 / 0.160` |
| `stabilized` + 图重力锁（`9ec9837`） | `runs_base/D2_9ec9837_stabilized` | 65 | 231.6 s | `0.1125` | `0.0895 (0.204)` | `0.0674` | `0.0500 (0.0329)` | `0.38° / 1.50°` | `0.058 / 0.251` |
| `stabilized_sparse` + 图重力锁（`9ec9837`） | `runs_base/D2_9ec9837_stabilized_sparse` | 29 | 213.8 s | `0.0900` | `0.0795 (0.211)` | `0.0663` | `0.0510 (0.0346)` | `0.37° / 1.48°` | `0.037 / 0.156` |
| **新基线 `stabilized`（`icp_grav`, 无图锁）** | `runs_v2/D2_stabilized_icpgrav_only` | **65** | **232.4 s** | `0.0949` | **`0.0674 (0.176)`** | **`0.0489`** | **`0.0465 (0.0291)`** | **`0.26° / 1.61°`** | `0.055 / 0.247` |
| `v2-measured-submaps`（`spacing=0.5`, 图锁） | `runs_v2/D2_v2_measured_submaps` | 67 | 230.3 s | `0.2966` | `0.2851 (0.556)` | `0.0628` | `0.0732 (0.0321)` | `0.37° / 1.53°` | `0.039 / 0.171` |
| **`v2-measured-submaps`（`sparse=1.5`, 图锁）** | `runs_v2/D2_v2_measured_submaps_sparse` | **28** | **210.9 s** | **`0.0784`** | **`0.0741 (0.192)`** | `0.0626` | **`0.0456 (0.0339)`** | `0.37° / 1.50°` | **`0.019 / 0.117`** |
| `v2-surface-gate`（严格 `cov=0.6`, 图锁） | `runs_v2/D2_v2_surface_gate` | 19 | 278.0 s | `0.0990` | `0.0986 (0.205)` | `0.0659` | `0.0524 (0.0356)` | `0.38° / 1.48°` | `0.017 / 0.161` |
| `v2-surface-gate`（柔和 `cov=0.35`, 图锁） | `runs_v2/D2_v2_surface_gate_soft` | 58 | 275.4 s | `0.1014` | `0.0822 (0.190)` | `0.0668` | `0.0494 (0.0331)` | `0.38° / 1.49°` | `0.049 / 0.226` |
| `v2-visual`（`akaze_only`, `spacing=0.5/0.0`） | `runs_v2/D2_v2_visual_akaze_only*` | 11 / 24 | 51–61 s | `0.151 / 0.141` | `0.144 / 0.135` | `0.072 / 0.076` | `0.106 / 0.094` | `0.37° / 1.58°` | `0.002 / 0.168` |
| `v2-visual`（融合 `SC+AKAZE`, 图锁） | `runs_v2/D2_v2_visual_akaze` | 66 | 271.2 s | `0.1200` | `0.0765 (0.181)` | `0.0625` | `0.0484 (0.0320)` | `0.38° / 1.50°` | `0.099 / 0.244` |
| **`v2-visual`（融合 `SC+AKAZE`, `icp_grav`）** | `runs_v2/D2_v2_visual_akaze_icpgrav` | **67** | **266.7 s** | `0.1106` | **`0.0670 (0.174)`** | **`0.0475`** | **`0.0472 (0.0304)`** | **`0.26° / 1.62°`** | `0.081 / 0.271` |

### 2.3 D3（两层返航 `same-876`，前端 ATE `0.2106 m` / Z `0.2073 m` / Tilt `0.54°`；HBA `0.2947 m`）

| 方案 / 分支 | 归档目录 | 回环 | 耗时 | Online ATE | Final ATE (max) | Final Z | Umeyama (Z) | L3 `IQR95 / jmax` |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- | --- |
| `B0_aniso` | `runs_base/D3_b0_aniso` | 329 | 350.7 s | `0.2026` | `0.1609 (0.454)` | `0.1479` | `0.0771 (0.0662)` | `0.152 / 2.675` |
| 旧 `exp/slam-measured-submaps` | `runs_base/D3_measured_submaps` | 325 | 325.6 s | `0.0957` | `0.1006 (0.292)` | `0.0923` | `0.0660 (0.0538)` | `0.070 / 1.826` |
| 旧 `exp/slam-visual-retrieval` (`akaze_only`) | `runs_base/D3_visual_akaze_only` | 106 | 92.3 s | `0.0911` | `0.0826 (0.191)` | `0.0620` | `0.0655 (0.0487)` | `0.067 / 2.199` |
| `stabilized` + 图重力锁（`9ec9837`） | `runs_base/D3_9ec9837_stabilized` | 166 | 257.8 s | `0.2162` | `0.1970 (0.608)` | `0.1897` | `0.0714 (0.0586)` | **`0.025 / 0.712`** |
| **新基线 `stabilized`（`icp_grav`, 无图锁）** | `runs_v2/D3_stabilized_icpgrav_only` | **166** | **266.4 s** | `0.2392` | **`0.1905 (0.570)`** | **`0.1819`** | `0.0742 (0.0625)` | `0.112 / 1.374` |
| `v2-measured-submaps`（图锁） | `runs_v2/D3_v2_measured_submaps` | 160 | 248.9 s | `0.2379` | `0.2361 (0.698)` | `0.2319` | `0.0789 (0.0689)` | `0.025 / 0.811` |
| **`v2-measured-submaps`（`icp_grav`, 无图锁）** | `runs_v2/D3_v2_measured_submaps_icpgrav` | **160** | **245.4 s** | **`0.1989`** | **`0.1714 (0.519)`** | **`0.1643`** | **`0.0693 (0.0583)`** | `0.075 / 1.124` |
| `v2-surface-gate`（严格 `cov=0.6`, 图锁） | `runs_v2/D3_v2_surface_gate` | 155 | 377.7 s | `0.1987` | `0.1813 (0.560)` | `0.1735` | `0.0707 (0.0545)` | **`0.015 / 0.794`** |
| `v2-visual`（`akaze_only`, `spacing=0.0`, 图锁） | `runs_v2/D3_v2_visual_akaze_only_nospacing` | 115 | 119.7 s | `0.2110` | `0.2065 (0.641)` | `0.1996` | `0.0750 (0.0587)` | **`0.008 / 0.513`** |
| **`v2-visual`（`akaze_only`, `icp_grav`, 无图锁）** | `runs_v2/D3_v2_visual_akaze_only_icpgrav` | **115** | **109.1 s** | **`0.1732`** | **`0.1629 (0.542)`** | **`0.1519`** | `0.0779 (0.0621)` | **`0.020 / 0.949`** |
| `v2-visual`（融合 `SC+AKAZE`, `icp_grav`） | `runs_v2/D3_v2_visual_akaze_icpgrav` | 177 | 299.9 s | `0.2296` | `0.2200 (0.640)` | `0.2136` | `0.0856 (0.0726)` | `0.054 / 1.228` |

---

## 3. 核心科学发现：正交性与真实耦合机理

1. **最大发现——逐节点位姿图一元重力锁 vs 仅回环 ICP 重力先验**：
   - 当对位姿图**所有 876~1567 个节点**施加 `graph_gravity_sigma_rad=0.02` 时，上千个并联一元因子会把整条轨迹的 Roll/Pitch 刚性锁死在前端带偏置的斜率（`0.38°–0.54°`）上，导致回环边无法压平长程 Z 漂移。
   - **改为仅在回环 ICP 加重力先验（`icp_gravity_sigma_rad=0.02`）+ 候选倾角门（`max_loop_tilt_rad=0.06`）、置 `graph_gravity_sigma_rad=null`** 后，既彻底堵死了倾斜坏回环，又释放了位姿图平滑吸收 0.3°–0.5° 俯仰偏置的自由度：
     - 纯稳定化新基线在 `D2` 上直接从 `0.0895 m` 降至 **`0.0674 m`**（`Final Z 0.0489 m`、`Umeyama Z 2.91 cm`、`Tilt RMSE 0.26°` 甚至平于前端 `0.38°`！）；
     - `v2-visual (akaze_only)` 在 `D3` 上从 `0.2065 m` 降至 **`0.1629 m`**；`v2-measured-submaps` 在 `D3` 上从 `0.2361 m` 降至 **`0.1714 m`**。
2. **三路方向并非完全无耦合，存在 3 处实测边界**：
   - **`visual-retrieval`**：纯 `akaze_only` 在 `D2` 低纹理白墙走廊偏稀（仅 11–24 条回环，`0.135–0.144 m`）；**融合模式 `SC+AKAZE` + `icp_grav`** 在 `D2` 刷新全场最优 **`0.0670 m (Z 0.0475 m)`**，但在 `D3` 仍引入了少量噪声边（`0.2200 m`）。
   - **`measured-submaps`**：对回环间距极度敏感——在 `D2` 上 `sparse (1.5 m)` 达到 **`Online 0.0784 m / Final 0.0741 m / jmax 0.117 m`**（全场唯一在线 ATE 也低于前端的方案！），但 `spacing=0.5 m` 混入早期 1 条 Yaw 偏斜边导致 `Init-SE3` 劣化至 `0.2851 m`。
   - **`surface-gate`**：严格版（`cov=0.6`）与 `B1` 叠加在 `D2` 上过度滤边（19 条，`0.0986 m`）；**柔和安全阀版（`cov=0.35`）**在 `D2` 筛掉 7 条最差回环将终图压到 `0.0822 m`，但单帧开销增加约 +45 s。
3. **在线精度取舍（诚实披露）**：
   - 推荐默认基线 `slam_backend_stabilized.json`（`icp_grav`）在 `D1 / D2 / D3` 三套数据集的 **Final ATE（`0.0985 / 0.0674 / 0.1905 m`）、Final Z 与最大误差上全部显著优于原始前端**；但在 `Online ATE` 上，`D2`（`0.0949 m` vs 前端 `0.0876 m`）与 `D3`（`0.2392 m` vs 前端 `0.2106 m`）仍略高于前端（来自回环采纳延迟与单次大闭环前的未闭合段），后续通过 `measured-submaps (sparse)` 与低延迟 C++ 后端进一步压缩在线误差。

---

## 4. 新基线确立与下一步分支路线

1. **立即确立新基线**：建议用户审核推送后，将 `exp/slam-backend-stabilization`（含 ADR-0004 三层评测器、`B1+B2+B3` 与 `slam_backend_stabilized.json`）作为新的 SLAM 后端基线；所有历史旧分支（`exp/slam-measured-submaps`、`exp/slam-surface-gate`、`exp/slam-visual-retrieval`）均因在 `D2` 严重负优化不再直接合入。
2. **三路正交 `v2` 分支已各自独立落盘**：`exp/slam-v2-measured-submaps`（`57f8d68`）、`exp/slam-v2-surface-gate`（`35db0de`）、`exp/slam-v2-visual-retrieval`（`6b600dc`）均已基于新基线构建完毕，后续优先在 `v2-measured-submaps` 排查 `D2` `spacing=0.5 m` 的单条 Yaw 坏边（或与柔和 `surface-gate` 组合），并在 Linux 主机 Isaac Sim + ROS 2 环境验证实机在线闭环。
