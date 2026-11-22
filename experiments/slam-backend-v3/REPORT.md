# SLAM Backend V3 Multi-Branch Optimization & Overnight Validation Report (2026-11-23)

本报告记录 2026-11-22 至 2026-11-23 夜间自主推进闭环的全部量化结果，涵盖：
- **Phase A**：Visual-First 2D 占据图 + `--backend-pose` 实机/仿真两层长程自主探索与 13 路全量录制（数据集 **`D4_twofloor`**）；
- **Phase B**：C++17/Eigen3 热点加速 + 3 条正交方向后端分支 + 统一集成 `exp/slam-v3-combined` + Codex `gpt-6.2-astra` 独立审计与审后干净基准复测；
- **Phase C**：语义线 A（`feat/semantic-keyframe-graph`）与具身线 B（`feat/agent-preemptable-active-slam`）及其各自 Codex `gpt-6.2-astra` 独立审计与审后修复。

所有运行产物均保存在 `$PROJECT_DATA_ROOT/embodied-navigation/runs/`，仓库内不提交任何日志、点云或大文件。

---

## 1. Phase A：两层实机/仿真自主探索与录制（`D4_twofloor`）

- **分支与提交**：`feat/stage1-exploration-and-backend-wiring` (`ce523f9`)，单测 735 OK、回归 60 OK。
- **运行目录**：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261123-stage1-twofloor/stage1_visual_backend_twofloor_20261123T002000Z/`
- **实测结果**：
  - 完整自主运行 **86 分钟**（仿真时间 `3577.8 s`），行进 **`283.8 m`**，执行 `61` 个目标，0 崩溃，正常退出（`done.rc = 0`）。
  - 在线稳定后端（`slam_backend_stabilized.json` + `--backend-pose`）累计录入 **`1,916` 关键帧**、闭环 **`78` 次**、ZUPT 锚定 **`435` 次**，`last_error: null`。
  - 抵达二楼平台（`sim_s = 1556.2 s, 133.6 m`）时在线位姿相对真值仅 **`ΔZ = +6.8 cm, ΔYaw = -0.23°, 3D = 9.2 cm`**（对照原始 LIO 前端 `-103.8 cm` 垂直下沉）。
  - 完整归档：`map/`（1,916 关键帧）、`capture/truth_scoring_stream.jsonl`（715,560 行 200 Hz 真值）、`capture/interactive_frames/`（1,789 帧 RGB PNG）、双段完整 rosbag（13 路含 RGB8 彩色点云）、离线视觉缓存 `$PROJECT_DATA_ROOT/embodied-navigation/cache/slam-backend-phase4/D4_visual/`（1,916/1,916 全匹配）。
- **前端/RViz 内存根因定位**：`dynanav_live_pointcloud.rviz` 对两路 10 Hz 点云设置 `Decay Time: 3600` 导致 RViz2 内存膨胀至 `26.1 GB`（`FAST-LIVO2` 前端进程本身仅常规占用）；后续应由后端发布去重降采样全局图并令 RViz `Decay Time: 0`。

---

## 2. Phase B：多分支 SLAM 后端优化与统一基准（`D1 / D2 / D3 / D4_twofloor`）

### 2.1 分支与架构清单

| 分支 | 关键提交 | 职责与核心改动 | 单测 / 回归 |
|---|---|---|---|
| `perf/slam-backend-cpp-icp` | `ffd7882` | C++17/Eigen3 热路径加速（体素降采样、point-to-plane ICP + Fisher Hessian、稀疏 SE(3) 位姿图 LM），与 Python 比特级一致（`D2` 242s→88s、`D4` 308s→107s） | 739 OK / 60 OK |
| `exp/slam-v3-plane-factors` | `e1ff826` | 机身倾角无关宽幅地面法向检测（精度 `0.04°–0.13°`）、纯 Z 共面里程计调平（不扰动 yaw）、饱和式 X-ICP 退化调制 | 743 OK / 60 OK |
| `exp/slam-v3-surface-submap-gate` | `ed54adc` | 运动有界实测子图（`window=2, span<=4s, yaw<=35°, candidate` 锚点）+ 软平面覆盖门控 + X-ICP + 细体素 ICP | 747 OK / 60 OK |
| `exp/slam-v3-hybrid-color-visual` | `1c3fb8c` | `ColorScanContext` + 多视角视觉检索窗口 + SeqSLAM 时序一致 + 航向门控混合准入（同向 `<65°` 视觉核验、反向 `>=65°` 360° LiDAR） | 753 OK / 60 OK |
| **`exp/slam-v3-combined`** | `2fb09c1` | 融合 C++ 加速与全部三条正交分支，并完成 Codex `gpt-6.2-astra` 审计全部 3 P0 / 16 P1 / 8 P2 修正 | **781 OK / 60 OK** |

### 2.2 统一评测口径与数据集

所有基准使用 `scripts/eval_slam_backend.py`（base-origin 统一外参 `imu_in_base=[0.020153, 0.0, -0.005109]`，`dirty=False`，0 error）：
- **D1**：219 KFs（短程单层无回环，前端 `19.23 cm`）
- **D2**：1,567 KFs / 314 m（两层高回环覆盖，前端 `7.72 cm`）
- **D3**：876 KFs / 423 m（长程多层回环，前端 `20.85 cm`）
- **D4_twofloor**：1,916 KFs / 284 m（86 分钟实测两层探索，前端 Z 漂移达 `105.73 cm`）

结果归档目录：
- 审前正交对照：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261123-phase45-backend-v3/matrix/` 与 `combined_clean/`
- **审后最终干净基准（`2fb09c1`）**：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261123-phase45-backend-v3/post_audit_final/`

### 2.3 审后最终干净基准对照表（`exp/slam-v3-combined @ 2fb09c1`，单位：cm / deg）

| 数据集 | 方案 | Loops | Online RMSE | Final RMSE (init) | Final Z | Final Max | Umeyama RMSE | Umeyama Max | Tilt RMSE |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **D1** | `stabilized` (基线) | 0 | 10.22 | 9.97 | 6.31 | 20.59 | 6.69 | 12.10 | 1.01° |
| **D1** | `combined_lidar` / `fine` / `hybrid` | 0 | **10.13** | **9.88** | **6.17** | **20.14** | **6.39** | **11.56** | 1.01° |
| **D2** | `stabilized` (基线) | 65 | 8.80 | **5.54** | 3.80 | 14.22 | 4.54 | 16.96 | 0.37° |
| **D2** | **`combined_lidar`** | 51 | 12.96 | 11.72 | 7.92 | 26.19 | **3.88** | **14.09** | **0.34°** |
| **D2** | `combined_lidar_fine` | 52 | 12.56 | 11.78 | 8.93 | 26.45 | **3.98** | 14.31 | 0.37° |
| **D2** | `combined_hybrid` | 48 | 20.58 | 21.05 | 14.66 | 46.40 | 5.10 | 20.47 | 0.43° |
| **D3** | `stabilized` (基线) | 166 | 23.17 | 18.82 | 18.15 | 55.66 | 7.54 | 18.50 | 0.55° |
| **D3** | **`combined_lidar`** | 164 | **9.71** (-58%) | **7.76** (-59%) | **6.21** | **20.41** | 5.39 | 16.04 | 0.45° |
| **D3** | **`combined_lidar_fine`** | 165 | 13.31 | 8.18 | 6.58 | 22.43 | **4.73** (-37%) | **12.69** | **0.44°** |
| **D3** | `combined_hybrid` | 163 | 18.41 | 11.86 | 11.03 | 35.64 | 5.74 | 14.22 | 0.47° |
| **D4** | `stabilized` (基线) | 82 | 94.34 | 95.11 | 94.80 | 309.17 | 32.98 | 99.67 | 1.75° |
| **D4** | **`combined_lidar`** | 65 | **17.62** (-81%) | **20.26** (-79%) | **19.70** | **68.85** | 17.28 (-48%) | 62.41 | 0.80° |
| **D4** | **`combined_lidar_fine`** | 67 | 19.26 | 20.39 | 19.86 | 68.14 | 17.12 | **60.19** | **0.72°** |
| **D4** | `combined_hybrid` | 59 | 17.96 | 20.63 | 19.90 | 70.03 | **17.05** | 61.67 | 0.77° |

### 2.4 审后关键工程发现与未决局限（诚实汇报）

1. **审后精度提升与物理规律**：
   - 落实 Astra P1-1（机体系原点精确 Fisher Hessian）与 P1-6（查询/目标子图严格不共享源帧）后，`D3 combined_lidar` 从 `10.20 cm` 进一步降至 **`7.76 cm`**（`fine` `UM = 4.73 cm`），`D4 combined_lidar` 从 `52.16 cm` 大幅降至 **`20.26 cm`**（线上 **`17.62 cm`，-81.3%**），`D2 combined_lidar` 全局刚体 `UM` 降至 **`3.88 cm`**（`Z = 2.43 cm`，已进入 2–4 cm 目标区间）。
   - 消融探测证实：`floor_normal_graph_sigma_rad = 0.02` 可将无初始倾角偏差的 `D3` 压至 **`UM = 3.57 cm / Tilt = 0.19°`**，但对存在初始倾角偏置的 `D4` 会与固定锚点 `kf_0` 产生杠杆冲突；因此预设保持 `floor_normal_graph_sigma_rad: null` 为默认安全值，后续应通过 **`kf_0` 首帧地面法向初始化对齐** 彻底统一 `init_se3` 与 `Umeyama` 精度。
   - **D4 远翼开环上限**：`D4_twofloor` 二楼远翼存在长段无回环开环探索，即使开启共面调平与表面门控，开环段残余 Z 误差仍约 `17–20 cm`；需结合具身线 B 的 **Active-SLAM 主动回环** 缩短无回环行程。
2. **线上混合视觉链路缺口（Astra P0-2）**：
   - `exp/slam-v3-combined` 已在离线与 `VisualObservation` 边界完成全部视觉/彩色点云混合逻辑；但 ROS 在线 `third_party/fast_livo2/.../snapshot_bridge.cpp` 尚未写入配对 `visual.npz`（需单独 C++ 补丁与原生编译），因此在线默认推荐先用 `slam_v3_combined_lidar.json` 或 `slam_v3_combined_lidar_fine.json`。

---

## 3. Phase C：语义线 A 与具身线 B（均已通过 Codex `gpt-6.2-astra` 审计与审后修复）

| 分支 | 审后提交 | 核心交付与审后修复要点 | 单测 / 回归 |
|---|---|---|---|
| **`feat/semantic-keyframe-graph`** | `46e0b48` | 关键帧锚定 3D 场景图（楼层/楼梯/房间/门洞/对象）+ 后端回环原子双图同步适配器 + 流式 CLI（55 行）。审后修复全部 8 P1 + 10 P2：平台纵坡过滤（`D4` 1,916 帧精确输出 `2 层 1 楼梯 11 房间 10 门洞`，消除漂移假第 3 层）、Z 平移不变 NMS、跨 epoch 断层隔离、瓶颈锚点重投影与幂等合并、防穿墙 36 扇区 LOS + `cKDTree`（10,000 帧仅 `0.416 s`）、流式读取峰值 RSS `214 MB → 74.8 MB`、外参合成与 README 同步 | **752 OK / 60 OK** |
| **`feat/agent-preemptable-active-slam`** | `8aebc95` | IMU 阻尼 PD+条件抗饱和 I 转向控制（独立 `yaw` 可配置至 `0.55–0.80 rad/s`，贯通 `capture/control.py` 断言，严格保持 `goto/maneuver` `0.15 rad/s`）+ `fcntl.flock` 原子收件箱队列与严格 `action_epoch` 抢占隔离 + 协作中止 `pct_astar` 子进程 + 真 SSE 流式早停哨兵客户端（适配 `deepseek-flash` 禁思考、`qwen3.8-flash` 禁思考与双图 VLM、`openai_compatible`）+ `sentinel_coordinator.py` 闭环协调器。`scripts/explore_waypoint_client.py` 反向精简 350 行（2,314→1,964 行）。审后修复全部 1 P0 + 11 P1 + 6 P2 | **770 OK / 60 OK** |

---

## 4. 未验证范围与下一步建议

- **本机未验证项**：Phase B 审后与 Phase C 均未重新拉起 Isaac Sim 在线实机闭环，亦未调用收费云端 DeepSeek/Qwen API（SSE 流式早停、超时中止、抢占闭环均由本地 `ThreadingHTTPServer` 与真实 `NativeWaypointTracker` 确定性测试覆盖）。
- **建议按序推进**：
  1. 逐支审阅并按需合入：`perf/slam-backend-cpp-icp` → `exp/slam-v3-combined` → `feat/semantic-keyframe-graph` / `feat/agent-preemptable-active-slam`；
  2. 在 Isaac Sim 实机联调中先启用 `slam_v3_combined_lidar_fine.json` + 快速独立 `yaw`（`0.55 rad/s`）+ `sentinel_coordinator` 抢占队列验证端到端响应延迟；
  3. 针对 `kf_0` 首帧地面法向初始化对齐与 `snapshot_bridge.cpp` 在线视觉 spool 补丁分别立小分支攻关。
