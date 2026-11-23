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
| **`exp/slam-v3-combined`** | `63d6975` | 融合 C++ 加速与全部三条正交分支、Codex `gpt-6.2-astra` 审计全量修正（`2fb09c1`），以及保旋转帧间点云 ICP 异常跳变修复 + 移动里程计边 Cauchy 鲁棒核（`63d6975`） | **784 OK / 60 OK** |

### 2.2 统一评测口径与数据集

所有基准使用 `scripts/eval_slam_backend.py`（base-origin 统一外参 `imu_in_base=[0.020153, 0.0, -0.005109]`，`dirty=False`，0 error）：
- **D1**：219 KFs（短程单层无回环，前端 `19.23 cm`）
- **D2**：1,567 KFs / 314 m（两层高回环覆盖，前端 `7.72 cm`）
- **D3**：876 KFs / 423 m（长程多层回环，前端 `20.85 cm`）
- **D4_twofloor**：1,916 KFs / 284 m（86 分钟实测两层探索，前端 Z 漂移达 `105.73 cm`）

结果归档目录：
- 审前正交对照：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261123-phase45-backend-v3/matrix/` 与 `combined_clean/`
- 审后基线（`2fb09c1`）：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261123-phase45-backend-v3/post_audit_final/`
- **含里程计异常 ICP 修复最终基准（`63d6975`）**：`$PROJECT_DATA_ROOT/embodied-navigation/runs/20261123-phase45-backend-v3/post_odom_repair/`

### 2.3 最终干净基准对照表（`exp/slam-v3-combined @ 63d6975`，单位：cm / deg）

| 数据集 | 方案 | Loops (Rep) | Online RMSE | Final RMSE (init) | Final Z | Final Max | Umeyama RMSE | Umeyama XY | Umeyama Max | Tilt RMSE |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **D1** | `stabilized` (基线) | 0 (0) | 10.22 | 9.97 | 6.31 | 20.59 | 6.69 | 5.69 | 12.10 | 1.01° |
| **D1** | `combined_lidar` / `fine` / `hybrid` | 0 (0) | **10.13** | **9.88** | **6.17** | **20.14** | **6.39** | 5.69 | **11.56** | 1.01° |
| **D2** | `stabilized` (基线) | 65 (0) | 8.80 | **5.54** | 3.80 | 14.22 | 4.54 | 3.33 | 16.96 | 0.37° |
| **D2** | **`combined_lidar`** | 51 (0) | 12.96 | 11.72 | 7.92 | 26.19 | **3.88** | **3.02** | **14.09** | **0.34°** |
| **D2** | `combined_lidar_fine` | 52 (0) | 12.56 | 11.78 | 8.93 | 26.45 | **3.98** | **3.02** | 14.31 | 0.37° |
| **D2** | `combined_hybrid` | 48 (0) | 20.58 | 21.05 | 14.66 | 46.40 | 5.10 | 3.79 | 20.47 | 0.43° |
| **D3** | `stabilized` (基线) | 166 (0) | 23.17 | 18.82 | 18.15 | 55.66 | 7.54 | 3.38 | 18.50 | 0.55° |
| **D3** | **`combined_lidar`** | 164 (1) | **9.72** (-58%) | **9.40** (-50%) | **8.21** | **24.11** | 5.97 | 3.78 | 16.79 | 0.45° |
| **D3** | **`combined_lidar_fine`** | 165 (1) | 13.30 | 10.33 | 8.99 | 26.80 | **5.33** (-29%) | **2.98** | **12.65** | **0.44°** |
| **D3** | `combined_hybrid` | 163 (1) | 18.35 | 14.59 | 13.93 | 40.35 | 6.21 | 3.29 | 15.24 | 0.46° |
| **D4** | `stabilized` (基线) | 82 (0) | 94.34 | 95.11 | 94.80 | 309.17 | 32.98 | 11.57 | 99.67 | 1.75° |
| **D4** | **`combined_lidar`** | 65 (3) | **13.00** (-86%) | **12.12** (-87%) | **10.93** | **28.43** (-91%) | **8.52** (-74%) | 3.65 | **20.80** (-79%) | 0.76° |
| **D4** | **`combined_lidar_fine`** | 69 (3) | 13.18 | 12.25 | 11.35 | 30.54 | **8.53** | **3.46** | **20.81** | 0.76° |
| **D4** | `combined_hybrid` | 58 (3) | 18.90 | 18.26 | 17.47 | 66.12 | 10.23 | 4.74 | 27.47 | **0.42°** |

### 2.4 关键工程发现与未决局限（诚实汇报）

1. **里程计异常 ICP 修复彻底消除 D4 脏边顶飞**：
   - 诊断发现 `D4` 原 `20.26 cm` / 峰值 `68.85 cm` 并非回环误配（65 条回环 0 跨层误配），而是 `FAST-LIVO2` 前端在平地上发生 3 次单步 `-40 cm` 垂直跳变（`kf 239→240→241` 与 `449→450`）后，非鲁棒 L2 里程计边在回环闭合时反将前段轨迹顶飞 `+68.8 cm`。
   - 启用保旋转（不改动短时精准 IMU 旋转 `R`）帧间点云 ICP 平移修复 + 移动里程计边 Cauchy 鲁棒核后，`D1`/`D2` 零误修（`rep=0`），`D4` 精准修复全部 3 处前端跳变，`FN` 从 `20.26 cm` 降至 **`12.12 cm`**（峰值 `68.85 → 28.43 cm`），`UM` 从 `17.28 cm` 降至 **`8.52 cm`**（`UM_XY = 3.46–3.65 cm`，峰值 `62.41 → 20.80 cm`）。
2. **后端无法改写前端内部状态的客观边界**：
   - 在当前松耦合架构下，后端可通过实测点云校验与位姿图实时修正 `T_world_odom` 并修复偶发前端跳变，但无法把修正后的位姿/地图回写进 `FAST-LIVO2` 内部的 IESKF 滤波器与 `ikd-Tree`。若前端自身持续退化或发散，仍需在前端侧定位并修复 `FAST-LIVO2` 垂直跳变根因（或引入后端→前端重置/紧耦合回灌）。
3. **其余未决项**：
   - 消融证实 `floor_normal_graph_sigma_rad = 0.02` 可将无初始倾角偏差的 `D3` 压至 `UM = 3.57 cm / Tilt = 0.19°`，后续需做 **`kf_0` 首帧地面法向初始化对齐** 以消除 `D2`/`D4` 的 `init_se3` 倾角杠杆；
   - ROS 在线 `third_party/fast_livo2/.../snapshot_bridge.cpp` 尚未写入配对 `visual.npz`（Astra P0-2），在线默认推荐先用 `slam_v3_combined_lidar_fine.json`。

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
