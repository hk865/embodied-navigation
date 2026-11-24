# SLAM 零基础入门与前后端源码精讲教程

> **整理日期**：2026-11-24
> **对应代码基线**：前端 `third_party/fast_livo2/` + Phase 4 稳定化后端底座 `exp/slam-backend-stabilization`（`2b77a5d`）、三条正交 v2 分支（`exp/slam-v2-*`）、v3 统一组合与里程计修复主线（`exp/slam-v3-combined@63d6975`）、v4 世界重力校准与在线 Matrix-Free PCG 定向 3D Planar BA（`exp/slam-v4-gravity-calibration@e459e67` → `exp/slam-v4-oriented-planar-ba` 含统一在线目标函数去抖动与室外配置）以及前后端滑窗、去重全局图与防穿墙多机位录制分支（`exp/live-v4-ba-sliding-window`）。
> **阅读说明**：本目录为面向 SLAM 初学者的体系化源码与公式讲义，不参与运行时；如需查看最新评测总表，请结合 [docs/plan/ROADMAP.md](../plan/ROADMAP.md)、[experiments/slam-backend-v3/REPORT.md](../../experiments/slam-backend-v3/REPORT.md) 阅读。

## 推荐阅读顺序

| 序号 | 文档 | 核心内容 |
| --- | --- | --- |
| 01 | [01_slam_basics_and_fast_livo2_frontend.md](01_slam_basics_and_fast_livo2_frontend.md) | SLAM 零基础 5 大核心概念、FAST-LIVO2 前端完整 ROS 回调与线程数据流图、19 维状态与 ESIKF/VIO 伪代码、C++ 库 API、YAML 参数与 `snapshot_bridge` 交接机制 |
| 02 | [02_backend_base_and_stabilization.md](02_backend_base_and_stabilization.md) | `BackendNode` 双线程数据流图、公共底座（`ScanContext` / 点对面 `ICP` / 稀疏 `LM` 位姿图 / `CorrectionAdmission`）与稳定化三大手术（`B1` 节流、`B2` 重力先验与倾角门、`B3` 纯传感器 ZUPT）的源码、数据结构、伪代码与调参表 |
| 03 | [03_branch_v2_measured_submaps.md](03_branch_v2_measured_submaps.md) | 正交分支一 `exp/slam-v2-measured-submaps`：短窗前端相对里程计拼多帧实测子图、6 道连续性截断刹车、`anchor_mode="candidate"` 锚点闭环、`max_gap_s` 踩坑复盘与 D2/D3 对比 |
| 04 | [04_branch_v2_surface_gate.md](04_branch_v2_surface_gate.md) | 正交分支二 `exp/slam-v2-surface-gate`：重力对齐系下 PCA 局部平面提取、6 邻域 DFS 区域生长、水平/竖直覆盖率门控、ZUPT 体素解耦修复与严格/柔和参数对比 |
| 05 | [05_branch_v2_visual_retrieval_and_comparison.md](05_branch_v2_visual_retrieval_and_comparison.md) | 正交分支三 `exp/slam-v2-visual-retrieval`（OpenCV `AKAZE`/`BFMatcher` + 实测 LiDAR 3D 最近深度绑定 + 随机投影聚合 + 1 mm 去重 Kabsch RANSAC）、ADR-0004 三层评测体系速通与五方案横向对照 |
| 06 | [06_branch_v3_combined_and_odom_repair.md](06_branch_v3_combined_and_odom_repair.md) | V3 统一主线（`combined_lidar` / `combined_lidar_fine` / `combined_hybrid`）八大优化精讲、C++17 加速、宽幅地面共面调平、保旋转帧间 ICP 异常跳变修复（`63d6975`）、全分支 D1–D4 横向对照与后续跟踪清单 |
| 07 | [07_branch_v4_gravity_and_online_planar_ba.md](07_branch_v4_gravity_and_online_planar_ba.md) | V4 突破主线：`kf_0` 实测地面法向世界重力校准、法向定向 3D Planar BA（防薄楼板/隔墙压扁）、Matrix-Free PCG 722× 提速、统一在线目标函数去抖动修复（`343.7 m` 两层实机 `online 2.68 cm / final 2.02 cm`）、以及真实室外 R3LIVE + M3DGR 验证与室内外配置分流 |
