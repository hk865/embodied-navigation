# 活体任务板（TASKS）

> 盘点日期：2026-11-28 · 集成基线 `main` `2144e5f`（= `origin/main`） · 主力记忆/Agent 分支 `feat/static-memory-streaming`（文档基线 `a7fd71a` / 代码基线 `e159c8a`，948 单测 / 60 回归全绿） · D 路径 V4 闭环分支 `exp/live-v4-ba-sliding-window`（`aada1f1`） · 冻结基线与边界见 [BASELINES.md](BASELINES.md) · 阶段目标与完成判据见 [ROADMAP.md](ROADMAP.md)，最新权衡与 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) 见 [3D/4D 表征 Trade-Off](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)，协作规则见 [AGENTS.md](../../AGENTS.md)。

## 使用说明

- 每次会话开始先读 [ROADMAP.md](ROADMAP.md) 的阶段状态，再在本文件勾选/追加任务；完成的任务移到「已完成」，不要删除（保留决策痕迹）。
- 角色约定：**主控** = 架构判断与破坏性重构；**DSH** = 低成本批量执行；**Codex** = 独立审核与前提查错；**Flash** = 边界清晰的功能开发 / 单测。
- 每条任务必须有可执行验收标准；「进程退出 0」「CPU 测试通过」不能单独作为导航验收。
- 状态标记：`[ ]` 未开始 / `[~]` 进行中 / `[x]` 已完成。

## 已完成

| 任务 | 角色 | 对应模块 | 验收结果 |
| --- | --- | --- | --- |
| 干净底座重构（P1+P2 死代码清理、A 路径退役、动态行人移出主线） | 主控 + DSH | 全仓库 | `refactor/06-clean-base`（`f061a7e`），累计删减 >8.2 万行，646 单测 / 60 回归 0 失败 |
| 第一优先真值闭环验证（真值位姿 + 200 Hz 真值去畸变 RTX） | 主控 | 规划 / 仿真 | `exp/pct-scan-stairs-truth` 合入 `bb2eef2`：地下室→二楼大厅 +3.25 m，零撞墙/扶手、零 SCAN 失败 |
| 通用 3D PCT+SCAN 替换楼梯特化规则 | 主控 | `planning/` + `third_party/scan` | `reference_z_profile.patch` 上包络 + PCT 起点脚印邻域实测支撑，零楼梯特化规则 |
| SLAM 公共接口初步规范化 | 主控 + Codex | `localization_mapping/backend/` + `third_party/fast_livo2/patches/` | `0011-imu-bias-covariance-params.patch`、`anisotropic_information`、`BackendConfig.from_dict/from_json`、`node.py --config`，落 `5b880b2` |
| 全仓库计划与模块文档重建 | DSH | 根 `README.md`、`docs/plan/`、22 份模块 `README.md` | 所有相对链接可达；模块 README 逐文件表与 `git ls-files` 一致；无退役文件名；27 份 `.md` 独立脚本校验 0 错 |
| 默认 FAST-LIVO2 前端 + PCT/SCAN + Agent 探索实机烟雾测试 | 主控 | `scripts/run_dynanav_native_exploration.sh`、`simulation/dynanav/`、`navigation/` | `runs/20261120-main-smoke/livo-frontend-smoke-01` 通过：`observe` 正常，平地 + 180° 折返爬楼梯（+0.53 m）均 `waypoint_reached`，`capture_exit=0`，零撞墙/扶手与躯干碰撞 |
| 修复 LIVO 模式下独立 `yaw` 动作 `rotation_space_stale` + 接入 `0011-imu-bias-covariance-params.patch` 原生构建链 | 主控 | `simulation/dynanav/pct_scan_worker.py`、`third_party/scan/patches/z_grid_map_latency.patch`、`localization_mapping/fast_livo2_source.py`、`docker/{build_native,verify_native_sources}.py`、`scripts/build_dynanav_fast_livo2.py` | 6 项 `dynanav_*` CTest + `yaw_consumption` + 13/13 `raycast_regression` 全绿；`runs/20261120-yaw-and-imu-fix/livo-yaw-04` 在默认 live FAST-LIVO2 下完成独立 `yaw +180°`（`yaw_complete`，误差 `-0.036 rad`，连续驻留 `0.52 s`，未放松 `max_sim_age_s=0.25 s`）+ `goto 1.5 m`（`waypoint_reached`），`capture_exit=0` |
| `fix/livo-yaw-and-imu-patch` 审阅并合入 `main` | 用户 | 仓库 | merge `9b6dc85` 合入 `5049ec7`；`main` = `origin/main` = `2144e5f`；650 单测 / 60 回归 0 失败 |
| Phase 4 三条后端对照分支初版切出 + Codex 审计 | 主控 + Codex | `localization_mapping/backend/` | `exp/slam-measured-submaps`（`5b300b0`）、`exp/slam-surface-gate`（`958ec01`）、`exp/slam-visual-retrieval`（`16883d1`）；均基于 `9816a2b` 切出、只带本方向最小改动，审计修复提交已落各自分支 |
| `exp/startup-walk-imu-bias` 实机闭环修复（12 个提交） | 主控 + Codex / DSH | `simulation/dynanav/`、`navigation/`、`planning/`、`configs/` | 起步预热稳态门 + 物理级带噪 IMU（`ce3bac1`）、RViz RGB8 彩色重建图层（`6f79507`）、局部滑窗建图 / 近期扫描恢复 / `--backend-pose` 订阅（`95fc675`）、PCT 净空 0.65→0.95 m（`04ceede`）、不依赖 PCT/SCAN 的直通脱困 `maneuver`（`2f64d8d`）；当前工作树 679 单测 0 失败 |
| 架构决策记录 ADR-0003 / ADR-0004 | 主控 | `docs/plan/adr/` | `8ed74c2`（流式 Fork + 关键帧语义图 + 停驻期 ZUPT）、`9e3408d`（增量 Mesh/3DGS + 三层后端评测口径）；链接已同步全部入口文档 |
| 两份最新带噪 IMU 13 话题黄金数据集录制 | 主控 + Codex | `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261121-noisy-imu-rgb-twofloor-explore/` | `20261121T133734Z`（一楼窄角前后端双失效，≈9.3 GB）与 `20261121T142308Z`（双层 46 分钟 70 m × 27 m，≈80 GB）；13 话题、真值流与 `manifest.json` 齐备，详见 [ROADMAP 第 5 节](ROADMAP.md) |
| ADR-0004 三层离线评测器 + Phase 4 后端稳定化（`B1+B2+B3`） | 主控 + Codex | `localization_mapping/backend/`、`scripts/eval_slam_backend.py`、`configs/localization_mapping/` | `exp/slam-backend-stabilization`（`faf9cbf` → `9ec9837` → `a27f186` → `2b77a5d`）；实现三层评测（含机身原点转换、同前缀 Layer 2、入图起点 Layer 3）、B1 节流、B2 精确 MAP 重力先验与倾角门、B3 纯传感器 ZUPT；698 单测 / 60 回归全绿 |
| GPT-6.2-Astra（非 fast）独立全量审计与四分支 P0/P1/P2 修复 | Codex + 主控 | `pose_graph.py`、`registration.py`、`backend.py`、`correction.py`、`evaluation.py`、`node.py`、`map_store.py`、`visual.py` | 修复全部 4 项 P0（IMU/机身评分原点、位姿图精确 $\mathcal{J}_r^{-1}(e)$、同前缀回环收益、视觉在线 spool 加载与存图）及 ZUPT 体素解耦、`candidate` 锚点、3D 去重、ICP 质心居中等 P1/P2，四分支单测全绿 |
| 三路正交 v2 分支重切 + `D1/D2/D3` 30/30 组干净对照与完整文档交付 | 主控 | `exp/slam-v2-{measured-submaps,surface-gate,visual-retrieval}`、[REPORT.md](../../experiments/slam-backend-phase4/REPORT.md)、[slam-backend-branches-guide.md](../design/slam-backend-branches-guide.md) | 30 组全量 `dirty=False` 机身原点对照归档于 `runs/20261122-slam-backend-phase4-clean/`；D1 `0.0997 m`、D2 `stabilized` `0.0554 m` / `v2_sg_soft` `0.0608 m`、D3 `v2_ms` `0.1126–0.1268 m` / `v2_visual_akaze_only` `0.0825 m (84.1 s)` |
| ADR-0002 / 0003 / 0004 论文开源调研与数据盘只读清理台账 | 主控 | [adr-0002-0003-0004-survey.md](research/adr-0002-0003-0004-survey.md) | 覆盖 InternNav/NaVILA/VLFM/SGLang/3D-Mem/HOV-SG/Hydra/ImMesh/Kimera-PGMO/DUFOMap/Khronos/PGSR-2DGS、P0–P3 分支路线与 ~199 GB 只读清理台账 |
| V4 在线定向 Planar BA + 前端/PCT 滑窗 + 物体级 3D Surface Nets 网格记忆 + 教程文档 | 主控 + Codex | `localization_mapping/backend/`、`semantic_mapping/object_mesh_memory.py`、`docs/slam-tutorial/`、`docs/agent-tutorial/` | `exp/live-v4-ba-sliding-window`（`fb35dd8`）：1h 51m / 343.72 m 两层闭环（Umeyama 终图 `2.02 cm` / 在线 `2.68 cm`，`max_cum_jitter` `13.67 → 1.52 m`），42.7 s 产出 145 物体彩色网格与 88 裁剪图，807 单测 / 60 回归全绿 |
| 2026-11 七层冻结基线、学术创新查重、VLN 基准 Tricks 核查与三轴演进总纲 | 主控 + Codex | [BASELINES.md](BASELINES.md)、[2026-11-novelty-audit-and-directions.md](research/2026-11-novelty-audit-and-directions.md)、[2026-11-vln-perception-memory-strategy-synthesis.md](research/2026-11-vln-perception-memory-strategy-synthesis.md) | 冻结 L1–L7 内部基线与 16 GB 显存红线；完成 SuperNav/ASENA/HarnessVLN/VLN-PE/HA-VLN 2.0/MemoryAgentBench/VSI-Bench 与上游 DynaNav 85 题审计；收敛为「感知 + 记忆 + 失败驱动控制」三轴主线与两条 2 周止损探针 |
| L7 独立 `agent/` 运行时 + 常驻视觉服务 + 三态熔断 + `SharedHabitatSceneHost` 并发 (`agent-v4`) | 主控 + Flash + Codex | `src/embodied_navigation/agent/`、`localization_mapping/semantic_mapping/`、`perception/vision_tools.py` | `feat/static-memory-streaming`（`e159c8a`，**948 单测 / 60 回归全绿**）；5 场景 HM3D Dev（16 有效子任务，1.8 m，单目 90° RGB-D，GT 位姿）跑出 `SR 81.25% (13/16)`（95% Wilson CI `57%–93%`）、`OSR 87.50% (14/16)`、Sub 2–4 `81.82% (9/11)`、KV `96.40%`、压缩 `79.54%`、`0` 超时 |
| 3D/4D 表征与混合检索 Trade-Off、2026.1–2026.11 文献校准、`ADR-0005` 与 `gpt-6.2-astra` 深度评审 | 主控 + Codex | [2026-11-3d-representation-memory-reconstruction-tradeoffs.md](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)、[ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)、[ROADMAP.md](ROADMAP.md) | 核实 `CGFM-Nav` (`36 ep/278` vs 全量 `360 ep/2,669`)、`SuperNav` (`120 题 1 m/0.25 m/学习执行器`)、`GOAT nav.py` 实现；定稿 4 项跨层因果契约、3 天 Mesh 证伪实验、11 分支推进清单与 Colab `1,000 CU` 封顶预算 |
| 单目虚拟双目与 Object-Centric 光度追踪数学严审 + 3 主线/4D 8 分支分工解构 | 主控 + Codex | [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)、[ROADMAP.md](ROADMAP.md)、[TASKS.md](TASKS.md) | 放弃纯单目三角化硬算深度，拍板 `FAST-LIVO2 + 10 Hz LiDAR 静中剥动 + 30 Hz KF/IMM + 切向光度可拒绝修正`；取消独立几何动态危险模型并瘦身流式监控为事件驱动仲裁器；核实并登记 2 处现存流式抢占 Bug（`llm_agent_client.py` 判旧、`native_waypoints.py` `stop` 语义） |

## 进行中（用户审阅与实机收口）

| 状态 | 任务 | 角色 | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[~]` | 审阅并推送 Phase 4 五个本地分支与 `feat/static-memory-streaming`（`e159c8a`）等领先提交 | 用户 | 仓库 | 逐行读 diff + 确认 30 组报告/评测归档后由用户执行合并/推送 |
| `[~]` | 审批数据盘 Tier 1（~100.4 GB）/ Tier 2（~99.5 GB）清理清单 | 用户 | `$PROJECT_DATA_ROOT/embodied-navigation/` | 用户针对具体对象确认后人工触发删除，释放空间用于下一轮完整双层探索录制 |

## 待办队列（Phase 5 · 4 阶段 × 8 分支施工顺序与 Phase 4 实机收口）

| 状态 | 任务 | 角色 | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[ ]` | **【阶段 1 · 分支 3 `feat/memory-hybrid-store-build`】** 统一记忆核心数据结构（`observation_id` + 16-bit 压缩深度 PNG + 里程计过程链 + 推理先验 + `pose_revision`）与 Mesh/点云混合底座，并做 2–3 天低成本 Mesh 遮挡重排验证（基于 `H`，提取 `46e0b48`） | 主控 + Flash | `localization_mapping/semantic_mapping/`、`agent/adapters/habitat_daemon.py` | 同一回放注入重复消息、迟到视觉结果与 1 次回环后输出一致可溯源实例历史，前缀零未来泄漏；Habitat 保留帧同步落盘 16-bit 压缩深度；遮挡子实验达标（Top-1 ≥ +5 pp、p95 ≤ 50 ms）才启用过滤 |
| `[ ]` | **【阶段 1 · 分支 1 `feat/agent-habitat-benchmarks`】** 整理 360° 观测与记忆工具，优化全局/局部地图喂入，对接官方 `HM3D / OVON / GOAT` Scorer 并按需校准轻量状态/`STOP` 判别头（基于 `H` + Colab 批跑） | 主控 | `agent/`、`planning/exploration/habitat_memory_runner.py` | 输出官方 Scorer 逐题对齐报告（成功位、`geodesic SPL`、分母对账），双列 `1.0 m / 1.8 m` 与 Dev/盲测拆报；OVON/GOAT 官方分净提升 |
| `[ ]` | **【阶段 2 · 分支 4 `feat/memory-tree-graph-retrieval`】** 落地围绕里程计 + 回环 + 推理先验的层次树与四类边时空过程图检索及回环增量修改，并用 Colab 训冻结 `SigLIP` 同屋实例重排头（基于分支 `3`） | 主控 + Codex | `localization_mapping/semantic_mapping/`、`agent/tools/` | 外观库按 `observation_id` 建库（严禁拼世界坐标）且回环零重算；GOAT 同起点「持久 vs 清空」配对 $\Delta\text{SR}/\Delta\text{SPL}$ 净增益；未见房 Top-1 ≥ +5 pp |
| `[ ]` | **【阶段 2 · 分支 5 `exp/recon-3d-mesh-assets`】** 基于统一定型底账离线派生转连通 Mesh、`Hunyuan3D` 物体补全生成、Blender/UE5/Isaac 游戏 PBR 材质素材库与 `2DGS` 对照（基于分支 `3` / `D`，离线旁路） | 主控 + Flash | `localization_mapping/semantic_mapping/object_mesh_memory.py`、`scripts/build_object_mesh_memory.py` | 适配单卡 16 GB 离线串行复用；相对原生 USD 评估 3D IoU/Chamfer/侵入率；严守离线边界，零在线帧率拖累 |
| `[ ]` | **【阶段 2 · 分支 2 `feat/agent-streaming-monitor`】** 实现瘦身事件驱动流式监控（取消独立动态危险模型，消费 `DynamicOccupancy / SceneDelta`，仅临近岔口或歧义按需 Fork）、修复 `llm_agent_client.py` 与 `native_waypoints.py` 2 处抢占 Bug，并新增 `dynanav_adapter.py` 接通 Isaac Sim 静态两层（基于分支 `1` + 提取 `8aebc95`） | 主控 + Codex | `agent/adapters/dynanav_adapter.py`、`planning/exploration/llm_agent_client.py`、`simulation/dynanav/native_waypoints.py` | 慢返回（推理 > 抽帧间隔）下仍能正常触发有效抢占；中途打断重规划发 `cancel`/`preempt` 后可续跑不锁死；Habitat 与 Isaac Sim 静态两层共用 L7 运行时 |
| `[ ]` | **【阶段 3 · 分支 6 `exp/slam-dynamic-and-generalize`】** 推进多模态互补动态 SLAM：`FAST-LIVO2` 静态底座 + 10 Hz LiDAR 静中剥动（逐点时戳对齐）+ 30 Hz KF/IMM 外推 + 切向光度可拒绝修正，并完成室外坡道/其余场景泛化收尾（基于 `D` + `ref/dynanav-dynamic-pedestrian`，按需 `ffd7882`） | 主控 + Codex | `perception/`、`localization_mapping/`、`third_party/fast_livo2/` | 同一输入消融 `LiDAR+KF` vs `加切向光度`，记录横向/径向误差、动态残影、自由空间误清率与静态 2 cm 基准零退化；32 线束间空白禁清空占据；`cmd_vel` 禁当里程计 |
| `[ ]` | **【阶段 4 · 分支 7 `feat/4d-object-seg-tracking`】** 在动态 SLAM 站稳后用 `observation_id` 串联短时 `track_id` 与长期 `instance_id`，完成 4D 物体分割、跨帧跟踪与遮挡补全，并按需延伸刚体/铰接/分段骨架（基于分支 `6 + 3` + 可选 Colab） | 主控 + Codex | `localization_mapping/semantic_mapping/`、`perception/` | 同一 `pose_revision` 下先扣静态背景位移，显式区分 `not_visible / observed_empty_at_old_pose / identity_swapped`；旧址误访相对降 ≥ 25% 且不增错删 |
| `[ ]` | **【阶段 4 · 分支 8 `feat/4d-memory-and-embodied-loop`】** 将 4D 分割跟踪接入记忆查改与事件驱动流式监控（动态堵路/家具搬移/回环即时改记忆并撤销过期航点），在 Isaac Sim 动态多层环境跑通全闭环（汇合 `2 + 4 + 7`） | 主控 + Codex | `agent/`、`localization_mapping/`、`planning/`、`navigation/` | 动态多层真物理闭环零静态错判误取消；回环机体系相对净空与前置 `STOP` 校验通过；PCT/SCAN + MoE-CTS 安全门独立兜底 |
| `[ ]` | 将推荐后端配置显式冻结接入 `scripts/run_dynanav_native_exploration.sh` 并完成 Isaac Sim / ROS 2 实跑验收 | 主控 | `scripts/run_dynanav_native_exploration.sh`、`backend/node.py` | 启动器显式传 `--config` 并冻结进 run 目录；核验实机回环峰值耗时下 ROS 发布时延、视觉 spool 在线入库与先验地图重载、以及 `clearance_m=0.95 m` / `maneuver` 脱困 |
| `[ ]` | 探索后端精度向 `2–4 cm` 进阶（航向门控混合准入 + 精配准细体素/平面因子 + X-ICP 退化感知协方差 / STD-BTC） | 主控 + Codex | `localization_mapping/backend/` | 解决 D2/D3 XY 精度保护（不劣化前端 2–3 cm XY）与 D3 拱形 Z 漂移；在 `D1/D2/D3` 统一机身原点口径下较现有 30 组基线显著提升 |
| `[ ]` | 排查并优化前端起步重力/俯仰初值收敛（消除 D3 初窗 ~0.5° 俯仰偏置贡献的 ~14 cm Z 杆臂误差） | 主控 + Codex | `third_party/fast_livo2/`、`simulation/dynanav/` | D3 前端 Init-SE3 Z RMSE 与 Umeyama Z RMSE 差距明显收窄，不引入静态起步发散 |
| `[ ]` | 后端模块文档补充板载部署/线程模型说明 | DSH | `localization_mapping/backend/README.md` | 与 `node.py --config` 实际参数一致；无过期文件引用 |
| `[ ]` | 持续维护：每次改动同步根 README 与 `docs/plan/` | 全体 | `docs/` | 盘点日期更新；`git diff --check` 通过；链接可达 |

## 架构决策记录（ADR）索引

| ADR | 标题 | 状态 | 与任务的关系 |
| --- | --- | --- | --- |
| [ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md) | LIVO 独立 `yaw` 清阻塞与 ≤ 0.2 s 点云融合重叠 | 已采纳（2026-11-20） | `yaw` 修复已完成并合入 `main`；远期触发项见「远期 / 触发式」 |
| [ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md) | 具身 Agent 异步行动、语义/动态地图与热路径演进备忘 | 提案 / 远期（2026-11-20） | 流式控制、语义图与 C++ 热路径下沉的触发条件 |
| [ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md) | 单主干流式 Fork 异步控制与关键帧锚定语义图片图数据库 | 提案 / 2026-11-28 按 ADR-0005 瘦身 | System 1 已按 ADR-0005 瘦身为事件驱动任务仲裁器（取消独立几何动态危险模型，并发/停止两处现存 Bug 留分支 `2` 对齐） |
| [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) | 面向 FAST-LIVO2 的增量 Mesh / 3DGS 演进与三层 SLAM 后端评测口径 | 提案 / 评测规范（2026-11-21） | Phase 4 首项评测器与三项后端修复的唯一打分尺 |
| [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md) | 因果关键帧记忆、Mesh+骨架混合底座、树/图过程检索与 3 主线 + 4D 8 分支规划 | 已定稿 · 2026-11-28 修订 | 3 主线 + 4D 叠加 4 阶段 8 分支施工表、4 条跨主线契约、动态前端切向光度拍板与 Colab `1,000 CU` 预算 |

## 远期 / 触发式架构优化（关联 ADR，达触发条件时立项）

| 状态 | 任务 | 触发条件 / ADR | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[ ]` | 评估将 `pct_scan_worker` 文件邮箱替换为低延迟直连 IPC / ROS Action，并排查 FAST-LIVO2 200 ms 成对突发发包节奏 | 需 >95% `yaw` 占空比、高动态追踪或高频抢占（见 [ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md)） | `simulation/dynanav/pct_scan_worker.py`、`native_waypoints.py`、`third_party/fast_livo2/` | 端到端控制延迟显著下降且不放松 `max_sim_age_s=0.25 s` |
| `[ ]` | 恢复动态行人时重审独立 `yaw` 的 0.2 s 点云融合重叠窗口 | 从 `ref/dynanav-dynamic-pedestrian` 恢复动态障碍（见 [ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md)） | `simulation/dynanav/pct_scan_worker.py` | 近身移动障碍进入扫掠区时及时拦截，无漏报 |
| `[ ]` | 设计具身 Agent 异步“边想边走”（Think-While-Acting）单主干流式 Fork 哨兵与可抢占航点缓冲 | 启动在线长程 VLM/Agent 探索（见 [ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md)、[ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md)） | `scripts/explore_waypoint_client.py`、`simulation/dynanav/native_waypoints.py`、Agent 接口层 | Agent 行进中通过阅后即焚 Fork 实现 `action_epoch` 语义抢占，不膨胀主干上下文且不越层写速度命令 |
| `[ ]` | 设计关键帧锚定的层次化语义拓扑图/图片数据库与 SLAM 后端双向闭环（含停驻期 ZUPT 与重力先验） | Phase 4 纯几何后端收敛并接入在线语义/动态层（见 [ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md)、[ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md)） | `localization_mapping/backend/`、`localization_mapping/semantic_mapping/`、`planning/` | 房间/走廊代表图绑定 `keyframe_id` 随回环重投影并反哺视觉回环过滤；停驻期 ZUPT 与重力约束消除窄角漂移 |
| `[ ]` | 将定型后的 SLAM 后端热路径（体素降采样、ICP、地图检索）与点云转发桥按需下沉至 C++ | Phase 4 定型或准备实机/高帧率部署（见 [ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md)） | `localization_mapping/backend/`、`simulation/dynanav/pct_scan_worker.py` | 保持与 Python 基线数值等价测试通过，单帧耗时满足实机实时预算 |
| `[ ]` | 增量 Mesh / 3DGS 远期演进（子图 TSDF + 增量 Marching Cubes、2DGS/3DGS 联合精修） | Phase 4 后端定型后（见 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)） | 离线重建管线、数据契约 | 先用现有 13 话题数据验证数据契约；不干扰几何后端收口，不引入核心 CPU 测试的 GPU 依赖 |
| `[ ]` | 为自有核心 Python 模块补充人类易读的中文注释（不改逻辑、不动 `third_party/`） | 用户需要集中阅读代码前随时可派 DSH 执行 | `src/embodied_navigation/`、`scripts/` | 仅注释与 docstring 变更，679 单测与 60 回归零变化 |

## 已知限制（写任务前先读）

- 仿真、GPU 推理、ROS 图与实机验收不在 CPU 测试套件内；`run` 成功只代表进程/计划成功。
- Isaac Sim 版本与场景资产需按配置另行提供，仓库不含权重与场景。
- 本机若无 Isaac/ROS，只能做 CPU 单测与文档工作；仿真验证须在有 Isaac 的 Linux 主机执行。数据包体积大（≈9.3 GB / ≈80 GB），离线评测先做关键帧缓存，禁止在调参循环里反复扫描原始 MCAP。
- 数据卷路径由 `.env.local` 的 `PROJECT_DATA_ROOT` 提供；缺失时脚本 fail-closed，不回退到仓库内路径。

## 当前基线快照（2026-11-28）

| 项 | 值 |
| --- | --- |
| 集成基线 | `main` = `origin/main` = `2144e5f`（`fix(viz): use dynanav_lio_map as Fixed Frame in dynanav_live_pointcloud.rviz`） |
| 活跃工作分支 | **主力记忆/Agent 底座（`H`）**：`feat/static-memory-streaming`（文档基线 `a7fd71a` / 代码 `e159c8a`，**948 单测 / 60 回归**）；**D 路径 V4 闭环底座（`D`）**：`exp/live-v4-ba-sliding-window`（`aada1f1`）；**可提取能力分支**：`feat/semantic-keyframe-graph`（`46e0b48` → `3/4`）、`feat/agent-preemptable-active-slam`（`8aebc95` → `2`）、`perf/slam-backend-cpp-icp`（`ffd7882` → `6`）；**Phase 4 底座**：`exp/slam-backend-stabilization`（`6e1a489`）及 `exp/slam-v2-*` |
| 测试基线 | `feat/static-memory-streaming`（`a7fd71a` / 代码 `e159c8a`）：`tests/unit` **948 条 OK、0 失败 0 跳过**；`tests/regression` **60 条 OK、0 失败**（本机 3 条按环境 skip）；`exp/live-v4-ba-sliding-window@aada1f1` 保持 `807` 单测全绿；`main@2144e5f` 保持 `650` 单测全绿 |
| 默认入口 | `scripts/run_dynanav_native_exploration.sh`（默认 FAST-LIVO2 + PCT/SCAN + RoboGauge），`--truth-pose` 切真值模式；冻结基线配置见 [BASELINES.md](BASELINES.md) |
| 数据卷与云端算力 | `$PROJECT_DATA_ROOT/embodied-navigation/`（核心冻结包 `D-Indoor-2F` / `D-Outdoor-R3LIVE` / `D-Dynamic-M3DGR`、`agent_v4` HM3D 归档）；云端增援 **Google Colab `1,000 CU`**（封顶分配见 [ADR-0005](adr/0005-causal-keyframe-memory-hybrid-retrieval-and-4d-evolution.md)） |
| 文档入口 | 根 [README.md](../../README.md)、[ROADMAP.md](ROADMAP.md)、[BASELINES.md](BASELINES.md)、本文件、[TEST_PLANS.md](TEST_PLANS.md)、[2026-11 学术查重总评](research/2026-11-novelty-audit-and-directions.md)、[VLN 评测真相与三轴演进总纲](research/2026-11-vln-perception-memory-strategy-synthesis.md)、[3D/4D 表征与混合检索 Trade-Off](research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)、[ADR-0001–0005](adr/)、[Agent 教程](../agent-tutorial/README.md)、[SLAM 教程](../slam-tutorial/README.md) |

## 新任务登记模板

往「待办队列」加行时使用：

```markdown
| `[ ]` | <动词开头的任务名> | <主控/DSH/Codex/Flash> | <涉及目录或文件> | <可执行的验收标准：命令/产物/对照口径> |
```

登记前自检：

1. 任务属于 D 路径七层中的哪一层？是否越层？
2. 是否真的需要新分支？能否在当前收口分支上完成？
3. 验收标准是否绑定可复核证据（run 目录、测试名单、diff、离线缓存与评测指标）？
4. 是否与「已完成」中的结论冲突或重复？
5. 涉及后端指标时，是否按 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) 的三层口径（在线/终图 + 逐关键帧 + 逐秒分布）给出对照？

## 会话交接清单

新会话接手时按顺序做，避免只看旧结论：

1. `git status --short --branch`、`git branch -avv`、`git log --oneline -n 6 <branch>` 核对实际分支、HEAD、远端同步与工作树；
2. 读根 [README.md](../../README.md)、[ROADMAP.md](ROADMAP.md) 的阶段状态与 [ADR-0001–0005](adr/)；
3. 跑 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`（`feat/static-memory-streaming` 948 OK / `main` 650 OK）与 `... tests/regression`（60 OK，3 skip），确认基线未被破坏；
4. 按本文件「进行中」逐项确认状态，更新勾选后再新增任务；
5. 进入 Phase 4 后端工作时，先读 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) 与 [ADR-0003 第 4 节](adr/0003-streaming-fork-control-and-semantic-scene-graph.md)，再基于数据卷 `runs/20261121-noisy-imu-rgb-twofloor-explore/` 的两份包做离线缓存；
6. 需要仿真验证时确认 Isaac / ROS 环境与数据卷挂载，再走默认入口；无环境时只做 CPU 与文档工作并在汇报中说明未验证项；
7. 汇报用中文，写清改了什么、验证到什么程度、哪些未验证、服务器上重点看什么；agent 只提交本地分支，合并与推送由用户完成。
