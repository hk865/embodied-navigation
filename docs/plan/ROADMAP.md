# 开发路线图（ROADMAP）

> 盘点日期：2026-11-20 · 当前分支 `refactor/06-clean-base` · HEAD `5b880b2` · 活体文档，随阶段推进更新。

本文件是项目唯一主线的开发计划入口。阅读顺序：根 [README.md](../../README.md) → 本文件 → [TASKS.md](TASKS.md) → 各模块 `README.md`。协作与提交规则以 [AGENTS.md](../../AGENTS.md) 为准。

## 1. 唯一主线与七层架构边界

项目唯一主线是 **D 路径：Isaac Sim / DynaNav + Unitree Go2 + RoboGauge MoE-CTS**。A 路径（Gazebo、`start.py` → `launch/main.py`、`control/`、私有 Fast DDS）已在干净底座中整块退役删除，仅保留于 `main`、`archive/pre-review-20261109` 等历史分支，不再恢复。

代码按七层边界组织，跨层只走显式接口，不越层直连：

| # | 层 | 主要目录 / 入口 | 职责边界 |
| --- | --- | --- | --- |
| 1 | 仿真 | `src/embodied_navigation/simulation/` | Isaac Sim / DynaNav 场景、Go2 RGB-D / RTX LiDAR / IMU 采集、`--truth-pose` 真值模式、仿真暂停与回执 |
| 2 | 感知与通信 | `src/embodied_navigation/perception/` + `common/` | 传感器消息适配、去畸变配对、健康门、位姿/速度契约；`common/` 提供几何、运行时 IO、进程日志与遥测基础设施 |
| 3 | 定位建图（前端 + 后端） | `localization_mapping/` + `third_party/fast_livo2/` | FAST-LIVO2 前端（post-VIO 位姿 / 实测点云）、SLAM 后端（关键帧、回环、位姿图、地图存取、先验重定位）、语义建图 |
| 4 | 规划 PCT + SCAN | `planning/` + `third_party/pct/` + `third_party/scan/` | PCT 实测分层支撑全局路线、原生 SCAN 局部轨迹、扫描交接与来源、Agent 航点候选 |
| 5 | 控制跟踪 | `navigation/` | `RouteTracker` / `ContinuousScanFollower` 跟踪、`yaw_action.py` 转向动作、RoboGauge 路线跟踪接口；输出到身体速度命令为止 |
| 6 | 机器人执行 | `robot/` | Go2 关节契约、RoboGauge MoE-CTS 策略运行时、策略原生观察与执行桥 |
| 7 | Agent 感知与控制介入层 | `simulation/dynanav/operator_map.py`、`localization_mapping/semantic_mapping/`、`planning/exploration/`、`scripts/explore_waypoint_client.py`、`.agents/skills/dynanav-explore/` | 目标下发、运行中干预、状态/地图查询、语义建图与稀疏航点决策；不写速度命令、不读仿真真值 |

默认端到端入口见 [根 README 的快速启动](../../README.md)；PCT/SCAN 时序与证据边界见 [PCT/SCAN 说明](../dynanav-pct-scan.md)。

## 2. 阶段里程碑

### Phase 1 — 干净底座重构（已完成）

- 分支：`refactor/06-clean-base`（基于 `refactor/05-agents-rules`）。
- 成果：累计删减 >8.2 万行；A 路径整块退役；P1/P2 废弃模块清理；动态行人支线完整归档至参考分支 `ref/dynanav-dynamic-pedestrian`；单测零失败。
- 验收：全量单测/回归失败名单相对起点不新增；逐行 diff 复核后由用户决定合并。

### Phase 2 — 第一优先真值闭环验证（已完成并合入 `refactor/06-clean-base`）

- 分支：`exp/pct-scan-stairs-truth`（合入提交 `bb2eef2`）。
- 目标：跳过 SLAM 定位漂移干扰，用仿真真值位姿 + 200 Hz 真值去畸变 RTX 点云验证规划完整性。
- 通用 3D PCT+SCAN：
  - `third_party/scan/patches/reference_z_profile.patch` 提供参考 z 上包络；
  - PCT 全局规划接入起点脚印邻域实测支撑，不再依赖场景先验；
  - 零楼梯特化规则、零场景 tomogram、零目标吸附、零直线兜底。
- 结果：完成地下室至二楼大厅全链路，净爬升 **+3.25 m**，零撞墙 / 零撞扶手、零 SCAN 失败。
- 保留：分支保留作复现入口；不改变默认 FAST-LIVO2 前端链路。

### Phase 3 — 当前收口冲刺（进行中，准备合入 `main` 基线）

目标：把已验证能力收口成可交付、可审核的干净基线。四项收口工作：

1. **SLAM 公共接口初步规范化（已完成，`5b880b2`）**：
   - `third_party/fast_livo2/patches/0011-imu-bias-covariance-params.patch`：IMU 偏置协方差进参数，不再写死；
   - 后端 `anisotropic_information` 各向异性信息矩阵；
   - `BackendConfig.from_dict/from_json` 严格配置加载（未知键/类型错误直接失败），`node.py --config` 显式入口。
2. **默认 FAST-LIVO2 前端 + PCT/SCAN + Agent 探索实机烟雾测试（已完成，`livo-frontend-smoke-01`）**：用 `scripts/run_dynanav_native_exploration.sh` 默认链路在 `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261120-main-smoke/livo-frontend-smoke-01` 跑通平地导航与 180° 折返爬楼梯（+0.53 m），`capture_exit=0`、零撞墙/扶手、零实质躯干碰撞、零 SCAN 失败（仅记录独立 `yaw` 动作因 LIVO 点云/位姿 0.1 s 时差被外层新鲜度门拦停，记入 [TASKS.md](TASKS.md) 后续微调）。
3. **全仓库计划与模块文档重建（已完成）**：根 `README.md`、`docs/plan/`、全部活跃模块 `README.md` 与 `git ls-files` 对齐（本文件即该工作的产物之一）。
4. **交付用户审核**：用户新开会话阅读讲解并逐行审核 diff 与文档后，由用户合并、推送为新的干净 `main` 基线（agent 不 push、不 force-push）。

Phase 3 完成判据（全部满足才算收口）：

- [x] `git diff --check` 通过，工作树只包含文档与已审核代码改动；
- [x] 646 条单测、60 条回归全绿（本机 3 条回归按环境 skip），失败名单相对 `5b880b2` 不新增；
- [x] 默认入口烟雾测试有可复核的 run 目录与结论（含 `--truth-pose` 对照 `truth-stairs-09` 与默认 LIVO `livo-frontend-smoke-01`）；
- [x] 根 README 与 `docs/plan/` 链接全部可达，模块 README 与 `git ls-files` 一致；
- [ ] 用户完成审核、合并并推送 `main`。

### Phase 4 — 下一阶段重点（Phase 3 合入后启动）

- 从新 `main` 基线切出 3 条精简、互不耦合的 SLAM 后端对照分支，做**同输入、同时间轴**横向评测：
  | 分支 | 对照点 |
  | --- | --- |
  | `exp/slam-measured-submaps` | 多帧实测 LiDAR 子图对回环配准的影响 |
  | `exp/slam-surface-gate` | 平面 / 表面覆盖门控对 ICP 回环误配与漏配的影响 |
  | `exp/slam-visual-retrieval` | 图像检索补充候选、几何配准验证相对位姿 |
- 每条分支只带自己的最小改动与独立配置，禁止互相叠加；评测口径固定为同一份输入、同一时间轴、同一对齐方式，报告收益、失败与适用场景。
- 依据评测结论按需重构动态行人支线（从 `ref/dynanav-dynamic-pedestrian` 提取，不整支合并）。

## 3. 分支归属

2026-11-20 本地与远端核对结果；提交号是分支头，复现时以运行记录的源码为准。

| 分支 | 状态 | 用途 | 后续动作 |
| --- | --- | --- | --- |
| `main` / `origin/main` | 集成与发布 | 独立验收入口（当前 `51209ba`，仍是旧底座） | Phase 3 收口后由用户合并推送为新基线 |
| `refactor/00-plan-docs` … `refactor/05-agents-rules` | 已验收栈 | 计划、几何去重、死代码、旧路径移除、SLAM 后端、协作规则逐包叠加 | 保留历史 |
| `refactor/06-clean-base` | **当前工作基线** | 干净底座 + Phase 2 成果（`bb2eef2`）+ Phase 3 SLAM 规范化（`5b880b2`） | 收口冲刺，交付用户审核 |
| `exp/pct-scan-stairs-truth` | 已完成 | 真值位姿 + 200 Hz 真值去畸变 RTX 驱动 PCT/SCAN 多层闭环 | 保留作复现入口 |
| `ref/dynanav-dynamic-pedestrian` | 参考 | 动态行人 / 学习分割支线完整保全 | Phase 4 按需提取重构 |
| `exp/loop-measured-submaps`、`exp/loop-surface-gate`、`exp/loop-visual-retrieval` | 历史实验 | 上一轮回环共同比较口径下的三条对照（远端已核实） | 结论保留，Phase 4 重新按新基线切分支 |
| `refactor/05-loop-evidence-base` | 历史 | 回环比较实现、配置与证据保全 | 按需查证 |
| `archive/pre-review-20261109`、`wip/navigation-local-snapshot-20261115`、`backup/slam-controlled-experiment-20261116`、`backup/exp-two-floor-demo-wip-20261120`、本地 `exp/two-floor-demo` | 归档 / 备份 | 审查前源码、在途快照、受控 SLAM 追踪、拆分前两层联调 | 只读查证，不继续叠加 |
| `improve/module-docs-readability`、本地 `feat/navigation-review-web` | 文档 / 工具 | 模块文档与阅读入口、导航审阅页面 | 按需摘取 |

## 4. 数据卷与产物约定

工程数据根统一为 `$PROJECT_DATA_ROOT/embodied-navigation/`（由 checkout 的 `.env.local` 提供，Linux 上解析到数据盘），仓库只放源码、配置、实验定义与结论。

| 路径 | 内容 |
| --- | --- |
| `runs/<批次>/<run-id>/` | 新原生运行与回放的完整产物：冻结配置、`runtime_context.json`、日志、回执、`manifest.json` 等 |
| `experiments/<run>/` | DynaNav / Isaac 实验产物（capture、observer、点云与逐帧回执） |
| `build_cache/` | 原生 SCAN / LIO 构建产物与安装前缀 |
| `runtime_assets/{assets,models,runs}/` | 仓库根 `assets/`、`models/`、`runs/` 软链指向的部署资产、权重与历史运行 |
| `reports/`、`deliverables/` | 整理后的报告与交付物 |

规则：运行产物不进仓库；单个文件 >200 KB 默认不提交；开跑前用 `findmnt -T` 与 `df` 核对挂载与容量；详细放置与清理规则见数据根 `AGENTS.md` 与仓库 [AGENTS.md 第 6 节](../../AGENTS.md)。

## 5. 推进原则

1. **一条主线**：只做 D 路径；不恢复 A 路径、B/C 基准或已移出的行人支线。
2. **分支按主题切**：一个分支只验证一个假设，不在地基分支上叠加混合改动；验证通过的行为就是唯一实现，不用环境变量做开关。
3. **证据优先**：CPU / 合成测试 / 进程退出码都不等于物理到达；结论必须绑定 run 目录与时间轴。
4. **先收口再扩张**：Phase 3 未完成前不开新的 SLAM 对照分支；新分支从干净 `main` 切出。
5. **由用户推送**：agent 只提交到本地分支，合并与推送由用户完成。
