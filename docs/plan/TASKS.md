# 活体任务板（TASKS）

> 盘点日期：2026-11-21 · 集成基线 `main` `2144e5f`（= `origin/main`） · 当前开发推进分支 `exp/startup-walk-imu-bias` `9e3408d`（本地领先 origin 6 个提交）。阶段目标与完成判据见 [ROADMAP.md](ROADMAP.md)，协作规则见 [AGENTS.md](../../AGENTS.md)。

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

## 进行中（Phase 4 启动）

| 状态 | 任务 | 角色 | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[~]` | 审阅 `exp/startup-walk-imu-bias`（领先 origin 6 个提交）并决定合入 `main` | 用户 | 仓库 | 逐行读 diff + 全量测试通过后由用户合并推送；合入前保持工作树干净，不 push |

## 待办队列（Phase 4 主线及以后，按顺序）

| 状态 | 任务 | 角色 | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[ ]` | **Phase 4 首项**：离线关键帧抽取缓存 + 三层轨迹评测器（[ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md)） | 主控 + Flash | 离线评测脚本、`localization_mapping/backend/` | 数据包入口 `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261121-noisy-imu-rgb-twofloor-explore/`；从两份数据包只读抽取关键帧点云/前端里程计/IMU 状态/真值时间轴为 ≈200–500 MB 缓存；输出在线与终图两条轨迹的 ATE/AOE/RTE/ROE（分 3D/XY/Z、Roll/Pitch/Yaw，多窗口）、逐关键帧演化曲线、逐秒位姿分布与标准 TUM 轨迹；CPU 向量化、单次全套 <1 s、`.npz` <15 MB；真值入口为两份数据包的 `capture/truth_scoring_stream.jsonl`，可用 `runs/20261117-loop-comparison`、`runs/20261119-hba-project-comparison` 作历史口径对照 |
| `[ ]` | 后端修复 ①：6-DoF ICP / 位姿图重力 **Roll/Pitch 强先验** | 主控 + Codex | `localization_mapping/backend/registration.py`、位姿图优化 | 在 `20261121T133734Z` 上消除 6°~8.3° 的 Roll/Pitch 拉歪与 z 跳变；同一离线缓存上 ATE/AOE 不劣化，回归失败名单不新增 |
| `[ ]` | 后端修复 ②：**原地/近邻过密回环抑制** | 主控 + Codex | `localization_mapping/backend/` 回环筛选与图边剔除 | 在 `20261121T142308Z` 上抑制 407+ 过密回环，二楼 Z 偏差从 -74 cm 收敛；回环接受率、误配/漏配与位姿误差同口径对比 |
| `[ ]` | 后端修复 ③：Agent 思考停驻期 **ZUPT 停车锚定约束** | 主控 + Codex | 前端/后端静止检测与相对位姿零变动约束、IMU 零偏收敛 | 长停滞段不再诱发前端发散；静止窗内位姿漂移与 IMU 零偏收敛有量化曲线，回归不新增失败 |
| `[ ]` | 三条后端对照分支在同一份离线缓存上横向评测并汇总 | 主控 + Codex | `exp/slam-measured-submaps` / `exp/slam-surface-gate` / `exp/slam-visual-retrieval` | 统一输入/时间轴/对齐；报告收益、失败与适用场景；出现回归、缺少可比证据或用户搁置时保留结论并停止推广 |
| `[ ]` | 按评测结论重构动态行人支线 | 主控 | 从 `ref/dynanav-dynamic-pedestrian` 提取 | 只提取明确需要的模块；主底座保持静态场景可复现；新增/修改有单测与真值对照 |
| `[ ]` | SLAM 公共接口第二轮规范化（配置 schema、信息矩阵参数统一） | 主控 + Codex | `backend/backend.py`、`backend/node.py` | 未知键/类型错误显式失败；旧配置仍有兼容路径；回归不新增失败 |
| `[ ]` | 后端模块文档补充板载部署/线程模型说明 | DSH | `localization_mapping/backend/README.md` | 与 `node.py --config` 实际参数一致；无过期文件引用 |
| `[ ]` | 持续维护：每次改动同步根 README 与 `docs/plan/` | 全体 | `docs/` | 盘点日期更新；`git diff --check` 通过；链接可达 |

## 架构决策记录（ADR）索引

| ADR | 标题 | 状态 | 与任务的关系 |
| --- | --- | --- | --- |
| [ADR-0001](adr/0001-livo-yaw-pipeline-overlap.md) | LIVO 独立 `yaw` 清阻塞与 ≤ 0.2 s 点云融合重叠 | 已采纳（2026-11-20） | `yaw` 修复已完成并合入 `main`；远期触发项见「远期 / 触发式」 |
| [ADR-0002](adr/0002-embodied-agent-async-and-map-outlook.md) | 具身 Agent 异步行动、语义/动态地图与热路径演进备忘 | 提案 / 远期（2026-11-20） | 流式控制、语义图与 C++ 热路径下沉的触发条件 |
| [ADR-0003](adr/0003-streaming-fork-control-and-semantic-scene-graph.md) | 单主干流式 Fork 异步控制与关键帧锚定语义图片图数据库 | 提案 / 架构演进（2026-11-21） | 阶段 A（Phase 4 纯几何后端）即为本文件三项后端修复的来源；语义图与流式 Fork 在其后 |
| [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) | 面向 FAST-LIVO2 的增量 Mesh / 3DGS 演进与三层 SLAM 后端评测口径 | 提案 / 评测规范（2026-11-21） | Phase 4 首项评测器与三项后端修复的唯一打分尺 |

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

## 当前基线快照（2026-11-21）

| 项 | 值 |
| --- | --- |
| 集成基线 | `main` = `origin/main` = `2144e5f`（`fix(viz): use dynanav_lio_map as Fixed Frame in dynanav_live_pointcloud.rviz`） |
| 工作分支 / HEAD | `exp/startup-walk-imu-bias` / `9e3408d`（本地领先 origin 6 个提交；基线 `main`，共 12 个提交） |
| 代码规模 | `src/` 73 个 `.py` / 约 1.86 万行（按当前工作分支 `git ls-files` 实测） |
| 测试基线 | 当前工作树 `tests/unit` **679 条 OK、0 失败 0 跳过**（集成基线 `main` 为 650 条 OK）；`tests/regression` **60 条 OK、0 失败**（本机 3 条按环境 skip） |
| 默认入口 | `scripts/run_dynanav_native_exploration.sh`（默认 FAST-LIVO2 + PCT/SCAN + RoboGauge），`--truth-pose` 切真值模式 |
| 数据卷 | `$PROJECT_DATA_ROOT/embodied-navigation/`；Phase 4 评测数据集在 `runs/20261121-noisy-imu-rgb-twofloor-explore/` |
| 文档入口 | 根 [README.md](../../README.md)、[ROADMAP.md](ROADMAP.md)、本文件、[ADR-0001–0004](adr/)、各模块 `README.md` |

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
2. 读根 [README.md](../../README.md)、[ROADMAP.md](ROADMAP.md) 的阶段状态与 [ADR-0001–0004](adr/)；
3. 跑 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`（679 OK）与 `... tests/regression`（60 OK，3 skip），确认基线未被破坏；
4. 按本文件「进行中」逐项确认状态，更新勾选后再新增任务；
5. 进入 Phase 4 后端工作时，先读 [ADR-0004](adr/0004-incremental-mesh-3dgs-and-layered-backend-eval.md) 与 [ADR-0003 第 4 节](adr/0003-streaming-fork-control-and-semantic-scene-graph.md)，再基于数据卷 `runs/20261121-noisy-imu-rgb-twofloor-explore/` 的两份包做离线缓存；
6. 需要仿真验证时确认 Isaac / ROS 环境与数据卷挂载，再走默认入口；无环境时只做 CPU 与文档工作并在汇报中说明未验证项；
7. 汇报用中文，写清改了什么、验证到什么程度、哪些未验证、服务器上重点看什么；agent 只提交本地分支，合并与推送由用户完成。
