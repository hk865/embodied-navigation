# 活体任务板（TASKS）

> 盘点日期：2026-11-20 · 分支 `refactor/06-clean-base` · HEAD `5b880b2`。阶段目标与完成判据见 [ROADMAP.md](ROADMAP.md)，协作规则见 [AGENTS.md](../../AGENTS.md)。

## 使用说明

- 每次会话开始先读 [ROADMAP.md](ROADMAP.md) 的阶段状态，再在本文件勾选/追加任务；完成的任务移到「已完成」，不要删除（保留决策痕迹）。
- 角色约定：**主控** = 架构判断与破坏性重构；**DSH** = 低成本批量执行；**Codex** = 独立审核与前提查错；**Flash** = 边界清晰的功能开发 / 单测。
- 每条任务必须有可执行验收标准；「进程退出 0」「CPU 测试通过」不能单独作为导航验收。
- 状态标记：`[ ]` 未开始 / `[~]` 进行中 / `[x]` 已完成。

## 已完成

| 任务 | 角色 | 对应模块 | 验收结果 |
| --- | --- | --- | --- |
| 干净底座重构（P1+P2 死代码清理、A 路径退役、动态行人移出主线） | 主控 + DSH | 全仓库 | `refactor/06-clean-base`，累计删减 >8.2 万行，646 单测 / 60 回归 0 失败 |
| 第一优先真值闭环验证（真值位姿 + 200 Hz 真值去畸变 RTX） | 主控 | 规划 / 仿真 | `exp/pct-scan-stairs-truth` 合入 `bb2eef2`：地下室→二楼大厅 +3.25 m，零撞墙/扶手、零 SCAN 失败 |
| 通用 3D PCT+SCAN 替换楼梯特化规则 | 主控 | `planning/` + `third_party/scan` | `reference_z_profile.patch` 上包络 + PCT 起点脚印邻域实测支撑，零楼梯特化规则 |
| SLAM 公共接口初步规范化 | 主控 + Codex | `localization_mapping/backend/` + `third_party/fast_livo2/patches/` | `0011-imu-bias-covariance-params.patch`、`anisotropic_information`、`BackendConfig.from_dict/from_json`、`node.py --config`，落 `5b880b2` |
| 全仓库计划与模块文档重建 | DSH | 根 `README.md`、`docs/plan/`、22 份模块 `README.md` | 所有相对链接可达；模块 README 逐文件表与 `git ls-files` 一致；无退役文件名；27 份 `.md` 独立脚本校验 0 错 |
| 默认 FAST-LIVO2 前端 + PCT/SCAN + Agent 探索实机烟雾测试 | 主控 | `scripts/run_dynanav_native_exploration.sh`、`simulation/dynanav/`、`navigation/` | `runs/20261120-main-smoke/livo-frontend-smoke-01` 通过：`observe` 正常，平地 + 180° 折返爬楼梯（+0.53 m）均 `waypoint_reached`，`capture_exit=0`，零撞墙/扶手与躯干碰撞 |

## 进行中（Phase 3 收口冲刺）

| 状态 | 任务 | 角色 | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[x]` | `--truth-pose` 对照验证 | 主控 | 同上 | `runs/20261120-pct-scan-stairs-truth/truth-stairs-09`（地下室→二楼 +3.25 m）与默认 LIVO `livo-frontend-smoke-01` 分别留存 run 目录，结论分列 |
| `[x]` | Phase 3 全量回归复跑与文档校验 | 主控 + DSH | 全仓库 | 646 单测、60 回归全绿（本机 3 条按环境 skip），27 份 `.md` 零死链/零绝对路径/零僵尸引用 |
| `[ ]` | 交付用户新会话验收并合并推送 `main` | 用户 | 仓库 | 用户新开会话按文档讲解验收；确认后由用户执行合并与 push，agent 不 push |

## 待办队列（Phase 4 及以后，未开始）

| 状态 | 任务 | 角色 | 对应模块 | 验收标准 |
| --- | --- | --- | --- | --- |
| `[ ]` | 修复 LIVO 模式下独立 `yaw` 动作 `rotation_space_stale` 新鲜度门（0.1 s 点云/位姿时差） | 主控 / Flash | `navigation/yaw_action.py`、`simulation/dynanav/pct_scan_worker.py` | 默认 LIVO 模式下 `explore_waypoint_client.py yaw --delta-deg 180` 稳定返回 `yaw_complete`，不放松安全门 |
| `[ ]` | 从新 `main` 切 `exp/slam-measured-submaps`，对照多帧实测子图回环配准 | 主控 + Flash | `localization_mapping/backend/registration.py`、`keyframes.py` | 同输入、同时间轴、同对齐；报告回环接受率、误配/漏配与位姿误差，给出适用场景 |
| `[ ]` | 切 `exp/slam-surface-gate`，对照平面/表面覆盖门控 | 主控 + Flash | 同上 | 同上口径；分别报告误配与漏配，独立地面高度因子另立实验 |
| `[ ]` | 切 `exp/slam-visual-retrieval`，对照图像检索候选 | 主控 + Flash | `localization_mapping/backend/scan_context.py`、视觉检索接入点 | 记录实际候选与图边；光度因子与视觉 BA 另立实验 |
| `[ ]` | 三条分支横向评测汇总 | 主控 + Codex | `localization_mapping/` | 统一输入/时间轴/对齐；报告收益、失败与适用场景；回归或证据不足时停止推广 |
| `[ ]` | 按评测结论重构动态行人支线 | 主控 | 从 `ref/dynanav-dynamic-pedestrian` 提取 | 只提取明确需要的模块；主底座保持静态场景可复现；新增/修改有单测与真值对照 |
| `[ ]` | SLAM 公共接口第二轮规范化（配置 schema、信息矩阵参数统一） | 主控 + Codex | `backend/backend.py`、`backend/node.py` | 未知键/类型错误显式失败；旧配置仍有兼容路径；回归不新增失败 |
| `[ ]` | 后端模块文档补充板载部署/线程模型说明 | DSH | `localization_mapping/backend/README.md` | 与 `node.py --config` 实际参数一致；无过期文件引用 |
| `[ ]` | 持续维护：每次改动同步根 README 与 `docs/plan/` | 全体 | `docs/` | 盘点日期更新；`git diff --check` 通过；链接可达 |

## 已知限制（写任务前先读）

- 仿真、GPU 推理、ROS 图与实机验收不在 CPU 测试套件内；`run` 成功只代表进程/计划成功。
- Isaac Sim 版本与场景资产需按配置另行提供，仓库不含权重与场景。
- 本机若无 Isaac/ROS，只能做 CPU 单测与文档工作；仿真验证须在有 Isaac 的 Linux 主机执行。
- 数据卷路径由 `.env.local` 的 `PROJECT_DATA_ROOT` 提供；缺失时脚本 fail-closed，不回退到仓库内路径。

## 当前基线快照

| 项 | 值 |
| --- | --- |
| 工作分支 / HEAD | `refactor/06-clean-base` / `5b880b2`（`feat(slam): standardize IMU bias covariance patch and anisotropic backend config`） |
| 代码规模 | `src/` 70 个 `.py` / 约 1.7 万行；20 个活跃子目录 |
| 测试基线 | `tests/unit` **646 条 OK**；`tests/regression` **60 条 OK**（本机 3 条按环境 skip） |
| 默认入口 | `scripts/run_dynanav_native_exploration.sh`（默认 FAST-LIVO2 + PCT/SCAN + RoboGauge），`--truth-pose` 切真值模式 |
| 数据卷 | `$PROJECT_DATA_ROOT/embodied-navigation/` |
| 文档入口 | 根 [README.md](../../README.md)、[ROADMAP.md](ROADMAP.md)、本文件、各模块 `README.md` |

## 新任务登记模板

往「待办队列」加行时使用：

```markdown
| `[ ]` | <动词开头的任务名> | <主控/DSH/Codex/Flash> | <涉及目录或文件> | <可执行的验收标准：命令/产物/对照口径> |
```

登记前自检：

1. 任务属于 D 路径七层中的哪一层？是否越层？
2. 是否真的需要新分支？能否在当前收口分支上完成？
3. 验收标准是否绑定可复核证据（run 目录、测试名单、diff）？
4. 是否与「已完成」中的结论冲突或重复？

## 会话交接清单

新会话接手时按顺序做，避免只看旧结论：

1. `git status --short --branch`、`git branch -avv` 核对实际分支与工作树；
2. 读根 [README.md](../../README.md) 与 [ROADMAP.md](ROADMAP.md) 的阶段状态；
3. 跑 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit` 与 `... tests/regression`，确认基线未被破坏；
4. 按本文件「进行中」逐项确认状态，更新勾选后再新增任务；
5. 需要仿真验证时确认 Isaac / ROS 环境与数据卷挂载，再走默认入口；无环境时只做 CPU 与文档工作并在汇报中说明未验证项。
