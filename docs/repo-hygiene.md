# 仓库卫生约束（草案，2026-11-15）

适用于这个导航仿真工程：同一套代码会跑很多实验批次（DynaNav、RoboGauge、Building、主路径），
每批会改配置、加开关、产出大量证据。本文规定**什么东西放哪里、开关怎么出生和死亡、哪些规则由脚本强制**。

现状的问题都来自同一件事：**实验批次的身份被写进了代码和基线配置**。

| 现象（b314337 实测） | 数量 |
|---|---|
| 代码里以轮次命名的标识符（`_r85_*`、`r99_*`、`R85...Gate`…） | 1,408 处，387 个不同名字 |
| 基线配置里以轮次命名的键（`configs/control/settings.json` 的 `r74_*`、`r85_*`…） | 23 个键，schema 也是 `r85_xxx/v1` |
| 写死本机路径 `/home/hyh001`、`/media/hyh001`（不含 archive） | 34 个文件（docs 28、src 2、configs 2、tests 1） |
| 一次性审计 JSON 进仓库 | 4 个文件超过 200 KB，`docs/review` 共 +2.86 万行 |
| 实验日志进 `docs/experiments/` | 45 个按日期命名的文件，和长期文档混在一起 |
| vendored 源码原地改写，patches/ 无法应用 | `third_party/` +2.9 万 / −2.4 万行，约 57% 是空白 |
| 历史脚本副本进 `scripts/` | `archive_20261111/` 29 个文件 |

## 1. 四层存放：代码、基线配置、实验定义、运行产物

| 层 | 放哪里 | 进 Git | 内容 | 命名 |
|---|---|---|---|---|
| 代码 | `src/`、`scripts/`（薄 CLI） | 是 | 只有“现在的行为” | 按职责命名，**不出现轮次号、日期、批次名** |
| 基线配置 | `configs/` | 是 | 当前推荐的那一套 profile | 键按职责命名；schema 写 `student_lower_loop_ack/v1`，不写 `r85_...` |
| 实验定义 | `experiments/<批次>/` | 是（只放小文本） | `experiment.yaml`（引用的基线 + 只写差异的 overlay + 假设 + 命令 + 判定指标）、`REPORT.md`（结论，≤ 300 行） | 批次 ID：`YYYYMMDD-<slug>`，如 `20261111-basement-dsh-exploration` |
| 运行产物 | `$PROJECT_DATA_ROOT/embodied-navigation/runs/<批次>/<run-id>/` | 否 | 日志、bag、点云、视频、逐帧 JSON、审计台账、source snapshot | run-id：`<UTC>_<label>_<4hex>`（沿用 `launch/main.py`） |

规则：

1. **一次运行必须能从“git sha + 实验定义”重现。** 每个 run 目录写 `run_manifest.json`：git sha、工作树是否脏（脏就附 `git diff` 的 sha256 和 patch 文件）、合并后的最终配置、随机种子、Isaac/Gazebo 版本、命令行。`experiment.py` 已有的 `source_manifest.json` 并入这里。
2. **实验 overlay 只写差异。** 不复制整份 yaml。现在 `configs/simulation/dynanav/fastlivo_lio.yaml`（100 行）、`fastlivo_livo.yaml`（129 行）就是基线 `localization_mapping/fastlivo.yaml` 的整份副本，应改成 overlay。
3. **文档里引用产物只用占位符**：`${PROJECT_DATA_ROOT}/embodied-navigation/runs/...`。禁止 `/media/hyh001/新加卷/...` 这类本机路径。
4. **`docs/` 只放长期有效的文档**：架构、操作手册、依赖、配置说明。过程记录（实验日志、审查账本、DSH 实施记录）进 `experiments/<批次>/REPORT.md`，或留在数据卷里。
5. **大文件不进仓库**：单个文件超过 200 KB 默认拒绝（白名单只留 `docs/dependencies/vendored-source-manifest.json`，因为 `docker/verify_native_sources.py` 要读它）。机器可读的审计台账放数据卷或 release asset，仓库里只留 summary。

## 2. 实验开关的生命周期

实验期间加开关是正常的，问题是开关只生不死。规定：

```
提出（experiments/<批次>/overlay）→ 判定 → 晋升 或 拒绝 → 删除开关
```

| 阶段 | 要求 |
|---|---|
| 提出 | 开关**只**出现在实验 overlay 和代码里，基线配置不加。代码里用职责名（`active_turn_braking`），批次号写进 `flags.yaml` 的 `introduced_by` 字段，不写进标识符。 |
| 登记 | `configs/flags.yaml` 一行一个开关：名字、负责人、引入批次、默认值、状态（experimental / promoted / rejected）、**到期日**（默认引入后 30 天）。 |
| 晋升 | 写进基线配置，默认开；**同一个 PR 里删掉关闭分支和开关本身**。晋升后它就是“现在的行为”，不再是开关。 |
| 拒绝 | 删除代码和 overlay，`REPORT.md` 记一行结论。 |
| 到期 | 检查脚本报红，必须二选一处理。 |

这样，基线配置里不会再有“恒为开”的开关，代码里也不会残留“恒为关”的死分支。不必要校验的清查结果（第 5 节）就是存量的清理清单。

**不用环境变量做功能开关。** `R88_ASYNC_PHYSICAL`、`R92_READY_COMMIT` 这类环境变量开关一律取消。经过验证的行为成为唯一实现，旧分支删除。需要排查时，按下面两种方式之一：
- 开一个 `exp/<批次>` 分支，在分支上改代码；
- 新建一份配置 overlay，用来调阈值。

控制器只从 `start.py → launch/main.py` 启动。运行时配置只来自 run 目录里冻结的那一份；入口发现没有冻结配置就拒绝启动。

**诊断开关不能改变决策。** 诊断或观察模式只能多写证据，不能把 `allowed=False` 改成 `True`。现在 `pid_support.py` 的 `observe_business_gate` 就是这种情况。生产 profile 在配置解析时直接拒绝诊断开关。

## 3. vendored 代码

1. `third_party/<包>/` 保持**上游原样**，加上 `patches/NNNN-*.patch` 顺序应用；`third_party/<包>/UPSTREAM` 记录上游 URL 和 commit。
2. 禁止在 vendored 树里做格式化或纯注释改动。真需要说明，写进 patch 的头注释或 `third_party/<包>/NOTES.md`。
3. 检查脚本做三件事：patch 能 `git apply --check`；应用后的树和仓库里的树逐文件 sha256 一致；manifest 和 blob 一致。
4. 加 `.gitattributes`（`* text=auto eol=lf`，二进制显式标注）。当前 Windows 上 `core.autocrlf=true` 会让 282 条 manifest 哈希对不上。

## 4. 由脚本强制的规则（`scripts/check_repo_hygiene.py`）

存量太多，没法一次清零，所以采用**棘轮**：`configs/hygiene_baseline.json` 记录当前计数，**只许减少、不许增加**。检查脚本挂在 `tests/unit`（Dockerfile 已经会跑）和 pre-commit 上。

| # | 规则 | 方式 | 当前基线 |
|---|---|---|---|
| H1 | `src/ scripts/ configs/ tests/ docs/` 里不得出现 `/home/<user>`、`/media/<user>` | 硬禁（仅 `archive` 白名单） | 34 个文件，先清掉 |
| H2 | 新增标识符和配置键不得匹配 `\b_?[rR]\d{2,3}_` | 棘轮 | 标识符 1,408 处，配置键 23 个 |
| H3 | 已跟踪的单个文件不得超过 200 KB | 白名单 | 4 个文件，清掉 3 个 |
| H4 | `configs/flags.yaml` 里的开关不得过期；代码读取的开关必须已登记 | 硬禁 | 新规则 |
| H5 | `third_party` 等于“上游 + patches” | 硬禁 | 现在不成立，需先重建 |
| H6 | `scripts/*.py` 不超过 300 行，逻辑放进包；测试不得 `sys.path.insert` 到 `scripts/` | 棘轮 | 11 个脚本超过 300 行；24 个测试文件在做 path hack |
| H7 | 测试只读取仓库内或 `tmp` 下的文件；`unittest discover` 收集到全部 `test_*.py`（不许出现 pytest 风格、unittest 收集不到的文件） | 硬禁 | `test_livo_yaw_sampler_policy.py` 读取被 gitignore 的本机文件，而且是 pytest 风格 |
| H8 | `docs/experiments/`、`docs/review/` 不再新增文件，新的过程记录进 `experiments/<批次>/` | 硬禁 | 新规则 |

## 5. 分支与提交

1. **WIP 快照分支不整体合并**，按层拆成 PR：vendored 格式化（或直接撤回）→ vendored 逻辑 patch → configs overlay → scripts 逻辑下沉进包 → 代码重构 → 文档。每个 PR 单独跑通 `unittest discover`。
2. 纯格式化提交单独成一个 commit，并记进 `.git-blame-ignore-revs`。
3. 一个实验批次对应一个分支 `exp/<批次>`；结论落地时，只把晋升的那部分合进 main，分支本身留作记录。

## 6. 落地顺序建议

1. 先加 H1、H3、H7 的检查（最便宜，立刻止血），同时清掉 34 个本机路径和 3 个大 JSON。
2. 建 `experiments/` 目录和 `run_manifest.json`，把 `docs/experiments/` 的 45 份记录迁过去（数据链接改成占位符）。
3. 建 `configs/flags.yaml`，按不必要校验的清查结果把恒开或恒关的 `rNN` 开关清掉，再上 H2 和 H4 的棘轮。
4. 重建 vendored patches，上 H5。
5. 随 `explore_waypoint_client` 等脚本的拆分逐步收紧 H6。
