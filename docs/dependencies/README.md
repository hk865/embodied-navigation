# 导航源码依赖闭包

本目录记录 vendored 第三方源码的来源、构建闭包、许可状态与迁移核验证据；它不参与运行时，也没有代码在启动时读取这里的 JSON。

审计与复制日期：2026-11-08。源仓库为 [hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control](https://github.com/hk865/Fast-Livo2-SCAN-Gazebo-dog-sim-control)，提交 `5688f2868e241ca4a7cc80ba9f166f8afa84c727`，分支 `feature/fastlivo2-backend-building-20261030`。原工作树跟踪文件干净，仅未跟踪的 `history/` 未复制。没有 Git 子模块或 gitlink；所有必要包为实体文件。

## 目录结构

```text
docs/dependencies/
├── README.md                          # 本文件：依赖闭包索引与边界
├── BUILD.md                           # Docker 原生层构建/运行边界（77 行）
├── BUILDING-ENTRYPOINT.md             # Building 入口闭包与 SLAM-only 限制（11 行）
├── LICENSE-STATUS.md                  # 许可、署名与来源边界表（21 行）
├── MODEL-NOTICE.md                    # Student 权重来源/发布依据/许可（38 行）
├── PYTHON-AND-DEPLOYMENT.md           # 迁移前自有 Python 模块清单（49 行）
├── building-source-closure.json       # Building 运行时来源快照与哈希（15 行）
├── native-build-plan.json             # 原生构建计划：源根、包、pin、补丁（48 行）
├── runtime-extra-source-closure.json  # 隔离启动发现到的额外运行时来源（41 行）
├── vendored-source-manifest.json      # 逐文件复制清单：304 个文件条目（3728 行）
└── verification.json                  # 2026-11-08 构建/来源核验结果（35 行）
```

## 代码、日志与配置

本目录没有代码、没有日志；按“说明 / 证据 / 配置”三类理解：

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 说明（构建） | [`BUILD.md`](BUILD.md) | Docker 原生层的构建命令、镜像边界与“不要声称基础层可跑完整场景”的限制；实际构建脚本是 `docker/build_native.py`/`docker/Dockerfile.native`。 |
| 说明（入口/部署） | [`BUILDING-ENTRYPOINT.md`](BUILDING-ENTRYPOINT.md)、[`PYTHON-AND-DEPLOYMENT.md`](PYTHON-AND-DEPLOYMENT.md) | Building 入口只接受 `slam`/`fast_livo2_async` 的闭包说明，以及迁移前自有 Python 模块路径清单（供核对依赖闭包，不是 `PYTHONPATH` 安装指引）。 |
| 说明（许可） | [`LICENSE-STATUS.md`](LICENSE-STATUS.md)、[`MODEL-NOTICE.md`](MODEL-NOTICE.md) | 逐组件许可证据与缺口表；Student 权重的来源链、字节数、导出差异与“不是新授权”的边界。两者都明确“不能代替法律审查”。 |
| 证据（来源/manifest） | [`vendored-source-manifest.json`](vendored-source-manifest.json) | 304 个 `files` 条目（每个含来源路径/SHA256）、3 个 `excluded` 条目；上游仓库/commit/分支与“hash 只用于复制完整性，不是运行验收门”的说明。 |
| 证据（核验/闭包） | [`verification.json`](verification.json)、[`building-source-closure.json`](building-source-closure.json)、[`native-build-plan.json`](native-build-plan.json)、[`runtime-extra-source-closure.json`](runtime-extra-source-closure.json) | 2026-11-08 核验结果（`manifest_files_verified=304`、11 个 ROS 包、VIO 补丁逆向 dry-run、PCT 编译、`ros_compile/docker_build/simulation=not run`）；Building 源文件 SHA/字节/替换记录；11 包构建计划与补丁；隔离启动发现的额外运行时来源。 |
| 配置 | 无仓库运行配置读取点 | 本目录不含运行时配置，`configs/` 也不读取这里的 JSON。`native-build-plan.json` 的 `source_roots/packages/source_pins` 是构建计划数据，由 `docker/` 脚本按需读取/复制（`grep -rn "docs/dependencies" src scripts` 无代码读取）。 |
| 日志/输出 | 无 | 无写文件、无 stdout 产物；所有“核验结果”是人工/脚本回填的静态 JSON，不是每次构建自动刷新。 |

## 模块内文件耦合

- 依赖方向：`README.md → 其余 10 个文件` 是唯一索引关系；JSON 之间互不 import。README 的“文件入口”原只列了 5 个文件（BUILD、PYTHON-AND-DEPLOYMENT、LICENSE-STATUS、manifest、verification），漏列 `BUILDING-ENTRYPOINT.md`、`MODEL-NOTICE.md`、`building-source-closure.json`、`native-build-plan.json`、`runtime-extra-source-closure.json`；本次重写已把 5 个补进上面的目录树与分类表。
- 外部引用：`BUILD.md` 被源码 README 引用（[src 包索引](../../src/embodied_navigation/README.md)、[PCT](../../src/embodied_navigation/planning/pct/README.md)、[native backend](../../src/embodied_navigation/localization_mapping/backend/native/README.md)、[RoboGauge motor_src](../../src/embodied_navigation/simulation/gazebo/robogauge/motor_src/README.md)、[native LIO fixtures](../../tests/native/dynanav_lio/README.md)）。这些是链接引用，不产生代码依赖。
- 隐式契约：`vendored-source-manifest.json` 的 304 条记录与 `third_party/` 实际文件必须逐条一致（由 `docker/verify_native_sources.py` 校验）；`verification.json` 的 `manifest_files_verified` 只是某次核验的快照，不同步刷新就会与磁盘漂移。`building-source-closure.json` 的哈希与文件字节同样需要人工维护。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README.md` | 本文件 | 10 个同目录文件 | 人 | 清楚，但索引需与目录同步 |
| `BUILD.md` | 77 | `docker/` 脚本 | 多个源码 README 链接 | 清楚：命令+边界 |
| `BUILDING-ENTRYPOINT.md` | 11 | Building 代码/测试事实 | 本 README | 清楚但短：结论无对应测试路径 |
| `PYTHON-AND-DEPLOYMENT.md` | 49 | 迁移前模块路径 | 本 README | 可接受：历史清单，需标注“迁移前” |
| `LICENSE-STATUS.md` | 21 | 各组件 LICENSE/NOTICE | 本 README | 清楚：逐组件表 |
| `MODEL-NOTICE.md` | 38 | 模型档案/训练记录 | 本 README | 可接受：来源链完整，缺口已注明 |
| `vendored-source-manifest.json` | 3728 | 实际 `third_party/` 文件 | `docker/verify_native_sources.py` | 清楚（机器可读），但人工审阅成本高 |
| `verification.json` | 35 | 构建/核验脚本 | 本 README | 可接受：未刷新时会过期 |
| 其余 3 个 JSON | 15–48 | 构建/运行时闭包 | 构建脚本/人工 | 清楚：字段自解释 |

结论：**没有代码依赖环**，耦合是“索引 ↔ 一次性证据快照”的弱耦合；主要风险不是结构，而是证据 JSON 不会自动随磁盘刷新（`verification.json` 的 `not run` 项尤其容易被后来者误读为“没做”）。

## 可读性评估

- `README.md`（原 32 行）：**需改进（索引完整性）**。已核实事实（源仓库/commit/分支、304 文件 manifest、11 包、许可缺口）都在，但“文件入口”漏了 5 个同目录文件（见上），且“当前已提供/依赖关系”两段是全仓级结论，读者无法从本文件判断每条结论的证据在哪个 JSON。
- `BUILD.md`：**良好**。命令块标注“只打印计划/不联网”，并明确原生层不等于完整应用；77 行内给出 Docker 目标与 `ros2 pkg list` 检查。
- `LICENSE-STATUS.md` / `MODEL-NOTICE.md`：**可接受**。表格化、逐组件列“已保留证据/尚需注意”，并明确不作法律结论；`MODEL-NOTICE` 的训练链表格较长但每行都有阶段名与证据，信息密度高。
- `PYTHON-AND-DEPLOYMENT.md`：**可接受**。开头说明“迁移前模块路径”, 但清单里 `localization/backend/*` 这类旧路径容易与当前 `src/embodied_navigation/localization_mapping/backend/` 混淆，需要读者自己映射。
- `verification.json` / 其余 JSON：**可接受**。机器可读、字段自解释；但 `not run` 与 `passed` 混排，无“快照日期/是否仍有效”的一行声明（`audit_date_utc` 在 JSON 内，README 未引用）。这是文档问题，不是数据问题。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `README.md` 每个结论后加“证据文件”列/链接（如 304 → manifest、11 包 → `native-build-plan.json`、许可缺口 → `LICENSE-STATUS.md`） | 现在读者要自己找证据，且原索引漏 5 个文件 | 无（纯文档） | 逐条点开链接确认文件存在（自检脚本已校验） |
| P2 | 给 `verification.json`/`building-source-closure.json` 加“快照日期 + 是否仍为当前结论”的 README 表头说明 | 静态证据最容易被当成实时状态；`ros_compile/docker_build/simulation=not run` 需要与“未验证”区分开 | 无 | 人工对照 JSON 的 `audit_date_utc` |
| P2 | 在 `PYTHON-AND-DEPLOYMENT.md` 开头加“旧路径 → 当前路径”映射示例（至少 `localization/backend` → `src/embodied_navigation/localization_mapping/backend`） | 旧路径与当前命名空间同名不同层，易误导 | 无 | 抽查 3 条映射在磁盘上存在 |
| P3 | 把 `vendored-source-manifest.json` 的 304 条人工摘要为“按组件计数表”（fast_livo2/vikit/scan/livox/fastdds 等各多少条）放在 README | 3728 行 JSON 无法人工通读，但读者需要“复制了多少” | 无；计数须从 JSON 生成 | 用 `python3 -c` 统计 `files` 前缀后与表对照 |

没有 P0/P1：本目录不参与运行，证据 JSON 过期只会误导阅读，不会改变行为；结构上不需要拆分或重构。

## 当前已提供

- FAST-LIVO2 core + ROS wrapper，含实际使用的 VIO 四处 floor 修复。
- SCAN `plan_manage`、`plan_env`、`path_searching`、`bspline_opt`、`traj_utils`，以及生成 `Bspline` 的 `scan_planner_msgs`。
- PCT `pct_cli.cpp`、`native/a_star/a_star_search.{cc,h}` 和 `native/common/data_types.h`，加原有 LICENSE/NOTICE 和独立 CMake 构建文件。
- Vikit common/ROS、Livox ROS Driver2 的实际源码和现有声明。
- FastDDS 2.14.6/Asio 固定公开源码来源和三份实际补丁、构建脚本。
- 可从公开依赖构建的 CPU ROS Jazzy Docker 原生层。没有复制旧二进制或使用其它本地定制工程。

## 文件入口

- [构建与运行边界](BUILD.md)
- [Python/部署与额外原生模块](PYTHON-AND-DEPLOYMENT.md)
- [许可状态](LICENSE-STATUS.md)
- [逐文件复制清单](vendored-source-manifest.json)
- [本次验证结果](verification.json)
- [Building 入口闭包](BUILDING-ENTRYPOINT.md)
- [Student 权重来源与许可](MODEL-NOTICE.md)
- [Building 运行时来源快照](building-source-closure.json)
- [原生构建计划](native-build-plan.json)
- [额外运行时来源闭包](runtime-extra-source-closure.json)

## 依赖关系

`livox_ros_driver2 + vikit_common + vikit_ros -> fast_livo2_core -> fast_livo2_ros/fastlivo_mapping`。

`plan_env -> path_searching -> bspline_opt -> traj_utils`，这些包与 `scan_planner_msgs` 一同供 `scan_planner` 使用。实际入口启动 `scan_planner_node` 和 `closed_loop_controller`，因此不能只复制单个规划器 `.cpp`。

FAST-LIVO2 的两个包必须维持相邻结构，wrapper 的 CMake 包含 `../fast_livo2_core/include/...`。SCAN `plan_env` 的回归测试源在原 CMake 中无条件构建，已保留，不能仅通过 `BUILD_TESTING=OFF` 排除它们。

原 SCAN `package.xml` 中还保留 `go2_description`、`map_generator`、`mockamap`、`local_sensing_node`、`odom_visualization` 等上游演示依赖。这些不在当前直接启动节点的编译闭包内，本目录没有伪造这些包；上游演示 launch 不因此自动可用。构建脚本显式选择本目录的 11 个包，避免把未使用演示误当作系统入口。

PCT 的 Python 接口、`common/smoothing`、GTSAM、OSQP、pybind11 不在当前 CLI 调用链中，未复制。其原 NOTICE 提及这些完整上游工程组件，已原样保留；这不是声明本工程提供或需要它们。

## 验证边界

- `verification.json` 是 2026-11-08 的核验快照，不是当前构建的实时状态；其中 `ros_compile`、`docker_build`、`simulation` 记录为 `not run`，表示该次核验没有执行，不等于“已通过”或“不可用”。
- `vendored-source-manifest.json` 的哈希只用于复制完整性/来源追溯，不是运行验收门；源码更新后必须重新核验，否则清单与磁盘漂移。
- 本目录不证明完整 ROS/Gazebo/Isaac 软件栈可运行；许可表也不构成法律结论。
