# 工程流程修复与重构实施记录（2026-11-09）

本文件记录在 [工程流程导读与可读性整理](dsh-workflow-and-readability-20261109.md)（下称"导读报告"）之后**实际实施**的修改。

> **后续进展（2026-11-09 同日第四轮）**：本文 §3 中"拆分 `navigation/pid_navigation.py`"与"把 `capture_go2_rgbd.py` 整体包进 `main()`"两项已在独立结构重构轮实际实施；`capture_go2_rgbd.py` 的 `RuntimeError` 导入守卫已被移除（普通 import 无副作用 + 显式 `run(argv)`/`main()`），`experiment.py` 的源码快照也随拆分补上 `capture/*.py`。详见 [现役 controller 与 DynaNav 采集入口结构重构](dsh-structural-refactor-20261109.md)。本文以下内容保留为当轮记录，不再代表当前工作树。导读报告提出的是候选问题；本轮的规则是**先顺着调用关系确认，能安全改的就改，判断不成立的在文中更正，不适合本轮处理的写清理由**。

## 0. 边界与未做事项

- 只做代码/文档修改与**离线 CPU 单元/回归检查**；没有启动仿真、Isaac、GPU 推理、训练或机器人，没有录包。
- 没有 `git add/commit/push`，没有改 DSH/API/网络设置，没有改文件权限。
- 工作树原有的未提交成果（在线感知、person worker、learned mapping 等）按磁盘内容保留，未回退到 HEAD `43cba33`，未清理数据。
- 数值没有为了"看起来统一"而跨层统一；不同控制层/时间域的取值差异改为**在文档中显式列出**。

## 1. 已实施的修改

### 1.1 入口与实际状态机的对应关系（导读报告 §2、B-1/B-2）

| 文件 | 修改 |
| --- | --- |
| `docs/architecture.md` | 新增"四条执行路径"表（A native / B building / C robogauge / D DynaNav）与"默认主路径的任务状态机"小节；改正第 4 条把 `task.py` 写成现役状态机、第 6 条把 `simulation/dynanav` 写成只做 capture。 |
| `README.md` | 系统结构第 9/15 条与"两个运行边界"第 2 条改为：现役状态机是 `base_node.Navigation.control()`（`ros_node` 包装、`pid_navigation` 装配）；`task.py` 只服务独立 `relative-slam-navigation`；`mission/state_machine.py` 本检出未实例化；DynaNav 当前入口是 `run_dynanav_experiment.py` + `run_dynanav_lio.sh` + `run_dynanav_person_worker.py`。 |
| `src/embodied_navigation/navigation/README.md` | 新增"入口、节点类与进程角色"表（controller / mission / Building / relative-SLAM 四行），标明谁在 `demo.launch.py` 中；`task.py` 条目标注"不在默认 launch"。 |
| `src/embodied_navigation/navigation/mission/README.md` | 把 `state_machine.py` 标注为历史迁移实现、无实例化点；`continuous_route_contract.enabled=false`；测试段改为"`mission_runtime.py` 进入默认主路径"。 |
| `src/embodied_navigation/control/README.md` | "不做任务级状态机"改为指向 `base_node.Navigation.control()` / `mission_runtime.py` / `task.py` 三者各自的边界。 |
| `src/embodied_navigation/navigation/task.py` | 模块 docstring 增加作用域说明：仅 `relative-slam-navigation` 入口，改它不影响默认导航行为。 |
| `src/embodied_navigation/navigation/mission/state_machine.py` | 模块 docstring 增加"本检出未实例化"说明。 |
| `src/embodied_navigation/navigation/base_node.py` | 模块 docstring 明确它拥有现役 `Navigation.control()` 状态机，以及默认 launch 经 `ros_node`/`pid_navigation` 到达它。 |
| `src/embodied_navigation/navigation/pid_navigation.py` | 模块 docstring 增加入口顺序（controller → `main` → `IntegratedNavigation` → … → `base_node`）与"默认关闭的是观察器 hook，wrapper 仍调用原 predicate/admission"的说明。 |

**行为影响**：仅 docstring/文档，无运行语义变化。

### 1.2 超大文件混杂职责：先做低风险映射，不拆文件

| 文件 | 修改 |
| --- | --- |
| `pid_navigation.py`（4702 行） | 在模块 docstring 写清三层职责（状态机在 `base_node`、本文件加准入/级联限速/停车归属/证据），并说明历史 probe wrapper 默认关闭的只是观察器 hook、wrapper 仍调用原 predicate/admission 并可附失败证据（第二轮修正措辞，见 §6.7）。**未拆分文件**，理由见 §3。 |
| `mission/mission_runtime.py` | 未改代码；职责在 mission README 与 architecture 中写明（只发布目标/参考，不产生速度）。 |

### 1.3 CLI/查询/模型客户端的归属（导读报告 B-7/B-8）

| 文件 | 修改 |
| --- | --- |
| `src/embodied_navigation/localization_mapping/semantic_mapping/query.py`（**新增**） | 把 `select` / `context_text` 从 `scripts/query_scene_map.py` 移入包内，成为在线 learned 建图与离线重建共享的库函数（纯函数，无 ROS/模型依赖）。 |
| `scripts/query_scene_map.py` | 变为薄 CLI：解析参数、`read_snapshot`、格式化输出；仍**重新导出** `select`/`context_text`，现有按路径加载该脚本的测试与调用方不需要改。 |
| `src/embodied_navigation/simulation/dynanav/learned_mapping.py` | 删除运行时 `importlib.util.spec_from_file_location` 去加载 `scripts/query_scene_map.py` 的写法，改为直接 `from ...semantic_mapping import query`。 |
| `scripts/reconstruct_dynanav_learned_person_map.py` | 同上改为直接导入；`load_script` 仅保留给 slam-map 回放脚本。 |
| `src/embodied_navigation/perception/person_worker.py` | docstring 明确：本模块是 IPC 客户端 + 事件策略，不加载模型；模型所有者是 `scripts/run_dynanav_person_worker.py`。 |
| `src/embodied_navigation/perception/README.md` | 同一条边界写进模块 README，并说明 `person_worker` 不依赖 `ROS_DOMAIN_ID`。 |

**行为影响**：查询结果与 CLI 输出格式不变；调用方由"运行时加载 .py 脚本"变为"包内导入"，减少一条隐式依赖脚本路径的链路。

### 1.4 重复参数与隐式配置覆盖（导读报告 B-3/B-4）

| 文件 | 修改 |
| --- | --- |
| `src/embodied_navigation/launch/prepare.py` | 新增 `_pin()`：六个启动器自有键（`daily_navigation`、`cascade.daily_navigation`、`r85_auto_reference_contract.reference_file`、`experiment`、`engineering_recording.raw_source_capture`、`engineering_recording.registered_cloud_archive`）仍强制写入，但**与配置不一致时向 stderr 打印旧值/新值/原因**，不再静默覆盖。`runtime_context` 的 schema 未改。 |
| `configs/launch/settings.json` | `R107_SCAN_TRANSPORT_PROBE`、`MATCH_QUERY_CAPTURE` 由 `"1"` 改为 `"0"`，与 `launch/main.py` 的强制值一致。已确认 `src/`/`scripts/` 中除 `main.py` 的强制列表外**没有其它读取点**，`building.py`/`benchmark.py` 也不注入 `features`，因此该改动不改变任何路径的行为。 |
| `configs/README.md`（**新增**） | 写明配置读取顺序、prepare pin、被启动器覆盖的键、无读取点的归档数据（`robogauge/contract.json`、`scenarios/terrain_targets/`），以及"不要手改 runs/ 里的生成参数"。 |
| `README.md`、`src/embodied_navigation/README.md`、`docs/README.md` | 链到新的配置/测试/术语页。 |

**行为影响**：`--prepare-only`/普通启动在配置与 pin 不一致时多输出 stderr 行；其余不变。若某配置确实把这两个探针设为 `"1"` 期待生效，现在会在源码层面对齐为 `"0"`（此前运行期也已是 `"0"`）。

### 1.5 在线流程编排分散（导读报告 A-9/D-4）

| 文件 | 修改 |
| --- | --- |
| `scripts/run_dynanav_online_person.py`（**新增**） | 把在线行人实验的三终端顺序（person worker → LIVO observer → capture）收成一个可检查、可监督的入口：<br>• `--print-commands`：只打印等价的 shell 命令（含环境变量、worker 通道、observer namespace/snapshot 参数、capture 绑定 `--pose-launch-receipt`），不启动任何进程；<br>• `--check`：只核对仓库脚本、解释器、权重/数据目录/二进制前缀/引用文件与新输出目录，不启动 ROS/Isaac/模型，退出码 0/2；<br>• 实际运行：按序 `Popen`，等 worker `state.json` 为 `ready`、observer `launch.json` 落盘后再启动 capture；成功、失败、超时或 SIGINT/SIGTERM 时先停 capture、再逆序停 observer/worker，逐进程日志写到 `<run>/logs/`，并写 `supervision.json`（argv/pid/退出码/错误/是否由监督停止、`ok`、范围声明"只是进程监督，不是导航/感知验收"）。第二轮已重写收尾与信号处理，见 §6.1–§6.3。 |
| `tests/unit/test_dynanav_online_supervisor.py`（**新增**） | 第一轮 5 项只读测试；第二轮扩展为 12 项真实 fake 子进程用例；第三轮再补 5 项成功判定用例（capture=0 但 observer/worker 已自行退出时不得 ok，见 §6.10）。 |
| `src/embodied_navigation/simulation/dynanav/README.md`、`README.md` | 启动与检查一节、DynaNav 路径说明处加入 supervisor 及其"只打印/只核对"入口，并重申它与手工三终端是同一编排、不替代验收。 |

**行为影响**：纯新增脚本/文档；原三终端手工命令仍可直接使用。

### 1.6 文档事实与导航修正

| 文件 | 修正 |
| --- | --- |
| `docs/glossary.md`（**新增**） | 术语对照：四条路径、run 目录、provenance、`navigation_ground_truth_used`、canonical 反馈、raw/corrected odom、oracle/learned、post-VIO、LIO/LIVO、默认状态机 vs `task.py`/`state_machine.py`、admission/fence、hold/resume、native64/247D/exact45、RoboGauge 45D、命令文件 vs teacher socket、速度上限分层、features/R 编号、时钟域、`ROS_DOMAIN_ID`/`DYNANAV_RUN`。 |
| `tests/README.md`（**新增**） | 测试命令、目录职责、`discover -t .` 会报 `ImportError` 的原因、源码/AST 解析型测试的注意点、skip 不等于通过、当前计数与日期。 |
| `src/embodied_navigation/simulation/dynanav/README.md` | 发布话题补上 `/dynanav/lidar_perception_filtered`（learned 模式）与 `lidar_static`（仅 oracle）；`capture_go2_rgbd.py` 条目写明直接 `import` 会抛错、入口用 `runpy`。 |
| `src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py` | 顶部加导入守卫：以 `__main__` 执行（`runpy`/直接运行）行为不变；被当作模块 import 时立即抛 `RuntimeError` 说明原因，而不是先执行采集副作用（读 `sys.argv[1]`、开 USD）。 |
| `src/embodied_navigation/localization_mapping/README.md` | 澄清同名 odom 话题：`odom_adapter` 默认发布 `/demo/slam/{body,lidar}_odom`，默认 launch remap 成 `/demo/slam/raw_*`；corrected 同名话题由 `localization_runtime` 发布（已对照 `localization_mapping/launch/components.py` 的 `-r` 参数核实）。 |
| `src/embodied_navigation/localization_mapping/frontend/fast_livo2_ros/README.md` | 更正"两个文件逐字节相同"：只有 `scripts/decompress_images.py` 逐字节相同，`launch/mapping_aviz.launch.py` 多了说明 docstring。 |
| `src/embodied_navigation/localization_mapping/backend/run.py` | docstring 的模块路径由 `localization.backend.run` 改为 `embodied_navigation.localization_mapping.backend.run`。 |
| `src/embodied_navigation/planning/admission/README.md` | 澄清未闭合的是 `FrozenFence` **fallback**；默认启用 `r85_auto_reference_contract` 时走 `AutomaticRouteFence` + `R85PhysicalPathGate`。 |
| `src/embodied_navigation/robot/unitree/README.md` | 把单一 `LIMITS=(.3,.2,.3)` 改为**分层速度上限表**（cascade 0.6/0.2/0.6、bridge fallback 0.3/0.2/0.3、Student `command_bounds` 0.3/0.2/0.3、RoboGauge 0.6/0.2/0.3、`base_node.max_speed` 0.12），并明确"数值不同是设计，不要为统一而改值"。 |
| `src/embodied_navigation/perception/README.md` | `depth.py` 条目写明返回 `points_in_imu_frame`（原点相对 base 差 `imu_mount` 安装偏移、相机安装点用 `camera_mount` 标定，**不是 COM 偏置**），包围盒按 base 帧判断；`person_worker.py`/`run_dynanav_person_worker.py` 的模型所有者归属写清。 |
| `src/embodied_navigation/simulation/README.md` | `dynanav` 职责由"只做 capture"改为场景接入 + capture + oracle/learned 建图 + 实测位姿路线 + 动态行人。 |
| `docs/robogauge/README.md` | `contract.json` 标注为归档副本、无 Python 读取方，权威常量在 `robot/unitree/robogauge/contract.py`。 |
| `configs/scenarios/README.md` | `terrain_targets/lower12.json` 标注"当前无代码读取"，并说明 `mission46.py` 的 provider 默认不构造。 |
| `src/embodied_navigation/simulation/terrain/mission46.py` | 在历史 manifest 路径处加注释：`tests/terrain_targets/` 本检出不存在、`upper23.json` 缺失、provider 只在开关启用时构造。**未改路径值**（见 §3）。 |
| `scripts/run_dynanav_lio.sh` | usage 中 `--observer-namespace` 示例由 `/dynanav/lio_comparison` 改为默认 `/dynanav/lio_observer`。 |
| `docs/README.md` | 索引加入术语页、本轮修复记录、测试/配置入口。 |
| `docs/review/dsh-workflow-and-readability-20261109.md` | 顶部加"后续进展"链接到本文件；不重写其历史结论。 |

## 2. 导读报告中被更正/未成立的判断

- **"`navigation/task.py` 是任务状态机"**：作为模块描述成立，但把它当作**默认 launch 的状态机**不成立。默认路径是 `base_node.Navigation.control()`；`task.py` 只服务独立 `relative-slam-navigation`。已在 architecture/README/navigation README 三处更正。
- **"`mission/state_machine.py` 参与默认任务迁移"**：不成立，本检出没有实例化点（`mission_runtime.py` 才是默认 `mission` 进程）。已改为"历史实现/未实例化"。
- **"`simulation/dynanav` 只做 capture"**：不成立，它还承载 oracle/learned 建图、实测位姿路线控制与动态行人；已更正。
- **"speed limits 应统一"**：不采纳。各层数值不同是职责不同（控制器输出界 / 命令文件准入 / 策略动作界 / 另一条部署 / 发布前参数），本轮只把它们列成表并注明不可统一。
- **"把 `contract.json` 变成权威"**：未实施。它是归档副本，代码常量在 `contract.py`；本轮只把"谁是权威"写清，未改读取路径（见 §3）。
- **"`mission46` manifest 路径写错，改到 configs 即可"**：只部分成立。`tests/terrain_targets/` 确实不存在，但 `upper23.json` 在 `configs/scenarios/terrain_targets/` 也不存在，只改前缀会造成"改了一半仍不可用"的假修复；且 provider 默认不构造。本轮只加事实注释。
- **"`capture_go2_rgbd` 应包 `main()`"**：本轮用**导入守卫**替代，未整体缩进重构（理由见 §3）。
- **"configs 在 prepare 之后一律不再生效"**（第一轮 `configs/README.md` 第 3 步）：不成立。`ros_node`/`bridge`/`mission_runtime`/`localization_runtime`/`sensor_gate` 读 run 的 `runtime_context`，但 `StudentRuntime` 仍 `read()["student_runtime"]`、building/benchmark 启动器仍读 configs、SCAN supervisor 读 `navigation_profile.json`；已按实际消费者重写（见 §6.6）。
- **"`_r47_*` 等 wrapper 默认 no-op"**：过强。默认关闭的是观察器 hook，wrapper 仍调用原 predicate/admission 并可能附加失败证据；已更正 docstring（见 §6.7）。
- **"`mission46_required` 与 `terrain_layer_switch` 都选 `MissionTerrainProvider`"**：不成立。`mission46_required` 单独选择 `MissionTerrainProvider`；只有 `terrain_layer_switch` 时选通用 `SwitchingTerrainProvider`；已更正注释（见 §6.7）。

## 3. 未实施项与原因

| 项目 | 原因 |
| --- | --- |
| 拆分 `navigation/pid_navigation.py`（4702 行） | ~~当轮未做~~ **第四轮已实施**：按 ingress/admission/admission_apply/route/control 拆成 mixins + `pid_support.py` + `pid_entrypoint.py`，`pid_navigation.py` 保留类组合与旧导入/`-m` 入口；`test_ros_shutdown_lifecycle.py` 的源码解析路径同步迁移。见 [结构重构记录](dsh-structural-refactor-20261109.md)。 |
| 把 `capture_go2_rgbd.py` 整体包进 `main()` | ~~当轮未做~~ **第四轮已实施**：改为 479 行显式入口（`run`/`main`/`parse_config`/`run_capture`），实现拆到 `capture/` 包；导入守卫改为"普通 import 无副作用"，`runpy`/argv/`DYNANAV_EXPERIMENT` 契约保持不变。见 [结构重构记录](dsh-structural-refactor-20261109.md)。 |
| 解决 `policy/worker.py` 中 232 vs 247 维不一致 | 需要策略/观测契约的领域确认，无法仅从调用关系判定哪边是错；改错会直接破坏策略输入。保留为已知未决项。 |
| 删除无读取点的历史配置/兼容结构 | `robogauge/contract.json`、`scenarios/terrain_targets/`、部分 `features` 键仍被文档或历史报告引用；删除会让既有证据链断链。本轮改为标注"归档/无读取点"。 |
| 统一 worker 权重 SHA 的单一来源 | 该 SHA 同时出现在 `experiment.py` 的校验、supervisor 默认值、模块 README 与历史实验记录；建立单一来源需要同时改校验逻辑与已冻结的实验记录，超出本轮安全范围。 |
| 把 `--imu-hz`/capture legacy stride 合并成一个参数 | 两者属于不同入口（`experiment.py` 的 100/200 与旧 capture 的 `stride=2` fallback），合并会改变旧 capture 的复现语义；本轮只在模块文档中说明默认值差异。 |
| 实际启动 supervisor/Isaac/ROS 验证 | 用户明确要求继续暂停仿真、GPU 推理、训练与机器人运行。supervisor 的监督逻辑已用真实 fake 子进程覆盖生命周期（含 timeout/early-exit/SIGTERM），但不启动真实 ROS/Isaac；真实三终端接线仍未执行（见 §6.9）。 |

## 4. 已执行的检查与结果

全部为离线 CPU 检查，未启动仿真/ROS/Isaac/GPU。

```sh
# 1) 单元与回归（仓库根目录）
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
# 2) 语法/编译
python3 -m py_compile <全部改动过的 .py>
bash -n scripts/run_dynanav_lio.sh
# 3) supervisor 只读入口（--check/--print-commands 允许缺 capture-result/slam-reference）
python3 scripts/run_dynanav_online_person.py --print-commands --output /tmp/... --data-root ...
python3 scripts/run_dynanav_online_person.py --check --output /tmp/... --data-root ...
# 4) supervisor 生命周期（真实 fake 子进程，不启动 ROS/Isaac）
PYTHONPATH=src python3 -B -m unittest tests.unit.test_dynanav_online_supervisor -v
# 4) capture 导入守卫的两条路径
python3 -c "import runpy; runpy.run_module('embodied_navigation.simulation.dynanav.capture_go2_rgbd', run_name='__main__')"
python3 -c "import embodied_navigation.simulation.dynanav.capture_go2_rgbd"
```

结果（基线 → 修复后）：

| 检查 | 基线（本轮开始前） | 修复后 |
| --- | --- | --- |
| unit | `Ran 289 tests ... OK (skipped=3)` | `Ran 311 tests ... OK (skipped=3)`（第三轮后；第一轮 297、第二轮 306） |
| regression | `Ran 86 tests ... OK (skipped=5)` | `Ran 86 tests ... OK (skipped=5)` |
| `py_compile`（改动文件） | — | 通过 |
| `bash -n scripts/run_dynanav_lio.sh` | — | 通过 |
| supervisor `--print-commands` / `--check` | — | 打印三命令/返回 2（当前环境无外部数据）；单元测试覆盖 0/2 两种，且都不创建 run 目录 |
| capture `runpy` 路径 | 执行到 `sys.argv[1]` | 同前（`IndexError`，说明守卫未拦截） |
| capture `import` 路径 | 立即产生采集副作用/崩溃 | 立即 `RuntimeError`，消息说明原因 |

新增测试：`test_dynanav_online_supervisor.py`（17 项，其中 7 项运行真实 fake 子进程、5 项覆盖成功判定）、`test_launch_prepare_pins.py`（3 项）、`test_dynanav_source_snapshot.py`（2 项，锁定 `query.py` 进入 DynaNav 快照与重建 provenance）。skip 归因：unit 3 项需要本机 USD 运行时/人形资产（`LocalHumanUSDTests`）；regression 5 项是可选 CPU Torch 未安装。未新增依赖。

## 5. 行为可见变更与复核建议

行为可见的点只有五处，其余为文档/docstring：

1. `prepare.py` 在 pin 与配置不一致时向 **stderr** 输出一行（配置与 pin 一致时无输出）。
2. `configs/launch/settings.json` 两个探针键 `"1"→"0"`（运行期本就强制 `"0"`；已确认无其它读取点）。
3. ~~`capture_go2_rgbd.py` 被 `import` 时抛 `RuntimeError`~~（第四轮已改为普通 import 无副作用 + 显式 `run`/`main`；以 `__main__`/`runpy` 执行契约不变）。
4. 新增 `scripts/run_dynanav_online_person.py`（不运行则无影响）。第二轮后：子进程日志写到 `<run>/logs/`；`supervision.json` 升为 v3（含 error/interrupted/ros_setup/stopped_by_supervisor/unexpected_exits）；SIGINT/SIGTERM 会先回收子进程再返回 128+信号；`--ros-setup/--no-ros-setup` 决定是否 source ROS；`--check/--print-commands` 允许缺省 `--capture-result/--slam-reference` 且不创建 run 目录。
5. `experiment.py` 的源码快照与重建 provenance 多记录一个 `semantic_mapping/query.py`（内容哈希/清单条目增加；旧 CLI 仍保留）。

建议独立复核时优先：

```sh
git status --porcelain
git diff -- src/embodied_navigation/launch/prepare.py configs/launch/settings.json \
  src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
PYTHONPATH=src python3 -B -m unittest tests.unit.test_dynanav_online_supervisor -v   # 真实 fake 子进程
python3 scripts/run_dynanav_online_person.py --help
```

（`git diff` 只取前两个路径示例，新增/未跟踪文件用 `git status` 查看。）

## 6. 第二轮独立复核后的修正

复核确认了第一轮的主体方向（query 提取的函数 AST/样本/CLI 输出、person_worker 执行逻辑、capture 的 runpy 入口），同时指出 8 处需要修正的问题。本节按复核编号记录第二轮改动；约束不变：离线 CPU 检查、无仿真/GPU/训练/机器人、不提交 Git、不改权限。

### 6.1 supervisor 的进程收尾（对应复核 1）

- capture timeout、子进程提前失败、运行时 worker/observer 提前退出、SIGINT/SIGTERM 统一走 `finally`：按 **capture → observer → worker** 逆序停全部已启动子进程；`stop()` 先向进程组发 SIGINT、超时再 SIGKILL，并记录 `stopped_by_supervisor` 与最终退出码。
- `wait_ready()` 同时轮询子进程：启动等待期子进程退出立即报 `ChildExit`，不再空等到超时；`monitor_capture()` 同时轮询 worker/observer，任一提前退出即失败并回收 capture。
- `ok` 判据改为：capture 退出码为 0 **且**无 error、无中断；capture 正常退出但 worker/observer 已提前死亡时为 False。
- 每个子进程的 stdout+stderr 合并写到 `<run>/logs/<name>.log`，不再丢到 `/dev/null`。
- 测试从"只查命令字符串"扩展为真实 fake 子进程生命周期（见 §6.9）。

### 6.2 `--check`/`--print-commands` 的缺项（对应复核 2）

- `build_plan()` 允许 `--capture-result` / `--slam-reference` 缺省，argv 中用 `<required: ...>` 占位；`--check` 把两项列为 `present=false` 并返回 2，`--print-commands` 打印带占位的命令；两者都**不创建** run 目录。
- 真正运行且缺项时由 argparse 报错（退出 2），不再是 `Path(None)` 的 `TypeError`。

### 6.3 数据目录与 ROS 环境（对应复核 3）

- `child_environment()` 强制 `PROJECT_DATA_ROOT = --data-root 的父目录`，与 `--print-commands` 导出的值一致；测试用 fake worker 落盘环境验证。
- 新增 `--ros-setup`（默认 `/opt/ros/jazzy/setup.bash`）与 `--no-ros-setup`：实际监督前由 `bash -c 'set -e; source ...; env -0'` 解析环境并传给全部子进程；setup 缺失/失败显式报错，不静默继承；`--check` 增加 "ROS setup" 项。
- `--print-commands` 在 `cat > snapshots.yaml` 前输出 `mkdir -p <run>/livo`。

### 6.4 源码快照与 provenance（对应复核 4）

- `experiment.py` 的 `sources`、`reconstruct_dynanav_learned_person_map.py` 的 `source_files`：保留 `scripts/query_scene_map.py`（兼容 CLI），并加入真正执行的 `src/embodied_navigation/localization_mapping/semantic_mapping/query.py`（进入 source_snapshot/哈希清单）。
- 新增 `tests/unit/test_dynanav_source_snapshot.py` 锁定这两处清单。

### 6.5 prepare pin 的理由（对应复核 5）

- `raw_source_capture=False`：只关闭**可选的原始源归档**；Native 仍使用实时 Gazebo→ROS 传感器链。
- `registered_cloud_archive="disabled"`：只关闭**额外的 registered-cloud 历史归档**；`map_archive` 角色仍在运行。理由字符串已按此改写。

### 6.6 `configs/README.md`（对应复核 6）

按实际消费者重写读取顺序：启动器 `read()`/`path()` + prepare pin；`runtime_context` 消费者（`ros_node`、`bridge`、`mission_runtime`、`localization_runtime`、LOCALIZATION launch components、`sensor_gate`）；仍直接 `read()` 的 `StudentRuntime`、building/benchmark 启动器；直接读 `navigation_profile.json` 的 SCAN supervisor。并标明 `control/relative_pid.yaml`、`navigation/relative_shuttle.yaml`、`robot/unitree_usd_manifest.json`、`runtime_assets_manifest.json` **当前无 Python 读取点**，不再写成"已接入启动检查/独立入口"。

### 6.7 docstring/注释（对应复核 7）

- `pid_navigation.py`：默认关闭的是**观察器 hook**，wrapper 仍调用原 predicate/admission 并可能附加失败证据；不把整组 wrapper 说成可丢的空操作。
- `mission46.py`：`mission46_required` 单独选择 `MissionTerrainProvider`；只有 `terrain_layer_switch` 时选通用 `SwitchingTerrainProvider`；默认 profile 两者都不启用。

### 6.8 文档事实同步（对应复核 8）

- `docs/architecture.md` 的 DynaNav 行：加入可选统一入口 `run_dynanav_online_person.py`（只监督三进程顺序与收尾，不替代手工命令/验收）。
- 根 `README.md`：`scripts/capture_dynanav.py` 不是"只保留 help/check"，非 `--check` 参数仍会启动采集。
- `perception/README.md`：IMU 与 base 的偏移是 `imu_mount` **安装偏移**（相机为 `camera_mount`），不是 COM 偏置。
- `simulation/README.md`：改为"生产整栈与实机未验收"，不再笼统说 Isaac 完整路线未运行。
- `tests/README.md`：单文件示例改为实际存在的 `tests.unit.test_layered_mapping.QueryCliTests.test_invalid_filter_arguments_fail_without_traceback`；skip 归因改为 unit=本机无 USD 运行时/人形资产（3 项）、regression=可选 CPU Torch 未安装（5 项）。
- 本文 §4 的计数、skip 归因和 supervisor 描述同步更新。

### 6.9 第二轮检查与剩余边界

- unit `Ran 311 tests ... OK (skipped=3)`；regression `Ran 86 tests ... OK (skipped=5)`，两套均通过。
- supervisor 17 项测试：7 项运行真实 fake 子进程（成功收尾、capture 失败、capture timeout 时停止 capture、worker 启动前退出、observer 运行中退出、SIGTERM 中断后回收、`PROJECT_DATA_ROOT`/ROS setup 透传），5 项覆盖 `monitor_capture`/`final_verdict` 的成功判定（capture=0 不掩盖已死亡的 worker/observer），其余为只读模式与命令接线。
- 仍未执行：真实 ROS/Isaac 三终端运行、GPU 推理、训练、机器人；fake 子进程测试不等于真实编排验收，也不证明 worker/observer/capture 之间的话题/时间契约。剩余边界与第一轮相同（§3）。

### 6.10 第三轮复核后的成功判定修正（capture=0 掩盖子进程失败）

- 复核用真实 `Child` 封装 fake process 复现：capture 退出 0、observer 已退出 7（或 worker 已退出 9）时，旧 `monitor_capture()` 先 poll capture 就返回 0，`supervise()` 的 `ok` 只看 capture 退出码，于是错误地 `ok:true`。
- `monitor_capture()` 改为**先 poll worker/observer，再 poll capture**：同一轮里 capture 与支持进程都已退出时，先报 `ChildExit`，不会因 capture 恰好为 0 而提前返回。
- 新增 `final_verdict(capture, children, error, interrupted)` 统一判定：`ok` 要求 capture 退出码 0、无 error/中断，且 **worker/observer 没有非监督器主动停止的退出**；这类退出写入 `supervision.json` 的 `unexpected_exits` 并转成 `error`，schema 升为 v3。正常完成后由 `stop()` 主动 SIGINT 收尾的 worker/observer 带 `stopped_by_supervisor=true`，不计为失败。
- 回归测试（5 项）：真实 `Child` + 两个已退出的 fake 进程验证 `monitor_capture` 不再掩盖 observer=7；`final_verdict` 覆盖“正常 SIGINT 收尾 ok”“capture=0+observer=7 不 ok”“capture=0+worker=9 不 ok”“capture 非 0/中断/error 不 ok”。
- 小收尾：`prepare._pin` 的 r85 理由改为“prepare 固定文件名并写入 profile/context，`global_reference.json` 实体由 `mission_runtime` 生成”；新增 supervisor 脚本按仓库入口惯例设置可执行位（单文件 `chmod 755`，未批量改权限），消除 Ruff `EXE001`（shebang 存在但不可执行）。
- 本轮后：unit `Ran 311 tests ... OK (skipped=3)`；regression `Ran 86 tests ... OK (skipped=5)`。仍未运行真实 ROS/Isaac/模型。

差分中如出现与本文不符的改动，或 `query.py` 提取后在线/离线查询结果有差异，以复核为准；本文不宣称已完成仿真、Isaac、GPU、训练或实机验证。

## 7. Codex 最终独立核查

独立复现、退回修正和最终检查见 [核查记录](dsh-workflow-independent-review-20261109.md)。Codex 收尾补正了配置读取范围、测试依赖与本页的 wrapper 摘要，并完成不改变行为的 Ruff 清理；真实仿真继续暂停。
