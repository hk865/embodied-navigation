# RoboGauge MoE-CTS portable deployment

The required source is in this repository, not imported or linked from a training project or old experiment directory. This is a simulation deployment, not a real-robot controller. RL training stays outside this project.

本目录是 RoboGauge 部署与历史证据的**说明入口**：README 讲部署/数据/构建/历史结果与边界，同目录 JSON/许可证文件是证据快照；它不参与运行时，也没有代码读取这里的 JSON。

## 目录结构

```text
docs/robogauge/
├── README.md                              # 本文件：部署说明、历史结果与边界（160 行）
├── BUILDING_SOURCE.json                   # Building 资产仓库/提交/许可记录（15 行）
├── BUILDING_UPSTREAM_PCT_GPL_2.0          # PCT 上游 GPL-2.0 文本（339 行）
├── BUILDING_UPSTREAM_PCT_NOTICE           # PCT 上游 NOTICE（27 行）
├── BUILDING_UPSTREAM_UNITREE_BSD          # Unitree BSD 许可文本（28 行）
├── PORT_VALIDATION.json                   # 迁移验证：源文件/哈希/CPU 回归（30 行）
├── UPSTREAM_GO2_RL_LAB_LICENSE.txt        # 模型档案随附许可与下游声明（262 行）
├── cpu_validation.json                    # CPU 策略 fixture 实测结果（29 行）
├── historical_execution_recheck.json      # 历史 run 的同刻命令/力矩复算（37 行）
├── isaac_closure_audit.json               # Isaac 源闭包审计：已移植/未移植（275 行）
└── source_manifest.json                   # 迁移源文件哈希与来源映射（175 行）
```

## 代码、日志与配置

本目录没有代码，也没有运行日志；按“报告 / 证据 / 配置”三类理解：

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 报告 | [`README.md`](README.md) | 部署边界、权重与数据位置、原生 actuator 构建、历史结果、来源与权利、benchmark 入口与共享安全门禁。 |
| 证据（迁移验证） | [`PORT_VALIDATION.json`](PORT_VALIDATION.json)、[`source_manifest.json`](source_manifest.json) | `PORT_VALIDATION`：39 个可移植源文件、19 个源归档哈希复核、0 符号链接、0 个人绝对路径、系统 CPU 回归 26 项（25 通过/1 skip），以及 `real_model_cpu_fixture`。`source_manifest`：源归档标识、逐文件 SHA256、上游模型仓库/修订与“`training_commit_verified=false`”的记录。 |
| 证据（运行/数值） | [`cpu_validation.json`](cpu_validation.json)、[`historical_execution_recheck.json`](historical_execution_recheck.json)、[`isaac_closure_audit.json`](isaac_closure_audit.json) | CPU fixture：`passed=true`、`torch 2.7.1+cpu`、30 帧、`sequential_action_max_error=0.0`、`latency_ms_p50≈0.2595`。历史复算：turn/stairs 两次 run 的同刻 policy/physics 帧匹配与零命令/力矩 delta。Isaac 闭包：`executed=false`，列出已移植纯组件与未移植 live runner。 |
| 证据（许可/资产） | [`BUILDING_SOURCE.json`](BUILDING_SOURCE.json) + 3 份 `BUILDING_UPSTREAM_*`、[`UPSTREAM_GO2_RL_LAB_LICENSE.txt`](UPSTREAM_GO2_RL_LAB_LICENSE.txt) | Building 资产来源（RoboNav legbot_3D_Nav、PCT 的 mesh/pcd/tomogram 路径与许可）、上游 PCT/Unitree 许可文本、模型档案实际随附的 MIT+下游声明。README 明确“保留文件不等于获得再分发权”。 |
| 配置（指向仓库真实文件） | `configs/robogauge/` | 本目录**不是**配置源。真正被代码读取的是 [configs/robogauge](../../configs/robogauge/)：`world.py` 读 `routes/<case>.json`、`deployment_profile.json`、`world.sdf.in`，并复制 `terrain_targets.json`→`terrain_target_manifest.json`/`sensor_*`；`contract.json` 是归档副本，权威常量在 [`robot/unitree/robogauge/contract.py`](../../src/embodied_navigation/robot/unitree/robogauge/contract.py)，无 Python 读取者。 |
| 日志/输出 | 外部 `PROJECT_DATA_ROOT`/run 目录 | 本目录不写日志。benchmark 运行产物与历史 run 证据在外部数据卷；`historical_execution_recheck.json`/`cpu_validation.json` 是对那些证据的**离线复算记录**，不是实时日志。 |
| 代码读取点 | 无 | `grep -rn "docs/robogauge" src scripts tests` 无代码读取；本目录只被源码 README 以链接方式引用（[robot/unitree/robogauge](../../src/embodied_navigation/robot/unitree/robogauge/README.md)、[simulation/gazebo/robogauge/motor_src](../../src/embodied_navigation/simulation/gazebo/robogauge/motor_src/README.md)、[navigation/robogauge](../../src/embodied_navigation/navigation/robogauge/README.md) 等）。 |

## 模块内文件耦合

- 依赖方向：`README.md → 10 个证据/许可文件` 是索引关系，JSON 之间无 import、无生成脚本依赖。证据链是单向的：`source_manifest.json`（源哈希）→ `PORT_VALIDATION.json`（迁移核验）→ `cpu_validation.json`/`historical_execution_recheck.json`（数值/历史复算）→ `isaac_closure_audit.json`（未移植边界）。
- 与仓库代码的耦合是**文档声明级**：README 中的模型 SHA256 `bde5210a…`、`configs/robogauge/world.sdf.in` 的 current SHA256 `e5e4bba1…`、`source_manifest.json` 的 `records`/字段都必须与磁盘一致；本轮已用 `sha256sum configs/robogauge/world.sdf.in` 复核 `e5e4bba1…` 与 README 相同（2026-11-09）。这类哈希没有自动校验，重建后可能过期。
- 隐式契约：README 的“历史结果”来自归档 run，不是本检出执行的；`PORT_VALIDATION.json` 的 `system_cpu_regression.tests=26` 与 README 提到的 `tests/regression/test_native_safety_integration.py` 是不同层面的检查，不能互相替代。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README.md` | 84 | 10 个同目录证据/许可文件 | 人；3 个源码 README 链接 | 偏重：一份文件承载部署、历史、来源、入口、安全门禁五类信息 |
| `PORT_VALIDATION.json` | 30 | 源归档/本机 CPU | 本 README | 清楚：字段即结论 |
| `cpu_validation.json` | 29 | CPU Torch + 模型 | 本 README | 清楚：一次 fixture 实测 |
| `historical_execution_recheck.json` | 37 | 历史 run 日志 | 本 README | 清楚：只覆盖两次历史 run |
| `isaac_closure_audit.json` | 275 | Isaac 源码闭包 | 本 README | 可接受：长但机器可读 |
| `source_manifest.json` | 175 | 源归档 | 本 README | 清楚：provenance 记录 |
| `BUILDING_SOURCE.json` + 3 许可文本 + 模型许可 | 15/339/27/28/262 | 上游仓库/档案 | 本 README | 清楚：原样保留的许可证据 |

结论：**没有代码依赖环**；耦合是文档 ↔ 证据快照的弱耦合，结构清楚。风险点是 README 同时承担“当前部署说明”和“历史结果记录”两种角色，且哈希/数字没有与证据 JSON 的自动对照。

## 可读性评估

- `README.md`：**可接受**。84 行、7 个 H2（Functional ownership / Data and weights / Native Gazebo actuator build / Historical results versus this port / Provenance and rights / Portable benchmark entry / Shared active simulation safety），结构清楚，且明确区分“历史归档测量”与“本次迁移”。缺口：段落偏长（多段 8–12 行）、数字密集，且没有指向同目录 JSON 证据的链接——读者要自己猜 `PORT_VALIDATION.json`/`cpu_validation.json` 对应哪段结论。
- 证据 JSON：**可接受**。字段名自解释（`passed`、`tests`、`skipped`、`skip_reason`、`model_sha256`、`training_commit_verified`），适合机器复核；缺点是文件间没有统一 schema/索引，人工阅读需逐个打开。
- 许可文件：**良好**。原样保留上游文本，README 明确“不新增许可、不等于再分发权”。

没有发现魔法数滥用：历史 N=1 结果、阈值与延迟都标注了来源/局限（“not a claim”/“not reliability statistics”）。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 README 每节末尾加“证据：`PORT_VALIDATION.json` / `cpu_validation.json` …”链接 | 同目录 10 个证据文件目前没有任何入口链接，读者只能靠猜 | 无（纯文档） | 逐条点开链接确认文件存在 |
| P2 | 把“当前可运行边界”（CPU fixture、Gazebo actuator、未移植 Isaac runner）汇总成一张三列表：已验证 / 未验证 / 历史归档 | 现在三者在不同段落，容易被摘要成“已支持 Isaac” | 无 | 人工对照 `isaac_closure_audit.json` 的 `executed=false` 与 `not_ported_live_components` |
| P2 | 在 README 与 `PORT_VALIDATION.json` 之间加一份“哈希/计数对照表”（模型 SHA、world.sdf.in SHA、39/19/26 项计数） | 文档数字与 JSON 数值靠人眼同步 | 低：只加表；数字须从 JSON/`sha256sum` 重新取 | 用 `sha256sum` 与 JSON 逐项比对 |
| P3 | README 用英文而相邻的 `docs/dependencies/README.md` 用中文；若保持英文，文首加一句“本页保留英文原文，索引/边界以中文页面为准” | 同 docs 目录语言风格不一 | 无 | 人工阅读 |

没有 P0/P1：本目录不参与运行，最坏后果是读者高估部署完成度；已有 `--check`/`--prepare-only`/`--run` 分层和 `executed=false` 证据，不需要结构调整。

## Functional ownership

- `robot/unitree/robogauge`: model hash verification, named joint mapping, 45D observation, CPU TorchScript inference, Go2HV numerical contract and replay fixture.
- `simulation/gazebo/robogauge`: native request64/response16 socket loop, contact/command journaling and the complete Gazebo Go2HV C++ actuator.
- `navigation/robogauge`: fail-closed SLAM command admission and bounded Building benchmark orchestration.
- `configs/robogauge`: readable policy and benchmark contracts.

`robot/unitree/robogauge/contract.py` holds the frozen scales, ordering, history, motor and timing assumptions as code constants; `configs/robogauge/contract.json` is a readable archival copy with **no Python reader** (do not treat it as the authority, and keep the two in sync if either changes). The policy consumes body angular velocity, projected gravity, body commands, joints and last unscaled action. It does not consume COM linear velocity, height scans or camera input. Privileged Gazebo pose/clearance remains diagnostic and safety evidence. No COM velocity PID is inserted. Gazebo keeps the archived command slew and immediate zero-command behavior; Isaac used direct commands and is not silently assigned the Gazebo slew.

## Data and weights

Set `PROJECT_DATA_ROOT` to your global external data directory. The shared project helper adds `embodied-navigation`; put the verified export at `embodied-navigation/runtime_assets/models/robogauge_policy.pt` below the global root. It must have SHA256 `bde5210a8114aa08bb0c611e49def1482faa1b2441673e5681374c30b935e32e`. Weights are excluded from Git. A data-only symlink to an existing copy is sufficient; no duplication is required. An explicit `--model` can select the same verified export elsewhere. Loading a different export is rejected, not silently accepted.

The optional CPU fixture needs public PyTorch. Base NumPy tests do not import PyTorch:

```sh
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s tests/regression -p 'test_robogauge*.py' -v
PYTHONPATH=src python3 -m embodied_navigation.robot.unitree.robogauge.cpu_fixture
```

The fixture compares thirty stateful frames with an independent raw TorchScript instance and verifies reset. It is not a physics or navigation test.

## Native Gazebo actuator build

```sh
cmake -S src/embodied_navigation/simulation/gazebo/robogauge/motor_src -B build/robogauge-actuator -DCMAKE_INSTALL_PREFIX="$PWD/install/robogauge"
cmake --build build/robogauge-actuator --parallel 2
cmake --install build/robogauge-actuator
```

Public dependencies: C++17, `gz-sim8`, `gz-plugin2` (Gazebo Harmonic). Target `robogauge_actuator` installs `install/robogauge/lib/librobogauge_actuator.so`. It deliberately retains the `teacher_sim::TeacherActuator` plugin ABI name and `TEACHER_SOCKET`/`TEACHER_ACTUATOR_LOG` environment keys used by the archived request64 transport. These names do not select Teacher weights. The source is the actual November 8 Go2HV variant, not the different Student motor implementation.

## Historical results versus this port

These are archived measurements, not new runs of the portable project:

- Gazebo single-floor turn: real FAST-LIVO2/SCAN navigation, one run, 68.22 s, original parking test passed. Native goal XY error 0.01655 m; estimated SLAM XY error 0.01452 m.
- Gazebo short stairs: one real-SLAM run, 83.46 s, parking passed; native height gain 1.4928 m. Native goal XY error 0.06353 m differs from estimated SLAM XY error 0.04558 m.
- Isaac original office first flight: 12/12 physical stair levels and stable landing support in one run; actual rises 14.59–16.46 cm. **GT RouteTracker, not SLAM navigation**. No COM velocity PID. Arrival at 68.815 s and parking end at 73.815 s. Auxiliary ROS bag cleanup was incomplete; do not claim that bag is complete.
- Teacher 015/model1000 with its original trajectory PID: 12/12 levels, strict physical completion and parked; nonfoot contact observed, so clean gait was not certified.
- Teacher Rough002_v4/model12999: **the 6.015 s standing gate failed; the stairs test never started**. Speed v95 0.10946 m/s exceeded 0.02; angular rate v95 0.20273 rad/s exceeded 0.05. The guard was not weakened.

The Isaac live runner, stage/collision restoration, recorder, same-tick effort readback and Teacher-PID comparison are not advertised as portable runnable integration. Their archived runners depended on additional historical Isaac bootstraps and scene helpers. The shared, named 45D policy, Go2HV contract and complete native-pose RouteTracker (14 unchanged behavioral tests) are ported; an actual portable Isaac runner still requires a complete audited simulation source closure, the correct public Isaac installation/assets, and separate integration verification. No empty launcher is substituted.

No simulator, ROS node, GPU job or hardware was started during this source migration. Prior N=1 successes are not reliability statistics. Gazebo and Isaac physics are not claimed equivalent to the training environment.

## Provenance and rights

See `source_manifest.json` for original file hashes and migration mapping. The archive cites upstream model repository `wty-yy/go2_rl_gym_data`, revision `d1f19f0a486e171c327d8c3da2d31dba294e6586`, and author-source freeze `6c95d9feef4814c25eb6536e61bb38e1d1d718d4`. The original navigation checkout was `5688f2868e241ca4a7cc80ba9f166f8afa84c727`. These are recorded evidence, not a claim that the model has a verified training Git SHA: the archive explicitly says it does not.

The `config_source_hashes` in `source_manifest.json` describe original archived inputs. The migrated `configs/robogauge/world.sdf.in` uses `${ASSET_ROOT}` and `${ROBOGAUGE_ACTUATOR}`; its current reviewed SHA256 is `e5e4bba139e26db9a4594cd28b4c837ac72aebdb56405f0cd9dc3fc74751d5c3` (the pre-review parameterized template was `9646f43915de6f157bbfe1a9a936b2bc17a85429c64b357b80c10c2b2391f692`, kept as the recorded baseline parameterized hash; `source_manifest.json`'s `f268c468…` remains the original archived input hash). `simulation/gazebo/robogauge/world.py` substitutes these placeholders when preparing a run. The archived hash is retained as provenance. This review did not compare the original archived bytes, so it has not established that the placeholders account for every difference.

`UPSTREAM_GO2_RL_LAB_LICENSE.txt` is the license file actually retained with the model archive (MIT notice plus downstream notices). Its presence alone does not establish model, dataset, asset or private deployment-source redistribution rights. No new license is invented or assigned to those files. The complete frozen author source was not present in the recovered model `source_contract` directory, so provenance is limited to recorded revisions, hashes, retained license and the actual deployment source.

## Portable benchmark entry

```sh
# CPU preparation only; writes inputs under the external project data root.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.benchmark --case single_floor_turn --label UNIQUE_NAME --prepare-only
# Missing data or unbuilt binaries are reported; this is not an execution test.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.benchmark --check
# Explicit ROS/Gazebo start, only after building dependencies and supplying assets.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.benchmark --case single_floor_turn --label ANOTHER_UNIQUE_NAME --run
# Offline same-time policy exchange, target and motor-envelope audit.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.analyze "$RUN_DIRECTORY"
```

This entry preserves the real-SLAM process chain: CPU policy actor, sensor bridge, camera parameters, LiDAR filter, FAST-LIVO2, global backend, localization publication, existing Building navigation, SCAN and Gazebo. It requires the repository's own compiled libraries plus public ROS/Gazebo dependencies. `NAVIGATION_ACTOR_PYTHON` or `--actor-python` selects a Python with public CPU Torch; system ROS Python is used separately. The optional historical camera recorder/video encoder and resource monitor are not necessary navigation dependencies and are not started here.

Route JSON, world template and sensor configuration are real tracked files in `configs/robogauge`. Large meshes, point clouds, numeric mesh caches, models and logs remain outside Git. `BUILDING_SOURCE.json` and associated notices preserve the recorded Building/PCT provenance. The prepared process plan is not proof that any process ran. `analyze` will not pass empty logs or pre-send inference receipts without a matching actuator exchange and physics step.

`isaac_closure_audit.json` lists the actual recovered runner imports and missing historical integration edges. `configs/robogauge/isaac_first_flight_reference.json` contains only the relevant archived scenario values and explicitly does not advertise a live runner. The pure `RouteTracker` is GT diagnostic logic, separate from the real-SLAM benchmark and from the main relative-SLAM navigation task.

## Shared active simulation safety

Both active native backends now call `control/safety/simulation_gate.py`: the original Student 247D loop at `robot/unitree/policy/native/worker.py` and the RoboGauge 45D loop at `simulation/gazebo/robogauge/native_loop.py`. Simulation provides typed, explicitly marked native64 truth through `simulation/safety_state.py`. The shared `common/state_contract.py` forbids admitting that truth as a navigation/SLAM observation and rejects producer, run, clock-domain or timestamp mismatches.

The control gate owns the original fall/clearance decisions and fault latch. Policy faults precede native actuator faults, which precede the physical fall/clearance reason. The existing 1.5 s / 2 s grace times, .8 / 1.1 rad tilt thresholds, .15 / .10 m clearance thresholds and four-second persistent-fall window are unchanged. Existing faults remain latched even if a terrain query throws. Invalid provenance or nonfinite safety measurements fail closed. The original joint targets, observation dimensions, ordinary parking logic and native C++ protection remain in place.

CPU fixtures in `tests/regression/test_native_safety_integration.py` execute both loops with their real observation builders and a fake socket peer. They check that the shared gate is actually called every frame, target outputs agree for the same actor action, fault damping precedes termination at the original grace boundary, mismatched clocks stop the exchange, and a policy fault survives a later terrain exception. This does not claim a new simulator run or real-hardware support.

## 限制

- 本目录是报告与证据，不是验收；历史 N=1 结果、CPU fixture 通过、原生 actuator 能编译都不等于 Isaac/Gazebo 整栈可运行。
- 权重不在 Git；`PROJECT_DATA_ROOT` 未配置或模型 SHA 不匹配时加载会被拒绝，不能把“缺权重”当成代码缺陷。
- 证据 JSON 是 2026-11-08/09 的快照，不随代码自动刷新；引用哈希/数字前应重新核对磁盘与 JSON。
