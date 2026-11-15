# RoboGauge MoE-CTS portable deployment

The required source is in this repository, not imported or linked from a training project or old experiment directory. This is a simulation deployment, not a real-robot controller. RL training stays outside this project.

本目录是 RoboGauge 部署与历史证据的**说明入口**：README 讲运行时/契约、权重与数据、历史结果与边界，同目录 JSON/许可证文件是证据快照；它不参与运行时，也没有代码读取这里的 JSON。

## 目录结构

```text
docs/robogauge/
├── README.md                              # 本文件
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
| 报告 | [`README.md`](README.md) | 部署边界、权重与数据位置、历史结果、来源与权利、共享安全门禁。 |
| 证据（迁移验证） | [`PORT_VALIDATION.json`](PORT_VALIDATION.json)、[`source_manifest.json`](source_manifest.json) | `PORT_VALIDATION`：39 个可移植源文件、19 个源归档哈希复核、0 符号链接、0 个人绝对路径、系统 CPU 回归 26 项（25 通过/1 skip），以及 `real_model_cpu_fixture`。`source_manifest`：源归档标识、逐文件 SHA256、上游模型仓库/修订与“`training_commit_verified=false`”的记录。 |
| 证据（运行/数值） | [`cpu_validation.json`](cpu_validation.json)、[`historical_execution_recheck.json`](historical_execution_recheck.json)、[`isaac_closure_audit.json`](isaac_closure_audit.json) | CPU fixture：`passed=true`、`torch 2.7.1+cpu`、30 帧、`sequential_action_max_error=0.0`、`latency_ms_p50≈0.2595`。历史复算：turn/stairs 两次 run 的同刻 policy/physics 帧匹配与零命令/力矩 delta。Isaac 闭包：`executed=false`，列出已移植纯组件与未移植 live runner。 |
| 证据（许可/资产） | [`UPSTREAM_GO2_RL_LAB_LICENSE.txt`](UPSTREAM_GO2_RL_LAB_LICENSE.txt) | 模型档案实际随附的 MIT+下游声明；README 明确“保留文件不等于获得再分发权”。 |
| 日志/输出 | 外部 `PROJECT_DATA_ROOT`/run 目录 | 本目录不写日志。历史 run 证据在外部数据卷；`historical_execution_recheck.json`/`cpu_validation.json` 是对那些证据的**离线复算记录**，不是实时日志。 |
| 代码读取点 | 无 | `grep -rn "docs/robogauge" src scripts tests` 无代码读取；本目录只被源码 README 以链接方式引用（[robot/unitree/robogauge](../../src/embodied_navigation/robot/unitree/robogauge/README.md)、[navigation/robogauge](../../src/embodied_navigation/navigation/robogauge/README.md) 等）。 |

## 模块内文件耦合

- 依赖方向：`README.md → 6 个证据/许可文件` 是索引关系，JSON 之间无 import、无生成脚本依赖。证据链是单向的：`source_manifest.json`（源哈希）→ `PORT_VALIDATION.json`（迁移核验）→ `cpu_validation.json`/`historical_execution_recheck.json`（数值/历史复算）→ `isaac_closure_audit.json`（未移植边界）。
- 与仓库代码的耦合是**文档声明级**：README 中的模型 SHA256 `bde5210a…`、`source_manifest.json` 的 `records`/字段都必须与磁盘一致；这类哈希没有自动校验，重建后可能过期。
- 隐式契约：README 的“历史结果”来自归档 run，不是本检出执行的；`PORT_VALIDATION.json` 的 `system_cpu_regression.tests=26` 与 `tests/regression/test_native_safety_integration.py` 是不同层面的检查，不能互相替代。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README.md` | 本文件 | 6 个同目录证据/许可文件 | 人；源码 README 链接 | 偏重：一份文件承载部署、历史、来源、安全门禁四类信息 |
| `PORT_VALIDATION.json` | 30 | 源归档/本机 CPU | 本 README | 清楚：字段即结论 |
| `cpu_validation.json` | 29 | CPU Torch + 模型 | 本 README | 清楚：一次 fixture 实测 |
| `historical_execution_recheck.json` | 37 | 历史 run 日志 | 本 README | 清楚：只覆盖两次历史 run |
| `isaac_closure_audit.json` | 275 | Isaac 源码闭包 | 本 README | 可接受：长但机器可读 |
| `source_manifest.json` | 175 | 源归档 | 本 README | 清楚：provenance 记录 |
| `UPSTREAM_GO2_RL_LAB_LICENSE.txt` | 262 | 上游模型档案 | 本 README | 清楚：原样保留的许可证据 |

结论：**没有代码依赖环**；耦合是文档 ↔ 证据快照的弱耦合，结构清楚。风险点是 README 同时承担“当前部署说明”和“历史结果记录”两种角色，且哈希/数字没有与证据 JSON 的自动对照。

## Functional ownership

- `robot/unitree/robogauge`: MoE-CTS motion-policy runtime (`runtime.py`), model hash verification, named joint mapping, 45D observation and Go2HV numerical contract (`contract.py`), CPU TorchScript inference and replay fixture.
- `navigation/robogauge`: the native-pose `RouteTracker` (`route_tracker.py`), a pure controller used by DynaNav capture/route code and covered by regression tests.
- `simulation` and `planning`: shared safety state, world/sensor preparation and measured PCT support grids; they are not RoboGauge deployment entry points.

## Data and weights

Set `PROJECT_DATA_ROOT` to your global external data directory. The shared project helper adds `embodied-navigation`; put the verified export at `embodied-navigation/runtime_assets/models/robogauge_policy.pt` below the global root. It must have SHA256 `bde5210a8114aa08bb0c611e49def1482faa1b2441673e5681374c30b935e32e`. Weights are excluded from Git. A data-only symlink to an existing copy is sufficient; no duplication is required. An explicit `--model` can select the same verified export elsewhere. Loading a different export is rejected, not silently accepted.

Base NumPy tests do not import PyTorch:

```sh
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s tests/regression -p 'test_robogauge*.py' -v
```

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

`UPSTREAM_GO2_RL_LAB_LICENSE.txt` is the license file actually retained with the model archive (MIT notice plus downstream notices). Its presence alone does not establish model, dataset, asset or private deployment-source redistribution rights. No new license is invented or assigned to those files. The complete frozen author source was not present in the recovered model `source_contract` directory, so provenance is limited to recorded revisions, hashes, retained license and the actual deployment source.

## Shared active simulation safety

The Student 247D loop at `robot/unitree/policy/native/worker.py` calls `control/safety/simulation_gate.py`. Simulation provides typed, explicitly marked native64 truth through `simulation/safety_state.py`. The shared `common/state_contract.py` forbids admitting that truth as a navigation/SLAM observation and rejects producer, run, clock-domain or timestamp mismatches.

The control gate owns the original fall/clearance decisions and fault latch. Policy faults precede native actuator faults, which precede the physical fall/clearance reason. The existing 1.5 s / 2 s grace times, .8 / 1.1 rad tilt thresholds, .15 / .10 m clearance thresholds and four-second persistent-fall window are unchanged. Existing faults remain latched even if a terrain query throws. Invalid provenance or nonfinite safety measurements fail closed. The original joint targets, observation dimensions, ordinary parking logic and native C++ protection remain in place.

CPU fixtures in `tests/regression/test_native_safety_integration.py` execute the Student loop with its real observation builder and a fake socket peer. They check that the shared gate is actually called every frame, fault damping precedes termination at the original grace boundary, mismatched clocks stop the exchange, and a policy fault survives a later terrain exception. This does not claim a new simulator run or real-hardware support.

## 限制

- 本目录是报告与证据，不是验收；历史 N=1 结果、CPU fixture 通过、原生 actuator 能编译都不等于 Isaac/Gazebo 整栈可运行。
- 权重不在 Git；`PROJECT_DATA_ROOT` 未配置或模型 SHA 不匹配时加载会被拒绝，不能把“缺权重”当成代码缺陷。
- 证据 JSON 是 2026-11-08/09 的快照，不随代码自动刷新；引用哈希/数字前应重新核对磁盘与 JSON。
