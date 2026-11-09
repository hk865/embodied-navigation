# 配置入口与生效顺序

`configs/` 是 base 配置与契约数据，不是每次运行的有效值。普通运行先读 [launch/navigation.json](launch/navigation.json) 列出的六个模块配置，再由入口生成本次参数写入 `runs/`（DynaNav 写外部数据卷的 `experiments/`）。

## 目录结构

```text
configs/
├── common/                # 运行目录/通信/遥测：settings.json（39 行）
├── control/               # cascade 速度界与停车：settings.json（567 行）+ relative_pid.yaml（10 行，无读取点）
├── launch/                # 模块清单与特性开关：navigation.json（10 行）+ settings.json（70 行）
├── localization_mapping/  # 前端/后端 base：settings.json（80 行）+ fastlivo.yaml（128 行）+ camera.yaml（17 行）
├── navigation/            # relative-shuttle 保留配置：relative_shuttle.yaml（8 行，无读取点）
├── planning/              # SCAN/PCT 与静态支撑：settings.json（53 行）+ static_support_limits.json（23 行）
├── robogauge/             # 策略/benchmark 契约与路线：14 文件、22,816 行（含 world.sdf.in 与 routes/）
├── robot/                 # 资产清单预留：unitree_usd_manifest.json（47 行，无读取点）
├── scenarios/             # 场景/地形数据（4 文件、178 行），见 scenarios/README.md
├── simulation/            # 世界/物理/传感器与 DynaNav 场景（10 文件、4660 行）：settings.json（3294 行）+ 2 个 collision JSON + dynanav/ 子目录（7 文件、693 行，其中 2 篇 README 共 435 行）
├── visualization/         # RViz 配置（2 文件、181 行），见 visualization/README.md
├── README.md              # 本文件：读取顺序、pin、消费者与边界
└── runtime_assets_manifest.json # 归档资产清单（232 行，当前无代码读取）
```

合计 44 个文件（`find configs -type f | wc -l`，2026-11-09 实测）。

## 代码、日志与配置

本目录是**配置与契约数据**，不含可执行代码；按“配置读取点 / 运行产物 / 配置项语义”三问回答：

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（读取点） | [`common/config.py`](../src/embodied_navigation/common/config.py) | `read()` 读 `configs/launch/navigation.json` 的 `modules` 列表，按顺序合并六个 JSON；同层重复键报 `ValueError`、越界路径报 `ValueError`、`${PROJECT_ROOT}` 展开、`path()` 解析资产路径。每次调用都重读（无缓存）。 |
| 代码（生成器） | [`launch/prepare.py`](../src/embodied_navigation/launch/prepare.py)、[`simulation/world/prepare.py`](../src/embodied_navigation/simulation/world/prepare.py)、[`perception/sensors/sampling.py`](../src/embodied_navigation/perception/sensors/sampling.py)、[`planning/scan_runtime_parameters.py`](../src/embodied_navigation/planning/scan_runtime_parameters.py) | `prepare.py` 深拷贝 profile、执行 pin（见“读取顺序”第 2 条），再生成 world/外参/SLAM 覆盖/SCAN 参数/路线并写 run 目录。 |
| 代码（运行期消费者） | 见“读取顺序”第 4 条 | 分“只读 runtime_context 的业务 profile”和“仍直接重读 configs”两类，不能一概说“configs 不再生效”。 |
| 日志/输出 | `runs/<run_id>/`（软链到外部数据卷） | 本目录**不写**任何文件。一次运行生成 `runtime_context.json`、`navigation_profile.json`、`navigation_scenario.json`、`navigation_fastlivo.yaml`、`navigation_camera.yaml`、`terrain_target_manifest.json`、`known_scene_reference.json`、`v34_scan_overrides.yaml`、`global_route.json`、`world.sdf` 等；这些是本次证据，不是可编辑配置。全局产物布局见 [readability 报告 §9](../docs/review/dsh-workflow-and-readability-20261109.md)。 |
| 配置（本目录） | 见“目录”表 | 只有部分文件有代码读取点；无读取点的文件（`control/relative_pid.yaml`、`navigation/relative_shuttle.yaml`、`robot/unitree_usd_manifest.json`、`runtime_assets_manifest.json`、`scenarios/terrain_targets/lower12.json`、`simulation/dynanav/office.yaml`）是保留/归档数据，不能当作生效配置。 |

## 读取顺序

1. **启动器读配置**：`start.py` → `launch/main.py` 调 `common.config.read()` 合并 [launch/navigation.json](launch/navigation.json) 里的六个文件（control、planning、localization_mapping、simulation、common、launch），`path()` 解析资产路径；`launch/prepare.py` 再按同一棵树生成 world/外参/SLAM 覆盖/SCAN 参数/路线。
2. **prepare 的 pin**：`prepare.py` 把若干键**强制写成固定值**（`daily_navigation`（profile 与 `cascade` 两处）、`experiment=navigation`、`r85_auto_reference_contract.reference_file="global_reference.json"`、`engineering_recording.raw_source_capture=false`、`engineering_recording.registered_cloud_archive="disabled"`）。与配置不一致时会向 stderr 报告旧值/新值/原因，不再静默覆盖。
3. **本次 profile 写入 run 目录**：`profiles` 写进 `runtime_context.json`（含 `profile`、`run_dir`；`experiment="navigation"`、`navigation_ground_truth_used=false` 等固定字段）与 `navigation_profile.json`。
4. **运行期消费者分两类，不能一概说"configs 不再生效"**：
   - 读 **runtime_context** 的进程：`navigation/ros_node.py`、`robot/unitree/bridge.py`、`navigation/mission/mission_runtime.py`、`localization_mapping/localization_runtime.py`、`localization_mapping/launch/components.py`、`perception/sensors/sensor_gate.py`。这些模块的业务 profile 来自本次 run 的 context；这不表示全部配置读取都被冻结，例如 localization launch 仍通过 `path("ros_prefix")` 重读资产路径配置。
   - 仍**直接重读 configs** 的运行期消费者：`robot/unitree/policy/student.py` 的 `read()["student_runtime"]`；`launch/building.py` 与 `navigation/robogauge/benchmark.py` 的 profile/环境构造；`common/runtime_environment.py` 及部分 planning/simulation/localization launch 组件通过 `path()` 重新读取路径配置。
   - 另有 `planning/scan_epoch_supervisor.py` 直接读 run 里的 `navigation_profile.json`。

因此"改了配置却没变化"的常见原因依次是：该键被 prepare pin、被 `launch/main.py` 的环境构造覆盖、只有读 runtime_context 的进程不读它、或该键根本没有读取点。

## 目录

| 路径 | 消费者 | 说明 |
| --- | --- | --- |
| `launch/navigation.json`、`launch/settings.json` | `common.config.read`、`launch/main.py` | 模块清单与 `features` 开关；`features` 里 45 个键中只有部分在 `src/` 有读取点，探针类键会被启动器强制 `"0"` |
| `control/settings.json` | `navigation`/`control` 经启动器与 profile | cascade 速度界、停车等参数 |
| `planning/settings.json`、`planning/static_support_limits.json` | `planning` | SCAN/PCT 与静态支撑 |
| `localization_mapping/settings.json`、`fastlivo.yaml`、`camera.yaml` | 定位前端/后端 | base 参数；运行时覆盖由 prepare 生成 |
| `simulation/settings.json`、`simulation/dynanav_*.json` | `simulation` | 世界/物理/传感器与 DynaNav 碰撞场景 |
| `common/settings.json` | `common` | 运行目录、通信、遥测 |
| `scenarios/` | `launch/prepare.py` | 路线与地形数据；`terrain_targets/` 当前无代码读取（见该目录 README） |
| `robogauge/` | `robot/unitree/robogauge`、`navigation/robogauge` | 策略/benchmark 契约；`contract.json` 是归档副本，权威常量在 `contract.py`；`routes/`、`world.sdf.in`、`deployment_profile.json` 由 `simulation/gazebo/robogauge/world.py` 读取 |
| `visualization/` | 人工/脚本 | RViz 配置等可视化资源；`.rviz` 无 Python 读取者（人工 `rviz2 -d`） |
| `control/relative_pid.yaml` | 当前无 Python 读取点 | 独立 `relative-slam-navigation` 入口的保留配置，未接线 |
| `navigation/relative_shuttle.yaml` | 当前无 Python 读取点 | 同上，保留未接线 |
| `robot/unitree_usd_manifest.json` | 当前无 Python 读取点 | 资产清单预留，启动检查不读它 |
| `runtime_assets_manifest.json` | 当前无 Python 读取点 | 同上，归档数据 |
| `simulation/dynanav/office.yaml` | 当前无 Python 读取点 | DynaNav 场景/资产历史记录；实际路径硬编码在 `simulation/dynanav/experiment.py`（见该目录 README） |

## 模块内文件耦合

- 依赖方向：配置之间**没有 include/继承**；耦合是“文件 → 读取点”的单向关系。唯一的多文件组合是 `launch/navigation.json` 的 `modules` 列表（固定六个模块，按序合并；重复顶层键直接报错），以及 `prepare.py` 从 base 派生 run 内配置。
- 与代码的耦合是**字符串键契约**：`settings.json` 的键名、`features` 的 `R\d+_*` 开关、`runtime_paths` 的四个路径键、`fastlivo.yaml`/`camera.yaml` 的 ROS 参数名，都没有 schema 校验（`read()` 只查 JSON 类型/重复键/路径越界）。改键名/类型不会在这里报错，只会让读取方走默认值或失败。
- 隐式契约（已核实的典型）：
  - `launch/navigation.json` 的六个模块必须是 JSON object 且顶层键不重复；entry 自身其它顶层键最后覆盖（会静默赢过模块值）。
  - `prepare.py` 的 pin 覆盖 profile；`runtime_context.write()` 固定写 `experiment="navigation"`、`navigation_ground_truth_used=false`。
  - `configs/robogauge/world.sdf.in` 的 `${ASSET_ROOT}`/`${ROBOGAUGE_ACTUATOR}` 占位符由 `simulation/gazebo/robogauge/world.py` 替换；`contract.json` 与 `contract.py` 需人工同步。

| 文件/目录 | 行数/规模 | 读取点 | 判断 |
| --- | ---: | --- | --- |
| `launch/navigation.json` + `launch/settings.json` | 10 + 70 | `common.config.read` + 启动器 | 清楚：入口清单短 |
| `control/settings.json` | 567 | 启动器/profile | 偏重：单文件参数多，靠本 README 说明 |
| `simulation/settings.json` | 3294 | 启动器/profile | **偏重**：全仓最大配置，人工审阅成本高 |
| `localization_mapping/*` | 225 | 前端/后端 + prepare | 清楚：base 与生成覆盖分离 |
| `planning/*` | 76 | planning | 清楚 |
| `common/settings.json` | 39 | common/启动器 | 清楚 |
| `robogauge/*` | 14 文件 / 22,816 行 | `world.py` 等 | 可接受：大文件是 SDF 模板与路线数据 |
| 6 个无读取点文件 | 8–232 | 无 | 可疑（孤儿配置）：易被误当生效值 |

结论：配置层**没有循环依赖**，但“文件多、读取点散、无 schema”使隐式契约偏多；这是配置系统的固有成本，不是结构错误。最大的可维护性风险是 `simulation/settings.json`（3294 行）与 6 个无读取点文件。

## 可读性评估

- `README.md`（原 39 行）：**可接受偏高**。读取顺序、pin、运行期消费者分类写得很具体（这是全仓最有价值的配置说明之一），但没有目录树/规模信息，也没有把“无读取点”集中到一处；本轮已补。
- `JSON/YAML` 配置文件：**可接受**。JSON 不能写注释，键名基本自解释（`duration_s`、`runtime_paths`、`features`、`control_clock_contract`）；问题是数值/开关的**含义**只能靠本 README 和源码，`features` 45 个键是否被读取、哪些被强制 `"0"` 需要 grep 才能确认。
- `simulation/settings.json`：**需改进（规模）**。3294 行全仓最大配置，无 schema、无分区注释；读者很难判断某段是否还在用。
- `fastlivo.yaml`/`camera.yaml`：**可接受**。128/17 行，ROS 参数结构清晰；`camera.yaml` 只有 17 行，容易核对。
- 没有发现虚构键或过期读取点：本轮抽查了 `office.yaml`、`relative_pid.yaml`、`relative_shuttle.yaml`、`unitree_usd_manifest.json`、`runtime_assets_manifest.json`、`lower12.json` 的 grep 结果，均无 Python 读取点。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 6 个无读取点文件加 `_archived_comment` 或统一移到 `configs/archive/`，并在本 README 用一张表集中列出 | 无读取点文件与生效配置同目录同格式，最容易被误改/误读 | 低：移动需同步文档链接；若无读取点则无代码影响 | `grep -rn "<file>" src scripts tests` 仍为空；`check_readmes` 链接通过 |
| P2 | 把 `features` 45 个键做成“键 → 是否有读取点/是否被启动器强制”的生成表（从源码 grep 生成） | `features` 是“配置写 1 也可能无效”的重灾区 | 无（只读生成文档；键多易过期，需标注生成日期） | 抽查 5 个键的 grep 结果与表一致 |
| P2 | 给 `simulation/settings.json` 分节（或用多个子文件 + `modules` 顺序合并） | 3294 行单文件难以审阅；`read()` 已支持多模块合并，拆分成本低于想象 | 中：拆分改变 `navigation.json` 的 modules 列表与重复键语义，需回归 | `common.config.read()` 单测 + `start.py --check` + 对比合并后 dict 完全相等 |
| P3 | 把“读取顺序”第 4 条的消费者清单与源码引用行号做成可再生成脚本 | 消费者会随重构变化，手工清单易过期 | 无 | 脚本输出与本 README 清单 diff 为空 |

没有 P0/P1：配置层是数据不是逻辑，最坏后果是参数不生效/误读；已有 pin 报告、`--check` 与单测保护，不需要为“整洁”重构配置文件。

## 修改约定

- 改数值前先确认读取点与所属时间/控制层；不同层的上限、时钟、阈值不为了"看起来一致"而统一。
- 不要手改 `runs/` 或 `experiments/` 里的生成参数当配置；那是一次运行的证据。
- 删除历史字段/兼容结构前，先确认没有读取点和文档引用；`configs/` 中仍有仅供归档、无 Python 读取点的数据。

## 边界

- 本目录是 base 配置，**不是**本次运行的有效值；`runs/<id>/` 与 DynaNav `experiments/<name>/` 里的生成文件才是。
- `robogauge/contract.json` 是归档副本，权威常量在 `robot/unitree/robogauge/contract.py`；两者需人工同步，当前无自动校验。
- 本 README 的读取点清单是 2026-11-09 的 grep 结果，结构重构后可能漂移；改动前以源码为准。
