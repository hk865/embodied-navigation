# simulation/terrain

本目录提供受保护的 Actor 地形切换：只有实测 SLAM 到达指定 connector 时，才把 Actor 的高度射线从 lower12 切到 upper23；它不写导航状态或控制命令。

## 目录结构

```text
src/embodied_navigation/simulation/terrain/
├── README.md       # 本文件
├── __init__.py     # 1 行，仅包 docstring（privileged terrain providers for explicitly selected simulation profiles）
├── mission46.py    # 218 行，MissionTerrainProvider：在 provider 上加重排的三次切换顺序与 connector_mid 完成条件
└── provider.py     # 428 行，SwitchingTerrainProvider：冻结输入校验 + 187 射线一致性 + 切换/锁存
```

2 个实现文件、合计 646 行；无子目录。本模块只在 `robot/unitree/policy/worker.py` 的 profile 显式开启时构造。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `provider.py` | 活代码（默认关闭）。`SwitchingTerrainProvider.__init__` 校验：`<run>/world.sdf` 冻结、两份 manifest（`terrain_target_manifest.json`/`alternate_terrain_target_manifest.json`）与仓库源定义逐字节等价、manifest 的 include_models 恰为 `{floor_1,ramp_12,floor_2}` 与 `{floor_2,ramp_23,floor_3}`、`<run>/navigation_source_snapshots.json` 里唯一 `navigation/goal_regions.py` 归档的 sha256 与内容、以及该归档必须在 `<run>/sources` 内。`check_switch()` 再读 `navigation_status.json` 的 `region_arrivals`、`navigation_request.json`、goal 定义哈希，最后用 187 条 Actor 射线要求两套地形在共享 floor_2 上命中一致（≤1e-9 m）才切到 alternate。 |
| 代码 | `mission46.py` | 活代码（默认关闭）。`MissionTerrainProvider` 继承 provider，构造时把 `profile["terrain_layer_switch"]` 覆盖成 `tests/terrain_targets/{lower12,upper23}.json`（该目录当前不存在，见"限制"），`check_switch()` 增加 exploration/return/navigation 三次固定切换顺序与 `connector_mid` 完成条件。 |
| 代码 | `__init__.py` | 包标记，无重导出、无副作用。 |
| 日志/输出 | `<run>/terrain_provider_events.jsonl` | provider 的 JSONL 事件日志，以 `"x"` 模式打开（同目录重复运行会失败）：`initialized`（含 contract）、`switched_once`/`switch_refused_latched`（含 187 射线证据）、`closed`。 |
| 日志/输出 | `<run>/terrain_provider_result.json` | `close()` 时以 `"x"` 写出，`status` 取 `failed` / `switched` / `not_reached_unverified`，是本次 provider 的最终收据。 |
| 日志/输出 | `<run>/mission46_terrain_evidence_<n>.json`、`<run>/mission46_terrain_ack.json` | 仅 mission46：每次成功切换写一份证据（含 intent、navigation status、请求、native clock、187 条射线起止高度、两份 manifest sha256、`navigation_ground_truth_used=false`），再用 `.tmp` + `replace` 原子写 ack（含 5 项 checks 与 `source_evidence_sha256`）。请求输入是 `<run>/mission46_terrain_request.json`。 |
| 日志/输出 | `<run>/terrain_provider_initialization_failure.json` | 不是本目录写的，是 `robot/unitree/policy/worker.py` 在构造 provider 抛错时以 `"x"` 写出的失败收据（`status=failed`、`stage=initialization`）；记在这里是为了让读者知道初始化失败也留证据。 |
| 配置 | `runtime_context.json` 的 `profile.terrain_layer_switch` / `mission46_required` | 唯一配置读取点：`provider` 读 `profile["terrain_layer_switch"]`（要求 `completed_goal_id == "connector_mid"`、`alternate_include_models` 与 manifest 一致）；`worker.py:112` 用 `terrain_layer_switch is not None` 或 `mission46_required` 决定是否构造。`configs/control/settings.json` 只有 `mission46_required: false`，**没有 `terrain_layer_switch` 键**（`grep -rn terrain_layer_switch configs/` 无命中），所以 generic provider 只能由别的写 profile 的入口注入，mission46 则自己注入。 |

本模块不读 `configs/` 下任何文件，也没有环境变量读取点（事实包 grep：`config=1` 指 mission46 写死的 manifest 路径字面量，不是配置读取）。

## 模块内文件耦合

- 依赖方向：`mission46.py → provider.py`（继承，单向）；`provider.py → embodied_navigation.project`。两者都被 `robot/unitree/policy/worker.py` 导入。**无环**。
- 与上层是"鸭子类型 + monkeypatch"耦合：worker 用 `ActorTerrainFactory` 替换 `legacy.TerrainHeightMap`，只有在 include_models 与非默认 excluded_models 都匹配时才返回 provider，否则抛错（"no geometry fallback allowed"）；provider 需要 observation 模块对象传入以构造地图。这层契约是字符串/属性约定，不是接口类型。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `provider.py` | 428 | `project`、`<run>` 下的 world/manifest/snapshot/status/request 文件、`map_class`/`observation` 注入 | `mission46.py`、`robot/unitree/policy/worker.py` | 偏重：428 行、20 个函数里只有 1 个 docstring，`check_switch` 单函数 171 行 |
| `mission46.py` | 218 | `provider.py` | `robot/unitree/policy/worker.py`（`mission46_required` 时） | 偏重：4 个函数中 `check_switch` 159 行，但每个方法都有 docstring |
| `__init__.py` | 1 | 无 | 无 | 清楚 |

四类问题：**上帝文件无**（428 < 800，但已接近"单文件多职责"：冻结校验、goal-normalizer 动态加载、187 射线几何一致性、事件日志、结果收据都在一个类里）；重复实现无；**跨层直达有**：provider 直接用 `importlib` 从 run 目录执行归档的 `goal_regions.py`，绕过正常 import 边界（这是有意的"冻结归档"设计，但读者需要知道）；**隐式契约有**：`region_arrivals`/`navigation_request.json` 的字符串键、`terrain_layer_switch` 的四个键名、`"x"` 模式对 run 目录唯一性的要求，都靠约定维持。

## 可读性评估

- `provider.py`：**需改进**。证据：20 个函数只有 1 个有 docstring（`check_switch`，占 5%），`__init__` 66 行承担 8 类校验但无注释标题；`check_switch` 171 行、`_goal_module` 24 行把"读快照→找唯一归档→校验 sha→exec_module"压在无 docstring 的私有方法里；魔法值 187、1e-9、0.5 s dwell 有上下文但缺少单位说明；`_record`/`_read_json` 是清晰的私有助手。结构本身是"校验闸门"型代码，难度在于校验项多，不是算法复杂。
- `mission46.py`：**可接受，偏低**。优点：构造时用注释解释 `tests/terrain_targets/` 历史命名与两个开关的选择关系，`check_switch` 的 docstring 明确"返回 False 表示无事可做/已提交，True 表示必须保持运动（锁存或因果等待 ≤50 ms 的未来时间戳）"——这个反直觉返回值已有文档。不足：`check_switch` 159 行；`TRANSITIONS` 三个阶段名（`"navigation_f1_f3:7"` 等）是字符串契约，代码里没有生成处。
- `__init__.py`：1 行 docstring，与内容一致。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `provider.py` 的 `__init__`、`_goal_module`、`contract`、`observe_state`、`close` 补 docstring，至少写"校验哪几类输入、失败抛什么" | 5% 的 docstring 覆盖率让 428 行几乎只能靠读代码理解 | 无（文档） | 人工复核 |
| P2 | 把 `check_switch` 拆成 3 个私有步骤（读到达证据 / 校验请求与 goal 哈希 / 187 射线一致性），`mission46.check_switch` 复用前两步 | 171 行与 159 行的两个同名方法现在难以对齐比较 | 中：改行为边界的风险高，且该分支默认关闭、没有可运行样例 | 先造 fixtures（假 world/manifest/status）做纯函数测试；`mission46_required=false` 的默认路径回归必须无变化 |
| P2 | 在 README/注释写明 `terrain_layer_switch` 在 `configs/` 无定义点、只有 mission46 注入 | 否则读者会去改 `configs/control/settings.json` 并发现没有效果 | 无（文档） | 人工复核，`grep -rn terrain_layer_switch configs/` 应为空 |
| P2 | 记录 `"x"` 模式对 run 目录的一次性约束（events/result/evidence 都用 `open("x")`） | 重跑同一 run 目录会在 close 时抛错，容易被当成逻辑故障 | 无（文档） | 人工复核 |

没有 P0/P1：本模块默认关闭、不参与默认主路径，改动任何校验顺序都无法用现有测试或可运行样例验证，先把输入闭合（见"限制"）比拆函数更重要；因此这里的重构收益主要是文档与可测性，属 P2。本轮只写建议、不改代码。

## 限制

- 这是默认不启用的可选分支：`configs/control/settings.json` 中 `mission46_required` 为 false，普通默认启动不构造该 provider。
- 当前普通准备入口生成的 `runtime_context.json` 使用空 `references`，也没有准备 alternate manifest 与 `navigation_source_snapshots.json`。父类先校验 `world.sdf` 的冻结引用，再校验两套 source/run manifest 和 goal-normalizer snapshot；这些输入尚未由普通准备入口闭合。
- `mission46.py` 还写死引用 `tests/terrain_targets/{lower12,upper23}.json`，该目录不存在；当前 `configs/scenarios/terrain_targets/` 只有 `lower12.json`。只替换路径或打开开关不能恢复此分支，本次未验证可运行。
- 无专用 unittest；本次未启动仿真。本模块不把仿真位置重标为 SLAM 导航位姿；高度/射线用于现有仿真观测与校验，不是导航定位来源。
- 失败语义（已读码核实）：切换失败会锁存初始 provider，事件里记 `requested_velocity_after=[0.0,0.0,0.0]`，由上层按零速保护处理并继续策略反馈；`provider.failure` 一旦置位，`close()` 的结果状态是 `failed`。
