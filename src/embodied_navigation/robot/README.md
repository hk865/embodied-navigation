# robot：机器人契约与策略部署

`robot` 是机器人观测、关节契约与策略推理的顶层命名空间，当前只有 Unitree Go2 部署实现。本仓库没有训练流程：视觉策略训练在 RL_for_unitree，这里只保留导航部署所需的策略推理与机器人契约（见 [docs/architecture.md](../../../docs/architecture.md) 的「从入口阅读实际流程」第 6 条）。

## 边界

- 不训练、不采集数据、不导出模型；没有实机后端。本包自身不注册 ROS 节点，实际节点由子目录的 `bridge.py` 与各模块 `launch/components.py` 启动。
- 顶层 [sensors/](../sensors/README.md) 只是跨感知/定位/控制的数据契约，[perception/sensors/](../perception/sensors/README.md) 是感知侧 ROS 适配；二者都不是本目录的策略输入。
- 当前有两条部署路径：Student 使用 exact45 推理，`unitree/policy/native` 保留历史 Teacher 247D 观测用于诊断/适配；`unitree/robogauge` 使用独立的 45D 直接观测接口。两条路径的输入和 actuator 契约不能混用；普通导航不加载 Teacher actor。

## 目录结构

```text
src/embodied_navigation/robot/
├── README.md      # 本文件：顶层职责、边界与三套观测契约
├── __init__.py    # 1 行，包 docstring，声明"不含策略训练代码"
└── unitree/       # 子模块，见 unitree/README.md
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `__init__.py`（1 行） | 只有包 docstring “Robot observation and actuation contracts. No policy training code.”；无符号、无 import、无运行时代码 |
| 代码 | `unitree/` | 唯一实现子模块，见 [unitree/README.md](unitree/README.md) |
| 日志/输出 | 无 | 本包自身不写日志、不写文件；所有输出都在 `unitree/` 的进程内产生 |
| 配置 | 无配置读取点 | `robot/` 不读 `configs/`；`configs/robot/unitree_usd_manifest.json` 当前无 Python 读取点（资产清单预留），读取顺序见 [configs/README.md](../../../configs/README.md) |

证据：`__init__.py` 实测 1 行；事实包对该文件的 grep 统计（写文件 / getLogger / print / env / config）全部为 0。

## 模块内文件耦合

- 依赖方向：`robot/__init__.py` **不 import 任何内部模块，也不被任何模块 import**（事实包 import 图为空）。真正的依赖都在 `unitree/` 里：`unitree/` 单向依赖 `common/`、`control/`、`planning/`、`simulation/`，同时又被 `control/` 反向 import（见 [unitree/README.md](unitree/README.md)）。因此这一层本身无环。
- 本包不提供门面重导出：调用方一律写全路径，例如 `control/parking/causal_join.py` 直接 `from embodied_navigation.robot.unitree.bridge import validate_controller_source`。好处是没有“robot 包到底导出什么”的歧义，代价是调用点较长。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 1 | 无 | 无 | 清楚 |
| `unitree/`（子模块） | 1069 行（2 个 py） | 见下级 README | 见下级 README | 见下级评估 |

四类问题：没有上帝文件（本包无 >800 行文件）、没有重复实现、没有跨层直达（本文件不 import 任何东西）、没有隐式契约。结论：**耦合清楚**——这一层只是命名空间与边界声明，跨层耦合全部发生在 `unitree/` 内部，不在本级评价。

## 可读性评估

- `__init__.py`：**良好**。1 行且写明边界；没有函数可评指标。
- 包级文档：本文件原来把“两条部署路径”写在边界段里，容易被读成“两个子目录”；实际是 Student exact45、历史 Teacher 247D（仅诊断）、RoboGauge 45D 三件不同的契约，本文件已单列一节。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `robot/__init__.py` 的 docstring 里补一句 “unitree is the only implementation subpackage” | 现在只有契约声明，新读者不知道从哪个子包进入 | 无（只改字符串） | `PYTHONPATH=src python3 -c "import embodied_navigation.robot"` |

没有 P0/P1：本包只有 1 行运行时代码、无配置读取、无日志写入，不存在正确性或结构性风险。本轮只写建议、不改代码；任何建议都应先有测试再动。

## 三条策略观测契约不要混用

| 契约 | 维度 | 位置 | 当前角色 |
| --- | ---: | --- | --- |
| Student exact45 | 45 | [unitree/policy/student.py](unitree/policy/student.py) 的 `exact_student45()`，从 native64 状态算出 | 当前推理输入；实际加载 Student 导出 |
| 历史 Teacher native 中间观测 | 247 | [unitree/policy/native/observation.py](unitree/policy/native/observation.py) 的 `build_observation()` | 仅诊断/适配；普通导航不加载 Teacher actor（`teacher247_used_for_student_inference=False`） |
| RoboGauge 直接观测 | 45 | [unitree/robogauge/contract.py](unitree/robogauge/contract.py) 的 `observation()` | 另一条部署路径；独立 scale 与关节顺序，不是 exact45 的裁剪，也不含 Student 的 COM 速度 PID |

三者不是同一个策略输入或 actuator 契约；术语定义见 [docs/glossary.md](../../../docs/glossary.md)。

## 测试

本目录无专用测试；[unitree/README.md](unitree/README.md) 所述的策略与仿真安全门禁测试位于 [tests/README.md](../../../tests/README.md) 描述的 `tests/regression`。注意 `test_native_safety_integration.py` 含 AST 源码断言与假 socket，不是实机/物理行为测试。
