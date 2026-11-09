# common：跨模块公共基础

`common` 提供导航各模块共享的配置入口、状态来源与时间契约、后台 I/O、路径和进程关停语义，自身不实现业务状态机、控制律或 ROS 节点。

## 边界

- `observation.py` 是正常运行时默认关闭的诊断观测空端口：不创建线程、定时器或文件，历史 observer 不在 `src` 内；该空端口不记录诊断数据。
- ROS 相关导入都放在函数内，本目录可被纯 CPU 测试直接导入。
- 没有实机安全后端；`state_contract.py` 中 `REAL_SENSOR` 只是保留枚举。
- `common` 只被上层 import，不 import 任何上层模块（唯一依赖是同层的 `project` 与子包），因此不承担"跨层编排"。

## 目录结构

```text
src/embodied_navigation/common/
├── communication/  # 子模块，见 communication/README.md
├── time/  # 子模块，见 time/README.md
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），空包标记
├── config.py  # 90 行，配置合并、${PROJECT_ROOT} 展开与路径解析
├── input_freshness.py  # 79 行，正常源新鲜度直通适配器
├── integration_limits.py  # 14 行，冻结集成上限常量
├── observation.py  # 88 行，默认关闭的观测空端口
├── paths.py  # 16 行，工程内规范路径常量
├── runtime_context.py  # 37 行，run 配置读写
├── runtime_environment.py  # 52 行，source 公共 ROS 前缀
├── runtime_io.py  # 232 行，有界后台写盘原语
├── runtime_telemetry.py  # 100 行，默认关闭的诊断记录空实现
├── shutdown.py  # 39 行，信号驱动排空与关停异常
└── state_contract.py  # 133 行，typed source/time provenance 与物理安全状态
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | 12 个 `.py` 共 881 行（最大 `runtime_io.py` 232 行） | 全部是活代码，没有兼容门面/生成物/历史保留文件。无 `__main__` 入口：本目录只提供库，进程由各模块自己起。`__init__.py` 是空包标记（0 行，空文件）。 |
| 日志/输出 | 本模块不写日志，只提供写盘原语 | 12 个文件里 `getLogger` 与 `print` 命中均为 0，本目录不产生日志文件。实际写盘发生在被调用处：`runtime_io.EvidenceWriter.append/atomic`（单后台线程 + 4096 容量有界队列，溢出即报错）、`LatestCommandWriter`（一条在途 + 可替换 pending）、`runtime_telemetry.RuntimeTelemetry.atomic`（只写 live status，不写历史归档）、`runtime_context.write`（写本次 run 的 `runtime_context.json`，含 `run_dir`/`profile`/`actor_kind`/`controller_kind`/`navigation_ground_truth_used`/`references`/`experiment`）。 |
| 配置 | `config.py` 是唯一配置入口 | `read()` 读 [configs/launch/navigation.json](../../../configs/launch/navigation.json)，按 `modules` 顺序合并模块配置、拒绝跨模块重复顶层键与越界路径，再把入口文件其它顶层键最后覆盖；所有字符串展开 `${PROJECT_ROOT}`。`path(name)` 解析 `runtime_paths`，`actor_python='auto'` 时读环境变量 `NAVIGATION_ACTOR_PYTHON`（缺省 `sys.executable`）。`input_freshness.enabled(profile)` 只接受缺省或 `{'enabled': False}` 的 `r73_minimal_diagnostic_baseline`，恒返回 `False`。`runtime_context.load()` 读 run 的 `runtime_context.json` 并校验 `profile` 是对象、`run_dir` 与文件所在目录一致。配置读取顺序与 prepare pin 的权威说明见 [configs/README.md](../../../configs/README.md)。 |

## 模块内文件耦合

依赖方向单一且向下：模块内只有两条同层依赖——`runtime_environment.py → config.py`（要 `ROOT`/`path()`）、`time/clock_groups.py → input_freshness.py`（复用诊断开关判定）；没有环，也没有任何文件 import 上层。反向依赖规模很大：`observation.py` 被 23 个文件导入（既有模块顶层 import，也有大量函数内 import 的零成本空端口），`config.py` 被 18 个文件导入，`input_freshness.py` 13 个，`runtime_context.py` 与 `integration_limits.py` 各 7 个，`runtime_io.py` 5 个，`state_contract.py` 4 个，`runtime_environment.py` 3 个，`paths.py`、`shutdown.py` 各 2 个。

| 文件 | 行数 | 依赖谁 | 被谁依赖（文件数） | 判断 |
| --- | ---: | --- | --- | --- |
| `config.py` | 90 | `project` | 18 | 清楚（单一配置入口） |
| `input_freshness.py` | 79 | 无 | 13（含 `time/clock_groups.py`） | 清楚（纯直通，不解释参数） |
| `integration_limits.py` | 14 | 无 | 7 | 清楚（只读常量） |
| `observation.py` | 88 | 无 | 23 | 清楚但接口面大（空端口模拟历史 observer API） |
| `paths.py` | 16 | `project` | 2 | 清楚（常量表） |
| `runtime_context.py` | 37 | 无 | 7 | 清楚（读写一对函数） |
| `runtime_environment.py` | 52 | `config` | 3 | 可接受（启动器专用） |
| `runtime_io.py` | 232 | 无 | 5 | 可接受（后台 I/O 原语集中） |
| `runtime_telemetry.py` | 100 | 无 | 4 | 可接受（全部是空实现 + 一个 atomic） |
| `shutdown.py` | 39 | 无 | 2 | 清楚（信号 → 异常，不销毁 context） |
| `state_contract.py` | 133 | 无 | 4 | 清楚（产出方与门禁共用同一校验） |

四类问题核查：

- 上帝文件：没有（最大 232 行）。
- 重复实现：有，跨模块的原子 JSON 写至少 4 份——`runtime_io.py:51`（`EvidenceWriter.atomic`）、`runtime_telemetry.py:24`（`RuntimeTelemetry.atomic`）、`perception/sensors/sensor_gate.py:136`、`perception/person_worker.py:54`；临时文件后缀、`allow_nan`、`ensure_ascii` 略有不同。
- 跨层直达：没有。本目录只 import `project`；反向由上层 import 属于正常分层。
- 隐式契约：有三处。`config.path("actor_python")` 的 `'auto'` 哨兵依赖环境变量；`runtime_context` 要求 `run_dir` 与文件位置一致、`profile` 是对象；`observation.api(name)` 返回同一个 `_DISABLED` 单例，其方法名/属性名（`mark`、`span`、`consumer_call` 等 40+ 别名）是对历史 observer 接口的鸭子类型模拟，改名会让上层静默退化为 no-op。

结论：耦合清楚且方向正确（被依赖多但依赖少、无环），主要负担是**重复的写盘辅助函数**与 `observation.py` 的宽接口面，而不是文件之间互相纠缠。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `config.py` | 良好 | 4 函数，docstring 4/4=1.0，最长 32 行（`read`），含合并/越界/重复键的完整 docstring |
| `input_freshness.py` | 良好 | 6 函数，docstring 1.0，最长 18 行；每个适配器都写明"不解释参数、原样返回" |
| `integration_limits.py` | 良好 | 14 行常量表 + 模块 docstring，按量纲/时间基准命名（3 s 老化、300 ms 新鲜度、0.35 m p95、LiDAR 机体平移） |
| `paths.py` | 良好 | 16 行常量，模块 docstring 说明历史目录不进入运行时查找 |
| `runtime_context.py` | 良好 | 2 函数，docstring 1.0，最长 14 行；`load` 显式校验两个不变式 |
| `state_contract.py` | 良好 | 3 函数，docstring 0.333（类均有 docstring），最长 42 行；错误码是稳定字符串枚举 |
| `runtime_io.py` | 可接受 | 17 函数，docstring 0.235，最长 32 行，0 个 >80 行；后台线程/队列语义靠 4 行注释 |
| `runtime_environment.py` | 可接受 | 1 函数 45 行，docstring 完整；函数长是因为要 source 两个脚本再清理 7 个继承变量 |
| `observation.py` | 可接受（文档问题） | 11 函数，docstring 0.091，最长 3 行；结构是声明式别名表，不是复杂逻辑，但几乎无 docstring |
| `runtime_telemetry.py` | 可接受（文档问题） | 22 方法，docstring 0.0，最长 6 行；都是 no-op 或单行 atomic，缺"这是刻意空实现"的说明 |
| `shutdown.py` | 可接受（文档问题） | 4 方法，docstring 0.0，最长 9 行；类 docstring 说明了"不自动 shutdown context" |

区分两类问题：**结构问题**只有一处——写盘原语分散在 4 个模块，没有单一入口；其余都是**文档问题**（`observation.py`、`runtime_telemetry.py`、`shutdown.py` 的方法缺 docstring）。本目录没有静默吞错的写盘：`EvidenceWriter` 队列满会抛 `RuntimeError`，`runtime_telemetry.atomic` 的异常交给调用方。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 在 `runtime_io.py` 提供唯一的原子 JSON 写 helper，替换 `runtime_telemetry.py:24`、`perception/sensors/sensor_gate.py:136`、`perception/person_worker.py:54` 三处拷贝 | 同一模式 4 份实现，临时后缀/编码参数已不一致，改一处容易漏 | 低（都是"写 tmp 再 rename"） | 先列出 4 个调用点，逐处替换后跑 `tests/unit/test_ros_shutdown_lifecycle.py`、`tests/unit/test_person_worker.py` 与相关 launch 测试 |
| P2 | 给 `runtime_telemetry.py` 的 22 个方法补一行 docstring，明确"默认关闭、不是证据归档" | docstring 比 0.0，新人无法判断 no-op 是临时还是设计 | 低 | 纯文档 |
| P2 | 给 `observation.py` 加一段注释：`api(name)` 的方法名对应历史 observer 接口，改名即静默 no-op | 40+ 别名是对历史接口的隐式契约，是本目录最容易误改的地方 | 低 | 纯文档 + 抽查上层 `observation.api(...)` 调用点 |
| P2 | 在 `config.read()` 的 docstring 里点明"每次调用都重读文件、无缓存，热路径不要反复调用" | 目前只在代码行为里体现，调用方可能误以为有缓存 | 低 | 纯文档 |

本目录没有 P0/P1 的结构性风险；上面唯一的 P1 是跨模块的重复代码收敛。本轮只写建议、不改代码；替换写盘 helper 前应先确认各处异常语义（谁抛、谁记录）一致。

## 关键文件与接口（原"关键文件与接口"）

- `config.py`：`read()` 合并 [configs/launch/navigation.json](../../../configs/launch/navigation.json) 列出的模块配置并展开 `${PROJECT_ROOT}`；`path(name)` 解析 `runtime_paths`，`actor_python=auto` 时读 `NAVIGATION_ACTOR_PYTHON`。
- `state_contract.py`：`StateProvenance`、`StateOrigin`、`StatePurpose` 与 `validate_state_provenance()`。该契约的 `StatePurpose.NAVIGATION` 位姿用途只接受 `SLAM_ESTIMATE`；仿真真值可用于 `SIMULATION_SAFETY` 和 `EVALUATION`，不能重标为导航定位；`PhysicalSafetyState` 是物理安全事实，不是位姿提供者。
- `runtime_io.py`：`EvidenceWriter`、`LatestCommandWriter`（一条在途 + 可替换 pending，不排队）、`BackgroundCheck`、`bounded_shutdown_signals`；传感器回调不等待磁盘落盘。
- `integration_limits.py`：冻结常量，按量纲与时间基准命名（3 s 匹配/拼接老化、300 ms 命令与 bundle 新鲜度、0.35 m 射线 p95、LiDAR 机体平移等），只被消费，不是新配置项。
- `input_freshness.py`：正常源新鲜度适配器，`enabled(profile)` 拒绝把诊断放宽带进正常导航。
- `shutdown.py`：`ShutdownSignals` 先通过信号请求退出，让调用方在销毁 ROS context 前排空回调；`ShutdownRequested` 只由本进程信号触发。
- 其余：`runtime_context.py`（run 配置读写）、`runtime_environment.py`（source 公共 ROS 前缀）、`paths.py`（工程内规范路径）、`runtime_telemetry.py`（当前关闭历史归档，保留 live status 的同步原子文件写入）。
- 通信与时钟子目录见 [communication/README.md](communication/README.md) 与 [time/README.md](time/README.md)。

## 调用与依赖

被 `launch` 与多数模块导入；`runtime_io`、`integration_limits`、`input_freshness` 出现在热路径，`state_contract` 由 [simulation/safety_state.py](../simulation/safety_state.py) 等产出方与门禁共同消费。具体导入计数见上文"模块内文件耦合"表。

## 测试与限制

[tests/unit/test_simulation_safety.py](../../../tests/unit/test_simulation_safety.py) 覆盖 `state_contract`；[tests/unit/test_ros_shutdown_lifecycle.py](../../../tests/unit/test_ros_shutdown_lifecycle.py) 覆盖 `shutdown`；[tests/regression/test_navigation_safety.py](../../../tests/regression/test_navigation_safety.py) 使用 `config.read`；[tests/unit/test_launch_prepare_pins.py](../../../tests/unit/test_launch_prepare_pins.py) 覆盖 prepare 的 pin 报告路径（`_pin` 向 stderr 打印旧值/新值）。`runtime_io`、`runtime_telemetry`、`input_freshness`、`integration_limits`、`paths`、`runtime_context`、`runtime_environment`、`observation` 没有本目录专用测试，只经上层路径间接执行。本目录无独立功能测试文件；CPU 入口为 `PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v`。来源契约是程序内数据契约而非密码学认证；`install/` 缺失只代表本机未构建，不代表缺源码。
