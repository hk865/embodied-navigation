# evaluation：离线指标与进程资源采样

本目录只做两件事：离线轨迹指标，以及独立进程的 CPU/RSS 采样。它不参与导航控制、不产生实时决策，导入本包也不会启动进程。

## 边界

- 指标只做离线计算；真值可以用于评测误差，但真值仍只属于评测/仿真安全边界，不得写成 SLAM/导航定位输入。
- `process_resources.py` 是独立 `python -m` 进程，导入模块本身不产生副作用。

## 目录结构

```text
src/embodied_navigation/evaluation/
├── README.md                 # 本文件
├── __init__.py               # 仅重导出 trajectory 的两个函数（1 行）
├── process_resources.py      # 独立 `python -m` 进程：每秒采样各角色 CPU/RSS（54 行）
└── trajectory.py             # 纯 NumPy 离线轨迹指标（37 行）
```

3 个 Python 文件、合计 92 行（2026-11-09 磁盘实测）。没有子目录，不读 `configs/`，不订阅 ROS 话题。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `trajectory.py` | 纯函数：`path_length_xy(positions)` 求 XY 路径长度，`position_error_xy(estimated, reference)` 返回 `rmse_m`/`max_m`/`final_m`。只取数组前两列，无 I/O、无 ROS。 |
| 代码 | `process_resources.py` | 进程入口：从 `$DEMO_RUN_DIR/processes.json` 读角色→PID，每秒解析 `/proc/<pid>/stat`，把 CPU 与 RSS 追加到 CSV。只有 `main()` 有副作用，`import` 该模块不建信号、不写文件（`tests/unit/test_building_entry_closure.py` 用 patch 验证）。 |
| 代码 | `__init__.py` | 兼容重导出。`from embodied_navigation.evaluation import path_length_xy` 可用，代价是导入本包即传递导入 NumPy。 |
| 日志/输出 | `$DEMO_RUN_DIR/process_resources.csv` | 本模块唯一写出的文件。列名即语义：`wall_monotonic_s`（单调墙钟秒）、`role`（进程角色，来自 `processes.json`）、`pid`、`cpu_total_s`（进程累计 CPU 秒，`utime+stime` 除以 `CLK_TCK`）、`cpu_cores_interval`（本采样区间内的平均核数占用）、`rss_mib`（常驻内存，页数 × `PAGE_SIZE` / 1 MiB）。以 `"x"` 模式创建，同一 run 目录已有同名文件会直接失败：一次运行只能有一个采样器。 |
| 输入（非日志） | `$DEMO_RUN_DIR/processes.json` | 由 `launch/main.py`、`launch/building.py`、`navigation/robogauge/benchmark.py` 写出，格式 `[{role, pid}, …]`。本模块只读不写。 |
| 配置 | 无 | 本模块不读 `configs/` 下任何文件；行为只由环境变量 `DEMO_RUN_DIR`（无默认值，缺失即 `KeyError`）和函数入参决定。 |

`process_resources.csv` 当前在本仓库**没有代码消费者**（`grep -rn "process_resources.csv" src scripts tests` 只命中本 README、包索引的一句提及与生成处，无代码消费者），属人工/离线证据，不能当作自动验收结果。采样器目前只由 Building 入口 `launch/building.py` 以 `-m` 启动；主路径与 RoboGauge 只写 `processes.json`、不起采样进程。

## 模块内文件耦合

- 依赖方向：`__init__.py → trajectory.py`；`process_resources.py` 不 import 任何内部模块。**无环**。
- 反向依赖：`trajectory` 只被 `evaluation/__init__.py` 与 `tests/unit/test_metrics.py` 导入；`process_resources` 不被任何模块 import，只被 `launch/building.py` 以子进程 `-m` 启动（该启动方式被 `test_building_entry_closure.py` 用 AST 锁定）。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `trajectory.py` | 37 | 仅 NumPy | `__init__.py`、单测 | 清楚：纯函数，输入输出明确 |
| `process_resources.py` | 54 | 仅标准库 | 无 import 方；子进程入口 | 清楚但偏"自成一体"：与包内其它文件零耦合，换位置不影响别人 |
| `__init__.py` | 1 | `trajectory` | 无 | 清楚：唯一作用是可读的重导出 |

结论：耦合**清楚且合适**。两个实现文件互不依赖，各自单一职责，没有共享可变状态、没有隐式 dict 契约、没有跨层直达。唯一需要注意的耦合是 `__init__` 的重导出会把 NumPy 带进任何 `import embodied_navigation.evaluation` 的调用方——当前 NumPy 已是基础依赖，可接受；但如果以后往本目录放 PyTorch/GPU 这类重模块，不应再从 `__init__` 重导出。

## 可读性评估

- `trajectory.py`：**良好**。模块 docstring 与两个函数 docstring 完整（函数 docstring 覆盖 1.0），写明了形状要求 `(N, ≥2)`、只用 `0:2`、单位，以及少于 2 个样本时返回 `0.0` 的边界。
- `process_resources.py`：**可接受，偏低**。证据：2 个函数 docstring 覆盖 0.0；`main()` 单函数 44 行承担读输入、解析 `/proc`、写 CSV、信号处理四件事；`except (OSError, ValueError, IndexError): pass` 静默吞掉坏 PID/短读取，运行时看不出丢了多少样本；`/proc/<pid>/stat` 的下标 11/12/21 只有一行注释可依；CSV 表头字面量与 `writer.writerow([...])` 的位置约定靠顺序维持，两处需同步改；无类型注解。
- `__init__.py`：1 行重导出，无需评价。

没有发现命名含糊或多层嵌套的问题；可读性缺口集中在"进程入口的边界条件没有文档化"和"静默容错"两点，而不是结构混乱。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `main()` 与 `stop()` 补 docstring，写清输入来源、CSV 列语义、`"x"` 模式的一次性约束 | 这是本目录唯一有副作用的文件，却是注释最少的 | 无（纯文档） | 人工复核 diff；`test_building_entry_closure` 的"导入无副作用"断言不变 |
| P2 | 把 CSV 列名提为模块常量 `CSV_COLUMNS`，表头与数据行都引用它 | 现在两处顺序约定只能靠人眼同步 | 无（行为等价） | 用临时 run 目录跑 2 个 tick，逐列比对表头；无真实 run 时以单测断言 `CSV_COLUMNS` 与写出顺序一致 |
| P2 | 把静默 `pass` 改为计数（例如每轮统计 `skipped_samples`，或至少在退出时打印一行汇总） | 丢样本不可见会让资源结论偏乐观 | 低：只加诊断输出，不改采样逻辑 | 单测注入不存在的 PID，断言计数>0 且不抛 |
| P3 | 在 docstring 写明"首个采样点的 `cpu_cores_interval` 恒为 0.0，因为无前序基线" | 现状只能从 `previous` 分支反推 | 无 | 读代码复核 |
| P3 | 明确本目录定位：要么补一个最简聚合/消费脚本（读 CSV 出 CPU/RSS 摘要），要么在 README 声明"只产出原始 CSV、分析在外部" | 避免后来者以为有现成汇总 | 低；新增脚本需单测 | 若加脚本，配一个用合成 CSV 的单测 |

没有 P0/P1：本模块 92 行、无环、两个纯函数有单测覆盖，重构收益主要是"进程入口文档化"和"容错可观测"，不值得为结构而改结构。

## 测试与限制

[tests/unit/test_metrics.py](../../../tests/unit/test_metrics.py) 覆盖 `path_length_xy` 与 `position_error_xy` 的正常路径（已知路径长 7、同一序列误差 0）；[tests/unit/test_building_entry_closure.py](../../../tests/unit/test_building_entry_closure.py) 验证 `process_resources` 导入无副作用、且 Building 入口以 `-m embodied_navigation.evaluation.process_resources` 启动、模块文件存在。

本目录不是完整评测框架：没有路线成功判定、汇总报告或与 run 结果绑定的验收流程。资源采样只读 Linux `/proc`，其它平台不可用；它采样的是进程级 CPU/RSS，不是实时性/延迟分析。
