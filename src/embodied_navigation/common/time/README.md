# common/time：时钟与执行器适配

本目录提供 rclpy 的 `/clock` 回调组、clock service 以及若干可选的 executor 调度适配，用于调整 ROS 时间与主循环行为，不产生时钟值也不改变控制计算。

## 边界

- 不产生时钟值、不做时间戳变换、不新增订阅/线程/定时器；`/clock` 的生产者仍是 rclpy 的 TimeSource。
- 只改分发时序与回调组归属，不改控制计算、callback 所有权或业务状态；所有实验都默认关闭。
- 这些选项没有实时上界：sleep 与实际调度延迟都不可承诺（见各文件 docstring）。
- 依赖特定 rclpy 内部实现与调用顺序；版本/实现不匹配时**显式抛错**，而不是静默退化。

## 目录结构

```text
src/embodied_navigation/common/time/
├── README.md  # 本文件
├── __init__.py  # 0 行（空文件），空包标记
├── clock_groups.py  # 49 行，R74 为 /clock 建独立 callback group
├── clock_service.py  # 240 行，R114 原始 /clock 上的非等待 service
├── cooperative_executor.py  # 159 行，R90 单 spinner 正 yield 实验
├── empty_retry_yield.py  # 182 行，R91 空轮询 yield（AST 复制方法）
└── executor_selection.py  # 27 行，R74 public spin_once 执行器选择
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | 6 个 `.py` 共 658 行（最大 `clock_service.py` 240 行） | 全部是活代码：`clock_groups`/`executor_selection`/`cooperative_executor`/`empty_retry_yield` 是默认关闭的调度适配，`clock_service` 是 R114 实验实现。没有兼容门面或生成物。 |
| 日志/输出 | 写 run 目录（仅在各自开关打开时） | `clock_service.finish(node)` 写 `r114_clock_service.json`（schema、`environment_value`、计数、列名、容量、tail、`sources` 中各 rclpy 文件 SHA256、`hard_wall_bound_claimed=False`）；`cooperative_executor.finish(node, executor)` 写 `r90_spinner_statistics.json`（选择回执 + spin/yield 统计）；`clock_groups` 只把 setup 回执放到 `node._r74_clock_setup_receipt`（不落盘）；`empty_retry_yield` 只返回 `snapshot()` dict，由调用方处理；`executor_selection` 无 I/O。诊断写失败不改变业务结果：两个 `finish` 都用 `except Exception` 兜住。 |
| 配置 | 环境变量 + `configs/control/settings.json` 的 `r74_*` 键 | 环境变量（进程级、启动前设置）：`R114_CLOCK_SERVICE`（`clock_service` 在**导入时**读取，非法值直接 `ValueError`）、`R90_SPINNER_POSITIVE_YIELD`（`cooperative_executor.selected()`）、`R91_EMPTY_RETRY_YIELD`（调用方传参给 `empty_retry_yield.install`）。profile 键：`r74_independent_ros_clock`（`clock_groups.enabled`）、`r74_cooperative_executor`（`executor_selection.enabled`）。`clock_service.install()` 还要求 `R99_CLOCK_FEEDBACK == "0"`、节点已有 `_r86_prepared_mailbox`、且存在唯一的原始 `/clock` 订阅。 |

这里的"默认"指未设置环境变量时的代码缺省值。当前普通导航的 [configs/launch/settings.json](../../../../configs/launch/settings.json) 将 `R90_SPINNER_POSITIVE_YIELD`、`R91_EMPTY_RETRY_YIELD`、`R114_CLOCK_SERVICE` 设为 `1`；[configs/control/settings.json](../../../../configs/control/settings.json) 也启用了相关 `r74_*` 选项。配置读取顺序与"哪一层覆盖哪一层"见 [configs/README.md](../../../../configs/README.md)。

## 模块内文件耦合

依赖方向单一、无环：`cooperative_executor.py → executor_selection.py`（R90 必须建立在 R74 执行器之上，否则 `ValueError`）、`clock_groups.py → common.input_freshness`（复用诊断开关判定）。其余 3 个文件互不 import。反向只有三个消费者：[navigation/pid_entrypoint.py](../../navigation/pid_entrypoint.py)（装 cooperative + empty_retry）、[navigation/pid_support.py](../../navigation/pid_support.py)（clock_service）、[navigation/base_node.py](../../navigation/base_node.py)（`subscription_options`）。本目录不 import 控制/导航层，无环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `clock_groups.py` | 49 | `common.input_freshness` | `navigation/base_node.py` | 清楚（只改回调组选项） |
| `clock_service.py` | 240 | 无（内省 rclpy） | `navigation/pid_support.py` | 偏重（86 行 `service()` + 59 行 `install()` 的严格前置校验） |
| `cooperative_executor.py` | 159 | `executor_selection` | `navigation/pid_entrypoint.py` | 可接受（96 行 `make_executor` 内含执行器子类） |
| `empty_retry_yield.py` | 182 | 无（AST 复制已安装 executor 源码） | `navigation/pid_entrypoint.py` | 可疑（依赖 rclpy 源码文本） |
| `executor_selection.py` | 27 | 无 | `cooperative_executor.py` | 清楚（开关 + 透传） |
| `__init__.py` | 0 | 无 | 无 | 清楚 |

四类问题核查：

- 上帝文件：没有（最大 240 行）。
- 重复实现：没有。三个 yield/executor 实验各自实现不同的调度干预，未复制同一算法。
- 跨层直达：没有。只 import `common.input_freshness`，其余依赖都是 rclpy 标准库。
- 隐式契约：有，但都是 **fail-closed** 的版本契约。`clock_service.install()` 逐项断言 `type(sub) is rclpy.subscription.Subscription`、`callback.__func__ is TimeSource.clock_callback`、`executor._take_subscription` 是原始实现等；`empty_retry_yield.install()` 用 `inspect.getfile` + SHA256 记录被复制的 executor 源码，并要求两个原始方法来自同一文件。rclpy 升级或换实现时会抛 `RuntimeError` 而不是悄悄改变调度语义——这是正确的防御，但也意味着**升级 rclpy 必须先复核这几个文件**。

结论：耦合清楚、方向单一，实验代码之间没有互相纠缠。主要成本不是耦合而是 rclpy 内部实现耦合，当前用显式断言把风险转成了启动失败。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `executor_selection.py` | 良好 | 27 行，2 函数（`enabled`/`make_executor`），docstring 0.0 但函数各 9/2 行、模块 docstring 说明"不是等价替换" |
| `clock_groups.py` | 可接受 | 2 函数，docstring 0.0，最长 25 行；回执 dict 的键名自解释 |
| `empty_retry_yield.py` | 可接受 | 6 函数，docstring 0.167，最长 54 行（`_compile`），0 个 >80 行函数；模块 docstring 很长且准确（写明"sleep 可更长、shutdown 可竞争、失败原样传播"） |
| `clock_service.py` | 需改进 | 6 函数，docstring 0.167，最长 86 行（`Service.service`），1 个 >80 行函数，5 注释行；`install()` 的断言块靠错误字符串自解释 |
| `cooperative_executor.py` | 需改进 | 8 函数，docstring 0.375，最长 96 行（`make_executor`，内含 `PositiveYieldExecutor` 子类），1 个 >80 行函数 |

结构问题：`cooperative_executor.py` 把执行器子类定义在 `make_executor()` 内部（96 行），与 `sensor_gate.py` 内嵌类同型；`clock_service.Service.service()` 86 行承担非等待分发 + 记录 + 计数。其余是文档问题：`enabled`/`selected` 等小函数缺 docstring，但模块级 docstring 已交代边界。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把 `cooperative_executor.py` 的 `PositiveYieldExecutor` 提到模块级 | 96 行 `make_executor` 里定义类，无法单独 import/测试 | 中（调度路径敏感） | 先加不 spin 的构造测试，再移动定义并保持 `receipt()` 字段不变 |
| P2 | 补 docstring：`clock_groups.py`(0.0)、`executor_selection.py`(0.0)、`clock_service.py`(0.167)、`empty_retry_yield.py`(0.167) | 开关语义（默认值/谁读/失败形态）目前只在代码里 | 低 | 纯文档 |
| P2 | 在本 README 记录当前验证过的 rclpy 版本/发行版（Jazzy） | `install()` 断言的是内部实现，升级需人工复核 | 低 | 与 `pyproject`/环境文档对齐 |
| P2 | 为本目录补纯函数单测：`enabled`/`selected` 的 0/1/非法值三态、`executor_selection.enabled` 缺省与类型错误 | 本目录当前 0 测试覆盖，这几个函数不需要 ROS | 低 | 新增 `tests/unit/test_common_time_options.py`，不实例化节点 |

本模块规模小、耦合清楚：**没有 P0/P1**，只有 P2（文档化 rclpy 版本 + 补 docstring + 补纯函数测试）。本轮只写建议、不改代码；任何调度相关改动必须先有对照运行，不能只看单测。

## 关键文件与接口（原"关键文件与接口"）

- `clock_groups.py`：`subscription_options()` 在 profile 打开 `r74_independent_ros_clock` 时，为 `/clock` 单独创建 `MutuallyExclusiveCallbackGroup`；其他 topic 与未选中的 `/clock` 原样返回，原 TimeSource 订阅 QoS 和时间戳不变。
- `clock_service.py`：由环境变量 `R114_CLOCK_SERVICE`（`0`/`1`，默认 `0`）控制。`install()` 校验原始 `/clock` 订阅、callback、callback group 与 executor take 均未被替换，否则抛错；`service()` 在原始 body 方法内做一次非等待分发，并在退出时写 `r114_clock_service.json` 诊断回执。
- `cooperative_executor.py`：`R90_SPINNER_POSITIVE_YIELD`（默认 `0`）在唯一 spinner 的 `spin_once()` 后 sleep 100 µs，记录慢 spin 区间；`executor_selection.py` 是 `r74_cooperative_executor` 的 profile 选择入口。
- `empty_retry_yield.py`：`R91_EMPTY_RETRY_YIELD`（默认关闭）用 AST 复制并安装两个 executor 方法，每 8 次完整空轮询 sleep 100 µs，不新增线程。

## 输入输出与调用

输入是 profile（[configs/control/settings.json](../../../../configs/control/settings.json) 中的 `r74_*` 键）或进程环境变量；输出是 callback group 选项、executor 包装或诊断 JSON。[navigation/pid_entrypoint.py](../../navigation/pid_entrypoint.py) 负责 install/收尾，[navigation/base_node.py](../../navigation/base_node.py) 使用 `subscription_options()`。

## 测试与限制

本目录无专用测试；相关行为只在 `navigation/pid_entrypoint.py` 的运行路径上被间接调用，CPU 单测未直接覆盖。这些选项只改变分发时序，sleep 与实际调度延迟都没有实时上界；`install()` 依赖特定原始 rclpy 实现与调用顺序，版本不匹配会显式失败。
