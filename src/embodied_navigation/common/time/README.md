# common/time：时钟与执行器适配

本目录提供 rclpy 的 `/clock` 回调组、clock service 以及若干可选的 executor 调度适配，用于调整 ROS 时间与主循环行为，不产生时钟值也不改变控制计算。

## 关键文件与接口

- `clock_groups.py`：`subscription_options()` 在 profile 打开 `r74_independent_ros_clock` 时，为 `/clock` 单独创建 `MutuallyExclusiveCallbackGroup`；其他 topic 与未选中的 `/clock` 原样返回，原 TimeSource 订阅 QoS 和时间戳不变。
- `clock_service.py`：由环境变量 `R114_CLOCK_SERVICE`（`0`/`1`，默认 `0`）控制。`install()` 校验原始 `/clock` 订阅、callback、callback group 与 executor take 均未被替换，否则抛错；`service()` 在原始 body 方法内做一次非等待分发，并在退出时写 `r114_clock_service.json` 诊断回执。
- `cooperative_executor.py`：`R90_SPINNER_POSITIVE_YIELD`（默认 `0`）在唯一 spinner 的 `spin_once()` 后 sleep 100 µs，记录慢 spin 区间；`executor_selection.py` 是 `r74_cooperative_executor` 的 profile 选择入口。
- `empty_retry_yield.py`：`R91_EMPTY_RETRY_YIELD`（默认关闭）用 AST 复制并安装两个 executor 方法，每 8 次完整空轮询 sleep 100 µs，不新增线程。

这里的“默认”指未设置环境变量时的代码缺省值。当前普通导航的 [configs/launch/settings.json](../../../../configs/launch/settings.json) 将 `R90_SPINNER_POSITIVE_YIELD`、`R91_EMPTY_RETRY_YIELD`、`R114_CLOCK_SERVICE` 设为 `1`；[configs/control/settings.json](../../../../configs/control/settings.json) 也启用了相关 `r74_*` 选项。

## 输入输出与调用

输入是 profile（[configs/control/settings.json](../../../../configs/control/settings.json) 中的 `r74_*` 键）或进程环境变量；输出是 callback group 选项、executor 包装或诊断 JSON。[navigation/pid_navigation.py](../../navigation/pid_navigation.py) 负责 install/收尾，[navigation/base_node.py](../../navigation/base_node.py) 使用 `subscription_options()`。

## 测试与限制

本目录无专用测试；相关行为只在 `navigation/pid_navigation.py` 的运行路径上被间接调用，CPU 单测未直接覆盖。这些选项只改变分发时序，sleep 与实际调度延迟都没有实时上界；`install()` 依赖特定原始 rclpy 实现与调用顺序，版本不匹配会显式失败。
