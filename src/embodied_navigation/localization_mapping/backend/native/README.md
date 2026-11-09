# localization_mapping/backend/native

本目录是仓库自带的一个 C++ SE(3) 稀疏图求解器，供后台 Python 通过 C ABI 调用。它很小（单 `.cpp` + 一个 CMake）、无 ROS、无 vendored 第三方代码，Eigen3 之外不引入新依赖。

## 边界

- 只做"有界 pose graph 求解"：输入位姿/边/σ/loop 标记，输出优化后位姿与报告；**不含** ROS、前端状态、点云、文件 I/O、线程或所有权管理。
- 不做配准/回环判定：是否调用、用哪些边、结果是否采纳都由 Python 侧（[../history.py](../history.py)、[../worker.py](../worker.py)）决定；求解失败不阻断历史收集。

## 目录结构

```text
src/embodied_navigation/localization_mapping/backend/native/
├── CMakeLists.txt      # Eigen3 + C++17，构建共享库 livo_graph 并 install 到 lib/（8 行）
├── README.md           # 本文件
└── graph_solver.cpp    # 导出 extern "C" livo_optimize_graph 的高斯-牛顿求解器（90 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `graph_solver.cpp` | 唯一实现：`skew`/`exponential`/`logarithm` 三个 SE(3) 工具（匿名 namespace），`residual` 计算 `Log(Z^-1·A^-1·B)` 并按 σ 归一化，`costs` 同时给 raw 与 robust 代价（loop 边超过 2σ 用 pseudo-Huber `4n-4`）。`livo_optimize_graph` 是唯一 `extern "C"` 符号。 |
| 代码 | `CMakeLists.txt` | `add_library(livo_graph SHARED graph_solver.cpp)`、`find_package(Eigen3 REQUIRED)`、`-O2 -Wall -Wextra`、`install(... LIBRARY DESTINATION lib)`。产物即 `install/backend/lib/liblivo_graph.so`。 |
| 日志/输出 | 无 | 不写文件、不 print、不写 stdout；诊断只通过返回值与 `report[5]` 数组回传（`[initial_raw_cost, final_raw_cost, iterations, wall_ms, converged]`）。 |
| 配置 | 无运行时配置 | 边界是**硬编码**在 `livo_optimize_graph` 入参校验里的：`2≤count≤2000`、`1≤edge_count≤6000`、`1≤window≤80`、`1≤iterations≤8`，越界直接返回 `-1`。Python 侧 `native_graph.optimize_graph` 传 `iterations=min(8, …)` 与之对齐。 |
| 错误码 | `livo_optimize_graph` 返回值 | `0`=接受并回写位姿，`1`=未改善（位姿不改），`-1`=入参越界/非有限，`-2`=分解失败/非有限步长，`-3`=异常捕获。Python 侧只把 `0` 且"有限且 final≤initial"当成采纳。 |

## 模块内文件耦合

依赖方向：`CMakeLists.txt → graph_solver.cpp`（构建关系）；C++ 侧与 Python 侧只通过 `extern "C" livo_optimize_graph` 一个符号耦合。**无环、无共享头文件、除 Eigen3 外无第三方依赖**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `graph_solver.cpp` | 90 | Eigen3 | [../native_graph.py](../native_graph.py)（`ctypes` 加载 `.so`） | 清楚：单文件单符号，边界在函数入口一处校验 |
| `CMakeLists.txt` | 8 | Eigen3 | `docker/build_owned_native.py`（`backend` 构建模块） | 清楚：无自定义变量、install 目录无歧义 |
| `README.md` | 本文件 | — | — | — |

四类问题：**都没有**。没有上帝文件（90 行）、没有重复实现、没有跨层直达（C ABI 就是显式契约）；唯一隐式契约是 `report` 的 5 个槽位与返回码语义，它们只在 `graph_solver.cpp` 注释和 [../native_graph.py](../native_graph.py) 的 docstring 各写一次，属于"两处需同步"的轻微成本。

结论：耦合**清楚且合适**，是本模块边界最干净的一处。

## 可读性评估

- **可接受**：`graph_solver.cpp` 90 行，关键段落都有注释（`hat operator`、`SE(3) exponential`、`robust pseudo-Huber`、`bounded work`、`backtracking line search`）；但风格是**压缩写法**（一行多语句、`Pose t=Pose::Identity();const Eigen::Vector3d w=…` 混排），函数内没有空行分区，阅读需要熟悉 Eigen 与 SE(3) 记号。`report` 槽位只有一行注释，返回码语义（`-1/-2/-3`）要从 3 处 `return` 反推。
- **良好**：`CMakeLists.txt` 8 行，无歧义。
- C++ 文件不在事实包的 AST 指标里，没有 "docstring 覆盖率" 可引；以上结论来自逐行阅读 90 行源码。这属于**文档问题**而非结构问题：单文件、单入口、单副作用（回写位姿数组），不存在可拆的耦合。

## 重构建议

本轮只写建议、不改代码。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `livo_optimize_graph` 上方补一张"返回码 → 语义"注释表，与 `report[5]` 槽位并列 | 现在错误码只能从 3 处 `return` 反推，Python 侧却按返回码分支 | 无（纯注释） | 人工复核；`test_backend_flow.py` 的图优化断言不变 |
| P2 | 把 `1e-6`（数值雅可比步长）、`1e-7`（先验）、`1e-9`/`1e-8`（收敛阈值）、`5`（line search 次数）提为具名常量 | 压缩写法里魔法数密集，改数值时无法 grep | 低：纯常量替换，行为等价 | 重建 `.so` 后跑 `test_backend_flow.py` 的 `test_verified_geometry_loop_optimizes_and_rebuilds_history`，比对 `report` |
| P3 | 若要把窗口扩到 >80 节点，先在 C++ 与 `native_graph.py` 同步改边界并记录到本 README | 边界现在是两处硬编码（C++ 校验 + Python 传参） | 中：改错会静默返回 `-1` | 单测覆盖"超界返回 -1" |

没有 P0/P1：90 行单文件、C ABI 契约明确、有真实回归用例间接覆盖（本机 `.so` 已构建）。

## 构建与调用

真实实现就在本目录，执行入口不在 C++：[../native_graph.py](../native_graph.py) 固定加载 `install/backend/lib/liblivo_graph.so`，由 [../history.py](../history.py) 的 `History.optimize` 间接调用；库缺失时返回 `native_graph_solver_unavailable`，不阻断历史收集。构建走 [docker/build_owned_native.py](../../../../../docker/build_owned_native.py) 的 `backend` 模块（第 13 行映射 `localization_mapping/backend/native → backend`），说明见 [docs/dependencies/BUILD.md](../../../../../docs/dependencies/BUILD.md)。

本检出 `install/backend/lib/liblivo_graph.so` 已存在（76,960 字节，2026-11-09 实测）。`install/` 中缺少该 `.so` 只表示本机未构建，源码完整；求解路径在图优化用例中被间接覆盖：[tests/regression/test_backend_flow.py](../../../../../tests/regression/test_backend_flow.py) 的 `test_verified_geometry_loop_optimizes_and_rebuilds_history` 需要该共享库存在。

## 测试

本目录没有独立单测，也没有注册 CTest。覆盖方式是间接的：上面的 `test_backend_flow.py` 用例通过 Python `History.optimize` 走到 `ctypes` 调用；`tests/native/` 只覆盖 DynaNav LIO 的 C++ 部分，不包含本求解器。未验证项：`-2/-3` 分支（分解失败/异常）没有专门用例，只能静态阅读。
