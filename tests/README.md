# 测试入口与边界

本仓库的测试全部是离线检查：不启动 Gazebo/Isaac，不做 GPU 推理，不连接机器人。仿真、训练和实机验收不在测试套件内。

## 目录结构

```text
tests/
├── native/          # C++/原生构建 fixture（dynanav_lio/），独立 CMake/CTest，不在 unittest 内
├── regression/      # 16 个 test_*.py、2520 行：定位/规划/停车/策略契约、CLI 与入口闭包、原生安全集成
├── unit/            # 45 个 test_*.py、8913 行：控制律、导航状态机、传感几何、来源/时钟、安全边界与纯逻辑
└── README.md        # 本文件：命令、环境/输出语义与边界
```

`tests/` 与各子目录**没有 `__init__.py`**；因此 `discover -s tests/unit` 可用，而 `discover -t .` 会报 `ImportError`。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（测试） | `tests/unit/test_*.py`（45 个）、`tests/regression/test_*.py`（16 个） | 纯 CPU、离线运行；没有生产代码、没有共享 helper 模块（`find -name '*.py' ! -name 'test_*.py'` 为空）。部分文件用 `tempfile`/`TemporaryDirectory`（32 个文件）写临时目录，不写仓库内文件。 |
| 代码（原生） | [`native/dynanav_lio/`](native/dynanav_lio/README.md) | 真实 C++ `LIVMapper`/`ImuProcess` fixture，由独立 CMake 构建、CTest 执行；需要本机 ROS Jazzy 与已编译 core，不在两条 unittest 命令内。 |
| 日志/输出 | unittest stdout/stderr | 唯一输出。成功形态 `Ran <N> tests ... OK (skipped=<K>)`；失败为 traceback + 非零退出。测试会在 stdout 打印预期内的 traceback（如 capture 编排的 sentinel 异常），**有 traceback 不等于失败**，以最终 `OK/FAILED` 与退出码为准。 |
| 日志/输出（skip 语义） | `skipped '<reason>'` | `skipped` 既不是通过也不是失败：表示本机缺少可选依赖/资产。`unit` 的 USD 用例（3 项，`LocalHumanUSDTests`）与 `regression` 的可选 CPU Torch 用例（5 项）按此处理；若 `rclpy` 不可导入，PID 兼容测试类会额外整类 skip。 |
| 日志/输出（原生） | 外部 `TEST_BUILD` 目录 | CTest 结果写在 `PROJECT_DATA_ROOT/.../build_cache/native-lio-check-<UTC>/`，不入 Git；仓库内无测试产物。 |
| 配置 | 环境变量 + 测试内 patch | 本目录不读 `configs/`。运行配置是 `PYTHONPATH=src`、`-B`（不写 `__pycache__`）与少量测试内 `unittest.mock.patch.dict(os.environ, …)`；回归测试的可选资产/权重通过环境或跳过处理，不在仓库内配置。 |
| 数据/fixture | 无独立 fixture 文件 | `find tests -name '*.json'` 为空；JSON/fixture 内容以内联字符串或临时文件构造，避免把外部数据卷变成测试依赖。 |

记录口径提醒：skip 数量取决于本机环境（USD 运行时、可选 CPU Torch、`rclpy` 是否可导入），不同机器上 `Ran`/`skipped` 数字可以不同；不要用“数字不一致”直接判定测试回归。

## 模块内文件耦合

- 依赖方向：`tests/unit`、`tests/regression`、`tests/native` 是三个**互不 import** 的发现根；测试文件之间不共享 helper（无 support 模块），只共享标准库/第三方库与被测包。
- 与生产代码的耦合有两类：
  1. 行为耦合：正常 `import` 被测模块并断言输入/输出。
  2. 结构与源码文本耦合：**24 个测试文件**使用 `ast.parse(...read_text())` 或直接读源码文件（如 [`test_dynanav_source_snapshot.py`](unit/test_dynanav_source_snapshot.py)、[`test_dynanav_odometry_source.py`](unit/test_dynanav_odometry_source.py)、[`test_production_slam_mode.py`](unit/test_production_slam_mode.py)、[`test_building_entry_closure.py`](unit/test_building_entry_closure.py)）。移动/拆分函数或改文件路径会先让这类测试失败，而不一定是行为变了。
- 隐式契约：测试通过 `subprocess`/`-m` 启动的生产入口模块名、`configs/` 中少量被读取的文件路径（如 `test_building_entry_closure.py` 断言 `configs/scenarios/building/{scan,student_profile}.json` 存在）、以及 skip 条件中的可选依赖名，都是字符串级约定；改入口/路径时要同步测试。

| 目录/文件 | 规模 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `unit/` | 45 文件 / 8913 行 | NumPy/SciPy/PyYAML、被测包、少量源码文本 | 无 | 清楚：独立发现根；少数文件偏大 |
| `regression/` | 16 文件 / 2520 行 | 同上 + 可选 CPU Torch/编译器/资产 | 无 | 清楚：契约与入口闭包为主 |
| `unit/test_dynanav_slam_route.py` | 903 | 被测包 + 内联 fixture | 无 | **偏重**：单文件最大，读改成本高 |
| `regression/test_dynanav_lio_launcher.py` | 545 | shell 启动器/外部 prefix 约定 | 无 | 可接受：命令组合多，需长文件覆盖 |
| `native/dynanav_lio/` | 4 文件 / 451 行（含本 README） | ROS Jazzy + 已编译 core | `ctest` | 清楚：见其 README |

结论：测试层**没有 import 环、没有共享可变状态**，耦合清楚；主要的结构风险是“源码文本/AST 断言”与“长测试文件”——它们让测试对重构敏感，但这是有意的设计（防止入口闭包被悄悄改掉），不是缺陷。

## 可读性评估

- 命名与组织：**良好**。文件统一 `test_*.py`，类名表达被测主题（如 `QueryCliTests`、`NativeSafetyLoopIntegration`）；用例名是完整句子，失败时无需读代码即可定位。测试不写 docstring 是常规约定，不按“文档覆盖率低”扣分。
- `unit/`：**可接受**。45 文件/8913 行，最长 [`test_dynanav_slam_route.py`](unit/test_dynanav_slam_route.py) 903 行、[`test_dynanav_capture_orchestration.py`](unit/test_dynanav_capture_orchestration.py) 542 行；编排类测试用 fake stepper/monkeypatch，读起来比断言密集的纯函数测试更重。unit 内 `@unittest.skipUnless` 1 处；全 `tests/` 共 5 处（unit 1、regression 4：`regression/test_navigation_safety.py` 2、`regression/test_robogauge_native_loop.py` 1、`regression/test_native_safety_integration.py` 1、`unit/test_dynanav_pedestrian.py` 1），条件都直接可读。
- `regression/`：**可接受**。16 文件/2520 行，入口闭包与 CLI 测试需要构造 `subprocess`/假 socket；`test_native_safety_integration.py` 用真实观测构造 + 假 peer，属于刻意的集成测试。
- `native/dynanav_lio/`：**可接受**，见其 README 的分级（C++ fixture 的断言组与 native 契约一一对应，但单 `main()` 承载多组断言）。
- 没有发现空断言或只测 mock 的“假测试”；测试普遍校验具体值/结构，而不是仅 `assertTrue(mock.called)`。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把 [`unit/test_dynanav_slam_route.py`](unit/test_dynanav_slam_route.py)（903 行）按主题拆成 2–3 个 `test_*.py` | 单文件接近千行，失败定位与 review 成本高 | 低：只移动用例，不改断言；注意模块级常量/helper 也要显式共享 | 拆分前后 `unittest discover` 的用例数与结果一致 |
| P2 | 给 24 个“读源码/AST”测试加统一注释头，写明“这是结构契约测试，重构需同步”，并在本文列出清单/模式 | 现在只能从代码看出，重构者容易误判为普通行为测试 | 无（纯注释/文档） | 抽查 5 个文件有注释；行为断言不变 |
| P2 | 在本文补“环境相关 skip 矩阵”：依赖（USD/Torch/rclpy）→ 影响文件/类 → 预期 skip 数 | skip 数随环境变化，容易被当成回归 | 无 | 与本机 `unittest -v` 的 `skipped` 行对照 |
| P3 | 若后续引入测试 helper，放到显式包（如 `tests/support/`）而不是各文件复制；当前复制量小，暂不必动 | 现无共享 helper，复制尚可接受 | 低；需改 import 路径 | `discover` 通过、无 `__init__` 冲突 |

没有 P0/P1：测试离线、无副作用、无共享状态；长文件与源码耦合测试是可维护性问题，不是正确性风险，不值得为“整齐”大改。

## 命令

从仓库根目录执行：

```sh
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit        # 纯 CPU，秒级
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression  # 纯 CPU，秒级
```

单个文件/用例：

```sh
PYTHONPATH=src:tests/unit python3 -B -m unittest test_layered_mapping -v
PYTHONPATH=src:tests/unit python3 -B -m unittest test_layered_mapping.QueryCliTests.test_invalid_filter_arguments_fail_without_traceback -v
```

**不要加 `-t .`**：`tests/` 与各子目录没有 `__init__.py`，`discover -t .` 会报 `ImportError: Start directory is not importable`。用上面的写法即可；`-B` 避免生成 `__pycache__`。

## 目录覆盖与依赖

| 目录 | 覆盖内容 | 依赖 |
| --- | --- | --- |
| `tests/unit/` | 控制律、导航状态机、传感几何、来源/时钟、安全边界、配置解析、离线查询与脚本纯逻辑 | NumPy、SciPy、PyYAML 与标准库；`LocalHumanUSDTests` 需要本机 USD 运行时/人形资产，缺失时 skip（3 项） |
| `tests/regression/` | 定位/规划/停车/策略契约、CLI 与入口闭包、原生安全集成（假 socket） | NumPy、SciPy、PyYAML 与标准库；部分用例另需可选 CPU Torch、C++ 编译器或私有测量资产，缺失时按用例 skip（本次 5 项均因 Torch 缺失） |
| `tests/native/` | C++/原生构建与 fixtures 测试（`dynanav_lio/` 含 README 与夹具） | 需要本机构建产物；不在上面两条命令内 |

## 编写与阅读注意

- 部分测试直接读取源码文本或 AST（例如入口闭包、原生安全接线），**改代码结构可能先影响这些测试，而不是行为**。移动函数时先看是否有按文件路径加载的测试。
- 需要私有测量图、外部数据卷或模型权重的用例会 skip，并在输出里标 `skipped`；skip 不等于通过，也不等于失败。
- 当前记录（2026-11-09，第四轮结构重构+独立复核修复后）：unit `Ran 323 tests ... OK (skipped=3)`（3 项为本机无 USD 运行时/人形资产；比第三轮 311 多 12 项：capture 普通 import 无副作用、capture 显式入口契约、DynaNav 源码快照覆盖拆分后的 capture 模块、PID facade 旧导入/直接脚本入口 2 项、capture 零/一步/首 tick 失败/清理二次失败/receipt 写入失败 orchestration 6 项（以上 11 项可逐项对应，另 1 项未单列，以实际 `Ran` 计数为准））；regression `Ran 86 tests ... OK (skipped=5)`（5 项为可选 CPU Torch 未安装）。
- 本轮文档任务复跑（2026-11-09）：unit `Ran 321 tests ... OK (skipped=4)`，差异是 `rclpy` 在本机不可导入，`PidNavigationCompatibilityTest` 整类 skip；regression `Ran 86 tests ... OK (skipped=5)` 与上述记录一致。数字差异来自环境，不是测试减少。
- 新增测试默认放 `tests/unit/`，且保持无 ROS、无网络、无 GPU；需要真实数据或仿真的检查写成显式脚本，并在文档中标注“未在测试套件内”。

## 限制

- 测试套件不覆盖 Gazebo/Isaac 仿真、GPU 推理、ROS 图和实机；`--check`/`--prepare-only` 成功也不等于系统可运行。
- unit/regression 是离线 CPU 测试，不代表实时性、资源占用或导航成功率；原生 CTest 通过也不是物理路线验收。
- 本目录的测试数量/skip 是环境相关快照；换机器（尤其有无 `rclpy`/USD/Torch）会变化。

## 2026-11-09 生命周期修复后的最终复跑

在 source ROS Jazzy 的宿主 Python 环境，最终 unit `Ran 351 tests ... OK (skipped=3)`，regression `Ran 86 tests ... OK (skipped=5)`。新增故障路径覆盖收据二次异常、就绪/回收、SDK信号吞异常及不返回的native关闭；3项unit跳过为USD/本地人形资产，5项regression跳过为可选CPU Torch。此前计数保留为历史快照，不与本次相加。真实Isaac行人仍有图像接收缺失，Gazebo未到达终态停车，见[本轮真实重验](../docs/review/runtime-repair-validation-20261109.md)。
