# common/communication：DDS 通信配置

本目录集中管理项目的 Fast-DDS 运行环境、传输 XML 与只读的 writer QoS 检查工具，不承载应用层消息逻辑。

## 边界

- 不创建 DataWriter/订阅者、不发布消息、不直接修改已有 writer 的 QoS；启动前可预载命名 publisher profile，并读取、校验实际 writer 配置。
- 不修改 RMW 版本或 Build-ID；只要求使用仓库自建的 DDS 库。
- 不写日志、不写 run 产物；本目录只提供环境字典变换、XML profile 与预编译 `.so`。

## 目录结构

```text
src/embodied_navigation/common/communication/
├── CMakeLists.txt  # 15 行，编译 writer_qos.so
├── README.md  # 本文件
├── __init__.py  # 空文件，包标记
├── body_async.xml  # 8 行，/demo/slam/body_odom 设为 ASYNCHRONOUS
├── dds_environment.py  # 59 行，选择项目 DDS 库并校验子进程环境
├── dds_transport.xml  # 31 行，本机 UDP + SHM 传输描述
├── metadata_async.xml  # 10 行，localization_bundle 设为 ASYNCHRONOUS
└── writer_qos.cpp  # 63 行，只读 writer QoS / profile 解析的 C++ 导出
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `dds_environment.py` 59 行（活代码）；`writer_qos.cpp` 63 行 + `CMakeLists.txt` 15 行（构建出 `writer_qos.so`）；`__init__.py` 空文件 | 无 Python 兼容门面。`configure()` 被启动器调用；`verify_environment()` 只被 `native_environment()` 调用，而后者在 `src/`、`scripts/`、`tests/` 里都没有调用点，属预留 API。`writer_qos.cpp` 导出 `r119_read_writer`（填 14 项 int64）与 `r119_parse_profile`（把 XML 载入 Fast DDS profile manager 并读 publish mode）。 |
| 日志/输出 | 无 | 本目录两个源文件都不写文件、不写日志：无 `print`、无 `getLogger`、无 `open(..., "w")`。`runs/dds_graph` 只是 `DOMAIN_QUERY_DIRECTORY` 常量（`dds_environment.py:8`），当前 grep 不到任何读取点，属预留未接线。 |
| 配置 | 无配置文件读取点；配置以环境字典与 XML profile 传递 | `dds_environment.py` 只从 `common.config` 取 `ROOT`（`LIBRARY = install/dds/lib/libfastrtps.so.2.14.6`、`DOMAIN_QUERY_DIRECTORY = runs/dds_graph`），不读 `configs/`。真正生效的键是环境变量：`LD_LIBRARY_PATH`（项目 DDS 目录置于首位）、`LD_PRELOAD`（必须为空，否则 `RuntimeError`）、`FASTRTPS_DEFAULT_PROFILES_FILE`（由 [launch/main.py](../../launch/main.py)、[launch/building.py](../../launch/building.py) 指向 `dds_transport.xml`；`body_async.xml`/`metadata_async.xml` 由 [perception/sensors/async_writers.py](../../perception/sensors/async_writers.py) 预载）。 |

环境与配置的整体优先级见 [configs/README.md](../../../../configs/README.md)；本目录只负责"DDS 库选择 + 传输 profile"这一段。

## 模块内文件耦合

Python 侧只有 `dds_environment.py` 一个文件，模块内没有 import 关系；文件之间靠**构建产物与 XML profile 名**耦合，而不是代码 import。依赖方向：`dds_environment.py → common.config`（仅 ROOT 常量），上层 `launch` 与 `navigation/robogauge` 消费它；XML 被启动器与 `async_writers` 读取；`.so` 被 `async_writers`/`sensor_relay` 通过 ctypes 使用，并被 `preflight` 当依赖文件检查。无环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `dds_environment.py` | 59 | `common.config`（ROOT） | `launch/main.py`、`launch/building.py`、`navigation/robogauge/benchmark.py` | 清楚（纯环境字典变换） |
| `dds_transport.xml` | 31 | 无 | `launch/main.py:43`、`launch/building.py:168`、`perception/sensors/sensor_relay.py`（复制到 run 内） | 清楚（通过 `FASTRTPS_DEFAULT_PROFILES_FILE`） |
| `body_async.xml` | 8 | profile 名须匹配 topic | `perception/sensors/async_writers.py` | 可疑（字符串 profile 名是隐式契约） |
| `metadata_async.xml` | 10 | 同上 | `perception/sensors/async_writers.py` | 可疑（同上） |
| `writer_qos.cpp` | 63 | Fast DDS / rmw_fastrtps 头文件 | 构建为 `writer_qos.so`，被 `async_writers`、`sensor_relay`、`preflight` 使用 | 偏重（C++ ABI + 14 列输出顺序约定） |
| `CMakeLists.txt` | 15 | ament/rcl/rmw_fastrtps/fastrtps | 构建系统 | 清楚 |
| `__init__.py` | 空 | 无 | 无 | 清楚 |

四类问题核查：

- 上帝文件：没有（最大 63 行）。
- 重复实现：没有。
- 跨层直达：没有。本目录是底层，只被上层 import。
- 隐式契约：有三处，且都是"字符串/二进制布局"级约定——① XML 的 `profile_name` 必须与 topic 完全一致（`/demo/slam/body_odom`、`/demo/navigation/localization_bundle`）；② `writer_qos.cpp` 的 14 项输出顺序必须与 `async_writers.FIELDS` 一一对应（该文件注释里明确写了这一点）；③ `libfastrtps.so.2.14.6` 的精确文件名与 ABI 由 `preflight` 和 `dds_environment` 共同假设。

结论：文件少、职责清楚，但**耦合方式是外部约定**（XML profile 名、`.so` 列顺序、库文件名），这些约定没有单一文档表，跨文件改名不会在编译期报错。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `dds_environment.py` | 良好 | 6 函数，docstring 3/6=0.5，最长 18 行（`configure`），0 个 >80 行函数；错误信息带具体路径 |
| `writer_qos.cpp` | 可接受 | 63 行，文件头 10 行注释列全 14 项语义与 5 个返回码，但函数体内无逐行注释 |
| `dds_transport.xml` | 良好 | 31 行，文件头注释说明"loopback-only + SHM、builtin transports disabled、每个 run 独立图" |
| `body_async.xml` / `metadata_async.xml` | 可接受 | 各 8/10 行；`metadata_async.xml` 有用途注释，`body_async.xml` 只有结构、无注释 |
| `CMakeLists.txt` | 良好 | 15 行，头 3 行注释说明"只读读取实际 QoS、不改 publish 调用" |

结构问题：无。文档问题：`body_async.xml` 缺一行用途注释；`dds_environment.py` 的 `configure/verify_environment` 已足够清楚，但 `native_environment` 只在 docstring 里说"已验证的环境片段"，未点明调用方只把它当 `DEMO_RUN_DIR` 覆写字典用。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在本 README 或 `writer_qos.cpp` 与 `async_writers.FIELDS` 之间建一张 14 项对照表，只留一处权威 | 列顺序是 C++/Python 双方约定，改一处不会编译失败 | 低 | 纯文档；按表逐项核对 `FIELDS` 元组 |
| P2 | 给 `body_async.xml` 加用途注释（"body-only 扩展；需 R119_METADATA_ASYNC 已开"） | 与 `metadata_async.xml` 文档不对称 | 低 | 纯文档 |
| P2 | 处理 `DOMAIN_QUERY_DIRECTORY`：删除或注明"预留、无读取点" | 定义但无消费者，容易被当成有效配置 | 低 | grep 调用点为空 |
| P2 | 给 `dds_environment.py` 补一个不依赖构建产物的单测：用假 env dict 验证 `reject_unknown_preloads` 与 `configure` 的 `LD_LIBRARY_PATH` 顺序 | 目前 0 测试覆盖，纯函数部分可以无 ROS/无构建测试 | 低 | `python3 -m unittest` 新增用例 |

本模块规模小、耦合清楚：**没有 P0/P1，只有 P2 文档与测试项**。本轮只写建议、不改代码；补测试时注意 `configure()` 会检查库文件是否存在，需要 mock `library_path()`。

## 关键文件与接口（原"关键文件与接口"）

- `dds_environment.py`：`configure(environment)` 选择 `install/dds/lib/libfastrtps.so.2.14.6`，把其目录放在 `LD_LIBRARY_PATH` 首位，并拒绝继承未知 `LD_PRELOAD`；`verify_environment()` 再校验；`native_environment(run, environment)` 校验传入环境后，仅返回含 `DEMO_RUN_DIR` 的覆写字典，不合并或修改传入环境。库文件缺失时抛 `FileNotFoundError`。
- `dds_transport.xml`：仅本机 UDP（127.0.0.1）+ SHM 的传输描述，供运行进程通过 `FASTRTPS_DEFAULT_PROFILES_FILE` 使用。
- `body_async.xml` / `metadata_async.xml`：分别把 `/demo/slam/body_odom`、`/demo/navigation/localization_bundle` 的 `publishMode` 设为 `ASYNCHRONOUS`，不改其他 QoS。
- `writer_qos.cpp` + `CMakeLists.txt`：编译 `writer_qos.so`，导出 `r119_read_writer`（只读获取 writer QoS 与 topic 一致性等 14 项值）与 `r119_parse_profile`（把 XML 加载到进程的 Fast DDS profile manager，再读取指定 profile 的 publish mode）；后者具有预载 profile 的进程内副作用。

## 输入输出与调用

输入是子进程环境字典；输出是修改后的环境字典或异常。[launch/main.py](../../launch/main.py)、[launch/building.py](../../launch/building.py)、[navigation/robogauge/benchmark.py](../../navigation/robogauge/benchmark.py) 在起进程前调用 `configure()`；[perception/sensors/async_writers.py](../../perception/sensors/async_writers.py) 通过 ctypes 使用 `writer_qos.so` 校验实际 writer。

## 测试与限制

本目录无专用测试；`dds_environment` 只经上述启动入口间接使用，`writer_qos.cpp` 需要本机构建产物才能验证。`install/` 中缺少 `libfastrtps.so.2.14.6` 或 `writer_qos.so` 属于本机未构建，不是源码缺失。传输 XML 与补丁库的兼容性只在既有构建环境核对，未在本会话重新构建或运行。
