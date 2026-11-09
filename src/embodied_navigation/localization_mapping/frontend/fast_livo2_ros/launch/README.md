# localization_mapping/frontend/fast_livo2_ros/launch

本目录只保留上游 FAST-LIVO2 ROS 包的示例 launch 副本，不是本工程的启动入口。

## 边界

- 不是本仓库的 SLAM 启动点：真正的启动在 [localization_mapping/launch/slam.launch.py](../../../launch/slam.launch.py) 及其 `components.py`。
- 不能直接 `ros2 launch`：本仓库这一层没有对应的 `package.xml` 和 CMake 安装，`get_package_share_directory("fast_livo2_ros")` 在本工程找不到包。
- 不参与运行链路：没有任何 Python 模块 import 本目录。

## 目录结构

```text
src/embodied_navigation/localization_mapping/frontend/fast_livo2_ros/launch/
├── README.md              # 本文件
├── __init__.py            # 空占位（0 行，空文件）
└── mapping_aviz.launch.py # 上游 bag/AViz 演示 launch 的副本（126 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `mapping_aviz.launch.py` | `generate_launch_description()`（108 行）声明 `use_rviz`（默认 `False`）、`use_respawn`（默认 `True`）、`avia_params_file`、`camera_params_file` 四个 launch 参数；主体是 `Node(...)` 拼装 + 一段被注释掉的 `ros2 bag play` 示例。 |
| 代码 | `__init__.py` | 空文件（0 行），无内容。 |
| 日志/输出 | 无 | launch 副本自身不写文件；即使运行，日志也是 ROS 启动器/节点日志。 |
| 配置 | 上游包 share 路径（本仓库不存在） | 从 `get_package_share_directory("fast_livo2_ros")/config` 取 `avia.yaml`、`camera_pinhole.yaml`，从同包 `rviz/fast_livo2.rviz` 取 RViz 配置。本副本没有 `config/`、`rviz/`、`package.xml`，因此这些默认路径在本仓库必然失效。 |

## 模块内文件耦合

依赖方向：无。`__init__.py` 不 import 任何东西；`mapping_aviz.launch.py` 只 import `os`、ROS `launch` 模块与 `get_package_share_directory`，不 import 本仓库任何代码。**无环、无共享状态**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `mapping_aviz.launch.py` | 126 | ROS `launch`、上游包 share 资源 | 无 | 清楚：副本，且资源缺失 |
| `__init__.py` | 0 | 无 | 无 | 清楚：占位 |
| `README.md` | 本文件 | — | — | — |

四类问题：**都没有**。这是单文件副本，唯一隐式契约是"上游包名 `fast_livo2_ros` 与 share 目录布局"，而本仓库并不满足它——这一点已在本文写明。

结论：耦合**无**；本目录只提供"上游 launch 长什么样"的阅读入口。

## 可读性评估

- **可接受**：`mapping_aviz.launch.py` 126 行，`generate_launch_description()` 单函数 108 行、无函数 docstring，但模块头有 6 行说明（本副本相对上游的唯一差异），内部有 `# Find path`/`# Load parameters`/`# Param use_rviz`/`# play ros2 bag` 等分段注释，命令拼装是线性的。
- **良好**：`__init__.py` 0 行（空文件）占位，无可误读内容。
- 这是**文档问题**而非结构问题：副本不应被改结构（改了就不是副本），只需说清"它属于上游、本仓库不能跑"。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `__init__.py` 加一行"副本，勿作为入口"注释 | 空 `__init__.py` 容易让人尝试 `from …launch import mapping_aviz` | 无 | 人工复核 |
| P3 | 若要长期保留，补记录同步的上游 commit | 目前只能现场 diff | 无 | 人工复核 |

没有 P0/P1：不参与运行。

## 与上游和本工程的关系

- 上游同名文件见 [third_party/fast_livo2/fast_livo2_ros/launch/mapping_aviz.launch.py](../../../../../../third_party/fast_livo2/fast_livo2_ros/launch/mapping_aviz.launch.py)，**不是**逐字节一致：`diff -u` 实测本副本多 6 行模块 docstring（上游没有），其余内容相同；旧版本文写"内容与本副本一致"，按磁盘更正为"语义一致、多 6 行注释"。
- 本工程实际启动 SLAM 的位置是 [localization_mapping/launch/slam.launch.py](../../../launch/slam.launch.py) 及其 `components.py`，它们直接使用 `install/ros` 下的 `fastlivo_mapping`（`ros_prefix` 来自 `configs/launch/settings.json`）和运行目录内生成的 `navigation_fastlivo.yaml`/`navigation_camera.yaml`。

## 测试与限制

无专用测试；不要把示例 launch 当作导航链路已验证的启动方式。本检出 `install/ros` 不存在（`fastlivo_mapping` 未构建），因此本目录内容无法在本机执行验证（未核实运行行为）。
