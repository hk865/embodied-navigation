# localization_mapping/frontend/fast_livo2_ros

本目录是 `third_party/fast_livo2` 中真实 FAST-LIVO2 ROS 包的接入/包装副本，只保留 `launch/` 与 `scripts/` 两个参考文件，不是算法本体，也不是本工程编译出的 ROS 包。

## 边界

- 不是算法实现：没有 `src/main.cpp`、没有 `config/`、没有 `rviz/`、没有 `package.xml`/`CMakeLists.txt`（上游有，本副本没有复制）。算法本体见 [third_party/fast_livo2/fast_livo2_ros/README.md](../../../../../third_party/fast_livo2/fast_livo2_ros/README.md)。
- 不是本仓库的 SLAM 启动方式：本工程导航不通过这两个文件启动 SLAM，真正的启动点在 [../../launch/components.py](../../launch/components.py)（`slam` 角色）。
- 不被任何代码 import：`grep -rn "fast_livo2_ros" src scripts tests --include=*.py` 只命中本目录路径，没有 import 方。

## 目录结构

```text
src/embodied_navigation/localization_mapping/frontend/fast_livo2_ros/
├── launch/                 # 上游示例 launch 副本（bag/AViz 回放），见 launch/README.md
├── scripts/                # 上游示例脚本副本（图像解压），见 scripts/README.md
├── README.md               # 本文件
└── __init__.py             # 空占位（0 行，空文件）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `__init__.py` | 空文件（0 行），无 docstring/import；只为让包路径可解析。 |
| 代码（副本） | `launch/mapping_aviz.launch.py`（126 行） | `generate_launch_description()` 启动参数黑板的 parameter_blackboard、`image_transport republish`、`fastlivo_mapping`，并按 `use_rviz` 可选起 RViz2。与上游**不是**逐字节相同：本副本多了 6 行模块 docstring（见下）。 |
| 代码（副本） | `scripts/decompress_images.py`（40 行） | rclpy 节点 `DecompressNode`：订阅 `/camera/image_color/compressed`，`cv_bridge` 解成 `bgr8`，按原 header 时间戳发布 `/camera/image_color`；与本上游同名文件**逐字节相同**（`diff -u` 无输出，2026-11-09 实测）。 |
| 日志/输出 | 无 | 副本自身不写文件；运行时日志是 ROS 节点日志，不属于本目录。 |
| 配置 | 上游路径，本仓库不存在 | 副本 launch 从**上游包 share 目录**取 `config/avia.yaml`、`config/camera_pinhole.yaml` 和 `rviz/` 配置；本副本没有这些文件，本工程实际运行参数来自运行目录的 `navigation_fastlivo.yaml` + `navigation_camera.yaml`。 |

## 模块内文件耦合

依赖方向：无。两个副本文件互不 import，也不 import 本仓库模块（launch 副本只 import `launch`/`os`/`shutil`），唯一的共享物是上游包资源路径，而那份资源在本仓库并不存在。**无环、无隐式契约**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `launch/mapping_aviz.launch.py` | 126 | 上游包 share（`avia.yaml`/`camera_pinhole.yaml`/`rviz`）、ROS `launch` | 无（只作参考） | 清楚：副本，且缺资源不能直接跑 |
| `scripts/decompress_images.py` | 40 | rclpy、cv_bridge、OpenCV、`sensor_msgs` | 无（只作参考） | 清楚：副本 |
| `__init__.py` | 0 | 无 | 无 | 清楚：占位 |
| `README.md` | 本文件 | — | — | — |

四类问题：**都没有**（无实现代码可耦合）。真正要防的是"把副本当可构建包/算法本体"，属文档问题，已在本文与两个子 README 写明。

结论：耦合**无**；本目录与生产链路的唯一关系是"提供上游接口的阅读入口"，运行期零参与。

## 可读性评估

- **可接受**：`scripts/decompress_images.py` 40 行、函数 docstring 0.0，但逻辑单一直线（订阅压缩图 → 解码 → 原 stamp 发布），变量名 `self.bridge`/`self.publisher` 自解释；`launch/mapping_aviz.launch.py` 的 `generate_launch_description` 无 docstring，126 行里约 17 行注释，主体是命令拼装，可读性取决于对 ROS launch 的熟悉度。
- **文档层面较好**：本仓库给 launch 副本补的 6 行 docstring（"Loads the LiDAR (avia.yaml)…RViz2 only when use_rviz is true"）正是上游缺的说明，这也是本副本与上游唯一的差异。
- 无 C++/生成物；两个文件都不在 AST 指标表里做"docstring 覆盖率"对比意义有限，行数为 `wc -l` 实测。

## 重构建议

本轮只写建议、不改代码。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `scripts/__init__.py`、`launch/__init__.py` 各加一行注释："副本，勿作为入口导入" | 现在两个 `__init__.py` 全空，容易让人以为可 `from …fast_livo2_ros.launch import …` | 无（纯注释） | 人工复核 |
| P2 | 若保留副本，加一份 `SYNC.md` 或 README 表格记录两文件的同步来源 commit 与日期 | 现在只能靠 `diff -u` 现场判断是否漂移 | 无 | 人工复核 |
| P3 | 删除副本（与 frontend/README 的同类建议二选一） | 两个文件都不参与运行，维护成本纯支出 | 低：先确认无脚本引用 | `grep -rn "mapping_aviz\|decompress_images" src scripts tests docs` |

没有 P0/P1：本目录不参与运行链路，无可破坏的正确性面。

## 与上游和本工程的关系

- 逐字节结论只适用于 `scripts/decompress_images.py`；launch 文件是加了 6 行注释的副本，其余上游文件（`src/main.cpp`、`config/`、`rviz/`）没有复制进来。
- 本工程的接入点在 [../../launch/components.py](../../launch/components.py) 与 [../../odom_adapter.py](../../odom_adapter.py)：前者启动 `fastlivo_mapping`，后者把 `/aft_mapped_to_init` 适配成 `/demo/slam/{body,lidar}_odom`。可执行文件路径由 `configs/launch/settings.json` 的 `ros_prefix`（`install/ros`）拼成 `install/ros/lib/fast_livo2_ros/fastlivo_mapping`。
- 本检出 `install/ros` **不存在**（`install/` 下只有 `backend/lib/liblivo_graph.so`，2026-11-09 实测），所以 `fastlivo_mapping` 未构建，相关 launch 组件只能静态阅读。

## 测试与限制

本目录无专用测试。图像解压脚本保留 `main()` 入口，示例 launch 保留 `generate_launch_description()`；它们需要已安装的 ROS 依赖与上游包资源，不是本工程导航入口。不要把这里的副本当作可构建包或算法实现。本次仅静态核对 diff，未运行任何副本节点，也未验证副本与上游其它文件的一致性（未核实）。
