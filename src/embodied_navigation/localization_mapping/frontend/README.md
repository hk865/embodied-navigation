# localization_mapping/frontend

本目录归置 FAST-LIVO2 的 ROS 接入资料；它本身不是可独立运行的算法包，真正的 FAST-LIVO2 实现在 `third_party/fast_livo2`。目录里只有几个 `__init__.py` 占位文件和一个上游文件副本子目录，导入它没有副作用。

## 边界

- 不做前端算法：没有 IMU 预积分、滤波、视觉更新、建图或点云注册代码。
- 不是 ROS 包：本目录（含子目录）没有 `package.xml`、`CMakeLists.txt`（2026-11-09 实测 `find` 为空），不会被 `colcon build` 识别，也不能 `ros2 launch`。
- 不被生产代码 import：`grep -rn "localization_mapping.frontend" src scripts tests` 只命中本目录自身，没有任何模块把它当作库使用。

## 目录结构

```text
src/embodied_navigation/localization_mapping/frontend/
├── fast_livo2_ros/     # 上游 ROS 包的接入/包装副本（launch、scripts），见 fast_livo2_ros/README.md
├── README.md           # 本文件
└── __init__.py         # 空占位（0 行，空文件），仅使目录成为 Python 包
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `__init__.py` | 空文件（0 行），无 docstring、无 import。存在的唯一理由是让 `embodied_navigation.localization_mapping.frontend` 可被 import 路径解析；没有业务代码。 |
| 代码（副本） | `fast_livo2_ros/` | 上游 `third_party/fast_livo2/fast_livo2_ros` 的**部分文件副本**：只有 `launch/mapping_aviz.launch.py` 与 `scripts/decompress_images.py` 两个参考实现 + 各自 `__init__.py`。不是在本仓库重新编译的包。 |
| 日志/输出 | 无 | 本目录不写文件、不 print、不订阅/发布话题；副本脚本即使运行，日志也由 ROS 侧输出，与本目录无关。 |
| 配置 | 无配置读取点 | 不读 `configs/`；副本 launch 里的 `avia.yaml`/`camera_pinhole.yaml` 是**上游包的路径**，本仓库没有这些文件（见 [fast_livo2_ros/README.md](fast_livo2_ros/README.md)）。 |

## 模块内文件耦合

依赖方向：无。`__init__.py` 不 import 任何东西，`fast_livo2_ros/` 的副本文件也不 import 本仓库任何模块（副本 launch 只 import `launch`/`os`/`shutil`，副本脚本只 import rclpy、cv_bridge 等 ROS 依赖）。**无环、无共享状态、无隐式契约**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 0 | 无 | 无（无 import 方） | 清楚：纯占位 |
| `fast_livo2_ros/__init__.py` | 0 | 无 | 无 | 同上 |
| `fast_livo2_ros/launch/__init__.py` | 0 | 无 | 无 | 同上 |
| `fast_livo2_ros/scripts/__init__.py` | 0 | 无 | 无 | 同上 |
| `fast_livo2_ros/launch/mapping_aviz.launch.py` | 126 | ROS `launch`、上游包 share 资源 | 无（只作参考） | 清楚：副本 |
| `fast_livo2_ros/scripts/decompress_images.py` | 40 | rclpy/cv_bridge/OpenCV | 无（只作参考） | 清楚：副本 |

四类问题：**都没有**。这里没有实现代码可耦合；真正要防的是"副本被误当成算法本体"，那属于文档问题（已在本 README 与子目录 README 写明）。

结论：耦合**无**，谈不上合适与否——本目录是资料区，不参与运行。真正的前端耦合在上一层 [../odom_adapter.py](../odom_adapter.py) 与 [../launch/components.py](../launch/components.py)。

## 可读性评估

- **良好**：`__init__.py`（0 行（空文件）占位，无内容可误读）。
- **可接受**：副本文件是上游风格——`scripts/decompress_images.py` 40 行、3 个函数、函数 docstring 0.0，但逻辑是直线：订阅 `/camera/image_color/compressed` → `cv_bridge` 解码 → 按原 header 发布；`launch/mapping_aviz.launch.py` 的 `generate_launch_description` 无 docstring，模块头有 6 行说明（这 6 行是本仓库相对上游唯一的内容差异）。
- 本目录的问题是**文档问题**：副本与上游的关系（逐字节/加注释）、副本路径不属于本仓库安装树，这两点只能靠 README 说明，代码本身没有可改进的结构。
- 无 C++/生成物，AST 指标不适用；行数均为 `wc -l` 实测。

## 重构建议

本轮只写建议、不改代码。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 4 个 `__init__.py` 里各加一行注释说明"这里是副本资料区，不要从这里 import 算法" | 现在全空，读者第一眼会以为这是可用包 | 无（纯注释） | 人工复核 |
| P2 | 若上游副本不再需要，整体删除 `frontend/` 并在根 README 记录"仅保留 third_party 链接" | 副本零代码依赖、零测试；每多一份副本就多一处与上游漂移的风险 | 低：先确认没有文档/脚本引用这些路径 | `grep -rn "fast_livo2_ros/launch\|decompress_images" src scripts tests docs`，确认只剩文档引用 |
| P3 | 在 [fast_livo2_ros/README.md](fast_livo2_ros/README.md) 记录副本同步日期与上游 commit（或标注"未记录"） | 现在只能靠 diff 判断是否与上游一致 | 无 | 人工复核 |

没有 P0/P1：本目录不是运行代码，不存在正确性/维护成本问题；唯一的真实风险是"副本被误认为算法本体"，已用文档覆盖。

## 本目录与生产链路的关系

- 生产链路里的前端适配节点在上一层：[../odom_adapter.py](../odom_adapter.py) 负责把 FAST-LIVO2 的 IMU 状态换算成 body/lidar 里程计，由 [../launch/components.py](../launch/components.py) 启动；SLAM 可执行文件 `fastlivo_mapping` 从 `install/ros` 加载。
- 与上游的关系见 [third_party/fast_livo2/fast_livo2_ros/README.md](../../../../third_party/fast_livo2/fast_livo2_ros/README.md)；本目录只保留 `launch/` 与 `scripts/` 两份参考文件，`src/main.cpp`、`config/`、`rviz/` 等没有复制进来。
- 保留副本的目的只是让接手者在源码树里看到接入点对应的上游文件，不必跳到 `third_party/` 才能对照。

## 测试与限制

本目录无专用测试（`tests/` 中没有任何文件引用 `localization_mapping.frontend`）。不要把这里的 `__init__.py` 或 launch 副本当成算法本体，也不要把副本路径当作 ROS 包安装路径。本次仅静态核对了副本与上游的 diff，未运行任何副本文本；副本是否随上游更新属**未核实**项。
