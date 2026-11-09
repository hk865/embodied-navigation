# localization_mapping/frontend/fast_livo2_ros/scripts

本目录存放上游 FAST-LIVO2 示例脚本的副本，用于理解 bag 回放时的图像解压；不是本工程运行链路的一部分。

## 边界

- 不是本工程的节点：启动配置没有调用它（[../../../launch/components.py](../../../launch/components.py) 的 8 个角色里没有它）。
- 不是必需实现：上游 launch 自己用 `image_transport republish` 完成同样的事。
- 不被任何 Python 模块 import：本目录只有一个可执行脚本副本与占位 `__init__.py`。

## 目录结构

```text
src/embodied_navigation/localization_mapping/frontend/fast_livo2_ros/scripts/
├── README.md            # 本文件
├── __init__.py          # 空占位（0 行，空文件）
└── decompress_images.py # 上游图像解压节点副本（40 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `decompress_images.py` | `DecompressNode`（rclpy `image_decompress`）订阅 `sensor_msgs/CompressedImage` `/camera/image_color/compressed`，`CvBridge.compressed_imgmsg_to_cv2(..., "bgr8")` 解码后用 `cv2_to_imgmsg` 发布到 `/camera/image_color`，并 `img_msg.header = msg.header` 保留原时间戳；解码异常时 `get_logger().error(...)`。 |
| 代码 | `__init__.py` | 空文件（0 行）。 |
| 日志/输出 | ROS 日志（仅运行时） | 节点启动 `info` 一条、解码失败 `error` 一条；不写文件。 |
| 配置 | 无配置读取点 | 不读 `configs/`、无参数声明；话题名硬编码在 `create_subscription`/`create_publisher` 里。 |

## 模块内文件耦合

依赖方向：无。`decompress_images.py` 只依赖 ROS 生态（`rclpy`、`sensor_msgs`、`cv_bridge`、`cv2`），不 import 本仓库任何模块；`__init__.py` 不 import 任何东西。**无环**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `decompress_images.py` | 40 | rclpy、sensor_msgs、cv_bridge、OpenCV | 无（只作参考） | 清楚：单节点单职责 |
| `__init__.py` | 0 | 无 | 无 | 清楚：占位 |
| `README.md` | 本文件 | — | — | — |

四类问题：**都没有**（40 行、3 个函数、无共享状态）。唯一隐式契约是话题名 `/camera/image_color/compressed` 与 `/camera/image_color`，它们与上游 launch 的 remap 配套，改动话题需两边同步。

结论：耦合**无**；本目录是上游接口的阅读参考。

## 可读性评估

- **可接受**：`decompress_images.py` 40 行、函数 docstring 0.0（模块 docstring 有 1 行说明用途），但 `DecompressNode`/`callback`/`main` 命名直白，`# preserve original timestamp` 注释点出了关键行为；`except Exception as e` 后记 `error` 日志而不是静默吞掉，比同目录其它文件的容错写法好。
- **良好**：`__init__.py` 无内容。
- 文档问题而非结构问题：唯一的阅读门槛是"它和上游 launch 的 republish 是替代关系"。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在 `__init__.py` 加一行"副本，勿作为入口"注释 | 空占位容易被误 import | 无 | 人工复核 |
| P3 | 若保留，补记录同步来源（上游 commit/日期） | 现在只能 `diff -u` 现场比对 | 无 | 人工复核 |

没有 P0/P1：本文件不参与本工程运行，且与上游逐字节相同，改它反而会破坏"副本"定位。

## 与上游和本工程的关系

- 上游原文件在 [third_party/fast_livo2/fast_livo2_ros/scripts/decompress_images.py](../../../../../../third_party/fast_livo2/fast_livo2_ros/scripts/decompress_images.py)，与本副本**逐字节一致**（`diff -u` 无输出，2026-11-09 实测）；本工程的启动配置没有调用它。
- 上游 launch 用 `image_transport republish` 完成同样的事，因此这个脚本是参考实现，不是必需节点。

## 测试与限制

无专用测试。本工程启动链不调用该脚本；脚本保留独立 `main()` 入口，需要 ROS 2 Python 环境、`sensor_msgs`、`cv_bridge` 与 OpenCV。本次仅静态核对，未运行节点。
