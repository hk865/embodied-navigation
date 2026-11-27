# perception：传感数据适配

`perception` 把仿真/估计器给出的传感数据变成下游可直接用的几何（RGB-D 反投影到 Go2 IMU 系、源时间速度估计），并在保持**顶层 100% 纯 CPU** 的前提下，通过 [`vision_tools.py`](vision_tools.py) 对接独立进程的可选 GPU 开放词表分割与图像嵌入服务 [`vision_tool_server.py`](vision_tool_server.py)（SAM 3 / GroundingDINO / YOLOE + SigLIP）。

盘点日期 2026-11-26（分支 `feat/static-memory-streaming`，HEAD `4f81110`）。

## 模块定位与架构层级

- 架构层级：第 2 层「感知与通信」中的传感器适配与开放词表检测客户端子包（见仓库根 `AGENTS.md` 第 2 节）。
- 输入：`simulation/dynanav/capture/frames.py` 与规划侧传入的深度图/标定数组；以及 `vision_tools.VisionToolClient` 传入的 RGB 图像与提示词。
- 输出：`DepthCloudPacket`、速度向量、`GroundedDetection`（含 3D 局部坐标/包络、`range_source`、`candidate_scores` 多假设对比分布、`abstain` 低置信弃权与可选 SigLIP embedding）。
- 上游/下游：上游是 `simulation/dynanav/`、`agent/adapters/habitat_daemon.py` 与 `scripts/run_vision_tool_service.py`；`vision_tool_server.py` 将 PyTorch / FastAPI / SAM 3 / Transformers 隔离在函数内延迟导入，确保纯 CPU 单元测试不触发 GPU 依赖。

## 当前文件夹结构

```text
src/embodied_navigation/perception/
├── README.md              # 本文件
├── __init__.py            # 31 行：重导出 vision_tools 核心结构与几何/对比聚类接口
├── depth.py               # 115 行：RGB-D → Go2 IMU 系点云与像素对应
├── pose_velocity.py       # 26 行：源时间位姿窗速度
├── vision_tools.py        # 786 行：HTTP VisionToolClient、掩膜深度反投影、跨类 NMS、cluster_multilabel_detections
└── vision_tool_server.py  # 551 行：延迟导入 FastAPI + Torch 服务，托管 SAM 3 / GroundingDINO / YOLOE + SigLIP
```

6 个跟踪文件（5 个 `.py` + 本 README），共 1,509 行 Python 代码。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：模块定位、逐文件职责、测试与配置注意。 | — |
| [__init__.py](__init__.py) | 门面：导出 `VisionDetection`、`GroundedDetection`、`VisionToolClient`、`ground_detections_with_depth`、`cluster_multilabel_detections`、`suppress_overlapping_detections` 等纯 CPU 符号。 | — |
| [depth.py](depth.py) | 深度图按针孔模型反投影到 Go2 IMU 系，过滤量程与自车回波后按约 8000 点预算均匀抽稀；可选返回原图像素对应。 | `DepthCloudPacket`、`depth_to_cloud`、`depth_cloud_packet`；调用方 `simulation/dynanav/capture/frames.py` |
| [pose_velocity.py](pose_velocity.py) | 从调用方拥有的 `deque` 位姿历史按源时间算地图系速度；窗口不完整返回 `None`。 | `update_pose_velocity(...)`；调用方 `native_waypoints.py`、`pct_scan_worker.py` |
| [vision_tools.py](vision_tools.py) | **纯 CPU** 视觉工具客户端与几何后处理：通过 HTTP 对接本地检测/分割服务（请求异常返回空列表或 `None` 并将错因记入 `last_error`）；使用掩膜或缩小框按前景第 25 百分位深度取三维中位数与第 20 百分位水平距离（区分 `range_source`，有效像素不足则丢弃）；提供普通跨类 NMS（高重叠且分数 `< 0.85 * 强者` 抑制）与互斥的 `cluster_multilabel_detections`（按代表 IoU `>= 0.65` 贪心聚类每标签最高原始分，代表分 `< 0.25` 标 `abstain=True`，不因近似同分自动弃权）。 | `VisionToolClient`、`VisionDetection`、`GroundedDetection`、`ground_detections_with_depth`、`cluster_multilabel_detections`、`suppress_overlapping_detections`、`build_vision_prompts`、`render_visual_enhancement_overlay` |
| [vision_tool_server.py](vision_tool_server.py) | **延迟导入 GPU 依赖**的 FastAPI HTTP 服务（SAM 3 / GroundingDINO / YOLOE + SigLIP 图像与文本嵌入），通过 `Body(...)` + `model_rebuild()` 规避 `from __future__ import annotations` 校验冲突。 | `create_app(...)`、`run_vision_service(...)`；由 `scripts/run_vision_tool_service.py` 启动 |

## 数据结构与不变量

- `DepthCloudPacket.pixel_flat_indices` 与 `points` 逐行对应，索引按 C 序展平的原始深度图；`depth_keep_mask` 与输入深度图同形状；`self_mask` 索引的是**有效深度点**（自车过滤与抽稀之前）。数组归调用方所有，本包不复制数据。
- 像素对应的顺序固定为「有效深度 → 自车过滤 → 同一 stride」，因此调用方可以用 `pixel_flat_indices` 直接索引原始 RGB 展平数组；任何一步顺序改变都会破坏这个契约。
- `update_pose_velocity` 只读调用方的 `deque`（元素为 `(stamp_s, position)`），速度是窗口两端点差商，窗口不完整返回 `None`；`distinct=False` 时不追加重复样本。

## 被调用点（2026-11-26）

| 调用点 | 用法 |
| --- | --- |
| `simulation/dynanav/capture/frames.py` | `from embodied_navigation.perception.depth import depth_cloud_packet`；每帧 RGB-D 用 `depth_cloud_packet(zz, sensor_yaw, mount, imu_mount)` 生成点云与像素对应，自车盒注释与 `depth.py` 的过滤盒保持一致。 |
| `simulation/dynanav/native_waypoints.py` | `from embodied_navigation.perception.pose_velocity import update_pose_velocity`；用调用方维护的位姿历史算源时间速度。 |
| `simulation/dynanav/pct_scan_worker.py` | 同上；在 PCT/SCAN worker 里用速度历史把位置差换算成 map 系速度，写回 `state['body_pose']` 的 twist（原始 post-VIO 消息不改，它自带的零 twist 不是实测速度）。 |
| `agent/adapters/habitat_daemon.py` | 通过 `vision_tools.VisionToolClient`、`ground_detections_with_depth` 与 `cluster_multilabel_detections` 获取多假设 `candidate_scores` 与 3D 反投影坐标。 |
| `scripts/run_vision_tool_service.py` | 启动 `vision_tool_server.run_vision_service`，按 CLI `--backend` 与 `--embed-model` 延迟加载本地 GPU 模型。 |
| `tests/unit/` | `test_depth_packet.py`、`test_perception.py`、`test_pose_velocity_contract.py`、`test_vision_tools.py`、`test_agent_runtime.py` 直接 import 纯 CPU 函数。 |

另有 `simulation/dynanav/experiment.py` 的源码快照清单把 `perception/depth.py` 与 `perception/pose_velocity.py` 记入 run 的 source snapshot（只哈希文件，不 import）。

## 验证与测试入口 / 配置注意

- 直接覆盖本目录的测试：[tests/unit/test_depth_packet.py](../../../tests/unit/test_depth_packet.py)、[tests/unit/test_perception.py](../../../tests/unit/test_perception.py)、[tests/unit/test_pose_velocity_contract.py](../../../tests/unit/test_pose_velocity_contract.py)、[tests/unit/test_vision_tools.py](../../../tests/unit/test_vision_tools.py)。
- 全量 CPU 测试：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 本地视觉服务：需由 `scripts/run_vision_tool_service.py` 在外部 GPU Python 环境启动并经 `--vision-endpoint` 传入 URL；`vision_tools.py` 自身保持纯 CPU 无 Torch 依赖。
- 调用注意：`depth_cloud_packet` 的 `pixel_flat_indices` 按「有效深度 → 自车过滤 → 同一 stride」顺序索引原始深度图，调用方必须与 `depth_to_cloud` 使用同一套内外参，否则像素对应不成立；`update_pose_velocity` 不做新鲜度、frame/generation 变化或历史重置判断，这些由调用方负责。
