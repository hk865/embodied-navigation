# perception：传感数据适配

`perception` 把仿真/估计器给出的传感数据变成下游可直接用的几何：RGB-D 反投影到 Go2 IMU 系并保留原图像素对应，以及按源时间窗口估计位姿速度。它不做 SLAM、定位、规划、控制或模型推理。

盘点日期 2026-11-20（分支 `refactor/06-clean-base`，HEAD 5b880b2）。

## 模块定位与架构层级

- 架构层级：第 2 层「感知与通信」中的传感器适配子包（见仓库根 `AGENTS.md` 第 2 节）。
- 输入：`simulation/dynanav/capture/frames.py` 直接传入深度图与标定数组；`simulation/dynanav/native_waypoints.py` 与 `simulation/dynanav/pct_scan_worker.py` 传入已经校验过的源时间戳与位姿。
- 输出：`DepthCloudPacket`（IMU 系点云 + C 序展平的像素索引 + 深度有效掩码 + 自车掩码）；速度向量（`np.ndarray` 或 `None`）。
- 上游/下游：上游是 `simulation/dynanav/`；本包不 import 任何业务模块（只依赖 numpy），被 2 个 `src/` 文件与 3 个测试导入。
- 边界：不读 ROS/Isaac、不读配置、不写文件、不建线程；不做去畸变配对、自车过滤或健康门（在调用方与上游）；点云预算与自车盒是固定常量，不是配置项。

## 当前文件夹结构

```text
src/embodied_navigation/perception/
├── README.md          # 本文件
├── __init__.py        # 1 行：包 docstring
├── depth.py           # 115 行：RGB-D → Go2 IMU 系点云与像素对应
└── pose_velocity.py   # 26 行：源时间位姿窗速度
```

4 个跟踪文件（3 个 `.py` + 本 README），共 142 行。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| [README.md](README.md) | 本文件：模块定位、逐文件职责、测试与配置注意。 | — |
| [__init__.py](__init__.py) | 包 docstring（1 行），无重导出。 | 无 |
| [depth.py](depth.py) | 深度图按针孔模型反投影到 Go2 IMU 系，过滤量程与自车回波后按约 8000 点预算均匀抽稀；可选返回原图像素对应。 | `DepthCloudPacket`（frozen dataclass：`points`、`pixel_flat_indices`、`depth_keep_mask`、`self_mask`）、`depth_to_cloud`（0.05–20 m 有效窗、自车盒、`_POINT_BUDGET=8000`）、`depth_cloud_packet`（校验深度二维实数与 `sensor_yaw (3,3)`/`camera_mount (3,)`/`imu_mount (3,)`/`center_px (2,)`/`focal_px` 为正标量）；调用方 `simulation/dynanav/capture/frames.py` |
| [pose_velocity.py](pose_velocity.py) | 从调用方拥有的 `deque` 位姿历史按源时间算地图系速度；窗口不完整返回 `None`。 | `update_pose_velocity(history, stamp_s, position, window_s, *, distinct=True)`：保留窗口前一个端点、要求完整源时间跨度、重复样本不延长历史；调用方 `simulation/dynanav/native_waypoints.py`、`simulation/dynanav/pct_scan_worker.py` |

## 数据结构与不变量

- `DepthCloudPacket.pixel_flat_indices` 与 `points` 逐行对应，索引按 C 序展平的原始深度图；`depth_keep_mask` 与输入深度图同形状；`self_mask` 索引的是**有效深度点**（自车过滤与抽稀之前）。数组归调用方所有，本包不复制数据。
- 像素对应的顺序固定为「有效深度 → 自车过滤 → 同一 stride」，因此调用方可以用 `pixel_flat_indices` 直接索引原始 RGB 展平数组；任何一步顺序改变都会破坏这个契约。
- `update_pose_velocity` 只读调用方的 `deque`（元素为 `(stamp_s, position)`），速度是窗口两端点差商，窗口不完整返回 `None`；`distinct=False` 时不追加重复样本。

## 被调用点（`git grep` 实测，2026-11-20）

| 调用点 | 用法 |
| --- | --- |
| `simulation/dynanav/capture/frames.py` | `from embodied_navigation.perception.depth import depth_cloud_packet`；每帧 RGB-D 用 `depth_cloud_packet(zz, sensor_yaw, mount, imu_mount)` 生成点云与像素对应，自车盒注释与 `depth.py` 的过滤盒保持一致。 |
| `simulation/dynanav/native_waypoints.py` | `from embodied_navigation.perception.pose_velocity import update_pose_velocity`；用调用方维护的位姿历史算源时间速度。 |
| `simulation/dynanav/pct_scan_worker.py` | 同上；在 PCT/SCAN worker 里用速度历史把位置差换算成 map 系速度，写回 `state['body_pose']` 的 twist（原始 post-VIO 消息不改，它自带的零 twist 不是实测速度）。 |
| `tests/unit/` | `test_depth_packet.py`、`test_perception.py`、`test_pose_velocity_contract.py` 直接 import 本包的纯函数。 |

另有 `simulation/dynanav/experiment.py` 的源码快照清单把 `perception/depth.py` 与 `perception/pose_velocity.py` 记入 run 的 source snapshot（只哈希文件，不 import）。

## 验证与测试入口 / 配置注意

- 直接覆盖本目录的测试：[tests/unit/test_depth_packet.py](../../../tests/unit/test_depth_packet.py)（6 个用例：像素对应、标定形状校验）、[tests/unit/test_perception.py](../../../tests/unit/test_perception.py)（1 个用例：投影与自车过滤公式）、[tests/unit/test_pose_velocity_contract.py](../../../tests/unit/test_pose_velocity_contract.py)（5 个用例：多轴速度、完整窗口、重复样本、历史重置）。
- 全量 CPU 测试：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`。
- 配置：本目录**没有配置读取点**，也没有环境变量开关。行为由函数默认参数与常量决定：有效深度 0.05–20 m、自车盒 base 系 `|x|<0.5`、`|y|<0.3`、`-0.45<z<0.3`、默认 `focal_px = 512*24/42`、默认主点 `(256, 256)`、点预算 8000。
- 调用注意：`depth_cloud_packet` 的 `pixel_flat_indices` 按「有效深度 → 自车过滤 → 同一 stride」顺序索引原始深度图，调用方必须与 `depth_to_cloud` 使用同一套内外参，否则像素对应不成立；`update_pose_velocity` 不做新鲜度、frame/generation 变化或历史重置判断，这些由调用方负责。
