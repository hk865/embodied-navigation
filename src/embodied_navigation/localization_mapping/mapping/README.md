# localization_mapping/mapping

本目录负责本次运行的地图归档：把 FAST-LIVO2 配准后的彩色点云累积成体素地图，并原子地发布给浏览器或离线消费方。它只消费话题、只写运行目录，不参与控制。

## 边界

- 不产生定位结果：只订阅已配准的彩色点云与若干新鲜度话题，不做配准/滤波/位姿估计。
- 不覆盖旧地图：运行目录已有 `map_metadata.json` 时 `MapArchive.__init__` 直接 `RuntimeError`；没有 `--run-dir` 且无 `DEMO_RUN_DIR` 时 `parser.error`（"never reuse an old map"）。
- 不做自动验收：`/demo/slam/save_map` 只有在健康门全过时才写 `colored_map.pcd`，拒绝时返回 `success=False` 并写原因；仓库内没有读取这些产物的代码（见"代码、日志与配置"）。

## 目录结构

```text
src/embodied_navigation/localization_mapping/mapping/
├── README.md        # 本文件
├── __init__.py      # 空占位（0 行，空文件）
└── map_archive.py   # rclpy 节点 MapArchive + VoxelArchive + PCD 写出（307 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `map_archive.py` | 三块：`VoxelArchive(voxel_size=0.08, max_points=8_000_000)` 做有界稀疏累积；`MapArchive(Node)` 订阅/健康门/快照/服务；`atomic_write` 与 `pcd_bytes` 两个纯函数。`BROWSER_DTYPE` 定义浏览器格式：`xyz <f4 (3)` + `rgb u1 (3)` + `pad u1` = 16 字节/点。 |
| 代码 | `__init__.py` | 空文件（0 行）。 |
| 日志/输出 | `<run>/colored_map.NNNNNN.bin` | 每次有更新（`dirty`）时 `revision+1` 写一份不可变修订（`atomic_write`：同目录 `.tmp` + `os.replace`），只保留最近 3 份（`glob` 后 `[:-3]` 删除），供 HTTP 读取方在原子替换期间继续读旧文件。 |
| 日志/输出 | `<run>/map_metadata.json` | 每 2 s（`create_timer(2.0, snapshot)`）与退出时原子重写：`run_id`、`frame_id`、`revision`、`binary_filename`、`point_count`、`rgb_points`、`bounds`、`voxel_size`、`format{point_step,xyz,rgb,padding_offset}`、`source_topic`、`color_source`、`geometry_source`、`ground_truth_used=false`、`reference_map_loaded=false`、`observed_rgb_samples`、`capacity_rejections`、`capacity_points`、`counts`、`ages`、`stamps`、`slam_healthy`、`camera_healthy`、`message_age`、`save`、`error`。 |
| 日志/输出 | `<run>/colored_map.pcd` | 保存服务的最终产物（`pcd_bytes`，header 说明 frame 为 `camera_init`、RGB 为本 run 观测）。服务可被多次调用、每次重写该文件；没有"只允许保存一次"的代码保护，是否重复保存由调用方负责。 |
| 日志/输出 | `/demo/slam/map_status`（话题） | 每次 `snapshot()` 把**同一份 metadata JSON** 发到该 `String` 话题；不是独立日志格式。 |
| 日志/输出 | 无 Python 日志 | `map_archive.py` 不 import `logging`、不 `print`；错误通过 metadata 的 `error` 字段、`counts` 计数与 `save` 服务响应表达。 |
| 配置 | `--voxel-size`（默认 0.08，限 [0.02, 0.5]） | 唯一命令行数值；不在 `configs/` 里。 |
| 配置 | `--run-dir` / 环境变量 `DEMO_RUN_DIR` | 运行目录来源；两者都缺则启动失败。本目录不读 `configs/` 任何文件。 |
| 输入（非产物） | `/cloud_registered` | 主输入（`PointCloud2`），frame 必须是 `camera_init`，否则计入 `rejected_frame`；时间戳必须严格递增，否则 `rejected_clock_reset`。 |
| 输入（新鲜度观察） | `/livox/lidar`、`/livox/imu`、`/camera/image_color`、`/demo/slam/body_odom`、`/cloud_registered_full` | 只更新 `received`/`stamps`/`counts`，点云内容不参与建图；用于 `ages` 与 `slam_healthy`/`camera_healthy`。 |

**消费方现状**：`map_metadata.json`、`colored_map.*.bin`、`colored_map.pcd` 在仓库内**没有代码读取方**（`grep -rn "map_metadata.json\|colored_map" src scripts tests` 只命中写入处与本文件）；消费者是仓库外的浏览器/HTTP 读取方或人工检查。因此它们是运行证据，不是自动验收输入。

## 模块内文件耦合

依赖方向：单文件实现，本目录内 `__init__.py` 不 import 任何东西；`map_archive.py` 只 import `../geometry.py` 的 `cloud_records`（点与颜色解码）和 ROS/NumPy 生态。**无环**。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `map_archive.py` | 307 | `...localization_mapping.geometry.cloud_records`、rclpy、numpy | 无 import 方（`../launch/components.py` 以 `-m` 启动） | 可接受：3 个类/函数块边界清楚，但 `save()` 48 行把健康检查串成 4 个 early-return |
| `__init__.py` | 0 | 无 | 无 | 清楚：占位 |

四类问题：

- **上帝文件**：无（307 行）。
- **重复实现**：`cloud_records` 在 `../geometry.py` 里，本目录不重复解码逻辑；但 `metadata()` 与 `save()` 各调一次 `self.metadata()` 取健康值，属正常复用。
- **跨层直达**：无。唯一的隐式跨层是话题名：`/demo/slam/body_odom`、`/cloud_registered_full` 等由 launch 重映射，代码里写的是"期望名"，**实际图里这些名字可能是 raw 或 corrected**——`../launch/components.py` 把 `/cloud_registered_full` remap 到 `/demo/slam/raw_registered_full`、把 `/demo/slam/body_odom` remap 到 `/demo/slam/raw_body_odom`，而 `/cloud_registered` **没有 remap**。
- **隐式契约**：`BROWSER_DTYPE` 的 16 字节布局同时被 `format` 字段（`xyz`/`rgb`/`padding_offset` 字符串）和浏览器端约定引用，两处需同步；`counts` 的 key（`rejected_frame`、`rejected_clock_reset`、`rejected_format`、`empty_colored_cloud` 等）是动态累加的，没有枚举定义。

结论：耦合**清楚**（单文件、单向依赖 `geometry`），主要成本是 307 行里节点类混合了订阅、健康门、快照、服务四类职责，但都在 `MapArchive` 一个类里、可读。

## 可读性评估

- **可接受**：`map_archive.py`——函数 docstring 0.0（12 个函数全无），但模块 docstring 与类 docstring 写清了二进制格式与"仅累积本 run 观测"；`save()` 48 行、`metadata()` 43 行是较长函数，均为线性 early-return；魔法数有名字（`VoxelArchive.__init__` 的默认参数）而非散落字面量。
- **良好**：`atomic_write`/`pcd_bytes`/`VoxelArchive` 三段职责单一，`pcd_bytes` 的 header 自带字段说明。
- **需改进**：健康门阈值的 `2`（秒）在三处内联出现（`slam_healthy`、`camera_healthy`、`save` 的 `message_age >= 2`），没有具名常量；`counts`/`ages`/`stamps` 的动态 key 无声明。
- 这是**文档问题**（缺 docstring）而非结构问题：单文件 307 行、无 import 环、无跨层。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把健康窗口 `2.0` s 提为模块常量（如 `HEALTH_WINDOW_S`），三处引用同一常量 | 现在改阈值要改三处，容易漏 | 无（行为等价） | 单测（目前没有）或人工比对 `metadata()` 输出 |
| P2 | 给 `MapArchive.snapshot/observe/cloud/metadata/save` 补 docstring | 12 个函数 0 docstring，接手者只能靠读实现猜语义 | 无 | 人工复核 diff |
| P2 | 把 `counts` 的 key 与 `format` 字段提为常量/`TypedDict` | 这些字符串是对外元数据契约，浏览器/离线消费方按名读 | 低 | 用一次假节点跑 `metadata()`，断言 key 集合不变 |
| P3 | 记录 `/cloud_registered` 不被 remap、而 `/cloud_registered_full` 被 remap 的原因 | 同名/近名话题在两张图里含义不同，是本仓库已知陷阱 | 无 | 人工复核 |

没有 P0/P1：本模块不参与控制、不写控制命令，且"拒绝覆盖旧地图""健康门才保存"两道保护都在代码里；307 行单文件也不构成结构债。

## 运行与输入输出

- 唯一实现 [map_archive.py](map_archive.py)，入口是 `python3 -m embodied_navigation.localization_mapping.mapping.map_archive --run-dir <run>`。
- 订阅 `/cloud_registered`（彩色点云，frame 必须是 `camera_init`），同时观察 `/livox/lidar`、`/livox/imu`、`/camera/image_color`、`/demo/slam/body_odom`、`/cloud_registered_full` 的新鲜度；写出 `map_metadata.json`、不可变的 `colored_map.NNNNNN.bin`（每点 16 字节：XYZ float32 + RGB uint8 + padding）与 `/demo/slam/map_status`。
- 保存服务是 `/demo/slam/save_map`（`std_srvs/Trigger`）：只有在 SLAM、相机与彩色点云都健康且没有容量拒绝时才写 `colored_map.pcd`；已存在 `map_metadata.json` 的运行目录会被拒绝复用。
- 由 [../launch/components.py](../launch/components.py) 以 `map_archive` 角色启动，并把输入 topic 重映射到运行目录的原始流；`--run-dir` 也可用 `DEMO_RUN_DIR`，没有对应变量会直接报错，避免覆盖旧地图。

## 测试与限制

本模块无专用测试（`tests/` 中没有文件 import `mapping.map_archive`），也没有在本次核对中运行 ROS 验证地图保存。体素大小默认 0.08 m、容量上限 8,000,000 点，都是代码内现有常量；`VoxelArchive` 是纯 Python 循环（逐体素 `for` 写入），大点云下的吞吐未验证。`cloud_records` 依赖 `../geometry.py`，该函数的输入校验（16 字节布局、有限值）由 `geometry.py` 负责，本模块不做二次校验。
