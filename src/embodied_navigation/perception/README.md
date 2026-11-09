# perception：图像、深度与行人分割

本模块把图像、深度与像素对应关系变成下游可用的数据；不承担 SLAM、定位、导航控制或模型训练。数组处理与 GPU 推理入口分开，导入本包不加载权重。

## 边界

- 纯 NumPy 部分（`depth.py`、`person_segmentation.py`、`person_tiling.py`、`person_worker.py`、`temporal_geometry.py` 等）不导入 Torch/SDK/ROS、不下载权重、不读 GT 位姿；GPU 推理只在 `scripts/` 的独立入口，见下文"实际运行与调用"。
- 离线评分器明确读取同批 cloud sample 的 oracle states 和 pedestrian instance paths 作标签，STATIC 为负例、UNKNOWN 不评分，且不读取归档位姿。它与 RGB-only 模型推理是不同数据边界，不能宣称评测过程完全无真值。二维 ID 不是三维 re-ID 或长期对象身份。
- 本模块不认证永久静态：`temporal_geometry.py` 与 `person_geometry_tracking.py` 的输出是临时、可撤销的几何证据。
- 跨模块边界与四条执行路径见 [docs/architecture.md](../../../docs/architecture.md)。

## 目录结构

```text
src/embodied_navigation/perception/
├── sensors/  # 子模块，见 sensors/README.md
├── README.md  # 本文件
├── __init__.py  # 1 行，只有模块 docstring
├── depth.py  # 115 行，深度反投影到 Go2 IMU 帧的点云与像素对应
├── learned_person_mapping.py  # 196 行，learned 行人预测→保守点证据
├── person_geometry_tracking.py  # 822 行，有界 person ID、可见几何 motion 与 AABB 隔离
├── person_mask_propagation.py  # 309 行，opt-in 有界 RGB 光流 mask 传播
├── person_segmentation.py  # 178 行，纯像素对应、confusion/metrics 与临时二维 ID
├── person_tiling.py  # 87 行，full+tile 掩码提升与 IoU 去重
├── person_worker.py  # 481 行，person 推理 IPC 客户端、触发策略与证据分档
└── temporal_geometry.py  # 380 行，有界重复几何候选缓存
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | 9 个 `.py` 共 2569 行（其中 `__init__.py` 只有 1 行 docstring） | 全部是活代码；没有兼容门面、历史保留文件、fixture 或生成物。`depth.py` 被 [simulation/dynanav/capture/frames.py](../simulation/dynanav/capture/frames.py) 当库函数调用；4 个 person 相关文件被 `simulation/dynanav/learned_mapping.py` 与 `scripts/` 消费。 |
| 日志/输出 | 无固定位置 | 本模块不直接写日志：8 个文件里 `getLogger` 命中 0、`print` 命中 0。唯一写盘在 `person_worker.py`（`atomic_npz`/`atomic_json`/`event_log`），路径全部由调用方给入，写 `<channel>/requests/*.npz`、`<channel>/results/*.npz`、`<channel>/client-events.jsonl`、`<channel>/stop.request.json`、`<channel>/request.lock`；这些是本模块的 IPC 状态，不是 run 证据目录。 |
| 配置 | 无配置读取点 | 不 import `common.config`、不读 `os.environ`、不读任何 JSON 配置（grep 命中为空）。行人相关键（`person_worker`、`person_model_sha256`、`person_max_interval_s` 等）由调用方 [simulation/dynanav/learned_mapping.py](../simulation/dynanav/learned_mapping.py) 从 profile 取出后以参数注入。 |

run 目录与产物布局不在本模块定义，读取顺序见 [configs/README.md](../../../configs/README.md)，产物布局见 [审查文档 §9](../../../docs/review/dsh-workflow-and-readability-20261109.md)。

## 模块内文件耦合

依赖方向单一：`person_segmentation.py` 是被复用最多的底层工具（2 个本包文件 + 2 个 scripts 导入），其余文件向它汇聚；`person_worker.py` 只依赖 stdlib，是 IPC 契约层；本包不导入上层模块（`perception` 只被 `simulation/`、`scripts/`、`tests/` 导入），无环、无反向依赖。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 1 | 无 | 无 | 清楚 |
| `depth.py` | 115 | numpy | `simulation/dynanav/capture/frames.py` | 清楚（纯函数库） |
| `person_segmentation.py` | 178 | numpy | `learned_person_mapping.py`、`person_geometry_tracking.py`、`scripts/evaluate_dynanav_person_segmentation.py`、`scripts/reconstruct_dynanav_learned_person_map.py` | 清楚（基础工具） |
| `learned_person_mapping.py` | 196 | `person_segmentation` | `simulation/dynanav/learned_mapping.py`、`scripts/reconstruct_dynanav_learned_person_map.py` | 清楚 |
| `person_geometry_tracking.py` | 822 | `person_segmentation` | `simulation/dynanav/learned_mapping.py` | 偏重（唯一 >800 行文件） |
| `person_worker.py` | 481 | numpy（无项目内模块） | `person_mask_propagation.py`、`simulation/dynanav/learned_mapping.py`、`scripts/run_dynanav_person_worker.py` | 清楚（文件 IPC 契约） |
| `person_mask_propagation.py` | 309 | `person_worker` | 无生产导入者（只有 `tests/unit/test_person_mask_propagation.py`） | 可疑（未接线） |
| `person_tiling.py` | 87 | numpy | `scripts/run_dynanav_person_worker.py` | 清楚 |
| `temporal_geometry.py` | 380 | numpy（无项目内模块） | `simulation/dynanav/learned_mapping.py` | 清楚 |

四类问题核查：

- 上帝文件：有。`person_geometry_tracking.py` 822 行（超过 800 行线），其中 `PersonGeometryTracker.update()` 占 361–822 行（462 行）。
- 重复实现：没有。三个纯 helper（segmentation/tiling/geometry）各自处理不同输入，未发现复制粘贴的同一算法。
- 跨层直达：没有。本包不 import `navigation`、`control`、`launch`；反向依赖（`simulation`、`scripts` 消费本包）符合分层。
- 隐式契约：有一处但带校验。`person_worker.py` 的 IPC 用文件名（`request.lock`）、npz 字段名（`source_ns`/`frame_id`/`request_id`/`session_id`/`rgb_sha256`）和 JSONL 事件名（`request_submitted`/`result_received`/`request_failed`）作为跨进程契约，接收端用 `decode_result(path, expected)` 逐字段校验，不是无检查的字符串约定。

结论：耦合方向清楚、无环；唯一的结构负担是 `person_geometry_tracking.py` 的体量与单函数长度，而不是依赖关系本身。`person_mask_propagation.py` 没有生产消费者，属于暂停前的 opt-in 试验代码。

## 可读性评估

按 AST 实测指标（函数总数 / 有 docstring 函数比 / 最长函数 / >80 行函数数 / 注释行），抽查 `depth_to_cloud`（30–65 行）与 `PersonGeometryTracker.update`（361–822 行）确认指标可信。

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `depth.py` | 良好 | 4 函数，docstring 2/4=0.5，最长 36 行，0 个 >80 行函数，3 注释行；但存在魔法数（0.05/20 m 量程、`_POINT_BUDGET=8000`、自车盒 0.5/0.3/-0.45/0.3） |
| `person_segmentation.py` | 良好 | 8 函数，docstring 4/8=0.5，最长 47 行，0 个 >80 行函数 |
| `person_tiling.py` | 可接受 | 3 函数，docstring 1/3=0.333，最长 44 行；逻辑短但函数缺说明 |
| `learned_person_mapping.py` | 可接受 | 5 函数，docstring 0.8，最长 60 行，0 个 >80 行函数 |
| `person_worker.py` | 需改进 | 23 函数，docstring 0.13，最长 112 行（`PersonWorkerClient.predict`），1 个 >80 行函数 |
| `person_mask_propagation.py` | 需改进 | 8 函数，docstring 0.25，最长 177 行（`PersonMaskPropagator.update`），1 个 >80 行函数 |
| `person_geometry_tracking.py` | 需改进 | 11 函数，docstring 0.182，最长 462 行（`update()`），2 个 >80 行函数，仅 5 注释行 |
| `temporal_geometry.py` | 需改进 | 14 函数，docstring 0.214，最长 168 行（`TemporalGeometryCache.update`），1 个 >80 行函数 |

区分两类问题：**结构问题**只集中在 `person_geometry_tracking.py`（单文件 822 行 + 单函数 462 行）；其余多是**文档问题**——核心算法本身短、命名与分段可读，只是缺函数级 docstring（`person_worker`/`temporal_geometry`/`person_mask_propagation`/`person_tiling` 的 docstring 比都在 0.33 以下）。没有发现静默 `except` 吞错：`person_worker.py` 只捕获 `(OSError, ValueError, PersonWorkerError, TimeoutError)` 并记录 `request_failed` 事件后返回 `None`，失败是可观测的。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `PersonGeometryTracker.update()`（`person_geometry_tracking.py:361-822`）按阶段抽成私有步骤函数（解析 pose → 关联 track → 更新 motion/AABB → 输出 quarantine） | 462 行单函数、docstring 0.182，改动时无法局部理解；这是全模块唯一超过 800 行的文件 | 中：身份/隔离边界对数值敏感 | 先跑 `tests/unit/test_person_geometry_tracking.py` 与 `tests/unit/test_dynanav_learned_wiring.py` 建立基线，再逐段抽取，每步保持输入输出与事件序列不变 |
| P2 | 给 `person_worker.py`（docstring 0.13）、`temporal_geometry.py`（0.214）、`person_mask_propagation.py`（0.25）、`person_tiling.py`（0.333）补函数 docstring，重点写清"输入/输出/何时返回 None" | IPC 与证据语义主要在 docstring 传递，新人无法从签名判断"失败是否写事件" | 低 | 仅文档改动；用 `python3 -m pydoc` 或人工抽查 |
| P2 | 把 `depth.py` 的魔法数提为命名常量：量程 0.05/20 m、`_POINT_BUDGET=8000`、自车包围盒 0.5/0.3/-0.45/0.3 | 这些数值同时出现在测试与文档里，散落在函数体内易改漏 | 低 | `tests/unit/test_perception.py`、`tests/unit/test_depth_packet.py` 数值不变 |
| P2 | 明确 `person_mask_propagation.py` 的去留：补一句"未接 capture/experiment、当前基线未启用"，或迁到 experiments | 无生产导入者，容易被误读为在线能力 | 低 | grep 导入关系 + 人工确认 |

本轮只写建议、不改代码；任何拆分都应先有对照测试再动。

## 关键文件与接口（原"文件与边界"）

- [depth.py](depth.py)：`depth_to_cloud(...)` 把深度反投影到 **IMU 帧**（`points_in_imu_frame`）：原点相对 base 差 `imu_mount` 安装偏移，相机安装点由 `camera_mount` 标定，**不是 COM 偏置**。函数内自车包围盒在 base 帧判断，所以返回点与包围盒检查用的帧不同。按有限值、量程和机体包围盒过滤，保留原始像素对应与固定采样上限。输入是数组、标定和安装参数，没有 ROS/GT 位姿读取。`depth_cloud_packet(...)` 返回 `DepthCloudPacket`（`points`/`pixel_flat_indices`/`depth_keep_mask`/`self_mask`），像素索引在有效深度过滤、自车 mask、同一 stride 之后仍与原图对齐。
- [person_segmentation.py](person_segmentation.py)：纯 NumPy 的关键帧读取、像素 mask 到点对应、known-sample confusion/metrics、真实 source_ns 的周期取样，以及相邻选中帧的临时二维 mask-IoU ID；不导入 Torch/SDK/ROS，不下载模型，不将未证实背景认证为 STATIC。
- [evaluate_dynanav_person_segmentation.py](../../../scripts/evaluate_dynanav_person_segmentation.py)：离线 Torchvision Mask R-CNN GPU 入口，显式使用本地权重和新输出目录。模型输入只有 RGB；模型不接收 depth、pose、oracle 或文本提示。
- [learned_person_mapping.py](learned_person_mapping.py)：只将预测 person 点标 DYNAMIC，其余 UNKNOWN；重叠 mask 按最高 score 分配，生成单帧 instance ID，由 LayeredMap 进行后续三维可见中心关联。
- [reconstruct_dynanav_learned_person_map.py](../../../scripts/reconstruct_dynanav_learned_person_map.py)：缓存预测+exact 实测 LIO pose 的离线对象历史/context 入口，不重复 GPU 推理。
- [person_worker.py](person_worker.py) / [run_dynanav_person_worker.py](../../../scripts/run_dynanav_person_worker.py)：前者是 **IPC 客户端 + 触发策略 + 证据分档**（不 import torch/SDK/ROS，不加载模型，也不依赖 `ROS_DOMAIN_ID`）；后者是 **模型所有者**，持久加载一次 Mask R-CNN 并循环推理。"模型一次加载"属于后者；exact 请求身份与 source-time 事件调度由两者共同实现，不会把失败/旧结果当新语义。
- [person_tiling.py](person_tiling.py)：可选 full+tile 掩码提升/合并。4 个旧 miss 的真实 tile2 诊断没有新增 TP、增加 62 sampled FP，在线仍用 tile1，不宣称改善。
- [person_geometry_tracking.py](person_geometry_tracking.py)：纯 NumPy 内部 person ID、可见几何 motion 与短时 AABB 隔离；不读 GT、不把背景变 STATIC，二维支持/因果 pose 范围显式保留。
- [person_mask_propagation.py](person_mask_propagation.py)：暂停前新增的 opt-in 有界 RGB 光流 helper；只有 5 项注入 flow 纯测试，未接 capture/experiment、未跑真实 RGB 重放或在线效果/延迟验收，本轮基线未启用。
- [temporal_geometry.py](temporal_geometry.py)：独立有界重复几何候选缓存，3 distinct 帧/≥0.5s、retention 300s/200k voxel/8 票；危险可撤销并导出 source/index ledger，不认证永久静态。
- [sensors/](sensors/README.md)：感知侧 ROS 转发、自车过滤、健康门和反馈邮箱；跨层数据契约另见顶层 [sensors](../sensors/README.md)。

## 实际运行与调用

DynaNav [capture](../simulation/dynanav/README.md) 调用深度处理；默认 oracle 分支保留；显式 `learned_rgb_person` 现已接入独立 worker 与临时前端候选，45s 真实在线片段已独立核对：118 次真实推理、漏过 person 点 36.26%（主要来自未推理帧），历史几何准入 344 个 person 点；最终保留 ledger 无 person 也不抹去历史问题，见[本阶段报告](../../../docs/experiments/dynanav-online-person-20261109.md)。前述离线预测本身不等于在线验收。74 个已存关键帧已完成真实本机 GPU 推理；全帧 sampled-pixel micro Precision/Recall/IoU 为 0.923885/0.824131/0.771733，详见[实验续篇](../../../docs/experiments/dynanav-20261109-continuation.md)。2 s 子集只是从相同预测缓存选择 37 帧，不代表新的 live 推理实验。缺预测的帧和未证实背景保持 UNKNOWN。

从工程根目录复现离线评测，输出须为新目录；权重、输入、预测和图片均留外部数据卷：

```sh
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
DYNANAV_DATA="$PROJECT_DATA_ROOT/embodied-navigation"
python3 scripts/evaluate_dynanav_person_segmentation.py \
  --keyframes "$DYNANAV_DATA/experiments/dynanav-20261109-slam-landing-sdk200/capture/mapping/keyframes" \
  --weights "$DYNANAV_DATA/models/perception/maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth" \
  --expected-sha256 73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e \
  --output "$DYNANAV_DATA/experiments/person-eval-$(date -u +%Y%m%dT%H%M%SZ)" \
  --device cuda:0 --mode compare
```

脚本不会自动下载权重。此离线评测默认 score/mask 阈值 0.5、二维关联 IoU 0.25 是实验参数，不是历史协议或通用阈值。两场景审查当时支持 0.1 作为下一隔离 operating point；本轮独立在线 worker 显式选 0.1，离线 CLI 默认和原地图保持，仍有无候选的远小目标/近裁切漏检，降低阈值不能创造候选。详见续篇。

## Learned 对象历史的离线重建

实测 all 74 选中 69 exact pose、5 初始化前跳过，DYNAMIC 12,376/UNKNOWN 535,760；2s 缓存子集 37 对应 34+3，DYNAMIC 6,981/UNKNOWN 263,115。两者 STATIC 均 0，未证实背景不被当成可导航静态图。分配的 2/1 个 person ID 不是 re-ID 精度；最终 TTL 过期不能证明无人。

该步骤不读取 oracle mask/instance ID 或归档 GT 变换，新 mask 来源为 learned、直接位姿来自 SLAM。**上游旧 landing LIO 仍用 oracle 过滤的 static cloud，因此整条链尚未 oracle-free。** 当前是离线缓存/source-aligned 整合，不测 live 推理延迟、不替换在线 oracle，也不接导航控制。

使用原评测缓存复跑，两个模式各写一个新目录（`PREFIX-all` / `PREFIX-periodic2s`）：

```sh
python3 scripts/reconstruct_dynanav_learned_person_map.py \
  --evaluation "$DYNANAV_DATA/experiments/person-perception-20261109T055000Z/evaluation" \
  --bag "$DYNANAV_DATA/experiments/dynanav-20261109-lio-slam-landing-sdk200/bag" \
  --pose-launch-receipt "$DYNANAV_DATA/experiments/dynanav-20261109-lio-slam-landing-sdk200/launch.json" \
  --pose-preprocessing-receipt "$DYNANAV_DATA/experiments/dynanav-20261109-slam-landing-sdk200/capture/mapping/receipt.json" \
  --output-prefix "$DYNANAV_DATA/experiments/learned-person-$(date -u +%Y%m%dT%H%M%SZ)" \
  --mode compare
```

`--pose-launch-receipt` 缺省尝试 bag 旁 launch.json，缺预处理来源时严格记 unknown 而不是默认 oracle-free。当前 [v2 provenance 补注](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scenes-pose-provenance-addendum.json) 是原产物的 metadata overlay，保留 as-run 数值和哈希；不是重新推理或重建。

`--scene-person-count` 只是可选评测注解，不影响关联。输出 scene_map/person_history/query/context 及 SHA 来源收据；没有新 STATIC 证据，所以空 static_map 是有意保留的边界。

## 验证

[test_perception.py](../../../tests/unit/test_perception.py) 与 [test_depth_packet.py](../../../tests/unit/test_depth_packet.py) 对比深度投影/自过滤公式及像素对应；[test_person_segmentation.py](../../../tests/unit/test_person_segmentation.py) 验证对应关系、UNKNOWN 隔离、周期选择与临时 ID；[test_person_geometry_tracking.py](../../../tests/unit/test_person_geometry_tracking.py)、[test_temporal_geometry.py](../../../tests/unit/test_temporal_geometry.py)、[test_person_worker.py](../../../tests/unit/test_person_worker.py)、[test_person_mask_propagation.py](../../../tests/unit/test_person_mask_propagation.py)、[test_person_tiling.py](../../../tests/unit/test_person_tiling.py) 覆盖各自纯逻辑。[test_learned_person_mapping.py](../../../tests/unit/test_learned_person_mapping.py) 另验证保守点分类/单帧 ID 及三维地图接入。GPU 实测是独立证据，单元测试不代替模型质量、实时 deadline、完整地图覆盖或物理导航验收。
