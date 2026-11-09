# perception：图像、深度与行人分割

本模块把图像、深度与像素对应关系变成下游可用的数据；不承担 SLAM、定位、导航控制或模型训练。数组处理与 GPU 推理入口分开，导入本包不加载权重。

## 文件与边界

- [depth.py](depth.py)：`depth_to_cloud(...)` 将深度反投影到 Go2 IMU 机体坐标，按有限值、量程和机体包围盒过滤，保留原始像素对应与固定采样上限。输入是数组、标定和安装参数，没有 ROS/GT 位姿读取。
- [person_segmentation.py](person_segmentation.py)：纯 NumPy 的关键帧读取、像素 mask 到点对应、known-sample confusion/metrics、真实 source_ns 的周期取样，以及相邻选中帧的临时二维 mask-IoU ID；不导入 Torch/SDK/ROS，不下载模型，不将未证实背景认证为 STATIC。
- [evaluate_dynanav_person_segmentation.py](../../../scripts/evaluate_dynanav_person_segmentation.py)：离线 Torchvision Mask R-CNN GPU 入口，显式使用本地权重和新输出目录。模型输入只有 RGB；模型不接收 depth、pose、oracle 或文本提示。
- [learned_person_mapping.py](learned_person_mapping.py)：只将预测person点标DYNAMIC，其余UNKNOWN；重叠mask按最高score分配，生成单帧instance ID，由LayeredMap进行后续三维可见中心关联。
- [reconstruct_dynanav_learned_person_map.py](../../../scripts/reconstruct_dynanav_learned_person_map.py)：缓存预测+exact实测LIO pose的离线对象历史/context入口，不重复GPU推理。
- [sensors/](sensors/README.md)：感知侧 ROS 转发、自车过滤、健康门和反馈邮箱；跨层数据契约另见顶层 [sensors](../sensors/README.md)。

离线评分器明确读取同批 cloud sample 的 oracle states 和 pedestrian instance paths 作标签，STATIC 为负例、UNKNOWN 不评分，且不读取归档位姿。它与 RGB-only 模型推理是不同数据边界，不能宣称评测过程完全无真值。二维 ID 不是三维 re-ID 或长期对象身份。

## 实际运行与调用

DynaNav [capture](../simulation/dynanav/README.md) 调用深度处理；当前 live static-cloud 分支仍使用 simulator oracle，尚未被本离线模型替换。74 个已存关键帧已完成真实本机 GPU 推理；全帧 sampled-pixel micro Precision/Recall/IoU 为 0.923885/0.824131/0.771733，详见[实验续篇](../../../docs/experiments/dynanav-20261109-continuation.md)。2 s 子集只是从相同预测缓存选择 37 帧，不代表新的 live 推理实验。缺预测的帧和未证实背景保持 UNKNOWN。

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

脚本不会自动下载权重。当前 score/mask 阈值 0.5、二维关联 IoU 0.25 是实验参数，不是历史协议或通用阈值。新增两场景score审查支持把0.1作为下一隔离operating point，但未改默认或原地图；仍有无候选的远小目标/近裁切漏检，降低阈值不能创造候选。详见续篇。

## Learned对象历史的离线重建

实测all74选中69 exact pose、5初始化前跳过，DYNAMIC12,376/UNKNOWN535,760；2s缓存子集37对应34+3，DYNAMIC6,981/UNKNOWN263,115。两者STATIC均0，未证实背景不被当成可导航静态图。分配的2/1个person ID不是re-ID精度；最终TTL过期不能证明无人。

该步骤不读取oracle mask/instance ID或归档GT变换，新mask来源为learned、直接位姿来自SLAM。**上游旧landing LIO仍用oracle过滤的static cloud，因此整条链尚未oracle-free。** 当前是离线缓存/source-aligned整合，不测live推理延迟、不替换在线oracle，也不接导航控制。

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

`--pose-launch-receipt`缺省尝试bag旁launch.json，缺预处理来源时严格记unknown而不是默认oracle-free。当前 [v2 provenance补注](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-perception-20261109T055000Z/learned-scenes-pose-provenance-addendum.json) 是原产物的metadata overlay，保留as-run数值和哈希；不是重新推理或重建。

`--scene-person-count`只是可选评测注解，不影响关联。输出scene_map/person_history/query/context及SHA来源收据；没有新STATIC证据，所以空static_map是有意保留的边界。

## 验证

[test_perception.py](../../../tests/unit/test_perception.py) 对比深度投影/自过滤公式；[test_person_segmentation.py](../../../tests/unit/test_person_segmentation.py) 验证对应关系、UNKNOWN 隔离、周期选择与临时 ID。[test_learned_person_mapping.py](../../../tests/unit/test_learned_person_mapping.py) 另验证保守点分类/单帧ID及三维地图接入。GPU 实测是独立证据，单元测试不代替模型质量、实时 deadline、完整地图覆盖或物理导航验收。
