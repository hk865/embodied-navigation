# ADR-0005: 因果关键帧记忆、Mesh+骨架混合底座、树/图过程检索与 Route 1→2 规划

- 状态：已采纳草案 / 待首周三项门控验收（2026-11-28）
- 关联文档：[ROADMAP.md](../ROADMAP.md)、[TASKS.md](../TASKS.md)、[BASELINES.md](../BASELINES.md)、[ADR-0003](0003-streaming-fork-control-and-semantic-scene-graph.md)、[ADR-0004](0004-incremental-mesh-3dgs-and-layered-backend-eval.md)、[3D/4D 表征、混合检索与算力策略 Trade-Off](../research/2026-11-3d-representation-memory-reconstruction-tradeoffs.md)

---

## 1. 背景与核心目标

当前仓库在两条主线上各自跑出了硬底座：
- **L7 记忆与 Agent 路线（`feat/static-memory-streaming` @ `e159c8a`，948 单测 / 60 回归全绿）**：落地 `SAM 3 + SigLIP` 常驻视觉、Set-Cover 关键帧压缩（`79.54%`）、`GlobalRegionMemory`、`ObjectNavFinishVerifier`、三态熔断与 `SharedHabitatSceneHost`，5 场景 HM3D Dev 跑出 `SR 81.25% (13/16)`（95% Wilson CI `57%–93%`）、`OSR 87.50% (14/16)`、跨子任务 `81.82% (9/11)`、`0` 超时；
- **D 路径真物理与真 SLAM 路线（`exp/live-v4-ba-sliding-window` @ `aada1f1`）**：跑通 1h 51m / 343.72 m 两层带噪 IMU 闭环（Umeyama 终图 `2.02 cm` / 在线 `2.68 cm`）+ `29 MB` Object Surface Nets；另有 `feat/semantic-keyframe-graph@46e0b48`、`feat/agent-preemptable-active-slam@8aebc95`、`perf/slam-backend-cpp-icp@ffd7882` 与 `ref/dynanav-dynamic-pedestrian`。

本 ADR 用于解决多分支拼接带来的“草台感”，定下双栈统一的数据底账、各表征职责与 Route 1（刷穿 Habitat）→ Route 2（真实仿真 + 4D 动态）推进红线——核心要验证的能力是：**当位姿被回环修正、物品被搬走或被遮挡时，系统能结合当下观测、历史过程和推理先验，判断旧记忆是否还值得再跑一趟，并能把判断依据查出来。**

---

## 2. 核心架构决策

### 决策 1：单一不可变底账 + 纳入过程轨迹链与推理先验

1. **不可变观测原语**：全栈共用同一份权威日志，用不可变 `observation_id` 绑定 `(timestamp_ns, image_patch_ref, local_depth_or_lidar, submap_id, extrinsic_ver, feature_model_ver, lighting_anchor)`；
2. **过程记忆与推理先验正式入账**：在权威底账中显式同步记录 **SE(3) 刚体不变的过程数据**——**里程计步间位移轨迹链 (`Episodic Trajectory Chain`, $\Delta x, \Delta y, \Delta\theta$)**、关键帧访问顺序、以及探索中形成的**历史推理先验与语义直觉 (`Reasoning Prior & Semantic Intuition`，如区域功能假设、失败黑名单)**；
3. **版本化位姿与实例时序表**：实例仅引用 `observation_ids[]` 并记录 `existence_interval` 与关联置信度；位姿图仅发布带版本号的 `(submap_id, pose_revision)` 子图变换；`HabitatWorldDaemon` 对保留纹理帧同步落盘轻量 **16-bit 压缩深度 PNG**（单栋约 `+1.2 MB`），补齐双栈可重构闭环；
4. **必须守死的 4 条跨层接口红线**：
   - ① 统一 `pose_revision` 读取权限，禁止上下层混用回环前后两版位姿；
   - ② 异步视觉回包只准写回发起查询时的 `frame_id` 与实例版本；
   - ③ “目标已不在”的否定判断必须同时满足“在视野内 + 无几何遮挡 + 在有效测距内”；
   - ④ 航点、取消与 `STOP` 校验绑定 `action_epoch`。
   - **最小契约验收**：回放经历重复消息、视觉迟到与 1 次回环后输出一致可追溯的实例历史，历史前缀查询零未来证据泄漏。

### 决策 2：确认 Mesh+点云混合压缩与「骨架 + Mesh」预测地基，并单独门控“遮挡重排”

1. **明确 Mesh 在系统里的三大核心地位**：
   - **① 大平面压缩 + 细部残差**：地板、天花板、大墙面从大量点云压缩成少片三角 `Mesh`，复杂物体细节保留点云/高密观测残差（大面 Mesh + 细部点云）；
   - **② 骨架 + Mesh = 好建模、好预测的物理地基**：静态建筑拓扑骨架+支撑平面 Mesh 提供可达/放置先验；动态刚体 $T(t)$ 或铰接转轴骨架+Mesh 把数万碎点压成低维自由度运动学模型，支持前向物理预测；
   - **③ 无光照污染碰撞与资产出口**：脱离旧灯光烘培，直接服务物理避障与仿真资产；
   - **在线纹理优选原始 Patch 投影**（随时可回看原图）；`2DGS` 仅作未拟合视角下的离线/跨视角实验对照；引擎单材质 PBR 与 `GS-2M` 暂不入在线主链。
2. **独立裁决“Mesh 遮挡重排”的 3 天低成本实验（`exp/mesh-visibility-retrieval`）**：
   - 在 **≥ 3 个留出场景、120 个去重 `(query, viewpoint)` 对**（严守因果前缀）上，全量对比纯 Patch（A）、点云/体素射线（B）、历史深度重投影（便宜强对照）与 `Surface Nets` 首交（C）；
   - **注意范围边界**：本实验只决定“在线检索候选重排时加不加 Mesh 遮挡打分”（要求比最强便宜对照 Top-1 ≥ +5 pp、完全遮挡误留相对降 ≥ 25%、可见误杀 ≤ +2 pp、p95 ≤ 50 ms）；**即使子实验未达标，也不否定 Mesh 平面压缩和骨架+Mesh 建模地基的长期地位**。

### 决策 3：确立「树检索 + 时空图检索」上层方向，严格区分底层 ANN 加速

1. **绝不只靠纯外观向量干搜**：上层记忆检索正式立项 **层次树检索 + 四类边时空过程图检索**，联合利用四路信号：**外观特征 (`DINO/SigLIP` Patch) + SE(3) 不变里程计过程近邻 + 局部相对拓扑 + 推理语义直觉先验**：
   - **树检索**：沿 `建筑 → 楼层 → 房间/区域 → 探索过程段 → 关键帧/物体` 自顶向下筛选，门口/楼梯/开放区允许**多归属**，剪空时**永久保留全局回退**；
   - **时空过程图检索**：构建含 **① 里程计/时序过程边、② 门洞与相对几何邻接边、③ 跨视角外观关联边、④ 推理先验与语义共现边** 的记忆图，支持“入口召回 → 限制跳数 $h$ 的多跳过程扩散 → 空间校验”；
2. **特征索引红线与底层数据结构渐进演进**：
   - 外观向量严格按 `observation_id` 索引，**严禁将绝对世界坐标拼入高维视觉特征**（保证全球回环时特征库与过程相对边均免重算；绝对坐标过滤统一放在召回后执行）；
   - **首版固定简洁底座**：精确矩阵乘 $O(Nd)$ + 子图局部体素哈希 + 实例时序表 + 树/图检索；只有精确匹配在实测规模超延迟预算时才引入 `HNSW / FreshDiskANN`（带墓碑+增量压实）。若 BA 导致子图内部非刚性形变，子图局部体素/BVH 必须重投脏块。

### 决策 4：4D 动态实体存储分级与 Go2 跨模态传感器分工

1. **4D 动态分层**：依靠运动强先验切分前景——刚体移动物存局部 `Mesh + T(t)`；门/抽屉在观测充分且留出预测胜过占据时升级为 **父子 Mesh + 关节骨架**（`MoMa-SG` 思路）；**行人存储分两步**：阶段一导航避障优先存**时效 3D 占据体 + 速度 + 外观特征**（防半身遮挡瞎补 `SMPL` 腿位），阶段二出现近距肢体交互需求时再升格局部分段骨架。
2. **四类必防状态判据**：
   - 拆存 `movable_class`、`current_motion_evidence`、`last_existence_evidence`，停步行人不融进静态墙，**只准明确穿过旧址的实测自由射线削弱旧占据**；
   - 跨期变化在同一 `pose_revision` 先剥离静态背景共同位移，分存 `observation_time` 与 `system_learned_time`，显式区分 `not_visible`、`observed_empty_at_old_pose`、`identity_swapped`；遮挡交叉允许 `pending` 未决态。
3. **Go2 分工红线**：`RGB` 负责身份、交叉 ReID、细小候选与铰接线索；`32 线 LiDAR` 负责暗处制动净空、门扇剩余宽度、去畸变与静态定位（**严禁把 32 线束间空白当自由空间**；迁移 `Dynablox` 需先做束型与步态消融）；`IMU + 关节 + 接触` 做高频传播与打滑告警（**严禁把 `MoE-CTS` `cmd_vel` 当里程计**）。

### 决策 5：11 分支单假设路线图、既有分支复用与 Colab `1,000 CU` 封顶预算

1. **第一周三件门控交验**：① 官方 Scorer 逐题一致报告；② 3 天 Mesh 遮挡重排配对表；③ D 路径单次回环前后同物体因果回放。
2. **Route 1 → Route 2 11 分支单假设序列**（`H = feat/static-memory-streaming@e159c8a`，`D = exp/live-v4-ba-sliding-window@aada1f1`）：
   - **Route 1**：`1. exp/habitat-score-parity` (`H`) → `2. exp/memory-causal-replay` (`H`，补过程链/先验契约与 16-bit 深度) → `3. exp/mesh-visibility-retrieval` (`2`) → `4. exp/habitat-memory-retention` (`1`，含树/图过程记忆 vs 清空配对) → `5. exp/goal-frontier-prior` (`1`) → `6. exp/instance-reranking` (`Colab`, `2`) → `7. exp/stop-calibration` (`Colab`, `1`)；
   - **Route 2**：`8. exp/submap-pose-revisions` (`D`) → `9. exp/dynamic-free-space` (`8` + `ref/dynanav-dynamic-pedestrian`) → `10. exp/object-existence-history` (`9`，可选 `140 CU`) → `11. exp/agent-revisit-on-revision` (`10`)。
3. **既有分支按需提取（禁整枝硬并）**：`feat/semantic-keyframe-graph@46e0b48` 供分支 `2/10` 提取图定义；`feat/agent-preemptable-active-slam@8aebc95` 供分支 `11` 提取 `action_epoch` 抢占；`perf/slam-backend-cpp-icp@ffd7882` 仅剖面卡顿时单独验收；`ref/dynanav-dynamic-pedestrian` 提供动态复现。
4. **`Google Colab 1,000 CU` 分档封顶**：`30` 试跑 + `190` train 难负例库 + `210` 实例重排头 + `70` STOP 校准 + `140` 存在判别头 + `240` 全量官方批推理 + `120` 复验预留 = `1,000 CU`。禁训世界模型/VLA/大 VLM、禁验证集提前跑离线 GS/PBR 送在线、禁在高端卡空等仿真或网关。

---

## 3. 验收与复盘要求

- 所有实验分支沿用 [AGENTS.md](../../../AGENTS.md) 单分支单假设与逐行 diff 审阅，对比全量单测/回归失败名单，严禁删改旧断言。
- Route 1 须按官方原 scorer、`1.0/1.8 m`、5 场景 Dev 与盲测拆分报告；D 路径除 Umeyama 外须补固定尺度 SE(3)、起点对齐、分段 RPE、Z 误差、回环瞬间跳变及**决策当刻的目标局部位置误差**。
