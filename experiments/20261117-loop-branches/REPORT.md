# 回环方案分支与比较约定

## 共同基础

所有分支保留完整提交链：已发布 `42fc6fa` → 回环实现 `8184b65` →
接线验收与报告 → 独立图像检索 `d070be8` → 可选视觉不足时保留合格激光候选
`ea85130` → 原验证报告 `8afb64a`。`8afb64a` 本身只追加报告。
原始实现同时加入共享关键帧 I/O、地图保存、节点配置和测试；本次不硬拆这些依赖。
`refactor/05-loop-evidence-base` 在该基础上只补交付说明、关闭新模块的对照配置及
用户授权上传的规则例外。三个 `exp/` 分支均从此共同基础派生，彼此不依赖。
它们是同一实现的实验配置分支，不是三套独立算法代码，也不等同于 `main` 上的独立小补丁。

共用基础还继承 `42fc6fa` 的追踪、评估和录制支持及此前导航提交；
这些是既有依赖，不是本次从活跃工作区取出的改动。

## 对比矩阵

各分支都用 `configs/localization_mapping/slam_loop_comparison.json` 选择本方案。
单纯切换分支不会改变默认运行行为，必须将此文件传给启动器。

| 分支 | 实测多帧 | 表面门 | 视觉检索 | 强制验证所有激光候选 | 地图体素 |
| --- | --- | --- | --- | --- | --- |
| `refactor/05-loop-evidence-base` | 关 | 关 | 关 | 关 | 默认 0.2 m |
| `exp/loop-measured-submaps` | 开 | 关 | 关 | 关 | 默认 0.2 m |
| `exp/loop-surface-gate` | 关 | 开 | 关 | 关 | 0.05 m |
| `exp/loop-visual-retrieval` | 关 | 关 | 开 | 关 | 默认 0.2 m |

表面方案另附 `slam_loop_surface_control.json`，使用同样 0.05 m 地图体素、关闭表面门，
用来隔离门控本身的影响。默认 0.2 m 地图体素会使 0.3 m 表面栅格通常不足 6 点，
即使一致的稠密平面也可能在地图保存稀疏化后变成不可观测，因此不能把默认密度直接当有效表面对照。
本次不放松表面点数、残差或覆盖门限。0.05 m 的计算与存储成本仍需服务器测量。

## 实际作用与限制

- 多帧：按同 epoch、时间间隔、相对位移、高度及旋转预算向过去取最多 5 帧，
  查询与历史端均进入 ICP。前端相对位姿只是有界假设，没有协方差认证。
- 表面：在已知重力对齐坐标中提取实测水平/竖直面，ICP 后按覆盖与残差拒绝不一致候选。
  配置 `gravity_aligned:true` 是数据前提声明，不是对当前录制的证明。
  只可用于已独立核实 odom 重力对齐的数据；未核实前不能直接拿此配置启动实验。
  水平桌面与地面不具语义区分，无法排除全部重复台阶错误，也不提供地面真值或高度锁定。
- 视觉：16×16 图像均值描述子独立提名，与 ScanContext 合并去重；视觉独立候选必须通过
  实测激光投影 patch、3D RANSAC、ICP、视觉/ICP 一致性及原运动门。
  可选视觉不足时，原本合格的 ScanContext 候选仍可按激光路径继续。
  轻量描述子不保证一般视角不变性；索引最多 2000 项，淘汰会损失长路线召回。
- 所有方案接受的回环仍只有原 ICP 图边和信息矩阵；没有独立视觉因子、地面因子、
  前端状态回灌或持续 Z 校正。导航仍消费原前端里程计。
  证据命令：`git grep -n 'add_edge' -- src/embodied_navigation/localization_mapping/backend/backend.py`，
  返回 odometry 与 loop 两处；`_search` 的新模块只影响候选和准入。
- 先验地图 global search 部分经过新模块；进入 prior tracking 后 `_track` 直接 `_register`，
  不走新视觉/表面门。比较范围是建图回环，不能宣称覆盖全重定位流程。

## 启动和输入

```bash
bash scripts/run_dynanav_native_exploration.sh "$NEW_RUN_DIRECTORY" \
  --backend-evidence-config configs/localization_mapping/slam_loop_comparison.json \
  --record-slam-config configs/localization_mapping/slam_recording.json
```

运行目录必须新建在 `$PROJECT_DATA_ROOT/embodied-navigation/runs/` 下。
启动器会把配置冻结到 run/configuration，控制器只通过启动器启动。
不得对正在录制的 run 原地切换配置。各分支使用同一录制离线比较，或在独立新 run 中运行。

视觉配置故意不填猜测标定。离线关键帧需有合法 `visual.npz`：
`gray`、`K`、`T_body_camera`、`stamp_ns`；在线需在仓库外准备完整 JSON，
保留本分支 backend 字典并追加 `visual_input`，字段如下：

```text
rectified: true
image_topic: 实际整流图像话题
camera_info_topic: 对应 CameraInfo 话题
T_body_camera: 实测 optical-camera 到 body 的 4×4 外参
max_time_delta_s: 0.05
```

将该完整 JSON 传给同一 `--backend-evidence-config` 参数。不得用单位矩阵冒充外参。
未提供 `visual_input` 时节点不订阅相机；若关键帧本身也无图像，视觉分支只会运行原 SC/ICP。
必须核对 trace 中 `visual_candidates`、`visual_verification`、`merged_search_outcome`，
不能把这种退化运行报告为视觉闭环。已有稀疏截图不等于同步关键帧图像。

## 验收要求

按同一 Python 3.12、numpy/scipy 环境，在各分支的独立 LF 克隆运行：

```bash
PYTHONPATH=src python3 -B -m compileall -q src scripts
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

相对于本次重跑的 `42fc6fa` 起点，失败名单不得新增；不删除旧断言、加 skip 或改超时。
历史 `8afb64a` 报告的测试数不替代本次验收，最终分支、提交和复验结果列在交付回执。
合成测试只证明接线和数值契约，不是定位效果或完整两层探索实测。
本次不启动 ROS/Isaac，不干预其他对话的仿真与录制。
服务器应重点对照误回环、候选拒绝原因、图像缺失率、表面可观测性、时间与内存成本、
Z 修正和轨迹误差，并保留原始录制及每个新 run 的冻结配置。
