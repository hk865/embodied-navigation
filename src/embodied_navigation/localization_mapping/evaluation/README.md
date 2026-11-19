# SLAM 受控实验记录与评价

本包只观察和离线评价。它不发布速度、不修改位姿、不把真值送给导航。

## 运行链

```text
LIO ────────────┐
post-VIO ───────┼─→ recording → 同源完整位姿 JSONL
backend online ┘
关键帧 → 后端原算法 → Trace → 候选、ICP、图边与优化前后状态
仿真真值 ──────────────────→ 结束后 metrics
```

正式导航仍使用 post-VIO。在线后端纠正位姿是当时的纠正矩阵左乘新位姿；最终优化后的历史关键帧是另一份结果，评价时分开列出。

## 启动

在项目根目录，使用已有启动器附加记录配置：

```bash
bash scripts/run_dynanav_native_exploration.sh "$RUN_DIR" \
  --record-slam-config configs/localization_mapping/slam_recording.json \
  --until-stopped
```

其余仿真场景、导航模式和速度沿用该实验的显式启动配置。控制器只由启动器启动，不能单独另起控制进程。启动器把本次记录配置复制到 `$RUN_DIR/configuration/recording.json` 后，启动独立订阅进程。后端实际展开参数保存在 `backend_trace/events.jsonl` 的 configuration 事件中。

三个原始位姿文件放在 `trajectories/`：

| 文件 | 来源 |
|---|---|
| `frontend_lio.jsonl` | LIO 阶段原始位姿 |
| `frontend_post_vio.jsonl` | 视觉更新后原始位姿 |
| `backend_online.jsonl` | 在线纠正位姿 |
| `backend_status.jsonl` | 后端计数与错误状态，仅供辅助观察 |
| `health.json` | 接收条数、最后源时间、写盘错误与队列状态 |

每条保留整数 `stamp_ns`、父/子坐标系、XYZ、原始 xyzw 四元数、完整变换矩阵、派生 xyz 外旋 RPY（弧度）和原始协方差。RPY不代替四元数；后端发布的协方差未做纠正变换，不能把它当成已校准后验不确定性。接收时间单独记录，不能冒充采样时间。

## 阶段标记

每一圈、上楼、平台、下楼和停车分别标记，时间来自当前 post-VIO 观测：

```bash
PYTHONPATH=src python3 -m embodied_navigation.localization_mapping.evaluation.stage_events \
  --run-dir "$RUN_DIR" --stage basement_loop_1 --event start
```

结束时将 event 改为 end。阶段名只是记录，不改变任何导航决策。机器人停机后若当前观测已失效，应依据实际保存的源时间另作明确的离线标注，不捏造一个新的当前观测。

## 后端事件

`backend_trace/events.jsonl` 按独立序号记录：

- `loop_exclusions`：近期/时间间隔排除的关键帧及原因。
- `search_candidates` / `search_outcome`：候选距离、yaw和最终是否执行、验证、选中。
- `icp`：实际粗细结果，初值/结果 SE(3)、fitness、RMSE、converged及本次阈值。
- `loop_decision`：回环接受或拒绝、候选与图锚点、信息矩阵。
- `graph_before` / `graph_after`：节点完整姿态、测量边方向、信息矩阵、鲁棒权重、残差及优化返回结果。
- `correction`：提出的纠正矩阵、采纳状态和拒绝证据。
- `final_graph`：结束时最终图。

图内部保存逆测量，Trace导出的是实际 `T_a_b`；切空间顺序为平移 rho、旋转 phi。优化前后只复制位姿和图边，不重复复制点云。写盘复用已有后台 EvidenceWriter，I/O错误独立报告，不改变候选、阈值、优化或采纳结果。必须检查 `summary.json` 的 submitted、written、error，不能只看循环计数。

## 离线评价

仿真结束、最终地图保存后运行：

```bash
PYTHONPATH=src python3 -B -m embodied_navigation.localization_mapping.evaluation.metrics \
  --run-dir "$RUN_DIR"
```

输出新建 `evaluation/metrics.json`，不会覆盖已有评价。初始共同5秒必须由操作者确认稳定；必要时用 `--stable-start-ns` 指定实际稳定起点。

- 先把真值 go2_base 转成同一 go2_imu 原点，关键帧本来就是 IMU 状态原点，不再叠加外参。
- 初始姿态差的旋转均值加平移均值构成固定 SE(3) 对齐，所有结果共享；不拟合尺度，不按每个时间桶重新对齐。
- 绝对误差为 `inv(G) @ A @ X`，保存 XYZ、平移范数和 SO(3) 测地角；ATE是平移RMSE。
- 相对误差为 `inv(inv(G_i)@G_j) @ (inv(X_i)@X_j)`，分别取10秒和100秒；RTE明确为该误差的平移部分。
- 10/100秒统计桶和10/100秒相对间隔分别输出。阶段内RPE要求两个端点都在该阶段，样本不足显示空值。
- 位置线性插值、旋转SLERP，禁止外推；最终图只在关键帧时刻生成评价样本。
- 每次优化对同一节点保存 `T_after @ inv(T_before)` 和完整姿态变化，结合边与权重检查误差如何分摊；不能只减XYZ解释旋转纠正。

静止短测只验收记录链和数学实现，不是行走回环或楼梯性能验收。
