# 六视角观察与共享 LIO 地图原型：真实 RTX 验证

方案 A 已跑通静止场景的真实六视角采集、六宫格/视频和共享实测点云投影。RTX 5070 Ti 在授权宿主命令中可用，驱动为 580.178.04；普通沙箱不暴露设备不能据此判断驱动故障。本次未改驱动、安全设置、估计器二进制或原始场景资产，未发送导航目标，未提交或推送 Git。

测试实际发生于 2026-11-11 19:42–19:48 UTC；文件名和数据目录沿用当前任务的 `20261112` 标签。

## 实现与本次增量

- `multicamera.py` 提供六台理想针孔相机的固定内外参，朝向为 0/60/120/180/240/300 度，分辨率各 512×512，水平视场约 82.37 度。新增五台相机，前相机直接复用原来的渲染产品及原生绑定帧。
- `capture/multicamera_sensor.py` 使用每台相机自己的原生 render-product 完成事件，保留 RGB、轴向深度、源时间与绑定证据；约每源秒归档一次。没有同步拼接全景，也没有把当前机身时间替换为像素源时间。
- `project_shared_map` 对同一份 LIO map-frame 点云取估计 IMU 位姿的逆变换，再用每台相机的外参和内参投影。视锥、像素 z-buffer 和同帧轴向深度共同筛选；缺失深度按未知处理。同一点可出现在相邻相机中，不按方位把点云永久分割。
- `scripts/dynanav_exploration.py multicamera` 输出独立视角六宫格、各相机叠点、原始输入点索引、源时间报告与可选视频。默认关闭；通过 `MULTICAMERA_OBSERVATION=1` 或实验 CLI 的 `--multicamera-observation` 启用。
- 上述主体由交接工作树继承。本轮只在其上修正了 `uint64` 时间戳相减回绕可能选错最近位姿的问题，添加“原生接受帧/归档帧/无效帧”的分相机收尾统计，并增加两项针对性测试。收尾统计新字段经 CPU 测试；下表 GPU 实验在加载此增量之前启动，不把新字段称为已获得 GPU 运行验证。

前相机继续是 FAST-LIVO2 唯一 RGB 输入。额外深度仅作观察投影的遮挡依据，不进入 LIO、PCT 或位姿估计。独立多路 VIO 和多相机位姿融合均未实现。

## 三次真实仿真

数据根目录：`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/multicamera_validation_20261112`。

使用既有 `run_dynanav_native_exploration.sh`、缓存 Office 场景、同一原始 spawn/yaw=0、同一 FAST-LIVO2 二进制、200 Hz IMU、10 Hz RTX OS1 与原前 RGB。每次 20 仿真秒/4,000 物理步，`NAVIGATION_BACKEND=native_route`，命令上限 0.5 m/s，但不提交目标，实际导航命令为零。机器人仍由站立策略驱动。没有改动原有年龄、外参或其他控制检查。

| generation | ROS domain | 主循环墙秒 | 20s/主循环墙秒 | 整卡显存采样峰值 MiB | 原生 LIO / VIO handler 完成数 |
| --- | --- | ---: | ---: | ---: | ---: |
| `baseline_20s_v2`，单机器人相机 | 220 | 29.7035 | 0.6733 | 5,565 | 158 / 158 |
| `multicamera_20s`，六机器人相机 | 221 | 46.0122 | 0.4347 | 6,491 | 157 / 157 |
| `multicamera_map_20s`，六相机加只读 LIO collector | 222 | 46.0767 | 0.4341 | 6,475 | 157 / 157 |

三次均 capture exit 0、`outcome=completed`。基线原生 LIO/VIO 序列覆盖 4.3..20.0 秒，六相机两次均覆盖 4.3..19.9 秒；相邻源时间为 100ms（保留原有 ±1ns 浮点转换差）。基线发布 200 RGB、199 RTX/complete scans；六相机两次各 199 RGB、198 RTX/complete scans，不声称输入数量完全相同。前相机绑定无效帧计数均为 0。VIO handler 完成不代表每张图都保留了视觉 EKF 更新。

单对照中六相机主循环墙时增加约 54.9%，整卡显存采样峰值增加 926 MiB。前相机新增深度、五台额外 RGB/深度、归档压缩和 I/O 都计入开销。这是一次短静止对照，未随机化或重复基线，不能称为实时性能验收。采样约每秒一次，整卡含桌面和既有进程，不是瞬时峰值。原始 `nvidia-smi` 计算上下文峰值也保留在汇总，未以不完整的进程组列表声称完整 CPU/RSS 使用量。

第一次尝试把 observer 输出放在任务目录，既有启动器以“输出必须在项目数据根目录下”拒绝，未启动 Isaac 或 estimator。该失败目录和日志仍保留，随后在符合原限制的新目录重试，没有放宽路径检查。

## 实际图像、时间与投影

每次六相机运行均保存每相机 20 个 RGB/深度对，共 120 对。六宫格视频各 20 帧，播放 1 fps，原源时间在画面中标出。归档间隔最大 1.2 源秒，不能把视频称为逐原生帧录像或精确恒定 1Hz 采样。

第三次运行的只读收集器订阅同代 `/dynanav/rtx_livo/cloud_registered_full` 与 `/dynanav/rtx_livo/aft_mapped_to_init`，按整数纳秒精确配对；检查固定 frame、deskew marker 和点格式。收到 135 个 cloud 与 157 个 pose，接受 135 对，拒绝/缓存逐出均为 0。接收器没有收到全部原生 cloud，不将其称为完整输入存档。0.08m voxel 保留第一实际实测点及其首次源时间，未生成 voxel 中心或使用场景几何。

最终投影的六张图、六个 LIO 位姿均为 **19.3 秒**，相机间偏差和位姿-相机源时间差都是 **0ns**。地图共 79,413 点，统一排除 60 个未来点，六个方向共用剩下的 **79,353** 点。

| 相机 | 视锥点数 | 深度/z-buffer 拒绝 | 深度支持点数 |
| --- | ---: | ---: | ---: |
| 0 | 24,069 | 3,288 | 20,781 |
| 1 | 40,435 | 8,136 | 32,299 |
| 2 | 18,091 | 4,610 | 13,481 |
| 3 | 4,157 | 1,196 | 2,961 |
| 4 | 4,460 | 946 | 3,514 |
| 5 | 18,839 | 2,122 | 16,717 |

总计 89,753 个视角内点实例，对应 67,426 个唯一原始地图点。相邻相机交集依次为 3,758 / 9,809 / 163 / 886 / 1,697 / 6,014 点，实证允许跨视角重叠。上述统计使用 0.15m 轴向深度容差，是诊断筛选；没有以此宣称外参精度、遮挡正确率或定位精度已标定。已实际打开六宫格及叠点图检查方向和场景对应关系。

既有 native-route launcher 的 operator-map 进程未产出独立地图，并在清理 TERM 时记录 `ExternalShutdownException`；这不等同于估计器没有位姿。第三次使用独立只读 collector 补足本代地图证据，未更改该无关地图组件。普通 PCT/SCAN 模式的 `planner_measured_map.npz` 生产链本轮未验证。

## 复现与证据

正常开启（输出使用新目录，需宿主 GPU/ROS 权限）：

```sh
MULTICAMERA_OBSERVATION=1 NAVIGATION_BACKEND=native_route \
  DURATION_SIM_S=20 ROS_DOMAIN_ID=221 \
  bash scripts/run_dynanav_native_exploration.sh "$NEW_RUN"
```

读取此次冻结的同代地图与真实帧：

```sh
RUN=/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/multicamera_validation_20261112/multicamera_map_20s
python3 -B scripts/dynanav_exploration.py multicamera \
  --run-dir "$RUN/capture" \
  --map-npz "$RUN/capture/multicamera/lio_measured_map.npz" \
  --output "$NEW_EVIDENCE" --video
```

只看六图时使用 `--images-only`，不需要地图。原快照工具默认地图路径仍为同代 `interactive/planner_measured_map.npz`，可用 `--map-npz` 显式指定已配对测量地图。

本次证据目录：`/home/hyh001/Documents/Codex/2026-11-12/task/multicamera_validation`，含：

- `validation_summary.json`：运行结果、原生序列计数、显存采样、原始输入索引重叠、交接后变更清单。
- `six_views_lio/six_views.png`、`six_views_lio_overlay.png`、`six_views.mp4`、`snapshot.json`、`visible_input_indices.npz`。
- `*_resources.jsonl`、`*_lifecycle.json`、`*_launcher.log`，以及有限时、顺序运行的 `run_comparison.py` 和只订阅的 `collect_map.py`。
- `initial_worktree.patch` 与 `initial_worktree.json`：保留交接时全部 tracked diff 及 149 个改动文件的校验和。交接后仅四个实现/测试文件增量改变，其余既有文件校验和不变；此报告为新增文件。

CPU：11 项多相机测试、29 项原生相机绑定测试、7 项 RTX source 选择测试全部通过。没有新建代理并行修改。所有本次拥有的仿真、估计器和 collector 均已停止。

## 限制与下一步

验证仅覆盖原地站立与有限场景。六个水平视场的几何重叠不等于任意位置/高度无盲区，仍有机器人、楼梯和物体遮挡；稀疏激光、深度边缘、透明/反射材质、估计位姿误差和容差均可能影响叠点。没有真实运动外参稳定性验收、完整环视覆盖率测量、DSH 实际六图决策轮次、导航收益对照或多路视觉融合。

下一步应先让 DSH 读取当前六图执行有限仿真任务，并在相同预算下比较“额外转向获取观察”的次数和墙时开销，再决定降低额外相机帧率/分辨率或将归档压缩移出采集主循环。移动测试需继续检查原生帧绑定及投影一致性，不以增加视角直接宣称路径信息问题已解决。
