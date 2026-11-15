# DSH 主导的航点候选与连续路线收益

本功能是 PCT+SCAN 之外的**只读建议工具**。DSH 先根据当前 RGB、实测云与任务含义
给出长目标/辅助候选，再看连续 PCT 路线的收益代理量；只有 DSH 明确调用原有
`goto` 才会产生动作。工具不发送、排队、替换目标或发布速度/SCAN参考，不增加 goto 门禁。

入口：

```sh
python3 scripts/dynanav_exploration.py candidates --help
```

## 输入和输出

默认从明确指定的本代 run 读取 `capture/interactive/planner_measured_map.npz` 和
`observation.json`。复用同一个 `MeasuredPctPlanner` 计算每个候选的完整三维路线；
路线失败的候选保留原因和空 score，不隐藏，也不决定是否允许原 goto。

DSH 候选 JSON 为列表，每项包含：

- `id`：唯一名称。
- `goal_xyz_m`：当前 estimator child-origin XYZ；不是地面高度或旧代坐标。
- `role`：`long_goal` 或 `auxiliary`。
- `reason`：当前传感依据/任务理由。

结果给出完整路线、路径长度、潜在新增体素数/体积、单位路径长度收益、输入来源 hash、
frame、整数 source 区间、计算近似和失败原因。所有角色都会报告分数，排序不会擅自改变
DSH 的长目标意图。视觉定位收益明确为 `unavailable`、score=None；RGB存在不等于有
可靠特征轨迹或定位信息。未实现校准熵、互信息、视觉定位质量或全楼覆盖优化。

```sh
python3 scripts/dynanav_exploration.py candidates \
  --run-dir "$RUN" --candidates "$RUN/candidate_review/candidates.json" \
  --lidar-offset -0.020153064280748367 0 0.25510895531624556 \
  --body-support-offset 0.3148910446837544 --pct-start-blind-radius 1.5 \
  --output "$RUN/candidate_review/scored.json"
```

示例的 offset 是本机已核对 Go2 IMU child-origin 到真实 LiDAR origin 的固定安装差，
不是场景坐标。不同机器人/child frame 必须显式提供自己的外参，不能把 LiDAR 基座
安装高度直接加到 IMU-origin 路线上。

`--frontier-json` 可读本代 operator map 的单高度图 JSON，将8邻接边界格聚类，选择仍
处在原 free 边界格上的代表 XY。`--frontier-z` 由 DSH 明确提供后才形成3D辅助候选。
没有 Z 时只返回 XY 建议，不把图层高度/序号伪装成物理楼层。`frontier.actionable` 是
诊断，不用作 candidate/goto 门禁；上游200格等截断会明确保留。不同run路径和真值
标记的frontier不能静默作为本代输入。

## 收益代理量的确切含义

- 观察证据仅为本代保留的实测返回端点体素。PCT 推断的地面支撑不进入观测集合。
- 已验证官方 frozen OS1 sensor-only asset 的32条实际非均匀 elevation/azimuth-offset
  参数，vertical约±22.1°，near约0.3m、far120m。默认8m是计算分析范围，不是硬件量程。
- 沿候选**完整连续路线**按3D弧长采样，包含首尾；预算不足时加粗整个路线采样并报告，
  不静默只算前半段。采样点是数值计算位置，不是发给控制器的新航点。
- 对每个预测传感位姿做3D voxel DDA，遇到实测端点占据格停止。未知区域在模型中透明，
  稀疏墙面可能漏光，whole voxel边界存在近似。默认0.5m体素、32个azimuth列×32束；
  roll/pitch按gravity-aligned假设，future yaw沿路径切向，past yaw尽量用实际LIO四元数。
- 不把deskew云的配对IMU pose冒充旋转扫描每束光的物理原点。因此当前没有经过证明的
  observed-free 3D集合。历史/当前位姿的虚拟扫描属于**模型基线**，不叫实测自由空间。
- 未来全部采样点的可见未知体素作集合并集，然后扣除历史/当前模型基线。重复位置、
  重叠光线和路线重访不会简单累加。单位路程收益为新增体素数除以
  `max(route_length_m, voxel_m)`，不是已经校准的收益/时间或转向代价。

因此数值应称“基于实测遮挡的潜在新增可见量”或“未测体积机会代理”，不应称为
已测信息增益、实际新观测体积或物理可通行证明。改变体素、beam subsampling、分析
距离或地图稀疏程度可能改变排名，DSH还必须用实际RGB/3D和任务判断。

## 提前准备下一段

当前 seq 执行期间可加 `--prepare-next`。工具读取真实PCT准备完成时保存的
`planner_global_route.json`，核对 canonical run路径hash、seq、frame/child和目标，
从当前测得位置投影到剩余global路线，作为独立预算的模型基线。下一段从当前active
目标开始计算；其新增量扣掉当前段预计还会看见的模型区域。历史位姿很多时，不能
挤掉剩余active路线采样，这有独立测试。

输出绑定输入文件hash、active seq和开始锚点，并再次报告完成时绑定是否变化。
准备结果不安装缓存路线。到点后DSH刷新当前观察，再明确提交选择；原生worker仍从
新测量位置重新做PCT/SCAN。它没有取消原最终到点dwell，也没有实现跨长目标无停车接管。

```sh
python3 scripts/dynanav_exploration.py candidates \
  --run-dir "$RUN" --candidates "$RUN/candidate_review/candidates.json" \
  --prepare-next --lidar-offset -0.020153064280748367 0 0.25510895531624556 \
  --body-support-offset 0.3148910446837544 --pct-start-blind-radius 1.5 \
  --output "$RUN/candidate_review/prepared_next.json"
```

源龄、partial collection、pose freshness和绑定诊断只影响建议的解释，不新增运动门禁。
输出文件不能放进action mailbox。工具不会写request、operator-state、heartbeat或队列。

## 冻结真实数据核验

完成run的终态observation通常没有有效当前位置。离线重放必须显式加 `--offline`，
并从同一个run内提供冻结的 `--snapshot` 和 `--observation-json`。输出标记offline且
`source_times_rewritten=false`，不能把它说成当前实时建议。不得拿离线目标坐标去另一代执行。

`newly_measured_voxel_delta(before, after)` 对同代/同frame的前后实测端点做集合差，
可报告实际新增返回体素；这不是free体积，也不是没有执行的其它候选的反事实结果。

## 测试与实测范围

纯CPU覆盖：全路线而非仅终点、稠密/稀疏折线一致性、重叠去重、测量墙遮挡、
近/远距及离散垂直FoV、测量端点不计未知、推断支撑不入观测、同代端点差、历史pose
很多时的active路线独立基线、失败候选、frontier/Z、输入绑定和无控制写入。

真实点云离线与新Isaac两段实验见
[候选预备实测记录](experiments/dynanav-waypoint-utility-live-20261111.md)。实测只是建议
工具与既有执行器的接通，不证明代理排名就是实际探索最优。
