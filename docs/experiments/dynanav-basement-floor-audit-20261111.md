# 地下室近场传感器审计与 floor 拟合修复

2026-11-10 UTC，CPU-only。只读真实保存的 RTX cloud 与同 stamp LIO pose；未启动 Isaac/ROS/GPU，未用原 USD/GT 补导航，未改出生点或任何近场门。

## 确证与改动

新 run `rtx_basement_dsh_20261110T211225Z` 的 5.2s 实测点云中，旧 `fit_floor_plane` 将每个扇区/径向格的 z 低分位配到虚构的格中心 xy，叠加楼梯污染，得 plane `[-0.02076,-0.01525,-0.26748]`、样本残差中位数 56.5mm。后/左/右独立实点平面都在截距 -0.343m、倾角 6° 附近。

修复仅涉及 `rtx_deskewed_perception.py`、其单测与新真实 fixture。公开函数签名不变，使用实际 return 的成对 xyz 做确定性鲁棒候选与等格权重精炼。新 plane `[-0.1032379,-0.0224888,-0.3435087]`，6039 实点、37 方位/径向格、14 方位扇区跨径向支撑，中位实点残差 0.446mm。两 run 的全部 16 个保存同 stamp 样本都恢复同类平面。

### 保守门与范围

- 沿用 search radius 4m、below-base 0.08–1.40m、三个原径向带、每格至少 30 点、至少 8 格，以及原 slope≤0.30、residual≤0.10m 门。候选实际一致带更严，取 min(0.03m, 原 residual 上限)。
- 候选种子取各格实际低/中/高分位附近的完整 xyz，避免低分位隐藏共格的上层地面。固定随机种子 256 次三点候选；没有足够证据就不输出平面。
- 每个支撑格须至少 30 个一致实点且占该格候选点至少一半。可用 floor 必须至少一半方位扇区（默认 12/24）各自跨≥2径向带，且实点径向展开≥0.35m；此资格在 ambiguity 判断之前检验，单环点数/扇区数不能冒充周围支撑。较窄候选保留诊断，原始点仍参与障碍/unknown 分类。
- 如果两个或更多不同平面均满足上述广度资格，即使一层点更多也返回 `floor_ambiguous_surfaces`，不按最高、最低或最大 consensus 选站立层。对较窄候选先剔除与唯一广阔面共用的30mm一致点；若剩余独立点仍满足≥8多数格、≥4方位跨≥2径向带/spread≥0.35m，同样拒绝为独立第二层，保留较小真实上层/更大低层的保守性。窄平台单独不能建立周围 floor。
- 这是给定采样/一致性分辨率下的保守几何分类，不保证辨出所有薄层/遮挡/稀疏地形，不等于连续可通行性，更不能证明机器人脚下未观测地面。原 0.35m 起点、四 bin、各 bin floor 点数、azimuth、freshness、unknown 门均未改变。

## 近场仍拒绝

修后四方向首 floor bin 全 0，front 的 3m 内没有与该 floor 一致的点，最近 front 障碍 0.9121m。左/后/右最近 floor 为 1.2815/1.1578/1.1995m。没有任何方向获准平移。

现有 RTX 分支只产 RGB/IMU/LiDAR，`include_depth=False`，没有可冒称已存在的真实 depth。现有前视相机 mount `[0.3,0,0.13]`，yaw=0、512²、fx=fy=292.57px。对修后实测平面求全像素理想无遮挡交点，最小 floor 半径 **0.95617m**。此前基于错误旧 plane 的 0.77m 估计撤回。此值是视锥下界，**不是 depth 观测或支撑证据**，开启现有前视 depth 仍不能补到 0.35m。

现有 LiDAR 与 legacy depth 的 self box 都是 base x±0.5、y±0.3、z[-0.45,0.3]。当前真实 floor 高度下，front/rear ±45° 扇区内 r≤0.35m 的点全部落入该 box，哪怕新下视传感器看见也会被误删。当前两 run 实际所有 scan 的 self_returns=0，所以当前先受 FOV 限制；粗 self box 是新下视链的独立后续阻碍。不得简单关闭过滤/挪走廊起点，应为新增下视传感器独立验证自体遮挡/过滤契约。

## 可复现证据

独立目录：`$PROJECT_DATA_ROOT/embodied-navigation/experiments/rtx_basement_sensor_audit_20261110T2120Z/`，包含原 fitter 原样副本、`audit_sensor_support.py`、16帧前后完整 `audit_sensor_support.json`、全量测试日志与说明。原始 run 数据未重写。

fixture：`tests/fixtures/dynanav_rtx_floor_20261110_5200000000.npz`，完整保留新 run `collector/cloud_full_008.npz` 的原顺序 x/y/z/curvature、5.2s 同 stage LIO pose、IMU 安装标定；无过滤/抽样/GT/合成点，来源路径/sha/抽取规则内嵌 provenance_json。unit 不依赖外部数据盘。

## 本 scope 冻结验证

- focused：32 passed（原25+新增7），包括真实 fixture、actual-xyz、双层、同格双层、窄平台、楼梯、原 bound 回归。
- py_compile、git diff --check 通过。
- 并发工作树全 unit/regression：966 passed、8 skipped、1 failed；唯一失败为另一个 scope 新增 `capture/nearfield.py` 尚未加入 source snapshot。此时**不能声称全树通过**，父任务在全部 worker 冻结后统一复测。
- 本机真实约31k点的 floor fitter 约36ms/帧；下一次真实 run 仍须验证 freshness 预算，CPU 修复不构成运行验收。
- 无 commit/push、无原资产变更。

冻结 SHA256：
- `src/embodied_navigation/simulation/dynanav/rtx_deskewed_perception.py`: `8b2d3bb7f302e5dae401efcf2b31cb0e287d10f9548965a959a77d17431839d7`
- `tests/unit/test_dynanav_rtx_deskewed_perception.py`: `9f405f1e2945dd2cce4656d070e946e97712b77b786807de09f53409cbb73ef6`
- `tests/fixtures/dynanav_rtx_floor_20261110_5200000000.npz`: `2357381d4399f7a19944fa5cabe273d4f8caa81c6ab38973216f7867183ad4df`

原 fixture cloud 文件 SHA256：`2b27c2120b54306f9f290ff129f0c21431d81fcde0f3ca6271f01d470ab3cb1b`。

## 第三代 heading0 live 反例后的修复（2026-11-10 21:44 UTC）

`rtx_nearfield_heading0_diag_20261110T213641Z` 是明确初始化 yaw=0 的新 generation，只做零动作诊断，不是 DSH 原地转身，也不跨 run 合图。原保守实现把仅4个跨径向扇区的候选先加入 ambiguity，再检查12个总方位扇区，导致正确的广阔floor被墙/踏面斜交假设阻塞。

- 全部 live 候选记录：66 native、60 joint，按“至少12扇区各自真正跨径向”资格重新分类，126帧都恰好一个合格面。仅按12个总扇区不足：23 native和57 joint帧仍有多个这样的伪面。
- 对保存8个完整 native cloud/同stamp LIO pose逐点核查，额外候选仅8–11总扇区、4–7跨径向扇区；56%–87%的inliers其实和真实floor共享。剔除共享点后，独立多数支撑仅第一径向带，跨径向方位数为0。这是墙/窄踏面与地板斜交的拟合，不是独立宽广第二层。
- 新实现将上述真正广度资格前置；窄候选记入 `rejected_candidate_surfaces`。真正独立的较小第二层仍按独立实点门拒绝，既有120°upper/240°lower与同格双层测试不改断言继续通过。
- 8个保存真实native cloud重新完整拟合全部恢复14个跨径向扇区的唯一floor，c≈−0.3435m，native近场门仍拒绝。126帧候选几何资格审计不冒称126帧全部点云重放；joint实际输入重放由融合worker单独报告。
- 新增 heading0 fixture：`tests/fixtures/dynanav_rtx_floor_heading0_20261110_5200000000.npz`，完整原始点+同stamp pose/来源hash，显式标注generation reset。来源该run的 `collector/cloud_full_007.npz`。
- 本次 focused **35 passed**（原32断言不变+新增3：宽广180°双层、窄踏面仍为障碍、heading0真实反例），py_compile/diff-check通过。父任务负责所有worker最终冻结后的全树测试，本次未宣称全树通过。

第三代反例原始候选、独立实点分析、修前源码副本和修后8帧证据保存在同一独立audit目录的 `heading0_live_candidate_widths.json`、`heading0_independent_candidate_support.json`、`rtx_deskewed_perception_before_heading0.py`、`heading0_after_repair.json`。上文首版冻结SHA仅指首版；本次最终冻结SHA如下：
- `src/embodied_navigation/simulation/dynanav/rtx_deskewed_perception.py`: `599defc48924ae11bddb624fbe53be04dd0b0e5256ab142757ab2aa634ce243a`
- `tests/unit/test_dynanav_rtx_deskewed_perception.py`: `d0ff48793adf33fc2c8cdd81e6a05d0d7b36867da30c249b58ce5cfe89f7e083`
- `tests/fixtures/dynanav_rtx_floor_heading0_20261110_5200000000.npz`: `e907460e22120ee889d80a159dee34c9870187fcd43d9acec04ec8b68930f1e3`

## 等价 CPU 性能优化与最终冻结

仅对第三代保存的8个native＋同stamp真实depth联合输入做CPU profile与隔离A/B，不启动GPU。cProfile下8 joint总0.744s，其中supporting_cells 1175次/0.179s、lstsq584次/0.113s。原算法重复构造相同支持集与重复解相同最小二乘系统是主要可修开销之一。

采用的四项等价优化：
1. 先使用原有径向布尔mask缩小待分扇区数组，保持原点序与完全相同的方位比较；没有按点预算抽样。
2. 同一cell的5/50/95分位一次调用计算，选点规则不变。
3. 只有有序完整支持点数组与每格权重构成完全相同时才停止重复lstsq；近似相同不提前收敛。
4. 当前fit内用plane完整float bytes＋独立参考plane bytes作key缓存相同验证结果，严格最多8条LRU；淘汰只导致重算，不跨帧复用，不用上一帧plane。缓存只保存已经完整验证的同输入结果。

保持256候选、原候选去重数值运算、所有coverage/残差/角度/近场/第二层检查不变。没有采用候选GEMM向量化或减少候选数。

5轮交错A/B中位：native **30.35→24.52ms**；joint **76.23→55.32ms**，节省 **20.91ms（约27.4%）**。同一真实joint样本tracemalloc峰值4.66→7.87MB，额外约3.21MB；固定8条上限，内存不会随候选条数无限增长。这是CPU离线时间，不是live freshness验收。

最终生产重算全部16组native/joint完整payload，排序JSON SHA256和先前冻结 `599defc…` **逐项完全一致**，包括plane所有float、方向点数/原因/clearance、诊断。原35 focused不改断言全过，另加缓存上限/逐帧隔离及关闭cache和收敛、分开算分位后的完整payload等价测试，共 **37 passed**。py_compile、diff-check通过；父任务负责全树最终重测。

脚本、cProfile、各隔离变体、逐帧SHA、内存数据保存在同一audit目录 `performance_20261110T2200Z/`，关键为 `lru8_final.json` 和 `production_equivalence.json`。源码此前快照与对照记录未覆盖。

最终生产SHA256：`bca56c7394cc7294ab1a1c008acd7f86255087d97019ab94b862b5d373df01c1`。
最终单测SHA256：`517ecd656b9125aea976fe5c80ef02c90248c1d1a8e95d5eaf7137d671c2710d`。
