# 航点候选工具：离线与实时预备核验

实现：[DSH 候选与连续路线收益](../dynanav-waypoint-utility.md)。这次验证工具接通、
路线去重和不越权执行，不证明代理排名是实际探索最优。视觉定位收益未实现，明确 unavailable。

## 完成与边界

- 新增只读 `dynanav_exploration.py candidates`：DSH提供长目标/辅助候选，PCT规划完整
  路线，沿路线虚拟雷达可见集合去重，再扣除历史/当前以及剩余active路线的模型基线。
- 已核对官方OS1 sensor-only配置；默认8m分析范围、0.5m体素、32列×32束，是显式
  近似而非完整1024列扫描。模型未知区透明、仅实测端点遮挡；不存在已证明3D free集合。
- PCT推断地面不作观测；视觉定位、校准熵、互信息、覆盖率和收益/时间最优均未实现。
- frontier可聚类为辅助XY；Z必须明确提供。原actionable/源龄/评分不是goto门禁。
- 当前段执行中只准备下一段；不自动排队/发送、不安装缓存、不免除最终到点dwell。
  两段之间仍由DSH刷新观察并明确选择，未实现跨长目标无停车交接。
- 原FAST-LIVO2、RTX、post-VIO、RoboGauge、PCT+SCAN执行链保留，无commit/push。

## 保存真实点云的离线运行

来源为已完成run `pct_scan_live_validated_20261111T182343Z` 内的同代冻结输入：
`live_review/measured_snapshot.npz`、`live_review/observation.json`。显式 `--offline`，
不改source时间，不把离线坐标用于其它代。

3个候选均由真实PCT完整规划成功。墙钟约0.83秒：

- 前方长目标：路线4.726m，1385个潜在新增体素，293.0个/米。
- 左侧辅助：路线3.608m，1310个，363.1个/米。
- 右侧辅助：路线3.576m，1185个，331.3个/米。

这是基于所选模型的路线机会/成本对比，不是三个候选实际执行后的信息收益比较。
产物在该run的 `candidate_review/candidates_offline.json`、`scored_offline.json`。

## 新代实时实验

run：`waypoint_prepare_live_20261111T183934Z`，外部 experiments目录，ROS domain224，
240模拟秒有限预算、命令上限1m/s。目标均来自本代actual front RGB和完整/局部
XY/XZ/YZ/斜视3D云及post-VIO；未用USD、真值地图、旧代坐标或旧路线选点。

1. DSH先提交本代前方3m目标seq1。
2. seq1仍在cruise执行时，67.615–68.355模拟秒运行 `--prepare-next`。
   约0.927墙钟秒完成3候选，前后active seq均为1，request/operator-state hash未变。
   “执行期间”包括朝向纠正，不声称此时一直有正前向速度。
3. 扣除剩余当前段模型可见区后，预备结果：
   - 左侧辅助：路线2.263m，388个潜在新增体素，171.4个/米。
   - 前方长目标：路线2.113m，289个，136.7个/米。
   - 右侧辅助：路线2.280m，310个，136.0个/米。
4. seq1到点后，DSH重新实际查看本代RGB和3D云，再明确选择左侧辅助目标，调用官方
   `goto`提交seq2。没有自动dispatch或复用缓存路线。
5. 预备锚点是seq1目标 `[2.9987,-0.3382,0.0081]`；seq2原生SCAN实际起点是新测量
   `[2.7925,-0.5592,0.0178]`，相差0.3024m，证实实际规划从新pose开始。

## 实测结果

- 2 accepted /2 reached /0 failed。
- 两个最终XY误差分别0.29587、0.27367m；z误差0.01511、0.01214m，保留原到点范围。
- 最终source停止dwell时刻80.5、166.7模拟秒；共消费57条真实SCAN运动参考。
- 未记录body-contact步骤，`scan_connected=true`来自真实消费证据。
- stop seq3已确认，`navigation_terminal_and_parked`，capture exit0；全部owned进程
  和两个SCAN子进程均确认退出。
- 最后约1秒最大水平速度0.000109m/s、角速度0.001066rad/s。
- seq2前后同代0.5m端点集合：2967→3236，集合差新增275格。输入source末尾分别
  108.2、178.0模拟秒。这是保留代表点的新增返回格，受采样/重访影响，不是free体积，
  不可用275/388算“预测准确率”，也没有未执行其它候选的实际反事实结果。
- 第一段到第二段间有人工/DSH观察选择的idle时间。本次预计算提前完成，但不声称已
  消除交接等待或改善总探索用时。

产物位于本run `candidate_review/`：

- `dsh_intent.json`、`live_candidates.json`：明确目标和来源。
- `prepared_next.json`、`prepare_while_moving_audit.json`：评分/绑定与无动作写入证据。
- `before_first/`、`before_second/`：冻结云、实际front图与source、完整/局部3D图及显示来源。
- `second_decision.json`：刷新后DSH明确选择。
- `after_second_measured_snapshot.npz`、`actual_new_return_voxels.json`：实际端点差。
- `validation_summary.json`：结构化到点、停车、清理及实际重新规划起点。

原始reference/receipt/control/policy/native log与source snapshot仍在capture目录；
只在结束后读取truth_scoring_stream作物理评分。预测结果保留输入hash/代码hash和源区间，
不把之后更新的live snapshot冒充当时输入。

## 测试与打包

- 新核心12项CPU、候选工具9项CPU通过，含1000历史pose不会挤掉active路线基线的测试。
- 完整unit：1123 tests，3 skipped；regression：108 tests，8 skipped。
- `git diff --check`、CLI dispatch/help、语法检查通过。
- 显式配置package data，隔离目录构建sdist确认包含OS1 calibration JSON。
  本机缺pip/wheel，wheel构建未运行，不把sdist检查说成wheel验收。

代理分数的模型局限与原生SCAN配对/时间式碰撞检查局限继续保留，详见
[PCT+SCAN核验](dynanav-pct-scan-live-20261111.md)。本次不声称完整自主探索或视觉定位优化。
