# 两层大厅探索与 SCAN 接入

## 版本和范围

05 基线为 `10de6cfa3e150d1e1580ee98012005572991bbbe`，首轮已运行适配为 `94847d4fff3eb998e84c9fed4645ca945fa66efa`。首轮只完成初始平层局部航点，未完成两层；精剪视频明确标注此范围。

当前修复等待消费 ACK 再发参考路径，以匹配的 route definition 确认提交；输入拒绝、真正放弃目标、交接超时均返回对应航点失败。可恢复急停仍为 hold。post-VIO 未提供实测 twist，SCAN 改用既有半秒位姿窗口的地图系速度向量，原消息不变。

vendor 逻辑仅通过 `third_party/scan/patches/reference_feedback.patch` 应用到新运行目录的源码副本；旧缓存和 vendor 原文件不变。定位仍为单前视、RTX LiDAR、IMU，bias 与 gravity 开启；后端修正尚未进入控制。探索不设总时限；10 秒交接超时不限制执行路线。

## 加减速与参数

使用平地诊断代次已有的 50 Hz 实际命令和 200 Hz 物理速度；真值仅作事后统计，目标由本代实测地图、RGB 和 post-VIO 选择。命令从零升至 1 m/s，后续为原生接近目标时逐步降速，不是紧急停车阶跃。

- 0.2 秒窗口：正加速度 P95 为 2.883 m/s²，峰值 4.273 m/s²，10–90% 上升约 0.296 秒。居中滤波有提前效应，不解释为控制延迟。
- 渐降段：减速度幅值 P95 为 0.478 m/s²，峰值 0.624 m/s²；从开始降命令至停稳约 4.775 秒、1.677 米。
- 片段机身接触计数 0；不证明同等急制动或楼梯能力。
- 按用户要求，两处 `manager.max_acc`、`optimization.max_acc` 从 0.5 同步改为 1.5 m/s²；`optimization.acc_tolerance` 保持 1.0，最终三维范数门为 2.5 m/s²。速度上限仍为 1 m/s。
- 离线原生复现显示，增大参数也缩短初始轨迹时间，个别候选仍超限；不能宣称参数消除了所有失败，不继续扩大最终门。

## 验证

独立 LF 克隆：1157 项单测保留原 5 项失败，81 项回归通过，原有 skip 不变；新增 24 项全通过。真实暂存 C++ 输入校验抽取编译执行 12 项通过。已逐行审查 Python 和 patch。

完整 native 构建、ROS 协议及后续仿真结果以数据卷收据为准，不由 CPU 检查推断。两层覆盖仍需实际到达、楼层证据和未访问区域记录。

日志、视频、点云、原始 patch 和测试输出位于 `$PROJECT_DATA_ROOT/embodied-navigation/runs/20261115-two-floor-demo/`，不入仓库。未推送。

## 后续指定参数对照

用户最新指定两处 `max_acc` 设为 2.0 m/s²。保留 `acc_tolerance=1.0`、原发布前加速度硬拒绝、速度和碰撞检查，最终加速度阈值为 3.0 m/s²。此配置覆盖下次仿真；上一 1.5 参数代次首航点明确失败（末次 2.592 > 2.500），失败反馈正确，原生停车并退出 0。放弃的加速度诊断补丁不进入本版本；对应部分构建已取消，不能使用。

## 收据热路径修复：等待新代仿真

完整 SCAN provenance 现在按实际消费的 revision 只追加一次到 `planner_provenance.jsonl`；事件和逐 tick 收据保留轻量字段及可回读的偏移、长度和 SHA256。`result.json` 的 0.25 秒非强制保存间隔从原子替换完成时计算，强制终态保存不变。本次不改规划或控制决策，也不改 `manager.max_acc=2.0`、`optimization.max_acc=2.0`、`optimization.acc_tolerance=1.0`。

独立 LF 克隆验收：1160 项单测保留原 5 项失败（1 failure、4 errors、3 skipped），81 项回归通过（9 skipped），无新增失败；新增 3 项验收通过。两份实现和三份测试已与验收候选逐字节比对，并逐行审查差异。CPU 验收只证明记录可追溯、节流语义及原控制输出保持；实际 RTF 和长运行开销尚待下一仿真代次，不把取消前后的整体 A/B 差异归因为单一保存路径。

## 用户指定的终检加速度容差

按最新请求，`optimization.acc_tolerance` 从 1.0 改为 2.0 m/s²，`manager.max_acc` 与 `optimization.max_acc` 均保持 2.0 m/s²。真实终检来自 `third_party/scan/plan_manage/src/planner_manager.cpp` 的 `checkDynamicFeasibility()`：`acc_limit = pp_.max_acc_ + pp_.acc_tolerance_`，比较样条加速度三维范数，因此新阈值为 4.0 m/s²；此参数由同文件的 `optimization.acc_tolerance` 读取项加载。

本次提交只调整该容差及报告，不改终检逻辑、速度或碰撞检查，也不改轨迹时间分配。构造参数验证与 23 项针对性测试已通过；联合独立全量验收尚待与时间分配补丁一并完成，实际运动仍以新代仿真为准。
