# DSH 模型在环低频控制（RTX LiDAR + FAST-LIVO2 post-VIO）：run4 受控结果与更正

日期：2026-11-11。本轮只做**准确收尾**：不新开仿真、不改源码；以下数字以监督对
原始数据的独立核算为准（`.dsh-subagents/rtx-pre-tick-ros-pump-comparison-20261111.json`、
`.dsh-subagents/rtx-live-supervisor-audit-20261111.json`）。历史 run 事实不覆盖。

## 1. 重要更正：外层 ros.drain(2) 是冗余的，已撤回

`capture/control.py` 的 `ControlLoop.tick` 在 `i % 4 == 0` 分支里**本来就有** 8 次
`ros.spin_once(0.0)`（约 216–218 行），随后才读取 `latest_odom`/policy 状态。因此监督
在 `capture_go2_rgbd.py` 外层新增的、仅 interactive 生效的 `ros.drain(2)` **不是修复
遗漏的前置 pump**，是冗余的；它和 2 项对应测试已精确撤回，用户原有改动保留。本轮
**没有**再把它加回，也没有改动任何 freshness/stall/TTL/unknown/姿态门。

结论：run4 航点成功**不能归因于**该 pump。run4 的 capture source-age 分布
（p50 .215 / p95 .295 / p99 .315 s，≤.25 比例 65.66%）与 baseline live2
（.215/.295/.315，66.51%）实质相同；同 stamp 的 capture 回调相对 collector 到达差
中位 3.45 ms（live2 3.82 ms）、p95 38.95 ms（live2 41.41 ms）。单次成功/停滞差异
不足以证明因果收益。

## 2. 模型在环的事实边界

- **run4 是模型在环**：DSH 模型在真实运行期间分别读取了当时的
  `observation.json`、`perception.json`、operator map level 快照与真实 RGB 截图
  （`rgb_000014.png`，source 28.3 s；`rgb_000023.png`，source 46.4 s），据此决定并
  提交了 1 次 0.08 m/s×1.5 s 前向 maneuver 和 1 个 0.32 m 前向航点
  （goal `[0.338,-0.02829,0.00088]`，由当时的 post-VIO pose/yaw 计算），随后读取
  收据/位姿反馈，再提交 stop。收据：`maneuver_complete`(seq1)、
  `waypoint_reached`(seq2)、`stop_accepted`(seq3)。
- **runG 不是模型在环**：runG 的 `control_fwd.py` 由 DSH 生成并启动，但动作是预写的
  自动链（等就绪→自动 maneuver→按当时 obs 自动 goto→自动 stop）；运行期间模型没有
  逐次读图/读状态决策。两类结果不能混为一谈。

## 3. run4 可核实测（domain 160，150 sim s 条件，heading 3π/2 如实记录）

- run：`experiments/rtx_live_dsh4_20261110T202726Z`；capture `outcome=completed`、
  `end_reason=navigation_terminal_and_parked`、`terminal_request.arrived=true`。
- 动作前站稳基线 **37.695 s → 110.015 s 停车**（不是 spawn 起算）：
  native GT（仅离线评分）平面净 **0.126475 m**、3D **0.126545 m**；
  post-VIO 平面 **0.129302 m**、3D **0.129314 m**；
  **249** 个非零 command tick、实际通令 **4.98 sim s**；native 机身接触 0。
- 云/位姿：native full 云 **926/926** 与 LIO pose 整数 stamp 精确匹配，unmatched 0。
- 地图：**904 接受 / 3 拒绝**；局部观察覆盖 **188.53 m²**，unknown 邻域下界
  **38.9125 m²**，level floor −0.35196 m。仅局部地图，不是楼层探索完成。
- hold 计数（原门，未放宽）：stale_sim_odometry 382、perception_stale 110、not_ready 1。

## 4. 对照（同类指标，均来自监督核算）

| run | pump | capture age p50/p95/p99 (s) | ≤0.25 比例 | 航点 | native 平面/3D (m) | post-VIO 平面/3D (m) |
| --- | --- | --- | --- | --- | --- | --- |
| live2 | 原有内层 8 | .215/.295/.315 | 66.51% | stall (`route_low_progress_stall`) | 0.01525/0.01552 | 0.01791/0.01822 |
| run4 | 内层 8 + 外层 2（已撤回） | .215/.295/.315 | 65.66% | reached | 0.12648/0.12655 | 0.12930/0.12931 |

结论：年龄分布无实质改善；run4 航点在原门下**持续执行并到达**，但成功不能归因于
冗余 pump；live1（无外层 pump、但当时存在 bridge 生命周期 bug）也曾到达过航点。

## 5. 仍未解决 / 未证明

- freshness 导致的**间歇 hold**仍在：约 34% 的 50 Hz tick 因原 0.25 s pose-age 门
  hold；实际通令 4.98 s 分散在整段航点执行期。未放宽任何门。
- RTX 近场地面盲区（本姿态 `[0,1.675] m`）仍靠模型读取真实 RGB 补充判断，不是
  实测地面支持；机身正下方残余区域两传感器都看不到。
- 右侧/右后世界固定无回波开口在现有朝向下仍未知，未做侧向运动。
- harness map 预算 `DURATION+180` 只修这一批 run 的桥寿命，**不是**任意低 RTF /
  超长运行的长期生命周期保证。
- 未做：1 m/s 巡航、多房间/楼层探索、长期定位精度验收。GT 仅隔离离线评分。
