# 非实时试行的 perception 消费时相只读审计

范围：`rtx_basement_nonrealtime_speed05_retry_20261110T230043Z`，domain184，capture 已正常退出；仅重读原记录并使用其冻结源码做 CPU gate 回放。未启动 GPU/ROS，未修改生产源码、stamp、arrival、阈值或控制请求。

## 精确连续 hold 窗口

- 最后 tracking：119.895sim / monotonic 1625582.867659311。
- 首连续 hold：119.915sim / 1625583.055929675。
- terminal：121.815sim / 1625588.059043351，连续 **5.003113676wall秒**，96个控制tick。
- 92次 `perception_stale` 首先失败于外层 `now_wall - payload.arrival_wall_s > .5`，不是首先失败于sim门。payload arrival是depth/RTX/LIO最早原arrival，本窗全部为depth。
- 另4次是 `perception_clearance_unknown`，对应120.3与120.9source：年龄尚可，但实际联合floor为unknown。不能把它们归为时序hold或把postcompute接受当作几何可走。
- 原先“最后5.5秒16/18写出fresh”是较宽窗口，包含最后tracking附近的输出；不等于连续5秒96tick全stale。精确连续窗为16次输出，14次独立postpub年龄全部fresh。

## 14个 fresh 输出为何没有解除 hold

以每个原payload的post_compute_wall_s为写出下界、独立读取完成wall为写出上界，逐一关联下一真实控制tick：

- **8个输出**到下一tick前已经过期。写出到下一tick约170.6–188.6ms，其中记录明确的冻结锁步等待重叠约150.6–154.6ms。不能把剩余时间具体归于JSON日志、物理步或额外render，因为没有这些分段wall仪表。
- **4个输出**在下一tick仍可通过原源年龄和实际几何门，但控制器按原20Hz文件poll缓存继续用旧payload。上次poll到此tick只有15.00–16.54ms，小于原50ms缓存间隔。这是冻结源码与原时序CPU回放验证，不是凭“相位问题”猜测。
- **2个输出**年龄可用但floor/clearance本身unknown，产生4次hold，正确拒绝。

每个输出的原source ns、arrival、写出wall上下界、下一tick wall与使用的source、各源年龄、已知锁步重叠均在 `perception_consumption_timeline.json`。

## 最小实测缓存复现

使用两个未改payload：`live_perception_samples/001007.json`（source120.1s）和`001008.json`（source120.2s）。

1. 在原tick wall1625584.190300244读取旧payload。
2. 新payload已于下一tick之前真实写出并被独立读到。
3. 下一tick为120.395sim / wall1625584.206841884，只距上次poll16.54164ms。
4. 原50ms缓存返回旧source120.1，其depth wall age=.75863884s，因此 `perception_stale`。
5. 新source120.2在同一原tick的depth age=.49239498s，RTX=.16783224s、LIO=.17086779s，sim age=.195s，实际floor支持；离线强制读取新文件可通过**原**gate，不修改任何阈值。
6. 再下一原tick到120.415sim时，新source本身也已过期（depth wall=.68239682），因此只在下一个poll读到它已太晚。

`perception_consumption_minimal_repro.json`保留两个文件路径和精确时刻。运行：

```sh
python3 .dsh-subagents/basement_runtime_audit/replay_perception_consumption.py \
  '/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_basement_nonrealtime_speed05_retry_20261110T230043Z'
```

脚本仅导入该run的source_snapshot，在临时本地mailbox调用 `_perception_gate`，不调用tracker.tick/client/policy/ROS。完整96tick按写出上下界分别回放，**两种正常模式的source和92 stale/4 unknown都逐条完全复现**。反事实每tick读新文件，保守上界有4个pass，乐观下界有6个；两种边界差异反映写出瞬间未单独记录，不能宣称精确6个。反事实只是诊断，未进入生产，不能据此宣称机器人会成功到达或异步链已经修好。

## lockstep ready 是否误导下一tick

精确连续hold窗19次等待全部timeout，last hint16次cloud_wall_expired、3次cloud_sim_expired；没有hint_ready后马上错过的案例。

全run10次hint_ready，对应下一**真实正常tick**（按simulation time定位，而非跳到下一条可wall-join的记录）：2次仍处初始化not_ready，8次tracking。可精确wall-join的5次ready→tick为1.61–1.91ms；另外3次tracking缺可独立重建wall字段，只知道状态通过，不捏造wall延迟。初始化2次没有完整odom/perception join，同样不捏造时刻。

代码顺序是 `(i+1)%20==0` 等待结束后，下一正常 `i%4==0` 控制tick在相同simulation time执行，然后才下一physics步。不能声称ready后必然还跨四个physics步。其他非sensor边界的控制间隔仍为四个正常physics步。

## 可知边界

- tracker tick wall由**原payload arrival + 原日志perception_wall_age**重建，未修改源时间；odom callback wall可由此减原odom wall age交叉核对，同stamp推导spread为0。
- 没有直接文件I/O开始/结束、validation结束wall或poll状态日志。具体读取时刻由冻结代码的50ms条件和全部连续tick重放确定，不能冒称直接记录。
- 第一连续hold tick距上一tick188.27ms，必然满足原50ms poll条件；此后每个tick都进入perception gate。因此缓存状态可确定，含“重读同一source”的poll，而不只根据source变化猜测。
- 原mtime为epoch；保留原值但不随意与monotonic相减。写出上下界直接使用同一monotonic域的producer postcompute与独立读取证据。
- 原日志没有足够字段将剩余非等待时间拆成额外render、JSON日志、文件系统flush或每个physics耗时；这里只报告已测锁步重叠下界。
- CPU反事实是在原tick传入now下执行原gate；不证明新增实时I/O开销、后续时序和轨迹。不能把此局部缓存现象替代完整扫描watermark/端到端时效验证。

产物：`perception_consumption_timeline.json`、`perception_consumption_cpu_replay.json`、`perception_consumption_minimal_repro.json`；脚本位于本repo的basement_runtime_audit目录。原0.5试行路线与收据仍以主探索报告为准。
