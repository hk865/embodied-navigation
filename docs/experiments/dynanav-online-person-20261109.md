# DynaNav：速度四档与在线RGB行人实验

记录日期：2026-11-09。本页接续提交`43cba33`后的实验。已完成单人authored crossing、0.30m/s命令档的完整三阶段/双楼梯路线，但感知质量未通过；用户已要求暂停，任务进程已释放，不再启动后续运行。保留[此前视觉闭环报告](dynanav-20261109-continuation.md)和[历史原话](../research/dynamic-semantic-mapping-agreement.md)的原始范围。每次运行的source snapshot、模型/二进制SHA和收据决定实际身份，后续源码不追溯替换as-run事实。

| 项目 | 已测状态 | 验证范围 |
| --- | --- | --- |
| 0.15/0.3/0.6/1.0 m/s命令档 | 四档均完成三阶段、两次停车转向及终点物理停车 | 无行人、legacy oracle-cloud输入的视觉闭环；不是持续全程实测1m/s |
| 持久RGB worker /tile | 真实本地IPC和一次模型加载已测；tile2的4个选定漏检样本没有新增TP、增加62 sampled FP | 小样本离线试验，在线仍用tile1 |
| person几何ID helper | 旧缓存69 exact/5跳过的离线重放已测 | 上游旧LIO仍用oracle-static cloud，不是在线通过或IDF1 |
| 在线行人45s片段 | 真实worker与新点云接入已独立核实；person点仍漏过36.26%，历史geometry准入344个person点 | 未到达完整路线，主要漏过来自未推理帧；最终ledger无person不抹去历史污染 |
| 同配置完整行人路线 | 0.30m/s命令档完成三阶段/两次转向、arrived与停车，五进程退出0 | 单人authored crossing；person漏过39.65%，感知质量未通过，不是1m/s动态导航 |

## 四档完整视觉闭环速度对照

[最终final-review.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-speed-100-20261109T063742Z/speed-audit/final-review.json)和[可读报告](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-speed-100-20261109T063742Z/speed-audit/final-review.txt)记录已终止并独立核对的四档。三阶段/两次停车转向、arrived及物理停车均通过，各轮四进程退出0。18份冻结源码、策略/模型、测量参考、core与相机标定一致；目标及原有`max(0.65,target+0.5)`物理速度门随档位变化，没有增加楼梯限速。公平性依据实际[protocol-consistency.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-speed-100-20261109T063742Z/speed-audit/protocol-consistency.json)，不拿当前新增learned源码替代旧快照。

| 目标命令m/s | native总时长s | 全程XY均速m/s | cruise时长s | cruise XY均值/p95/max m/s | 末端world/base XY误差m |
| --- | --- | --- | --- | --- | --- |
| 0.15 | 254.40 | 0.100692 | 191.98 | 0.125682 /0.183444 /0.293267 | 0.132945 |
| 0.30 | 151.30 | 0.163873 | 82.44 | 0.273475 /0.354190 /0.455705 | 0.203497 |
| 0.60 | 103.60 | 0.238619 | 37.88 | 0.548642 /0.641647 /0.726991 | 0.205121 |
| 1.00 | 92.60 | 0.266996 | 20.70 | 0.902422 /1.010626 /1.114652 | 0.192291 |

只读速度汇总入口为[audit_dynanav_speed_ladder.py](../../scripts/audit_dynanav_speed_ladder.py)，接受多个已结束`--run`和新的`--output`；不启动SDK或修改速度门。

cruise按`actual_command[0]>=0.95×目标`选实际命令保持区间，再用native post-step速度按物理时长加权，p95同样按时长计算。全程均速包含初始化、转向和停车。**1.0档是命令上限完整路线成功**：20.70s cruise中，实测XY≥1.0的累计时间只有1.57s（7.5845%），不能表述为持续或全程实测1.0m/s。

1.0档[原门禁独立审计](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-speed-100-20261109T063742Z/speed-audit/gate-audit.json)核对4,630对control/request/actual command逐时间一致；63条stale hold、累计1.26s均零命令。五个stop/turn dwell各有6个distinct源测量、跨0.5s；87.515s arrived，87.615–92.615s连续物理停车5s。851条post-VIO pose/status与851原生状态按唯一原header配对，位置差0，4,410个controller pose输入逐双精度一致，writer/bridge无已记录drop/error。

该档active窗口3D峰值1.137509m/s，小于原物理guard1.5；启动豁免期的初始下落峰1.337630另记。关节读回峰30.000221rad/s略高于名义30，但仍在既有30.001比较门内，没有钳制原始状态。末端误差是GT仅作离线评分的world/base误差，不能直接与另一estimated-map/IMU定义的0.20m capture门限比较。高层路线用实测post-VIO；底层SDK本体、物理guards和当时oracle诊断仍保留。四档没有验证行人、learned过滤或任意spawn定位。

## RGB-only持久worker与tile反例

[run_dynanav_person_worker.py](../../scripts/run_dynanav_person_worker.py)在独立进程加载本地Mask R-CNN一次，反复处理当前RGB请求；[person_worker.py](../../src/embodied_navigation/perception/person_worker.py)校验source/frame/RGB SHA、request/session/model身份，记录新推理事件和wall/GPU延迟。不自动下载模型，不给模型传depth、pose、renderer labels或GT。工作通道、权重和图像仅保留新数据卷。

当前worker实验候选score0.1、mask0.5、高置信score0.5；这些是本轮operating point，不是过去已约定的通用阈值。低候选的二维时序支持不等于三维几何/身份验证。未调度帧不产生新语义；失败不复用旧mask伪造fresh，保持UNKNOWN并暂停这一帧估计候选云，连续失败按有界实验规则终止。物理在同步等待worker时暂停，不能用sim时间中的调用间隔证明实时deadline。

[person-tiles receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-tiles-20261109T064314Z/receipt.json)记录真实`--tile-grid 2`离线IPC：4个事先选定的旧miss（3个远小目标、1个近距离裁切），4次新请求、20次实际forward（每次full frame+4crop）、model_load_count=1。baseline TP/FP/FN为0/0/178，tile2为0/62/178；它没有恢复这些漏检，反而增加误报。在线片段显式使用`--tile-grid 1`，不能因tile功能存在就声称改善或推广到全数据集。

## 模块、几何候选与结构化context

- [person_geometry_tracking.py](../../src/embodied_navigation/perception/person_geometry_tracking.py)：内部person ID、可见点中心/extent关联、短时quarantine。高score即时DYNAMIC；低候选保留UNKNOWN，待实际时序/几何支持。只在有显式二维MotionRegion时可短时传播，该接口存在不代表本次已接光流。AABB只是map可见几何隔离，不是当前像素mask、完整人体体积或永久身份。
- [temporal_geometry.py](../../src/embodied_navigation/perception/temporal_geometry.py)：独立重复几何候选缓存。默认0.1m voxel、至少3个distinct帧且span≥0.5s；最多200,000 voxel/每voxel8票，source retention300s；person隔离TTL2s。重复stamp不加票、危险证据仍可撤销整个voxel；隔离容量不足全局有界fail-closed并记录拒绝。默认参数是本轮prototype选择，不是静态物体真值。
- [learned_mapping.py](../../src/embodied_navigation/simulation/dynanav/learned_mapping.py)：将当前RGB结果转成临时前端候选，再等待exact post-VIO header接入几何/对象历史；pending有界、迟到旧匹配记录拒绝，不能用last pose或GT补洞。因最近person隔离而需要因果pose但pose缺失/过期时，不放行该候选云。
- [learned_scoring.py](../../src/embodied_navigation/simulation/dynanav/learned_scoring.py)：独立评估sink，renderer identity按指定person(2)与已知非person(1)作二分类，未知/机器人/prototype保留UNKNOWN(0)；不读GT pose，不把标签、评分或筛选结果传回算法。已知非person只是本协议负类，不证明物理静止；可评估person二分类，不能解释为全场动态/静态真值或P/R。

送入LIVO的`/dynanav/lidar_perception_filtered`是排除person/短时quarantine后的**临时UNKNOWN候选**，不是已经认证的静态云。持久geometry地图另经重复支持准入，`static_support_ledger.json`记录最后实测代表点的source/index及全部保留支持票；代表点污染和历史支持票污染分别评分，缺label也保留UNKNOWN分母。不能把最终代表点非person推成全部支持无person，更不能把重复一致推成物体本体永久静止。

本评分协议的`raw_static`/`removed_static`等历史字段实际计已知非person负类，不能把它们称作全场物理静态。person-only LayeredMap只管动态ID/history，几何候选单独输出；Agent context披露实际model calls、exact匹配、全流UNKNOWN、pending/evicted及候选点数，不把动态子集的局部覆盖当全流完成。该context不是已实现VLN、主动探索、完整房间拓扑或通用重定位。VIO仍使用原RGB图像patch，未给native视觉前端加动态图像mask；输入过滤也不等于native内部地图已通过逐点无污染证明。

此前[person几何helper离线receipt](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/person-geometry-tracker-offline-20261109T064230Z/receipt.json)有69 exact/5跳过、13,488 DYNAMIC/534,648 UNKNOWN/0 STATIC、2内部ID、33匹配/3歧义拒配和33可见运动样本。它只重放旧预测缓存，旧LIO上游仍是oracle-static cloud，end_to_end_oracle_free=false；不是在线通过、刚体运动恢复或IDF1。

## 45s真实在线片段：技术链路已核实，质量问题保留

实际目录：[dynanav-online-person-prefix-20261109T065302Z](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z)。[supervisor.py](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/supervisor.py)记录domain116、目标0.30m/s、duration45s、spawn-crossing、worker tile1、语义max/min间隔0.4/0.2s、SDK IMU200Hz，以及同一measured三目标参考。该调用间隔是本片段参数，不是历史固定语义频率。

[supervision.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/supervision.json)记录worker/LIVO/recorder/resource-monitor/capture五进程退出0；[最终独立审计JSON](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/audit/independent-review-online-person.json)和[可读报告](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/audit/independent-review-online-person.txt)进一步核对真实数据。`mission_completed=false`：9000个0.005s物理步完成，路线仍在stage0 cruise，40s运动预算截止置零，没有到达终点。监督success只代表技术字段，不等于完整导航或动态感知验收。

### 实际推理、投递与exact-pose接入

- worker权重原文件SHA一致、加载1次，118个真实请求/forward/结果按requestID、原RGB像素SHA和source header逐一配对。触发原因为initial1、motion_new_view52、max_interval61、low_confidence_confirmation4；118/45约2.622次/仿真秒。
- RPC均值84.73ms、p9588.40ms、最大447.10ms，总9.998s；GPU forward总4.229s。同步等待时物理暂停，45s物理耗loop wall121.616s，不能认证实时deadline，也不能把全部wall开销归于模型。
- 450张原始RGB归档连续、header唯一，bag只有421张，已录像素逐字节一致但缺29张。native收到412张RGB；原始归档、bag计数和native callback不是同一覆盖。
- 原始点云450帧仍保留行人；bag learned过滤云449/450，已录点均是原始有序子集。native实际收到450帧新learned候选云，逐帧点数与frontend_keep一致，25帧区别raw（含两个零宽帧），证明它确实接入了新输入，而非继续消费旧oracle-static云。
- 371条live post-VIO pose/status与371 native post-frame状态逐原header/pose一致，2030个nonnull controller pose输入来自这些输出，writer/bridge无已记录drop/error。mapping匹配371/450，16帧有界evicted、63 pending，没有用GT或旧pose补齐。

### Person漏过与几何支持票污染

[独立perception复算](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/audit/independent-online-perception.json)按实际调度状态区分点级漏过：

| 帧范围 | person采样点 | 保留person点 | 漏过比例 |
| --- | --- | --- | --- |
| 全部450帧 | 40,684 | 14,752 | 36.26% |
| fresh inference的118帧 | 14,620 | 775 | 5.30% |
| 未推理的332帧 | 26,064 | 13,977 | 53.63% |

主要漏过来自没有当前mask的未推理帧；有限历史quarantine不能认证无人。VIO仍用无动态mask的原RGB，不能把cloud过滤等价成视觉残差或native内部地图已去动态。

历史geometry admissions含**344个person点观测**，不能省略。最终几何候选则有13,407个代表点与NPZ逐行一致，全部104,753个保留支持票逐source/index/评估label独立核对，person0、unknown0；每voxel至少3个distinct exact-pose帧且跨0.5s。最终保留ledger没有person，只说明这一最终快照及其保留票，不能抹去历史344次污染、证明永久静态或证明自由空间。

### ID、hold及接触的限定

[独立ID复算](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/audit/independent-online-ID.json)从450原始云按标定回像素，与41个归档KF pixel index精确一致，重算371次integrated状态/ID overlap。7个fresh exact-pose且person可见帧中只有4个有confirmed重叠，3个miss；匹配帧只有1个ID、0 switch。这不是多行人IDF1或完整追踪覆盖。末对象last8.015s、到最终查询已过期；历史可见中心导数不是当前刚体速度，过期也不证明场景无人。

[独立control/hold审计](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/audit/independent-online-control.json)记录4.6s hold命令全零（blocked2.3s、unknown0.6s、clear_pending1.7s）。阻挡/恢复frame事件为6.715→9.215、13.515→14.215、16.215→17.215s；第一次在起步之前，是阻止起步与恢复，不能称高速紧急制动测试。

专用Go2-human contact列表0事件且未饱和，native body contact0；正常foot接触行8973，通用contact日志已达3000条存储上限，不能说所有接触为零。`contact_contract_validated=false`保留；停车距离等仍是实验参数，不是制动包络或安全净空验收。

本轮高层路线/在线持久建图不读native GT pose，输入用当前learned候选cloud；底层policy本体和物理guards仍消费SDK native状态。测量参考仍来自先前oracle过滤路线，repeat-spawn/无全局配准边界保留。person-only renderer评估是隔离分支，不能因此泛称整系统无任何模拟器真值。

## 同配置完整路线：路线完成，感知质量未通过

实际目录为[dynanav-online-person-full-20261109T070008Z](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z)。[supervision.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z/supervision.json)记录domain117、speed0.30m/s、duration上限210s；person-worker/LIVO/recorder/resource-monitor/capture五进程退出0，mission_completed=true。[现有记录最终汇总JSON](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z/audit/final-existing-records-summary.json)和[可读摘要](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z/audit/final-existing-records-summary.txt)只分析已结束数据，没有新测试、推理、仿真或算法重放。

该次实际完成三阶段、两次停车转向；source148.914993286s到达，first arrived policy为149.015s，最后policy tick154.095s，native末样本154.115s（154.1s物理）。末端连续零命令窗5.1s。目标仅为单人预设crossing的0.30m/s路线，不能把此前无行人1.0命令档拼接成“1m/s动态导航成功”。

| 完整运行的已记录指标 | 数值与范围 |
| --- | --- |
| 实测cruise XY速度 | 82.2s选择区间，均值0.270776/p950.354526/max0.429226m/s；命令为0.30 |
| 其它速度/终点评分 | 全程XY均速0.161063、非零命令均速0.180955m/s；离线world/base终点XY误差0.232307m、Z误差−0.002174m，不是SLAM RMSE，也不是estimated-map/IMU捕获门限 |
| 当前运行内mapping收据 | 1,541帧、353次模型调用、0模型error、1,396 exact配对、82 evicted+63 pending、149KF |
| worker/耗时 | 模型加载1次、353 forward；RPC均值79.80ms/p9584.88ms/max412.82ms；loop wall453.333s，同步暂停物理，未认证实时deadline |
| 前端person漏过 | 16,133/40,684点观测，39.6544%；感知质量未通过 |
| 历史geometry准入 | 272个person点观测进入过候选层，不能省略 |
| producer最终geometry评分 | 68,557代表点、499,600保留支持票均报person0；本次未重复逐票/像素/pose的全量独立审计，不借用45s prefix的独立结论冒充Full证明 |
| 单人ID overlap | 5个fresh exact-pose person可见帧，3 confirmed/2 miss，1 ID/匹配帧0 switch，非IDF1或完整追踪 |
| 动态hold与接触 | 动态hold累计5.1s全零命令（blocked3.1/unknown0.3/clear_pending1.7）；专用human contact0、body0，但foot接触行30,793、通用日志3000 cap、contact契约false，不是高速制动或全碰撞认证 |

运行内[perception评分收据](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z/capture/learned_scoring/receipt.json)与[mapping收据](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z/capture/mapping/receipt.json)分别保留这些计数。路线完成和最终Python候选快照的person0都不等于质量验收：漏过、历史准入、未配对和未检出仍存在；VIO未加动态图像mask，native FAST-LIVO持久地图无污染没有得到证明。

[45s prefix的原生污染可观测性审计](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-prefix-20261109T065302Z/audit/prefix-map-pollution-audit-20261109.json)另确认现有记录不足以逐点追溯native持久plane支持；Python最终clean不能推出FAST-LIVO内部clean。历史344个准入点也无法全部逐事件归因为撤销或支持票替换，最终快照不补造清理过程。

该完整基线没有启用新光流；未更改已冻结45s数据。repeat-spawn、旧测量参考上游oracle过滤、底层native本体/guards、单人预设路线和非完整办公室覆盖边界全部保留。

## 用户暂停决定与未完成能力

用户最新明确暂停后续工作：只允许当时已开始的domain117完整路线运行结束，此后仅分析已有结果、报告并释放本任务资源，不再启动新仿真、推理或测试；该运行现已结束。暂停也包括文档link/shell校验；本页最后补写不再以新的检查执行作为收尾前提。下方命令保留给未来显式恢复，不表示现在继续运行。

[person_mask_propagation.py](../../src/embodied_navigation/perception/person_mask_propagation.py)的`PersonMaskPropagator`和[测试文件](../../tests/unit/test_person_mask_propagation.py)已写入；停止前已完成Ruff及5个**注入flow**的纯测试，覆盖双向一致性/低纹理失败、严格source链及0.5s原anchor horizon。这些没有运行真实450RGB光流重放，未验证实际Farneback效果、延迟或非person覆盖；helper未接capture/experiment，也未上线。不能把5个注入测试写成真实光流或在线改善通过。

基线已有RGB view-change/实测motion触发（本45s运行记录52次motion_new_view），它不等于跨地图、跨视角的新颖度记忆系统。worker模块未新增独立novelty helper；其它新增能力按各自实际停止点记录，均不能据构想推断已接入此基线。早先辅助geometry counterfactual脚本因IndexError退出，没有结果，暂停后不重跑。

此文只记录已发生的运行和已完成的验证，不再等新试验改善数字。已按[resource-release.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-online-person-full-20261109T070008Z/resource-release.json)完成收尾：UTC07:09:37的核查中，已验证任务scope无存活进程，worker stopped/load1/inference353；本任务RViz进程已不存在，session39767退出0，GPU compute-apps为空。显存仍有1174/16303MiB图形等占用，未清理其他用户服务，不能称整张GPU显存为0。

源码和本阶段文档保留在未提交工作树；root停止点通报28个源文件未提交，最近本地提交仍为`43cba33`，本阶段未推送远程。未把未验证光流等能力纳入运行结果；后续只有用户明确恢复后再安排工作。

## 从仓库入口复跑相同配置

以下为实际argv对应的仓库入口，不依赖临时supervisor。使用新输出目录；三个终端先设置相同ROS环境、复制同一个`DYNANAV_RUN`绝对路径和domain。先worker ready，再LIVO observer和launch receipt，最后capture。不要打开仍在录制的SQLite做查询。

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=116  # 三个终端相同，复跑请用空闲隔离domain。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
DYNANAV_DATA="$PROJECT_DATA_ROOT/embodied-navigation"
DYNANAV_RUN="$DYNANAV_DATA/experiments/online-person-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$DYNANAV_RUN"
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_person_worker.py \
  --channel "$DYNANAV_RUN/person-worker" \
  --weights "$DYNANAV_DATA/models/perception/maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth" \
  --expected-sha256 73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e \
  --device cuda:0 --tile-grid 1
```

第二终端使用相同环境和已创建的`DYNANAV_RUN`，配置有界native快照：

```sh
mkdir "$DYNANAV_RUN/livo"
cat > "$DYNANAV_RUN/livo/snapshots.yaml" <<EOF
/**:
  ros__parameters:
    backend.snapshot_directory: "$DYNANAV_RUN/livo/observer/native_keyframes"
    backend.max_ready_directories: 256
    backend.min_interval_s: 1.0
    backend.max_interval_s: 2.0
EOF
scripts/run_dynanav_lio.sh --mode livo \
  --binary-prefix "$DYNANAV_DATA/build_cache/dynanav_post_vio_20261109T052245Z" \
  --observer-namespace /dynanav/livo_navigation \
  --lidar-topic /dynanav/lidar_perception_filtered \
  --capture-result "$DYNANAV_DATA/experiments/dynanav-20261109-livo-dual-stairs/capture/result.json" \
  --parameter-override "$DYNANAV_RUN/livo/snapshots.yaml" \
  --output-dir "$DYNANAV_RUN/livo/observer"
```

第三终端同环境、同目录；`--pose-launch-receipt`须来自这次observer，topic与`ROS_DOMAIN_ID`都必须匹配。旧校准source只提供安装标定，不提供导航pose：

```sh
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$DYNANAV_RUN/capture" --route basement-to-ground \
  --speed 0.30 --duration 45 --render-backend usd --imu-hz 200 --archive-rgb \
  --slam-reference "$DYNANAV_DATA/experiments/dynanav-20261109-post-vio-three-goal-reference-v1/reference.json" \
  --odometry-topic /dynanav/livo_navigation/post_vio_odometry \
  --slam-capture-remaining-m 0.05 --slam-capture-distance-m 0.10 \
  --slam-final-capture-remaining-m 0.15 --slam-final-capture-distance-m 0.20 \
  --pedestrian --pedestrian-scenario spawn-crossing \
  --semantic-provider learned_rgb_person --person-worker "$DYNANAV_RUN/person-worker" \
  --pose-launch-receipt "$DYNANAV_RUN/livo/observer/launch.json" \
  --learned-evaluation-oracle --person-max-interval 0.4 --person-min-interval 0.2
```

可先给capture命令加`--check`读取文件/参数和当前worker/launch收据，不启动新的ROS/SDK。已有worker和observer的ready状态仍是前提。省去`--learned-evaluation-oracle`才不会请求这个person身份评估分支；不改变算法输入。完成后只停止自己启动的capture、observer、worker与录制进程。完整日志/录包/资源监督见实际run收据，以上直接命令不冒充相同的监督/录包产物。

## 检查与后续实证

暂停前曾完成unit283项（3 skip）与regression86项（5 skip）的完整套件；后续接线9项与注入flow5项分别单独通过，**没有重新执行覆盖最终全部文件的完整套件**，不能把子集计数相加成新的整套通过数。temporal cache13项、在线边界review10项用于定位具体职责，不与整套数相加。native2/2是提交`43cba33`阶段的历史结果，本阶段native未改、未重跑。

worker/tile与几何ID helper保留各自离线收据；在线完成/质量结论按上面的现有运行记录分别陈述。暂停后没有再运行link、shell或其它测试检查，也没有启动新推理/仿真。历史正文未用本阶段结果回写，当前报告至此冻结。
