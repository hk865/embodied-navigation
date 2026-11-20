# 真实办公室地下层入口：DSH 探索试行

本报告记录恢复真实墙体/楼梯后新的探索试行，不沿用原 RTX 缺几何场景的地图或视觉判断。现已记录真实模型选择、原控制器执行的短距离移动；目标仍因原30wall秒TTL到期未达，未完成地下室、多房间或回环探索。详见第七、八轮。

## 首轮：背景进程非正常退出，不计视觉模型闭环成功

- Run：`rtx_basement_dsh_20261110T211225Z`，domain174，运行目录为数据根 `embodied-navigation/experiments/` 下同名目录。
- 使用 `rtx_scene_verified_20261110T210212Z` 的原 `run_full.sh`、只读 `production_evidence.py` 和 same-pair native build `20261110T171010Z`。默认出生和 yaw=π 未改；240s 是配置预算，不是实际运行时间。
- 未改原资产、源码、安全门、TTL、freshness、pose/clock 门；无真机操作。RTX 生产分支不启用 depth，不能说已通过 RGB-D 补盲。
- 最后物理时间9.175s，最后采样文件修改于 UTC21:12:55.656；结果仍为running，无capture_exit、owned_pids清理回执或terminal。后续宿主ps确认全部owned进程消失。无kernel OOM证据，不能当正常停车。
- 启动方式是退出调用中的nohup后台进程。它约30s后消失。一个独立45s后台sleep探针在24s时仍活，到46.6s已消失，且没有complete或TERM trap输出。该对照支持执行调用生命周期清理后台后代，尚未定位内部实现。后续改用前台持久exec session等待结束，不改仿真逻辑。

## 实际感知与DSH输入

DSH任务为 `2026-11-10T21-12-25-655Z-81d023bb`，使用已运行host，无新服务、权限或配置变化。

- UTC21:12:52.553正式observe、21:12:52.694只读helper，均在首轮采样截止前。helper成功取得sim6.315、post-VIO pose6.1s，地图10接受帧/25.96m²、free7616/occupied2768，last同stage LIO pair5.5s。地图读取成功，不是仅尝试路径。
- 第一张实际read_image为UTC21:13:04.162，`rgb_000003.png`；第二张为21:15:09.999，`rgb_000004.png`。两张图均在采样截止后才被模型打开。因此本轮仅有实时状态读取，不是完成的实时视觉模型闭环。
- 图像显示正前方真实楼梯、墙和扶手，不再是旧试行的天空/土路画面。不能因局部SLAM level `+0.00`就把地图称为完整地下室或完整楼层。
- 最终保留地图23接受帧、37.47m²、free11987/occupied3001；这一累计观测面积不等于可通行面积或探索进度。
- 9.0s感知：31200点，native full deskew云/同stage LIO pose/reference均9.0s。四方向都为`insufficient_observation`、clearance=null、floor_support_complete=false、fail_closed=true。前向最近障碍0.8988m，近场floor未知[0,1.0347]m；其他方向约[0,1.06–1.17]m。全角度有射线并不能补齐近场连续地面支撑。
- 没有提交goto/yaw/maneuver/stop；只记录not_ready和perception_clearance_unknown两个hold，实际命令全零。静态站立自然摇摆不计为探索移动。

DSH对数据卷写入探针收到Read-only file system后，监督明确停止该操作，不改变其权限；模型任务于UTC21:15:16.128取消，status=cancelled/aborted。后续只用既有DSH结果通道传递决定，不再要求写数据卷checkpoint。

## 证据

- 首run：`supervisor_snapshot_01/`、`supervisor_termination_audit.json`、`fabric_time.jsonl`、原生采样/控制日志、`capture/operator_map/`、`operator_grid/`。
- DSH精确工具参数审计：`.dsh-subagents/basement_dsh_command_audit.json`；成功输入读取内容：`.dsh-subagents/basement_runtime_audit/input_read_evidence.json`。未复制模型内部推理为交付物。
- 背景生命周期对照：`.dsh-subagents/basement_runtime_audit/background_*`。
- 首轮视频编码未正常关闭，不能作为完整有效视频交付。

待独立CPU修正楼梯混入floor拟合并冻结后，再以新generation及前台持久会话重试。地面拟合修正不等于消除传感器近场盲区，不能把unknown改成FREE。

## 第二轮：新增下视depth传感器零动作验证完成

Run `rtx_nearfield_sensor_20261110T212944Z`，domain175，前台持久exec session。原朝向π、12sim秒、仅`--simulated-nearfield-depth`，不启fusion、不dispatch DSH、不发航点。

- capture正常exit0，2400个0.005s物理步、600个policy frame实际命令全零，原motion budget收尾；所有owned仿真/ROS进程退出。它是传感器验收，不是探索或移动验收。
- depth118帧，各NPZ hash、像素掩码和全部反投影点逐帧复算一致；原生间隔0.1s±1ns。depth与前视RGB逐条stamp相同，event/reference/after_reads一致，未使用body当前时刻替代。
- 原生FAST-LIVO diagnostics完整收到IMU2400、RTX118、RGB118，三种逐帧stamp与producer一致；writer attempted=written=4845、dropped=0、IO失败=false。76个LIO和76个post-VIO输出。旁路collector仍有best-effort丢包（RTX69/118、full cloud60/76），不冒称旁路无损。
- 下视与前视同stamp固定外参对118组；位置误差最大1.37e-8m、角误差1.71e-6°；下视renderer位置范围[.0321,.0408,.0988]m，实际跟随机身而非冻结。前视/RTX独立GT诊断保留原有初始化误差，5s后前视最大.955mm/.296°、RTX.418mm/.0883°；仅离线诊断，不输入导航。
- 同帧robot ID像素中位7104，全部留为unknown，未补遮挡后地面；最近前向端点中位.24935m。最后0–.35m中心10°扇区0点，仍有遮挡。端点数量不等于floor支撑或通行证；当前朝向仍面对楼梯。
- depth处理p50/p95=10.08/10.86ms，归档1.91/2.17ms，队列等待p95=79.66ms、最大队列2帧；原生采集physics lag最大.110s。118个NPZ总195,637,212字节。loop22.8346wall秒跑12sim秒，RTF=.5255；旧同12s零动作基线RTF=.6190，非严格同负载性能因果对照。未启动峰值GPU内存采样，不宣称峰值内存。
- source manifest50条与当前树及snapshot逐hash一致；原office.usd SHA未变。地图继续仅RTX、floor-only、无ray carving；depth的control/map/SLAM字段逐帧为false。
- 视频正常编码118帧，1024×512、10fps、11.8s。`capture/multiwaypoint05_overview_and_rgb.mp4`仅站立观察，不是探索视频。

第二轮详细证据：`nearfield_audit.json`、`final_audit.json`、`pose_audit.json`、`sensor_integration_audit.json`、`owned_started.tsv`、`lifecycle.log`、depth不可变档案及原生diagnostics。下一步须由真实数据严格floor/障碍/同时间配对验收决定，不能仅因相机补了部分近场端点就允许向楼梯推进。

## 第三轮：同位置朝向0重置，诊断联合覆盖与写出后年龄

Run `rtx_nearfield_heading0_diag_20261110T213641Z`，domain176。根据上一轮真实后向LiDAR的同层地面/约2.26m障碍证据，将新generation初始yaw由π设为0，出生位置不变；这是配置重置，不是DSH转身成绩，不复用地图。独立entry `capture_heading0.py` SHA `2a4e8592b3d7d3da74204ba47bd50dcf87210e7f202308926ae074b2fb76d683`，CPU比对profile仅spawn_yaw一个键变化，result明确记录initial_heading_reset。

本轮20sim秒预算，下视开启，bridge仅传`--nearfield-depth-manifest`诊断，不加enable，不dispatch DSH，无航点。读取真实/clock的独立旁路在每次perception写出后保存原payload语义、每源stamp/arrival、收到的clock/receipt wall、文件mtime、新文件读入wall；不改stamp/arrival、不反馈控制。该旁路包含write→read/poll延迟，因此是保守消费者年龄，不冒称精确计算结束时刻或实际physics-now。

结果：

- 9.455sim秒，连续hold最后由stale_sim_odometry触发原`interactive_pose_lost`，停车到14.5sim秒后正常exit0；不是20s跑满。2900物理步、725policy命令全部零。所有owned进程退出。
- 66份写出样本：60个compute前合同accepted，6个invalid_clock。**写出后0个满足全部原年龄门**：60个接受样本中59个sim age>.25s（p50=.350、p95=.410）；depth wall age19个>.5s（p50=.468、p95=.574）。RTX/pose wall age中位约.160/.162s。唯一sim age=.215样本depth wall仍超限。负年龄没有归零；独立clock receipt age中位.0029s，不以当前body时钟替换。
- 60个联合candidate全部`floor_ambiguous_surfaces`，3–9个候选，所有front仍floor_unmeasured。新RGB可见室内地板和红门，但图像外观不替代连续地面几何，当前不能进入控制。
- 144个depth档案复算/哈希与同stamp前视外参通过；IMU2900、RTX144、RGB144 native逐stamp完整一致，6126diagnostics零丢。50条source manifest无漂移。全部perception仍原native source，所有control_used=false，原native map不混入depth。
- 实际loop RTF=.5462，视频144帧/14.4s正常结束；无运动或探索成功声明。

第三轮证据：`live_age_summary.json`、`live_perception_ages.jsonl`、`live_perception_samples/`、`live_perception_integrity.json`、`nearfield_audit.json`、`final_audit.json`、`pose_audit.json`、entry哈希和lifecycle。integrity SHA针对旁路保存的语义等价重序列化JSON，不声称与producer原文件字节格式相同。

### 第三轮终态归因：连续混合hold，不是VIO停止

追加只读native/collector/capture时间审计，未新启GPU。`native_timing_audit.json`、`native_timing_rows.json`与`capture_callback_join.json`给出逐stamp证据。

- LIO及post-VIO各102帧，source4.3→14.4s无缺帧；99个间隔100ms，另±1ns各一次，未修正原stamp。collector收全102帧。101帧visual_update_applied=true，仅初始化首帧false。朝向0不是VIO因无纹理停住。
- LIO估计耗时p50/p95=6.89/9.52ms；VIO process段3.65/5.10ms，最大5.92ms。pipeline正常收尾，16561个ingress全部committed，pending/inflight/ready=0、peak_pending=1、无容量拒绝。没有启更细phase/sync ledger，不能声称内部每一分支都被记录。
- image ingress→post-VIO呈交替快慢：51帧<60ms，51帧>200ms。慢帧大部分时间在等同stamp RTX完整扫描到达；两输入都到后LIO启动p50=2.80ms。它不是VIO计算占了两三百毫秒。
- 利用原perception arrival与controller的perception_wall_age精确重建tick now_wall，再减odom wall_age，得到32个独立stamp的capture callback receipt。重复样本重建差0；相对collector同stamp到达延迟p50/p95/max=9.91/24.82/31.94ms，未见capture不消费的长冻结。
- native post-VIO完成时，以独立收到的/clock计source age p95=.220s、max=.235s。50Hz消费10Hz保持样本时，active tick的pose age p50=.235/p95=.315s，62.79%满足原.25门；另有间歇pose过期。
- 第一个active hold在6.015sim秒（perception_stale），最后9.455sim秒（stale_sim_odometry），两者精确相隔**5.000475wall秒**。终态前172tick中108个pose新鲜但perception过期、64个pose过期；terminal tick再计一次pose过期。完整门从未通过，统一hold计时未复位，最后一步选择了interactive_pose_lost标签。不能把这个标签理解成post-VIO已经连续停了5秒。
- 上轮无fusion诊断的sensor-only active pose age同为p50=.235/p95=.315s。朝向和诊断计算同时改变，未做严格性能因果对照；当前证据支持先修感知计算延迟/伪floor候选，不能归咎红门视角、DDS reliable背压或宣称应放宽pose门。

## 第四代：开启融合安全感知的零动作验收（domain177）

- Run：`rtx_nearfield_fusion_zero_20261110T215025Z`。同出生点、heading0 新generation reset，非 DSH 转身；不复用旧图。计划24sim秒，未发送 DSH、waypoint 或 maneuver。
- 桥显式同时传入 `--nearfield-depth-manifest .../nearfield_depth/latest.json --enable-nearfield-depth-fusion`。这轮确实将融合结果接入 safety perception。原始depth录制器的 `control_enabled=false` 仅表示不由传感器直接写控制；通用sensor-only审计里固定的 `control_fusion_authorized=false` 不能作为本轮配置依据。准确配置在 `harness_meta.txt` 和归档 `run_foreground.sh`。
- 冻结源：floor `599defc48924ae11bddb624fbe53be04dd0b0e5256ab142757ab2aa634ce243a`，fusion `fa173174c42cc9b6d7054136a5a4904c601d2258736e1c0843e161b1670b4c60`，bridge `cb29a15e9be43be0e5d99550b73bd6864c0928376985a951be78d57cb1d8951f`。50份生产source manifest与归档/当前树全部相符。未修改资产。
- **几何阻塞已有改善，但时效仍未通过持续验收。** 80份只读归档均有完整front地面支持、clearance3.0m，未知后向不被填FREE。实际producer输出57份fused，23份退原RTX（17份 `post_compute:invalid_clock`，6份 `post_compute:cloud_wall_expired`）。地图仍仅用native RTX，不融合depth。
- 独立收到的真实 `/clock` 与publication后原source arrival核验：仅6/80全源满足原门，其中5份是实际fused；accepted fused source sim age p50/p95/max=.315/.376/.400s，depth wall age p50/p95=.367/.493s。文件mtime→旁路记录p50/max=3.19/6.47ms，不能把此差异归咎低频旁路。未替换或重贴任何源时间。
- 原tracker只在8.435sim秒通过一次（post-VIO和融合source均8.199999999s，pose age=.235000001s），phase=idle，command=[0,0,0]。下一tick8.455s恢复hold，到11.715s连续混合hold5.04676wall秒后触发 `interactive_pose_lost`。终态前285tick：178 perception_stale、106 stale_sim_odometry、1 tracking；terminal触发tick再计1次stale pose。不能称为已稳定可走或DSH探索。
- 正常停车后16.8sim秒/3360physics结束，840个policy实际command与全部controller command为零。capture exit0于21:51:28.758Z，cleanup21:51:31.763Z，前台持久session正常退出，所有owned PID核验已退。服务寿命已改为capture所有权cleanup，独立3600wall秒硬上限，不再从sim duration加常数推wall寿命；真实heartbeat时间及动作TTL未改变。
- 166depth/166RGB/166RTX/3360IMU，native7245/7245诊断写入、0丢弃；native RGB与camera源stamp完全一致，native RTX与capture scan start stamp完全一致（原始LiDAR scan start不能误比RGB render stamp）。166depth档案全部SHA/重投影验证，相对front外参最大位置误差2.59e-10m、角误差1.71e-6deg。记录video166帧/16.6s正常，只展示站立，不能当行进演示。
- Native LIO/post-VIO各124帧，4.3→16.6s全10Hz nominal无缺帧，123visual update applied。VIO process p50/p95=3.75/5.29ms，capture回调相对collector同stamp延迟p50/p95/max=9.13/24.93/34.68ms，未见VIO停产或回调冻结。source age在50Hz消费时仍p50=.235/p95=.315s。后续必须处理真正source时延/桥clock消费，不放宽门。
- loop29.782wall秒，RTF=.5641（含站立、停车，不可单独归因某sensor）。首次1Hz只读GPU采样57次无错误：整卡最大5596MiB，实验Isaac进程最大4365MiB，整卡GPU利用率最大90%；整卡含桌面，离散采样不等于瞬时峰值，不足以做单因素因果结论。
- 证据：`fusion_zero_audit.json`、`nearfield_audit.json`、`native_timing_audit.json`、`capture_callback_join.json`、`live_perception_ages.jsonl`及80份原payload、`gpu_usage.jsonl`。没有启动第五代。
- 给DSH的自有只读helper已改为实际source/directions、compact每源stamp/arrival/hash、融合candidate、tracker validated/current/fresh和latest hold。无world/renderer/GT字段；历史validated明确不等于当前可用。尚未实际dispatch下一次DSH。

## 第五代：真实DSH只读视觉决策，但仍因感知时效终止（domain178）

Run `rtx_basement_dsh_fusion_20261110T220446Z`，计划180sim秒，同heading0 reset，不跨代合图。冻结floor `bca56c7394cc7294ab1a1c008acd7f86255087d97019ab94b862b5d373df01c1`、fusion `bc9157cf5888bf3c6294d8a2689f43515960578b2f14d85fdb47f3bd92310ee9`、bridge `145d466e7e9972a7487fdfcefa2423715af83bb377cd13e3abb392559d4debf3`；50份source snapshot和审计时当前树均无漂移。原门未变。

- **本代未完成导航。** 原tracker从6.015sim秒起持续混合hold，9.515sim秒触发 `interactive_perception_lost`，没有一次validated/tracking。之后正常停车至14.6sim秒、2920physics。730policy实际command和所有controller command均零；post-VIO站立净XY变化0.01693m、累计XY抖动路径0.10524m，不记为探索位移。
- 新桥确实在计算后pump真实clock并拒绝过期：60份candidate前向均有完整floor/3.0m clearance，但54份 `post_compute:cloud_sim_expired`，仅6份accepted_post_compute=true，4份独立postpub全源fresh。全部payload的source名字都是fused，所以仅按source统计会得到错误结论。
- 6份接受source为5.0、7.2、9.8、11.4、13.6、14.4s。5.0在controller启动6s之前；7.2是唯一active期候选，bridge after-pump clock=7.449999809s、独立读到时clock=7.454999924s，年龄从约.250变为.255，赶不上原pose/融合门。其余接受都在终态之后。compute p50/p95=97.67/114.36ms；postcompute→独立旁路p50/p95=2.08/3.00ms，不能把过期归咎旁路延迟。
- native LIO/post-VIO各102帧持续产生，原source间隔10Hz nominal无缺帧；capture回调相对collector同stamp p50/p95/max=10.01/24.52/30.51ms，没有停产或回调冻结证据。原始sensor144RGB、144RTX、2920IMU；144份depth SHA/重投影通过。原地图仍仅native RTX。

### DSH模型结果与读图时间边界

- Task `2026-11-10T22-05-30-004Z-100b4bde`，turn1，round `20261110T220521Z-b6b96211`。22:05:39.738Z通过原adapter提交；原生completed为22:05:56.931Z，不靠result.md存在推断完成。
- 原生成功工具证据：22:05:41.640Z读helper JSON；22:05:44.049Z实际 `read_image` rgb_000006.png；22:05:48.984Z刷新JSON；22:05:51.636Z实际read_image rgb_000007.png。仅归档公开工具结果和最终意图，不复制内部推理。
- 首次read_image早于末sensor文件写入22:05:44.644Z，但已是controller终态后的停车阶段。第二次read_image晚于physics/sensor结束、早于app关闭。**不能说模型在可控状态下完成了视觉导航**；可以确证它真实看了本代两张图和本代数据。
- 模型最终输出单个结构化hold，理由是terminal、当前pose为空、地图过期、tracker从未validated。它注意到最后payload可能accepted，但没有将其当作可走许可。没有任何/media写入、进程操作、GT导航读取或绕过拒绝。
- host relay核验completed turn、run/round、真实read_image成功记录、完全相同helper命令及实际输出后保存 `model_hold_no_control_write`。`controller_seq=null`、`control_request_submitted=false`：hold不是一个伪造的controller动作，也没有自动微动作链。
- 唯一启动预检误报来自同一个父shell命令字符串包含pgrep模式，第一次退出91且未创建任何服务；随后单独运行启动命令通过独占锁/无并发检查。没有重复仿真。前台session79114正常exit0，capture_exit=0于22:05:58.543Z，cleanup22:06:01.547Z，所有owned PID已退。

### 资源与归档

- 1HzGPU采样55次、无错误。整卡采样峰5342MiB（包含桌面），实验Isaac进程4109MiB，整卡利用率max85%。不是瞬时峰值或单因素性能因果证据。loop27.21472wall秒、14.6sim秒，RTF=.5365。
- video144帧/14.4s正常，只记录站立停车。轨迹使用collector post-VIO和控制记录，不能把预置朝向变化/站立摇摆计为导航。
- 核验入口 `fifth_dsh_audit.json`。原始模型意图、task/turn、图片SHA/读图时间、helper完整实际输出、relay状态都在 `dsh_rounds/20261110T220521Z-b6b96211/`。另有nearfield/native timing/独立postpub clock/GPU档案。未启动第六代。


## 第六轮：显式非实时锁步模式，模型意图没有成为动作

Run `rtx_basement_nonrealtime_20261110T222614Z`，domain179，180sim秒。同出生点yaw0重置，不跨代合图。显式 `--nonrealtime-sensor-lockstep`，其余depth/fusion和安全合同与第五代相同。该模式只是静态模拟试验，不是异步实时或真机延迟修复。

- 36000正常physics步，1800次render-only等待前后time/count/Fabric/timeline/native-state hash严格不变，额外physics/policy/clock均0，source/arrival未重写。180sim耗470.555wall，RTF .38253。deadline配置.15wall，非可抢占render调用使实际最大单次.24912wall；不能宣称硬实时上界.15秒。
- 1479个旁路perception样本，1380个postcompute接受，1376个独立postpub原年龄门均过。实时异步阶段的问题没有被掩饰，锁步换取可用窗口，同时仍保留真实wall-age拒绝。
- DSH任务c8e5a449：前两轮hold；第三轮真实选择约.9m目标，但原relay纯JSON格式解析拒绝了带前缀的最终JSON，claim前退出。格式修复仅提取唯一无歧义末尾schema对象、不eval、不改字段/坐标。第四轮重新读新图独立选择，但模型completed后人工工具调度延迟15.4wall，转交时已经175sim原budget终态，仍拒绝。四轮没有任何控制请求；9000policy实际行走命令全0。
- 下一代改为提交模型前预置严格completed→relay监视，消除人为转交延迟，不自动选目标或放宽门。只读JSONL读取修复仅跳过写入中的不完整尾行，保留完整原事件；历史≤5wall同source匹配窗口明确标为历史，不冒称当前fresh。
- bit-exact depth审计发现24.1s帧49552×3点中一个分量差9.09e-13m，原NPZ SHA有效；`nearfield_exact_reprojection_failure.json`保留。未更改生产精度阈值，不能宣称全帧bit一致。在线4次points_base_m校验失败均按原门拒绝。
- 正常capture exit0、全部owned退出。证据：`completed_motion_audit.json`、逐wait档案、原DSH结果、各轮无claim/无request以及完整视频/地图。

## 第七轮：0.1命令上限下首次模型驱动真实短移

Run `rtx_basement_nonrealtime_retry_20261110T223709Z`，domain180；DSH任务78768421。仍非实时、同出生点yaw0重置、独立地图、原guard .65，TTL30wall。

- 新图/pose/地图真实读取后，模型自主选.75m目标 `[.7612,-.0803,.0055]`，seq1接受。completed→请求约.146wall，预置监视不改目标；首非零23.975sim，最后34.895sim。
- 198非零tick，合3.96sim秒；前向命令积分.396m不等于位移。首动作前稳态23.955至34.915sim，native净XY .07937m，post-VIO .07571m；351次perception_stale hold。30wall TTL到期，未到达，未发生stall/多房间探索。
- 第二模型根据新观测另选.6m目标，但host将62.5与62.50000000000001作浮点严格相等导致claim前拒绝。已改为原生整数ns对比：二者均62500000000，真实±1ns仍拒绝；不改源stamp或sensor门。旧意图超过30wall未重投。
- 经监督明确stop seq2，于114.415sim停车，119.7sim正常结束；这是监督stop，不是模型停或导航成绩。全部owned退出。1197次wait前后冻结、无额外clock/physics/policy；RTF .38024。
- `seq1_execution_audit.json`与`completed_motion_audit.json`给出原始基线/末点/命令积分/收据。关节站立抖动和总路径积分不作为探索距离。

## 第八轮：0.3命令上限受控对照，真实推进约0.27m

先行run `rtx_basement_nonrealtime_speed03_20261110T224357Z` / domain181在原生RTX场景/material启动阶段segfault，未运行physics/模型动作；退出后另建新代，不改资产/源码/cache。

有效run `rtx_basement_nonrealtime_speed03_retry_20261110T224554Z`，domain182。独立entry保持yaw0与原physical guard .65；对上一heading0配置只有speed .1→.3变化。生产make_profile(.3)默认guard .8没有使用，`profile_override_audit.json`明确留证。命令上限不等于实速，用户1m/s巡航目标尚未完成。

- 180sim/36000physics步，耗471.16089wall，RTF .38204。1800次wait前后time/count/Fabric/timeline/native hash相同，额外clock/physics/policy均0。单次实际等待最大.23755wall（配置deadline.15wall，不是硬抢占上限）。
- DSH任务1f7b6241。第一轮虽读图，但私自把helper重定向到/tmp再输出自选摘要，违反本轮只读原命令约束，strict proof拒绝，未claim/未request；这不是环境拒绝或绕过/media拒写。host随后把默认stdout压至10.65KB，保留四方向、原source/arrival、pose、真实窗口、map/receipt。第二轮因host瞬时读到过期observation拒绝；新增最多1wall只读等待真正新文件，门限仍.5wall。旧意图未重投。
- 第三轮真实新图 `rgb_000062.png`于UTC22:51:45.187打开，状态读取22:51:42.100，均在运行期间。模型自主选前向约.7m `[.712,-.074,.008]`，completed22:51:50.370，host relay在.1631wall内转交，seq1接受。rear没有floor支撑，模型没有选后方；未知不被当作FREE。
- 首非零128.375sim、最后139.495sim。137个非零tick共2.74sim秒；前向命令积分.72601m不等于位移。前动作稳态128.355至139.515sim的native净XY .27278m、post-VIO .27621m；native瞬时3D峰.26101m/s，post-VIO .5s窗峰.13340m/s。未触.65guard。执行窗421次perception_stale hold，30wall TTL到期，目标剩余约.42677m（到达容差.30m），未到达。最终停车稳态native净移.27475m。
- 1466perception样本中1400postcompute接受、1397独立postpub全fresh，仍未保证每个控制tickfresh。1帧points_base_m严格校验失败正确拒绝，其余wall/sim/clock拒绝原样保留。51个source manifest与当前树一致，生产源未漂移。
- 有界旁路保存40对真实运动期native full cloud + 同stage LIO pose，source128.4–133.2sim严格同ns；每对完整CDR、raw cloud bytes SHA、fields/header、原arrival和真实clock均留。所有CDR hash复核一致，不是raw scan或post-VIO替代。目录 `motion_native_pairs/`。
- 显存498次1Hz采样：整卡含桌面峰5344MiB，owned实验进程峰4109MiB，整卡util峰93%。非严格同负载因果对照。视频1799帧/179.9s正常编码，map仅RTX floor-only，累计观测面积不是安全可通行或探索覆盖。
- UTC22:54:12.233 capture正常exit0，22:54:15.236 cleanup，全部owned退出。完整指标 `completed_motion_audit.json`；原model最终意图、proof、claim、seq1请求和接受/到期收据均在run目录。


## 第九轮：0.5命令上限，三次模型新决策，最终感知过期停车

首启 `rtx_basement_nonrealtime_speed05_20261110T225751Z` / domain183 在首.015sim的native render/orchestrator step崩溃，日志 `corrupted double-linked list`，DSH未提交、无请求。保留原crash/Python stack；TERM wrapper后exit143，卡住的明确owned native PID3592732经KILL退出。原资产/cache/生产源码未改。确认无活动仿真后，同配置新generation重试一次。

有效run `rtx_basement_nonrealtime_speed05_retry_20261110T230043Z`，domain184，DSH `2026-11-10T23-00-43-191Z-a01c3a46`。只将上一heading0配置speed .3→.5；生产make_profile(.5)默认guard1.0显式还原为.65。初始yaw0仍是重置，地图独立。TTL严格created_wall+30，未转成sim时钟，所有source/age/unknown门不变。

6.115sim通过60次真实冻结等待及同source近期窗口后立即提交第一轮模型；每轮提交前均预置completed→relay监视，原意图exactly-once，原图与完整紧凑helper输出证据约10.7KB，无文件写入/GT导航。

1. seq1：新图独立选前方约.70m `[.7092,-.0759,.0049]`，接受后148非零tick/2.96sim。前动作21.855至33.115sim，native净XY .26593m、post-VIO .25447m；剩余目标.44361m，30wall TTL到期，未达。实际指令峰仅.34932m/s，native 3D峰.133998m/s。非零tick占26.29%，415个stale hold。
2. seq2：收到原到期receipt和实移后，模型重新读图/pose/map自主缩为.55m `[.8169,-.0875,.0133]`，没有重发seq1。148非零tick/2.96sim；58.555至69.615sim native净.18017m、post-VIO .18544m，剩余.36630m，原TTL再次到期。实际指令峰.27514，native峰.11399m/s；非零占26.76%，405个stale hold。
3. seq3：明确告知原follower公式 `vx=min(speed_ceiling,max(.04,.5*remaining))` 与真实wall TTL后，模型再读新图/地图，自主选前方已测3m范围内1.15m目标 `[1.5593,-.2377,.0158]`。seq3接受，实际发出.5m/s命令，但仅16非零tick/.32sim。115.855至119.915sim native净.03645m、post-VIO .02178m；native峰.15920m/s。121.815sim触原 `interactive_perception_lost`，非TTL到期或.65速度guard，未到达。pose当tick仍新鲜(.115sim/.0116wall)，perception wall=.6113>.5，unknown/过期继续停车。

非零命令期native速度均值分别.03824/.02898/.05843m/s，峰分别.11162/.08992/.11898m/s；这些是对应非零tick窗口的真实物理状态采样，不是命令值。上述较高的全执行窗峰包括hold/过渡期。没有证据达到.5m/s巡航，更没有达到用户1m/s目标。

三seq均未capture reached。总312非零tick=6.24sim，前向命令积分1.63292m不是位移。首动作前稳态至最终停车native净XY .42890m、post-VIO .42657m；若端点取最后动作后瞬间则.45232/.43803m，必须注明取样时刻，不能把逐段净移或身体摇摆路径相加宣称探索距离。仅在同一局部走廊有限推进，没有多房间、完整地下室或回环成绩。

终态原因须保留消费者时相这一细节：最后5.5wall内18个perception输出中16个postcompute和独立postpub都fresh，只有2个过期，但留给控制消费的余量很小（119.6source写出时depth wall=.4593s，compute=.1315s，余量约41ms）。不能说bridge连续5秒完全没有fresh输出。终态121.6source depth wall=.5074，已按原producer门拒绝。`terminal_perception_window.json`保存原source/arrival/clock，未把历史fresh当作当前许可。

最终126.9sim/25380正常physics步，耗337.65390wall，RTF .37583；1269次等待前后物理/time/Fabric/timeline/native-state均相同，额外physics/clock/policy均0；配置deadline.15wall，最大实际等待.24626wall。1043perception样本，997postcompute接受、996独立postpub全fresh。LIO与post-VIO各1225连续10Hz源输出，无阶段中断；VIO p95 6.2ms，capture相对collector消费回调p95 8.59ms。不能把问题归为VIO卡死或可通过提高速度解决。

所有51个source SHA无漂移，原office资产不变。365次GPU采样：整卡含桌面峰5601MiB，owned实验进程峰4365MiB。40对本代运动期native full cloud/同stage LIO pose与原字段/时间/CDR/raw bytes SHA另存，非raw scan替代。视频正常关闭。capture于UTC23:06:59.573正常exit0，23:07:02.576 cleanup，全部owned/native进程退出。

交付证据：`sequence_motion_audit.json`（各seq原receipt/速度/位移/hold）、`completed_motion_audit.json`（总体、明确多目标不能用单目标剩余距离）、`native_timing_audit.json`、`terminal_perception_window.json`、原DSH轮次proof/JSON/request、`motion_native_pairs/`、原轨迹/地图/视频。后续优先验证完整扫描时间watermark与异步消费者时序，不继续盲目加速或延长TTL。

## 后续只读消费时相审计

见 `dynanav-perception-consumption-audit-20261111.md`。精确连续5.003wall窗不是所有tick都stale：96tick中92次最早depth wall过期、4次fresh但floor unknown。16个写出中14fresh，分为8个到下个tick前过期、4个被原50ms文件缓存错过、2个几何unknown；冻结源码CPU回放逐tick完全复现。19次锁步全timeout，全run8次已ready的非初始化下一tick均tracking。此处没有修改源码或重启GPU，不据局部反事实宣称端到端已修好。
