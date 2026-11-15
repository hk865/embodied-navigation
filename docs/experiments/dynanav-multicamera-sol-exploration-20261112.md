# 四向 SOL 探索、ROS bag、漂移与原生四路更新实验

本任务完成了由工具明确指定 **gpt-6.1-sol** 操作的四向传感探索、可回放 bag 和事后漂移评分。随后新增默认关闭的原生四路更新路径，并完成同一运动包前 180 秒的单路/四路对照。**四路实验尚未证明稳定精度收益或统计协方差一致性，默认保持关闭。**

必须区分两段工作：

| 范围 | 实际定位输入与反馈 | 已验证范围 |
| --- | --- | --- |
| 475.2s 在线 SOL 探索 | 单前相机 120° RGB + RTX/IMU FAST-LIVO2；四向 RGB 给模型决策；旧视觉旁路/CI不回写 | 大厅多方向、走廊往返、转向与初始附近回访，正常停车 |
| 原生四路实验 | 冻结归档中同源四图约 1Hz；在 `LIVMapper::handleVIO` 内真正更新 `_state` 并沿原流程发布 post-VIO，影响后续 LIO；其它图像时刻使用单前相机 | 同包 0..180s，输出 4.3..179.9s；175个四图组，173次共同状态提交、2次单前回退 |
| 尚未完成 | 实时同步四路10Hz、整包四路运动回归、可信统计协方差及正式默认切换 | 不把本轮说成实时四路定位已验收或全楼探索 |

## 模型、技能和执行链路

当前机器默认配置是 `gpt-6-astra`，因此基础设施代理没有自称 SOL。通过 `collaboration.spawn_agent` 显式选择 `model=gpt-6.1-sol`、`reasoning_effort=high`，成功创建 `/root/sol_navigation_controller`，由它独占 goto/cancel/stop 决策。工具选择参数、成功创建结果和分工保存在 `controller_model.json`；工具未提供额外的服务器端模型身份签名。没有替换 DeepSeek，也没有调用外部付费 API。

SOL 读取当前[项目技能](../../.agents/skills/dynanav-explore/SKILL.md)与[原生接口](../../.agents/skills/dynanav-explore/references/native-interface.md)。每次决策实际打开四向原图、局部/全域 XY/XZ/YZ/斜3D及高度图，记录原始源时间、偏差、依据和剩余盲区。没有用旧路线、旧坐标、USD几何或模拟真值选点。真值只在停车退出后由基础设施代理评分。

新增 `observe-multicamera` 将原SOP快照与全部独立RGB视角合并为一份只读观察。它保留每路图像源时间与 post-VIO 的偏差，不偷偷改时间，不把点云多投影图冒充多台RGB，也不新增goto准入门。

启动器当前默认 PCT/SCAN，但本轮启动检查时该默认缓存目录不存在。本轮明确设置 `NAVIGATION_BACKEND=native_route`，沿用已有实跑的直线 RouteTracker，由 SOL 选择当前传感证据支持的小段目标。未把它冒称 PCT/SCAN 或多层规划通过。

## 在线探索范围与结果

原运行目录：

`/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/multicamera_sol_exploration_baseline_20261112`

数据与分析工作目录（下文简称 EVIDENCE）：

`/home/hyh001/Documents/Codex/2026-11-12/task/multicamera_explore`

- domain 230，预算600仿真秒，主动停止后实际475.2s，95,040个物理步；命令上限1m/s。
- 四台512×512理想pinhole，120°水平视场，朝向0/90/180/270°，前相机复用主渲染产品。原生内参逐帧校验，前端fx/fy≈147.801669，与相机标定一致。
- 每路接受4,751帧，归档475对lossless PNG/轴向深度；共19,004个接受帧、1,900对归档。无无效绑定帧、采集错误或残留队列。
- 10次waypoint_accepted；**到达3/6/7/8/9/10，失败1/2/4/5**，失败原因均为 `corner_brake_timeout`。没有把近目标刹停或CLI超时算成到达，也没有盲重试原seq。
- 明确stop seq11并收到匹配run_id/seq的`stop_accepted`；最终`navigation_terminal_and_parked`，capture exit0，bag/bridge/collector/launcher均退出0。
- 纯post-VIO 5.9..469.9s轨迹约51.515m（含抖动），净位移0.971m。净位移不是闭环误差，因为最终目标并非精确起点。
- 事后native诊断：4s后水平速度峰值约1.048m/s，p95约0.936m/s；采样未见机身接触，最大绝对roll/pitch约4.18°/9.43°。这些不是接触/支撑测量约定的独立认证。
- 探索了大厅多个方向、走廊前进与180°回访；楼梯、侧房、楼梯下区域和闭门后空间没有进入。不以轨迹长度、点云包络或高度图数量宣称全建筑覆盖。

`main/model_decisions.jsonl` 保留10次航点、stop和终审，共12条记录；`navigation_summary.json`、`final_execution_review.json`、`evidence/s01..s11_final/`保留查看和同seq闭合证据。

## Bag录制、完整性与回放

主bag：

`EVIDENCE/main/bag`

使用MCAP内置 `zstd_fast` 分块压缩，1GiB/120s分段设置，实际15段、16,078,764,198字节（约16.08GB），247,688条消息。录制在模拟器启动前已打开MCAP并确认进程存活；结束正常落盘，`ros2 bag info`退出0。录制接收时钟区间约 **0.035..475.215s**；`bag info`的1970日期来自相对仿真时钟，并非真实录制日期。

| 关键数据 | 主bag实际消息数 | 完整性说明 |
| --- | ---: | --- |
| IMU | 95,037 | 发布95,040；缺最初3个启动样本，之后无内部时间缺口 |
| 原始前RGB | 4,743 | 发布4,751；连续0.1..474.3s，缺最后停车尾段474.4..475.1s的8帧，未造帧补齐 |
| 原始RTX | 4,750 | 与发布数一致 |
| typed complete scan | 4,750 | 与发布数一致，保留证书及原始cloud |
| LIO odometry / post-VIO / measured full cloud | 各4,709 | 源时间4.3..475.1s，均连续10Hz |
| 四向PNG / depth / CameraInfo | 每相机每类475 | PNG字节及源时间与本代归档逐一完全一致 |
| TF、TF静态、标定、path/status、时钟 | 已录 | 详见bag_info与bag_audit |

新增 `multicamera-ros` 只发布本代已有原生归档：四向1Hz PNG、原始32FC1轴向深度和CameraInfo，不注入定位。`/tf_static`来自固定安装标定；全局`/tf`是原样复制测得LIO odometry供显示，非新估计器。原native命名空间TF也保留。`LIVO2/imu_propagate`和`cloud_visual_sub_map_before`没有实际消息；前者未启用，后者本轮未发出，并非被悄悄遗漏。

第一次best-effort录制预检存在原始RGB/扫描丢包，原目录保留；随后为本任务显式启用可靠writer与录制QoS，静止复测获得RGB199/199、扫描/complete各198/198。默认源QoS不变。主bag的少量首尾缺口按上表披露，所有运动区间未见内部缺口。

独立domain229实测主bag前8秒的 **2×播放**：四路各8张PNG、前RGB79、complete78、LIO/full-cloud各37、TF和标定均收到，player exit0。正确区分“2×查看可用”和“2×重新运行整个定位器成功”。

```sh
# 在没有仿真/控制器的独立ROS域，source ROS及本项目生成的消息环境后：
ros2 bag info "$EVIDENCE/main/bag"
ros2 bag play "$EVIDENCE/main/bag" --rate 2 --clock 100 --exclude-topics /clock
rviz2 -d "$EVIDENCE/main/replay.rviz" --ros-args -p use_sim_time:=true
```

不同时播放已录/新生成两份clock。RViz配置已生成并通过YAML检查，但没有录制RViz屏幕，也未声称GUI已验收。另有已解码检查的475帧原生四向视频 `main/four_views_video/view_grid.mp4`，标注每路实际源时间，按名义1Hz归档播放，不补合成像素。

命令/回执并非ROS cmd_vel：原始 `interactive_waypoint_control.jsonl`、`receipts.jsonl` 和模型ledger完整保留。为回放辅助，另生成 **事后审计包** `main/annotations_bag`：23,760条原生控制tick、11条按“观察源时”标记的模型记录、4条自带原生源时的失败回执。它只有审计String话题，不是执行器输入。17条原生accept/reached/stop回执原本不带源时间，仍按seq留在原JSONL；未补造精确时刻。模型记录的观察时间不冒充提交/完成时间。`annotations_manifest.json`明确这些差异。

## 本次点云/轨迹漂移观察

停车后才读取隔离的native真值流。采用第一次配对位姿的**单个固定SE(3)**对齐，不做全轨迹拟合或尺度拟合；按记录的IMU安装偏移将根姿态换算到IMU点。它依赖当前仿真读回/安装约定，不能外推实体机器人精度。

| 475.2s在线基线 | LIO | post-VIO |
| --- | ---: | ---: |
| 位置ATE RMS | 7.94mm | **8.75mm** |
| 位置p95 | 15.88mm | 17.06mm |
| 最大位置误差 | 33.82mm | 36.29mm |
| 最后位置误差 | 17.02mm | 17.64mm |
| 姿态RMS | 0.140° | 0.139° |

本轮大厅/走廊运动没有看到明显失控漂移，但存在毫米至厘米级偏差；这不是“已证明无漂移”。没有用SLAM估计自身重合来替代真值评分。

另对实际12.1s和450.7s的两份配对LIO扫描，在同一map系的显式大厅ROI比较，**没有ICP或额外对齐**。局部地面拟合在map(2,0)处高度差约1.39mm，法向差约0.121°；只能说明该局部重复观测相近。两次位姿/束采样/遮挡不同，最近点距离p50约6.9cm、p95约24.8cm不能直接称为漂移。局部平面、轨迹评分均不等于全地图精度。全域/局部3D视图、`drift_scoring.png`和`return_cloud_overlay.png`保留供核看。

## 新增原生四路更新的具体行为

开关：`FASTLIVO_MULTICAMERA_MODE=native_archive_marginal_ci_v1`；默认未设置或`off`保留原单前路径。明确要求 `FASTLIVO_MULTICAMERA_ARCHIVE` 指向已完成且同代的四图归档，`FASTLIVO_MULTICAMERA_LOG`为新文件。源/标定混用、缺失文件或前RGB像素不一致会失败关闭；联合有效观测不足或数值检查失败则回退原生单前更新。未改goto门、速度控制或容量保护。

- 原生VIO增加“提取测量因子/共同更新后提交参考帧”接口；四路有独立相机、图像、patch/视觉地图缓存和曝光状态。
- 检查四路安装、内参、原始整数时间；camera_0 PNG与当前原生前RGB包逐像素相同。四路共享同一次LIO点/查询树，没有先角度切点云。
- 从native19D状态取18D物理**边缘分布Pxx**，不是条件化减去曝光交叉项；每图曝光使用独立N(1,0.1)显式模型，未把0.1当已校准传感器不确定度。
- 每路曝光先在其光度因子中边缘化，再以权重和为1的物理likelihood池（各1/4）叠加到**一次**共同先验。不是把四个含同一先验的后验当独立相加。相关复制观测不应乘四提高物理置信度，有解析测试；这并不等于未知地图-状态相关性已经被完整建模。
- 仅计最细层一次公共先验线性化，不把重复金字塔像素当独立观测；物理更新含18D交叉协方差，并对旋转的新局部切空间作右Jacobian协方差变换，重建每相机曝光交叉项。
- 检查因子PSD、先验/后验SPD、有限值、曝光及数值步长范围；数值检查不是统计一致性认证。

该路径在原生handleVIO里提交 `_state`，沿原post-VIO发布及后续LIO运行，属于**真实原生状态更新实验**，不是把旧旁路的6D位姿结果改个名称；但目前输入是完成后的1Hz归档。延后归档图像不能追插已经结束的实时VIO时刻，实时四路10Hz同步仍是未完成项。

## 相同运动输入的单路/四路A/B

最终有效对照是 `motion180_matched2_off` 与 `motion180_joint`，同一核心SHA、同包、1×、前180s。两组实际收到 **IMU36,000 / 原始图像1,799 / complete scan1,798**；三类完整时间戳序列SHA256逐项一致，post-VIO各1,757个相同源时。首次四路更新前，最大位姿分量差仅约3.6e-15。未拿丢输入的早期off尝试作精度基线。

| 4.3..179.9s post-VIO | 单前 | 四路实验 |
| --- | ---: | ---: |
| 位置ATE RMS | **5.44mm** | 5.50mm |
| 位置p95 | **8.57mm** | 10.98mm |
| 最大位置误差 | 35.94mm | **28.74mm** |
| 最后位置误差 | **5.87mm** | 11.87mm |
| 姿态RMS | **0.1177°** | 0.1206° |
| 10s相对位移RMS | 6.91mm | **6.46mm** |

同源179.9s的两组原生registered cloud均有31,505点，未作ICP/额外对齐，按输出索引比较的差值p50约4.45mm、p95约7.01mm、最大约9.58mm；这是同扫不同状态更新的输出差，不是对真值的点云精度，也未独立认证点ID。`native_cloud_comparison/`保留原始两云及图。

四路175组中173组共同提交，2组有效因子不足回退；有效像素项累计约638,912/404,480/296,192/464,384。时间偏差0ns；所有记录的native完整协方差均数值正定，联合物理最小特征值约3.79e-10。

**没有稳定收益结论。** RMS差很小，p95和终点更差，最大误差与相对误差有所改善；单次局部场景不能给出精度优势。原生内部6D位姿协方差的NEES-like误差二次型中位数，单路约4,828、四路约4,497，远大于名义6D95%阈值12.59；两组仅约0.46%样本落在该阈值内。初始gauge对齐不确定性、时间相关样本及地图-状态交叉协方差未建模，因此这不是正式统计检验，但足以阻止把“SPD”说成“可信协方差”。没有用ROS占位covariance，也没有事后调占位噪声让它假通过。

原生进程抽样RSS高水位约766MiB（单路早期off尝试）和960MiB（四路）；观测累计CPU约63.3/67.7s。资源样本的单路不是最终matched-off，且启动段有缺口，**只作描述，不作严格因果性能比较**。180s播放墙时由播放器节奏支配，不能视作求解速度。

全程2×原生重估在源360.9s触发 `LiDAR buffered source capacity`，原native队列保护未放宽；失败日志保留。故只报告完成的1×180s对照，不声称整包2×定位通过。早期静止重放收包不全、启动目录/依赖预检拒绝、播放器参数拒绝以及初版off启动缺头的记录也保留；最终有效对照采用可靠接收和暂停/匹配后恢复播放。

## 交付、验证与下一步

主要证据位于EVIDENCE：

- `final_summary.json`、`controller_model.json`、`main/model_decisions.jsonl`、`main/navigation_summary.json`。
- `main/bag/`、`bag_info.txt`、`audit/bag_audit.json`、`playback2x/playback.json`及`annotations_bag/`。
- `main/four_views_video/view_grid.mp4`、`main/replay.rviz`、`main/drift/`、`main/return_clouds/`。
- `motion_input_epoch_comparison.json`、`motion180_matched2_off/`、`motion180_joint/`、`joint_build_manifest.json`、`joint_math_tests.log`。
- `.cpp/.h`精确构建输入、增量基线备份、录制/重放/评分脚本及所有失败代均保留。此报告不评估其它任务目录的实验。

代码新增 `observe-multicamera`、`multicamera-ros` 两个项目入口，以及默认关闭的原生因子/更新接口；窄补丁为 `0010-experimental-native-multicamera-update.patch`，保留以前来源修订。64项相关CPU单测通过（16多相机、29绑定、7RTX源、12入口）；6项原生数学测试及项目CMake/ctest入口通过；315个来源文件校验、视频解码与diff格式检查通过。

下一步需要同步发布和缓冲实时四路图像组，确保不会追插过期时刻；在更多运动/遮挡/纹理与退化场景检验像素、地图及状态相关性的噪声模型，并做整包及实时资源回归。**本任务未切换在线生产默认、未认证四路统计一致性或完整覆盖，未推送Git。** 本任务拥有的仿真、估计器和录制进程均已退出。
