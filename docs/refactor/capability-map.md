# 能力地图与精简计划

## 0. 先回答：核心是不是“仿真 + 感知 + 控制”？

大方向对，但三层不够，建议拆成 **7 个运行层，加 2 个支撑层**。理由都来自盘点。

- **定位建图应单独成层。** 它是体量最大、四条路径都在用的能力：FAST-LIVO2 前端、先验地图修正、操作员地图，合计约 35k 行，其中约 17k 是 third_party。它既不是“感知适配”，也不是“控制”。A 和 B/C 现在各有一整套定位栈，必须作为一层统一处理。
- **规划应单独成层，不放进控制侧。** 你说的“全局+局部导航路线”，实际是 PCT 全局规划、SCAN 局部规划和路径准入。SCAN 是独立进程，准入（pid_admission*、physical_gate）也是规划语义。控制只负责把路径跟踪成速度。
- **机器人执行层应单独成层。** 它包括运动策略（Student/MoE-CTS）、50 Hz 关节环、actor 进程和执行器插件。控制层的输出到 cmd_vel 为止，后面由这一层接手。现在 actor 有 3 份、关节环 2 份、执行器插件 2 份，需要单独收敛。
- **agent 接口应单独成层。** 包括目标下发、运行中干预和操作员观察。它现在散落在文件 mailbox、mission_runtime 和 skill 脚本里，计划统一改成 ROS2 action/service + MCP。
- **两个支撑层**：infra（启动编排、配置、DDS，以及新的 Tracer）和 tooling（离线评估、可视化）。

修正后的分层：

| 层 | 职责边界 |
|---|---|
| sim 仿真 | 场景资产、传感器仿真、物理步进、仿真到 ROS 的桥 |
| robot 机器人执行 | 速度命令 → 策略 → 关节目标 → 执行器；仿真安全门 |
| perception 感知 | 传感器消息适配（自车过滤、深度转点云、去畸变配对）、传感器健康门 |
| localization 定位建图 | SLAM 前端、先验地图修正、地图保存、操作员/语义地图 |
| planning 规划 | PCT 全局、SCAN 局部、参考发布、路径准入 |
| control 控制 | 路线跟踪、停-转-稳、末端停车、运行期安全保护 |
| agent 接口 | 目标与干预服务、观察查询（MCP） |
| infra / tooling | 启动、配置、DDS、Tracer / 离线工具 |

## 1. 能力清单（按层）

路径缩写：A=Gazebo 主线 start.py→demo.launch；B=Building launch/building.py；C=RoboGauge benchmark.py；D=DynaNav Isaac run_dynanav_native_exploration.sh。行数为约数。

### sim 仿真
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| Isaac 启动与场景 | SimulationApp、USD、碰撞修复、生成 Go2 | dynanav/capture/scene.py、session_collision.py | D | 900 | simplify | 碰撞修复应改到离线资产步骤做 |
| Gazebo 世界生成 | world.sdf、bridge yaml、frames.urdf | simulation/world/prepare.py、gazebo/robogauge/world.py、building.py:prepare | A/B/C | 1100 | simplify | 3 份（A/B/C），B 与 C 的 prepare 近乎逐行相同 |
| 传感器仿真 | RTX LiDAR、RGB、IMU；多相机/近场只写盘 | capture/lidar.py、sensors.py、multicamera*.py、nearfield_sensor.py | D | 1800 | simplify；多相机/近场 delete | 采样参数又被 perception/sensors/sampling.py 二次改写 |
| 仿真-ROS 桥 | 发布 /clock /imu /lidar /camera，订阅命令 | capture/ros_interface.py、frames.py、capture_go2_rgbd.py | D | 1700 | rewrite | 主循环混了 5 层职责 |
| 执行器插件 | 唯一的关节力矩写入方，socket 协议 | actuators/teacher_actuator.cpp、go2hv_actuator.cpp | A/B/C | 864 | 保留一份，删 go2hv | 两份约 95% 相同 |
| 地形高度与安全事实 | SDF/网格求地面高度，PhysicalSafetyState | native/observation.py(TerrainHeightMap)、mesh_terrain.py、safety_state.py、control/safety/simulation_gate.py | A/B/C | 640 | keep，挪到 sim 层 | — |
| 动态行人 | 骨骼行人和胶囊碰撞体 | dynanav/pedestrian.py | D（仅实验） | 521 | 待决定 | — |
| 运行时地形切换 | SLAM 到达后切换高度图 | simulation/terrain/* | 无 | 715 | delete | — |

### robot 机器人执行
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| actor 适配器 | 读命令、驱动策略、ACK、接触 | policy/worker.py、simulation/truth_worker.py、gazebo/robogauge/worker.py | A/B/C | 826 | rewrite（合一） | 3 份 |
| 关节目标环 | request64 → 观测 → 推理 → response16 | policy/native/worker.py、gazebo/robogauge/native_loop.py | A/B/C | 750 | rewrite（合一） | 2 份逐行相同；D 另有 Isaac 版 |
| Student45 策略与速度 PID | GRU 推理、COM 修正 | policy/student.py、speed_tiers.py | A/B/D | 989 | rewrite（核心约 200 行） | 一半是 4 个 helper 文件缺失的死扩展 |
| MoE-CTS 策略 | RoboGauge 策略运行时 | robogauge/runtime.py、contract.py | C/D | 175 | keep | 缺统一 Policy 接口 |
| 下层 ACK | 回报实际施加的命令 | lower_loop_ack.py | A/B/C | 561 | rewrite → applied_cmd 消息 | — |
| Isaac 策略 tick | Isaac 内策略推理 | capture/policy.py、physics.py | D | 582 | simplify | 与 robot/unitree 重复 |

### perception 感知
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| LiDAR 自车过滤 | 剔除机体包络内的点 | perception/sensors/self_echo_filter.py、localization_mapping/building_sensors.py、depth.py 机体盒 | A/B/C/D | 550 | simplify（合一） | 3–4 份，包络参数互不一致 |
| 传感器健康门 | 预热、恢复、新鲜度，发布 ready/hold | sensor_gate.py | A | 634 | simplify | 新鲜度检查散在 ≥3 处 |
| 深度转点云 | RGB-D 反投影 | perception/depth.py | D（rgbd） | 115 | keep | — |
| 去畸变点云/位姿配对 | 话题角色、整数纳秒配对 | rtx_deskewed_perception.py 前约 270 行 | D | 270 | simplify | 客户端和 worker 又各包一层 |
| 方向净空 perception.json | 净空、近场融合 | rtx_deskewed 后半、depth_safety_fusion、async_depth_fusion、capture/perception.py | 无消费者 | 2200 | delete | 与 rtx 版重复 |
| IMU/RGB 中继、async writer | 透传与 QoS 自证 | sensor_relay.py、async_writers.py | A | 600 | delete（改用静态 XML） | 两套切 async 的机制 |
| 学习式行人链 | Mask R-CNN 文件 IPC、跟踪 | perception/person_* 等 7 个文件、learned_mapping、learned_scoring、run_dynanav_person_worker | D（实验） | 约 4500 | 待决定（建议删） | 与 oracle/semantic 重叠 |

### localization 定位建图
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| SLAM 前端 | FAST-LIVO2 LIO/LIVO | third_party/fast_livo2、vikit；启动在 components.py、building.py、benchmark.py、run_dynanav_lio.sh | A/B/C/D | 23k | simplify（剥离诊断头） | 启动命令 4 份；src/frontend 下有副本 |
| 里程计适配 | IMU 参考 → body/lidar odom | odom_adapter.py、geometry.py | A | 333 | simplify | B/C 在 building_runtime 里自己转换 |
| 先验地图修正 | map→odom 修正，多次确认 + TTL | A：localization_runtime、known_scene_matcher、sensor_snapshot、reference_builder；B/C：backend/run、worker、tracking、registration、correction | A/B/C | 3800 | simplify→统一 | 两整套独立实现 |
| 回环与图优化 | keyframe 回环、SE3 图求解 | backend worker 回环段、history、native_graph、graph_solver.cpp | B/C | 600 | delete（生产无效果） | — |
| 地图保存 | 累积并落盘 | mapping/map_archive.py、History.save | A | 400 | rewrite（做成先验地图闭环） | 3 套累积器，都没有消费方 |
| 操作员占据/frontier 地图 | 给 agent 选点 | operator_map.py、explore_waypoint_client map | D | 4900 | rewrite（数百行） | 与 measured_tomogram、客户端内 OccupancyGrid 重复 |
| 语义分层地图 | 静态/动态/未知分层，文本查询 | semantic_mapping/* | D | 875 | keep（随 D 决定） | — |
| 消息身份证据链 | sha 身份证明 | point_provenance.py、known_scene_adapter.py | A | 1043 | delete | — |

### planning 规划
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| SCAN 托管 | 启动/重启子进程、就绪判定 | scan_epoch_supervisor、child_startup、scan_child_consumption、scan_runtime_parameters | A | 1100 | rewrite | SCAN 启动和参数共 4 份（A/B/C/D） |
| SCAN 本体 | B 样条局部规划 | third_party/scan | A/B/C/D | 13.5k | simplify（删 r74/r107 探针） | — |
| PCT 全局规划 | tomogram → 全局路线 | pct/measured_tomogram.py、generate_routes.py、third_party/pct | D（在线）、A（离线） | 1480 | simplify/rewrite（合成一个适配器） | 2 份 pct_map 序列化 |
| 全局 → 局部桥 | 发参考、截后缀、重规划请求 | reference/automatic_reference_runtime、route_suffix、scan_reference_protocol；D 的 pct_scan_worker | A/D | 1390 | simplify；D 侧 rewrite | 2 套 |
| SCAN 输出关联 | Bspline 与参考对齐 | reference_protocol.py（保留）、trajectory_contract.py（删） | A | 996 | simplify | payload/key 有 3 份 |
| 路径准入 | 走廊包含、静态物理证书 | path_admission、physical_gate、static_geometry、validate_route_support、route_fence | A | 3200 | simplify | 走廊概念 3 处实现 |
| 异步准入 | 证书计算放进子进程 | async_admission、geometry_worker | A | 1712 | rewrite | — |
| 准入生命周期 | 接受/保留/替换 | navigation/pid_admission*.py、admission/path_handoff、pending_path_commit | A | 2100 | rewrite（挪进 planning） | — |
| 探索候选 | 前沿 + 信息增益排序 | exploration/waypoint_candidates、route_information | D（人工） | 584 | 待决定 | 与 exploration_waypoints 重叠 |

### control 控制
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| 导航任务状态机 | 相位、到点、超时 | base_node、ros_node、pid_* mixin；building_navigation；route_tracker；task.py | A/B/C/D | 6000+ | rewrite（单一参数化节点） | 4 套（另有 1 套死的） |
| 路线跟踪控制律 | 位姿 + 路径 → [vx,vy,wz] | tracking/cascade_core、control_core、pid_core；relative_pid | A | 2300 | rewrite | 4 套；wrap 有 3 份 |
| 停-转-稳 | 转向门 | parking/transition_gate、navigation_stop_window | A | 2360 | rewrite（约 200 行） | HeadingGate 与 TeacherHeadingGate 重复 |
| 末端停车 | 捕获、减速、零保持 | ordinary_parking、parking_pose10hz、terminal_owner、terminal_reference、source_hold、parking_contract | A | 1500 | rewrite（一个状态机） | 停车常量有 4 份 |
| 零命令证明链 | sha 串联零命令 | parking_zero_contract、executor_command、duplicate_source、causal_join | A | 1700 | rewrite→stop 标志 + 序号；后两者 delete | — |
| 命令限幅 | slew、起步约束 | pid_core.slew、limit_acceleration、launch_command_policy | A | 200 | simplify（合一） | 5 处 |
| 运行期保护 | 时钟、IMU 倾角、障碍、bridge 否决 | clock_hold、imu_causal_buffer、imu_mailbox、bridge_safety_handoff、clear_guard_window | A | 1560 | rewrite | IMU 同步应归感知 |
| 速度安全门 | cmd_vel → actor 命令文件 | robot/unitree/bridge.py | A | 1068 | rewrite（挪到 control） | 与 executor_command 重复校验 |
| 控制端定位输入 | 接收 bundle、切换 generation | known_scene_inputs、known_scene_hold_guard、feedback_mailbox、ready_commit | A | 2500 | rewrite | 放错层 |
| D 进程内控制 | 50 Hz 跟踪 | capture/control.py、pct_scan_waypoints、slam_route、dynamic_hold | D | 1760 | rewrite/delete | 与 A 重复 |
| 楼宇地形辅助 | 楼梯限速、台阶过滤 | terrain_speed、planned_step_filter、route_progress | B/C | 221 | keep | — |

### agent 接口
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| 目标下发 | 启动时冻结 request/anchor | navigation/mission/mission_runtime.py | A | 373 | rewrite → action | 运行中不能干预 |
| 航点干预（mailbox） | goto/cancel/stop 文件协议 | native_waypoints、explore_waypoint_client（命令部分）、dynanav_sop、dynanav_exploration.py、SKILL.md | D | 约 1500 | rewrite → ROS2 action/service + MCP | 3 层包装 |
| 旧交互追踪器 | 早期 mailbox 追踪 | exploration_waypoints.py | 仅测试 | 2600 | 待决定（证据表明已死） | 被 native/pct 取代 |
| 命令文件 | 准入 robogauge 命令 | robogauge/commands.py、LatestCommandWriter | C/A | 280 | 随服务化重写 | — |
| 场景查询 | 语义地图文本上下文 | semantic_mapping/query.py | 离线 | 130 | keep（MCP 候选） | — |

### infra / tooling
| 能力 | 做什么 | 实现文件 | 路径 | 行数 | 判定 | 重复实现 |
|---|---|---|---|---|---|---|
| 进程编排 | 锁、环境、启动、关停 | launch/main.py、building.py、robogauge/benchmark.py、run_dynanav_*.sh、run_dynanav_online_person.py | A/B/C/D | 2900 | 统一重写 | 5 份 |
| run 目录物化 | 世界、外参、参数 | launch/prepare.py、runtime_context | A | 280 | simplify（去掉 _pin） | B/C 手写 |
| 配置 | 合并 JSON | common/config.py、configs/* | 全部 | — | simplify（约 65 个键零读取） | — |
| executor 调度 | R74/R90/R91/R114 | common/time/* | A | 714 | 待决定（建议换标准 executor） | — |
| 假诊断体系 | 空探针、空记录器 | observation.py、runtime_telemetry.py、input_freshness.py、writer_qos.cpp | A/B | 420 | delete，Tracer 重写 | — |
| 私有 Fast DDS | SHM 心跳补丁 | docker/private_dds、third_party/fastdds、dds_environment | A/B/C | 1100 | keep，日后评估 | D 用系统 DDS |
| 离线证据/回放 | 审计、回放、重建、渲染 | scripts/audit_*、replay_*、reconstruct_*、dynanav_exploration/*、archive_20261111 | 无 | 约 9000 | delete；summarize 按 Tracer 重写 | 渲染有 3–4 份 |

## 2. 按层 × 判定的行数

| 层 | keep | simplify | delete | rewrite | 合计 |
|---|---:|---:|---:|---:|---:|
| sim | 3476 | 7666 | 1976 | 1116 | 14234 |
| robot | 242 | 1157 | 1012 | 3276 | 5687 |
| perception | 242 | 21727 | 4936 | 1531 | 28436 |
| localization | 7986 | 20729 | 2439 | 4180 | 35334 |
| planning | 22749 | 21022 | 662 | 3858 | 48291 |
| control | 1073 | 2518 | 7099 | 15100 | 25790 |
| agent | 180 | 0 | 2667 | 4276 | 7123 |
| infra | 2120 | 3412 | 1343 | 3579 | 10454 |
| tooling | 361 | 1448 | 8301 | 932 | 11042 |
| **合计** | **38429（20.6%）** | **79679（42.7%）** | **30435（16.3%）** | **37848（20.3%）** | **186391** |

读表时注意：
- **third_party 和路线 JSON 占了大头。** simplify 和 keep 的大数主要来自 third_party 与数据文件，且都是约数：livox 约 19k（perception）、fast_livo2 约 17k（localization）、scan 约 13.5k（planning）、vikit 约 6.4k，以及 configs/robogauge/routes 约 20.7k 行 JSON（planning keep）。扣除后，项目自有 Python 中 delete 和 rewrite 合计约占一半以上。
- **control 层 rewrite 15100 是最大一块。** 它就是 A 路径的控制栈（base_node/pid_*/cascade/transition_gate/parking），也是诊断放行语义集中的地方。
- **delete 约 3 万行，可以先做。** 这部分的风险集中在“零调用方”和“开关恒定”这两类判断是否准确。

## 3. 执行路径

| 路径 | 依赖的能力 | 使用证据 |
|---|---|---|
| A Gazebo 主线 | 世界生成、teacher_actuator、actor+关节环+Student、bridge 安全门、FAST-LIVO2+自车过滤+relay+健康门+odom_adapter+known_scene 修正、SCAN 托管+参考+准入+异步准入、pid 控制栈+cascade+转向门+停车+安全、mission_runtime、common/time executor、私有 DDS | README/compose/Dockerfile 入口；全局路线是冻结文件，没有在线全局规划，也没有运行中 agent 干预 |
| B Building | building.py（prepare+编排）、truth_worker+关节环、building_sensors、FAST-LIVO2+backend PriorTracker、building_runtime、building_navigation、直接 Popen SCAN、process_resources | 只有 tests/unit/test_building_entry_closure.py 和人工 CLI |
| C RoboGauge | benchmark.py、robogauge/world.py、go2hv_actuator、robogauge worker+native_loop+MoE-CTS、building_sensors、FAST-LIVO2+backend、building_runtime、building_navigation、SCAN | 有回归测试 test_robogauge_benchmark |
| D DynaNav | capture_go2_rgbd+experiment、RTX LiDAR、RoboGauge 策略、run_dynanav_lio.sh、pct_scan_worker（PCT+SCAN）、pct_scan_waypoints/native_waypoints+route_tracker、operator_map+explore_waypoint_client、agent skill | 现役探索主脚本；唯一同时具备在线全局规划和 agent 接口的路径 |

**需要你决定的路径问题（这里不替你决定）：**
1. **B 是否放弃？** B 和 C 共用 building_navigation、backend、building_runtime，两边的 prepare 几乎逐行重复，而 B 只有测试和人工入口。放弃 B 可以直接去掉 building.py（684 行）、truth_worker、process_resources 和 assets/building 下的重复数据。
2. **A 和 D 以哪条为主线？** A 的控制和安全栈完整，但路线冻结、没有 agent 接口；D 有在线 PCT 和 agent 接口，但控制跑在仿真进程里、走文件 mailbox。这个选择决定阶段 3 统一控制节点的基线。
3. **C 是否只作为基准保留？** 如果保留，要和 A 共用 actor、执行器插件和启动编排。
4. **D 的 rgbd/行人支线（run_dynanav_online_person.py，11-09 之后没有更新）是否正式放弃？**
5. **relative-slam-navigation 这个 console script** 不属于 A–D 任何一条路径，计划删除，请确认。

## 4. 先删后写：分阶段计划

### 阶段 1：只删除

每个包单独提交、单独验收。验收标准是：A 能跑通一次并对照命令序列，B/C 回归测试通过，D 能启动；同时同步删除对应的测试。**前置（P0）**：A 路径的 cascade_core、transition_gate、ordinary_parking、clock_hold 都没有单测，先录一段 A 的命令序列和状态序列，作为后续对照基线。

| 包 | 内容 | 主要文件 | 约行数 | 前置条件 |
|---|---|---|---:|---|
| **D1 零调用方死代码** | 孤立入口、未实例化的模块、副本目录、零读取配置 | known_scene_adapter、sensors/、evaluation/、navigation/task.py、relative_slam_node、relative_odometry、control/relative_pid、collision、control/__init__、planning/route.py、planning/__init__、mission/state_machine、route_regions、mission46_guard、simulation/terrain/*、student_xy_stop_takeover_ack、robogauge/cpu_fixture、localization_mapping/frontend/*（副本+multicamera 影子）、person_mask_propagation；configs 零读取文件 9 个、docker/Dockerfile.native；pyproject 的 relative-slam 入口 | 约 4,700 | P0；同步修改 lower_loop_ack 的 import、policy/worker 对 terrain 的条件 import、tests/native/dynanav_lio/CMakeLists |
| **D2 诊断/探针/证据/_r47** | 全部 _observer_api 和诊断放行语义、空证据、计时 | common/observation.py（先内联透传，diag.lock 改成 self.lock）、约 30 个文件的 rNN 探针；放行分支：bridge.py:460-510、feedback_mailbox:756-805、physical_gate:472-583、scan_epoch_supervisor:431、transition_gate:1860-1872、ordinary_parking:243-330、duplicate_source、base_node _diagnostic_business_observed、pid_admission* 的 diagnostic_observe_only；pid_support.py（业务函数先搬走）、_r47 计时、publication_ledger、guard_audit、dynamic_stop_event、robogauge/analyze、writer_qos.cpp、process_resources、collision_bounds、render_diagnostics、experiment.py source_snapshot、SCAN 的 r74/r107 C++ 探针；scripts 离线审计/回放/重建/dynanav_exploration/archive_20261111、record_current_lio_cloud、archive-manifest | 约 4,500 src + 约 9,000 scripts | D1；注意 async_admission 里 evidence 写失败会触发 fault，要连同这条耦合一起去掉；runtime_telemetry.atomic（live status）保留到 Tracer 上线 |
| **D3 环境开关取消** | 按生产值固化开启的开关，删除关闭的分支 | 固化为开：R88、R103_EARLY_REQUEST_ACK、R103_EXHAUSTED、SCAN_SHORT_INTAKE、R94、R110、R92、R103_IMU、BRIDGE_SAFETY_INGRESS、R105、R119（并入 dds_transport.xml）、r74 时钟组、DYNANAV_RTX_COMPLETE_SCAN。删除：R116、R93、R89 分支、R104（causal_join 纯证据）、features 中 21 个零读取键、queue_budget、exhausted_replacement、replan_ack、metadata_qos、executor_selection、async_writers、ready_commit.selected。D 的 GO2_SPAWN/CAMERA_YAW/MULTICAMERA_OBSERVATION 等改为配置项 | 约 2,000 | D2；**不能按代码默认值 0 删分支**，否则 A 的行为会变；R90/R91/R114 见“待决定” |
| **D4 R85 旧路径与恒定配置死分支** | R85 关闭时的路径、r73 minimal 基线、恒假配置 | 先在入口加“R85 关闭就拒绝启动”的检查，再删：trajectory_contract、FrozenFence、非 R85 定时器、continuous_route 及其分支（V2 约 450 行）、input_freshness 的 minimal_baseline 分支（约 550 行）、scan_epoch_supervisor _minimal_*、corridor shadow（CorridorObserver）、matcher legacy 策略、student.py 4 个缺失 helper 的扩展和 expected_snapshot 残骸、actor teacher 死分支、static_geometry/validate_route_support 的 CLI | 约 3,500 | D3；**pending_path_commit 不删**（见 §6） |
| **D5 D 路径已放弃实验 + lockstep** | 用户已决定删除和证据确认已死的部分 | lockstep.py 及 capture_go2_rgbd:129-130/224-227/527-612、experiment.py:332-339/679-686；nearfield*、multicamera_sensor 写盘、depth_safety_fusion、async_depth_fusion、capture/perception.py、rtx_deskewed 后半约 800 行、frames 的 RTX 死分支、explore_waypoint_client 的 --perception-out 和融合参数（约 400 行）、rgb_archive、routes.py（碰撞路径先挪进场景配置）、scripts/capture_dynanav.py 及 Student/approved_cmd 分支、oracle_mapping、dynanav_rtx_deskew_adapter、forward_turn/swept_area 脚本 | 约 5,000 | D2、D3；exploration_waypoints、行人链、slam_route 不在本包（见“待决定”） |
| **D6 无效果的重复实现** | 生产上没有效果，或可以用静态配置替代 | backend 回环与图优化（worker 187-311、history 的回环/save、native_graph、graph_solver.cpp、CMake；building.py:601 和 benchmark.py:168 的 .so 检查，Dockerfile:49）、registration 的 Relocalizer（约半个文件）、external_map 分块、sensor_relay（FAST-LIVO 改订 /livox/imu 和 /demo/camera，sensor_gate.graph 同步修改）、self_echo_filter 的身份话题、odom_adapter 的 R60 身份链、point_provenance、fastdds 测试补丁、third_party 上游残留（avia、rviz、original_*.cpp） | 约 3,000 | D3（R119 已固化）；B/C 回归测试 |

### 阶段 2：精简（不改接口）
- **仿真**：physics/scene/sensors/lidar/ros_interface 去掉证据和 assert 堆；world/prepare 去掉旧地形和证据（目标约 150 行）；sampling 并入世界生成；Gazebo 执行器插件合一，电机模型改为 SDF 参数。
- **感知**：自车过滤合成一个库函数加一个节点（3–4 份合一，先统一包络参数）；sensor_gate 去掉死分支和写死的节点名；rtx_deskewed 只保留配对契约。
- **定位**：odom_adapter、localization_runtime（停车判定改订话题，不再读文件）、matcher、building_runtime、backend 去掉证据；SE3 工具合成一份。
- **规划**：reference_protocol 去掉逐字段校验；path_admission、physical_gate、static_geometry 合并 digest；measured_tomogram 改用 install/pct；SCAN 参数生成统一为一个函数。
- **机器人**：speed_tiers 退化为配置项；contract.json 只保留关节、布局和扫描网格。
- **控制**：control_core、parking_pose10hz、navigation_stop_window 去掉等值校验（safety_configuration 删除）；命令限幅器合一。
- **配置与基础设施**：configs 约 45 个零读取键；prepare 的 _pin（先修正 experiment 的错误值）；launch/main 的硬编码 FASTLIVO_* 和 3 把锁；third_party/fast_livo2 剥离诊断头；livox 缩为只含消息的包。

### 阶段 3：重写
| 能力 | 理由 |
|---|---|
| Tracer（infra） | 现在的 RuntimeTelemetry.append 是空操作，所有“记录”实际被丢弃。新 Tracer：每 tick 一行 jsonl、单调时钟、阶段起止和输入摘要、无返回值，用配置打开；替代 receipts.py 和散落各处的证据 |
| agent 控制接口 | 文件 mailbox（request.json/path.json/receipt）、mission_runtime 启动时一次性冻结、3 层脚本包装，都改成 ROS2 action/service + MCP；操作员 RGB 静帧改为话题或服务 |
| 统一导航控制节点 | 4 套状态机和跟踪律合成一个：输入接入、相位状态机（drive/turn/stop）、停-转-稳（约 200 行）、末端停车一个状态机、clock_hold（约 50 行）、bridge 安全门挪到 control；D 的控制从仿真进程移出 |
| 命令通道 | 零命令 sha 证明链和 lower_loop_ack 改成“序号 + 时间戳 + 限幅 + stop 标志”加 applied_cmd 回执 |
| 统一规划节点 | 常驻 SCAN lifecycle 节点（4 份启动合一）、一个 PCT 适配器、异步准入按“单 worker、单请求”重写、准入从 navigation 挪进 planning |
| 统一 actor | 一个 socket 关节环 + Policy 接口（Student/MoE-CTS）+ 一个命令源；修复 Teacher247 观测的 finite 性决定 Student 是否推理这个耦合 |
| 定位统一与地图保存 | A 和 B/C 两套定位合成“先验地图 + 先验 ICP + 确认器 + 修正发布节点”，进程间用 ROS 通信；地图保存要能作为下一次运行的先验；operator_map 与 measured_tomogram 合成一个实测地图 |
| sim bridge 与启动编排 | capture_go2_rgbd 只负责步进和发布传感器；experiment.py 改成 YAML 场景配置；5 份编排合成一个 launch 层，场景和 actor 作为参数 |

## 5. 待决定（你尚未拍板）
| 事项 | 证据 | 建议 |
|---|---|---|
| 主动刹车链 | integral_unload（483 行）、transition_gate 约 470 行 quiet/brake、cascade_core 约 183 行、lower_loop_ack:264/367/513、yaw_measurement_feedback；r85_active_turn_braking、quiet_handoff、neutral、taper 在配置里全为 false | 删除；但要保留 transition_gate._clear_quiet 和零速边沿的语义 |
| 旧交互追踪器 exploration_waypoints（2600 行） | src/scripts 中没有 import；只被测试和待删的 lockstep 依赖 | 删除，连同 3 个测试（约 3,700 行） |
| 放弃执行路径 B / 主线选 A 还是 D / C 的定位 | 见 §3 | — |
| 行人与动态场景支线 | person 链、learned_mapping、pedestrian、dynamic_hold、slam_route、build_dynanav_slam_route；与 rtx_lidar 互斥（experiment.py:103） | 如果近期不做动态场景，整条删除（约 7,000 行）；删 slam_route 前先把 _number/pose_transform 挪走 |
| 探索候选（waypoint_candidates/route_information） | 只能经人工 CLI 调用 | 如果保留，改为 agent 的查询服务 |
| operator_map 重写还是删除 | 取决于 agent 选点是否直接使用 PCT 的 tomogram | — |
| R90/R91/R114 executor | 当前全部开启，A 路径依赖；R91 会在运行时 AST 改写 rclpy | 换成标准 MultiThreadedExecutor + 独立 /clock 回调组，需要做 A 回归 |
| R106 source_hold | 开启时停车遇到拒绝选择保持而不是失败；删除会改变行为 | 停车状态机重写时重新定义这条语义 |
| 多相机能力 | multicamera.py 标定在 D 中可开，multicamera_sensor 只写盘 | 删写盘部分；标定是否保留由你决定 |
| 私有 Fast DDS | A/B/C 必需，D 不用 | 之后评估系统 DDS 是否够用 |

## 6. 矛盾与不确定点
1. **pending_path_commit**：docs/refactor 草稿把它归为 R85 关闭的旧路径，但 pid_admission_apply:430 在 R85 分支之前就无条件调用它，A 路径可以走到。先不删，要再核实。
2. **lower12.json**：configs/README 和 scenarios/README 都说它没有读取方，实际 simulation/settings.json 的 terrain_target_manifest 指向它，prepare 会写进 run。文档有误，不要删。
3. **统计口径**：harness 报告“dynanav-map、sim-robot、infra、scripts-vendor 未返回”，但盘点数据里有这些区域的条目。rgb_archive（95）、render_diagnostics（241）、dynanav/__init__（1）被两个区域重复统计，totals 大约多算 337 行。
4. **render_diagnostics 调用行号**：一个区域写 frames.py:74，另一个写 frames.py:373，需要核对。
5. **depth.py 判 keep**：依据是“D 默认 rgbd 每帧调用”，但生产启动器传的是 rtx_lidar，生产中实际走不到。是否保留取决于 rgbd 支线的决定。
6. **pedestrian.py**：一处判 simplify，同一处又说“可以整体删”，取决于行人支线的决定。
7. **multicamera**：dynanav-run 区域对 multicamera_sensor 判 delete，dynanav-map 区域对 multicamera.py 判 simplify，标定部分是否保留存在冲突。
8. **Teacher247 观测**：robot 区域说“只剩 D 需要”，但它在 A/B 的关节环里每帧都算，并且决定 Student 是否推理。删除前必须先解耦。
9. **自车过滤包络**：self_echo_filter 用躯干加腿两个盒，building_sensors/depth 用 0.5/0.3/-0.45/0.3 单盒。哪个正确未经验证，合并前要确认。
10. **层归属与目录不符**：统计用的是“实际归属层”，例如 feedback_mailbox、known_scene_inputs 归 control，terminal_heading_contract、handoff_stop_policy 归 planning，simulation_gate、contact_mailbox 归 sim，bridge.py 归 control。和目录不一致，删改时要注意。
11. **third_party 未验证项**：行数是估计值；vikit 实际用到哪些相机模型没有核实；SCAN 的 fresh_reference_protocol 和 fresh_child_odometry_ack 是否属于不必要校验，需要对照 docs/refactor/unneeded-validations-20261115.md 再看。
12. **build_dynanav_slam_route 判删**：前提是 D 的默认后端是 pct_scan/native_route。如果还要保留楼梯重放模式，需要复核。
13. **README 行数过时**：多份 README 记录的行数已经不对（例如 base_node 写 1617，实际 1820；transition_gate 写 1730，实际 2066），不能作为依据。
14. **测试覆盖**：A 路径的定位链（odom_adapter、localization_runtime、known_scene_*）和控制核心都没有单测。阶段 1 每个包的验收只能依赖 P0 录下的对照序列。
