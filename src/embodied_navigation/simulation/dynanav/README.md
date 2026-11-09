# simulation/dynanav

本目录负责办公室 DynaNav 场景、Go2 RGB-D/IMU 采集、显式楼梯实验与 oracle 分层地图适配。定位算法、语义地图数据结构和控制器仍属于各自模块。已完成的完整 GT 路线旁路 LIO/LIVO 对照与实际视觉闭环须分别看待，最新结果见[实验续篇](../../../../docs/experiments/dynanav-20261109-continuation.md)；先前失败保留于[历史报告](../../../../docs/experiments/dynanav-20261109.md)。

本目录是**隔离的 Isaac/DynaNav 实验路径**（`docs/architecture.md` 的四条执行路径 D），与默认主路径/RoboGauge benchmark 不共享进程图与 ROS 话题；产物写外部数据卷 `experiments/<run>/`，**不写 `runs/`**。

## 目录结构

```text
src/embodied_navigation/simulation/dynanav/
├── capture/                # 子模块，见 capture/README.md（采集实现包，11 个文件 2661 行）
├── README.md               # 本文件
├── __init__.py             # 1 行，包 docstring（场景集成；场景/纹理资产属外部数据）
├── capture_go2_rgbd.py     # 485 行，采集入口：argv/experiment 配置、编排、失败收据（实现全在 capture/）
├── collision_bounds.py     # 72 行，导出非机器人碰撞体包围盒 JSON（会话证据）
├── dynamic_hold.py         # 307 行，传感器坐标系内的行人 hold/resume 决策（实验专用）
├── experiment.py           # 531 行，显式 Isaac 诊断 CLI：profile/控制器、输出目录与源码快照
├── learned_mapping.py      # 488 行，在线 RGB 行人过滤 + exact post-VIO pending 配对与持久几何
├── learned_scoring.py      # 251 行，person-only renderer 身份评分 sink（只评、不回传标签）
├── oracle_mapping.py       # 259 行，仿真 mask/pose oracle 建图（诊断管道，不是 learned 感知）
├── pedestrian.py           # 521 行，可选本地骨骼行人 actor（只写在 live USD 会话）
├── render_diagnostics.py   # 241 行，只读 USD/Fabric 渲染状态收据（不刷新、不改 stage）
├── rgb_archive.py          # 95 行，叠字前原始 RGB 无损归档（ZIP_STORED NPZ）
├── routes.py               # 46 行，办公室路线表（首段/地下室到地面层）
├── session_collision.py    # 242 行，地下室碰撞会话修复 + PhysX 支撑射线 preflight
└── slam_route.py           # 657 行，实测估计位姿路线回放与停车（live LIO / post-VIO）
```

14 个顶层 Python 文件、合计 4196 行（最大 `slam_route.py` 657 行）；另有 `capture/` 包。没有 >800 行的文件，因此按 SPEC 口径没有"上帝文件"；400–657 行区间有 5 个文件，其中 4 个（`slam_route.py` 657、`experiment.py` 531、`pedestrian.py` 521、`learned_mapping.py` 488）已接近单文件多职责，第 5 个 `capture_go2_rgbd.py`（485 行）是编排薄入口（实现已拆到 `capture/`），不计入。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `capture_go2_rgbd.py` | 活代码，薄入口（485 行）：`parse_config` 读 argv/`DYNANAV_EXPERIMENT`，`run(argv)`/`main()` 是显式入口，`run_capture` 按 config→session→ROS→mapping→loop→receipts 顺序构造。普通 import 无副作用（不读 `sys.argv`、不建目录、不导入 Isaac/ROS）。 |
| 代码 | `experiment.py` | 活代码，`--check`/实跑双入口：校验模型与场景、把输出限制在 `PROJECT_DATA_ROOT/embodied-navigation` 之下且要求目录不存在、写 `experiment.json` + `source_snapshot/` + `source_manifest.json`，再设 `DYNANAV_EXPERIMENT`/`PROBE_STEPS`/`PROBE_RTF`/`NAV_STOP_REQUEST` 并 `runpy` 执行入口。 |
| 代码 | `slam_route.py`/`routes.py` | 活代码：`SlamRouteTracker` 消费实测估计位姿（`lio_estimate` 或 `livo_post_vio`）、做 0.5 s pose window 停车与终止判断；`routes.py` 只提供路线常量。 |
| 代码 | `oracle_mapping.py`/`learned_mapping.py`/`learned_scoring.py` | 活代码，三种建图/评分 sink，由 `capture/mapping.py` 按 `semantic_provider`/`oracle_evaluation` 选择构造；oracle 用 GT pose/mask 作诊断，learned 只用 exact post-VIO 与 RGB 证据，scoring 只评不回传。 |
| 代码 | `pedestrian.py`/`dynamic_hold.py`/`session_collision.py`/`collision_bounds.py` | 活代码，实验场景层：本地行人 actor、hold/resume 决策、地下室碰撞修复与支撑射线、碰撞体包围盒导出。 |
| 代码 | `rgb_archive.py`/`render_diagnostics.py`/`__init__.py` | 活代码/包标记：原始 RGB 归档、渲染状态收据、包 docstring（无重导出）。 |
| 日志/输出 | `<run>/capture/experiment.json`、`<run>/capture/source_manifest.json`、`<run>/capture/source_snapshot/` | `experiment.py` 写：本次 profile、40 个源码/配置文件的 sha256 与逐文件快照（含 `capture/*.py`、oracle/learned/scoring、perception 依赖、两份碰撞配置）。这是"这次运行到底跑了哪版代码"的身份证据。 |
| 日志/输出 | `<run>/capture/result.json`、`*.jsonl`、`mapping/`、`learned_scoring/`、`rgb_raw/`、`*.mp4` | 采集进程写（字段与文件清单见 [capture/README.md](capture/README.md)）；`result.json` 是主收据，JSONL 是逐帧/事件证据。 |
| 日志/输出 | `<run>/capture/stop_request.json` | `experiment.py` 把 `NAV_STOP_REQUEST` 指到这里；外部停车请求的落点。 |
| 日志/输出 | `<run>/logs/`、`<run>/supervision.json` | **不是本模块写的**：由 `scripts/run_dynanav_online_person.py` 监督器写（逐进程日志、schema v3 的 argv/pid/退出码/`unexpected_exits`）。本模块只产生被监督的进程，不能把 `supervision.json` 当导航/感知验收。 |
| 配置 | `configs/simulation/dynanav_basement_collision.json` | `session_collision.py:15` 直接读（`CONFIG_PATH = project_root()/…`），也列在 `experiment.py` 的源码快照里。 |
| 配置 | `configs/simulation/dynanav_basement_to_ground_collision.json` | `routes.py:33` 读路线点，`experiment.py` 快照里也有一份。 |
| 配置 | `configs/simulation/dynanav/{camera_rgbd,fastlivo_lio,fastlivo_livo}.yaml` | **本包不读**：由 `scripts/run_dynanav_lio.sh` 读取（`fastlivo_${mode}.yaml` + `camera_rgbd.yaml`）来启动外部 LIO/LIVO 观测器（见 [LIO/LIVO README](../../../../configs/simulation/dynanav/README_LIO.md)）。 |
| 配置 | `configs/simulation/dynanav/office.yaml`、`fastlivo_lio_gravity_estimation.yaml` | **本检出无读取点**（`grep -rn office.yaml src/ scripts/` 无命中）；不要把它们当成有效配置。 |

## 模块内文件耦合

- 依赖方向（无 import 环，向上依赖都藏在函数内）：`experiment → {routes, pedestrian, slam_route}`；`routes → session_collision`；`learned_mapping → slam_route`（只 import `pose_transform`）；`capture_go2_rgbd → capture/*`；`capture.mapping → {oracle_mapping, learned_mapping, learned_scoring, rgb_archive}`；`capture.control/policy → experiment`（函数内）；`capture.scene → {pedestrian, session_collision, collision_bounds}`；`capture.frames → render_diagnostics`。
- 与 `scripts/` 之间是"进程 + 环境变量 + sys.argv"契约：`run_dynanav_experiment.py` 调 `experiment.main`；`capture_dynanav.py` 直接 `runpy` 入口；`NAV_STOP_REQUEST`/`DYNANAV_EXPERIMENT`/`PROBE_STEPS`/`PROBE_RTF` 是字符串键。
- 无跨文件共享可变状态：每次 run 的 sink、Receipts、route tracker 都是实例级。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `slam_route.py` | 657 | `navigation.robogauge.route_tracker` | `experiment`、`learned_mapping` | 偏重：17 个函数 docstring 0.353，`tick` 184 行，>80 行函数 3 个 |
| `experiment.py` | 531 | `project`、robogauge runtime、`pedestrian`、`routes`、`slam_route` | `capture.control`/`capture.policy`（函数内）、`scripts/run_dynanav_experiment.py` | 偏重：`main` 387 行；源码清单 40 项硬编码 |
| `pedestrian.py` | 521 | `session_collision` | `experiment`、`capture.scene` | 偏重：11 个函数 docstring 0.273，2 个 >80 行 |
| `learned_mapping.py` | 488 | `semantic_mapping`(+query)、4 个 perception 模块、`slam_route` | `capture.mapping` | 偏重：8 个函数 **0 docstring**，3 个 >80 行 |
| `capture_go2_rgbd.py` | 485 | `capture/*`（10 个模块） | 无 import 方；被 `runpy` 执行 | 可接受：5 个函数 docstring 全有，`run_capture` 336 行 |
| `dynamic_hold.py` | 307 | 无内部依赖 | `capture.control` | 可接受：`update` 114 行有 docstring，其余 docstring 0.222 |
| `oracle_mapping.py` | 259 | `semantic_mapping` | `capture.mapping` | 需改进：5 个函数 0 docstring，`process` 140 行 |
| `learned_scoring.py` | 251 | 无内部依赖 | `capture.mapping` | 需改进：6 个函数 docstring 0.167 |
| `session_collision.py` | 242 | `project` | `capture.scene`、`pedestrian`、`routes` | 可接受：5 个函数 docstring 0.6，2 个 >80 行 |
| `render_diagnostics.py` | 241 | 无内部依赖 | `capture.frames` | 可接受：`capture_stage_receipt` 196 行但 docstring 有 |
| `rgb_archive.py` | 95 | 无内部依赖 | `capture.mapping` | 清楚：3 个函数 docstring 0.667 |
| `collision_bounds.py` | 72 | 无内部依赖 | `capture.scene` | 清楚：单函数 docstring 齐全 |
| `routes.py` | 46 | `project`、`session_collision` | `experiment` | 清楚：单函数 docstring 齐全 |
| `__init__.py` | 1 | 无 | 无 | 清楚 |

四类问题：**上帝文件无**（657 < 800）；**重复实现有**：`oracle_mapping.py` 与 `learned_mapping.py` 各有一套 `frames.jsonl`/`static_map.npz`/`receipt.json` 写出逻辑（有意分模式，但落盘代码相似）；**跨层直达有**：`learned_mapping` 直接 import 4 个 `perception/*` 模块与 `slam_route`，`capture.control/policy` 反向 import 父级 `experiment`；**隐式契约有**：`experiment.py` 的 `sources` 列表（新增文件必须手动加入，测试只守护 `capture/*.py`）、`DYNANAV_EXPERIMENT`/`NAV_STOP_REQUEST` 环境键、`capture_go2_rgbd` 的 `argv[1]`/`argv[2]` 位置参数、`pose_source` 字符串（`lio_estimate`/`livo_post_vio`）。

## 可读性评估

- `capture_go2_rgbd.py`、`rgb_archive.py`、`collision_bounds.py`、`routes.py`、`session_collision.py`、`dynamic_hold.py`、`render_diagnostics.py`：**可接受**。入口 5 个函数 docstring 全有、模块 docstring 说清拆分与"import 无副作用"；`routes`/`collision_bounds` 单函数 docstring 完整；`session_collision`/`render_diagnostics` 各有 1 个 >80 行函数但都有 docstring。
- `slam_route.py`、`experiment.py`、`pedestrian.py`：**需改进**。证据：`slam_route` docstring 覆盖率 0.353（17 个函数）且 3 个 >80 行（`tick` 184）；`experiment.main` 387 行把资产校验、profile 组装、源码快照、env 设置、`runpy` 串在一起；`pedestrian` 11 个函数 docstring 0.273、`create_pedestrian` 163 行。
- `learned_mapping.py`、`oracle_mapping.py`、`learned_scoring.py`：**需改进（文档）**。三个文件函数 docstring 分别为 0.0/0.0/0.167，而它们承载动态/静态语义与"不回传标签"这类关键约束——约束目前写在模块 docstring 与 README 里，函数级没有。结构不差（每个类一个 sink、依赖注入清楚），缺的是函数级契约说明。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `experiment.main`（387 行）拆成 `_parse_args`/`_validate_assets`/`_write_source_snapshot`/`_export_env`/`_run_capture` 五段，`sources` 列表改成模块常量 | 这是 DynaNav 唯一实验入口，改 profile 或快照清单时最容易碰坏别的段；`sources` 硬编码在 387 行函数体内 | 中：`--check` 输出与 source_manifest 是运行身份证据；`tests/unit/test_dynanav_source_snapshot.py` 直接 AST 解析 `experiment.py` 的 `sources` 赋值 | 先跑 `--check` 对照 JSON 字段；`python3 -B -m unittest tests.unit.test_dynanav_source_snapshot`；再跑一次 `--check` 比对 `experiment.json` |
| P1 | 给 `slam_route.SlamRouteTracker.tick`（184 行）按"位姿窗口/到达判定/停车/终止"拆方法 | 657 行文件里 3 个 >80 行函数，停车与终止条件是最需要审计的部分 | 中：路线控制是实测闭环的一部分，顺序改变会影响到达/停车时刻 | 先跑 `tests/unit/test_dynanav_slam_route.py`（6 处引用）；用已归档 run 的 pose 段做离线重放比对 `arrived`/停车时刻 |
| P2 | 给 `learned_mapping`/`oracle_mapping`/`learned_scoring` 的函数补 docstring（尤其是"哪条分支读什么位姿、标签是否回传"） | 三个文件 docstring ≈ 0，而语义约束正是审阅重点 | 无（文档） | 人工复核；`tests/unit/test_dynanav_learned_mapping.py`/`test_dynanav_learned_review.py` 仍通过 |
| P2 | 把 `experiment.sources` 的完整性检查扩展到顶层 `dynanav/*.py`（不只 `capture/*.py`） | 现在新增顶层模块不会被快照测试抓到，运行身份可能缺文件 | 低：只加测试断言 | 新增一个断言：`dynanav` 目录下所有被 `capture/*` 或 `experiment` import 的模块必须在 sources 里 |
| P2 | 统一 `--output` 的数据卷/不存在校验：`experiment.py` 与 `capture/mapping.open_rgb_archive` 各有一份路径检查 | 两处规则文字不同（一个校验运行目录、一个校验归档子目录），改动容易只改一处 | 低 | 两处对照检查同一组路径用例 |

没有 P0：真值边界在代码里是显式的（`navigation_ground_truth_used` 字段、oracle/learned 分支分离、learned 不读 renderer 标签或 GT pose），本轮没有发现把 GT 写成导航定位输入的路径；已知的感知质量问题（行人漏过率等）是实验结论，不是本目录代码缺陷。P1 是"入口与路线控制的可审阅性"，两者都有离线测试可依托。本轮只写建议、不改代码；任何改动都要先跑 CPU fixture 测试，且不能用 CPU 结果替代 Isaac/ROS 实跑。

## 入口与文件

- [capture_go2_rgbd.py](capture_go2_rgbd.py)：DynaNav RGB-D 采集入口，显式 `run(argv)`/`main()`；200 Hz 物理、50 Hz 策略，真实 articulation 相机/IMU，独立流 JSONL。实现按职责拆到 [capture/](capture/) 包（`receipts`/`scene`/`sensors`/`ros_interface`/`mapping`/`policy`/`control`/`physics`/`frames`/`video`），入口只负责 argv/experiment 配置、编排与失败收据（失败收据与资源清理逐项尽力完成，单个 cleanup 失败不替换 primary 异常）。普通 `import` 无副作用（不读 `sys.argv`、不建目录、不导入 Isaac/ROS、不写文件）；入口仍由 [experiment.py](experiment.py) 与 [capture_dynanav.py](../../../../scripts/capture_dynanav.py) 用 `runpy.run_module(..., run_name="__main__")` 执行，`sys.argv = ["capture_go2_rgbd", <output>, <scene>]`（`argv[1]` 输出目录、`argv[2]` 场景）与 `DYNANAV_EXPERIMENT` 契约不变；已移除无条件 periodic Python traceback dump，保留标量遥测。
- [experiment.py](experiment.py)、[routes.py](routes.py)：RoboGauge 楼梯 profile、首段或地下室到地面层路线、资产/模型检查与新输出管理。入口为 [run_dynanav_experiment.py](../../../../scripts/run_dynanav_experiment.py)。
- [slam_route.py](slam_route.py)：显式 live LIO 或最终 post-VIO 路线控制；[build_dynanav_slam_route.py](../../../../scripts/build_dynanav_slam_route.py) 从实际估计位姿构造参考。repeat-spawn、无全局配准假设须保留。
- [session_collision.py](session_collision.py)：地下室碰撞会话修复与实际 PhysX 支撑射线 preflight，不改原场景资产；[collision_bounds.py](collision_bounds.py) 导出非机器人碰撞体包围盒。
- [oracle_mapping.py](oracle_mapping.py)：同步深度点、像素实例分类与 GT 位姿接入 [semantic_mapping](../../localization_mapping/semantic_mapping/README.md)，输出即时局部诊断图、对象历史和结构化快照。它不是 learned 或 SLAM 图。
- [learned_mapping.py](learned_mapping.py)：显式RGB worker前端过滤、exact live post-VIO pending配对、独立重复几何候选与person对象context；不读取renderer标签或GT pose。
- [learned_scoring.py](learned_scoring.py)：可选person-only renderer身份评分sink，不向算法回传标签，也不提供全场动静真值；代表点和全部保留支持票分开评分。
- [rgb_archive.py](rgb_archive.py)：`--archive-rgb` 保存叠字前原始 RGB 与源时间，ZIP_STORED NPZ；完成时核对 acquired/published/archive/video 计数。
- [pedestrian.py](pedestrian.py)、[dynamic_hold.py](dynamic_hold.py)：可选 skeletal human 和 oracle cloud hold/resume；首段 live LIO 行进中单次穿越已测，重复穿越失败保留。
- [replay_dynanav_imu.py](../../../../scripts/replay_dynanav_imu.py)、[replay_dynanav_rgb.py](../../../../scripts/replay_dynanav_rgb.py)：只按收到的 `/clock` 原序发布真实 source stamp 的 SDK IMU 或原始 RGB；不生成时间、不从视频伪造图像、不读取 GT 位姿。
- [reconstruct_dynanav_slam_map.py](../../../../scripts/reconstruct_dynanav_slam_map.py)：用实际估计位姿与 exact 关键帧时间重建图；mask 仍 oracle。历史独立 LIO 图为 19,812 点、post-VIO 派生地图为 91,449 点；本轮live post-VIO地图92,846点（207exactKF、48跳过），均不代表完整办公室覆盖。
- [view_dynanav_map.py](../../../../scripts/view_dynanav_map.py)：只发布归档静态点云供 RViz，`--check` 不启动 ROS；无机器人 TF 或控制，见 [visualization README](../../../../configs/visualization/README.md)。
- [summarize_dynanav_experiment.py](../../../../scripts/summarize_dynanav_experiment.py)：只读计算实际非零命令时段的 native 速度、z 差、RGB-D 质量、到达与停车，不把进程退出 0 当导航成功。

## 控制路径与位姿来源

旧 [capture 入口](../../../../scripts/capture_dynanav.py) 保留 Student/COM 速度 PID、`/dynanav/nav/approved_cmd` 和 `/aft_mapped_to_init` 新鲜度门。默认 4400×0.005 s（约22 s），0.5 m/s 策略档位、0.65 m/s 物理速度断言、`MAX_DISPLACEMENT=0.25` m XY 终止条件保留；它是采集探针。

新 experiment 默认 RoboGauge 45D 本体策略和 Go2HV effort，GT native pose 驱动 RouteTracker，无 COM 速度 PID/slew，`ground_truth_navigation=true`。可给目标命令上界 1 m/s，没有按楼梯位置自动限速；倾倒、电机和时钟等物理保护仍会终止。新实验仅给关节读回比较加入 0.001 rad/s 数值容差，没有更改引擎 30 rad/s 上限、torque 或原始观测。实测速度从流记录计算，不能把命令上界写成全程实际速度。

`--slam-reference` 按参考的 `pose_source` 选择 `lio_estimate` 或 `livo_post_vio`，并设置 `ground_truth_navigation=false`。LIO 默认消费 `/dynanav/lio_observer/aft_mapped_to_init`；post-VIO 必须消费 dedicated `/post_vio_odometry`，可用 `--odometry-topic` 选择独立 namespace，不能用同帧 pre-VIO `aft_mapped_to_init` 冒充最终视觉位姿。新 post-VIO 输出不提供测得 twist/covariance；控制停车用 0.5 s pose window 与实际 SDK gyro，零 twist 占位不能表示静止。当前capture只订Odometry，不订status；实际门禁为pose/frame/freshness、pose-window及SDK gyro，没有使用status/covariance门禁。LIO reported-twist 与 pose-window 速度差异仍须记录。

所有路径中的 native 状态仍用于 policy 本体观测和物理 guards；GT还用于离线评分；oracle模式另有GT位姿诊断图，显式learned模式的持久建图只用exact实测post-VIO。旧oracle模式的dynamic hold使用oracle cloud；显式learned模式只用新RGB证据/有界隔离更新hold，不把未知/漏检视为确认无人。因此 `truth_only_for_scoring=false`，但 live 模式的 route pose 不用 GT。参考是 repeat-spawn 的先验测量目标，不是全局定位；无任意初始姿态/地图配准保证。`navigation_running=false` 不作为整栈完成声明。

## 传感与地图

新实验默认 `--render-backend usd`，关闭 Fabric Scene Delegate/PhysX Fabric 并用 CPU 更新 USD，已恢复办公室墙、扶手和楼梯深度；CameraParams 与 native 相机一致。旧 capture 默认 Fabric 保留，实际路径记录于 result。新实验默认 SDK IMU 200 Hz（可选100复现旧抽样），ROS SDK orientation 标 unavailable；旧 capture 仍 legacy100 Hz。点云由 RGB-D 反投影，不是独立旋转 LiDAR。

发布 `/dynanav/{imu,camera,depth,camera_info,lidar_filtered,lidar_static}` 与 `/clock`；learned 模式另发布 `/dynanav/lidar_perception_filtered`（临时 UNKNOWN 候选，是观测器的 learned 输入端），而 `lidar_static` 只在 oracle 模式发布。深度/标定/最终点保存像素对应，同次观测分类才能应用于该批点：

- 完整障碍云保留DYNAMIC和UNKNOWN；默认oracle静态分支只接收有效STATIC。learned分支另用临时UNKNOWN候选供估计器，持久几何须重复支持，不能把两条分支混称静态真值。
- live oracle mask 每帧计算；标签调度为 2 s 周期 + 最短 0.5 s 新实例冷却，地图 KF 为 1 s。这是本轮实验参数，不是历史已决定协议。
- 旧oracle/default capture记录`learned_model_calls=0`；显式在线`--semantic-provider learned_rgb_person`现使用独立worker与`/dynanav/lidar_perception_filtered`临时UNKNOWN候选。此前缓存对象重建仍是离线流程（上游旧LIO pose用oracle-static cloud），见 [perception README](../../perception/README.md)，本次新模式不改写旧运行身份；质量/污染结论见[新阶段报告](../../../../docs/experiments/dynanav-online-person-20261109.md)。
- oracle模式即时`mapping/`使用GT pose/oracle mask作诊断；learned模式使用exact post-VIO、独立geometry候选和person历史。离线重建显式 `pose_source=slam`、mask 仍 `sim_oracle`；旧 post-VIO 派生 Odometry bag 也不同于新 live final-pose topic。
- oracle 过滤归档不能证明 native 内部图完全无动态污染，视觉前端仍无图像动态 mask。对象 ID/context 查询边界见 [semantic_mapping](../../localization_mapping/semantic_mapping/README.md) 与 [query_scene_map.py](../../../../scripts/query_scene_map.py)。

## 已测范围

历史报告包含 USD 首段67.815 s到达/72.915 s停车、完整GT双楼梯230.335/235.415 s、1m/s命令上界首段18.595/23.615 s，以及移动中 oracle 行人 hold/resume 的首段 ONLY_LIO闭环。高速度段 `actual_command[0]>=0.95` 的6.18 s内XY均速0.902891 m/s，全部非零命令11.58 s内0.629510 m/s，不能称全程1m/s。独立landing LIO69KF评分RMSE0.035572 m；早期完整多层LIO漂移未验收。

续篇新增：去 periodic dump 后完整同 capture 的GT路线旁路对照正常完成，201共同exact KF的ONLY_LIO/post-VIO RMSE为1.919313/0.083742 m（各源首帧固定SE3仅评分）；这是旁路定位，非导航反馈。2,196条live final pose/status与诊断逐条匹配，当前 `dynanav_post_vio_20261109T052245Z` native prefix的2/2 CTest通过。三目标 measured post-VIO首轮第三段stall保留；新终点专用0.15/0.20m捕获运行报告`arrived=true`、source249.214996s到达、native流至254.415s，13项独立检查确认三阶段/两转向及5.1s零命令停车；2,367条final pose/status/native一致性已核对；journal有280条stale零命令hold，不能称全程无停顿。中间0.05/0.10m及stall/dwell等保护未改。该阶段两次视觉闭环都无行人，learned mask当时仍离线。后续速度四档完整通过，以及新45s在线行人片段的当前状态另见[新报告](../../../../docs/experiments/dynanav-online-person-20261109.md)，不混用其验收范围。离线74帧learned mask评测和RViz已实际执行。新结果不倒填旧运行身份，二进制/source manifest以各次收据为准。

## 启动与检查

在工程根目录设置 ROS 与数据环境，输出必须为新目录：

```sh
source /opt/ros/jazzy/setup.bash
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=91
DYNANAV_RUN="$PROJECT_DATA_ROOT/embodied-navigation/experiments/dynanav-$(date -u +%Y%m%dT%H%M%SZ)"
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$DYNANAV_RUN/capture" --speed 0.15 --duration 90 --check
/home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py \
  --output "$DYNANAV_RUN/capture" --speed 0.15 --duration 90
```

`--check` 核对资产/哈希和参数；选择 live reference 时用同一工厂纯实例化控制器，不启动 ROS/Isaac，不证明场景物理或输入质量。实际需要本机 Isaac6.0.1 RC和有权使用的资产。估计器独立启动、标定与模式说明见 [LIO/LIVO README](../../../../configs/simulation/dynanav/README_LIO.md)。所有地图、视频、RGB、日志和模型放外部数据卷，异常保留 result 和可用执行器故障状态。

三终端在线行人实验（worker → LIVO observer → capture）可用 [run_dynanav_online_person.py](../../../../scripts/run_dynanav_online_person.py) 一次串起：`--print-commands` 只打印等价 shell 命令，`--check` 只核对文件/目录（不启动 ROS、Isaac 或模型），实际运行会等 worker `state.json` ready、observer `launch.json` 落盘后再启动 capture，逐进程日志写到 `<run>/logs/`；成功、失败、`--capture-timeout`、SIGINT/SIGTERM 都会先停 capture 再停 observer/worker，并写 `supervision.json`（schema v3，含 argv/pid/退出码/错误/是否由监督停止/`unexpected_exits`；capture 正常退出但 worker 或 observer 已自行死亡时 `ok=false`）。它按 `--ros-setup` 显式 source ROS（默认 `/opt/ros/jazzy/setup.bash`，`--no-ros-setup` 保留继承环境），并把 `PROJECT_DATA_ROOT` 设为 `--data-root` 的父目录。它只是进程监督，不等于导航或动态感知验收；已有手工三终端命令仍是同一编排的展开。

历史提交 `bffa2b6` 验证为unit170（skip3）、regression83（skip5）、native1/1及其lint；它不是新增post-VIO/learned代码的最终测试计数。提交`43cba33`当时的完整套件为unit217（skip3）、regression86（skip5），其余通过；native2/2，20个Python文件Ruff check/format、bash语法及vendor304文件大小/SHA通过。CPU fixture或文件检查不替代ROS/Isaac运行、完整地图或实机验收。

采集实现的逐模块职责、入口与状态归属见 [capture 包说明](capture/README.md)。
