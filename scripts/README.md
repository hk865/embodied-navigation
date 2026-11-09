# scripts

文件数量、行数和行号是 2026-11-09 文档审计时的结构快照；后续生命周期修复以当前源码和 [真实重验记录](../docs/review/runtime-repair-validation-20261109.md) 为准，保留历史统计便于对照。
本目录是 DynaNav/Isaac 实验（路径 D）与离线分析的**入口脚本集**：负责进程编排、数据采集、离线重建、回放、评测与审计；它不实现导航控制律，也不写主路径的 `runs/`。

## 边界

- 做：显式启动一次 DynaNav/Isaac 实验（capture、experiment、LIO/LIVO observer、行人工人推理 worker、以及把三者串起来并收尾的 supervisor）；从已有记录重建地图/路线参考；把已记录的 IMU/RGB/地图回放到 ROS 话题；离线评测、汇总与速度档位审计；建立数据卷软链。
- 不做：不改源码、不改守护/门禁参数；不启动主路径（`start.py`）或 RoboGauge 基准；不做验收——`run_dynanav_online_person.py` 写的 `supervision.json` 明确标注 `"technical process supervision only; not navigation or dynamic-perception acceptance"`；不写 `runs/`（主路径产物目录），产物落在外部数据卷 `$PROJECT_DATA_ROOT/embodied-navigation/` 下或调用方指定的新目录。
- 16 个 Python 脚本 + 1 个 shell（`run_dynanav_lio.sh`），共 17 个文件，合计 5863 行（`wc -l`，含 shell 438 行）。脚本不是包的一部分，也不是 `pyproject.toml` 的 console script；只有 `simulation/dynanav/experiment.py` 在源码快照清单里按**路径字符串**引用其中两个（见"模块内文件耦合"）。

## 目录结构

```text
scripts/
├── audit_dynanav_speed_ladder.py             # 288 行，离线速度档位证据审计（评测）
├── build_dynanav_slam_route.py               # 468 行，从记录的 Odometry 实测位姿生成路线参考（重建）
├── capture_dynanav.py                        # 65 行，显式 Isaac RGB-D capture 入口（入口）
├── configure_data.py                         # 51 行，检查/创建 3 个数据卷软链（工具）
├── evaluate_dynanav_person_segmentation.py   # 464 行，离线 RGB-only 行人分割评测（评测）
├── query_scene_map.py                         # 76 行，查询已落盘的 scene_map.json 快照（回放/查询）
├── reconstruct_dynanav_learned_person_map.py # 452 行，learned person 点 + LIO 位姿离线重建（重建）
├── reconstruct_dynanav_slam_map.py           # 602 行，用记录的 LIO 估计重建观测点层（重建）
├── replay_controller.py                      # 103 行，用已保存 run 校验提取出的驾驶律（回放）
├── replay_dynanav_imu.py                     # 444 行，按收到的 /clock 回放 SDK IMU 行（回放）
├── replay_dynanav_rgb.py                     # 651 行，按收到的 clock 回放原始 RGB NPZ 帧（回放）
├── run_dynanav_experiment.py                 # 7 行，experiment.main 的薄入口（入口）
├── run_dynanav_lio.sh                        # 438 行 shell，隔离的 static RGB-D/IMU observer 启动器（入口）
├── run_dynanav_online_person.py              # 636 行，三进程在线行人实验的 supervisor（入口）
├── run_dynanav_person_worker.py              # 436 行，持久 RGB-only 行人推理 worker（入口）
├── summarize_dynanav_experiment.py           # 414 行，汇总已记录实验证据（评测）
├── view_dynanav_map.py                       # 268 行，把已有估计地图发到隔离 review 话题（回放）
└── README.md                                 # 本文件（职责/数据/耦合/可读性说明；行数随改写变化）
```

`__pycache__/` 是运行产物（未设置 `-B` 时导入生成），不入 Git，也不在上表。

## 代码、日志与配置

### 代码：17 个文件按职责分类

| 分类 | 脚本 | 行数 | 职责（已核实） |
| --- | --- | ---: | --- |
| 入口 | `run_dynanav_experiment.py` | 7 | 纯委托：`from embodied_navigation.simulation.dynanav.experiment import main`，`raise SystemExit(main())`；所有参数都在被委托模块里 |
| 入口 | `capture_dynanav.py` | 65 | 显式 Isaac RGB-D capture 入口；`--check` 只检查 scene/robot_usd/Student 权重三个文件是否存在，**不启动 Isaac**；正常路径设 `PROBE_STEPS` 后用 `runpy.run_module("embodied_navigation.simulation.dynanav.capture_go2_rgbd")` 在**本进程内**执行 |
| 入口 | `run_dynanav_lio.sh` | 438 | 启动隔离 namespace 下的 ONLY_LIO / LIVO observer；`--check` 只做文件/hash/动态依赖预检（不建 run、不起 ROS）；`--mode livo` 额外起 `parameter_blackboard` 并校验相机参数，`trap` 清理自己启动的子进程 |
| 入口 | `run_dynanav_online_person.py` | 636 | 三进程 supervisor：worker（就绪）→ LIVO observer（`ready.json` 校验输入订阅，`launch.json` 留存来源）→ capture（绑定该收据）；按 `--check`、`--print-commands`、超时、信号、退出码决定成败，最后写 `supervision.json` |
| 入口 | `run_dynanav_person_worker.py` | 436 | 持久 RGB-only 行人推理 worker：只做本地 IPC（channel 目录）、不 import rclpy、不下载权重；模型为 `torchvision.maskrcnn_resnet50_fpn_v2`（`TorchvisionPersonModel`） |
| 重建 | `build_dynanav_slam_route.py` | 468 | 从 SQLite bag 里记录的 `nav_msgs/Odometry` 实测位姿生成路线参考；只读 bag、不读仿真状态/图像归档/真值；终止点用显式 measured-pose 静止窗口确认 |
| 重建 | `reconstruct_dynanav_slam_map.py` | 602 | 用记录的 LIO 估计重建观测点层（每个关键帧必须有同帧实测 `Odometry`）；输出不覆盖、不写 GT 位姿 |
| 重建 | `reconstruct_dynanav_learned_person_map.py` | 452 | learned person mask + 记录 LIO 位姿的离线重建；复用 `reconstruct_dynanav_slam_map.py` 的位姿索引；带上游 provenance 校验 |
| 回放 | `replay_dynanav_imu.py` | 444 | 按收到的 `/clock` 回放真实 SDK IMU 行到 `/dynanav/imu`；只取 time/accel_body/gyro_body；`--check` 不 import ROS，但仍会写 receipt |
| 回放 | `replay_dynanav_rgb.py` | 651 | 按收到的 clock 回放原始 RGB NPZ 帧到 `/dynanav/camera`；只读 rgb/source_ns/frame_id/encoding/frame_index 数组，别的数组（含真值）一概不加载；`--check` 同样会写 receipt |
| 回放 | `replay_controller.py` | 103 | 离线校验"提取出的驾驶律"与一次真实保存的 run 是否一致；帧选择只用记录的 target/heading/distance，不用被检验的 command |
| 回放/查询 | `view_dynanav_map.py` | 268 | 把已有估计 XYZ 地图发布到隔离静态话题 `/dynanav/review/static_map`；不启动控制/TF/仿真/重建 |
| 回放/查询 | `query_scene_map.py` | 76 | 查询一份已落盘 `scene_map.json` 快照；查询/过滤逻辑在 `localization_mapping.semantic_mapping.query`，本文件只是 CLI 包装 |
| 评测 | `evaluate_dynanav_person_segmentation.py` | 464 | 离线 RGB-only 行人 mask 对稀疏 oracle 采样评测；**不做在线控制**；输出必须是数据卷下的新目录 |
| 评测 | `summarize_dynanav_experiment.py` | 414 | 汇总已记录的 DynaNav 证据（`result.json`、`truth_scoring_stream.jsonl`、`policy_frames.jsonl`、`mapping/frames.jsonl`）；不启动 Isaac、不改 run |
| 评测 | `audit_dynanav_speed_ladder.py` | 288 | 离线速度档位证据审计；跑在 capture/recorder 停止之后；不是实验启动器、也不改 guard |
| 工具 | `configure_data.py` | 51 | 检查/创建 `assets`、`models`、`runs` 三个数据卷软链；已存在真实内容或指向别处时**报错不改** |

### 日志与数据：写到哪里

所有脚本的产物都在外部数据卷 `$PROJECT_DATA_ROOT/embodied-navigation/` 下（`runs/`、`assets/`、`models/` 是仓库内软链，`experiments/` 只在数据卷里）。**除 `configure_data.py` 在仓库根建 3 个软链外，没有脚本写 Git 内路径**；`--check` 类路径只读不建目录（两个 replay 的 `--check` 例外：仍会写一份 `validated_check_only` receipt）。

| 脚本 | 产物位置（数据卷内） | 关键文件与语义 |
| --- | --- | --- |
| `run_dynanav_online_person.py` | `<run>/` | `supervision.json`（schema `dynanav_online_supervision/v3`：argv/pids/退出码/`unexpected_exits`/`ok`，scope 明确"非验收"）；`<run>/livo/snapshots.yaml`（observer 的参数快照覆盖）；子进程 stdout/stderr 落各自 log |
| `capture_dynanav.py` / `run_dynanav_experiment.py` | `<run>/capture/`（supervisor 默认）或 `--output` | `result.json`、`experiment.json`、`source_manifest.json`（含 30+ 源码/两份 configs 的 sha256 与 `source_snapshot/` 副本）；`--archive-rgb` 时归档每帧原始 NPZ |
| `run_dynanav_lio.sh` | `<output-dir>/`（必须位于数据卷下且不存在） | `preflight.json`、`launch.json`（observer 收据：输入话题、namespace、二进制/配置 hash、domain）、`runner.log`、`ros_logs/`、`diagnostics/`；livo 另有 `camera_parameters.log`、`camera_parameter_ready.json`、`capture_calibration.yaml`、输入订阅 `ready.json` 和结束时的 `shutdown.json`（退出时移除 ready） |
| `run_dynanav_person_worker.py` | `<channel>/`（必须是数据卷下的新目录，权限 0700） | `requests/*.npz`、`results/*.npz`、`results/*.error.json`、`events.jsonl`、`state.json`（模型 hash + 计数）、`stop.request.json` |
| `reconstruct_dynanav_slam_map.py` | `--output`（新目录） | `static_map.npz`（内嵌 `pose_source="slam"`、`mask_source="sim_oracle"`、`no_GT_pose_used=true`、`semantic_oracle=true`、`complete_full_scene=false`）、`scene_map.json`、`receipt.json` |
| `reconstruct_dynanav_learned_person_map.py` | `<output-prefix>-<mode>/`（数据卷下、不得已存在） | `static_map.npz`、`scene_map.json`、`person_history.json`、`person_query.json`、`person_query_fresh.json`、`person_context.txt`、`receipt.json`、`provenance.json` |
| `build_dynanav_slam_route.py` | `--output`（新文件） | 路线参考 JSON：`version`/`pose_source`/`no_gt_pose_used`/`repeat_spawn_assumption`/`localization_scope`/`source{bag,files,sha256,...}`/`anchor`/`goals`/`goal_selection` |
| `evaluate_dynanav_person_segmentation.py` | `--output`（数据卷下的新目录） | `all.json`、`periodic2s.json`、`compare.json`、`receipt.json`（schema `offline_RGB_only_person_segmentation/v1`）、`predictions/*.npz`、`visualizations/` |
| `summarize_dynanav_experiment.py` | stdout 或 `--output`（新文件） | schema `dynanav-experiment-summary/v1`；缺证据写 `null`，不把"进程完成"当"导航成功" |
| `audit_dynanav_speed_ladder.py` | `--output`（新目录） | `speed-ladder-evidence.json`（schema `dynanav_speed_ladder_offline_audit/v1`）、`speed-ladder-table.csv`，并把自身与 helper 源码副本写成 `audit-used.py`、`summarize_dynanav_experiment.py` |
| `replay_dynanav_imu.py` | `--receipt`（新文件） | schema `dynanav_sdk_imu_replay_receipt_v1`；`--check` 也写 receipt（status=`validated_check_only`） |
| `replay_dynanav_rgb.py` | `--receipt`（新文件） | schema `dynanav_original_rgb_replay_receipt/v1`；receiver 字段明写 `native_observer_receipt_verified=false`、`ack_scope` 只代表 publisher 侧可靠确认 |
| `replay_controller.py` | stdout 或 `--output` | 逐命令误差/dropped 计数 |
| `view_dynanav_map.py` / `query_scene_map.py` | 无文件产出（前者发话题，后者 stdout） | 只读输入 |
| `configure_data.py` | 仓库根 `assets`/`models`/`runs` 三个软链 | stdout 打印 `data_present`/`linked` |

全局产物布局（含 `runs/` 与 `experiments/<run>/` 的完整字段）见 [`../docs/review/dsh-workflow-and-readability-20261109.md`](../docs/review/dsh-workflow-and-readability-20261109.md) §9，本目录不复述。

### 配置

Python 脚本**不直接读 `configs/`**（`grep -rn "configs/" scripts/*.py` 无命中）。配置只有两个真实入口：

| 谁读 | 读什么 | 何时读 |
| --- | --- | --- |
| `run_dynanav_lio.sh:90-91` | `configs/simulation/dynanav/fastlivo_lio.yaml` 或 `fastlivo_livo.yaml`，以及 `configs/simulation/dynanav/camera_rgbd.yaml` | 启动前拼 `--params-file`；`--mode` 决定用哪份 fastlivo 配置 |
| `run_dynanav_experiment.py` 委托的 `simulation/dynanav/experiment.py` | `configs/simulation/dynanav_basement_to_ground_collision.json`、`dynanav_basement_collision.json` | 只用于 **source_snapshot**：逐字节读入 + sha256 + 复制到输出目录，不是运行参数 |

其余全部靠命令行参数 + 环境变量 + 数据卷路径：

- `PROJECT_DATA_ROOT`：数据卷根（`project.data_root()` 返回 `<PROJECT_DATA_ROOT>/embodied-navigation`）。缺失时不会回退到仓库内路径：`configure_data.py` 与 `run_dynanav_online_person.py` 直接 `SystemExit`，其余脚本调用 `project.data_root()` 抛 `RuntimeError`。
- `PYTHONPATH`：`run_dynanav_online_person.py:163` 给子进程注入 `<repo>/src`；7 个脚本自己也做 `sys.path.insert(0, ROOT/"src")`。
- `ROS_DOMAIN_ID`：`run_dynanav_lio.sh:103` 默认 91；supervisor 用 `--domain`（默认 116）显式指定，并要求 worker/observer/capture 同域。
- 其它被读取的环境变量：`NAV_END_SIM`（`relative_slam_node.py`）、`NAVIGATION_ACTOR_PYTHON`（robogauge benchmark，不属本目录）。

## 模块内文件耦合

依赖方向是**星型而不是链式**：16 个 Python 脚本之间没有 `import` 关系（互相 import 会立刻踩到"脚本不是包"的问题），真实的耦合只有 4 条边：

1. `run_dynanav_online_person.py` → **子进程**调 `run_dynanav_person_worker.py`、`run_dynanav_lio.sh`、`run_dynanav_experiment.py`。脚本路径是模块常量（`WORKER_SCRIPT`/`OBSERVER_SCRIPT`/`CAPTURE_SCRIPT`，L35-37），argv 由纯函数 `build_plan()` 组装，就绪判据是同次启动的原子 `<observer>/ready.json` 中 `state=ready`，由 launcher 确认所拥有 mapper 的相机/IMU/点云订阅后写入；`launch.json` 仅记录来源，不代表估计器初始化或已收到帧。
2. `reconstruct_dynanav_learned_person_map.py` → 用 `importlib.util.spec_from_file_location` **按路径加载** `reconstruct_dynanav_slam_map.py`（`load_script` L38-44；调用点 L382-385），注入 `sys.modules` 后复用其 `PoseIndex`/`load_keyframe` 等。
3. `audit_dynanav_speed_ladder.py` → 同样按路径加载 `summarize_dynanav_experiment.py`（`HELPER_PATH` L22–24）复用 `speed_metrics`，并把两份源码复制进输出目录作为证据。
4. `capture_dynanav.py` → `runpy` 在**本进程内**执行 `simulation.dynanav.capture_go2_rgbd`（这是包内模块，不是脚本）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `audit_dynanav_speed_ladder.py` | 288 | importlib 加载 `summarize_dynanav_experiment.py` | 无 | 可疑（按路径加载 + 复制源码） |
| `build_dynanav_slam_route.py` | 468 | 标准库 + 延迟 import rclpy 解 CDR | 无 | 清楚 |
| `capture_dynanav.py` | 65 | `embodied_navigation.project`、`simulation.dynanav.capture_go2_rgbd`（runpy） | 无 | 清楚 |
| `configure_data.py` | 51 | 标准库 | 无 | 清楚 |
| `evaluate_dynanav_person_segmentation.py` | 464 | `perception.person_segmentation`、`project` | 无 | 清楚 |
| `query_scene_map.py` | 76 | `localization_mapping.semantic_mapping(.query)` | 无 | 清楚 |
| `reconstruct_dynanav_learned_person_map.py` | 452 | 按路径加载 `reconstruct_dynanav_slam_map.py` + `perception.learned_person_mapping`、`person_segmentation` | 无 | 可疑（按路径加载） |
| `reconstruct_dynanav_slam_map.py` | 602 | `localization_mapping.semantic_mapping` | 被同目录 1 个脚本按路径加载 | 偏重（602 行，同时做 bag 解码/位姿索引/重建/收据） |
| `replay_controller.py` | 103 | `control.relative_pid` | 无 | 清楚 |
| `replay_dynanav_imu.py` | 444 | 标准库 + 延迟 import ROS | 无 | 清楚（ROS 只在 `run_ros` 里 import） |
| `replay_dynanav_rgb.py` | 651 | 标准库 + 延迟 import ROS | 无 | 偏重（651 行 / 20 函数） |
| `run_dynanav_experiment.py` | 7 | `simulation.dynanav.experiment` | 被 supervisor 以子进程调 | 清楚 |
| `run_dynanav_lio.sh` | 438 | `configs/simulation/dynanav/*.yaml` + 原生二进制 | 被 supervisor 以子进程调 | 偏重（shell 里内嵌 4 段 Python） |
| `run_dynanav_online_person.py` | 636 | 3 个子进程 + 标准库 | 无（顶层入口） | 偏重（supervisor 职责多，但拆分清楚） |
| `run_dynanav_person_worker.py` | 436 | `perception.person_worker`、`perception.person_tiling`、`project` | 无 | 偏重（`main()` 242 行） |
| `summarize_dynanav_experiment.py` | 414 | 标准库 + NumPy | 被 audit 按路径加载 | 清楚 |
| `view_dynanav_map.py` | 268 | 标准库 + 延迟 import ROS | 无 | 清楚 |

四类问题：

1. **上帝文件（>800 行）**：无。最长的 `replay_dynanav_rgb.py` 651 行、`run_dynanav_online_person.py` 636 行、`reconstruct_dynanav_slam_map.py` 602 行，都在 600 量级但没有跨过 800。
2. **重复实现**：**有，且是本目录最主要的复制点**——文件 sha256 助手写了 4 份（`build_dynanav_slam_route.py:281` 的 `sha256_file`、`evaluate_dynanav_person_segmentation.py:34`、`reconstruct_dynanav_learned_person_map.py:46`、`run_dynanav_person_worker.py:32` 的 `digest`，实现几乎一致）；`write_json` 写了 3 份（`evaluate_…:42`、`reconstruct_learned_…:54`、`run_dynanav_online_person.py:374`）；`ROOT = Path(__file__).resolve().parents[1]` 重复 9 份、`sys.path.insert(0, ROOT/"src")` 重复 7 份。没有 `scripts/_common.py` 之类的共享模块。
3. **跨层直达**：脚本作为 CLI 必须 `sys.path.insert` 后直接 import `src/` 里的包（`perception.person_segmentation`、`localization_mapping.semantic_mapping`、`control.relative_pid` 等），这是"脚本层"的常态；需要注意的是它们直接依赖**内部**模块而不是稳定 API（如 `person_segmentation` 的私有函数、`semantic_mapping.query` 的 re-export）。
4. **隐式契约**：三处按路径/文件名耦合——`load_script` 按文件路径加载同目录脚本（改名/搬家即断）、supervisor 以 `<observer>/ready.json` 的输入订阅握手作为 observer 就绪判据，并另查 `launch.json` 来源、worker 与 capture 之间以 `requests/`、`results/` 目录里的 `.npz` 文件名（request_id）交互。这些契约没有 schema 常量，只在 docstring 与 receipt 字段里体现。

结论：**耦合清楚**——没有 import 环，没有共享可变状态，进程之间靠命令行与文件契约，这是脚本集合的合理形态；代价是**横向重复**（4 份 digest、3 份 write_json、9 份 ROOT）与**按路径加载**两处脆弱点。真正该抽公共 helper 的信号已经很明确，但抽取会改变"每个脚本可单独拷走运行"的现状，所以只应抽纯函数（hash/原子写），不要抽流程。

## 可读性评估

指标为实测 AST 值（函数总数 / 有 docstring 函数比 / 最长函数 / >80 行函数数 / 注释行），抽查 `evaluate_dynanav_person_segmentation.main`（L305-460，156 行）与 `run_dynanav_online_person.build_plan`（L172-279，108 行，纯函数）确认可信。

**良好**

- `run_dynanav_experiment.py`（7 行）、`configure_data.py`（51 行 / 1 函数 36 行，`main` L12–47） / 只做软链检查）：短、单一目的、无副作用。
- `capture_dynanav.py`（65 行 / `main` 46 行，L16–61）：`--check` 与执行的边界写在 docstring 与代码里（`--check` 不启动 Isaac）。
- `query_scene_map.py`（76 行 / `main` 49 行，L24–72）：明确把查询逻辑委托给 `semantic_mapping.query`，自己只做 CLI。

**可接受**

- `run_dynanav_online_person.py`（636 行 / 25 函数 / docstring 10/25，最长 `build_plan` 108 行）：结构是好的——`Child` 数据类、纯 `build_plan()`、纯 `final_verdict()`、`stop()` 单独负责信号与回收；扣分在函数多、docstring 只覆盖四成。
- `reconstruct_dynanav_slam_map.py`（602 行 / 12 函数 / docstring 4/12，最长 `reconstruct` 158 行，L366–523）：bag 解码、位姿索引、重建、收据分成独立函数，读得下去；长函数在装配段。
- `summarize_dynanav_experiment.py`（414 行 / 14 函数 / 0 个 >80 行 / docstring 1/14）：结构最好的一批（全是小函数），文档最差的一批；`speed_metrics` 70 行（L124–193）。
- `build_dynanav_slam_route.py`（468 行 / 14 函数 / docstring 3/14，最长 `build_reference` 122 行（L289–410））。
- `evaluate_dynanav_person_segmentation.py`（464 行 / 7 函数 / docstring 1/7，最长 `main` 156 行，2 个 >80）：推理/评测/可视化分开，但 `main` 承担全部参数校验与流程。
- `replay_dynanav_imu.py`（444 行 / 18 函数 / docstring 4/18，最长 `run_ros` 116 行（L254–369））：函数切得细，ROS 只在 `run_ros` 内延迟 import，`--check` 路径无 ROS 依赖。
- `view_dynanav_map.py`（268 行 / 7 函数 / 0 个 >80 / docstring 1/7，最长 `load_map` 67 行，L60–126）。
- `replay_controller.py`（103 行 / 3 函数 / 0 个 >80 / docstring 0/3，最长 `replay` 64 行，L20–83）。

**需改进**

- `replay_dynanav_rgb.py`（651 行 / 20 函数 / **docstring 1/20**，`run_ros` 129 行（L395–523）、2 个 >80 行函数）：同目录最长的文件，却几乎没有函数级文档；`load_archive`→`select_frames`→`fill_image`→`run_ros` 的时序约束只能从实现读。任务书里点名的 651 行是准确的。
- `run_dynanav_person_worker.py`（436 行 / 5 函数 / docstring 0/5，`main()` 242 行，L191–432）：模型加载、IPC 轮询、响应写盘、信号与状态机全在一个函数里；`--max-requests` 的语义（0=常驻）只在 CLI help 里。
- `reconstruct_dynanav_learned_person_map.py`（452 行 / 6 函数 / docstring 0/6，`reconstruct_mode` 232 行（L102–333）、2 个 >80）：逐帧重建 + 多层地图写盘压在一个函数里，与 `reconstruct_dynanav_slam_map.py` 的清晰分层形成对比。
- `audit_dynanav_speed_ladder.py`（288 行 / 10 函数 / docstring 1/10，最长 `audit_run` 118 行（L120–237））：**代码风格**与其余脚本不一致——大量单行 `if ...:raise ...`、单行 `with`、紧凑字典字面量（L252–283），正文注释只有 1 行；功能上没问题，但阅读成本明显高于同目录其他文件。

区分说明：`replay_dynanav_rgb.py`/`run_dynanav_person_worker.py`/`reconstruct_dynanav_learned_person_map.py` 是**结构问题**（超长函数）；`summarize_dynanav_experiment.py`/`view_dynanav_map.py`/`replay_controller.py` 只是**文档问题**（结构已足够小）；`audit_dynanav_speed_ladder.py` 是**风格问题**（压缩写法），不要误判成功能缺陷。

## 重构建议

本轮只写建议、不改代码；任何一条都要"先有对照再动"。**没有 P0**：所有入口都是 fail-closed（不存在的输出目录/已存在的 receipt/缺失 `PROJECT_DATA_ROOT`/hash 不符都直接报错退出），replay 类不伪造时间戳，summarize/audit 明确"进程完成 ≠ 导航成功"。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `reconstruct_dynanav_learned_person_map.reconstruct_mode()`（232 行）按"逐帧预测校验 / 分层地图写入 / 收据汇总"拆成 3 个函数 | 与同目录 `reconstruct_dynanav_slam_map.py` 的分层程度差距明显，是重建类脚本最难改的一处 | 中：顺序与收据字段敏感 | `PYTHONPATH=src python3 -B -m unittest tests.unit.test_learned_person_mapping tests.unit.test_dynanav_source_snapshot -v`；再用一份已有 run 做离线重跑对比 `receipt.json` |
| P1 | 把 `run_dynanav_person_worker.main()`（242 行）拆出 `run_loop(state)` 与 `handle_request()`，保留 `main()` 只做参数校验 | 常驻 worker 的主循环与模型加载混在一起，改 IPC 语义要读全函数 | 中：涉及信号、`--max-requests`、stop 文件语义 | `python3 -B -m unittest tests.unit.test_person_worker tests.unit.test_person_tiling -v`；用 `--max-requests 1` 做一次离线 smoke |
| P1 | 把 `load_script`（`reconstruct_dynanav_learned_person_map.py:38-44`、`audit_dynanav_speed_ladder.py:22-24`）的按路径加载改成正常 import 或显式声明（`sys.path` 前置 `scripts/` 后 `import`） | 按文件路径加载对改名/搬家零容错，且注入 `sys.modules` 后调试栈更难读 | 低：两个脚本都只在本机单跑 | 两个脚本的 `--check`/`--help` + 对应单测（`test_dynanav_speed_ladder_audit`、`test_dynanav_source_snapshot`） |
| P2 | 抽一个纯函数模块（如 `scripts/_util.py`）承接 `sha256_file`/`digest` 与 `write_json`，替换 4 份 digest 与 3 份 `write_json` | 横向重复最多的就是这两类，行为完全一致 | 低：需保留各脚本"可单独拷走"的用法（加 try-import 回退） | 逐脚本 `--help` + 全量单测 |
| P2 | 给 `replay_dynanav_rgb.py` 的 20 个函数补 docstring（尤其 `load_archive`/`select_frames`/`fill_image`/`run_ros`），写明时序与"不造时间"约束 | 651 行只有 1 个 docstring，回放的时序约束全靠读实现 | 无（纯文档） | 只改文档 |
| P2 | 给 `run_dynanav_person_worker.py` 与 `reconstruct_dynanav_learned_person_map.py` 的模块/公开函数补 docstring | 两文件 docstring 均为 0 | 无（纯文档） | 只改文档 |
| P2 | 统一 `audit_dynanav_speed_ladder.py` 的代码风格（拆单行 `if`/`with`，补注释） | 与同目录其它脚本风格割裂，评审成本高 | 低（纯格式） | 改后比对 `speed-ladder-evidence.json` 输出字节级不变 |

## 前置条件与常用命令

本目录所有实验类脚本都要求：已构建的 native/ROS 产物（`install/ros`）、数据卷 `PROJECT_DATA_ROOT`、模型权重与 SHA、以及 Isaac（仅 capture/experiment）。**不要**把 `--check` 通过当作可运行证据：`capture_dynanav.py --check` 只检查三个文件存在、`run_dynanav_lio.sh --check` 只检查文件/hash/动态依赖、`run_dynanav_online_person.py --check` 只做就绪项枚举，三者都不启动 ROS 或 Isaac。

- 手工三终端（路径 D 原始形态）与统一 supervisor 两种用法、以及每步的参数，见 [`../docs/experiments/dynanav-online-person-20261109.md`](../docs/experiments/dynanav-online-person-20261109.md)（第 127 行起"从仓库入口复跑相同配置"给的是实际命令）。
- 重建地图的完整命令与输出说明见 [`../docs/experiments/reconstruct_dynanav_slam_map.md`](../docs/experiments/reconstruct_dynanav_slam_map.md)。
- 路径 D 在四条执行路径中的定位见 [`../docs/architecture.md`](../docs/architecture.md)。

## 测试

脚本是**测试覆盖较好的一批**：11 个脚本有直接引用它们的单测/回归测试（`grep -rl` 命中见下），未被引用的有 6 个。

| 脚本 | 测试文件 |
| --- | --- |
| `audit_dynanav_speed_ladder.py` | `tests/unit/test_dynanav_speed_ladder_audit.py` |
| `build_dynanav_slam_route.py` | `tests/unit/test_dynanav_slam_route_builder.py` |
| `query_scene_map.py` | `tests/unit/test_dynanav_source_snapshot.py`、`test_layered_mapping.py`、`test_learned_person_mapping.py` |
| `reconstruct_dynanav_learned_person_map.py` | `tests/unit/test_dynanav_source_snapshot.py` |
| `reconstruct_dynanav_slam_map.py` | `tests/unit/test_dynanav_slam_reconstruction.py` |
| `replay_dynanav_imu.py` | `tests/unit/test_dynanav_imu_replay.py` |
| `replay_dynanav_rgb.py` | `tests/unit/test_dynanav_rgb_replay.py` |
| `run_dynanav_lio.sh` | `tests/regression/test_dynanav_lio_launcher.py`、`tests/unit/test_dynanav_learned_wiring.py` |
| `run_dynanav_online_person.py` | `tests/unit/test_dynanav_online_supervisor.py` |
| `summarize_dynanav_experiment.py` | `tests/unit/test_dynanav_experiment_summary.py` |
| `view_dynanav_map.py` | `tests/unit/test_dynanav_map_viewer.py` |

未被直接引用的 6 个：`capture_dynanav.py`、`configure_data.py`、`evaluate_dynanav_person_segmentation.py`、`replay_controller.py`、`run_dynanav_experiment.py`、`run_dynanav_person_worker.py`。其中 `capture_dynanav.py` 的行为经 `tests/unit/test_dynanav_capture_orchestration.py` 覆盖被它 `runpy` 执行的 `capture_go2_rgbd`（不是覆盖这个入口本身）；`replay_controller.py` 只在 `docs/architecture.md` 的测试边界里被提及，没有测试。测试命令与边界见 [`../tests/README.md`](../tests/README.md)。

## 限制

- 本目录所有实验脚本都依赖**外部数据卷**与**本机构建产物**；仓库内 `install/`、`assets/`、`models/`、`runs/` 是软链或空目录时，只能读源码，不能据此判断实验可运行。缺文件 ≠ 缺源码，但"有几个文件"也 ≠ ready。
- 完整 Isaac 运行、GPU 推理、LIVO 视觉更新与实机均未验证；`supervision.json` 的 `ok=true` 只表示进程监督成功，历史记录里已有 `mission_completed=false` 的片段，不要把它当导航或动态感知验收。
- 回放的 receipt 只证明 publisher 侧调度与可靠确认：`replay_dynanav_rgb.py` 的 receipt 自带 `native_observer_receipt_verified=false` 与 `ack_scope` 说明（BEST_EFFORT 订阅者是否收到未被证明）。
- `--check` 不是零副作用：两个 replay 的 `--check` 仍会写 `validated_check_only` receipt，重复跑到同一路径会因"receipt 已存在"失败。
- 重建类脚本保留各自来源边界：`reconstruct_dynanav_slam_map.py` 写 `no_GT_pose_used=true` 但 `mask_source="sim_oracle"`、`complete_full_scene=false`；learned 分支的上游仍可能带 oracle 前处理来源，不能称端到端 oracle-free。
