# configs（base 配置与契约数据）

盘点日期 2026-11-20。本目录是 DynaNav / Isaac（D 路径）的 base 配置与契约数据，共 12 个 git 跟踪文件，分 `simulation/` 与 `visualization/` 两个子目录。它**不是**某次运行的有效值：一次运行的真实参数是 run 目录里冻结的副本与收据，本目录只提供启动前的基线。

## 模块定位与架构层级

- 架构层级：基础设施层（配置数据），不属于七层中的任何业务层。它被仿真层（`simulation/dynanav/`）、估计器启动器（`scripts/run_dynanav_lio.sh`）、主入口（`scripts/run_dynanav_native_exploration.sh`）与人工可视化读取。
- 输入：维护者对 YAML/XML/JSON 的编辑；文件随 git 跟踪，无生成步骤、无 include/继承。
- 输出：不作为运行产物写回。运行期消费者把文件内容转成 ROS 参数、USD 会话修复指令、DDS 传输 profile 或 RViz 显示配置。
- 上游 / 下游：
  - 被 `scripts/run_dynanav_lio.sh` 读成 `--params-file`（`fastlivo_lio.yaml` / `fastlivo_livo.yaml`、`camera_rgbd.yaml`、按需 `fastlivo_rtx_os1_32.yaml`）；
  - 被 `scripts/dynanav_observer_transport.py` 选中为 DDS 接收 profile（`observer_udp_rx4m.xml`）；
  - 被 `src/embodied_navigation/simulation/dynanav/session_collision.py` 运行期读取（`dynanav_basement_collision.json`），并被 `experiment.py` 的 source snapshot 逐字节复制；
  - 被人工 `rviz2 -d` 加载（`visualization/dynanav_map.rviz`）。
- 边界：本目录不写日志/产物、不含可执行代码；同一 topic/frame 改名不会在这里报错。不要手改 run 目录里冻结的配置副本当基线，也不要把本目录的文件当成「已经生效」。

## 当前文件夹结构

```text
configs/
├── README.md                                         # 本文件
├── simulation/
│   ├── dynanav_basement_collision.json               # 349 行，地下室+地面碰撞会话修复契约（运行期读取）
│   ├── dynanav_basement_to_ground_collision.json     # 463 行，地下室→地面路线与碰撞选择（仅 source snapshot）
│   └── dynanav/
│       ├── README.md                                 # 68 行，估计器配置索引
│       ├── README_LIO.md                             # 331 行，LIO/LIVO 长文说明与实验记录
│       ├── camera_rgbd.yaml                          # 18 行，512×512 pinhole 内参基准
│       ├── fastlivo_lio.yaml                         # 100 行，ONLY_LIO 模式参数
│       ├── fastlivo_livo.yaml                        # 129 行，RGB 辅助 LIVO 模式参数
│       ├── fastlivo_rtx_os1_32.yaml                  # 27 行，RTX OS1 32 线传感器覆盖
│       └── observer_udp_rx4m.xml                     # 21 行，observer 4 MiB UDPv4 接收 profile
└── visualization/
    ├── README.md                                     # 81 行，人工 RViz 查看说明
    └── dynanav_map.rviz                              # 69 行，RViz2 显示配置
```

12 个文件、合计 1762 行（`wc -l`，2026-11-20）。本目录没有 `.py` 代码；早期 base 配置子目录已在本分支退役，**不存在**对应读取点，也不再提供任何模块清单入口。

## 逐文件职责

| 文件 | 核心职责 | 关键接口/读取点 |
| --- | --- | --- |
| `README.md` | 本文件：目录边界、逐文件职责、真实读取点与验证入口。 | — |
| `simulation/dynanav/README.md` | 估计器配置子目录索引：说明估计器 YAML 的模式差异、唯一读取点与不写日志的边界。 | 文档，无代码读取 |
| `simulation/dynanav/README_LIO.md` | LIO/LIVO 长文说明：启动命令、隔离 namespace、标定契约、post-VIO 评分与生命周期记录；含人工阅读的默认/历史对照。 | 文档，无代码读取 |
| `simulation/dynanav/camera_rgbd.yaml` | 当前采集相机的 512×512 pinhole 内参基准（`cam_fx=cam_fy=292.5714…`、`cam_cx=cam_cy=256`、畸变 0、`use_sim_time: true`）。 | `run_dynanav_lio.sh` 在 `--mode livo` 下作为参数服务基线；随后用 `--capture-result` 的 `sensor_installation` 覆盖并校验；`--mode lio` 不使用 |
| `simulation/dynanav/fastlivo_lio.yaml` | ONLY_LIO 模式参数：`common.img_en: 0`、`allow_lio_only: true`，输入 `/dynanav/lidar_static` + `/dynanav/imu`，`evo.pose_output_en: false`；IMU/LIO/local_map 等阈值。 | `run_dynanav_lio.sh --mode lio` 拼成 `fastlivo_lio.yaml`；preflight 校验模式与 post-VIO 开关一致 |
| `simulation/dynanav/fastlivo_livo.yaml` | RGB 辅助 LIVO 模式参数：`img_en: 1`、`allow_lio_only: false`，增加 `/dynanav/camera`，post-VIO 输出开启；视觉参数是本轮实验设置，不是精度结论。 | `run_dynanav_lio.sh --mode livo`；同样经 preflight 一致性校验 |
| `simulation/dynanav/fastlivo_rtx_os1_32.yaml` | RTX 旋转 Ouster OS1 覆盖：`allow_ouster_lidar: true`、`lidar_type: 3`、`scan_line: 32`、`feature_extract_enabled: false`、`point_filter_num: 1`、identity LiDAR 外参；另有缓冲硬上限与 `same_pair_vio_handoff`。 | 只在 `--lidar-topic /dynanav/lidar_rtx` 时由 `run_dynanav_lio.sh` 追加为第二个参数文件；主入口在 SLAM 模式下把它复制进 run 的 `configuration/` |
| `simulation/dynanav/observer_udp_rx4m.xml` | FastRTPS 传输 profile：只声明 UDPv4 与 `receiveBufferSize=4194304`，不设发送缓冲、白名单、消息尺寸或 QoS。 | `scripts/dynanav_observer_transport.py` 的 `select_transport` 只在 auto + `rmw_fastrtps_cpp` + `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` + 无既有 DDS 配置时选中；由 `run_dynanav_lio.sh` preflight 导出给 observer 进程 |
| `simulation/dynanav_basement_collision.json` | `dynanav_basement_collision/v1` 契约：119 个已审计可见水平楼板 mesh 路径 + 4 条原始楼梯路径 + 地面平面路径，附 provenance、`floor_validation` 与 `support_preflight` 采样与容差。 | `session_collision.load_config`（`CONFIG_PATH = project_root()/configs/simulation/dynanav_basement_collision.json`）在 `capture.scene.open_stage` 的 `basement_collision` 分支执行会话修复；`experiment.py` 另读其 `provenance` 写入 profile，并把它列入 source snapshot |
| `simulation/dynanav_basement_to_ground_collision.json` | 同一 schema 的地下室→地面路线契约：楼层/楼梯路径、`route.waypoints_xyz`、推荐诊断速度等。 | **无运行期读取点**：只在 `experiment.py` 的 source snapshot 清单里被逐字节读取 + sha256 + 复制；`waypoints_xyz` 不由它驱动（默认航点硬编码在 `experiment.py`） |
| `visualization/README.md` | 人工 RViz 查看说明：命令、`ROS_DOMAIN_ID`、QoS/frame 约定与「显示证据 ≠ 导航证据」的边界。 | 文档，无代码读取 |
| `visualization/dynanav_map.rviz` | RViz2 配置：`Fixed Frame=dynanav_lio_map`、PointCloud2 显示 `/dynanav/review/static_map`（Reliable + Transient Local、Keep Last depth 1）、Z 轴着色、0.03 m Flat Squares、Orbit 视角。 | 只有人工 `rviz2 -d configs/visualization/dynanav_map.rviz`；无 Python/ROS 节点自动加载 |

## 真实读取点（以代码为准）

| 读取方 | 读取什么 | 性质 |
| --- | --- | --- |
| [`scripts/run_dynanav_lio.sh`](../scripts/run_dynanav_lio.sh) | `configs/simulation/dynanav/fastlivo_lio.yaml` 或 `fastlivo_livo.yaml`（按 `--mode`）、`camera_rgbd.yaml`、`fastlivo_rtx_os1_32.yaml`（RTX LiDAR 时） | 拼 `--params-file` 启动外部 observer；`--config-dir` 可改指 run 内冻结副本 |
| [`scripts/dynanav_observer_transport.py`](../scripts/dynanav_observer_transport.py) | `configs/simulation/dynanav/observer_udp_rx4m.xml`（常量 `PROFILE`，相对 `repo` 解析） | 纯函数 `select_transport` 选择传输 profile 并写 hash；不修改环境 |
| [`scripts/run_dynanav_native_exploration.sh`](../scripts/run_dynanav_native_exploration.sh) | SLAM 模式下复制上述三份 YAML 到 `<run>/configuration/`；`--truth-pose` 模式不复制 | 冻结本次运行使用的配置副本 |
| [`src/embodied_navigation/simulation/dynanav/session_collision.py`](../src/embodied_navigation/simulation/dynanav/session_collision.py) | `configs/simulation/dynanav_basement_collision.json` | 运行期读：加载楼层/楼梯路径、校验 schema、会话内恢复碰撞 |
| [`src/embodied_navigation/simulation/dynanav/experiment.py`](../src/embodied_navigation/simulation/dynanav/experiment.py) | 两个 `configs/simulation/dynanav_*.json` | `dynanav_basement_collision.json` 的 `provenance` 被写进 profile；两者都进 `source_snapshot/` + `source_manifest.json`（sha256） |
| 人工 | `configs/visualization/dynanav_map.rviz` | `rviz2 -d` 手动加载，无自动读取点 |

除上表外没有其他代码读 `configs/`。一个常见误判是把子目录 README 里提到的历史实验约定当成当前生效配置：本目录不提供模块清单合并、无 include 继承、无 prepare 生成的运行期 profile，因此改一个 YAML 只影响上面列出的读取方。

## 验证与测试入口

```bash
PYTHONPATH=src python3 -B -m compileall -q src scripts
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

直接覆盖本目录契约的测试：

| 测试 | 覆盖 |
| --- | --- |
| `tests/regression/test_dynanav_lio_launcher.py` | `run_dynanav_lio.sh` 对 `fastlivo_lio.yaml` / `fastlivo_livo.yaml` / `camera_rgbd.yaml` / `fastlivo_rtx_os1_32.yaml` 的选取与 preflight |
| `tests/unit/test_dynanav_rtx_source.py` | `fastlivo_rtx_os1_32.yaml` 的 ROS 参数（`allow_ouster_lidar`、type 3、32 线等） |
| `tests/unit/test_dynanav_observer_transport.py` | `observer_udp_rx4m.xml` 的选择条件与 hash 记录 |
| `tests/unit/test_dynanav_source_snapshot.py` | 两个 collision JSON 的 source snapshot 行为 |
| `tests/unit/test_pct_grid_budget.py`（经 `pct_budget_acceptance_fixtures.py`） | 主入口复制三份 YAML 到 run `configuration/` 的清单 |
| `tests/regression/test_dynanav_session_collision.py` | `dynanav_basement_collision.json` 的 schema、路径与支撑 preflight |

## 修改约定与边界

- 改数值前先确认读取点与所属层；本目录的阈值不会因为「看起来一致」而跨层统一。
- 两份 collision JSON 是**契约数据**，不是场景资产：`floor_paths`/`stair_paths` 必须指回真实且可见的原始 mesh，改路径会直接让会话修复或 support preflight 失败。
- `dynanav_basement_to_ground_collision.json` 目前只有 snapshot 作用，其 `route.waypoints_xyz` **不驱动**导航；不要把它当成路线配置来改。
- 子目录 README（`simulation/dynanav/README.md`、`README_LIO.md`、`visualization/README.md`）与本文件同属文档层，不是运行期配置；其中若出现过期行数/路径，以源码与本文件为准。
