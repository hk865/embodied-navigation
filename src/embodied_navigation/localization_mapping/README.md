# localization_mapping

本目录是导航系统的定位与建图层：把 FAST-LIVO2 的里程计和点云接入导航坐标系，提供已知场景/地图辅助定位，归档本次运行观测到的地图，并给出可选的异步后端配准。导入本包不启动节点、不建线程、不写文件；有副作用的是各 `main()` 入口。

## 边界

- 不包含 FAST-LIVO2 算法本体。真实算法与 ROS 包在 [third_party/fast_livo2/fast_livo2_ros](../../../third_party/fast_livo2/fast_livo2_ros/README.md)，本目录只做接入、包装、校正与状态发布。
- 不把仿真真值当定位输入；bundle 和状态都显式携带 `robot_truth_pose_used=false` 或 `navigation_ground_truth_used=false`。`semantic_mapping` 里的 `sim_oracle` 是显式诊断来源，不重标为 SLAM/learned。
- 不负责规划、控制和任务状态机，也不直接生成导航命令。`known_scene_hold_guard.py` 只提供"零保持/恢复"记账函数，真正的停走与碰撞门禁仍在 `control/parking`、`navigation/base_node.py`。

## 目录结构

```text
src/embodied_navigation/localization_mapping/
├── backend/                        # 可选异步配准后端，见 backend/README.md
├── frontend/                       # FAST-LIVO2 接入资料副本，见 frontend/README.md
├── launch/                         # 本模块 ROS 进程装配，见 launch/README.md
├── mapping/                        # 运行期观测地图归档，见 mapping/README.md
├── semantic_mapping/               # 纯 CPU 语义点层与查询，见 semantic_mapping/README.md
├── README.md                       # 本文件
├── __init__.py                     # 仅模块 docstring（1 行）
├── building_runtime.py             # Building 唯一校正 owner：配对 LIO 位姿/点云并应用一个 SE3（269 行）
├── building_sensors.py             # Building LiDAR 自回波过滤，几何与原始 header（171 行）
├── feedback_state.py               # 已接受 SLAM commit 的规范状态字典构造器（33 行）
├── geometry.py                     # 前端共享的坐标/旋量代数（96 行）
├── known_scene_adapter.py          # 同源时间槽配对 + 有界后台匹配线程（646 行）
├── known_scene_hold_guard.py       # 零保持观测与 Teacher 恢复记账，供导航/停车复用（445 行）
├── known_scene_inputs.py           # 导航节点的已知场景输入 mixin：合流、提交与交接（1138 行）
├── known_scene_matcher.py          # 对显式静态场景图做 CPU 匹配与门禁（713 行）
├── known_scene_reference_builder.py # prepare 阶段从 SDF 静态几何生成参考图（81 行）
├── localization_runtime.py         # 已知场景地图辅助定位节点，发布 corrected 话题（429 行）
├── odom_adapter.py                 # FAST-LIVO2 IMU 状态 → body/lidar 里程计与有效 twist（237 行）
├── point_provenance.py             # 点/commit 身份编码、校验与几何证据打包（397 行）
├── relative_odometry.py            # 一次性对齐的相对 SLAM 里程计（独立 relative-slam 入口用，61 行）
└── sensor_snapshot.py              # 实时 SLAM 位姿/点云按源时间与 frame 配对（185 行）
```

直接 Python 文件 15 个、合计 4902 行（2026-11-09 磁盘实测，`wc -l`）；最大文件 `known_scene_inputs.py` 1138 行。5 个子目录各有自己的 README，本文件不递归展开。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（ROS 节点） | `odom_adapter.py`、`localization_runtime.py`、`building_runtime.py`、`building_sensors.py` | 都有 `main()`，由 launch 以 `python3 -m …` 启动；导入这些模块不会建节点（节点类在 `main()` 内或 `__init__` 里显式构造）。 |
| 代码（库/契约） | `known_scene_matcher.py`、`known_scene_adapter.py`、`point_provenance.py`、`sensor_snapshot.py`、`geometry.py` | 纯函数/类，无 ROS I/O；`known_scene_adapter.py` 例外：`MatcherWorker` 会起一个后台线程。 |
| 代码（host 扩展） | `known_scene_inputs.py`、`known_scene_hold_guard.py` | 不是独立节点：前者是被 `navigation/pid_entrypoint.py` 混入的 `KnownSceneInputMixin`，后者是一组接收 `node` 的函数。 |
| 代码（prepare 期） | `known_scene_reference_builder.py` | 由 `launch/prepare.py` 生成 `known_scene_reference.json`，运行期只被 `localization_runtime` 读取该 JSON。 |
| 代码（独立入口配套） | `relative_odometry.py`、`feedback_state.py` | 前者只被 `navigation/relative_slam_node.py` 使用；后者只被 `navigation/pid_ingress.py` 调用。 |
| 日志/输出 | `building_runtime.py` → run 目录 `slam_status.json`、`slam_poses.jsonl`、`correction_applications.jsonl` | `slam_status.json` 原子替换（`.tmp` + `replace`），`navigation/building_navigation.py:469` 读它的 `pairs` 做门禁；两个 `.jsonl` 以 `"x"` 模式打开，同一 run 目录重跑会直接失败。`slam_poses.jsonl` 在 `src/`+`scripts/` 里没有读取方，是运行证据。 |
| 日志/输出 | `localization_runtime.py` → run 目录 `known_scene_status.json` | 每个 tick 原子写一次，内容含 `schema="known_scene_navigation_status/v1"`、`generation`、`bundles_published`、`robot_truth_pose_used=false`。仓库内没有代码读取它（grep 只命中写入处），属证据/排障文件。 |
| 日志/输出 | `/demo/navigation/localization_bundle`、`/demo/navigation/localization_status` | 运行期契约话题（`String` JSON），分别由 `localization_runtime` 发布、被导航侧消费；不是日志文件。 |
| 日志/输出 | 无日志框架 | 本目录 `grep -rn "import logging\|getLogger\|print("` 命中 0：既不写 Python 日志，也不 print；诊断一律走 status JSON/话题或 run 目录 JSONL。进程 stdout/stderr 由 launch 的 `ExecuteProcess(output="screen")` 收走。 |
| 配置 | 无运行期 `configs/` 读取点（`odom_adapter` 默认值除外） | `odom_adapter.py --scenario` 默认指向 `configs/simulation/scenario.json`，但生产 launch 总是传 `run/navigation_scenario.json` 覆盖它；这是唯一直接指向 `configs/` 的默认值。 |
| 配置 | `runtime_context.json` | `localization_runtime.main()` 读 `profile`；`launch/components.py` 读 `profile` 和 `path("ros_prefix")`。读取顺序与 pin 规则见 [configs/README.md](../../../configs/README.md)。 |
| 配置 | run 目录的 prepare 产物 | `localization_runtime` 读 `navigation_scenario.json`、`known_scene_reference.json`、`navigation_status.json`、`navigation_command.json`；`building_runtime` 读 `experiment_setup.json`、`sensor_contract.json`、`backend_result.json`；`building_sensors` 读 `experiment_setup.json`、`sensor_contract.json`。这些文件都由 `launch/prepare.py` 或入口进程生成，本目录只读。 |
| 配置（常量） | `common/integration_limits.py` | 本目录的门限常量是 import 的，不是本地魔法数：`KNOWN_BUNDLE_FRESHNESS_NS=300 ms`、`MATCH_CORRECTION_MAX_AGE_NS=3 s`、`SOURCE_JOIN_SLOT_MAX_AGE_NS=3 s`、`RAY_ABS_P95_LIMIT_M=0.35`、`LIDAR_BODY_TRANSLATION_M=(0.2, 0.0, 0.1177)`。 |
| 配置 | `configs/localization_mapping/{camera,fastlivo}.yaml` | 不是本目录读的：`launch/prepare.py` 把它们转写成 run 目录的 `navigation_camera.yaml`/`navigation_fastlivo.yaml`，再由 `fastlivo_mapping` 二进制读。 |

## 模块内文件耦合

依赖方向：四个 ROS 节点（`odom_adapter`、`localization_runtime`、`building_runtime`、`building_sensors`）基本是"叶子"——只 import 本目录的库文件，外加 `common/*`、`project` 与 `perception/sensors/*` 的少量工具；库文件之间是**单向**的 `localization_runtime → {sensor_snapshot, known_scene_matcher}`、`known_scene_inputs → {known_scene_adapter → {known_scene_matcher, point_provenance}, known_scene_hold_guard}`。包内不存在 import 环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `known_scene_inputs.py` | 1138 | `known_scene_adapter`、`known_scene_hold_guard`、`known_scene_matcher`、`point_provenance`、`common/*`、`perception/sensors/*`、**`control/parking/{causal_join,terminal_owner}`** | `navigation/pid_entrypoint.py`（作为 mixin 混入） | 偏重/可疑：>800 行且跨层 import |
| `known_scene_matcher.py` | 713 | `common/integration_limits` | `known_scene_adapter`、`known_scene_inputs`、`localization_runtime`、`planning/admission/physical_gate`、`planning/scan_epoch_supervisor` | 可接受但偏重 |
| `known_scene_adapter.py` | 646 | `known_scene_matcher`、`point_provenance`、`common/integration_limits` | `known_scene_inputs` | 偏重（`_check` 176 行、`_run` 109 行） |
| `known_scene_hold_guard.py` | 445 | **`control/parking/transition_gate`**（函数内延迟 import） | `control/parking/terminal_owner`、`known_scene_inputs`、`navigation/{base_node,pid_support,ros_node}` | 可疑：双向跨层 + 私有 `_frame` 被外部直接 import |
| `localization_runtime.py` | 429 | `known_scene_matcher`、`sensor_snapshot`、`common/*`、`perception/sensors/async_writers` | 无 import 方（launch 以 `-m` 启动） | 清楚但 `main()` 272 行 |
| `point_provenance.py` | 397 | 标准库 + NumPy | `known_scene_adapter`、`known_scene_inputs`、`odom_adapter`、`perception/sensors/self_echo_filter` | 清楚 |
| `building_runtime.py` | 269 | `backend/correction`、`backend/geometry` | 无 import 方（`launch/building.py`、`navigation/robogauge/benchmark.py` 以 `-m` 启动） | 清楚 |
| `odom_adapter.py` | 237 | `point_provenance`、`geometry`、`project` | 无 import 方（launch 以 `-m` 启动） | 清楚 |
| `sensor_snapshot.py` | 185 | `common/integration_limits` | `localization_runtime` | 清楚 |
| `building_sensors.py` | 171 | 仅 rclpy/scipy | 无 import 方 | 清楚 |
| `geometry.py` | 96 | 仅 NumPy/SciPy | `mapping/map_archive`、`odom_adapter`、`perception/sensors/self_echo_filter` | 清楚 |
| `known_scene_reference_builder.py` | 81 | 仅标准库 + SciPy | `launch/prepare.py`（prepare 期一次） | 清楚 |
| `relative_odometry.py` | 61 | **`navigation/task.py`**（取 `Observation` 数据类） | `navigation/relative_slam_node.py` | 清楚但依赖方向反直觉 |
| `feedback_state.py` | 33 | 无 | `navigation/pid_ingress.py` | 清楚 |
| `__init__.py` | 1 | 无 | 无 | 清楚 |

四类问题（逐条给证据）：

- **上帝文件**：`known_scene_inputs.py` 1138 行、30 个方法、单函数最长 187 行（`_r86_commit_feedback`）。它同时承担订阅包装、时钟策略、R86/R92/R99/R104 状态机、反馈提交与 map-frame 交接，是典型的"按历史轮次堆叠的 mixin"。
- **重复实现**：PointCloud2 的 XYZ 解码有 3 份近似实现——`point_provenance.xyz`（无有限性检查）、`sensor_snapshot.xyz`（多的 `np.isfinite` 检查）、`known_scene_adapter.pointcloud_xyz`（另写一遍）；`checked_transform` 有 2 份且容差不一致（`known_scene_matcher.py:28` 末行 `atol=1e-10` vs `backend/geometry.py:13` 末行 `atol=1e-8`）。
- **跨层直达**：`known_scene_inputs.py:72-77` 直接 import `control.parking.causal_join`/`terminal_owner`；`known_scene_hold_guard.py:252,263` 在函数内 import `control.parking.transition_gate.TeacherHeadingGate`；反向 `control/parking/terminal_owner.py:6` 又 import `known_scene_hold_guard._frame`。于是**包级依赖成环**：`localization_mapping ↔ control.parking`（文件级无 import 环，因为一侧是延迟 import）。同理 `relative_odometry.py:8` 让 `localization_mapping → navigation.task`，而 `navigation/*` 反过来 import `known_scene_hold_guard`/`known_scene_inputs`/`feedback_state`。`localization_runtime.py:168` 还直接 import `perception.sensors.async_writers`。
- **隐式契约**：`KnownSceneInputMixin` 假定宿主节点已有 `self.run`、`self.profile`、`self.pose_timeout`、`self.cloud_timeout`、`_admit_matched_spline` 等属性，并用 `getattr(self, "profile", None)` 兜底；`pid_entrypoint.py` 还在运行时替换 `IntegratedNavigation._known_generation_ready`。`known_scene_hold_guard._frame()` 返回 `(generation, C_world_odom, T_navigation_world)` 三元组、`BINDING_KEYS` 十个字符串键，都是靠调用顺序和 dict key 维持的约定，没有类型或协议声明。

结论：**局部清楚、包级偏重**。节点→库的单向分层是好的（四个节点互不 import），纯函数文件的 docstring 与命名也规范；问题集中在"历史轮次 mixin 长大"和"localization ↔ control/navigation 双向 import"两处，属于结构债而非乱写。

## 可读性评估

- **良好**：`geometry.py`（函数 docstring 覆盖 1.0，5 个函数最长 43 行）、`feedback_state.py`（1.0，26 行单函数，docstring 明确写"不是 truth→SLAM 转换器"）、`known_scene_reference_builder.py`（1.0，81 行，`build()` docstring 说明只取静态 box）、`point_provenance.py`（0.312 但 16 个函数最长 71 行，模块 docstring 点明"几何是独立门禁"）、`sensor_snapshot.py`（0.545，最长 45 行）。
- **可接受**：`known_scene_matcher.py`（713 行 / 17 函数 / docstring 0.118；核心 `score()` 189 行、`match()` 154 行各承担"打分所有门限"和"假设+精配准+接受判定"，门限集中在 `MatchPolicy` dataclass 里、有名字，这是它比 `known_scene_inputs` 好读的主要原因）、`localization_runtime.py`（429 行，docstring 0.273，但 `main()` 272 行内嵌整个 `Localization(Node)` 类，读它必须一口气看完）、`odom_adapter.py`（237 行，**函数 docstring 0.0**，`receive()` 82 行含帧检查/时间单调/外参换算/twist 有效性）、`building_runtime.py`（269 行，docstring 0.1，`commit()` 83 行同时处理位姿、点云、scan twist、TF 与 JSONL）。
- **需改进**：`known_scene_inputs.py`——函数 docstring 0.1（30 个函数只有 3 个有），最长 187 行；文件头 80 行里 `from … import` 与赋值交替出现（`_observer_api` 同一句重复 import 10 次），`import json, time, math, copy, hashlib` 夹在 from-import 中间；Rxx 编号命名（`_r86_*`/`_r92_*`/`_r99_*`/`_r104_*`）对没读过历史轮次的人不可检索。`known_scene_adapter.py` 有 4 处 `except Exception: pass`（`known_scene_adapter.py:587,598,618,630`，其中 630 带注释"no observer exception replaces original result"），诊断信息在诊断捕获路径上被静默丢弃。

区分结构与文档：`odom_adapter`、`building_runtime`、`map_archive` 等是"缺 docstring/函数偏长"的**文档问题**（它们的分层和命名是清楚的）；只有 `known_scene_inputs.py` 同时有**结构问题**（1138 行 mixin + 跨层 import）。指标抽查：用 AST 复算 `backend/run.py`（4 函数、docstring 0.0）、`odom_adapter.py`（6 函数、0.0）、`known_scene_inputs.py`（30 函数、3 个 docstring、最长 187 行）与事实包一致。

## 重构建议

本轮只写建议、不改代码；任何一条都应"先补测试/对照再动"。本目录的 `known_scene_*` 链路在 `tests/` 里**没有直接用例**（`grep -rn "known_scene" tests/` 命中 0），所以这里的 P1 都标了"先补测试"。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `known_scene_inputs.py` 按轮次职责拆成 3 个 mixin（订阅/时钟策略、反馈提交、map-frame 交接），保留 `KnownSceneInputMixin` 作为组合门面 | 1138 行 30 方法无法单点修改；`_r86_commit_feedback` 187 行是主路径的反馈提交 | 高：`pid_entrypoint.py` 按方法名 monkeypatch（`_known_generation_ready`），拆分必须保留同名转发 | 先加一个不启 ROS 的契约测试（对假 node 调用 `_known_generation_ready`/`on_localization_bundle` 的最小桩），再逐块搬 |
| P1 | 断开 `localization_mapping ↔ control.parking` 的包级环：把 `_frame()`/`TeacherHeadingGate` 依赖改为显式参数或 protocol | 现在 `terminal_owner` import `_frame`（私有函数），`hold_guard` 又延迟 import `transition_gate`；`import` 顺序稍变就可能踩到半初始化模块 | 中：两侧都有真实调用者（停车/导航） | 用 `python -c "import embodied_navigation.control.parking.terminal_owner"` 与反向单独 import 各跑一次；补 `tests/unit` 的导入顺序用例 |
| P1 | 统一 3 份 PointCloud2 XYZ 解码为 `point_provenance.xyz` 一份（保留最严的有限性检查），`sensor_snapshot`/`known_scene_adapter` 改为调用 | 三份实现的错误消息与校验强度不同，修 bug 要改三处 | 中：热路径性能与错误文案变化 | 用一份录制的 PointCloud2 夹具分别调用三份解码，断言逐位相同（`tests/` 目前没有直接覆盖解码的用例，需新增一个）；再跑现有的后端回归确认没有改变行为 |
| P2 | 给 `odom_adapter.receive()`、`building_runtime.commit()`、`localization_runtime.main()` 补 docstring/拆内嵌类 | 三个入口是接手者第一眼要读的代码，却是 docstring 最少的地方 | 低（纯文档/等价搬移） | 只改注释时人工复核；搬函数时用 `test_building_entry_closure` + `test_production_slam_mode` 的 AST 断言 |
| P2 | `known_scene_adapter.py` 的 4 处 `except Exception: pass` 改为计数/记录到 result 字段 | 静默吞异常会让"为什么没校正"无从排查 | 低：只加诊断 | 注入会抛异常的假 matcher，断言 result 里出现 `diagnostic_errors>0` |
| P2 | 在文件头注明 Rxx 编号含义（或集中一张编号→职责表） | `_r86_*`/`_r92_*`/`_r99_*`/`_r104_*` 是本目录最重的隐性词汇表 | 无 | 人工复核 |

没有 P0：本目录不直接发布控制命令，所有校正都经过 `CorrectionState`/`BackendResultReader` 的 schema+年龄+epoch 检查，未发现"能靠改文档绕过安全门"的正确性缺口。

## 组成

- [backend/README.md](backend/README.md)：异步配准、关键帧历史、回环验证与 SE(3) 图优化。
- [frontend/README.md](frontend/README.md)：FAST-LIVO2 的 ROS 接入资料。
- [launch/README.md](launch/README.md)：本模块的 ROS 进程装配。
- [mapping/README.md](mapping/README.md)：运行期观测地图归档。
- [semantic_mapping/README.md](semantic_mapping/README.md)：纯 CPU 的动静分层、对象 ID、历史与结构化查询；显式保存 SLAM/oracle 来源，目前由 DynaNav 诊断接入，不代表生产语义感知已完成。
- 顶层节点：[odom_adapter.py](odom_adapter.py) 把 `/aft_mapped_to_init` 的 IMU 状态换算成 body/lidar 里程计与有效 twist；[localization_runtime.py](localization_runtime.py) 做已知场景地图辅助的实时定位与 bundle 发布；[building_runtime.py](building_runtime.py) 是 Building 唯一的校正 owner；[building_sensors.py](building_sensors.py) 过滤 LiDAR 自回波。
- 支撑模块：`geometry.py`（坐标代数）、`sensor_snapshot.py`（同源时间配对）、`point_provenance.py`（点与 commit 身份校验）、`known_scene_*.py`（场景参考、匹配、输入与保持门）、`relative_odometry.py`、`feedback_state.py`。

## 输入输出

前端输入是 `/aft_mapped_to_init` 与注册点云。注意同名话题在两张图里是不同东西：`odom_adapter` 节点默认发布 `/demo/slam/{body,lidar}_odom`，但默认 launch 把它 remap 成 `/demo/slam/raw_*`；真正的 corrected `/demo/slam/{body,lidar}_odom` 由 `localization_runtime` 发布。完整输出还有 `/cloud_registered_full`、`/demo/navigation/localization_bundle`、`/demo/navigation/localization_status`。名词定义见 [docs/glossary.md](../../../docs/glossary.md) 的 raw/corrected 与 provenance 条目。

Building 路径改读 `/building/raw_body_odom`、`/building/raw_cloud`，并把状态写到运行目录的 `slam_status.json`、`slam_poses.jsonl`、`correction_applications.jsonl`。四条执行路径与进程图见 [docs/architecture.md](../../../docs/architecture.md)；本目录只覆盖其中的定位/建图角色。

## 测试与限制

- 后端有专用用例 [tests/regression/test_backend_flow.py](../../../tests/regression/test_backend_flow.py)、[test_backend_registration.py](../../../tests/regression/test_backend_registration.py)、[test_backend_auto_loops.py](../../../tests/regression/test_backend_auto_loops.py)；语义层有 [tests/unit/test_layered_mapping.py](../../../tests/unit/test_layered_mapping.py)。
- 顶层 ROS 节点没有专用 unittest：[tests/unit/test_building_entry_closure.py](../../../tests/unit/test_building_entry_closure.py) 与 [tests/unit/test_production_slam_mode.py](../../../tests/unit/test_production_slam_mode.py) 覆盖的是启动/导航侧的模式与入口契约，不是本层算法。`known_scene_*` 与 `odom_adapter`/`localization_runtime`/`building_runtime` 在 `tests/` 中无直接断言（`grep -rn "known_scene\|odom_adapter\|building_runtime" tests/` 无命中），只能静态阅读。
- `fastlivo_mapping` 二进制来自 `install/ros`，缺失只表示本机未构建；`feedback_state` 的 `pose_producer="fast_livo2_async_backend"` 是**标签**，仓库内无法确认它指 C++ 内部异步后端还是 `backend/` 的 Python 后端，不要凭标签推断进程存在（未核实）。
- 本目录没有做端到端验收：没有真实 FAST-LIVO2 吞吐、没有 GPS/回环全局一致性、没有实机验证；本轮只读代码与磁盘，未运行 ROS。
