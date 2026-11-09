> Root 最终复验已完成：54 项单元通过；回归 57 通过、5 项缺少 Torch 跳过；304 项当前来源指纹与原 VIO 补丁检查通过，PCT CPU 编译/合成路径通过。以下保留 DSH 分工阶段记录，其中“待 root 刷新/复验”已由[最终验证记录](../verification.json)接续；完整 ROS/仿真/实机/训练仍未验证。

# owner=project 发现与阈值/校验审计

本文件汇总 310 个代码/配置目标与 84 个只读 reference 的审阅发现。**本轮没有修改任何公共接口、算法、变量/函数名、操作数/短路/调用顺序、常量、阈值、异常文本、I/O 行为、配置键和值，也没有删除校验或放宽阈值**；以下均为“记录 + 证据 + 建议”，是否需要修改需后续单独任务确认。所有生产目标的语义等价已由 AST/令牌/配置解析检查证明（见 [summary.md](summary.md)）。

## 1. 测试脆弱依赖与唯一一处测试 AST 变更

- `tests/unit/test_building_entry_closure.py`：原两条断言依赖 `src/embodied_navigation/launch/building.py` 的源码字符串拼写，`ruff format` 重排该生产文件后失效。已改为等价的 AST/结构断言（`start("resources", [...])` 的 `-m` 相邻模块名；`ROOT / <str>` 的 planner 配置相对路径），保留原有 `is_file()` 与 `assertNotIn` 断言，未删断言/加 skip/放宽容差。属于本轮唯一的测试 AST 有意变化，详见 summary.md 单列。
- 建议：后续同类“源码字符串断言”统一用 AST 或显式结构断言，避免格式化触发假失败。

## 2. 可能缺陷 / 不一致（仅记录，未改行为）

| 文件 | 证据 | 建议 |
| --- | --- | --- |
| `src/embodied_navigation/localization_mapping/known_scene_adapter.py` | `__import__("point_provenance").message_sha(...)`（385–388 行）在包导入下顶层模块名不存在，实际路径为 `embodied_navigation.localization_mapping.point_provenance`；405/408 行 `unique_count = len(np.unique(indices))` 重复赋值 | 改为包限定/局部 import；删除重复赋值。运行到该分支时可能 `ModuleNotFoundError` |
| `src/embodied_navigation/common/observation.py` | `from_sdf` 返回 `source_sha256=None`，而 provider/extra 逻辑期望有值 | 确认调用方是否需要该字段；只记录 |
| `src/embodied_navigation/simulation/terrain/provider.py` | `_goal_module` 先 `json.loads` 再 `_frozen` 校验 | 建议先校验字节再解析，避免未校验输入进入解析 |
| `src/embodied_navigation/simulation/gazebo/robogauge/motor_src/go2hv_actuator.cpp` | `saturation_effort`（默认 23.5）被解析并写入配置回执，但力矩钳位只用 `baseLimit(20.2/23.4)` | 疑似 dead parameter；确认是否有意保留 |
| `configs/robogauge/contract.json` vs `configs/scenarios/building/student_profile.json` | `gazebo_command_guard.absolute_limits=[0.6,0.2,0.3]`，而 Student cascade `command_limits=[0.6,0.2,0.6]`，yaw 上限 0.3 vs 0.6 | 两者属不同链路/守卫层；建议文档写明各自作用域，避免误读 |
| `configs/launch/settings.json` | `R107_SCAN_TRANSPORT_PROBE=1`、`MATCH_QUERY_CAPTURE=1`，但 `launch/main.py` 的 `environment()` 启动时强制覆盖为 `'0'` | 文档说明“settings 意图值 vs 运行时强制值”分层 |
| `configs/robogauge/routes/combined_floor1_inside.json` | `start_node [0,162,27]` 与 `cells_layer_row_col[0] [0,27,162]` 轴序疑似 row/col 转置（goal 同理） | 与消费端确认轴序；数据未改 |
| `src/embodied_navigation/navigation/building_navigation.py` | `native_rows()`/`process_clouds()` 无条件早退使 JSONL 回放分支不可达；`publish_pose()` 依赖 `__init__` 从未创建的 `bodypub/sensorpub/cloudpub/scanpub`；`slam_pose` 恒 `accept_pose(row, False)` | 保留的 legacy 死路径；已就地注释，建议后续单独立项删除或显式标 deprecated |
| `src/embodied_navigation/common/time/executor_selection.py` | 定义 `KEY`/`enabled()`，但 `make_executor` 直接返回 `base(num_threads=...)`，不读 profile | default-off 选择器实现；确认是否预留接口 |
| `src/embodied_navigation/perception/sensors/async_writers.py` | `validated_transport_baseline` 的 `references` 参数未使用；模块导入期 env 不符即 `ValueError` | 清理未用参数；确认导入期校验是否为预期设计 |
| `src/embodied_navigation/robot/unitree/policy/native/worker.py` | `command_expired`/`sensor_evidence` 恒为常值；`QDEFAULT` 字面量重复 contract 默认值 | 确认是否为占位；避免契约默认值两处漂移 |

## 3. 死代码 / 未引用符号 / 重复硬编码（仅记录）

- `src/embodied_navigation/control/parking/parking_contract.py`：`LEGACY_MODE`/`LEGACY_SPEC` 全仓无引用（v1 契约不再被运行时选用，当前用 `MODE`/`SPEC`）；已注释未删。
- `src/embodied_navigation/localization_mapping/known_scene_hold_guard.py`：`ARRAY_LIMIT_B`/`BUFFER_LIMIT_B`/`WINDOW_NS` 全仓无引用。
- `tests/regression/test_robogauge_route_tracker.py`：未使用的 `json`、`pathlib` 导入（保留未动，避免非格式化改动）。
- `src/embodied_navigation/navigation/pid_navigation.py`、`src/embodied_navigation/localization_mapping/known_scene_inputs.py`：多处重复 `from ... import api as _observer_api`（同别名重绑定，基线风格）；为保持导入顺序未改。
- `src/embodied_navigation/planning/admission/async_admission.py`：第 15/21 行重复 `import common.observation.api as _observer_api`（同别名，无害）。
- `src/embodied_navigation/control/parking/causal_join.py`：内联 `__import__("json")`，语义正确但可读性偏低。
- `src/embodied_navigation/control/parking/lower_loop_ack.py`：硬编码 `CONTRACT_SHA`/`EXPORT_SHA`/`CHECKPOINT_SHA` 与 PID 校正裁剪 `±[0.08,0.10,0.08]`；属冻结契约。
- `src/embodied_navigation/localization_mapping/point_provenance.py`：默认 mount `(0.2,0,0.1177)` 与 `configs/robogauge/sensor_contract.json` 的 lidar `body_xyz` 重复；`0.006/0.030` 残差门限也在 adapter 重复硬编码，存在漂移风险。
- `src/embodied_navigation/control/tracking/planned_step_filter.py`：`+1.3` 窗口与 `ConnectedRouteProgress(forward_arc=1.3)` 重复。
- `src/embodied_navigation/navigation/mission/mission46_guard.py`：硬编码新鲜度 `300000000` ns 与 `0.3` s（`<= 300000000`、`<= 0.3`），建议提为命名常量。
- `src/embodied_navigation/localization_mapping/backend/history.py`：环闭合准入 `0.55/0.15/0.025` 内联未提为常量。

## 4. 阈值 / 校验审计结论

- **本轮未修改任何阈值，也未删除任何校验**；AST/令牌/配置语义检查证明常量、比较符、短路与求值顺序未变（见 summary.md 验证表）。
- 逐文件审计未发现需要为了可读性删除的校验。以下为“内联/重复但语义必要或需数据才能判断”的候选，均保持原行为：
  - `src/embodied_navigation/planning/scan_child_consumption.py`：ACK 证据门槛（`<=4096` 字节、16 行窗口、300 ms、`>=2` 匹配 stamp）均有职责；`GID_BYTES=16` 已注释绑定 installed Jazzy rmw。
  - `src/embodied_navigation/navigation/robogauge/analyze.py`：`1e-12` qtarget / `1e-9` torque 与 `test_robogauge_contract` 一致；Go2HV `20.2/23.4 Nm`、`(30-|qd|)/16.5` 降额未放宽。
  - `src/embodied_navigation/localization_mapping/known_scene_matcher.py`：三个门限写在 `match` 体内而非 `MatchPolicy`——`valid.sum() < min(100, len(local)//2)`、`valid = d < 0.1`、rms cap `0.5`；建议后续提为策略常量，但需数据确认边界。
  - `src/embodied_navigation/localization_mapping/known_scene_hold_guard.py` 的未引用常量、`src/.../known_scene_adapter.py` 的重复赋值等见第 3 节。
- 历史审计的“候选但未修”项仍成立且本轮未改：Building `--duration` 缺 `math.isfinite`（`src/embodied_navigation/launch/building.py` 相关行）、`--check` 提前 return 跳过部分参数校验、R97 `queue_budget` 观察桩只拒非 2 MiB、时钟配置漂移、探针开关单一事实源。依据见只读参考 `docs/review/validation-and-threshold-audit.md`（行号固定旧 HEAD `b44f748d`，本轮未重写）。

## 5. 配置 / 数据 / 模板观察（未改值）

- `configs/robogauge/routes/single_floor_turn.json`：数据字段 `execution_success=false`、`length_m=9.960249544818957`、raw PCT 高度全为 `3.083984375`；属既有路线数据。
- `configs/robogauge/world.sdf.in`：模板实际只有 `${ASSET_ROOT}` 与 `${ROBOGAUGE_ACTUATOR}` 两个占位符，由 `simulation/gazebo/robogauge/world.py` 在 ET 解析前替换（`${PROJECT_ROOT}` 出现在 Building 自己的模板替换路径，不在本文件）；仅新增单行 XML 注释（多行注释会被 `significant_lines` 计入显著行；未改元素/属性/文本）。
- `configs/robogauge/isaac_first_flight_reference.json`：`bypassed_gates` 显式绕过 RGBD/SLAM 门、`navigation_pose: live native64 ground truth, NOT SLAM`、`stall_stop_enabled=false`；文件自述 archived 配方，非 live launch，只记录。
- `src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py`：独立无头 Isaac Sim 探针（`PROBE_STEPS` 默认 4400），补模块 docstring，未改探针逻辑。
- `src/embodied_navigation/simulation/terrain/mission46.py`：`check_switch()` 返回值反直觉（成功提交返回 `False`；已锁存失败/拒绝切换返回 `True` 并锁存 failure），已写入 docstring 防止误读。
- `src/embodied_navigation/localization_mapping/backend/native/graph_solver.cpp`：robust loss 仅对 loop 边且 `n>2` 生效，非 loop 边仍平方损失；已补注释，未改语义。
- `src/embodied_navigation/common/communication/writer_qos.cpp`：补 `out[0..13]` 槽位与返回码 `0/1/2/3/4`、`-1/-2/-3/-4` 注释，并按 100 列人工重排；`cpp_tokens` 令牌序列等价。

## 6. 只读参考（84 个 reference）中的历史/一致性发现

这些文件本轮**未修改**，逐字节 sha256 与基线一致；下列差异是“历史回执 vs 当前工作树”的并发/时间差，不是回执被篡改：

- `docs/dependencies/vendored-source-manifest.json`：304 条（`modified=false` 302、`true` 2）；当前工作树重算仅 294 条 destination 匹配，10 条已被其他任务本轮的第三方源码可读性改动（8 个 `third_party/livox_ros_driver2/launch_ROS1/*.launch` 与 `docker/private_dds/{build_private_dds.py,common.py}`）。manifest 本身未改，`docs/dependencies/verification.json`/`docs/verification/native-docker.json` 的 304/304 是改动前口径。
- `docs/verification/software-startup.json`：`key_source_sha256` 15 条为历史快照，当前仅 3 条匹配（12 条因他组改源码而变）；`key_source_changed_after_test=[]` 不能当作“当前源码通过”的证据。
- `docs/robogauge/source_manifest.json`：`config_source_hashes.world.sdf.in=f268c468…` 是原归档输入 hash；基线参数化模板为 `9646f439…`，尾轮 109 候选应用后的当前模板为 `e5e4bba1…`。README 与独立复核已声明这是“原归档输入 hash vs 参数化模板”两套 provenance，未比较原字节；`docs/robogauge/README.md` 的“当前模板 SHA”已按最终模板刷新，归档 hash 与外部 asset/模型身份保留。
- 测试计数跨回执差异属环境自洽：unit 48（2026-11-08）→ 54（readability-verification/independent-review）；regression 62 项 skip 数 2（私有地形缺失）/5（无 CPU Torch）/7（clean-export）。根 README 的两处表述分别对应 clean-export 与 independent-review。
- `docs/robogauge/PORT_VALIDATION.json`：CPU 26 测 25 过 1 skip（可选 Torch 缺）vs 容器 26/26；`cpu_validation.json` 30 帧与 README 一致；`isaac_closure_audit.json` `executed=false` 与 README“不放假 launcher”一致。
- 文档自述的未闭合契约：navigation/mission 的 `mission_runtime` anchor 字段集与 planning/admission 的 `FrozenFence` 期望不一致；`continuous_route.enabled=false`、46 区域未验证；`simulation/terrain` 的 mission46 分支默认不启用且 `tests/terrain_targets/upper23.json` 缺失（当前只有 `configs/scenarios/terrain_targets/lower12.json`）。
- 跨文档命令界差异不是矛盾：`control/tracking [0.6,0.2,0.6]`、`navigation/robogauge [0.6,0.2,0.3]`、`robot/unitree bridge (.3,.2,.3)` 分属 cascade/RoboGauge/Student 链，各自 README 已说明。
- 根 README 自述“46 个模块 README”，实测 `git ls-files 'src/**/README.md' | wc -l` = 46，一致。

## 7. 环境/工具限制（如实记录）

- **clang-format 不可执行**：用户提供的 `/tmp/embodied-readability-tools/clang-format` 在本会话 bash 的私有 `/tmp` 中不可见；VSCode cpptools 自带的 `clang-format` 无执行位。按“不改权限/不绕过拒绝”的约束，未复制、未 chmod、未用动态加载等方式绕过。处理方式：4 个 `.cpp` 全部人工审阅，其中 `teacher_actuator.cpp`、`graph_solver.cpp`、`writer_qos.cpp` 做人工等价排版/注释，`go2hv_actuator.cpp` 判定已清晰未改；全部通过 `validate.py` 的 `cpp_tokens` 令牌级等价检查。未做编译器级验证。
- **bash `/tmp` 隔离**：每次 bash 调用是独立 tmpfs；分组记录以工作区 `work/` 为准（部分组另写了 host `/tmp` 副本，但不参与合并）。这不是仓库文件的修改。
- **大文件审阅方法**：超长 JSON（如 5821 行 `combined_floor1_inside.json`、3294 行 `configs/simulation/settings.json`）采用程序化遍历全部行/对象 + 人工阅读顶层结构/关键片段；`vendored-source-manifest.json`（2455 行）解析全部 304 条并重算哈希 + 抽样；`validation-and-threshold-audit.md`（186 行，最长行 1205 字符）按标题/表格抽取 + 结论段人工读 + 抽读表格。具体方法写入对应 coverage 条目的 `validation`。
- 未运行 ROS/Gazebo/Isaac/Docker/原生编译/完整路线/实机/训练；因此本文件的所有“未发现”仅指静态审阅与既有 CPU 测试范围，不能替代运行验证。

## 8. 串行事实修正合入记录（DSH 全量实现，Codex 独立事实复核，DSH 串行合入）

批准清单共 **109** 个候选（owner=project 的为 **60** 个）。写入前逐一复核 current==preimage 且 candidate SHA 正确，109/109 按字节应用、mode 保持；Python 归一化 AST 不变。以下旧说法的初稿已被独立复核否定，并已在源码与本报告相应条目中更正（不是当前事实）：

- 0.8 不是“最小 0.8 m 扫掠”，而是最小时间尺度乘命令世界速度；直线运动只检 ±0.18 m 偏移、无中心采样（CF-COLLISION-HORIZON/-OFFSETS）。
- 四元数校验约束平方范数 `q@q` 与 1 的偏差，而非模长偏差（P2-013）；桥接侧同样用平方范数。
- 静止初始化建立重力；“关闭在线重力估计”不等于 gravity-free 初始化（OF04，`configs/localization_mapping/fastlivo.yaml` 注释）。
- `parking_contract` 的 `POSE_CONFIG` import 是普通模块级 import，不是延迟 import；`terminal_heading_contract` 启用时按 2*pi 取模在 1e-9 rad 内比对，禁用时立即返回（非无条件严格相等）。
- `building.py` 按插入顺序写 JSON（不排序）、prepare 复制 assets 并写可变运行文件（不冻结）、退出码为 0/2/130 且异常可传播。
- `staged_cascade_binding` 的未捕获分支是成功暂存路径；返回 None 的条件是 cascade 缺失/已捕获/key 不符（PR-F01）。
- `receive` 可同步调用业务回调、压力下同步 poll/commit，不是通用非阻塞契约（PR-F03）。
- runtime 唯一默认资产是 `robogauge_policy.pt`（`torch.jit.load` TorchScript），没有 ONNX 资产选择或加载器（PR-F12）。

以上修正不删校验、不放宽阈值、不改原行为；root 仍在全部串行合入后统一复验（本文件不代表整栈/实机/原生构建已通过）。
