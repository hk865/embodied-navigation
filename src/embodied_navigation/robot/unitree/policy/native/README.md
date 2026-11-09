# robot/unitree/policy/native：247D 中间观测与 native64 循环

本目录保留历史 Teacher 的 native 中间观测（247D），并提供当前 Student 复用的 socket 驱动 native64 关节控制循环。普通导航不加载 Teacher actor，247D 仅用于诊断，实际 Student 推理使用 exact45；本目录不直接面向实机。

## 目录结构

```text
src/embodied_navigation/robot/unitree/policy/native/
├── README.md       # 本文件：247D 诊断链与 native64 循环
├── __init__.py     # 0 行，空包声明（无 docstring）
├── contract.json   # 414 行，冻结的 247D 拼接/scale/clip、时序、actuator 数值契约
├── observation.py  # 322 行，247D 观测构造 + SDF 静态碰撞地形 raycast
└── worker.py       # 379 行，native64 socket 关节控制循环（依赖注入）
```

**更正一处常见误解**：本目录**没有** `CMakeLists.txt`，也没有 C++ 源文件；磁盘实测只有上表 5 个条目，`find src/embodied_navigation/robot -iname 'CMake*' -o -iname '*.cpp'` 为空。带 CMake 的原生模块在 RoboGauge 一侧（`simulation/gazebo/robogauge/motor_src`），构建命令见 [docs/robogauge/README.md](../../../../../../docs/robogauge/README.md) 的「Native Gazebo actuator build」。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `observation.py`（322 行） | 活代码：`build_observation()` 生成 float32[247] + 诊断 dict；`TerrainHeightMap.from_sdf()` 解析 SDF 静态碰撞做 187 条竖直 raycast；`quaternion_rotation()` 按 wxyz 算 world-from-body 旋转 |
| 代码 | `worker.py`（379 行） | 活代码：`main()` 是 socket 循环，actor/观测/命令/收发/发布全部以关键字参数注入，自身不加载模型也不读配置 |
| 数据 | `contract.json`（414 行） | 数值事实来源（不是配置）：`observations.concatenation_order` 的 247D 分段 scale/clip/noise、`height_scan`（187 维、1.6×1.0 m、11×17 网格、+20 m ray 起点）、`request64` 各字段含义、`actions.scale=0.25`、`actuator`（DCMotor）、`timing`（50 Hz / 0.005 s） |
| 代码 | `__init__.py`（0 行） | 空包声明 |
| 日志/输出 | `<run>/policy_manifest.json` | 观测/地形/安全来源的 manifest；`worker.py` 通过注入的 `publish` 写入（由上层 `policy/worker.py` 增补 Student metadata） |
| 日志/输出 | `<run>/worker_ready` | 监听成功后写入本进程 PID 的就绪标记 |
| 日志/输出 | `<run>/state.json` | 每 10 帧原子落一次状态行（`atomic_json` 临时文件 + `replace`） |
| 日志/输出 | `<run>/worker_result.json` | 退出收据：`completed`/`fault`/`samples`/`cpu_forward_ms`（p50/p95/max）/`last_sim_time` |
| 日志/输出 | `<run>/telemetry.jsonl`、`height_scan_sources.jsonl`、`observations_actions.npz` | **仅当环境变量 `NAVIGATION_RECORD_RAW=1`** 时写（`height_scan_sources.jsonl` 还要求显式 terrain manifest）；默认不产出 |
| 日志/输出 | 无 logging | 本目录 3 个 py 文件 `getLogger`/`print` 均为 0（事实包 grep）；socket 路径在退出时 `unlink` |
| 配置 | 无 `configs/` 读取点 | 本目录不 import `common.config`、不读 `configs/`；唯一数值来源是同目录 `contract.json`，在 `observation.py:17` 于 **import 时**读入（`CONTRACT = json.loads(...)`） |
| 配置 | 环境变量 `NAVIGATION_RECORD_RAW` | 只影响原始录制开关，不影响观测/控制逻辑 |

注意：`contract.json` 除本目录 `observation.py` 外，还被 [simulation/dynanav/capture/scene.py](../../../../simulation/dynanav/capture/scene.py) 以硬编码路径引用（`scene.py:366`）；改文件名会破坏仿真侧快照。

## 模块内文件耦合

- 依赖方向：`observation.py` → `simulation/mesh_terrain.py`（同仓库数值工具）；`worker.py` → `observation.py` + `common/state_contract.py` + `control/safety/simulation_gate.py` + `simulation/safety_state.py`。`contract.json` 只被 `observation.py` 读取。目录内是“数据 + 观测 + 循环”的单向依赖，**无环**。
- 反向依赖：`observation.py` 被 `robogauge/contract.py`、`simulation/dynanav/capture/{control,physics}.py`、`simulation/gazebo/robogauge/native_loop.py` 复用；`worker.py` 被 `policy/worker.py`（复用循环）与 `simulation/truth_worker.py` 复用。即本目录是**被复用的底层库**，不 import 上层。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `observation.py` | 322 | `simulation.mesh_terrain`；`contract.json` | robogauge.contract、dynanav capture×2、gazebo native_loop | 清楚：纯数值/几何，无 IO 副作用 |
| `worker.py` | 379 | `observation`、`common.state_contract`、`control.safety.simulation_gate`、`simulation.safety_state` | `policy.worker`、`simulation.truth_worker` | 偏重：`main()` 314 行，但依赖是显式注入，边界可读 |
| `contract.json` | 414 | 无（数据） | `observation.py`（+ dynanav scene 硬编码路径） | 清楚：单一数值来源 |
| `__init__.py` | 0 | 无 | 无 | 空包，可接受 |

四类问题：

- **上帝文件**：本目录没有（文件最大 379 行）；但 `worker.main()` 单函数 314 行。
- **重复实现**：本目录内没有。跨目录有一处**有意的数值重复**：`contract.json` 的 `joint_names`/`default_joint_positions` 与 [robogauge/contract.py](../../robogauge/contract.py) 的 `NATIVE_NAMES`/`QDEFAULT` 是同一组 Go2 常量，靠 `tests/regression/test_robogauge_contract.py` 断言两者相等来防漂移（该测试同时校验 `QDEFAULT == CONTRACT["default_joint_positions"]`）。
- **跨层直达**：有——`worker.py` 直接 import `control/safety` 与 `simulation/safety_state`（逐帧安全检查），`observation.py` 直接 import `simulation/mesh_terrain`。这是“底层库依赖同仓工具”而非反向调用，方向一致，风险可控。
- **隐式契约**：有——`contract.json` 的键名/切片（`observations.concatenation_order`、`request64`）是隐式 schema；socket 二进制帧长固定（收 64×float64=512 字节、回 16×float64=128 字节），由对端 Gazebo actuator 决定；`state[55]/[56]` 的 iteration/fault 语义只在 contract 里说明。

结论：**耦合清楚**。本目录是“数据契约 + 纯观测 + 注入式循环”的三层，依赖方向单一，主要成本在 `main()` 的长度与 `contract.json` 的隐式 schema。

## 可读性评估

| 对象 | 分级 | 证据 |
| --- | --- | --- |
| `observation.py` | **可接受** | 322 行 / 11 函数；docstring 0.182（2/11）；最长 `build_observation()` 86 行；>80 行 1 个；注释 4 行。优点：数值全部由 `contract.json` 驱动、`build_observation` docstring 写明 base_com_offset 语义；缺点：`_Box`/`_Plane`/`TerrainHeightMap.raycast` 无 docstring |
| `worker.py` | **可接受** | 379 行 / 5 函数；docstring 1.0（5/5）；`main()` 314 行；注释 9 行。依赖注入签名本身是文档 |
| `contract.json` | **可接受（数据）** | 414 行 JSON，分段自解释（每段有 `name/slice/scale_after_clip/clip`），`height_scan` 段带 `formula`/`missing_hits` 说明；没有 JSON Schema 校验 |
| `__init__.py` | **可接受** | 0 行空文件；同包 `policy/__init__.py` 也为空，属历史惯例而非缺陷 |

区分结构与文档：本目录没有结构缺陷（无环、纯函数、依赖注入）；缺口都在文档——`observation.py` 的几何类没有 docstring，247D 各段的“为什么这样缩放”只写在 `contract.json` 里。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `observation.py` 的 `_Box`/`_Plane`/`TerrainHeightMap.from_sdf`/`raycast` 补 docstring（ray 起点 +20 m、缺失命中为 -inf 的语义） | 11 个函数只有 2 个有 docstring；地形语义最容易被误读 | 无（只改注释） | 静态检查；`test_robogauge_contract` 仍通过 |
| P2 | 把 `worker.main()` 的 314 行按“解析参数 / 地形准备 / manifest 发布 / 循环 / 收据”分段提小函数 | 单函数过长，退出路径（fault / duration / termination）不易核对 | 中：注入回调与 socket 生命周期必须保持顺序 | 先补一条假 socket 的循环测试（现有 `test_native_safety_integration` 已覆盖门禁调用，可在此基础上加） |
| P2 | 在 `contract.json` 的 `joint_names` 旁加一条注释字段说明“与 robogauge/contract.py 的 OFFICIAL_NAMES 必须相等” | 两处常量靠测试防漂移，测试之外没有文档提示 | 无 | 已有 `test_robogauge_contract` 断言 |
| P2 | `scene.py:366` 对 `contract.json` 的硬编码路径：改为从 `observation` 模块导出的路径常量读取（需先给 `observation.py` 加 `CONTRACT_PATH`） | 改文件名/移动目录会静默破坏仿真快照 | 低 | `grep -rn "native/contract.json" src` 检查；跑 dynanav 相关 regression |

没有 P0/P1：本目录无配置读取、无日志写入；异常处理除 socket 清理时的 `except FileNotFoundError: pass` 外没有静默分支——`StateContractError` 会转成安全 fault，主循环其它异常写入 `worker_result.json` 的 `fault` 后退出。`main()` 长度是可读性问题，不是正确性问题。本轮只写建议、不改代码；任何建议都应先有测试/对照再动。

## 输入输出

`state[0]` 为世界时间，`[1:4]` base link 世界位置，`[4:8]` wxyz 四元数，`[8:11]` body-frame COM 速度（若对端给 link-origin 速度，必须显式传 `base_com_offset`，不要用训练偏移），`[11:14]` 角速度，`[14:26]` / `[26:38]` 关节位置/速度，`[38:50]` 上一帧 DC 裁剪后力矩，`[50:55]` 接触（-1 未知），`[55]` iteration，`[56]` fault code，`[57:64]` 保留。回包 16 个 float64：前 12 为关节目标，`[12]` 为 fault，`[13]` 为 done，`[14:16]` 保留为 0；对端是 Gazebo actuator 的 native64 socket。

## 与 45D 的关系

247D 是历史 Teacher 的中间观测适配链：[../worker.py](../worker.py) 复用它；Student 实际推理在 [../student.py](../student.py) 用 exact45，247D 仅供诊断。[robogauge/README.md](../../robogauge/README.md) 的 45D 是完全独立的直接观测接口，两者不是同一个策略输入或 actuator 契约。

## 测试与限制

[tests/regression/test_native_safety_integration.py](../../../../../../tests/regression/test_native_safety_integration.py) 用真实 247D 观测构造与假 socket 执行本循环并检查共享门禁；[tests/regression/test_robogauge_contract.py](../../../../../../tests/regression/test_robogauge_contract.py) 校验 `quaternion_rotation` 与 `contract.json` 字段。地形来自 SDF 静态碰撞而非在线估计，仅用于仿真；本目录不直接面向实机。测试整体边界与 skip 规则见 [tests/README.md](../../../../../../tests/README.md)。
