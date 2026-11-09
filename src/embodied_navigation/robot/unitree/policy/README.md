# robot/unitree/policy：策略适配与底层控制环

本目录承接当前 Student 策略运行：[worker.py](worker.py) 是 actor 子进程入口，复用 native 循环并注入 Student 推理、指令预适应与 ACK；[native/README.md](native/README.md) 提供 247D 中间观测与 native64 循环。

## 目录结构

```text
src/embodied_navigation/robot/unitree/policy/
├── README.md                        # 本文件：策略适配层职责与三套契约边界
├── __init__.py                      # 0 行，空包声明（无 docstring）
├── lower_loop_ack.py                # 561 行，Student 底层控制环 ACK 契约（纯函数，无模型/ROS/时钟）
├── native/                          # 子模块，见 native/README.md
├── speed_tiers.py                   # 152 行，0.2 / 0.5 m/s 两档速度选择
├── student.py                       # 837 行，exact45 计算 + COM 速度 PID + Student 运行时
├── student_xy_stop_takeover_ack.py  # 154 行，XY 停车接管的同帧 ACK 证明
├── worker.py                        # 479 行，actor 子进程入口（组合根）
└── yaw_measurement_feedback.py      # 367 行，条件式 yaw P 测量保持扩展
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `worker.py`（479 行） | 活代码/组合根：`--run` + `--socket` 的 actor 进程，读 `runtime_context.json` 的 `actor_kind`，构造 `StudentRuntime` 并复用 `native/worker.py` 循环 |
| 代码 | `student.py`（837 行） | 活代码：`exact_student45()` 纯函数、`CorrectionOnlyPID`、`StudentRuntime`（校验导出契约、加载 TorchScript、组装 telemetry/metadata） |
| 代码 | `lower_loop_ack.py`（561 行） | 活代码/纯契约：模块 docstring 明确 “without model/ROS/clock access”；被 control/navigation 复用 |
| 代码 | `speed_tiers.py`（152 行）、`student_xy_stop_takeover_ack.py`（154 行）、`yaw_measurement_feedback.py`（367 行） | 显式模式扩展；`speed_tiers.py` 是纯叶子（无内部 import） |
| 代码 | `__init__.py`（0 行） | 空包声明；没有 docstring |
| 日志/输出 | `<run>/closed_loop_executor_ack.json` | `worker.py` 的 `atomic_ack_rows()` 原子写，schema `teacher_known_velocity_ack/v1`；只在收到下一个 native64 请求证明上一帧完成后追加 |
| 日志/输出 | `<run>/policy_manifest.json`、`worker_ready`、`state.json`、`worker_result.json` | 由复用的 native 循环写：manifest 含 Student metadata（`worker.py` 的 `metadata()` 增补）、`worker_ready` 是 PID、`state.json` 每 10 帧落一次、`worker_result.json` 含 fault/samples/CPU 前向 p50/p95/max |
| 日志/输出 | `<run>/terrain_provider_initialization_failure.json` | 地形 provider 构造失败时的错误收据（`open("x")`，不覆盖已有文件） |
| 日志/输出 | `<run>/telemetry.jsonl`、`height_scan_sources.jsonl`、`observations_actions.npz` | **仅当环境变量 `NAVIGATION_RECORD_RAW=1`** 时由 native 循环写；默认不产出。`worker.py` 的 telemetry/ACK 流是 `NullStream`（记录上下文为 `NoRecordingContext`），不落盘 |
| 日志/输出 | 无 logging | 本目录 7 个 py 文件 `getLogger`/`print` 均为 0（事实包 grep 实测）；诊断输出全部走 run 目录 JSON/JSONL 收据 |
| 配置 | `read()["student_runtime"]` | `StudentRuntime.__init__` 里 **直接重读 `configs/`**（`student.py:332-334`，`from embodied_navigation.common.config import read`），不是从 `runtime_context.json` 取。这是 [configs/README.md](../../../../../configs/README.md) 明确列出的“运行期仍直接重读 configs”的消费者之一 |
| 配置 | `<run>/runtime_context.json` | `worker.main()` 读 `actor_kind`、`run_dir` 与 `profile`；`StudentRuntime` 另用它校验本次 run 绑定（读 `scope_bytes` 与 `profile`） |
| 配置 | `profile` 键 | `cascade.command_limits`（命令界）、`cascade.r85_student_lower_loop_ack`（ACK 契约）、`r109_speed_tier`（速度档）、`r85_pre_zero_integral_unload`、`r85_student_yaw_measurement_p_retention`、`r85_student_yaw_pd_fade`、`cascade.r85_student_lower_loop_ack.xy_stop_takeover_enabled` 等扩展开关，均从本次 profile 读 |

`student_runtime` 的实测契约（`configs/control/settings.json` → `read()["student_runtime"]`）：`schema=student_runtime_exact45_fixed_pid/v2`、`observation_dimension=45`、`noise=false`、`action_scale=0.25`、`action_clip=null`、`policy_step_s=0.02`、`physics_step_s=0.005`、`native_response_deadline_ms=200`、`source_freshness_ms=300`、`recurrent_shape=[1,1,256]`；任一不符 `StudentRuntime` 直接 `ValueError`。

## 模块内文件耦合

- 依赖方向：`worker.py`（组合根）→ `student.py`、`native/`、`lower_loop_ack.py`、`control/`、`simulation/terrain/`；`student.py` → `speed_tiers.py`、`common/config`、`common/input_freshness`；`lower_loop_ack.py` → `speed_tiers.py`、`student_xy_stop_takeover_ack.py`、`yaw_measurement_feedback.py`、`control/parking/integral_unload`；`speed_tiers.py` 与 `student_xy_stop_takeover_ack.py` 是叶子。**目录内无 import 环**（`worker → native/worker` 是单向复用）。
- 反向依赖（被谁用）：`lower_loop_ack.py` 被 `control/parking/transition_gate.py`、`control/tracking/cascade_core.py`、`navigation/base_node.py`、`navigation/pid_control.py`、`worker.py` 五处 import；`student.py` 被 `simulation/dynanav/capture/policy.py`、`simulation/truth_worker.py` 复用。即 robot 与 control/navigation 之间存在**包级双向依赖**，但文件级未成环。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `worker.py` | 479 | common×2、control×2、本目录×5（policy 包、lower_loop_ack、native 包、native.worker、student）、simulation/terrain×2 | 无（进程入口，由 launch 启动） | 偏重：11 个内部 import 的组合根，`main()` 416 行 |
| `student.py` | 837 | common×2、speed_tiers | worker、dynanav/capture/policy、simulation/truth_worker | 偏重：三件事（45D 计算 / COM 速度 PID / 运行时装配）在一个文件 |
| `lower_loop_ack.py` | 561 | speed_tiers、xy_stop_ack、yaw_feedback、control.parking.integral_unload | 5 处（见上） | 清楚但被跨层复用，改动影响面大 |
| `speed_tiers.py` | 152 | 无 | lower_loop_ack、student、yaw_feedback、dynanav/capture/policy | 清楚（纯叶子） |
| `student_xy_stop_takeover_ack.py` | 154 | 无 | lower_loop_ack | 清楚 |
| `yaw_measurement_feedback.py` | 367 | control.parking.integral_unload、speed_tiers | lower_loop_ack | 清楚 |
| `__init__.py` | 0 | 无 | worker、dynanav/capture/policy | 空包，可接受 |

四类问题：

- **上帝文件**：有——`student.py` 837 行（>800）；`worker.py` 文件 479 行但 `main()` 单函数 416 行。
- **重复实现**：有（小范围）——`exact_student45()` 内联构造 world-from-body 旋转矩阵（`student.py:43-49`，`exact_student45` 内），与 [native/observation.py](native/observation.py) 的 `quaternion_rotation()`（22-35 行）是同一公式的两份实现。Student 侧保持自包含可能有数值/审查理由，但代码没有注释说明这一点。
- **跨层直达**：有——`StudentRuntime` 直接 `read()["student_runtime"]` 重读 configs（绕过 prepare 冻结的 runtime context，见 [configs/README.md](../../../../../configs/README.md)）；`worker.py` 直接 import `simulation/terrain/`；`lower_loop_ack.py` 直接 import `control.parking.integral_unload`；反向地，[control/tracking/policy_selection.py](../../../control/tracking/policy_selection.py) 把本目录路径写死为 `ROOT / "src/embodied_navigation/robot/unitree/policy"`，用于快照/校验。
- **隐式契约**：有——字符串键（`actor_kind`、`student_runtime`、`r109_speed_tier`、`r85_student_lower_loop_ack`、`cascade.command_limits`）、固定文件名（`navigation_command.json`、`closed_loop_executor_ack.json`）、ACK 时钟差常量 `20_000_000` ns（`lower_loop_ack.py:439/475`）、以及**按路径动态加载的可选扩展**：`StudentRuntime` 用 `Path(__file__).with_name(...)` 加载 `pre_zero_integral_unload.py`、`student_xy_stop_takeover.py`、`student_yaw_measurement_feedback.py`、`student_yaw_pd_fade.py`（`__init__` 里的 unload 加载 + 3 个 `configure_*_candidate` 方法）。**这 4 个文件当前都不在仓库里**（全仓 `find` 无同名 `.py`；`history/` 下只有 `pre_zero_integral_unload` 的历史目录，不是可执行副本），只有 profile 打开对应开关（`r85_pre_zero_integral_unload`、`cascade.r85_student_lower_loop_ack.xy_stop_takeover_enabled`、`r85_student_yaw_measurement_p_retention`、`r85_student_yaw_pd_fade`）时才会走到；这几个开关在 `configs/control/settings.json` 中默认 `enabled:false`。

结论：**偏重但契约边界写得细**。`worker.py` 作为组合根 import 多是合理的，真正需要关注的是 `student.py` 的 837 行三合一与那条“配置开启才会踩到”的动态 import 路径。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `worker.py` | **需改进** | 479 行 / 19 个函数中只有 1 个有 docstring（0.053）；`main()` 416 行；11 行注释；局部闭包共享可变 `latest` dict，状态靠约定而不是类型 |
| `student.py` | **可接受（文档偏弱）** | 837 行、17 函数、docstring 0.235（4/17）；最长 `CorrectionOnlyPID.update()` 181 行；>80 行函数 2 个；9 行注释。结构本身清楚（纯函数 → PID 类 → Runtime 类） |
| `lower_loop_ack.py` | **可接受** | 561 行 / 15 函数、docstring 0.667（10/15）；最长 142 行、>80 行 3 个；注释 0 行；函数名（`canonical/digest/validate_copies/bind_causal_ack/residual`）自解释 |
| `speed_tiers.py` | **良好** | 152 行、docstring 1.0、最长 34 行、无 >80 行函数；`choice/validate/configuration` 明确写“不授予模型或安全权限” |
| `student_xy_stop_takeover_ack.py` | **良好** | 154 行、docstring 0.571、最长 67 行、无 >80 行函数 |
| `yaw_measurement_feedback.py` | **可接受** | 367 行、docstring 0.4、最长 135 行、>80 行 3 个；3 行注释 |
| `__init__.py` | **可接受** | 0 行，空包；`policy/__init__.py` 无 docstring（同级其他包有） |

指标可信度抽查：`native/worker.py` 的 5 个函数全部带 docstring，与事实包 `有docstring函数比=1.0` 一致，说明该列可直接用。区分两类问题：`speed_tiers`/`xy_ack` 只是小模块，不缺结构；`student.py` 缺的是 docstring 与职责拆分说明，`worker.py` 缺的是结构（416 行 `main`）。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把 `worker.main()` 的嵌套闭包（`receive/read_command/request/metadata/finalize_actor_frame`）提成模块级函数，用显式小状态对象代替 `latest` dict | 416 行单函数 + 可变 dict 闭包是改错成本最高的地方 | 中高：ACK/计时/收据顺序不能变；**当前无该文件的直接测试** | 先补一个假 socket + 假 native 循环的 CPU 测试，再逐步搬；改动后跑 `tests/regression` |
| P1 | 把 `CorrectionOnlyPID`（含 181 行 `update()`）拆到独立模块，`student.py` 只留 exact45 + Runtime | 837 行三合一，PID 与 45D 数值契约改动原因不同 | 中：数值行为必须逐位不变；`student.py` 被 capture/truth_worker 复用 | 拆前用现有回归 + 新增“同输入同输出”等价测试；确认 `grep -rn "student import"` 的 3 个消费者仍可用 |
| P2 | 4 处可选扩展的动态加载加显式前置检查：`helper_path.is_file()` 不成立时抛带路径的可读错误（现在会得到 `FileNotFoundError`）；并补文档说明这些 helper 不在仓库、默认关闭 | 4 个扩展文件（`pre_zero_integral_unload.py`、`student_xy_stop_takeover.py`、`student_yaw_measurement_feedback.py`、`student_yaw_pd_fade.py`）都不在仓库内，启用配置时的失败信息不易定位；默认关闭所以不是 P0/P1 | 低 | 临时启用其中一个配置跑启动，确认错误信息含 helper 路径 |
| P2 | `exact_student45()` 内联旋转矩阵与 `native/observation.py::quaternion_rotation()` 的重复：抽公共数值 helper，或在两处加注释说明为何保留两份 | 同一公式两份实现，改一处漏一处的风险 | 低（抽 helper 时注意 dtype/归一化） | `test_native_safety_integration` + `test_robogauge_contract`（都对比 `quaternion_rotation`） |
| P2 | 把 `20_000_000`（20 ms ACK 时钟差）提为命名常量并补 docstring；给 `worker.py` 的闭包补 docstring | 魔法数与 0.053 的 docstring 覆盖率是纯文档成本 | 无行为风险 | 静态检查 + 跑 regression |

没有 P0：本目录不直接驱动关节、不写安全决策，绝大多数失败路径显式抛错或写收据（fail closed）；唯一的静默分支是 `worker.request()` 里读取 `navigation_status.json`/`mission46_status.json` 的解析异常（`except (OSError, ValueError, TypeError): pass`），它只影响 completion 计时/元数据，不影响命令输出。动态 import 的缺口只在默认关闭的配置下可达。本轮只写建议、不改代码；P1 必须先有测试/对照再动。

## 关键文件与接口

- `worker.py`：`python -m embodied_navigation.robot.unitree.policy.worker --run <run> --socket <sock>`，由 [simulation/launch/components.py](../../../simulation/launch/components.py) 以 `actor` 子进程启动。读取 `runtime_context.json` 的 `actor_kind`，复用 `native/worker.py` 循环并构造 `StudentRuntime`；当前普通导航要求 `student`，历史 `teacher` 选项会被显式拒绝。
- `student.py`：`exact_student45(state, command, previous_action)` 从 native64 状态计算 45D Student 输入（`state[11:14]*0.2`、投影重力、command、`q-QDEFAULT`、`qd*0.05`、上一原始动作）；`CorrectionOnlyPID` 是 Student 的 COM 速度 PID；`StudentRuntime` 校验 45D、GRU、0.25 动作 scale、50 Hz 等契约并加载导出。`observation_frame()` 同时收到 247D Teacher 观测，但只作诊断（`teacher247_used_for_student_inference=False`），实际推理用 exact45。
- `lower_loop_ack.py` 是 Student 底层控制环 ACK 契约；`student_xy_stop_takeover_ack.py`、`yaw_measurement_feedback.py` 是可选控制扩展，均为显式模式。
- `speed_tiers.py`：只允许 0.2 / 0.5 m/s 两档，改的是 Student 纵向命令上限（0.3 / 0.6 m/s）与 0.65 m planning horizon，不改 PID 增益或 slew。

## 输入输出与依赖

输入是 run 目录、socket 与 profile；输出是 12 维关节目标、状态/ACK 文件和可选 telemetry。依赖 [native/README.md](native/README.md) 观测/循环、[control/tracking/policy_selection.py](../../../control/tracking/policy_selection.py)、[simulation/safety_state.py](../../../simulation/safety_state.py)。

## 测试与限制

本目录无专用单测；[native/worker.py](native/worker.py) 由 [tests/regression/test_native_safety_integration.py](../../../../../tests/regression/test_native_safety_integration.py) 用真实观测与假 socket 覆盖（247D），`StudentRuntime` 与实际权重导出**未被 CPU 测试直接执行**。运行需要外部模型/导出与可构建环境；无实机验证。当前记录见 [tests/README.md](../../../../../tests/README.md)：regression 86 项中 5 项因未安装可选 CPU Torch 被 skip，skip 不等于通过。
