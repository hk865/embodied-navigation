# simulation/gazebo/robogauge/motor_src

本目录是 RoboGauge 专用的 Go2HV 关节执行器原生源码，构建为 Gazebo 系统插件后由 benchmark 的世界文件加载。

## 目录结构

```text
src/embodied_navigation/simulation/gazebo/robogauge/motor_src/
├── CMakeLists.txt          # 8 行，构建 robogauge_actuator 共享库（gz-sim8 + gz-plugin2）
├── README.md               # 本文件
└── go2hv_actuator.cpp      # 422 行，Go2HV 力矩包络插件（沿用 TeacherActuator 的 ABI 名）
```

1 个 C++ 源文件、1 个 CMake 工程；无子目录、无 Python 文件。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `go2hv_actuator.cpp` | 活代码。与 `simulation/actuators/teacher_actuator.cpp` 结构相同（Configure/PreUpdate、request64/response16 over Unix socket），只换成 Go2HV 力矩包络：加速 20.2 N·m、制动 23.4 N·m（按 `qd*raw` 符号取），13.5 rad/s 以上线性降到 30 rad/s 归零。仍注册 `teacher_sim::TeacherActuator` 插件名以复用世界模板与 wire 契约。 |
| 代码 | `CMakeLists.txt` | `add_library(robogauge_actuator SHARED go2hv_actuator.cpp)`，只依赖 gz-sim8、gz-plugin2，安装到 `lib/`。 |
| 日志/输出 | `$TEACHER_ACTUATOR_LOG` | 与 Teacher 插件同格式：Configure 一行 `actuator_contract`（含 `effort_limit`/`saturation_effort`/`velocity_limit`/`decimation`），每步一行 `physics_step`，另有 `model_plugin_ownership` 行。RoboGauge 路径由 `navigation/robogauge/benchmark.py:223` 注入 `<run>/actuator_physics.jsonl`。 |
| 通信 | `$TEACHER_SOCKET` | 与策略进程的 Unix socket 传输，字节布局与 Teacher 插件一致；两个 `.so` 注册的是同一个系统名（`teacher_sim::TeacherActuator`），所以两个世界模板各自只引用其中一个；不要把两者装配进同一个世界。 |
| 配置 | 生成的 `world.sdf` 插件元素 | 插件参数从 SDF 读取，默认 `kp=25`、`kd=0.5`、`effort_limit=23.5`、`velocity_limit=30`、`decimation=4`。RoboGauge 的世界模板是 `configs/robogauge/world.sdf.in`（`${ROBOGAUGE_ACTUATOR}` 占位由 `../world.py` 展开）。本目录自身不读配置文件。 |

已核实的参数异常（未改代码，仅记录）：`saturation_effort` 在本文件里被 `Configure` 解析（默认 23.5）、纳入有限性校验、写进 `actuator_contract` 日志，但力矩钳位只使用 `baseLimit`（20.2/23.4）与速度线性衰减，`grep saturationEffort` 除解析/校验/日志外无第三类使用点。也就是说它在本插件里是"只记录不生效"的参数，与 Teacher 插件（`saturation_effort` 参与 DCMotor 包络公式）语义不同。

## 模块内文件耦合

- 依赖方向：`CMakeLists.txt → go2hv_actuator.cpp`（单向构建关系）。运行时不与任何 Python 文件耦合，只通过 socket 与世界文件参数接入。
- 与 `simulation/actuators/teacher_actuator.cpp` 是**源码级复制关系**而非 import 关系：`diff` 实测差异只有力矩包络计算块、wire 字段索引注释（本文件缺失）和几行说明注释，其余逐行一致，两份文件不共享头文件。
- 无环、无共享可变状态。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `go2hv_actuator.cpp` | 422 | gz-sim8、gz-plugin2、`TEACHER_SOCKET`/`TEACHER_ACTUATOR_LOG` | `../world.py` 的 `actuator_path()` 指向它的 `.so`，由生成的 `world.sdf` 加载 | 偏重：422 行单类，且与 Teacher 插件重复 |
| `CMakeLists.txt` | 8 | gz-sim8、gz-plugin2 | `docker/build_owned_native.py` 的 `robogauge` 模块 | 清楚 |
| `README.md` | 本文件 | — | 人 | — |

四类问题：上帝文件无（422 < 800）；**重复实现有**（与 Teacher 插件约 400 行同构）；跨层直达无；隐式契约有：插件名与 `TEACHER_SOCKET`/`TEACHER_ACTUATOR_LOG` 是历史 wire 兼容名，实际部署的是 Go2HV，靠注释而不是类型区分。

## 可读性评估

- `go2hv_actuator.cpp`：**可接受**。包络公式带出处注释（"Independent adapter: official go2_rl_lab Go2HV deployment envelope. Fs=Fd=0 in official default; unchanged Gazebo joint physics is disclosed."），并明确披露"引擎 30 rad/s 上限与原始关节物理未改"。不足：**没有 request64/response16 字段索引注释**（Teacher 文件有 17 行，本文件缺），读者必须回读 `actuators/teacher_actuator.cpp` 才懂 request 下标含义；`saturation_effort` 只解析不生效，容易误以为它限制力矩；数字 20.2/23.4/13.5/30 有注释但无单位表。
- `CMakeLists.txt`：**良好**，8 行，职责单一。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把 request64/response16 字段索引注释补回本文件（或抽到两文件共享的注释头） | Go2HV 是 RoboGauge 实际部署的插件，却要回 Teacher 文件查协议 | 无（纯注释） | 人工 diff |
| P2 | 在 `actuator_contract` 日志与 README 标注 `saturation_effort` 在本插件"只记录、不参与钳位" | 否则调它没有任何效果，且与 Teacher 语义相反 | 无（文档）；若要让它生效则属行为变更 | 若只改文档：读代码复核；若改行为：必须先有包络单测与 `actuator_physics.jsonl` 对照 |
| P2 | 评估把公共逻辑（socket 传输、接触采集、日志写出）抽成与 Teacher 插件共享的私有头文件 | 两份 400+ 行同构文件，改一处容易漏另一处 | 中：动 C++ 传输层可能改变时序，且需重新构建两个模块 | `docker/build_owned_native.py --execute` 构建两模块；`tests/regression/test_robogauge_native_loop.py` 与 `test_native_safety_integration.py` 仍通过 |

没有 P0/P1：本插件是单写入者、失败显式抛错，包络公式有出处注释；风险集中在"两份副本会漂移"和"saturation_effort 语义不一致"，属 P2。本轮只写建议、不改 C++，任何包络改动都要先有数值对照。

## 测试与限制

- 无 C++ 单测；[tests/regression/test_robogauge_benchmark.py](../../../../../../tests/regression/test_robogauge_benchmark.py) 只断言准备出的世界引用了正确 `.so` 路径，不加载或运行它。
- 构建入口是仓库根的 `docker/build_owned_native.py` 的 `robogauge` 模块，安装到 `install/robogauge/lib/`；使用方是 [../world.py](../world.py) 的 `actuator_path()`，背景见 [docs/robogauge/README.md](../../../../../../docs/robogauge/README.md)。
- `install/` 缺该产物只表示本机未构建，源码完整；本次核对未构建、未启动 Gazebo，Go2HV 包络的真实力矩行为未验证。
