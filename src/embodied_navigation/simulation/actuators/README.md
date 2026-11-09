# simulation/actuators

本目录是 Gazebo 仿真中唯一的关节力矩写入插件（Teacher/Student 链），以源码形式随仓库提供，构建后由世界文件加载。

## 目录结构

```text
src/embodied_navigation/simulation/actuators/
├── CMakeLists.txt          # 8 行，构建 teacher_actuator 共享库（gz-sim8 + gz-plugin2）
├── README.md               # 本文件
└── teacher_actuator.cpp    # 442 行，唯一的 JointForceCmd 写入者（gz::sim System 插件）
```

1 个 C++ 源文件、1 个 CMake 工程；无子目录、无 Python 文件。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `teacher_actuator.cpp` | 活代码。`teacher_sim::TeacherActuator` 实现 `ISystemConfigure`/`ISystemPreUpdate`：Configure 绑定 12 个关节与 5 组接触体、校验参数与环境变量；PreUpdate 每步按 `kp*(target-q)-kd*qd` 计算并钳位成 `JointForceCmd`。文件头注释声明它不写 base wrench、不做姿态伺服、启动后不重置位姿/速度。 |
| 代码 | `CMakeLists.txt` | 构建与安装契约：`add_library(teacher_actuator SHARED ...)`、`install(... LIBRARY DESTINATION lib)`。只依赖 gz-sim8、gz-plugin2，不从其它工程链接。 |
| 日志/输出 | `$TEACHER_ACTUATOR_LOG` | 插件唯一输出。Configure 写一行 `{"kind":"actuator_contract",…}`（protocol、writer、state_source、torque_source、kp/kd/effort_limit/saturation_effort/velocity_limit、decimation、joint_order、`joint_position_reset_count:12`、`body_pose_resets:0`）；每步写一行 `{"kind":"physics_step","t":…,"dt":…,"q":…,"qd":…,"qtarget":…,"target_outside_hard_limits":…,"pd_raw":…,"tau_lower":…,"tau_upper":…,"tau":…,"contacts":…,"body_lin_vel_com":…,"wrenches_raw":…}`。日志路径由调用方注入，见下。 |
| 通信 | `$TEACHER_SOCKET` | 不是日志，是执行器与策略进程之间的 Unix socket 传输：request=64×float64、response=16×float64（原生小端），字段索引写在文件头注释里。 |
| 配置 | 生成的 `world.sdf` 插件元素 | 插件参数从 SDF 读取：`kp`（默认 25）、`kd`（0.5）、`effort_limit`（23.5）、`saturation_effort`（23.5）、`velocity_limit`（30）、`decimation`（4）、`fault_damping`。`../world/prepare.py` 只显式写入 `kp/kd/effort_limit/velocity_limit` 四项，其余走代码默认值。 |
| 配置 | `configs/launch/settings.json` | `runtime_paths.actuator = "install/actuators/lib/libteacher_actuator.so"`；`launch/prepare.py` 据此把生成世界里的 `libteacher_actuator.so` 改写成 install 下的绝对路径。本目录自身不读任何配置文件。 |

`TEACHER_ACTUATOR_LOG` 的三个注入点（`grep -rn TEACHER_ACTUATOR_LOG src/` 实测）：`launch/main.py:52` 设 `/dev/null`（主路径不落盘）；`launch/building.py:188` 与 `navigation/robogauge/benchmark.py:223` 设 `<run>/actuator_physics.jsonl`。后两者的 `.so` 由不同世界文件加载（RoboGauge 用 motor_src 的产物），但环境键与日志格式同源。

插件要求 `TEACHER_SOCKET` 与 `TEACHER_ACTUATOR_LOG` 都非空；任一缺失、socket 路径超长、日志打不开都在 Configure 抛 `std::runtime_error`，不会静默降级成"无日志/无通信"。

## 模块内文件耦合

- 依赖方向：`CMakeLists.txt → teacher_actuator.cpp`，是单向构建关系，没有运行时文件耦合。C++ 不 import 任何 Python；与策略进程只通过 socket 字节帧耦合，与 Gazebo 只通过 gz-sim 组件耦合。
- 无环、无共享可变状态：iteration、target、fault、socket fd 都是类内成员，未导出。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `teacher_actuator.cpp` | 442 | gz-sim8、gz-plugin2、`TEACHER_SOCKET`/`TEACHER_ACTUATOR_LOG` | `../world/prepare.py` 生成的世界文件加载它；策略进程是 socket 对端 | 清楚：单一写入者，职责边界写在文件头 |
| `CMakeLists.txt` | 8 | gz-sim8、gz-plugin2 | `docker/build_owned_native.py` 的 `actuators` 模块 | 清楚：只产出 `.so`，不装头文件 |
| `README.md` | 本文件 | — | 人 | — |

四类问题：**上帝文件无**（442 行单类，未超 800）；**重复实现有**：`gazebo/robogauge/motor_src/go2hv_actuator.cpp` 与本文件结构同源，`diff` 实测差异集中在"request64/response16 字段索引注释（Go2HV 侧缺失）"与"力矩包络计算块"（Teacher 用 DCMotor 饱和公式，Go2HV 用 20.2/23.4 分段限幅）以及几行注释，其余逻辑逐行一致；**跨层直达无**（插件不读 Python 配置，只读 SDF 参数）；**隐式契约有**：request/response 字段索引、`decimation` 与策略侧控制频率的配对，都靠注释和顺序维持，没有编译期检查。

## 可读性评估

- `teacher_actuator.cpp`：**可接受，偏上**。证据：文件头 17 行完整列出 request[0..56]/response[0..13] 的字段索引；关键不变量有注释（"Sole joint-force writer for the isolated Teacher simulation."、"This reset occurs exactly once, before the first physics integration."、"Exact training DCMotor speed/torque envelope."、"Reject partial/corrupt frames instead of guessing joint targets."）；Configure 的每个失败都带具体对象名（`Teacher joint missing: <name>`、`Teacher contact link missing: <name>`）。不足：单类 442 行，`PreUpdate` 与 `Exchange` 两个大函数没有分段标题；参数默认值（23.5/30/13.5/200）散落，部分有出处注释；Go2HV 副本丢了字段索引注释后，可读性依赖回读本文件。
- `CMakeLists.txt`：**良好**，8 行、无隐式 include、安装目的地明确。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 把 request64/response16 的字段偏移写成命名常量，并在 `Exchange` 里加 `static_assert(sizeof(request)==64*sizeof(double))` 一类编译期约束 | 现在跨语言字节布局只有注释约束，改错不会编译失败 | 低：只加常量/断言，不改赋值顺序 | 跑 `tests/regression/test_native_safety_integration.py`（假 socket）；对比改动前后 `actuator_physics.jsonl` 的同一行字段值 |
| P2 | 在 `actuator_contract` 日志与 README 注明 `decimation=4` 与物理步/策略 tick 的对应关系 | "每 4 步取一次目标"目前只能从 `sdf->Get("decimation",4)` 与 `% decimation` 反推 | 无（文档） | 人工复核 |
| P2 | 把 `fault_damping`、`expected_dt` 补进 README 的参数清单 | 它们参与故障路径，却不在现有参数列表里 | 无（文档） | 读代码复核 |
| P2 | 评估把 socket 传输、接触采集、日志写出抽成两份 `.cpp` 共用的私有头文件/内联实现 | 两份 400+ 行、差异只在包络计算块与注释，改一处容易漏另一处 | 中：动 C++ 传输层可能改变时序；需两模块重新构建 | `docker/build_owned_native.py --execute` 两模块构建通过；假 socket 回归与 RoboGauge native loop 测试仍通过 |

没有 P0/P1：插件是单写入者、无共享状态、失败全部显式抛错，测试虽不加载 `.so` 但用假 socket 覆盖了协议与安全门禁接线。真正缺的是跨语言字节布局的编译期约束，属 P2。本轮只写建议、不改 C++；任何传输层改动都要先有假 socket 对照再动。

## 测试与限制

- C++ 侧无专用单测；socket 协议、目标保持与安全门禁调用由 [tests/regression/test_native_safety_integration.py](../../../../tests/regression/test_native_safety_integration.py) 用假 socket 间接覆盖（不加载本 `.so`）。
- 原生源码只在本仓库编译，不从其他工程链接；`install/` 缺产物只表示本机未构建（本次核对时 `install/` 只有 `backend/`，未构建本插件）。
- 本次核对没有启动 Gazebo、没有加载插件；`.so` 的实际时序、接触日志与 200 Hz 行为未验证。
