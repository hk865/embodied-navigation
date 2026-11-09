# 现役 controller 与 DynaNav 采集入口的结构重构（2026-11-09）

本轮针对两个"文件本身混职责"的问题做实际代码重构，而不是只改文档或只新增编排脚本：

- `src/embodied_navigation/navigation/pid_navigation.py`（4718 行）
- `src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py`（1785 行）

约束：保留当前全部未提交成果；不改数值/阈值/行为；保留旧导入与 `-m` 入口兼容；真实仿真、ROS 节点、模型/GPU 推理、训练与机器人运行仍暂停；只用离线 CPU/假依赖验证；不提交/推送 Git、不改权限、不改 DSH/API/网络配置。重构前原文备份在任务目录
`.dsh-subagents/2026-11-09T07-25-26-735Z-81d359de/notes/pre-refactor/`（仅本地核对用，不进入仓库）。

---

## 1. `navigation/pid_navigation.py`：现役 controller 的职责拆分

### 1.1 原结构与问题

拆分前该文件 4718 行、`PIDNavigation` 单类约 3400 行（49 个方法），并混有：

- 模块级导入与 27 个证据/回执/探针 helper（约 1000 行，含 `_TeacherSubstageTiming`、`ResetBookkeeping`、`_r47_*`/`_r56_*`/`_r67_*` 包装）；
- 一个类里的订阅回调、准入生命周期、851 行的 `_admit_matched_spline` 决策体、路线执行、cascade/PID 速度选择、停车/终端保护与命令发布；
- 文件尾部 240 行的 rclpy 生命周期/`_main`/executor drain 入口；
- 重复 11 次的 `from embodied_navigation.common.observation import api as _observer_api` 导入；
- 唯一相关单测 `tests/unit/test_ros_shutdown_lifecycle.py` 按**文件路径**解析 `_drain_executor`/`_main`，把入口与类耦合在同一个文件路径上。

### 1.2 拆分结果（新布局）

| 模块 | 行数 | 方法数 | 职责 |
| --- | --- | --- | --- |
| `pid_navigation.py`（入口兼容层） | 202 | 1（`__init__`） | 模块说明、类组合、`main()` 兼容入口；旧导入路径不变 |
| `pid_ingress.py` | 454 | 8 | 请求/走廊/IMU/odom/cloud 订阅回调与点云归档 |
| `pid_admission.py` | 616 | 16 | 准入身份/资格、已准入路径保留、替换/重规划/保持、SCAN 定义与 spline 接收 |
| `pid_admission_apply.py` | 891 | 1 | `_admit_matched_spline`（"候选 spline vs 冻结路线"的大决策体，原样迁出） |
| `pid_route.py` | 301 | 12 | checked trajectory 执行、连续进度、规划目标/请求/参考点、segment 激活、bundle 参考 |
| `pid_control.py` | 1173 | 11 | cascade 装配、ACK 缓存、PID 速度选择、motion guard、终端保护/停车、命令发布 |
| `pid_support.py` | 1030 | — | 原模块级导入、回执/证据 helper、探针包装、`ResetBookkeeping`、`_TeacherSubstageTiming` |
| `pid_entrypoint.py` | 303 | — | `_drain_executor`、`_main`、`main`：rclpy init、`IntegratedNavigation` 组合、executor/spinner 生命周期与最终 drain |

类组合顺序固定为：

```python
class PIDNavigation(
    IngressMixin, AdmissionMixin, AdmissionApplyMixin, RouteMixin, ControlMixin,
    TeacherNavigation,
):
```

`__init__` 留在 `PIDNavigation`（状态仍由实例持有）；其余方法体原样迁入对应 mixin。`super()` 的调用目标经 MRO 计算与原类一致（见 §1.5），因此回调/控制的基类行为不变。

### 1.3 依赖与命名处理

- 每个 mixin 显式 `from .pid_support import (...)`，没有 wildcard；`pid_support` 不反向依赖类或入口，避免循环导入。
- 唯一一处类内对自身方法的限定调用：`_admit_matched_spline` 里的
  `PIDNavigation.clear_committed_exhausted_capture(proxy)` 改为
  `AdmissionMixin.clear_committed_exhausted_capture(proxy)`，指向同一函数对象（原样语义），并在模块 docstring 说明。
- `pid_entrypoint` 不 import `pid_navigation`（类由 `pid_navigation.main()` 作为参数传入），因此 `python -m ...pid_navigation` 不会二次加载模块；同时支持直接调用 `pid_entrypoint.main()` 时的惰性导入。
- 入口的 `argparse` 描述仍是 controller 模块 docstring（`pid_navigation.main()` 把 `__doc__` 传给入口），`--help` 输出不再变成入口内部说明。

### 1.4 保留的兼容入口

- `python -m embodied_navigation.navigation.pid_navigation --run <run_dir>`：不变（`control/launch/components.py` 未改）。
- `from embodied_navigation.navigation.pid_navigation import PIDNavigation`：不变（类仍在原路径）。
- `pid_navigation.main()`：保留；内部转调 `pid_entrypoint.main(PIDNavigation)`。
- 旧测试路径依赖已迁移：`tests/unit/test_ros_shutdown_lifecycle.py` 的源码解析目标改为 `pid_entrypoint.py`，其余断言（`rclpy.init` 仅一次、`SignalHandlerOptions.NO`、`_main` 捕获 `_ShutdownRequested`、不吞 `RuntimeError` 等）保持不变。

### 1.5 行为验证

| 验证 | 结果 |
| --- | --- |
| AST 等价：49 个原方法逐个 `ast.dump` 对比（`_admit_matched_spline` 仅允许上述限定调用替换） | 49/49 完全一致，无缺失/多余/改动 |
| 模块级 27 个函数/类与原文对比 | 27/27 一致 |
| `_drain_executor` 与原文对比 | 一致 |
| `_main` 与原文对比（允许签名加 `navigation_class`、基类 `PIDNavigation`→`navigation_class`、description 透传三处有意修改） | 一致 |
| `super()` 目标类（14 个方法内共 16 处调用）按新 MRO 计算 vs 原 MRO | 16/16 一致 |
| 假依赖 import（stub `scan_planner_msgs`，本机缺该消息包） | `PIDNavigation` MRO 正确、11 个关键方法存在、入口模块可导入 |
| `python -m ...pid_navigation --help`（stub 消息包，不建节点） | 退出码 0，usage/描述正常 |
| `python3 -m py_compile` 全部新模块 | 通过 |
| unit / regression（pid 拆分后当时） | `Ran 311 tests, OK (skipped=3)` / `Ran 86 tests, OK (skipped=5)`；最终 322/86，见 §2.4 |

### 1.6 顺带修复的确认问题

- `pid_support.py` 中重复 10 次的同一 `_observer_api` 导入已去重（仅保留首次，所有赋值仍在其后），不改变绑定。
- 调用方 README 中"主要调用方是 pid_navigation.py"的过时描述已指向实际负责模块（`pid_control.py`、`pid_admission*.py`、`pid_route.py`、`pid_entrypoint.py`）。

### 1.8 独立复核后的兼容性修复（同日晚）

复核指出 thin facade 只保留了 `PIDNavigation`/`main`，原模块在导入路径上还有 27 个证据/探针 helper 以及 `_drain_executor`/`_main`，且旧文件使用 absolute imports、可直接当脚本运行。已实际修复：

- `pid_navigation.py` 改为 **absolute imports**，因此 `python3 src/embodied_navigation/navigation/pid_navigation.py --help` 与 `python3 -m embodied_navigation.navigation.pid_navigation --help` 都能工作（仍需要 ROS Python 依赖；测试用 `scan_planner_msgs` stub，不启动 ROS/节点）。
- 显式重导出原模块 **31 个模块级定义**（27 个 helper/类 + `PIDNavigation` + `_drain_executor` + `_main` + `main`）；实现仍在 `pid_support.py`/`pid_entrypoint.py`，facade 只做导入。
- `_main` 保留原签名 `_main(shutdown_signals)`，内部绑定 `PIDNavigation` 与 controller docstring；`_drain_executor` 直接指向 `pid_entrypoint` 的同一函数对象。
- 新增 `tests/unit/test_pid_navigation_compat.py`（2 项）：子进程验证 27 个 helper 无一缺失、`_main(shutdown_signals)`/`main()` 签名与 `_drain_executor` 来源；并真实执行直接脚本与 `-m` 两条 `--help` 入口，断言不再出现 relative-import 错误。

### 1.7 仍存限制

- 未启动真实 ROS launch/节点构造：本机缺 `scan_planner_msgs` 等第三方消息包，验证止于 AST 等价、MRO/super 目标、import/`--help` 入口和现有 CPU 测试；节点运行期时序未复测。facade 的 31 个旧导入名与两条进程入口已用 stub 实测（不建节点）。
- 拆分消除了"单文件/单类混职责"，但 mixin 仍共享 `self` 状态；进一步把准入/控制状态拆成显式状态对象是后续可做的解耦，不在本轮强行改（会动行为）。
- `pid_support.py` 仍约 1030 行（导入 + 证据 helper + 探针包装）；本轮只做去重，不做进一步拆分。它们共享同一组观测/回执设施，本轮按"共享支持层"保留为一个模块；若继续拆，可按"回执 I/O"与"探针包装"再分，但需要单独验证导入图。

---

## 2. `simulation/dynanav/capture_go2_rgbd.py`：采集入口的职责拆分

### 2.1 原结构与问题

拆分前该文件 1785 行：整个采集流程是一个模块级 `try:` 块（打开 USD、跑 200 Hz 物理循环、发 ROS、写 result/journals 都在导入时执行），并混有：

- 模块级可变状态：`_last_receipt_wall`、`_journals`、`save()`/`journal()`/`close_journals()`；
- 采集场景（SimulationApp/USD/Go2 生成/接触监控/行人）、传感器安装、rclpy 接口、oracle/learned 建图、策略与力矩、50 Hz 控制 tick、0.005 s 物理步与安全断言、10 Hz 帧/视频；
- 顶部 `if __name__ != "__main__": raise RuntimeError(...)` 导入守卫——只是让 import 立刻报错，没有把副作用移出入口；
- `import` 还会读 `sys.argv[1]`、`mkdir` 输出目录；失败收据依赖 `locals()` 判断哪些组件已构造。

### 2.2 拆分结果（新布局）

| 模块 | 行数 | 职责 |
| --- | ---: | --- |
| `capture_go2_rgbd.py`（入口） | 479 | `CaptureConfig`、`load_experiment`、`parse_config(argv)`、`run(argv)`、`main(argv)`、`run_capture(config)` 编排与失败收据；普通 import 无副作用 |
| `capture/receipts.py` | 152 | `result.json` 初始字典与 stage、0.25 s 节流原子保存（`result.json.tmp`）、JSONL journals、`CaptureContext`、失败收据（traceback 后重抛） |
| `capture/scene.py` | 507 | SimulationApp、headless display delegate、USD stage/坐标轴/地下碰撞修复、Go2 资产生成与驱动所有权、`ContactMonitor`、physics setup、articulation 契约、行人 actor、preflight、`play`/`stop` |
| `capture/sensors.py` | 185 | 相机安装与诊断相机、IMU 传感器、RGB/深度/CameraParams/overview render product、传感器安装收据 |
| `capture/ros_interface.py` | 270 | rclpy init/node、全部 `/dynanav/*` 发布与 odom/approved-command 订阅、回调、IMU/cloud/depth/info/RGB 消息构造、destroy |
| `capture/mapping.py` | 277 | 原始 RGB 归档门、oracle/learned 建图 sink、评分、`MappingSession`（process/finish/finalize_failure） |
| `capture/policy.py` | 199 | Student JIT / RoboGauge 策略加载与收据、`PolicyDriver.step`、力矩与关节速度读回保护 |
| `capture/control.py` | 472 | dynamic hold 构造、速度上限、native/SLAM route controller、50 Hz `ControlLoop.tick`（站立门、批准命令/新鲜度、路线、策略调用）、终止判定 |
| `capture/physics.py` | 215 | 一个 0.005 s 步（Fabric/USD）、时钟/步长断言、effort 读回、姿态/接触/速度/位置门、IMU 时钟与测量 |
| `capture/frames.py` | 251 | 10 Hz 帧：annotator 读取、RGB 归档、建图、ROS cloud/depth/RGB 发布、相机位姿/自过滤收据、周期 RGB-D 快照、概述/RGB 面板 |
| `capture/video.py` | 94 | ffmpeg 概述/RGB 编码进程、1024x512 面板与叠加文字、帧计数、关闭语义 |
| **新包合计** | **2636** | |

实现体不是重写：按原文件行区间原样拷贝后机械重排缩进（原 `try:` 体 4 空格、类方法 8 空格），只做以下已记录的机械替换：

- 模块级可变状态 → 显式对象：`save`/`journal`/`result` → `Receipts`；接触全局 → `ContactMonitor` 属性；`locals().get(...)` → `CaptureContext` 字段；
- `sys.argv[1]`/`sys.argv[2]` 只在 `parse_config` 读取一次，其余用 `config.output`/`config.scene`/`config.experiment`；
- `rclpy.spin_once` 等 → `ros.spin_once`（同一调用）；
- 帧发布/建图块 → 所属对象的方法（`RosInterface.publish_*`、`FrameStream.process` 等）；
- 控制循环里的 `break` → 判定函数返回 `True` 后由循环 break（仅限迁入函数的块）。

### 2.3 入口契约与兼容

- `import embodied_navigation.simulation.dynanav.capture_go2_rgbd` 现在只定义函数/数据类，不读 `sys.argv`、不建目录、不导入 Isaac/rclpy/PIL、不写文件；原 `RuntimeError` 守卫已删除。
- 旧启动方式不变：`experiment.py` 与 `scripts/capture_dynanav.py` 仍设置
  `sys.argv = ["capture_go2_rgbd", <output>, <scene>]` 并 `runpy.run_module(..., run_name="__main__")`；
  `DYNANAV_EXPERIMENT` 环境变量仍是实验 profile 来源；输出目录在 `parse_config`/`run` 被调用后才创建。
- `run(argv)` 接受显式 argv；`main(argv=None)` 供 `__main__`/runpy 使用；两者都不改变旧 argv 语义。
- `experiment.py` 的 provenance `sources` 快照已补上 `capture/*.py` 11 个模块（此前只记录入口文件，拆分后会把实现排除在 source_manifest/快照之外）；`tests/unit/test_dynanav_source_snapshot.py` 新增断言：入口与全部 capture 模块都必须在 sources 中。

### 2.4 行为保持证据

| 检查 | 结果 |
| --- | --- |
| 数值字面量集合（原文件 vs 新包+入口） | 完全一致（只比较数值，不引入新阈值） |
| `result[...]` 键 | 61/61 一致 |
| `result.update(...)` 键 | 32/32 一致 |
| journal 名称 | 9/9 一致 |
| 环境变量名 | 10/10 一致 |
| `/dynanav/*` 话题字符串 | 8/8 一致 |
| `assert` 语句数 | 1/1 一致（原唯一的 `open_stage` 断言仍在 `scene.open_stage`） |
| 语句行包含性（1710 条非注释代码行） | 149 条未逐字出现，逐条核对均为上述机械替换/移除的导入守卫/docstring，无行为语句丢失 |
| 普通 `import`（子进程、临时 cwd） | 退出 0、stdout 空、cwd 无新文件、无 Isaac/rclpy 导入 |
| `runpy` 入口（stub 环境无 isaacsim） | 走到 `start_simulation_app` 的首个真实依赖错误 `ModuleNotFoundError: isaacsim`，此前已写出 `result.json` 失败收据（error+traceback），与旧脚本失败路径一致 |
| 针对性测试 | 14 项通过（原 12 项保持，新增 import 无副作用 2 项）；另加 6 项真实 orchestration fake 依赖测试（见 §2.6） |
| 全套 unit / regression | `Ran 322 tests, OK (skipped=3)` / `Ran 86 tests, OK (skipped=5)` |
| `py_compile` 全部改动的 .py | 通过 |

### 2.6 独立复核后的接线与故障清理修复（同日晚）

独立复核用真实 signature 绑定与 fake 依赖执行发现两处真实回归，已修复：

1. **`PhysicsStepper` 漏传必填 `mount`**：入口构造只传了 `imu_mount`，`physics.PhysicsStepper` 的 kw-only 签名还要求 `mount`（相机位置计算用 `camera_rig.mount`）。现在显式传 `mount=camera_rig.mount`；新增的零步 orchestration 测试用**真实 `PhysicsStepper` 类**（不是 fake）跑通入口，并在 `__init__` 记录 kwargs 断言 `mount`/`imu_mount` 都是 `camera_rig` 上的同一对象；独立绑定脚本对入口 36 个模块级 factory 调用按真实签名重新绑定，0 mismatch。
2. **首 tick 失败收据遮蔽 primary 异常**：`context.art` 从未绑定，但循环每 tick 会绑定 `context.q/qd`；`record_failure` 访问 `context.art.dof_names` 抛 `AttributeError`，导致原异常被替换、receipt 停在 startup 且无 error、app/video/ROS 清理未执行。现在：
   - `context.art = art` 与 `context.ros = ros` 在构造后立即绑定；
   - `record_failure` 改为**逐项尽力完成**：先把 `error`/`traceback` 写入 receipt，再分别尝试 journals、RGB archive receipt、actuator state、mapping finalize、video close、ROS destroy、app close；任一步失败只记录到 `failure_cleanup_errors` 并继续尝试其余**本 run 已拥有**的资源，最后仍 re-raise 原始异常；
   - app.close 二次失败不再遮蔽 primary；receipt 写入失败后仍会打印 traceback，并在清理错误收集后再尝试写一次；
   - 正常路径语义不变（成功路径的 finish/close/destroy/stop 顺序未改）。

新增 `tests/unit/test_dynanav_capture_orchestration.py`（6 项，fake SDK/ROS/PIL，零或一步物理）：零步正常路径（真实 `PhysicsStepper` 签名）、一步正常路径（rows/journal/video 收据）、首 tick sentinel（receipt 含 primary error + actuator state、无 `AttributeError`、mapping/video/ROS/app 清理均发生）、清理二次失败（mapping/archive/video/app close 任一抛错仍尝试其余资源，receipt 记录 `failure_cleanup_errors`，最终抛出的仍是同一个 sentinel 对象）、receipt 写入本身失败（仍打印 traceback、仍尝试 video/ROS/mapping/app 清理，primary 不被替换）。

### 2.5 仍存限制

- 没有 Isaac Sim/ROS/GPU 运行，运行期 result/journals/视频没有重新生成；验证是静态等价 + 首个依赖错误前的入口路径 + CPU 测试。
- 失败收据把原 `locals()` 探测改成 `CaptureContext` 字段记录；字段设置顺序与原变量赋值顺序一致，但"异常恰好在两个组件构造之间"这种极端时序只能尽量保持，未做故障注入验证。
- `video.py` 把原在视频创建点执行的 PIL 导入移进 `OverviewVideo.__init__`（同一执行位置；PIL 缺失时失败点仍在该构造处，不提前到 import 模块）。
- 原文件里无害的历史写法（如 `calf_views` 未使用局部变量）按原样保留，未借机清理。


---

## 3. 测试与文档同步

- `tests/unit/test_ros_shutdown_lifecycle.py`：解析路径迁移到 `pid_entrypoint.py`，断言不删减。
- `tests/unit/test_dynanav_odometry_source.py`、`tests/unit/test_dynanav_learned_wiring.py`：（随 capture 重构迁移，见 §2。）
- `src/embodied_navigation/navigation/README.md`、`docs/architecture.md`、根 `README.md`、control/planning/common 相关 README：同步新的模块归属与入口。
- `src/embodied_navigation/simulation/dynanav/README.md`：（随 capture 重构同步，见 §2。）

## 4. 验证命令与结果汇总

```sh
# 结构等价
python3 .dsh-subagents/2026-11-09T07-25-26-735Z-81d359de/notes/verify_pid_split2.py
# 假依赖 import + MRO
PYTHONPATH=src:$PYTHONPATH python3 .dsh-subagents/.../notes/import_smoke.py
# 旧 -m 入口（stub 消息包，不建节点）
PYTHONPATH=src:$PYTHONPATH python3 .dsh-subagents/.../notes/entry_smoke.py
# 测试
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit
PYTHONPATH=src python3 -B -m unittest discover -s tests/regression
```

结果：unit `Ran 322 tests, OK (skipped=3)`（314 为第一轮拆分后计数；随后 +2 PID facade/直接脚本测试、+6 capture orchestration fake 依赖测试）；regression `Ran 86 tests, OK (skipped=5)`；结构等价/入口/import smoke/真实签名绑定全部通过。

## 5. 范围声明

本轮完成的是**结构重构与离线验证**，不是功能验收：没有运行真实仿真、ROS 节点、模型/GPU 推理、训练或机器人；没有提交/推送；没有改权限。controller 行为一致性由"方法体 AST 完全一致 + 16/16 super 目标一致 + facade/入口 stub 实测 + 现有测试通过"支撑；capture 除静态等价比对外，还用 fake SDK/ROS 依赖真实执行了 `run_capture` 的零步/一步/首 tick 失败/二次清理失败路径。真实 launch 下的时序/资源行为仍需独立复核。

## Codex 最终独立核查

见 [核查记录](dsh-structural-independent-review-20261109.md)。DSH 完成主体重构和退回修复后，Codex 补了 traceback 输出失败的清理保护及回归，去掉重复字段/未使用导入，并补 capture 包 README。最终完整 unit 为 323 项（skip 3）、regression 为 86 项（skip 5）；原有阈值和真实仿真暂停状态保持。
