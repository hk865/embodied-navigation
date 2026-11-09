# 术语与易混概念对照

本页只解释本仓库中反复出现、且容易读错的词。数值与契约以对应源码/配置为准；行号会漂移，按文件与符号查找。

## 运行与入口

- **四条执行路径**：A `start.py`/`demo.launch.py` 原生主路径；B `launch/building.py` SLAM-only；C `navigation/robogauge/benchmark.py`；D DynaNav/Isaac 实验。共享代码但不共享进程图与话题，见 [architecture.md](architecture.md)。
- **run 目录**：一次运行生成的目录（`runs/<id>/` 或 DynaNav 的 `experiments/<name>/`）。里面是本次有效参数、日志与收据；`configs/` 中的 base 值不等于本次有效值。
- **`navigation_profile.json`**：`launch/prepare.py` 把 profile 深拷贝后写出的本次参数快照。启动器有几处 pin（强制覆盖），现在会在 stderr 报告与配置不一致的值。
- **`supervision.json`**（DynaNav）：`scripts/run_dynanav_online_person.py` 写出的进程级监督记录，只证明进程启动/退出顺序与退出码，不代表导航或感知通过。

## 定位与来源

- **provenance（来源）标签**：数据里跟随记录的 `pose_producer`、`source`、`pose_source` 等字段。它们是被代码当条件读取的契约，不是注释。
- **`navigation_ground_truth_used=false`**：导航实际消费的位姿不是仿真真值。真值仍用于策略本体观测、COM 速度内环、安全/评测与显式诊断，见 `docs/architecture.md`。
- **canonical SLAM 反馈**：`/demo/slam/body_odom` 上带生产者与时间信息、`navigation_ground_truth_used=false` 的反馈；停车与安全门禁按它判断。
- **raw vs corrected odom**：`odom_adapter` 默认发布 `/demo/slam/{body,lidar}_odom`，但默认 launch 把它 remap 成 `/demo/slam/raw_*`；corrected 同名话题由 `localization_runtime` 发布。同名话题在不同图里含义不同。
- **post-VIO**：DynaNav 实验里由独立 LIO/LIVO 观测器输出的实测位姿（`/dynanav/<ns>/post_vio_odometry`），用于路线控制与在线建图；不等于生产 SLAM 反馈。
- **oracle vs learned**：oracle 指用仿真真值位姿/掩码的分支（诊断或旧实验）；learned 指 RGB 模型推理 + 实测 post-VIO 位姿的分支。两条分支的输入与结论不能互相替代。
- **LIO / LIVO**：LIO 只用 LiDAR+IMU；LIVO 另加相机视觉更新。`run_dynanav_lio.sh --mode` 选择，输出命名空间用 `--observer-namespace` 隔离。

## 状态机与任务

- **默认任务状态机**：`navigation/base_node.py` 的 `Navigation.control()`（idle/running/succeeded/failed/stopped 与 drive/pre_turn/align/settle 相位）。这是 `demo.launch.py` 实际运行的。
- **`navigation/task.py`**：独立 `relative-slam-navigation` 入口的 shuttle 状态机，不在默认 launch 中。
- **mission**：目标/参考层。`mission/mission_runtime.py` 是默认 launch 的 `mission` 进程；`mission/state_machine.py` 在本检出没有实例化点；`mission46`/`continuous_route` 契约默认关闭。
- **admission / fence**：路径准入证明。默认启用 `r85_auto_reference_contract` 时走 `AutomaticRouteFence` + `R85PhysicalPathGate`；`FrozenFence` 是未闭合的 fallback。
- **hold / resume**：在检测到动态障碍或准入失败后暂停、条件解除后继续；与停车（parking）是不同阶段。

## 观测与策略契约

- **native64 / 247D / exact45**：仿真原生状态（64 维）→ 历史 Teacher 中间观测（247D，现仅诊断）→ 当前 Student 推理输入（exact45）。三者不是同一个契约。
- **RoboGauge 45D**：另一条部署路径的直接观测，有独立 scale/关节顺序，不是 exact45 的裁剪，也不含 Student 的 COM 速度 PID。
- **`navigation_command.json` 与 teacher socket**：`robot/unitree/bridge.py` 把 `/demo/cmd_vel` 原子写成命令文件，策略 worker 读取后经 Unix socket（`TEACHER_SOCKET`）交给 `libteacher_actuator.so`。文件是中间层，socket 才是执行器接口。
- **速度上限分层**：cascade 配置、bridge fallback、Student `command_bounds`、RoboGauge 门、`base_node.max_speed` 各管一段，数值不同是设计，不是待统一的不一致。见 `robot/unitree/README.md` 的表。
- **features / R 编号开关**：`configs/launch/settings.json` 的 `features` 字典，键名多为 `R\d+_*`。只有部分键在 `src/` 有读取点；部分探针由 `launch/main.py` 强制为 `"0"`。看到配置写 `"1"` 不代表本次运行开启。

## 数据与时间

- **时钟域**：ROS/sim 时间、steady wall 时间、native 物理时间、DynaNav IMU 源时间（`round(it*1e9)`）。跨域比较必须显式转换；同一函数里出现的两个时间戳未必同域。
- **source_ns / receipt**：观测的源时间与接收收据。源龄、墙钟龄分别判定，不能混用。
- **ROS_DOMAIN_ID**：同一次 DynaNav 实验的 worker/observer/capture/查询终端必须一致；person worker 自身不依赖 ROS，但 capture 与 observer 依赖。
- **DYNANAV_RUN**：三终端共享的同一次实验目录；worker 通道、LIVO 观察器输出与 capture 都写在它下面。
