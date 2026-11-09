# embodied_navigation 源码包

本包按“仿真/传感器 → 感知 → 定位建图 → 规划 → 任务与跟踪控制 → 机器人执行”的数据流组织导航系统，包内是自有 Python/C++ 源码与 ROS 2 装配代码，不含第三方算法实现、运行数据或构建产物。

## 按数据流阅读

1. **启动与装配**：仓库根 `start.py` 调用 [launch](launch/README.md) 的 `main.py`，校验配置与依赖、生成单次运行目录，再由 [launch/launch](launch/launch/README.md) 的 `demo.launch.py` 包含定位、规划、控制、仿真四个模块 launch。
2. **世界与传感器**：[simulation](simulation/README.md) 生成 Gazebo 世界，启动策略 actor、传感器桥与仿真进程，把相机/LiDAR/IMU 数据送入 ROS。仿真原生状态还服务于策略本体观测、COM 速度内环及安全/评测；生产 SLAM 和上层导航位姿不以真值替代。
3. **传感流适配**：[perception](perception/README.md) 与 [perception/sensors](perception/sensors/README.md) 负责中继、自身回波过滤、采样和传感门禁；跨边界的数据类型定义在顶层 [sensors](sensors/README.md)。
4. **定位建图**：[localization_mapping](localization_mapping/README.md) 接入 FAST-LIVO2 前端（真实算法在 `third_party/fast_livo2`，本包只是接入与包装），后端负责注册与图优化，并输出带生产者和时间信息的定位反馈。 [semantic_mapping](localization_mapping/semantic_mapping/README.md) 提供观测分层、对象历史和只读结构化查询，实验接入范围单独记录。
5. **规划**：[planning](planning/README.md) 提供参考路线、SCAN 调度和 PCT 路线生成（PCT 真实算法在 `third_party/pct`）；admission 负责路径准入与交付。
6. **任务与跟踪控制**：[navigation](navigation/README.md) 维护任务状态与既有 PID 导航；[control](control/README.md) 计算跟踪/停车命令，并包含仿真安全门禁与定位反馈来源检查。
7. **机器人执行**：[robot](robot/README.md) 下的 [robot/unitree](robot/unitree/README.md) 负责 Go2 观测、策略推理和执行桥。当前 Student 用 exact45 推理，历史 Teacher 247D 观测保留作诊断；RoboGauge 使用另一套独立 45D 接口，观测与 actuator 契约不能混用。
8. **离线与横切**：[evaluation](evaluation/README.md) 提供离线轨迹指标及独立进程资源采样；[common](common/README.md) 提供配置、路径、时钟、通信与运行遥测等公共件。

## 包边界与实现归属

- 自有实现在本包；仓库 `configs`、`scripts`、`tests`、`docs` 分别放配置、脚本、测试与构建/来源说明。
- 第三方实现（FAST-LIVO2、SCAN、PCT、Vikit、Livox 驱动、FastDDS 补丁）在 `third_party`，本包只做接入与装配。
- `install/` 只是本机构建产物，缺文件通常表示尚未构建，不代表缺源码。
- native64 的本体状态用途与导航定位来源分开：前者用于现有仿真策略/控制内环及安全/评测；上层定位与导航位姿按实际生产者、标志和时钟契约接收 SLAM 反馈。

## 入口与验证

- 以下命令从仓库根目录执行；源码布局的 Python 模块入口需设置 `PYTHONPATH=src`，或先安装本包。
- 运行入口：`python3 start.py --check` 解析配置并检查依赖与资产；`--prepare-only` 准备运行文件而不启动 ROS/仿真；普通启动会实际运行系统，可用有限正数 `--duration` 限制墙钟时间。
- 测试入口：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v` 与 `... -s tests/regression -v`，均为离线 CPU 测试，不启动仿真。
- 原生构建、依赖与许可边界见 [构建说明](../../docs/dependencies/BUILD.md) 与 [依赖总览](../../docs/dependencies/README.md)。

## 未验证边界

Isaac/DynaNav 端到端、实机、完整路线与 GPU 训练尚未验证；prepare 成功、测试通过或 `install/` 有文件都不等于完整系统可运行，运行仍需匹配的 ROS/Gazebo 环境、资产与权重。
