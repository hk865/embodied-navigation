# 系统模块与开发入口

## 代码与数据边界

真实开发源码位于src/embodied_navigation，配置在configs，原生第三方源码在third_party。
无代码或文档从外部工程、历史副本或Documents索引导入。assets、models、runs可以是本地数据链接；它们被Git和Docker构建输入排除。仿真地图、机器人资产和策略权重须按配置另行提供，不把私有数据打包为源码。

## 从入口阅读实际流程

1. start.py调用launch/main.py。prepare.py从configs准备本次运行，demo.launch.py只装配进程和配置。
2. simulation负责世界、物理、传感器生命周期。perception/sensors将相机、LiDAR、IMU输入交给localization_mapping/frontend；third_party/fast_livo2提供实际SLAM实现。
3. localization_mapping/backend执行注册与图优化，feedback_state.py在已接受的SLAM回调处附带生产者和时间信息。它不把仿真真值重标为SLAM。
4. planning的参考路线、SCAN和PCT给出路径；navigation/pid_navigation.py管理既有任务，navigation/task.py实现从原多航点桥提取的等待、跟随、到点稳定、转向、完成与失败状态。building_navigation.py只允许显式SLAM模式。
5. control/tracking和control/relative_pid.py计算跟踪命令，control/parking执行停车相关控制律。navigation判断何时触发及何时完成停车，robot/unitree负责策略、关节契约和执行桥。没有在launch中实现任务状态机。
6. evaluation保存离线指标；simulation/dynanav只负责DynaNav场景接入和RGB-D capture。视觉策略训练另属于RL_for_unitree，不与导航部署混放。

## 安全门禁是实际调用，不只是约定

- simulation/safety_state.py从原native64帧产生PhysicalSafetyState，来源是simulation_truth / gazebo.native64.preupdate，时钟为gazebo_simulation，包含run_id与采样/帧时间。
- control/safety/simulation_gate.py作出fault、stop和reason决策。默认Student/Teacher循环及RoboGauge循环逐帧调用同一个门禁。
- 保留原阈值、宽限和锁存优先级：策略非有限值、执行器故障、倾倒/低离地；故障先锁存，随后才查询地形。身体接触在此模块仍仅诊断，不暗增停车条件。
- 错误来源、跨run、错误时钟或不合法采样时间立即拒绝并请求故障阻尼/停止。typed provenance是程序内数据契约，不是密码学来源认证。
- control/safety/feedback_source.py另检查停车使用的canonical SLAM反馈：frame、producer、nav_ground_truth_used=false及各自时间域。ROS头/回调时间仅在同一ROS时间域比较，单调接收时间单独保留。
- 原有输入新鲜度、碰撞、路径交付、通信/C++执行器保护继续有效，不宣称所有保护已经被合并成一个类。尚无实机安全后端；REAL_SENSOR只是保留类型。

## 测试入口与边界

- tests/unit：纯CPU控制、导航状态、传感几何、来源/时钟、安全边界和原公式等价。
- tests/regression：既有定位、规划、停车与策略契约；test_native_safety_integration.py使用真实观测构造与假socket执行两个循环，验证门禁调用及动作不变。
- start.py --check只读检查依赖；--prepare-only只准备运行文件；普通启动会实际运行系统，--duration可限制墙钟时间。不要将准备成功当作仿真完成。
- scripts/replay_controller.py只离线比较已记录驾驶命令，不是整栈路线验收。
- navigation/robogauge/benchmark.py保留prepare与显式--run边界；RoboGauge的45D直接观测接口与既有Student/Teacher的247D native中间观测适配链分别保留，不把它们当成相同的策略输入或actuator契约。

历史wire topic和部分日志名仍带truth字样以兼容消费端；允许导航的判据是实际生产者、标志与时间契约，不是名称。原始源码/日志仍保留在Git和Docker输入之外，不作为新工程运行依赖。
