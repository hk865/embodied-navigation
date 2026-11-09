# Embodied Navigation

一个按系统职责组织的导航工程。开发源码、配置、文档都在本仓库内，不从 Documents 或其他本机自建工程加载代码。视觉策略训练独立位于 RL_for_unitree；这里只保留导航部署所需策略推理和机器人契约。

## 系统结构

各自有模块的职责、入口和边界见 [源码模块索引](src/embodied_navigation/README.md)，此前整理的 46 个模块 README 及后续模块说明均从该索引进入。此前 589 个代码/配置目标的可读性报告见 [全量可读性总入口](docs/review/full-readability/README.md)（DSH 全量实现，Codex 独立事实复核，DSH 串行合入）。此前的 [本轮验证记录](docs/review/readability-verification.md)、[审计明细](docs/review/validation-and-threshold-audit.md) 与 46 个模块 README 索引保留为历史记录。

- src/embodied_navigation/simulation：Gazebo/Isaac 场景与物理生命周期。simulation/dynanav 负责 DynaNav 场景中的真实 Go2 RGB-D capture，不承载整个导航系统。
- perception：深度反投影、自身点过滤、传感流适配。
- localization_mapping：FAST-LIVO2 接入、相对里程计、地图配准和后端。实际 FAST-LIVO2 源码在 third_party/fast_livo2，不依赖另一个本机工程。
- planning：路线、SCAN 调度、PCT 以及路径交付。实际 SCAN/PCT 源码在 third_party。
- control：单步跟踪/控制律、碰撞与支撑检查、parking停车执行、safety统一仿真门禁和定位反馈来源检查。
- robot/unitree：Go2 观测、关节顺序、策略推理和执行桥。没有训练流程。
- navigation：完整任务状态和模块协作。task.py 是从原多航点 bridge 提取的真实跟随/到点稳定/转向/停车/失败状态机；已有PID任务状态位于 pid_navigation.py，Building任务位于 building_navigation.py，任务组件位于 mission/；ROS适配位于 ros_node.py、relative_slam_node.py。
- evaluation：轨迹长度/误差等离线指标及独立进程资源采样。仿真真值还用于现有策略本体观测和 COM 速度内环，不作为生产 SLAM 定位输入；显式 DynaNav 真值控制诊断另行记录来源。
- launch：模块装配、环境、进程生命周期。业务任务状态不放在 launch。
- configs、scripts、tests、docs：功能配置、少数可发现入口、回归测试和说明。

## 两个运行边界

1. Native Gazebo 系统：start.py 装配已有导航模块。Dockerfile 从公开 ROS Jazzy、Gazebo Harmonic、CPU Torch 2.7.1 构建，必要自建源码均包含，FAST-LIVO2/SCAN/PCT 与补丁也在仓库中。
2. Isaac / DynaNav：simulation/dynanav/capture_go2_rgbd.py 是已有真实 capture 实现，navigation/relative_slam_node.py 是分离后的任务节点。它们仍需匹配的 Isaac 安装、场景与机器人资产。scripts/capture_dynanav.py 提供明确的帮助、文件检查和显式启动入口，帮助和检查不会启动Isaac。原机使用 Isaac Sim 6.0.1 RC；该版本的公开可获取性尚未核实，不能声称一份 Native Docker 镜像就能完整复现 Isaac。

本轮 DynaNav 的楼梯通行、分层地图、动态行人与 LIO/LIVO 实测、启动命令和已知阻碍见 [实验记录](docs/experiments/dynanav-20261109.md)。历史原话及尚未确定的研究参数见 [动态语义建图约定](docs/research/dynamic-semantic-mapping-agreement.md)，其他文档见 [文档索引](docs/README.md)。这些隔离实验不等于生产导航整栈验收。

## 快速检查

- CPU单元测试：PYTHONPATH=src python3 -B -m unittest discover -s tests/unit -v
- 既有源码回归：PYTHONPATH=src python3 -B -m unittest discover -s tests/regression -v
- 干净公共镜像CPU验证：docker build -f docker/Dockerfile.cpu -t embodied-navigation-cpu .
- 原生源码验证：python3 docker/verify_native_sources.py --compile-pct
- Native完整镜像：docker compose --env-file .env.local build
- 依赖和资产检查：python3 start.py --check，或先设置 PROJECT_DATA_ROOT 再运行 docker compose --env-file .env.local run --rm navigation

Docker/Compose默认命令是检查。`python3 start.py --check` 解析配置、校验 ROS domain 的数值范围，并检查所需解释器、资产和构建产物是否为文件；它不验证 ROS 域占用、资产内容或系统可运行性。显式加 `--resource-check` 才检查本机 CPU 负载和可用内存；`--check --resource-check` 也会执行该资源检查。`install/` 缺失通常表示本机尚未构建，应按 [原生构建说明](docs/dependencies/BUILD.md) 或根 Dockerfile 生成，不把二进制加入 Git。直接运行 `python3 start.py` 会实际启动系统；可用有限正数 `--duration` 限制墙钟运行时间。

## 数据、权重和资产

Git不包含数据、日志、地图、权重、USD/纹理、图片、视频、build/install产物或个人路径设置。设置 PROJECT_DATA_ROOT 指向 6.project_data；原生运行可先用 python3 scripts/configure_data.py --data-root /path/to/6.project_data --link 建立三个数据入口。Compose将 embodied-navigation/runtime_assets/{assets,models,runs} 显式挂载。场景和DynaNav实验数据位于该项目的数据目录，清单见 configs 中的资产manifest。assets/models/runs 是本地被忽略的数据入口，代码不是软链接。

这些资产不是从其他本机自建工程临时借用：必要的部署USD、已有Student权重和测量地图已另存导航数据目录并逐个校验。换机器仍需提供有权使用的相同数据，或按配置换成公开资产。仓库不伪造这些自有权重/地图的公开下载地址。

## 验证状态与限制

2026-11-09 较早阶段的独立复核、两项启动参数修复及当时 54 项单元/62 项回归结果见 [独立复核记录](docs/verification/independent-review-2026-11-09.md)。该阶段 5 项可选 Torch 测试跳过，未重跑仿真。后续 DynaNav 实际运行与最新测试另见上方实验记录。

2026-11-08 历史验证（针对当时的源码快照，不能代替后续修改的运行验证）：

- 当时源码48项单元测试通过，包括1000组原PID公式精确对比、导航任务、停车来源契约、生产SLAM入口及仿真安全门禁。官方ROS Jazzy纯CPU镜像的当时结果见docs/verification/cpu-container.json。
- 无数据的干净Git候选导出：62项回归成功结束；7项因未提供Torch或私有测量地图而明确跳过。双后端门禁集成和原native fixture另在装有CPU Torch的公共依赖容器实测通过；RoboGauge真实权重30帧输出与原JIT一致。
- FAST-LIVO2/SCAN等11个ROS包、PCT、4个自有原生模块、私有DDS补丁测试已在隔离公共依赖环境中从本仓库源码编译。完整Docker镜像已从公开ROS基础镜像构建成功，实际资源preflight通过；最终源码刷新和验证结果记录于docs/verification。
- 最终自足镜像45秒隔离软件栈通过：只挂本项目assets、models和新runs，不挂源码或其它工程的install；全部关键源码SHA匹配。1330次CPU策略推理、798帧相机、797帧LiDAR、57580个地图点；SCAN接受2条轨迹，22个健康非零命令样本及25个native执行确认样本。103条仿真安全门禁来源遥测全部有效，无物理fault；末段原有命令新鲜度门禁因来源超时拒绝并置零。15个组件全部退出0，日志无Traceback或未取回异常。详见docs/verification/software-startup.json。
- 此前发现的SIGINT关停竞态已修复：共用信号生命周期先退出spin和排空回调，最后关闭ROS context；真实callback异常仍传播，子组件非零退出会使主入口失败。48项单元包含6项生命周期测试、4项退出汇总测试及7项Building入口测试。
- 真实记录驾驶层回放1052/1052个样本通过：按目标点、航向误差和距航点距离独立唯一选帧，不用待检命令选帧，最大命令差异1.91e-14。它不是任务状态机或整栈证明。

可选Building Student入口的资源记录脚本与SCAN配置路径已实际补齐，仅允许SLAM模式；真实prepare-only通过，尚未运行Building分支，详见docs/dependencies/BUILDING-ENTRYPOINT.md。

仍未验收：生产导航整栈的完整路线、实机、策略训练和完整 Isaac 生产链路。docs/robogauge/isaac_closure_audit.json 记录较早的策略闭包审计；本轮 DynaNav 已补齐显式会话碰撞、RoboGauge 执行与采集诊断，并分别记录真值控制路线、首段 LIO 动态闭环和视觉旁路定位证据，不能据此声称生产链路已完整复现。

原测试3个停车样例缺少当前要求的pose_producer字段，在原工程也失败。本仓库更新fixture并纠正一处原源码要求真值标志为true的冲突：现在只接受显式非真值且生产者与时钟有效的SLAM反馈，没有放松阈值。运行源码和来源契约另见docs/architecture.md。

## 许可证和来源

已保留第三方源码、版本、许可证声明和补丁来源，详见 docs/dependencies。尚有Vikit授权来源、组合分发、自有源码/模型与部分资产来源问题需要确认，因此不能宣称公开发布许可已齐备。分发前请核对这些许可边界。

## 历史副本

dynanav/、history/、perception_history/ 是保留的原始/阶段副本，已从Git和Docker输入排除，不是本工程开发入口。这些本地目录不属于运行依赖，也不会进入Git或Docker输入。
