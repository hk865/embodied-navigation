# plan_manage：状态机与规划编排

本包拥有SCAN可执行、重规划状态机、参考接收和候选发布。当前工程由Python epoch主管启动本包的`scan_planner_node`；本包的演示controller不负责实际Student控制。

## 核心入口

- `src/scan_planner_node.cpp::main`：SingleThreadedExecutor。
- `src/scan_replan_fsm.cpp::SCANReplanFSM`：10ms FSM、50ms安全timer，mode3参考/请求，实测状态与局部目标。
- `src/planner_manager.cpp::SCANPlannerManager::reboundReplan`：多项式种子→线性Z参考→B样条→碰撞引导/优化→可行性/refine。
- `fresh_reference_protocol.h`、`measured_start_state.h`、`fresh_child_odometry_ack.h`：当前参考和真实子进程消费边界。

## 输入输出与配置

输入body/sensor pose、registered cloud、Path参考和execution frozen；输出Bspline及request/epoch关联元数据。当前完整topic表见[SCAN总览](../../README.md)。外部模式保留完整全局终点，局部目标仍需free，不通过随意缩短路线声称任务完成。

有效参数由根`src/scan_runtime_parameters.py`覆盖`config/planner.yaml`。当前horizon=.65m、replan阈值=.35m、连续失败上限50；起始速度滤波τ=.4s。求解器的vmax/amax之外仍有tolerance检查，细节见[技术报告](../../../../docs/technical_report.zh-CN.md)。

使用根`start.py`；独立`launch/run.launch.py`属于上游/开发入口，未经当前整链等价验证。调试按epoch→参考消费→实测起点→局部目标→求解→外层接纳的顺序看日志。frame或generation变化后不可复用旧child占据和轨迹。协议/时间预算不是导航PASS证书。
