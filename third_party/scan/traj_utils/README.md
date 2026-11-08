# traj_utils：轨迹与显示工具

本包提供多项式轨迹种子及规划可视化，被`plan_manage`调用。`polynomial_traj.cpp`处理单段/多段及min-snap等多项式表达；`planning_visualization.cpp`发布调试标记。

输入为起终状态、途经点和时间等，输出供B样条初始化的连续多项式及RViz标记。实际局部轨迹以`Bspline`候选与元数据为准，显示曲线不是执行ACK。

包本身没有独立日常启动命令；用根`start.py`让manager按当前参数调用。调试时区分world/reference坐标、trajectory起始时间与ROS仿真时间；只看图形无法验证同源/epoch及静态连续空间准入。调用关系回到[plan_manage](../plan_manage/README.md)，整体原理见[SCAN](../../README.md)。
