# path_searching：投影A*引导

本包提供`AStar::AstarSearch`，对B样条碰撞段寻找绕行引导；它不是整个F1→F3全局路线的生成器。

输入起终点、grid map与搜索分辨率；输出离散引导点，交给`BsplineOptimizer::initControlPoints`计算rebound基点/方向。当前搜索池100³，但邻域使用XY八邻域，Z按起终点投影比例插值到搜索面。每条方向估计yaw，并查方向相关双圆柱碰撞。

核心在`src/dyn_a_star.cpp`及`include/path_searching/dyn_a_star.h`。搜索有约0.2s墙钟限；失败不证明全局无路，也不允许把未完成搜索当候选通过。

从[plan_manage](../plan_manage/README.md)跟进调用，普通运行仍由根`start.py`。调试应确认当前grid、投影高度与起终点，再看超时/封闭；不要把它理解为自由26邻域全3D搜索。算法上下文见[SCAN说明](../../README.md)和[技术报告](../../../../docs/technical_report.zh-CN.md)。
