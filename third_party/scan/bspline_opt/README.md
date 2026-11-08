# bspline_opt：B样条与rebound优化

本包把种子点参数化为三次均匀B样条，并优化内部控制点。轨迹及其导数交给planner做可行性检查和发布；本包不直接控制机器人。

## 入口与目标

- `uniform_bspline.cpp`：参数化、求值、导数与时间伸长。
- `BsplineOptimizer::initControlPoints`：识别碰撞段、调用投影A*、生成基点/推离方向。
- `combineCostRebound`：smoothness + collision + feasibility。
- `combineCostRefine`：smoothness + fitness + feasibility。
- `rebound_optimize`：L-BFGS，memory16、最多200次迭代；首末各3控制点固定。

平滑项是控制点三阶差分平方和，不应直接当作物理时间归一的jerk积分。碰撞项惩罚沿rebound方向的距离不足，速度/加速度项惩罚控制点差分超限。两套合成梯度均将Z行置零，保持manager初始化的高度参考，优化主要在XY。

## 参数与调试

当前vmax=.5m/s、amax=.15m/s²由runtime覆盖，其他权重/间距沿用planner默认。动态最终检查额外tolerance与时间重分配在manager层，读取max_jerk不等于硬约束jerk。公式、权重语义及限制见[技术报告](../../../../docs/technical_report.zh-CN.md)。

从[plan_manage](../plan_manage/README.md)调用，输入为种子点/边界导数与grid；输出控制点、时间间隔与成功状态。调试需分清种子失败、A*失败、梯度终止、可行性失败与外层准入拒绝。使用根入口和[SCAN配置说明](../../README.md)，不要仅凭优化返回成功宣称可执行。
