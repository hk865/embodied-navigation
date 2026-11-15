# navigation/robogauge

本目录只剩 RoboGauge 的独立 native-pose 路线跟踪器；它不是主相对 SLAM 导航任务，也不再包含基准编排或命令准入。

## 边界

- 做：`route_tracker.py` 提供纯 measured-pose 路线跟踪辅助类，有回归用例。
- 不做：策略训练、45D 观测构造与执行器 socket。它们分别在 `../../robot/unitree/robogauge/`。

## 目录结构

```text
src/embodied_navigation/navigation/robogauge/
├── README.md          # 本文件
├── __init__.py        # 只有包 docstring（route tracking），无重导出
└── route_tracker.py   # 329 行，纯 measured-pose 路线跟踪器（无 ROS/仿真副作用）
```

## 代码、日志与配置

| 文件 | 性质 | 判断依据 |
| --- | --- | --- |
| `route_tracker.py` | 活代码（库） | `class RouteTracker`（`route_tracker.py:18`）；被 `simulation/dynanav/capture/control.py` 与 `simulation/dynanav/slam_route.py` 使用，也是回归用例的对象 |
| `__init__.py` | 只有 docstring | 无 re-export；调用方必须写完整模块路径 |

本目录**不写自己的日志**，也不读 `configs/`。`route_tracker.py` 是零内部依赖的纯库（只依赖标准库 `math`），没有 ROS/仿真副作用；`tick()` 212 行（L118–329）是单一路径控制律。

## 测试

`tests/regression/test_robogauge_route_tracker.py` 覆盖路线跟踪行为。

## 限制

完整 Isaac 运行、GPU 与实机均未验证，历史 N=1 结果不是可靠性统计。详见 `../../../../docs/robogauge/README.md`。
