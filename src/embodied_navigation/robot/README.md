# robot：机器人契约与策略部署

`robot` 是机器人观测、关节契约与策略推理的顶层命名空间，当前只有 Unitree Go2 部署实现。

## 结构

- `robot/__init__.py` 明确本包“不含策略训练代码”。
- [unitree/README.md](unitree/README.md)：Go2 执行桥、Student 策略部署与 RoboGauge 部署，是实际实现所在。

## 边界

- 不训练、不采集数据、不导出模型；没有实机后端。本包自身不注册 ROS 节点，实际节点由子目录的 `bridge.py` 与各模块 `launch/components.py` 启动。
- 顶层 [sensors/](../sensors/README.md) 只是跨感知/定位/控制的数据契约，[perception/sensors/](../perception/sensors/README.md) 是感知侧 ROS 适配；二者都不是本目录的策略输入。
- 当前有两条部署路径：Student 使用 exact45 推理，`unitree/policy/native` 保留历史 Teacher 247D 观测用于诊断/适配；`unitree/robogauge` 使用独立的 45D 直接观测接口。两条路径的输入和 actuator 契约不能混用；普通导航不加载 Teacher actor。

## 测试

本目录无专用测试；[unitree/README.md](unitree/README.md) 所述的策略与仿真安全门禁测试位于 `tests/regression`。
