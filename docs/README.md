# 工程文档索引

- [系统模块与开发入口](architecture.md)：源码职责、数据边界、运行与测试入口。
- [动静态语义地图与环境编译器：历史原话、研究目标及实现边界](research/dynamic-semantic-mapping-agreement.md)：已核实用户原话、消息出处、未定参数与 DynaNav 工作基线。
- [DynaNav 楼梯、分层地图与 LIO 观察器实验](experiments/dynanav-20261109.md)：2026-11-09 首段 live LIO/移动中行人闭环、完整 GT 路线、SDK 200 Hz 对照、多层精度边界及 LIVO smoke 更新保留、单组VIO评分、录包/两次SDK阻碍及前缀对比边界。
- [DynaNav配置](../configs/simulation/dynanav/README.md)与[native LIO fixtures](../tests/native/dynanav_lio/README.md)：模式/标定索引和无模拟器的C++构建测试。
- [依赖与构建](dependencies/README.md)：第三方源码、构建、模型和许可证边界。
- [RoboGauge 部署与历史证据](robogauge/README.md)：策略契约、CPU 验证、Gazebo 入口与 Isaac 历史实验限制。
- [全量可读性审查](review/full-readability/README.md)：代码与配置可读性审查记录。
- [2026-11-09 独立复核](verification/independent-review-2026-11-09.md)：本轮开始前的修复与验证结果。

模块级职责与入口见[源码模块索引](../src/embodied_navigation/README.md)。大型场景、地图、日志、模型和视频保存于外部项目数据目录；工程文档引用其证据，不将这些数据纳入 Git。
