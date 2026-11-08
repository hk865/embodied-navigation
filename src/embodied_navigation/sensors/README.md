# sensors：跨层数据契约

`sensors` 只定义感知、定位与控制之间交换的轻量浅冻结数据结构，不是 ROS 传感流适配层，也不含传感器驱动。

## 关键文件与接口

- `frames.py` 定义三个 frozen dataclass：
  - `SensorFrame(time_s, frame_id, points_xyz)`
  - `PoseEstimate(time_s, frame_id, body_xyz, rotation, velocity_xyz)`
  - `VelocityCommand(time_s, forward_mps, lateral_mps, yaw_rate_rps)`
- `__init__.py` 重导出这三个类型。字段名已表达单位（秒、米、m/s、rad/s）；只存数据，不做单位换算、校验或序列化。`frozen=True` 只阻止字段重绑，NumPy 数组字段的内容仍可修改。

## 边界与调用

- 当前仓库源码中没有导入者（`src`、`tests`、`start.py` 均未引用），是预留的跨层契约，不要当作已接入的运行时接口。
- 与 [perception/sensors/](../perception/sensors/README.md) 不是同一层：后者是 ROS 传感流转发、自车点过滤、健康门与反馈邮箱。
- 真值边界不在本目录：来源/时间域合法性由 [common/state_contract.py](../common/state_contract.py) 的 typed provenance 负责，这里的结构本身不带 origin 字段。

## 测试与限制

本目录无专用测试，也没有当前调用者。将来接入时应先明确来源与时间域，避免只靠结构体传位姿。
