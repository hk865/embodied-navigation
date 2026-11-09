# sensors：跨层数据契约

`sensors` 只定义感知、定位与控制之间交换的轻量浅冻结数据结构，不是 ROS 传感流适配层，也不含传感器驱动。

## 边界

- 只存数据，不做单位换算、校验、序列化或来源认证；`frozen=True` 只阻止字段重绑，NumPy 数组字段的内容仍可修改。
- 当前仓库源码中没有导入者（`src`、`tests`、`start.py` 均未引用），是预留的跨层契约，不要当作已接入的运行时接口。
- 真值边界不在本目录：来源/时间域合法性由 [common/state_contract.py](../common/state_contract.py) 的 typed provenance 负责，这里的结构本身不带 origin 字段。

## 目录结构

```text
src/embodied_navigation/sensors/
├── README.md  # 本文件
├── __init__.py  # 1 行，重导出三个 dataclass
└── frames.py  # 34 行，三个 frozen dataclass 定义
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `frames.py` 34 行 + `__init__.py` 1 行 | 全部是活代码：3 个 `@dataclass(frozen=True)` 类型定义，无函数、无兼容门面、无生成物。`__init__.py` 只有 `from .frames import SensorFrame, PoseEstimate, VelocityCommand` 一行重导出。 |
| 日志/输出 | 无 | 本模块不直接写日志、不写文件：两个文件里 `getLogger`、`print`、`open(`/`write_text` 命中均为 0（事实包 grep 证据为空）。 |
| 配置 | 无配置读取点 | 不 import `common.config`、不读 `os.environ`、不读 JSON；字段单位直接写在字段名里（秒、米、m/s、rad/s）。 |

## 模块内文件耦合

依赖方向单一：`__init__.py → frames.py`（纯重导出），无环、无第三方运行依赖（除 NumPy 作为字段类型）。全仓没有任何模块 import 本包，所以"被谁依赖"一栏为空——这是本模块最重要的耦合事实。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `__init__.py` | 1 | `frames.py` | 无 | 清楚（重导出） |
| `frames.py` | 34 | numpy（字段类型） | `__init__.py` | 清楚（纯数据定义） |

四类问题核查：没有 >800 行文件（本目录共 35 行）；没有重复实现；没有跨层直达（本模块不 import 任何项目内模块）；没有隐式契约（字段名即契约，且无消费者，暂不存在"靠位置/字符串对齐"的风险）。唯一的结构性事实是**零消费者**：三个类型目前没有被任何代码使用，所以既谈不上耦合良好，也谈不上耦合混乱。

## 可读性评估

| 文件 | 分级 | 证据 |
| --- | --- | --- |
| `frames.py` | 良好 | 34 行、3 个 dataclass、3/3 类 docstring；字段名自带单位（`time_s`/`body_xyz`/`forward_mps`/`yaw_rate_rps`）；无魔法数、无函数、无分支 |
| `__init__.py` | 可接受 | 1 行重导出，无模块 docstring；作为包入口不影响阅读 |

结构问题：无（文件只有 34 行）。文档问题：`__init__.py` 没有一行说明"这是预留契约"，读者只能从 README 得知；`frames.py` 的 `SensorFrame`/`PoseEstimate` 未说明点云/位姿的坐标系约定（传感器帧 vs 机体帧），目前仅靠字段名与 docstring 的 "in its sensor frame" 暗示。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `__init__.py` 加一行模块 docstring："预留的跨层数据契约，当前无消费者" | 防止有人把导入成功当成"接口已接线" | 低 | 纯文档 |
| P2 | 在 `frames.py` 的 docstring 里写清 `points_xyz`/`body_xyz`/`rotation` 的坐标系与形状约定 | 未来接入时最先被问到的就是坐标约定 | 低 | 纯文档 |
| P2 | 若将来接入，先决定来源/时间域字段（或明确要求调用方同时传 `state_contract` provenance）再改结构 | 只传结构体不传 provenance 会绕过 `NAVIGATION ⟹ SLAM_ESTIMATE` 校验 | 中（改动契约本身） | 接入前补调用点与 provenance 校验测试 |

本模块规模小、耦合清楚：**没有 P0/P1，只有 P2 文档项**。本轮只写建议、不改代码；若决定删除这组预留类型，应先确认无文档引用，再单独提交。

## 关键文件与接口（原"关键文件与接口"）

- `frames.py` 定义三个 frozen dataclass：
  - `SensorFrame(time_s, frame_id, points_xyz)`
  - `PoseEstimate(time_s, frame_id, body_xyz, rotation, velocity_xyz)`
  - `VelocityCommand(time_s, forward_mps, lateral_mps, yaw_rate_rps)`
- `__init__.py` 重导出这三个类型。字段名已表达单位（秒、米、m/s、rad/s）；只存数据，不做单位换算、校验或序列化。`frozen=True` 只阻止字段重绑，NumPy 数组字段的内容仍可修改。

## 边界与调用（原"边界与调用"）

- 当前仓库源码中没有导入者（`src`、`tests`、`start.py` 均未引用），是预留的跨层契约，不要当作已接入的运行时接口。
- 与 [perception/sensors/](../perception/sensors/README.md) 不是同一层：后者是 ROS 传感流转发、自车点过滤、健康门与反馈邮箱。
- 真值边界不在本目录：来源/时间域合法性由 [common/state_contract.py](../common/state_contract.py) 的 typed provenance 负责，这里的结构本身不带 origin 字段。

## 测试与限制

本目录无专用测试，也没有当前调用者。将来接入时应先明确来源与时间域，避免只靠结构体传位姿。
