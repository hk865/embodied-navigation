# robot/unitree/policy/native：247D 冻结 Teacher 观测契约

盘点日期 2026-11-20。本目录保存一份冻结的 Teacher 观测契约（`contract.json`）与其 NumPy 实现：247 维观测、187 维 height scan 和显式 SDF 射线地形查询；它不训练、不加载 actor，当前只作诊断与适配参考。

## 模块定位与架构层级

- 架构层级：第 6 层「机器人执行」下的观测契约数据与数值实现，位于 `robot/unitree/policy/native/`。
- 输入：`state64`（`state[1:4]` base 位置、`state[4:8]` wxyz 姿态、`state[8:11]` body COM 线速度、`state[11:14]` 角速度、`state[14:38]` 12 关节 q/qd、`state[38:50]` 力矩）、`command3`、`last_action12`；地形输入是 SDF 场景文件路径（`TerrainHeightMap.from_sdf`）或已构造的 shape 列表。
- 输出：`float32[247]` 观测 + 诊断字典（height scan 原始值、命中模型名、有效/无效计数、clip 比例、overhead 计数等）；`contract.json` 同时作为关节名称与默认姿态的数据源被仿真侧读取。
- 上游 / 下游：`quaternion_rotation()` 被 [simulation/dynanav/capture/control.py](../../../../simulation/dynanav/capture/control.py)、[physics.py](../../../../simulation/dynanav/capture/physics.py)、[ros_interface.py](../../../../simulation/dynanav/capture/ros_interface.py) 与 [robogauge/contract.py](../../robogauge/contract.py) 复用；`contract.json` 被 [simulation/dynanav/capture/scene.py](../../../../simulation/dynanav/capture/scene.py) 读取。反向无依赖。
- 边界：模块只 import `numpy`、stdlib 和 `common/geometry.py`，不 import Isaac、ROS、torch。`build_observation()` 与 `TerrainHeightMap` 在本检出**没有生产调用方**（`git grep "build_observation|TerrainHeightMap" -- src` 只命中本文件），因此它不是导航或控制的输入；SDF 几何只允许作为 locomotion 观测，绝不能当导航地图或定位真值。

## 当前文件夹结构

```text
src/embodied_navigation/robot/unitree/policy/native/
├── README.md        # 本文件（新增）
├── __init__.py      # 0 行占位
├── contract.json    # 414 行：冻结契约数据（观测/关节/actuator/height scan/request64）
└── observation.py   # 312 行：契约加载、四元数、SDF 射线地形与 247D 构建
```

## 逐文件职责

| 文件 | 核心职责 | 关键接口/入口 |
| --- | --- | --- |
| `__init__.py` | 0 行占位文件，仅使目录成为包；无 docstring、无导出 | 无 |
| `README.md` | 本文件：契约语义、消费关系与验证入口 | 索引页 |
| `contract.json` | 冻结契约数据（`schema_version=1`）：teacher checkpoint sha256、actor 结构 `247→512→256→128→12` + ELU、12 关节名（`joint_names` 与 `gazebo_joint_names` 两套）与默认姿态/速度、`actions.scale=0.25`、DCMotor 参数与速度相关力矩包络、9 段观测的 clip/scale/训练噪声、187 维 height scan（0.1 m 分辨率、`1.6×1.0 m`、11×17）、`request64` 字段表与 IsaacLab 源码引用 | `observation.py` 在 import 时加载；`capture/scene.py` 读 `joint_names`/`default_joint_positions` |
| `observation.py` | 模块常量 `CONTRACT`/`DEFAULT_Q`/`JOINT_NAMES`；`quaternion_rotation(quat_wxyz)` 返回 world-from-body 旋转；`_Box`/`_Plane` 精确垂直射线求交与 `TerrainHeightMap.from_sdf()`（默认排除 `go2`/`teacher_go2`/`moving_obstacle`）；`SCAN_GRID_XY`（17×11 网格）；`build_observation()` 产出 247D + 诊断；`from_state` 为同函数别名 | `quaternion_rotation`、`build_observation(state, command, last_action, terrain, noisy=False, rng=None, base_com_offset=None)`、`TerrainHeightMap.raycast()/height()` |

## 契约要点（易错处）

- **height scan 公式**：`base_z − hit_z − 0.5` 后 clip 到 `[-1, 1]`；射线从 `base_z + 20 m` 起向下打，`+20 m` offset 只抬高射线起点，不移动 sensor 参考点，所以平地读数不是常数 1 或 5。
- **无命中不补零**：射线未命中时 raw height 为 `-inf`、诊断里以 JSON `null` 保留证据，clip 后为 `-1`；无效计数与原始值必须保留，禁止 zero imputation。
- **多楼层限制**：从 base 上方 20 m 发出的射线会命中上层楼板，这与训练时单一地形网格不同；诊断字段 `height_scan_overhead_count` 专门报告这类命中。
- **线速度参考点**：`state[8:11]` 必须是 body COM 速度；若来源给的是 link-origin 速度，必须传入真实的 `base_com_offset`，用 `omega × offset` 修正，不能套用训练标称偏移。
- **噪声**：`noisy=True` 时必须提供显式 seeded RNG，否则抛错；训练噪声先加后 clip、再 scale，顺序写在 `observations.processing_order`。

## 验证与测试入口 / 配置注意

- [tests/regression/test_robogauge_contract.py](../../../../../../tests/regression/test_robogauge_contract.py) 通过 `CONTRACT` 断言 `NATIVE_NAMES` 与 `default_joint_positions` 一致；[tests/unit/test_truth_deskew_and_mode.py](../../../../../../tests/unit/test_truth_deskew_and_mode.py) 用 `quaternion_rotation` 交叉校验四元数约定。
- `build_observation()`/`TerrainHeightMap` 在当前测试集中没有专用用例（`git grep -l "build_observation" tests` 无命中），改动前应先补 CPU fixture；不要用仿真实跑替代契约测试。
- 命令：`PYTHONPATH=src python3 -B -m unittest discover -s tests/unit`、`... -s tests/regression`。
- 契约引用的 checkpoint（`artifact:unitree_teacher_model_1000.pt`）、env/agent YAML 哈希与 IsaacLab commit 都是外部来源标识，仓库不包含这些文件，也没有下载/训练流程。
