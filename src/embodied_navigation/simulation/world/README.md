# simulation/world

本目录负责从仓库资产生成一次运行所需的仿真世界与传感器契约，不启动 Gazebo，也不是运行时节点。

## 目录结构

```text
src/embodied_navigation/simulation/world/
├── README.md       # 本文件
├── __init__.py     # 0 行（空文件，仅包标记）
└── prepare.py      # 422 行，世界/传感器契约/桥配置/只读 frame 树生成器（独立脚本，非 import 入口）
```

1 个实现文件；无子目录、无运行时节点。它只被当成脚本以子进程调用。

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `prepare.py` | 活代码，独立脚本（`main()` + `sha`/`rpy_quaternion`/`xml` 三个纯函数，外加内部 `bridged` 闭包）。以 `assets/world/go2_converted.sdf` 取模型、`assets/world/three_floors.sdf` 取世界，输出 5 个文件（见下）。 |
| 日志/输出 | `<output>/world.sdf` | 生成的隔离世界：物理步 `.005`、`real_time_factor` 可配、机器人 pose 被重写、原 CHAMP/ros2_control 等 plugin 全部移除只留 `libteacher_actuator.so` 插件元素（写入 `kp=25`/`kd=.5`/`effort_limit=23.5`/`velocity_limit=30`）；`--sensors` 时另加 `JointStatePublisher`（话题 `/demo/teacher/physical_joint_states`）。 |
| 日志/输出 | `<output>/sensor_contract.json` | 传感器契约（schema_version=1）：`joint_states` 与每类 IMU/LiDAR/camera 的 `body_xyz`、`body_rpy`、`rate_hz`、`frame`、`gz_topic`/`ros_topic`（IMU 固定 `/livox/imu`），camera 还带 `camera_info_topic`；`scope` 字段明写 "read-only sensor diagnostics; … no SLAM or navigation started"。供 FAST-LIVO2 外参、点云过滤和桥配置复用。 |
| 日志/输出 | `<output>/sensor_bridge.yaml` | 实际内容是 JSON（JSON 也是合法 YAML），逐条列出 GZ→ROS 的 topic、类型和 QoS。 |
| 日志/输出 | `<output>/frames.urdf` | 只读 frame/关节树：删掉 `ros2_control`、`gazebo`、`transmission` 以免与执行器插件争抢控制；关节 dynamics 的 damping/friction/spring 归零；碰撞摩擦统一写 `.7`，注释说明原始 URDF 是 `.2`，属显式受控基线。 |
| 日志/输出 | `<output>/asset_manifest.json` | 生成物清单：`source_hashes`、`world_sha256`、`sensor_contract_sha256`、`sensor_bridge_sha256`、`joint_mapping`/`joint_names`/`default_q`/`hard_lower`/`hard_upper`、`model_mass_kg`、`base_inertial_pose`、`friction_baseline`、`disabled`（CHAMP/body_stabilizer/ros2_control/JointTrajectoryController）、`exclusive_writer`、`decimation`/`policy_hz` 等。 |
| 日志/输出 | stdout | 唯一"日志"是 `print(json.dumps(meta))`，与 `asset_manifest.json` 同一份字典。唯一仓库内调用方 `launch/prepare.py` 用 `stdout=subprocess.DEVNULL` 丢弃它，所以这份 stdout 不是证据，落盘的 `asset_manifest.json` 才是。 |
| 配置 | 无配置读取点 | 全部行为来自 argparse，不 import `common.config`、不读 `configs/`、无环境变量。参数与默认值：`--output`（默认 `<repo>/simulation/generated`，真实调用方总是显式传 run 目录）、`--terrain`（`flat/ramp_up/ramp_down/step05/step10/step05_continue/step10_continue`）、`--sensors`、`--camera-only`、`--overview-pose`（6 个 float）、`--overview-fov`（0…π rad）、`--camera-rate`（1…20 Hz，默认 2）、`--real-time-factor`（0.1…1，默认 1）、`--render-engine`（ogre/ogre2）。取值由调用方从 profile 传入。 |

调用方是 [../../launch/prepare.py](../../launch/prepare.py) 的 `prepare()`：它用 `--sensors --camera-rate 10 --render-engine ogre2 --real-time-factor 1.0 --overview-fov math.radians(80)` 调本脚本，地形取自 `profile["expected_asset"]["terrain"]`、overview 位姿取自 `profile["overview_pose"]`；随后在 Python 里改写机器人 pose 与执行器 `.so` 绝对路径。世界生成参数里只有 terrain 与 overview_pose 来自 profile，其余 CLI 参数写死在 `launch/prepare.py` 内。

## 模块内文件耦合

- 依赖方向：`prepare.py → embodied_navigation.project`（只取 `project_root`）；无内部 import、无环、无共享状态。所有中间结构都是 `main()` 内的局部变量，生成器与消费者之间靠文件名+JSON 键耦合，不靠 Python API。
- 与调用方是**子进程 + 文件**耦合：`launch/prepare.py` 以 `subprocess.run(check=True)` 调它，之后再用 ElementTree 重写 `world.sdf` 的 pose 与插件 filename。因此 `world.sdf` 的元素结构（`world/model[@name='go2']/pose`、`plugin[@name='TeacherActuator']`）是隐式契约。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `prepare.py` | 422 | `project`；`assets/world/*.sdf`、`assets/meshes/*` | `launch/prepare.py`（子进程）；无 Python import 方 | 偏重：`main()` 单函数 380 行承担 5 类产物 |
| `__init__.py` | 0 | 无 | 无 | 清楚：空包标记 |
| `README.md` | 本文件 | — | 人 | — |

四类问题：**上帝文件无**（422 < 800，但 `main()` 380 行已是"事实上的上帝函数"）；重复实现无；跨层直达无（不读配置、不 import 上层）；**隐式契约有**：输出文件名与 JSON 键被 `launch/prepare.py`、`robot/unitree/policy/native/worker.py`（读 `sensor_bridge.yaml`/`world.sdf`）、RoboGauge 世界模板分别消费，改键名不会有编译期错误。

## 可读性评估

- `prepare.py`：**需改进**。证据：5 个函数 **0 个 docstring**（模块 docstring 只有一句 "Generate isolated Go2 assets. Never edit the camera or CHAMP assets."）；`main()` 380 行；15 行注释里有几条很关键（"The only torque writer is the new plugin…"、"The original URDF contact friction was .2… never conceal this controlled change"、"Flat tests are away from upper-deck overhead…"），说明作者知道哪些改动需要解释，但没有把这些解释提升到函数边界；魔法值密集（`23.5`/`30`/`.005`/`.7`、以及 `x, y, z, yaw = 6, -0.7, 0.40, 0` 这类逐地形 spawn），靠行内注释而不是常量名；`--terrain` 的 7 个取值只影响 spawn 与是否切楼梯资产。
- 可读性并非"结构混乱"：逻辑是线性的"读源→清洗→改写→写产物"，只是缺少分段函数与 docstring。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 给 `prepare.py` 补一个离线单测：用仓库自带 `assets/world/*.sdf` 跑 `--output <tmp> --terrain flat --sensors`，断言 5 个产物存在、`asset_manifest.json` 的键集合与 `world_sha256` 自洽 | 本次核对实测该脚本 **0.04 s 跑完**、不需要 Isaac/ROS/GPU，是仓库里少见的"可离线测试的生成器"，却完全没有测试；没有它，下面任何拆分都无法验证 | 低：只新增测试文件，不改实现 | 先跑现状拿基线（5 个文件的 sha256），测试里对固定输入断言文件集合与 manifest 键 |
| P1 | 把 `main()` 按产物拆成 4 个函数（模型清洗/世界装配/sensor_contract+bridge/frames.urdf+manifest），`main()` 只做参数校验与编排 | 380 行单函数里混了 5 类产物和 3 种输入源，定位"哪个旗标改了哪个文件"要靠通读 | 中：产物字节必须保持不变（manifest 里有 `world_sha256`） | 拆分前后各生成一次到两个 tmpdir，`diff -r` 5 个文件逐字节一致；再跑新增单测 |
| P2 | 删除或注明 `--output` 默认值 `<repo>/simulation/generated` 的历史来源 | 真实调用方总是显式传 run 目录，默认值会误导人以为有固定产物目录 | 无（文档） | 人工复核；`grep -rn "simulation/generated"` 确认无消费者 |
| P2 | 在 README 或 `--help` 里说明 `sensor_bridge.yaml` 是 JSON、stdout 会被调用方丢弃 | 两处都是"看起来像 A 实际是 B" | 无（文档） | 人工复核 |

没有 P0：生成器输出被下游当输入，但本次实测脚本可正常完成；P1 是"可测性"与"单函数过长"，不是正确性缺陷。本轮只写建议、不改代码；P1 的第二条必须等第一条的单测落地后再做。

## 测试与限制

- 本模块无专用 unittest；`prepare.py` 只在 `start.py --prepare-only` 等真实准备路径执行。
- 本次核对只做了离线生成验证：`PYTHONPATH=src python3 -B src/embodied_navigation/simulation/world/prepare.py --output /tmp/… --terrain flat --sensors --camera-rate 10 --render-engine ogre2` 退出 0，产出 `world.sdf`/`sensor_contract.json`/`sensor_bridge.yaml`/`frames.urdf`/`asset_manifest.json` 5 个文件；没有启动 Gazebo。
- 生成的 SDF 与契约是后续仿真输入，不代表仿真已经验证可运行。
