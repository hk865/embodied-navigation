# configs/scenarios：场景与地形 base 资源

本目录保存场景配置与地形目标等base资源。普通入口先读[configs/launch/navigation.json](../launch/navigation.json)列出的模块配置，再由生成器写本次参数到`runs/`；不要把base文件、旧profile或字段旁注当作最终有效值。

## 目录结构

```text
configs/scenarios/
├── building/              # 子目录：SCAN 参数 scan.json（30 行）与学生 profile（45 行），无独立 README
├── terrain_targets/       # 子目录：地形目标数据 lower12.json（14 行，当前无代码读取）
└── README.md              # 本文件：配置层次、当前覆盖与调试边界
```

## 代码、日志与配置

本目录是**配置源**，不是运行入口；它本身不写日志、不产生 run 目录。

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 配置（本目录） | [`building/scan.json`](building/scan.json) | `scan_planner_node.ros__parameters`：`fsm.navi_mode`、`planning_horizon=0.65`、`grid_map.frame_id=camera_init` 等。消费点有两处：[`simulation/gazebo/robogauge/world.py`](../../src/embodied_navigation/simulation/gazebo/robogauge/world.py) 直接 `json.loads`，[`launch/building.py`](../../src/embodied_navigation/launch/building.py) 读取字节并写入 run 目录。 |
| 配置（本目录） | [`building/student_profile.json`](building/student_profile.json) | `actor_kind`/`controller_kind`、`max_speed_mps`/横移/偏航界、`cascade`、`navigation_ground_truth_used` 等 profile 字段。消费点：[`launch/building.py`](../../src/embodied_navigation/launch/building.py) 在启动前 `json.loads` 并作为 Building 的 profile。 |
| 配置（本目录，无读取点） | [`terrain_targets/lower12.json`](terrain_targets/lower12.json) | 地形目标数据（`include_models`、`scope`、`training_target` 等）。**当前无代码读取**：`simulation/terrain/mission46.py` 的 provider 只在 `mission46_required`/`terrain_layer_switch` 启用时构造，且引用的是 `tests/terrain_targets/` 下另一个（本检出不存在）的路径。 |
| 代码（读取点） | 不在本目录 | 见上：`launch/building.py`、`simulation/gazebo/robogauge/world.py`，以及 [tests/unit/test_building_entry_closure.py](../../tests/unit/test_building_entry_closure.py) 对这两个配置存在性的断言。 |
| 日志/输出 | `runs/<run_id>/`（外部数据卷） | 本目录不写文件。`launch/prepare.py` 把本次参数派生到 run 目录（`navigation_profile.json`、`navigation_scenario.json`、`navigation_fastlivo.yaml`、`navigation_camera.yaml`、`terrain_target_manifest.json`、`known_scene_reference.json`、`v34_scan_overrides.yaml`、`global_route.json`、`runtime_context.json` 与 `world.sdf`）；本目录的 base 值不等于这些生成值。 |
| 其它配置（本文引用的全局项） | `configs/launch/`、`configs/localization_mapping/` 等 | 见下方“配置层次”表；读取顺序与 pin 的权威说明在 [configs/README.md](../README.md)。 |

## 模块内文件耦合

- 依赖方向：本目录文件之间**没有 include 或互相引用**；`building/scan.json` 与 `building/student_profile.json` 由不同代码路径各自读取（SCAN 参数 vs Student profile），`terrain_targets/lower12.json` 与任何代码都不相连。
- 与代码的耦合是**字段名契约**：`scan.json` 的 ROS 参数键必须与 C++ `scan_planner_node` 的参数声明一致；`student_profile.json` 的键必须被 `launch/building.py` 的 profile 解析逻辑识别。改键名/类型不会在本目录报错，只会让读取方失败或静默使用默认值。
- 反向依赖不存在：没有代码回写本目录；`prepare.py` 只读取/派生，写入目标是 run 目录。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `building/scan.json` | 30 | C++ SCAN 参数契约 | `world.py`、`launch/building.py` | 清楚：单一职责 |
| `building/student_profile.json` | 45 | Building profile 契约 | `launch/building.py` | 清楚：字段多但分组明确 |
| `terrain_targets/lower12.json` | 14 | 无 | 无代码读取点 | 可疑（孤儿数据）：与 `mission46.py` 的 `tests/terrain_targets/` 引用不一致 |
| `README.md` | 本文件 | 上述文件 + 全局配置 | 人 | 清楚：索引与覆盖说明 |

结论：耦合**清楚但存在一个孤儿文件**。`lower12.json` 既无读取点、又与 provider 期望路径不一致，属于“保留数据”，应在文档中明确标注（本文已标注），不需要代码改动。

## 可读性评估

- `README.md`（原 32 行）：**可接受偏低**。配置层次表、当前覆盖清单、使用与调试三段都在，事实已核实；问题是行内代码与中文之间没有空格（如“普通入口先读[configs/launch/navigation.json]列出的”），长句密度高（“当前重要覆盖”单段 6 条），新手需要读两遍。本轮没有删改任何数值/命令，只补结构与分类。
- `building/scan.json` / `building/student_profile.json`：**可接受**。JSON 无注释，但字段名可读、层级浅；`scan.json` 的参数名直接对应 ROS 参数，`student_profile.json` 的键与 `launch/building.py` 用法一致。
- `terrain_targets/lower12.json`：**需改进（语义）**。14 行数据本身可读，但“无读取点 + 与 mission46 引用路径不一致”这一关键事实只在 README 里，文件本身没有任何注释/`_comment` 字段。
- 没有发现魔法数滥用：数值（0.65 m horizon、speed 界等）都在“当前重要覆盖”里逐条解释。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 给 `terrain_targets/lower12.json` 加 `_comment`/`archived` 字段，或在文件名加 `archived_` 前缀，并在 README 用同一措辞标注 | 现在“有数据、无读取点、路径还不一致”只能靠读 README 发现 | 低：加注释字段不影响无读取点现状 | `grep -rn "lower12.json" src scripts` 仍为空；`mission46.py` 引用不变 |
| P2 | 把“当前重要覆盖”拆成表（项 / 数值 / 覆盖来源），保留原有全部数值 | 6 条密集 bullet 难扫读，且“base YAML 的 300/false 不代表日常值”这类关键边界容易被漏 | 无（纯文档） | 逐项对照源码/配置确认数值未改 |
| P2 | 在本文加“配置层次 → 生成物”映射（哪个 base 经哪个生成器写进 run 目录的哪个文件） | 现在只列生成器和生成物名，读者要自己拼关系 | 无 | 对照 `launch/prepare.py` 的 `write(...)` 调用 |
| P3 | 统一行内代码与中文的空格/标点风格 | 可读性小问题 | 无 | 人工浏览 |

没有 P0/P1：本目录是配置源，风险由读取方（launch/prepare/SCAN node）承担；现有 preflight/校验会暴露字段错误，且本轮不改任何配置值或代码。

## 配置层次

| 文件/生成器 | 职责 |
| --- | --- |
| [configs/launch/navigation.json](../launch/navigation.json) | 普通入口的六个模块配置清单；由[common.config.read](../../src/embodied_navigation/common/config.py)合并，提供默认mission、速度档、控制/策略、特性、runtime路径、DDS环境 |
| [configs/localization_mapping/fastlivo.yaml](../localization_mapping/fastlivo.yaml)、[camera.yaml](../localization_mapping/camera.yaml) | 前端base参数与相机内参 |
| [common/communication/dds_transport.xml](../../src/embodied_navigation/common/communication/dds_transport.xml) | loopback UDP+SHM拓扑，64MiB SHM、512 port queue、健康检查1000ms |
| `terrain_targets/lower12.json` | 保留的地形目标数据；当前无代码读取。`simulation/terrain/mission46.py` 的 provider 只在 `mission46_required`/`terrain_layer_switch` 启用时构造，且引用的是 `tests/terrain_targets/` 下的另一个（本检出不存在的）路径 |
| [embodied_navigation.launch.prepare.prepare](../../src/embodied_navigation/launch/prepare.py) | 生成world、外参/SLAM覆盖、解析定位地图、SCAN参数和路线 |
| [embodied_navigation.perception.sensors.sampling.override_sensors](../../src/embodied_navigation/perception/sensors/sampling.py) | 当前LiDAR/RGB采样覆盖 |
| [embodied_navigation.planning.scan_runtime_parameters.parameters](../../src/embodied_navigation/planning/scan_runtime_parameters.py) | 当前原生planner覆盖 |

`configs/scenarios/building/scan.json` 与 `student_profile.json` 是 Building 路径的两份显式输入，不经过上面六个模块配置的合并链；它们的消费点见“代码、日志与配置”表。

## 当前重要覆盖

- LiDAR/RGB名义30Hz、64×480；IMU由[simulation/world/prepare.py](../../src/embodied_navigation/simulation/world/prepare.py)设200Hz。
- IMU初始化600样本、≥2.9s静止；在线bias和gravity开启。base YAML的300/false不代表日常值。
- VIO `img_point_cov=1000`覆盖base 1e8；曝光、inverse composition、raycast关闭；map sliding关闭。
- SCAN .08m voxel、10×10×5m、.65m horizon、vmax=.5m/s、amax=.15m/s²；tolerance仍在代码中。
- 主控制使用`cascade`，不是顶层legacy `pid`；Student当前有效命令界[.6,.2,.3]，唯一slew[.6,.6,.8]/s。
- 停车首次T0后5s，10Hz因果pose，XY/yaw漂移.05m/.1rad，源龄/状态覆盖300ms。

## 使用与调试

先在根运行`python3 -B start.py --check`做有限文件检查；`--prepare-only`会写生成物，可人工核对`navigation_profile.json`、`navigation_fastlivo.yaml`、`v34_scan_overrides.yaml`和world。两条均不等于完整运行通过；这里列命令没有在本次文档任务实际执行。

上述模块配置保留许多历史字段/兼容结构及debug开关；是否被当前daily路径消费以源码为准。day环境还显式关闭部分探针，即便profile写1也不一定开启。修改需跟进读取/覆盖位置，别恢复旧hash/版本批准链。VIO线程的环境意图1与编译宏4存在差异，配置不能单独认证载入二进制团队。

默认mission是相对初始化的F3目标；goal Z不是地面真值。frame/安装/路线变更必须同步核对pose/cloud/Path约定，不能只改一个Z。说明见[结构文档](../../docs/architecture.md)。
