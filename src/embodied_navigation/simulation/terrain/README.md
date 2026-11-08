# simulation/terrain

本目录提供受保护的 Actor 地形切换：只有实测 SLAM 到达指定 connector 时，才把 Actor 的高度射线从 lower12 切到 upper23；它不写导航状态或控制命令。

- [provider.py](provider.py)：`SwitchingTerrainProvider` 校验运行目录中的 `world.sdf`、两份 terrain manifest、`navigation_source_snapshots.json` 的冻结哈希；读取 `navigation_status.json` 的 `region_arrivals`、`navigation_request.json` 与 goal 定义哈希，要求到达 dwell 证据；再用 187 条 Actor 射线验证两套地形在共享 floor_2 上的命中完全一致（≤1e-9 m）才切换。失败会锁存初始 provider、把请求速度斜降到零，并继续学习策略的反馈。
- [mission46.py](mission46.py)：`MissionTerrainProvider` 在 `provider.py` 上增加 exploration/return/navigation 三次固定切换顺序与 `connector_mid` 完成条件。
- 装配在 [../robot/unitree/policy/worker.py](../../robot/unitree/policy/worker.py)：profile 带 `terrain_layer_switch` 或 `mission46_required` 时启用，并把 provider 交给原生 observation 的 `TerrainHeightMap.from_sdf` 替换。

## 限制

- 这是默认不启用的可选分支：`configs/control/settings.json` 中 `mission46_required` 为 false，普通默认启动不构造该 provider。
- 当前普通准备入口生成的 `runtime_context.json` 使用空 `references`，也没有准备 alternate manifest 与 `navigation_source_snapshots.json`。父类先校验 `world.sdf` 的冻结引用，再校验两套 source/run manifest 和 goal-normalizer snapshot；这些输入尚未由普通准备入口闭合。
- `mission46.py` 还写死引用 `tests/terrain_targets/{lower12,upper23}.json`，该目录不存在；当前 `configs/scenarios/terrain_targets/` 只有 `lower12.json`。只替换路径或打开开关不能恢复此分支，本次未验证可运行。
- 无专用 unittest；本次未启动仿真。本模块不把仿真位置重标为 SLAM 导航位姿；高度/射线用于现有仿真观测与校验，不是导航定位来源。
