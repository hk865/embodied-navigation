# View a recorded DynaNav SLAM map

本目录只放 RViz 可视化配置与查看历史 DynaNav 估计地图的操作说明；不含代码、不产生数据，也不参与任何运行入口。

## 目录结构

```text
configs/visualization/
├── README.md            # 本文件：查看命令、QoS/坐标系约定与边界
└── dynanav_map.rviz     # RViz2 显示配置（69 行，PointCloud2 + Grid + Orbit 视角）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 配置 | [`dynanav_map.rviz`](dynanav_map.rviz) | RViz2 的 `rviz_common` 配置：`Fixed Frame=dynanav_lio_map`、PointCloud2 显示 `/dynanav/review/static_map`（Reliable + Transient Local、Keep Last、depth 1）、按 Z 轴着色、0.03 m Flat Squares，以及 Orbit 视角参数。消费者只有手动 `rviz2 -d`。 |
| 代码 | 无 | 本目录没有 Python/C++ 代码，也没有 import 方。实际发布脚本是 [`scripts/view_dynanav_map.py`](../../scripts/view_dynanav_map.py)，它不读 `.rviz` 文件；两者的唯一契约是 topic 名、QoS 与 frame 名在脚本 `TOPIC` 常量、本 README 和 `.rviz` 里三处保持一致。 |
| 日志/输出 | 无文件写入 | `view_dynanav_map.py` 只向 `/dynanav/review/static_map` 发布一条 PointCloud2 并保持进程存活，不写 run 目录、不写日志文件；输出仅为 stdout 与 ROS 话题。显示证据（截图、`review-receipt.json`）由外部 experiments 目录保存，不随仓库分发。 |

本目录的“配置读取点”就是人工命令：没有代码在启动时自动加载 `dynanav_map.rviz`，不要把它当成自动生效的默认可视化。

## 模块内文件耦合

- 依赖方向：本目录两个文件互不 import，也不被代码 import。`.rviz` 由 RViz2 解析，README 由人阅读。
- 唯一耦合是**隐式契约**：`.rviz` 里的 topic `/dynanav/review/static_map`、QoS（Reliable / Transient Local / Keep Last / depth 1）和固定坐标系必须与 `scripts/view_dynanav_map.py` 的 `TOPIC` 常量及 `map_frame` 发布逻辑一致；改名时三处（脚本常量、`.rviz`、本文）要同步，目前没有任何自动校验。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README.md` | 本文件 | 无 | 人 | 清楚：只描述人工查看流程 |
| `dynanav_map.rviz` | 69 | RViz2 插件 | 手动 `rviz2 -d` | 偏脆弱：topic/QoS 靠字符串约定，无代码消费者 |

结论：耦合**清楚但只是文档-配置级耦合**，没有代码依赖；风险点是 topic/frame 改名后 `.rviz` 不会报错，只会显示空场景。

## 可读性评估

- `dynanav_map.rviz`：**可接受**。69 行、无注释；`Class`/`Topic`/`QoS` 字段自解释，但读者无法从文件本身知道它对应哪次 run、哪个地图文件（同一 topic 名可被任何 NPZ 复用）。
- `README.md`：**良好**。命令、域隔离（`ROS_DOMAIN_ID=119`）、QoS 语义、frame 来源、`--frame` 限制、TF 身份轴线的含义都写清了，且区分了“显示证据”与“导航/估计证据”。缺口是上面说的三处契约没有表格化。

未发现结构问题；可读性缺口集中在“配置与脚本/文档的契约不可见”这一项。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 在本文加一张“契约表”：topic 名、QoS、`map_frame` 默认值、对应脚本常量位置 | 现在三处字符串靠人眼同步，改名后 RViz 静默空白 | 无（纯文档） | 人工比对 `scripts/view_dynanav_map.py` 的 `TOPIC` 与 `.rviz` 的 `Topic.Value` |
| P2 | 给 `dynanav_map.rviz` 顶部加注释说明“人工查看配置，无代码读取者，topic/frame 需与 view_dynanav_map.py 对齐” | RViz 配置常被误当成运行默认值 | 无（注释不影响 RViz 解析） | 用 `rviz2 -d` 打开一次确认仍能加载 |
| P3 | 若以后有多份地图配置，改为按地图命名（如 `dynanav_map_<run>.rviz`）并在 README 写选择规则 | 单一 `.rviz` 同时服务多个历史 run，容易看错地图 | 低；需同步更新命令 | 人工按 README 命令跑通一份 |

没有 P0/P1：本目录无代码、无写操作，最坏后果是可视化看不到点云，不涉及导航正确性。

## 查看历史地图

From the repository root, validate the estimated map without starting ROS:

```bash
MAP='/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-dual-stairs/post-vio-static-map/static_map.npz'
python3 scripts/view_dynanav_map.py --map "$MAP" --check
```

Keep the publisher running in one terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=119  # Use the same isolated review domain in both terminals.
python3 scripts/view_dynanav_map.py --map "$MAP"
```

Open RViz in another terminal from the same repository root:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=119
rviz2 -d configs/visualization/dynanav_map.rviz
```

The publisher sends one PointCloud2 sample on `/dynanav/review/static_map` with reliable, transient-local, keep-last-depth-1 QoS. It stays alive so RViz can join later. Stop it with Ctrl-C, or supply a finite positive `--duration` in wall seconds. The viewer has no controller, TF publisher, simulator, arbitrary output-topic option, or GT pose reader. RViz includes viewing tools only.

`map_frame` comes directly from the NPZ. Use `--frame` only for an archive that has no frame metadata; an existing frame cannot be overridden. The static aggregate uses header stamp zero because the file does not supply one acquisition timestamp. XYZ values are encoded as float32 without filtering or downsampling; values that would become nonfinite during conversion are rejected.

This example contains 91,449 points in `dynanav_lio_map`. Its measured bounds are `[-20.855167337, -23.484348744, -0.180133181]` to `[10.089567833, 3.554424299, 6.216008119]` metres. The RViz Orbit view uses their centre `[-5.382799752, -9.964962222, 3.017937469]`, distance 52 m, Z-axis colouring and 0.03 m flat squares. For another map, set RViz's Fixed Frame to the frame reported by `--check`, then adjust the view to its reported bounds.

The viewer reports metadata actually present in the NPZ, such as `pose_source`, `mask_source` and `semantic_oracle`. This file has no separate `provenance` array. Displaying it does not establish complete spatial coverage or turn semantic-oracle labels into learned perception.

The later actual closed-loop run has a separate map containing **92,846 points**, also in `dynanav_lio_map`. To inspect that run, replace the historical `MAP` above with:

```bash
MAP='/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/post-vio-map/static_map.npz'
python3 scripts/view_dynanav_map.py --map "$MAP" --check
```

Then use the same isolated publisher/RViz commands. Its actual screenshot is [rviz-closed-loop-map.png](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/rviz-closed-loop-map.png), with [rviz-review-receipt.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/rviz-review-receipt.json) recording the map hash, display scope and visual check. The earlier 91,449-point example and screenshot remain historical artifacts from a different run. This newer map uses estimated poses, retains `sim_oracle` semantic masks, and does not certify complete scene coverage; the screenshot itself is display evidence, separate from the run's [closed-loop audit](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/independent-review.json).

If RViz reports that its fixed frame is absent from the TF registry, the map can still render because its header and fixed frame are identical. Optionally register an identity **presentation axis** in the same isolated review domain, from a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=119
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id dynanav_lio_map --child-frame-id dynanav_review_axes
```

This only registers `dynanav_lio_map` and a child display axis for RViz. It is a mathematical identity for presentation, not a robot pose, localization transform, GT input, navigation result, or estimator evidence. Keep the RViz fixed frame as the map's actual `dynanav_lio_map`. The Python viewer itself continues to publish no TF or control messages. Stop only the review processes you started.

On this host a new review RViz window can use `DISPLAY=:0`, `XAUTHORITY=/run/user/1000/gdm/Xauthority`, and `XDG_RUNTIME_DIR=/run/user/1000`; no Xvfb is required. A validated review screenshot is at `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-rviz-review-20261109T053029Z/rviz-map-registered-frame.png`. Its `review-receipt.json` records the temporary display-axis scope. That review used domain109; use your own matching isolated domain in the commands above. Restart the map publisher to repopulate a newly opened RViz after its prior finite lifetime has ended.

## 限制

- 显示成功不等于空间覆盖完整，也不等于语义标签是 learned 感知；oracle 掩码仍只是 oracle 掩码。
- 同一 `/dynanav/review/static_map` topic 可被不同 run 的发布者复用，看图前先确认 `MAP` 与 `ROS_DOMAIN_ID`，否则可能把历史地图当成当前结果。
- 本文命令在本轮文档任务中未重新执行；已验证事实来自原 README 与脚本源码。
