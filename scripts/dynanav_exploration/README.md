# DynaNav 探索工具：项目内可执行入口

统一入口是 `scripts/dynanav_exploration.py`。代码只从当前 checkout 加载，不依赖 `Documents/Codex/.../task` 下的旧 Python 文件，也不调用个人安装目录里的 skill。

| 子命令 | 项目内实现 | 用途 |
| --- | --- | --- |
| `sop` | `.agents/skills/dynanav-explore/scripts/dynanav_sop.py` | 正式客户端的状态、快照、回执、journal 和显式操作；默认自动绑定当前 checkout 的 `--repo`。 |
| `collect` | 同目录 `record_current_lio_cloud.py` | 有限时、只订阅的 exact-stamp LIO collector；需要匹配的 ROS 环境和 domain。 |
| `view` | 同目录 `render_live_3d_views.py` | 实测点云局部/全域 XY、XZ、YZ、斜视 3D；不连接 ROS。 |
| `review` | `review_generation.py` | 同代输入审计、post-VIO 轨迹、回执前缀、实测点云和 height grids。 |
| `roi` | `review_roi.py` | 显式 XY ROI 的 YZ/斜视辅助检查。 |
| `multicamera` | `multicamera_snapshot.py` | 六向真实 RGB、共享 LIO 投影与源时间证据。 |
| `shadow-audit` | `audit_multicamera_shadow.py` | 冻结 native 快照的只读分支审计与 CI 诊断；不回写定位。 |

`review_generation.py` 与 `review_roi.py` 来自找回的原 generation/ROI 工具，作了最小可复用适配：同时支持旧快照 `measured_cloud.npz` / `level*_operator_map_*.json` 和当前 SOP 的 `measured_partial_cloud.npz` / `height_chart_*.json`；读取 journal 后关闭文件；检查快照/标签名；拒绝覆盖已有输出、把输出写回绑定输入或写入源码目录。采样、源时间、map-frame 和回执语义保留。两种 cloud 名同时存在时拒绝猜测。

原上层相对参照面、`774.7s` 下层阶段及早期 run 专用观察脚本继续按历史工具保存，不把这次实验阈值推广为任意建筑的楼层算法。

## 依赖与无动作帮助

CPU视图/快照依赖见 `requirements-dynanav-exploration.txt`，除已有 NumPy/SciPy/PyYAML 外需要 Matplotlib 和 Pillow。按自己的 Python 环境安装；本次迁移没有安装或升级包。`collect` 另需已构建的 ROS `rclpy`、sensor/nav messages，CPU依赖不能代替 ROS。

```sh
REPO=/path/to/embodied-navigation
TOOL="$REPO/scripts/dynanav_exploration.py"
python3 -B "$TOOL" --help
python3 -B "$TOOL" sop --help
python3 -B "$TOOL" view --help
python3 -B "$TOOL" review --help
python3 -B "$TOOL" roi --help
```

以上不会启动仿真、订阅或发目标。`collect --help` 在匹配 ROS Python 环境中查看。

## 当前 run：SOP、点云和查看

沿用 [项目 skill 的完整 SOP](../../.agents/skills/dynanav-explore/references/native-interface.md)。`RUN` 是明确的 generation root，脚本只附加一次 `capture`；`EVIDENCE` 和 `CLOUD` 是外部数据卷上本代新目录。

```sh
python3 -B "$TOOL" sop --run-dir "$RUN" --evidence-dir "$EVIDENCE" status
# 下面只订阅当前已运行的 ROS domain，不启动仿真：
ROS_DOMAIN_ID="$DOMAIN" python3 -B "$TOOL" collect \
  --run-id "$(basename "$RUN")" --output-dir "$CLOUD" \
  --budget-wall-s 300 --save-every-wall-s 5
python3 -B "$TOOL" sop --run-dir "$RUN" --evidence-dir "$EVIDENCE" \
  snapshot --tag s01 --cloud-dir "$CLOUD" --roi-span 20 --z-span 12
```

collector 是阻塞的有限采集任务；实时采集时用另一个持久会话执行 snapshot。`sop launch/goto/cancel/stop` 有实际操作效果，只用于当前明确的探索任务。统一入口不选择目标，不排队、不重发，不改 native 控制器。不要将历史 run 坐标当新目标。

独立离线看已经记录的点云：

```sh
python3 -B "$TOOL" view \
  --cloud "$EVIDENCE/s01/measured_partial_cloud.npz" \
  --observation "$EVIDENCE/s01/raw_native_observation.json" \
  --output-dir "$NEW_VIEWS" --prefix s01 \
  --expected-run-id "$(basename "$RUN")" \
  --observation-run-id "$(basename "$RUN")" --recorded-pose
```

`--recorded-pose` 标记历史快照，不改变原stamp或伪称当前新鲜。实际打开 RGB 与局部/全域 PNG 阅读，而不是只检查文件存在。

## 同代离线 review

在外部工作目录创建自己的 binding，示例键如下；所有路径填写真实同代输入，`output_root` 使用不同的新输出位置：

```json
{
  "run_id": "native_exploration_YYYYMMDDTHHMMSSZ",
  "run_root": "/data/experiments/native_exploration_YYYYMMDDTHHMMSSZ",
  "evidence_root": "/data/evidence/current-generation",
  "output_root": "/data/reviews/new-review",
  "allowed_input_roots": [
    "/data/experiments/native_exploration_YYYYMMDDTHHMMSSZ",
    "/data/evidence/current-generation"
  ],
  "cloud_stage": "LIO",
  "map_frame": "dynanav_lio_map",
  "pose_source": "livo_post_vio",
  "pose_child_frame": "go2_imu"
}
```

```sh
python3 -B "$TOOL" review --binding "$BINDING" --snapshot s01
# ROI输出同样要求新位置；另写一个 output_root 不同的 binding：
python3 -B "$TOOL" roi --binding "$ROI_BINDING" --snapshot s01 \
  --xy -4 4 -4 4 --label local
```

需要 snapshot 下的 observation、RGB及来源记录、cloud/summary/latest-pair、可用的 scene_view_metadata 和 height grids，以及 run 下的 control journal/receipts。原始 RGB 路径与 grid manifest 中的 source 仍须指向同代真实数据；迁移代码不会重写证据路径、复制大数据或凭 run label 自动对齐。`--cloud-dir` 可显式读取同代冻结 collector，仍须纳入 allowed roots。重跑使用新 `output_root`；不删除旧产物来绕过保护。

点数、投影面积、轨迹和 height-chart 数都不等于建筑覆盖率。地图空白不证明 free/unknown，回波不证明支持/净空，显示规则不成为新的 goto 门禁。

## 检查

```sh
PYTHONPATH=src python3 -B -m unittest discover -s tests/unit \
  -p 'test_dynanav_exploration*.py' -v
```

此测试组不连接 ROS、不依赖私有数据。真实记录的离线重放与结果见 [实际工具迁移报告](../../docs/experiments/dynanav-exploration-tools-20261111.md)。它们不代替在线仿真和导航验收。

真实 native 六路旁路、曝光边缘化、原生回放构建与数据路径见[实验报告](../../docs/experiments/dynanav-multicamera-native-shadow-20261112.md)。快照是 VIO 查询几何，不是可恢复 LIO 更新的完整地图；协方差为未校准模型输出。

五台 72° 间隔的真实采集、跟踪裁边与三/四台广视场候选比较见[相机数量对照](../../docs/experiments/dynanav-five-camera-comparison-20261112.md)。测试可显式设 `MULTICAMERA_OBSERVATION=1 MULTICAMERA_COUNT=5`；数量选项不会改变原约 82.4° 视场，也未切换默认定位输入。通用网格输出为 `view_grid.*`，六路保留兼容名称。


## 四向观察、录制桥与原生实验更新

`observe-multicamera --run-dir RUN --evidence-dir EVIDENCE --cloud-dir CLOUD --tag TAG` 将原SOP快照和选定数量的独立原生RGB归档一起冻结，并写出路间及位姿源时间偏差；它不发目标、不读真值。`multicamera-ros --capture RUN/capture --output NEW_BRIDGE --budget-wall-s BUDGET` 仅把本代已有1Hz PNG、32FC1深度、标定及LIO显示TF发布到审计话题，便于bag录制，不将额外图像插入定位。

真实SOL探索、分段MCAP与2×回放、单前/原生四路180s同包对照见[实验报告](../../docs/experiments/dynanav-multicamera-sol-exploration-20261112.md)。`FASTLIVO_MULTICAMERA_MODE=native_archive_marginal_ci_v1` 是默认关闭的已完成归档原生更新接口，不是实时四路10Hz验收；噪声/地图相关性未校准，当前不切换生产默认。报告保留失败和输入完整性限制。
