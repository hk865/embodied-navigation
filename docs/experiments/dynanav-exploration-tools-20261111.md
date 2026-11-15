# 2026-11-11 实际探索工具找回与项目索引

核对日期：2026-11-11。来源是用户指出的清理后今日工作目录 `/home/hyh001/Documents/Codex/2026-11-11/task`，不是根据描述重新造工具。本次未启动仿真、发目标、修改控制器、commit 或 push。

## 1. 找到了哪一轮

原探索线程名为 **探索大厅与地下室**，ID `6af42200-32d4-8191-adc3-26fc5f6b0005`。其最后一个已完成 turn `01a1ca4d-f06c-7626-b083-564ae2ccfb72` 的执行记录直接保存了 `explore_111150_read.py` 状态/快照、实际 `dynanav_sop.py goto`，以及 RGB、map、局部/全域 3D 图的查看。对应最新 run 为 `native_exploration_20261111T111150Z`，domain232。

[实际调用索引](dynanav-exploration-tools-20261111.execution.json)只收录这条原探索线程的命令和图像路径，不包括推理或聊天全文。命令记录证明调用和退出状态，不证明每个航点均完整读取所有要求的证据，也不把当前文件 SHA 反推为当时每次调用的版本。读取接口未给出模型 SKU，故这里不独立断言每个文件都由 6.1 SOL 编写。

## 2. 工具放在哪里

归档根：[scripts/dynanav_exploration/archive_20261111](../../scripts/dynanav_exploration/archive_20261111/README.md)。20 个 Python 原文件和8个小 JSON 共154,979字节，均保持原字节；[清单](dynanav-exploration-tools-20261111.manifest.json)记录每个来源路径、目标路径、长度与 SHA256。

- `snapshot_live.py`：调用正式 observe，冻结当代 observation、front RGB、图像来源和 operator grid，绘制高度组辅助图。
- `long_observe.py`、`active_status.py`：快照加实测点云多视角、只读状态/回执和高度误差。`explore_111150_read.py` 是本轮实际执行的 run 绑定入口。
- `active_observe.py`、`building_read.py`、`complete_read.py`、`continued_observe.py`、`continued_read.py`、`final_observe.py`、`repaired_read.py`：早期不同 run 的薄绑定入口。它们的存在和来源已确认，不用最后一轮的调用记录代替每个历史入口的独立调用证明。
- `coverage_audit/record_current_lio_cloud.py`、`render_live_3d_views.py`：整理为 skill 前的 collector/3D 原件。已有 [项目 skill](../../.agents/skills/dynanav-explore/SKILL.md) 提供当前可复用 SOP、收集器和渲染器，保留原有6文件快照，不重复安装，不以旧原件覆盖修订版。
- `coverage_111150/render_generation.py`：同代输入绑定、全输入域/局部 XY-XZ-YZ/多角度3D、真实 postVIO 轨迹、grid和回执审计。
- `coverage_111150/render_observed_roi.py`：显式 XY ROI 的 YZ/斜视检查。
- `coverage_111150/render_upper_region.py`：基于实测云参照面的上层相对高度显示；仍含本轮阶段时间，不能直接泛化为任意楼层算法。
- `coverage_111150/render_final_lower_stage.py`：首次上楼前774.7秒的下层显示。该阈值属于这代证据，不是通用分层规则。
- `coverage_111150/*.json`：本代 binding、region参数、上层参照面、缺口标注/ledger，原样随工具保存。
- `repaired_coverage/`：前一 `native_exploration_20261111T102621Z` 代的三个渲染/整理工具和三个配置，不与111150坐标系合并。

## 3. 实际使用证据

除原探索线程的直接调用外，保存的历史执行记录还确认最后一轮四个 coverage 渲染器都曾成功退出0；精确命令与记录 ID 位于清单的 `verified_coverage_invocations`。相应原始产物仍在今日归档，不迁入仓库：

- `explore_111150_evidence/s01/scene_view_metadata.json` 与各 `sNN` 快照保存实际渲染元数据。
- `coverage_audit/live_111150/summary.json` 保存15,129对精确同stage LIO、5cm体素、4.4–1777.7秒；collector晚于仿真启动，不能称完整原始全程地图。
- `coverage_111150/coverage_final/output_manifest_v3.json` 逐项记录最终图像、轨迹和回执产物 SHA。
- `coverage_111150/coverage_final/{coverage_record,upper_observed_record,lower_stage_record}.json` 保存显示参数、时段、来源和限制。
- `coverage_111150/s20/stair_platform_roi_record.json` 对应实际ROI检查。
- `coverage_111150/pairing_audit_final.json`、`explore_111150_evidence/coverage_final/freeze_manifest.json` 保留最终输入配对与冻结证据。

这些外部证据的当前长度与SHA也已收录清单，但文件本体没有复制。源代码备份/候选补丁、不再使用的collector/finalizer变体仍留原目录；当前项目实现没有被历史版本覆盖。

## 4. 怎样复用

当前探索操作使用项目已有 skill，参数必须显式绑定当前 run；不要运行历史薄入口去观察新run。归档只读渲染器中，generation/ROI/upper三个CLI支持无动作 `--help`：

```sh
REPO=/path/to/embodied-navigation
TOOLS="$REPO/scripts/dynanav_exploration/archive_20261111/coverage_111150"
python3 -B "$TOOLS/render_generation.py" --help
python3 -B "$TOOLS/render_observed_roi.py" --help
python3 -B "$TOOLS/render_upper_region.py" --help
```

重放 generation：把归档 `generation_binding.json` 复制到外部临时工作区，核实 `run_id/run_root/evidence_root/allowed_input_roots` 全部指向同代实际输入，把 `output_root` 改为新建的外部输出目录，然后运行：

```sh
python3 -B "$TOOLS/render_generation.py" --binding /external/work/binding.json --snapshot s01
```

不要只改run名称来混用旧云或旧pose。读取NPZ、PNG、JSON和整个控制journal仍需原数据盘；克隆仓库不会带来这些数据。重放会生成新图和统计，但不发ROS控制、不启动Isaac。upper和lower仍包含本轮阶段阈值/显示经验，先审阅再用于其他run。`render_final_lower_stage.py`、`build_final.py` 及根目录观察脚本存在顶层执行，不对它们做 `--help` 试跑，也不直接在仓库内重放。

留白不代表free/unknown，可见回波不证明支持/净空，height chart不是物理楼层数，轨迹长度不代表覆盖率。实测参照面仅用于显示，不将地图拍平或改动导航门禁。

## 5. 本次验证

本次s01重放读取当前归档脚本和最终完整journal，得到178,071点、132个postVIO样本、5张PNG；历史s01 `coverage_record.json`记录131个样本。文件版本/读取时点并非逐调用冻结，本次只验证工具可运行，不宣称逐像素或逐样本重现当时实时输出。

离线验证结果另存 [validation](dynanav-exploration-tools-20261111.validation.json)：归档SHA、全部Python语法、小JSON解析、3个无动作CLI、CPU单测，以及使用原s01输入写入临时外部目录的真实渲染重放。未跑完整仿真或全量测试，不以这些离线检查宣称新的导航验收。

[当前功能架构](../dynanav-exploration.md)及[32航点执行记录](dynanav-native-exploration-32-waypoints-20261111.md)仍是功能/执行边界入口。本次只补缺失工具与证据索引，不实现自动规划、地形分层或完整覆盖。

## 6. 可实际复用的项目集成

仅保留历史原件还不足以供新run直接使用。后续检查发现原generation renderer只识别旧`measured_cloud.npz`和`level*_operator_map_*.json`，而当前SOP写`measured_partial_cloud.npz`和`height_chart_*.json`。现增加：

- `scripts/dynanav_exploration.py`：可发现的统一入口，提供`sop/collect/view/review/roi`。只调用当前checkout内的代码；SOP自动绑定当前repo，个人skill不改。
- `scripts/dynanav_exploration/{review_generation,review_roi}.py`：从原generation/ROI工具提取的活动版本，支持两种快照格式；新输出、路径/标签和不覆盖输入保护；保留同代/frame/source-time、回执和轨迹限制。不调用归档或旧task目录下的Python文件。
- `requirements-dynanav-exploration.txt`与[实际使用说明](../../scripts/dynanav_exploration/README.md)：声明CPU绘图依赖、ROS边界、SOP/独立3D/review/ROI示例和binding契约。
- `tests/unit/test_dynanav_exploration_tools.py`：12项离线入口、路径隔离、格式兼容、源过滤和回执前缀检查。与10项原件检查合计22项通过。旧原件的未关闭读取句柄ResourceWarning保留可见，活动版本已关闭句柄。

活动review明确拒绝覆盖已有`snapshot`输出，ROI请使用另一个新`output_root`。上/下层和run专用观察入口仍按历史脚本保存，没有把实验阶段时间泛化为自动楼层识别。原28个文件、源目录及项目skill的SHA均未改变。

真实s01数据离线检查见[可复用工具验证](dynanav-exploration-tools-20261111.reusable-validation.json)。原输入格式和当前SOP文件名格式的同一实测数据均重放；另实际执行独立3D查看、ROI和正式SOP只读status。所有图写入新的外部临时目录，不覆盖历史产物；未启动仿真、实测实时采集或发目标。此前的validation保留为原件归档验证，不改写成新代码的验收。

最终全量CPU检查：unit `1086 tests OK (skipped=3)`；regression `108 tests OK (skipped=8)`，两条命令均exit0，skip不算通过。离线重放生成13张新PNG（临时目录另有1张复制的RGB输入），已查看全域多视角和ROI图；当前SOP布局测试是同一实测数据的文件名兼容重放，不是新在线快照。`git diff --check`通过。没有commit/push。
