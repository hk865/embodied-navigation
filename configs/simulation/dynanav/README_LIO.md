# DynaNav RGB-D LIO / RGB-assisted LIVO observer

The default `--mode lio` consumes `/dynanav/lidar_static`, `/dynanav/imu` and
`/clock`; explicit `--mode livo` also consumes raw `/dynanav/camera` RGB.
The cloud comes from camera depth, so it is not an independent LiDAR sensor.
It does not consume GT pose or send motion commands. Its outputs are remapped
under `/dynanav/lio_observer/`. The default GT diagnostic controller does not use
these estimates; explicit `run_dynanav_experiment.py --slam-reference ...`
consumes this observer's measured poses for the repeat-spawn route experiment.
The completed same-capture observer comparison and current closed-loop status are in the
[continuation report](../../../docs/experiments/dynanav-20261109-continuation.md);
the [earlier report](../../../docs/experiments/dynanav-20261109.md) preserves prior failures and partial comparisons.

## 目录结构

```text
configs/simulation/dynanav/
├── README.md                              # 配置索引与消费点（81 行）
├── README_LIO.md                          # 本文件：观察器启动、标定契约与实验记录
├── camera_rgbd.yaml                       # 512×512 pinhole 内参基准（18 行）
├── fastlivo_lio.yaml                      # ONLY_LIO 模式参数（100 行）
├── fastlivo_lio_gravity_estimation.yaml   # 单变量 gravity 对照覆盖（6 行）
├── fastlivo_livo.yaml                     # RGB 辅助 LIVO 模式参数（129 行）
└── office.yaml                            # 历史场景/资产路径记录，无代码读取（5 行）
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 配置 | [`fastlivo_lio.yaml`](fastlivo_lio.yaml) | 本文件默认 `--mode lio` 的参数文件：`img_en: 0`、`allow_lio_only: true`，post-VIO 输出关闭。由 [`scripts/run_dynanav_lio.sh`](../../../scripts/run_dynanav_lio.sh) 按 `fastlivo_${mode}.yaml` 选择并做模式/post-VIO 一致性 preflight。 |
| 配置 | [`fastlivo_livo.yaml`](fastlivo_livo.yaml) | `--mode livo` 的参数文件：`img_en: 1`、post-VIO 输出开启、`/dynanav/camera`；视觉参数是本轮实验设置，不是精度结论。 |
| 配置 | [`camera_rgbd.yaml`](camera_rgbd.yaml) | 相机内参基准。livo 分支读取它作为参数服务基线，再用 `--capture-result` 的 `sensor_installation` 覆盖并校验（`camera_parameter_ready.json` + `camera_parameters.log`）。lio 分支不使用该文件。 |
| 配置 | [`fastlivo_lio_gravity_estimation.yaml`](fastlivo_lio_gravity_estimation.yaml) | 通过 `--parameter-override` 追加的单变量 gravity 对照；base 默认仍为关闭。 |
| 配置（无读取点） | [`office.yaml`](office.yaml) | 历史路径记录，无代码读取；实际场景/机器人路径硬编码在 `simulation/dynanav/experiment.py` 的 `data_root()` 下。 |
| 代码 | 本目录无代码 | 读取点是 shell 启动器 `scripts/run_dynanav_lio.sh`、回归测试 [`tests/regression/test_dynanav_lio_launcher.py`](../../../tests/regression/test_dynanav_lio_launcher.py)，以及被观察器消费的 native 二进制（`fastlivo_mapping`/`libfast_livo2_core.so`）。 |
| 日志/输出 | 外部 `--output-dir`（必须新目录且在 `PROJECT_DATA_ROOT` 内） | 本目录不写日志。运行期产物：`preflight.json`、`launch.json`（含二进制/core/config 实际哈希）、`camera_parameter_ready.json`、`camera_parameters.log`、`runner.log`、native diagnostics、snapshot 的 `bridge_status.json` 等，全部属该次 run 的证据，不入 Git。 |

本文件是**说明文档**而不是配置读取点：没有任何代码解析 `README_LIO.md`，文中哈希/运行号/结论都只是记录。

## 模块内文件耦合

- 依赖方向：README 文档之间单向引用（本文件 ← → 同目录 [`README.md`](README.md)）；四个 YAML 互不 include，按 `--mode` 与 `--parameter-override` 组合，不存在继承链。
- 隐式契约（字符串级）：本文件引用的 topic 名、`/dynanav/<ns>/...` 命名空间、`kind12/stage1/payload[0]` 诊断键、`final_stage=livo`/`lio_vio_skipped` 快照字段、SHA256 与 run 标识，都必须与 native 代码/launch receipt 一致；改实现不改本文不会报错，但本文会过期。已发生过的例子：`0004`/`0005` patch 改变生命周期与 post-VIO 输出后，旧哈希段落被保留为“历史/旧 core”，而不是被删除。

| 文件 | 行数 | 依赖 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `README_LIO.md` | 354（原文 294 + 本次索引 60） | 同目录 YAML、外部 run 证据、native patch | 人；`tests/native/dynanav_lio/README.md` 与 `configs/simulation/dynanav/README.md` 引用 | 偏重：单文件承载“当前默认 + 历史证据 + 操作手册”三种角色 |
| 四个 YAML | 6–129 | native ROS 参数契约 | 启动器/参数服务 | 清楚，见同目录 `README.md` 的耦合表 |

结论：耦合**清楚**（无代码环、无隐藏继承），但文档与实现之间是弱校验的字符串契约；风险在于原文 294 行里的历史哈希/run 标识会随重建过期，需要“当前默认”和“历史记录”分层阅读。

## 可读性评估

- 结构：**需改进**（指原文正文）。原文 294 行、56 个自然段、8 个 H2（`RGB-assisted LIVO startup`、`Camera and sensor contract`、`Actual visual updates and post-VIO scoring`、`Live final post-VIO pose output`、`Same-capture comparison and raw RGB evidence`、`Single-variable observer parameters`、`Instantaneous point time correction`、`Camera-free object lifetime`），但原文没有目录（本次只加了文档索引与耦合说明，未重排正文），“当前默认”的信息散落在第 14–65 行与各历史段落之间；最长段落 16 行。
- 内容：**可接受**。命令、标定公式、拒绝条件、边界（“hash/CPU checks do not certify a physical route”）都写得很具体；缺点是同一主题（core 哈希、prefix）在多处重复，且 6 处 SHA256 段落需要读者自己判断哪一处是当前默认（正文已用 “current default / retained as historical” 区分，但没有汇总表）。
- 语言：英文长文与本目录其它中文 README 混用；这不影响正确性，但同一目录风格不统一。

这是**文档组织问题**，不是代码结构问题；没有发现虚构表述或未标注的推测。

## 重构建议

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P2 | 文首加目录（8 个 H2 链接）+ 一段“当前默认 vs 历史对比”导航 | 294 行线性叙述，新读者无法先拿到当前结论 | 无（只加索引段） | 人工从目录跳到各节；原有事实段落不改 |
| P2 | 增加“构建身份/哈希汇总表”：current default / retained historical 两列，列出 prefix、core/runner SHA256、CTest 结果 | 7 处 SHA256 分散，重建后极易引用错 | 无（纯文档；数字须从 build-identity.json/test-result.txt 复核） | 与 `build_cache/.../build-identity.json`、`test-result.txt` 逐项对照 |
| P2 | 把“实验设置 ≠ 精度验收”的边界句统一成固定句式（如每节末尾 `Boundary:` 行） | 当前边界散落在段落中，易被摘要时丢掉 | 无 | 抽查 8 节是否都有边界句 |
| P3 | 统一本目录文档语言（或在文首声明“本文件保留英文原文，索引见中文 README.md”） | 同目录中英混杂 | 无 | 人工阅读 |

没有 P0/P1：本文件不参与运行，错误只会误导读者；已有 `--check` 与 preflight 失败保护运行路径，不需要结构性改造。本轮只写建议、不改代码；任何哈希表更新都应以构建产物重新核对后再写。

## 观察器说明与实验记录（原文保留）

`fastlivo_lio.yaml` selects the existing native `ONLY_LIO` estimator with the
explicit `common.allow_lio_only: true` opt-in and `common.img_en: 0`. The native
default remains the original IMU+RGB V19 path. In opt-in mode no camera service,
VIO initialization or image subscription is required. `camera_rgbd.yaml` records
the current capture's 512×512 intrinsics (fx=fy=512×24/42, cx=cy=256, scale=1).
`--mode lio` does not apply that camera file. `--mode livo` starts the owned
camera parameter service and overlays actual capture calibration as described
below. Neither mode implements a visual dynamic mask; static-cloud filtering
does not mask image patches.

Depth points are already expressed at the `go2_imu` origin in body axes, so the
LiDAR-to-IMU transform is identity, with zero translation. Native estimated
poses use `dynanav_lio_map`, not the scene's GT world frame. Initialization
retains the existing 300-sample stationary IMU gate; its span requirement is
2.9 seconds. No guard is relaxed. Static-mask clouds containing zero or one
usable point are explicitly dropped with stamp/size/count warnings; subsequent
valid frames remain reachable, without filled or synthetic points.

From the repository root, with the ROS Jazzy host dependencies installed:

```sh
export PROJECT_DATA_ROOT=/media/hyh001/新加卷/6.project_data
scripts/run_dynanav_lio.sh --check
ROS_DOMAIN_ID=91 scripts/run_dynanav_lio.sh --output-dir \
  "$PROJECT_DATA_ROOT/embodied-navigation/experiments/UNIQUE_LIO_OBSERVER"
```

The output directory must be new and inside the external project data root.
`--check` only inspects files, the opt-in capability marker and dynamic-library
dependencies; it does not start a node or establish estimator quality. The run
command starts only the observer, with stdout/stderr in `runner.log`, ROS logs,
native diagnostics and `launch.json` containing the actual binary/core/config
hashes. Stop the launcher with SIGINT or SIGTERM; it only cleans up children it
created. The directly executed LIO native runner also supports cooperative
SIGUSR1 stop.

An explicit `--binary-prefix` selects a directory containing `install/ros`.
`--dependency-prefix` selects a ROS install prefix containing `lib` and `share`.
The defaults under `PROJECT_DATA_ROOT/embodied-navigation/build_cache` are the
separate `dynanav_post_vio_20261109T052245Z` build and the existing public-dependency
prefix `native_validation_20261108T124116Z/project/install/ros`. These paths are
build-cache selections, not imports from an old navigation checkout. The
pre-opt-in native-validation runner/core remain untouched: their SHA256 values
are respectively
`e4190901bc0f057462ca612f2df101f3a0506fdecc20ef2e7b65a985832f3da4` and
`fc111fce9da5cefffa5b9fd134df9c739c718931e3897cc03476a471d03bfb9a`.
That old core rejects image-free startup; it cannot run this configuration.
The new core is compiled from this repository's opt-in patch, reusing existing
Livox/vikit dependencies without downloading or rebuilding them. Launch hashes
identify the actual selected result; the prefix name alone is not a binary
version. Hash/CPU checks do not certify a physical route, dynamic filtering or
clean teardown; the later lifecycle finding is recorded below.

## RGB-assisted LIVO startup

Use [fastlivo_livo.yaml](fastlivo_livo.yaml) with `--mode livo`. This selects
`common.img_en=1`, `allow_lio_only=false`, IMU/LiDAR enabled, generic RGB-D
points, no GPS/HILTI, and `/dynanav/camera`. It cannot silently fall back to
image-free LIO. The explicit visual experiment uses native defaults:
`img_point_cov=100`, `outlier_threshold=100`, exposure estimation enabled,
8-pixel patches, four pyramid levels and five iterations. These are experiment
settings, not an accuracy acceptance result.

```sh
export PROJECT_DATA_ROOT='/media/hyh001/新加卷/6.project_data'
CALIBRATION_SOURCE="$PROJECT_DATA_ROOT/embodied-navigation/experiments/dynanav-20261109-basement-ground/capture/result.json"
scripts/run_dynanav_lio.sh --check --mode livo \
  --capture-result "$CALIBRATION_SOURCE"
```

`--capture-result` reads only `sensor_installation`: optical R/P from IMU,
intrinsics, camera mount and IMU mount. Native poses and trajectories from that
GT diagnostic are not estimator inputs. The launcher verifies finite, rigid
rotation and mount-consistent translation, hashes the source, and generates
`capture_calibration.yaml`. It validates actual parameter replies at
`/dynanav/lio_observer/parameter_blackboard/get_parameters` before starting the
mapper (8 s discovery plus 2 s reply timeout). The blackboard is owned by the
launcher, in the mapper's namespace and ROS domain. Missing service or wrong
values fail startup; Vikit's hardcoded fallback is not calibrated success.
`--check` starts no service, mapper or simulator.

To retain selected native post-VIO keyframes, prepare a new external run and an
explicit snapshot override. Use the same snapshot policy in the LIO comparison:

```sh
NEW_VISUAL="$PROJECT_DATA_ROOT/embodied-navigation/experiments/livo-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir "$NEW_VISUAL"
cat > "$NEW_VISUAL/snapshot-override.yaml" <<EOF
/**:
  ros__parameters:
    backend.snapshot_directory: "$NEW_VISUAL/observer/native_keyframes"
    backend.max_ready_directories: 256
    backend.min_interval_s: 1.0
    backend.max_interval_s: 2.0
EOF
ROS_DOMAIN_ID=91 scripts/run_dynanav_lio.sh --mode livo \
  --capture-result "$CALIBRATION_SOURCE" \
  --parameter-override "$NEW_VISUAL/snapshot-override.yaml" \
  --output-dir "$NEW_VISUAL/observer"
```

Start the sensor capture separately in the same ROS domain. This launcher
never starts a simulator or navigation controller. Check `preflight.json`,
`launch.json`, `camera_parameter_ready.json`, `camera_parameters.log` and the
actual camera INIT in `runner.log`; launch receipts only describe intent.
Monitor snapshot `bridge_status.json` queue/pool/spool drops and I/O failures.
The snapshot default ready-directory capacity of 16 is too small for preserving
an entire long route unless the spool is consumed; the override above records
its explicit experiment capacity.

## Camera and sensor contract

RGB is uncompressed `rgb8`, 512×512, `go2_camera_optical`; the native subscriber
uses SensorDataQoS and explicitly converts rgb8 to BGR. RGB and the derived
cloud carry the same assigned sensor stamp. `require_complete_imu=true` waits
for IMU coverage at the camera time. Offsets are zero for this capture; do not
change timestamps to conceal initialization or synchronization behavior.

The body-axis point cloud already has origin `go2_imu`. LiDAR-to-IMU R/T stays
identity; do not rotate it twice or substitute a USD camera world transform.
For the yaw-zero capture, the optical projection contract is:

```text
p_optical = Rcl * p_go2_imu + Pcl
Rcl = [ 0 -1  0;  0  0 -1;  1  0  0 ]
Pcl = [0, 0.13510895531624556, -0.2798469357192516] metres
fx = fy = 292.57142857142856; cx = cy = 256.0
width = height = 512; scale = 1.0; distortion = 0
```

Pcl equals `Rcl * (imu_mount - camera_mount)`. The loader requires double focal
and principal-point parameters. Native variable labels Rci/Pci are misleading;
the actual projection equations establish this IMU-to-optical direction. The
launcher derives each run's R/P from its recorded capture, including camera
yaw, instead of assuming this example matrix for every capture.

An independent saved-keyframe roundtrip checked all 7,944 points: positive
optical depth, maximum pixel-index reprojection error 5.68e-14 pixels and depth
error 8.88e-16 m. This validates that packet/calibration example, not all future
render timing or visual localization accuracy.

## Actual visual updates and post-VIO scoring

A diagnostic `kind12/stage1/payload[0]=1` stores state after `processFrame` at
image cadence. It is the post-VIO scoring source, including R/position; ordinary
`/dynanav/lio_observer/aft_mapped_to_init` is published in LIO before same-frame
VIO and must not be scored as post-VIO. Pair by actual source stamp and record
any numeric canonicalization, writer drops or missing states.

That kind12 flag alone does not prove a retained visual EKF correction.
`last_frame_ekf_updated` resets each frame and is set only after retained finite
updates with positive observations. Native snapshot metadata
`final_stage=livo` reflects it; `lio_vio_skipped` records no retained visual
update for that selected frame. Report update coverage separately from pose
error and from image receipt/camera INIT/colored-cloud counts.

The approximately 12 s [LIVO smoke run](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-smoke)
completed capture at physics 12.015 s. Its four selected native snapshots contain
three `livo` and one `lio_vio_skipped`; bridge written=4 with queue/pool/spool
and I/O failures all zero. This is actual retained finite visual-update evidence
for those selected frames, not full-route accuracy or all-frame coverage. The
LIVO launcher regression has eleven passing cases (eight mode cases plus
three namespace cases). The independent new-core
native lifecycle result below is separate from these selected visual frames.

## Live final post-VIO pose output

[0005-explicit-post-vio-pose-output.patch](../../../third_party/fast_livo2/patches/0005-explicit-post-vio-pose-output.patch) adds an explicit native `publish.post_vio_odometry_en` opt-in. The LIVO configuration enables it; ONLY_LIO and upstream defaults remain disabled. The launcher checks the new capability marker and maps pose/status topics inside the selected namespace. `aft_mapped_to_init` remains the pre-VIO topic.

Each valid final image state is published at most once on `/post_vio_odometry` and paired `/post_vio_odometry_status`, with the original image stamp, configured map/child frames and `visual_update_applied`. A valid frame with no retained visual correction still has a final pose and status false; duplicate, backward, nonfinite or LIO-stage samples are rejected. Publication does not modify estimator arithmetic. Pose/twist covariance and twist are unavailable: diagonal 1e12 and explicit status flags mark this, and zero twist is a placeholder. Consumers must not infer rest from that placeholder; the experiment uses a pose window and actual SDK gyro.

The completed paired run's [live integrity audit](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/post-vio-live-audit/live-post-vio-integrity.json) matches all 2,196 pose/status records to post-processFrame diagnostics, with position difference 0, correct stamps/frames and status counts 2,195 true/1 false. Writer/bridge recorded drops are 0. This validates the live output contract, not route control. The old offline derived Odometry bag remains a different product.

The current default [build identity](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/build-identity.json) and [CTest result](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_post_vio_20261109T052245Z/test-result.txt) record 2/2 native fixtures passing in 0.51 s. Core SHA256 is `d1ed90f137c1deb5949740bf2b11b9ae84406ca2492aa2151f397e9198c94e62`; runner SHA256 is `23eeac2abbc1531689e0317a8a3edf9f066294e6504fd62017225b9aaab88edd`. Both modes' actual `--check` have passed. See [native fixtures](../../../tests/native/dynanav_lio/README.md) for rebuilding the tests.

The new measured three-goal `livo_post_vio` reference is pose-only and repeat-spawn, with no global registration. The actual domain-110 closed-loop run used `/dynanav/livo_navigation/post_vio_odometry`; it completed two corners but ended in third-stage `route_low_progress_stall` at source268.714996338 s, with `arrived=false`. The continuation report links the final result and keeps the physical/control diagnosis pending. `--slam-reference`, `--odometry-topic`, `--slam-capture-remaining-m` and `--slam-capture-distance-m` select this route experiment; GT still supplies policy proprioception/physical guards/scoring, and never the route pose. The first run used 0.05/0.10 m capture bounds. New optional `--slam-final-capture-remaining-m` / `--slam-final-capture-distance-m` default to inherited bounds; an actual new run keeps intermediate 0.05/0.10 m and changes only the final goal to 0.15/0.20 m. Its controller reports arrival at source249.214996338 s and native parking samples end at254.415 s; all four supervised processes exit 0. The final independent review passes all 13 observed-run checks: three stages, two completed stop/turn transitions, measured final parking and exact controller-input provenance; it also has 2,367 exact pose/status/native correspondences. Its route journal includes 280 stale-sim-odometry zero-command holds (about 5.6 s of policy samples), so an empty legacy `freshness_hold_events` result array does not establish uninterrupted pose freshness. Capture subscribes only to final Odometry, not its status topic: guards use pose/frame/freshness, a pose-speed window and SDK gyro; configured unavailable twist is ignored. Status/covariance are not runtime controller gates, and their consistency is audited offline. Both visual closed-loop routes have no pedestrians; learned masks have not replaced the live oracle. Stall, speed, dwell and z checks remain in force. See the continuation report for repository-based launch commands, both run receipts and the final independent-review evidence. This actual repeat-spawn route completion is separate from earlier observer scores and from general navigation/perception certification.

A separately archived [0005 reverse/forward patch audit](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/0005-patch-roundtrip/audit.json) verifies the two vendor source hashes against the manifest and the retained `bffa2b6` baseline without editing vendor files.

## Same-capture comparison and raw RGB evidence

The completed [paired full route](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z) uses one GT-controlled capture and the same new core for both observers. The physical route ends at 235.415 s, all supervised processes exit 0, and acquired/published/archived/video RGB counts are all 2,354. This is an observer comparison, not visual navigation feedback.

| Mode | Namespace | Final navigation pose topic |
| --- | --- | --- |
| lio | `/dynanav/lio_comparison` | `/dynanav/lio_comparison/aft_mapped_to_init` |
| livo | `/dynanav/livo_comparison` | `/dynanav/livo_comparison/post_vio_odometry` |

Both receive the same 47,080 SDK IMU samples and 2,354 static clouds. Original RGB archive count 2,354, bag RGB count 2,176 and LIVO image ingress 2,234 describe different stages; complete capture does not establish complete DDS delivery. The primary common-exact-keyframe protocol covers 201/236 frames from 5.215000153 to 235.315002441 s. One initial SE3 per source is used only for scoring: ONLY_LIO RMSE/max/last-matched error is 1.919313/2.565622/2.545833 m; post-VIO is 0.083742/0.156613/0.155007 m. The shared ONLY_LIO initial alignment gives post-VIO RMSE 0.074619 m. No GT poses enter the estimators. See the continuation report for protocols, missing frames, snapshots and independent checks.

The default observer namespace remains `/dynanav/lio_observer`; `--observer-namespace` accepts an absolute non-root ROS name without a trailing slash. It isolates output topics, custom TF and the owned camera parameter service, while sensor topics remain shared. Give each process a separate new output directory and snapshot override path.

Earlier evidence remains in the historical report: the first complete standalone VIO route has a valid 185-KF score but a recorder-lock-truncated bag (675 cloud/CameraInfo, 644 RGB); two subsequent paired captures ended in SDK SIGSEGV with last persisted native truth near 36.11/62.71 s. Their prefix comparisons remain partial and are not replaced by new run identities. Later GDB stress directly observed a traceback-watchdog fault; it does not retroactively supply missing native stacks for those old runs. Removing unconditional periodic Python traceback dumping was followed by an 80 s capture test and the complete paired run above. Native lifecycle failure and SDK traceback failure are separate findings.

Capture `--archive-rgb` uses [rgb_archive.py](../../../src/embodied_navigation/simulation/dynanav/rgb_archive.py) to save each pre-overlay renderer RGB sample as ZIP_STORED `rgb8` NPZ with source_ns, frame_index, frame_id and encoding. At 512×512 this is about 0.75 MiB/frame, roughly 1.8 GB per route. Interrupted runs retain `capture_completed=false`. Video is not a source of original RGB.

[replay_dynanav_rgb.py](../../../scripts/replay_dynanav_rgb.py) reads only the RGB/source metadata whitelist, verifies source files and original order, and publishes due original images on received `/clock`. It does not create time, decode video, load GT arrays or resample. `--source-received-stamps` optionally freezes an exact archived subset; that is a subset experiment, not full source coverage. Its `--check` hashes/validates without importing ROS, and writes only a new receipt:

```sh
RGB_ARCHIVE="$PROJECT_DATA_ROOT/embodied-navigation/experiments/dynanav-no-timer-paired-full-20261109T052700Z/capture/rgb_raw"
python3 scripts/replay_dynanav_rgb.py --archive-dir "$RGB_ARCHIVE" \
  --receipt "$PROJECT_DATA_ROOT/embodied-navigation/experiments/rgb-check-$(date -u +%Y%m%dT%H%M%SZ).json" --check
```

The adapter also passed a real ROS-only [runtime audit](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rgb-replay-runtime-20261109T055456Z/runtime-audit.json): 627 original prefix frames published and independently received, with original headers/layout/pixel SHA verified. Only the finalized bag's clock was played, at wall rate 4 without changing source stamps. All owned processes exited 0; no SDK or native estimator was started. This does not establish full-route estimator replay or closed-loop navigation.

Recorders use explicit SQLite `write.pragmas` with WAL, synchronous NORMAL and busy_timeout 5000, as supported by [ROS 2 Jazzy rosbag2_storage_sqlite3](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_storage_sqlite3/README.md). These are recorded experiment settings, not permission to inspect an active writer; all bag validation waits for recorder shutdown.

## Single-variable observer parameters

`--parameter-override FILE` appends a second ROS parameter file, preserving the
base file and command array. The launch receipt records the override's absolute
path and SHA256. Input topics, selected LIO/LIVO mode, GPS-off selection, calibrated camera
parameters and simulation clock cannot be contradicted by an override. For the existing gravity-state estimator
experiment, use:

```sh
scripts/run_dynanav_lio.sh --check --parameter-override \
  configs/simulation/dynanav/fastlivo_lio_gravity_estimation.yaml
```

That file changes only `imu.gravity_est_en` to true. It is an explicit comparison
profile; the default YAML remains false, and no accuracy improvement is implied.

## Instantaneous point time correction

The generic RGB-D cloud represents one current-end acquisition. Its raw
per-point offsets are zero relative to its own header. The shared IMU deskew
routine instead expects offsets relative to the previous propagation end.
`ONLY_LIO` therefore rebases a working copy to that origin; source point order,
coordinates, intensities and raw offsets remain unchanged. Initialization
already advances that origin to every admitted frame, so its stationary window
is not replayed as one long scan. Timed non-generic sensors retain their
inherited offset/ownership path; this correction does not certify their
behavior across arbitrary gaps. This time-origin correction does not alter the default LIVO point-timing path.

The earlier `dynanav_lio_20261109T024731Z` prefix is retained as a pre-correction
comparison, not the current default. The native regression uses actual
`ImuProcess` propagation with nonzero yaw/acceleration between 100 ms and
200 ms instantaneous frames. The old core moved current-end points by
0.284937 m in that fixture and failed; the corrected core preserves them to
floating-point precision. It also verifies initialization reference time,
raw point identity and non-generic offsets. This is a numerical regression,
not evidence that real sensor SLAM has acceptable drift or that the office
route has been traversed under SLAM control. Existing pre-correction runs must
keep their own binary hashes and are not relabeled as post-correction results.

## Camera-free object lifetime

Patch `0004-safe-uninitialized-vio-lifecycle.patch` initializes the visual
manager's submap pointer to null. `ONLY_LIO` intentionally skips visual
initialization; its shared visual object can now be destroyed safely without
an allocated visual submap. The normal LIVO initialization still assigns its
allocated submap, with unchanged estimator arithmetic. The regression also
constructs the visual manager in deliberately nonzero storage, so a zero-filled
allocator cannot hide the missing initialization.

The retained offset-correction core with SHA256
`9f13c15ad9f326c55772066c12adc6378ddd463a96bff6b487c417c3c4913c51`
was observed to segfault in `VIOManager::~VIOManager` during native fixture
teardown, including with `rmw_fastrtps_cpp` and `UDPv4`. Earlier successful
numerical assertions do not establish safe process teardown. Existing run
receipts retain that core's hash; this source fix was rebuilt in a separate prefix and tested as recorded below.

The then-default lifecycle prefix was
`build_cache/dynanav_lio_lifetime_20261109T043801Z`. The current post-VIO prefix above includes this fix. Its
[build identity](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_lio_lifetime_20261109T043801Z/build-identity.json)
and [CTest result](/media/hyh001/新加卷/6.project_data/embodied-navigation/build_cache/dynanav_lio_lifetime_20261109T043801Z/test-result.txt)
record 1/1 native CTest passing in 0.28 s, covering deterministic poisoned
storage, LIO motion, default guard and sparse/mutex paths. The launcher default
was changed to that prefix and its file/hash/dependency `--check` passed.

- Core SHA256: `75ecf487c7d6f8f9f7856ec1cd7f2ac573e99b2ae80e3171b282f85ab5bbf7fc`.
- Runner SHA256: `98b4f1bf983bd3781e9e340702b87f554830095bfaab46e6bc528dfd47f2555e`.

The earlier `dynanav_lio_offset_20261109T030656Z` prefix is retained. Historical
LIO, LIVO smoke and the completed first dual-stair LIVO run keep their own
old-core hashes. LIVO calls `initializeVIO`, so it does not use the camera-free
uninitialized-pointer path. The two earlier interrupted paired attempts used the same new lifetime core for both modes; they
must not silently relabel live old-core results as new-core results. The lifecycle pass does not establish
full-route visual accuracy; that is measured separately in the completed observer comparison above.
