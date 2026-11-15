# Interactive waypoint exploration: operator manual (method validation, 2026-11-10)


### 2026-11-10 turn33 双阈值 12 度对照（新增，不覆盖以上历史）

* 新方案实测启用：控制死区 yaw_control_tolerance_rad=0.02、完成带
  yaw_completion_tolerance_rad=0.05、IMU 角速 <=0.03 rad/s、连续 >=0.5 sim s、
  eval gap 0.020001 sim、TTL 30 wall s；运行 limited_livo_yaw_dual_20261110T140000Z。
* 完成窗 45.815-46.415 sim（31 个 50 Hz 样本，最大 gap 0.020000000000003），误差
  0.026990-0.033580 rad，gyro |max| 0.021017，有效窗 0.60；control_band_met=false、
  completion_band_met=true；pre-release wz 0.032388，46.395 为最后非零，46.415 起持续零，
  因此 release = complete。
* 相对初始姿态：完成 native 10.480 / postVIO 10.454 度；+1 sim 9.253/9.465；+5 7.573/7.641；
  +10 7.251/7.341；+20 7.199/7.309；native 20 sim 回摆 3.281 度。最终 87.315 sim native 净
  7.200 度、末 1 sim 变化 0.00214 度，已稳定但稳定在非目标角；不得写成达到 0.02 或回摆已解决。
* 全 XY 命令 0；动作期漂移 5.36 mm、释放后 4.00 mm；bodycontact 0、tilt 7.312 度。
  本 run 有 truth_scoring_stream 可做独立评估，控制未使用 GT。
* 本 run 没有生成 operator map（map 启动器 import 失败），不是本 run 逐云 0 accepted；
  历史 map 0-accepted/floor 不足属其它 run。无可用导航 map，未前进/跨层/全楼探索。
* 视频 capture1/multiwaypoint05_overview_and_rgb.mp4：H264 1024x512，873 帧/87.3 sim s，
  ffprobe 与全解码无错误；是真实仿真录屏，非真实物理狗。
* 可复跑启动 recipe 与只读 import smoke：
  .dsh-subagents/dsh-full-building-exploration-20261110/launch_recipe.sh 与
  smoke_map_import.sh（同一前缀，import_exit=0、map_help_exit=0；仅证明环境导入可用）。

## 当前状态（2026-11-10 最新，以实际运行结果为准）

* 图像通道已实证可用：本会话 read_image 成功读取本次真实 run
  `limited_livo_static_validation_20261110T102500Z/capture1/interactive_frames/rgb_000039.png`
  （sha256 f9d9b0a9…, source_time_s 78.215）。历史上 “模型不声明 image input” 的失败是历史事件，
  现在不再成立；不得据此回退纯文字决策。
* SLAM 已切换为 FAST-LIVO2 LIVO：本次真实 launch receipt 为
  `post_vio_odometry_enabled=true`、`navigation_pose_source=livo_post_vio`、
  `navigation_pose_topic=/dynanav/livo_navigation/post_vio_odometry`；实测 odometry
  `header.frame_id=dynanav_lio_map`、`child_frame_id=go2_imu`，status 报文
  `visual_update_applied=true, state_stage=post_vio, twist_available=false`。
  早期 capture2 是 LIO mode，属历史，不能当作视觉融合证据。
* 操作图本代不可行动（最终产物计数：frames 229、accepted 0、pose_frame accepted 191，
  last cloud 79.415000916 sim s）：真实源周期是 0.1 s（post-VIO 中位 0.099998 s，另有一次
  0.300003 s 断档；`ros2 topic hz` 报的是 RTF<1 下的墙钟 Hz，不是源周期），失败原因是
  **cloud 到达即消费、post-VIO 对应姿态尚未到达**，不是“5 Hz 天然不能配 0.10 s”。
  已实现“先准入后写入”的有界等待/重试补丁：每条云在任何 add_frame 之前先按原 stamp、原
  arrival、当前权威 clock 校验 ROS 帧、有限性、未来/过期（sim≤0.25 s、wall≤0.5 s），再
  以只读 preview 判姿态可配；只有缺姿态才 pending（同一 wall 窗 + 64 条硬上限，溢出记
  pending_overflow），其余一律一次终局拒绝且不写 level/cell/accepted；坏帧立即
  untrusted_generation 且不重试。配对窗仍 0.10 s，不伪造姿态、不外推、不改 stamp。
  复核脚本现为 late_pose/sim_stale/future_arrival/nan_arrival 均 geometry 0。tracker 侧位姿与感知门正常
  （pose fresh、perception validated current、clearance 1.2019 m、7944 云点），
  全程未发任何动作。
* 流统计（取自 policy_frames 恢复的 685 个唯一 post-VIO source_ns）：源间隔中位
  0.100000381 s，约 0.1×559、0.2×106、0.3×17、0.4/0.5 各 1；wall arrival 中位 0.18492 s，
  RTF≈0.5105；camera/cloud 878 帧严格 0.1 sim s（不是 20 Hz）。地图逐帧到达/配对日志
  未保存，最终 −0.3003 的配对缺口只能作机制推断，不能当逐帧因果。
* 视觉证据层级：本次仅保存了**一次真实 status 样本**（`post_vio_status_sample.json`，
  `visual_update_applied=true, state_stage=post_vio, twist_available=false`，
  image_stamp_ns=39814998627），没有采到完整成功计数，因此不声称“计数通过”。
* 点云来源标注：`/dynanav/lidar_filtered` 是 RGB-D 深度反投影点云（header `go2_imu`），
  不是 LiDAR 扫描；该 topic 名只是历史命名。
* GT native pose / mapping / mask 仅用于分离诊断与评分，不是 SLAM 图或前沿输入；
  全楼探索尚未验收。

Status of this document: written by the method-validation session. It describes the
interface as implemented and the offline evidence that exists. It does NOT claim a
successful Isaac run: see "What has and has not been executed".

## 1. Scope

* The base is driven by an operator (a DSH session) that submits sparse waypoints
  (every 1-5 s) in the live estimator map frame. A 50 Hz follower inside the
  capture process turns each waypoint into a low-frequency speed command.
* Static simulation scene only. No pedestrians, no learned semantic pipeline, no
  dynamic-object policy.
* Ground truth is never a pose or waypoint input. It is allowed only for isolated
  oracle diagnostics and offline scoring, and every receipt records that fact.
* Hard command ceiling: 1.0 m/s. Stairs receive no automatic slow-down.

## 2. What has and has not been executed

Executed (offline, CPU only):

* Unit suite: 518 passed, 3 skipped.
* Regression suite: 82 passed, 5 skipped. Combined run of both: 600 passed, 8 skipped.
* Interactive interface file: 72 tests (including the stand-gate hold regression and
  the live perception.json consumption cases); operator map module: 59 tests;
  operator client: 32 tests; those three files together: 163 passed.
* Profile check through the real entry point (no simulation started):
    /home/hyh001/isaacsim/python.sh scripts/run_dynanav_experiment.py --check \
      --output $PROJECT_DATA_ROOT/embodied-navigation/experiments/<new> \
      --interactive --interactive-pose-source livo_post_vio --speed 1.0 --duration 300
  It exited 0 and printed the interactive profile, including
  navigation_pose_source = measured_livo_post_vio,
  odometry_topic = /dynanav/lio_observer/post_vio_odometry,
  interactive_require_perception = true,
  interactive_min_clearance_m = 0.55,
  interactive_gt_usage = ground truth only for isolated oracle diagnostics.

Executed for real (Isaac + LIO observer, run
interactive_method_trial_20261110T080631Z under the experiments directory):

* four capture runs, each ended with end_reason navigation_terminal_and_parked
  (the native parking verification, 5 s parked plus 1 s rest, passed every time);
* run capture: a live observer for about 60 s, a measured pose
  (sim_age 1.4e-6 s, wall_age 0.0016 s), a live perception.json (front clearance
  1.17 m, 7944 cloud points, depth_valid_ratio 1.0), depth_*.npz snapshots and an
  mp4 video;
* runs capture2..capture4: no observer, so the tracker held
  invalid_measured_odometry and then terminated interactive_pose_lost instead of
  driving without a pose (the gate did its job).

NOT executed: any commanded motion. The single accepted goto was accepted and then
the run ended with interactive_link_lost because the heartbeat was not kept alive;
runs capture2..capture4 had no observer (a mapper cannot survive a simulator restart
because the sim clock rolls back, and the launcher refuses an existing output
directory). Measured speed stayed at 0.0004 m/s. No arrival, no scan, no stop through
the mailbox in a real run.

## 3. Coordinates: the only frame that may be commanded

* Waypoints are (x, y, z) in the live estimator map frame, plus the measured base
  pose of the moment the request is created.
* The z of a waypoint is the base-pose z in the estimator frame, i.e. base origin.
  It is NOT floor height and NOT a world height. The base origin sits about 0.30 m
  above the floor in this scene; feeding a world floor height (-3.40 / 0.00 / +3.20)
  as a waypoint z is a category error that the tracker only catches through its
  vertical tolerance (0.18 m) after the request was already accepted.
* A new session must never reuse world z constants inside the repeat-spawn relative
  SLAM map. World heights belong to offline diagnostics only.
* The follower compares the waypoint to the measured pose: cross-track tolerance
  0.6 m, overshoot 0.35 m, vertical tolerance 0.18 m, arrival radius 0.85 m.

## 4. Interface

Request (one JSON file, atomically replaced), at <run>/interactive/request.json:

    {"version": 1, "seq": 7, "action": "goto",
     "goal_xyz_m": [1.0, 0.0, -3.15], "ttl_s": 30.0,
     "pose_source": "lio_estimate", "run_id": "run-...",
     "created_wall_s": <time.perf_counter() of the client>}

Supported actions: goto, waypoint, cancel, stop, yaw, maneuver.
* yaw: {"delta_yaw_rad": +/-3.15}. In-place turn relative to the measured yaw,
  tolerance 0.05 rad, gain 1.2, no translation.
* maneuver: {"body_forward_mps", "body_lateral_mps", "yaw_rate_rps", "duration_s"}.
  Bounds: |forward| <= 1.0, |lateral| <= 0.05, |yaw_rate| <= 0.15, 0.1 <= duration <= 2.0.
  It is a bounded local nudge (the WASD-like layer), never a replacement for waypoints.
* A "scan" is a client-side loop of yaw actions, not a new simulator action; it also
  writes one observation snapshot per step under <mailbox>/scan/.

Observation (2 Hz), at <run>/interactive/observation.json:
phase, position, yaw, measured speed, active waypoint and distance, queue length,
terminal reason, perception gate fields (source, frame, clearance, stamp, points).
Receipts are appended to <run>/interactive/receipts.jsonl.

Operator client: scripts/explore_waypoint_client.py with subcommands heartbeat, goto,
yaw, maneuver, scan, cancel, stop, observe, map. The protocol also accepts action
"waypoint", but the client submits it as goto; there is no separate waypoint verb. The
run selector --run-dir / --mailbox / --run-id is accepted both before and after the
subcommand; a negative case exits 3 (argument or protocol error) or 2 (topic not
received).

## 5. Live perception gate (new, unverified in simulation)

The capture process writes <mailbox>/perception.json at 10 Hz from the live RGB-D
obstacle cloud, in the base frame, with the self-echo box removed:

    corridor: x in (0.30, 4.0) m, |y| <= 0.45 m, z in [-0.05, 1.20] m
    clearance_m = min x of corridor points, else 4.0 (clear)
    cloud_points >= 1000 required

The tracker reads it at 20 Hz and commands zero, with a bounded reason, when the
sample is missing, unlabelled, older than 0.5 s wall or 0.30 s sim, has fewer than
1000 cloud points, or shows clearance below 0.55 m. A perception failure that lasts
longer than 5 s wall becomes the terminal reason interactive_perception_lost; a
close obstacle becomes interactive_obstacle_blocked. These are additional to, not
replacements for, the original joint/torque/tilt/speed/height/boundary asserts.

The corridor height band deliberately starts at -0.05 m: base origin is about
0.30 m above the floor, so a -0.35 m lower bound would have counted floor returns as
obstacles and pinned the clearance at 0.30 m, i.e. permanently blocked. That is a
geometric argument, not a measurement; the first real run must log the actual
clearance before any waypoint is submitted, and the numbers are not tunable from
the CLI.

## 6. Operator map: reuse the real cloud, never fabricate

Command:

    python3 scripts/explore_waypoint_client.py map --run-dir <run> \
      --cloud-topic /dynanav/lidar_filtered \
      --odometry-topic /dynanav/lio_observer/post_vio_odometry \
      --sensor-frame go2_imu --base-frame go2_base \
      --extrinsic-translation TX TY TZ --extrinsic-quaternion QX QY QZ QW \
      --duration 20

It subscribes to the live SLAM point cloud, feeds every measured odometry pose into
a stamp pairer (position lerp, quaternion slerp, |dt| <= 0.10 s or refuse), transforms
the cloud into the base frame with the caller-supplied extrinsic, clusters levels by
measured pose z only (0.30 m band, 0.15 m hysteresis), estimates the floor from the
low percentile of the level cloud, accumulates occupied/free/unknown cells per level,
and writes one JSON snapshot per level under <run>/operator_map/.

Every snapshot carries frame names, source labels, cloud and pose stamps, pairing dt,
level z, floor offset, extrinsic sha256, frontier with an actionable flag and its
reasons (stale pose, pairing gap, missing extrinsic, ambiguous level, fewer than 200
points), plus path_length_is_not_coverage = true, snapshot_is_accumulated_map, and
ground_truth_used = false. Missing or invalid extrinsic raises before any geometry is
used; the command exits 3.

Two things must not be confused: measured path length is not coverage, and a
single-frame snapshot is not an accumulated map. The snapshots are decision aids for
the operator, they are not fed back into the 50 Hz follower.

## 7. Camera evidence for the operator

The same capture pass writes one real rendered RGB still every 50 frames (5 s at
10 Hz) to <run>/interactive_frames/rgb_000000.png ... plus index.jsonl with the sim
stamp, and the observation bundle reports the newest still under sim_frames. These
are frames from this simulation, not offline ground-truth images, and they carry no
pose. If the consuming session cannot display images, it must say so instead of
claiming it looked at them.

## 8. Launch sequence (manual; there is no supervisor for this combination)

The existing supervisor scripts/run_dynanav_online_person.py cannot be used: it
forces --capture-result and --slam-reference and waits for a ready.json handshake,
none of which hold for an interactive measured-pose run. The launch is therefore
sequential and manual, and every step must be confirmed before the next:

1. Confirm no simulation is already running. Inside this sandbox a pid namespace is
   isolated, so pgrep and ros2 node list inside the sandbox are false negatives; the
   check must run where the host processes are visible. Do not start a second run
   blindly.
2. Choose a new run directory under $PROJECT_DATA_ROOT/embodied-navigation/experiments;
   the entry point refuses an existing directory and refuses paths outside it. The
   mailbox must not exist either.
3. Environment, in this order: source /opt/ros/jazzy/setup.bash; load .env.local
   (set -a; . ./.env.local; set +a); export PYTHONPATH=<repo>/src (the entry script is
   a shim and does not inject sys.path); choose an isolated ROS_DOMAIN_ID shared by
   the observer and the capture; then run /home/hyh001/isaacsim/python.sh.
4. Start the observer first, in livo mode, because this interactive profile reads
   /dynanav/lio_observer/post_vio_odometry:
     scripts/run_dynanav_lio.sh --mode livo --output-dir <observer dir under the run>
   LIO mode publishes a different topic and writes no ready.json; livo mode does.
   Verify liveness with ros2 node list and ros2 topic hz on the odometry topic.
5. Start the capture through the entry point with --interactive
   --interactive-pose-source livo_post_vio --speed 1.0 --duration <s>. Duration must
   be within [8, 300]; motion ends at duration - 5 s; the last 5 s of simulation plus
   1 s of rest are reserved for parking.
6. Read <run>/interactive/observation.json until the stand gate has opened. The
   tracker holds with reason not_ready during the initial stand gate (about 6 s of
   simulation plus the 7 s initial wait) and that hold is explicitly not a lost pose.
7. Check the perception block of the observation: source live_rgbd_obstacle_cloud,
   frame go2_base, clearance plausibly above 0.55 m, cloud_points >= 1000. If the
   clearance is pinned low, stop and fix the corridor instead of driving.
8. Submit one short waypoint with the client, within known free space, and watch the
   observation until reached or until an explicit terminal reason appears.
9. Optionally run one scan (yaw steps) and snapshot the observation each step.
10. Stop: submit the stop action through the mailbox, or write <run>/stop_request.json
    (the environment variable NAV_STOP_REQUEST already points there). The run is only
    complete when the receipt reports end_reason navigation_terminal_and_parked,
    which requires 5 s of parking plus 1 s of rest.
11. Keep the videos, logs, observation bundle, receipts, operator map snapshots and
    stop receipts in the run directory. Do not delete them.

## 9. Known gaps and unverified claims

* No real run happened in this session: perception thresholds, corridor band, hold
  timing, parking, video content and 1.0 m/s tracking are unverified in simulation.
* The 1.0 m/s figure is a command ceiling, not a measured speed. Measured speed only
  appears in the observation and in the run receipts.
* Continuous waypoint pass-through (no stop-turn at each waypoint) is a documented
  deviation from the older route controller and is unverified.
* The operator map uses the SLAM point cloud, but the extrinsic has to be supplied by
  the caller; there is no calibration file in the repository, so a wrong extrinsic is
  a silent geometry error. The map must not be used as a navigation input.
* Multi-floor exploration is explicitly out of scope for this session; the geometry
  audit (three walkable levels at about -3.40 / 0.00 / +3.20 m, stairs connecting all
  three, 140 mesh primitives without CollisionAPI in the current office scene) is
  offline evidence and must not be treated as a traversal guarantee.

## 10. Rollback

All changes are uncommitted. To revert the interface: delete
src/embodied_navigation/simulation/dynanav/exploration_waypoints.py,
src/embodied_navigation/simulation/dynanav/operator_map.py,
scripts/explore_waypoint_client.py, tests/unit/test_dynanav_interactive_waypoints.py,
tests/unit/test_operator_map.py, tests/unit/test_explore_waypoint_client.py, this
document, and revert the interactive_control hooks in capture/control.py,
capture/frames.py, capture_go2_rgbd.py and experiment.py. Nothing was committed.

## 2026-11-10 second validation run (capture6, ROS domain 92)

All numbers below come from one interactive run with a fresh owned observer
(observer6, /dynanav/lidar_filtered) and a continuous heartbeat. No ground truth
is used by the loop or by these receipts.

* waypoint_reached receipt seq 1, goal [0.611, -0.062, 0.014] m in the live
  estimator frame (z is the measured base height of flat ground, not a spawn
  height copied onto a different floor), stage 1, queued 0.
* yaw scan: two steps of 8 deg; achieved -0.0386 rad and +0.0362 rad with errors
  0.048 and 0.049 rad (within tolerance), each reported by yaw_complete.
* stop accepted (seq 4) -> terminal interactive_stop, run end_reason
  navigation_terminal_and_parked (native parking verified).
* measured path length 0.686 m (LIO estimate), 33 time based stills every 2 s
  (source times 0.115 s .. 64.215 s), video 8.34 MB.
* perception now labels each direction from its own rays: front measured
  (clearance 1.175 m, 325 corridor points, 7944 rays) while rear/left/right are
  insufficient_observation with clearance_m null and rays 0, so a forward frame
  can never authorise a rear or side move.


### 监督复核：yaw 请求量不等于实际转角

capture6 的两个 yaw 请求各为 8°（0.13962634 rad），完成容差为 0.05 rad。两个 yaw_complete 的剩余误差分别为 0.04781670 / 0.04909617 rad，因此对应实测完成转角约为 5.26° / 5.19°，不是精确 8°。运行时应继续依据真实估计 yaw 和每个动作回执判断姿态，不能把请求角累加当作实际扫描覆盖。这里是 LIO 估计转角；native 姿态评分须另列。


## 2026-11-10 perception chain repair: frame and pixel-index pairing

An independent read-only review found two defects in the interactive perception
producer (capture/frames.py). Both are now fixed, with producer-side tests that use
the real depth_cloud_packet arrays instead of ideal unfiltered clouds.

What the producer actually returns (src/embodied_navigation/perception/depth.py):

* points: N x 3 in the Go2 IMU frame, already filtered by valid depth, by the body
  self-echo box and by the point-budget stride;
* pixel_flat_indices: C-order image index per surviving point, same filters/stride;
* depth_keep_mask: image-shaped bool (512 x 512) of valid depth;
* self_mask: 1-D bool over the valid-depth points, i.e. length = keep.sum(), not the
  image size.

The old consumer mixed those two index spaces by position (keep_mask[:count] with
self_mask[:count] against an already filtered IMU cloud) and published the IMU cloud
as go2_base. The mask arrays live in different spaces, so the filter paired unrelated
entries; the frame relabel dropped a measured 0.02015 m x / -0.00511 m z offset
(imu_mount from the capture6 result).

Repair (scripts and modules stay pure numpy, no Isaac import):

* capture/perception.py: base_frame_cloud(packet, imu_mount) returns
  base = imu + imu_mount (the IMU is axis-aligned with the base, so this is a pure
  translation) and gathers the validity/self flags through packet.pixel_flat_indices.
  Any shape, dtype, length, range or calibration inconsistency raises.
* capture/perception.py: interactive_perception_payload(...) builds the published
  sample; a raising chain becomes a fail-closed sample (observation_state
  "producer_geometry_invalid", clearance null, cloud/corridor points 0), which the
  tracker turns into a hold, never into free space.
* capture/frames.py: _publish_interactive_perception now takes the packet itself and
  no longer slices masks by prefix.

Measured before/after on the real capture6 frames (same depth image, same mount):

* clearance 1.1748 m (IMU cloud) -> 1.2014 m (base cloud); corridor points 325 -> 298;
* synthetic frame with invalid top rows and a 0.6 m obstacle: old code produced an
  empty cloud and insufficient_observation (spurious hold), new code measures 0.90 m;
* synthetic frame with a self-echo cluster: old 0.5798 m, new 0.6000 m; the old subset
  was chosen by the mismatched mask, not by the pixel the point came from.

Tests: tests/unit/test_dynanav_capture_perception.py covers non-zero IMU offset, a
rotated sensor mount, stride with non-contiguous pixel indices, invalid depth and
self echoes at scattered pixels, front/left obstacles in real base coordinates, and
fail-closed behaviour for missing/malformed extrinsics, out-of-range or non-integer
indices, wrong lengths, wrong image-mask shape, bad self-mask length, bad point values
and empty clouds.

Not yet verified in simulation: the repair is offline-tested only. A fresh run must
confirm the live perception against a known obstacle and a self-echo sample before any
waypoint is submitted; capture6 perception.json is not a calibrated-safety reference.

Residual risk found while auditing (not changed here): the front corridor starts at
0.30 m, so a real obstacle closer than 0.30 m with |y| between 0.30 m and 0.45 m is
outside both the body self-echo box and the corridor; an otherwise empty corridor then
reports clear_free at 4.0 m. Lowering the forward bound to 0.05 m would make the gate
stricter (more points can only reduce clearance) but must be measured in a real run
first, so it is reported rather than silently changed.

Operator map snapshots: scripts/explore_waypoint_client.py map used to write level
snapshots only after its duration elapsed. It now writes the same snapshots plus
operator_map/latest.json every --snapshot-interval seconds (default 2.0) while it runs,
with snapshot_is_partial_until_final marking running writes, so the operator can read a
fresh accumulated map without waiting for the process to exit.

## 2026-11-10 control-tick fail-closed repair (CPU only)

Six independent control defects were reproduced against the current tracker before
being fixed. They share one theme: a safety gate existed but could be bypassed by a
path that never reached it. The reproduction script is
`.dsh-subagents/dsh-full-building-exploration-20261110/control_repro.py`; all numbers
below are its before/after output.

1. Duplicate-odometry ticks replayed the cached command. The control tick runs faster
   than the estimator publishes, so most ticks see the same stamp; that branch returned
   `_last_command` and skipped the maneuver direction gate. With an accepted -1 m/s
   reverse and the rear clearance changed from 2 m to 0.1 m under an unchanged stamp it
   still commanded -1.0 m/s. Now the duplicate branch re-runs the local-action command
   (direction gate, TTL, clocks) and clamps it with the same envelope: 0.0 m/s plus
   `maneuver_accepted` -> `action_cancelled obstacle_too_close:rear`. A forward maneuver
   with a close rear obstacle correctly keeps running; left/right strafes cancel on
   their own side.
2. `_last_odom_stamp_s` was only read, never written, so the monotonicity gate could
   never fire. A 10.0 s -> 9.95 s estimator rollback drove at 1.0 m/s. Now only an
   accepted, fresh, labelled estimate becomes the baseline (future, stale-sim and
   stale-wall samples return earlier and cannot move it), equal stamps stay duplicates,
   and a rewound stamp holds `nonmonotonic_odometry_stamp` while the baseline keeps the
   highest accepted value. The window span therefore can no longer go negative.
3. A future-dated heartbeat returned before the previous valid heartbeat deadline was
   checked, so after the last real heartbeat expired, continuing to write future ones
   kept the link alive (1.0 m/s at age 6 s wall). Every rejected heartbeat - future,
   non-finite wall, non-object payload, foreign run id - now still runs the deadline
   check against the last valid heartbeat: 0.0 m/s and terminal `interactive_link_lost`,
   with `_last_heartbeat_wall` unchanged. A tracker that has never accepted a heartbeat
   holds `operator_link_not_established` and never commands motion; that wait does not
   open the pose-loss clock.
4. Direction clearances were compared without validation, so NaN/Inf/None/`"1.0"` passed
   (`NaN < 0.55` is false) and authorised a reverse. Each required direction is now
   validated with `_finite` before any comparison and rejected as
   `perception_direction_invalid:<name>` (or `..._uncovered:<name>` when absent).
5. A heartbeat file containing a JSON list raised `AttributeError` out of the control
   tick. The payload must now be an object; object shape, `wall_s` type and `run_id` are
   validated, and no exception escapes the tick: 0.0 m/s, terminal
   `interactive_link_unreadable`, receipt `heartbeat_not_an_object`.
6. The maneuver direction test used a 0.01 deadband, so a 0.01 m/s reverse needed no rear
   coverage. Any non-zero body translation now requires coverage of its own direction,
   however small; a 0.01 m/s forward creep with front coverage is still accepted.

Regressions added to `tests/unit/test_dynanav_interactive_waypoints.py` (57 -> 102
tests): duplicate-odometry re-admission for reverse/left/right and the forward control,
rewind/bad-frame/first-sample stamp handling, non-finite and bad-typed clearances, tiny
maneuvers, heartbeat liveness (future, list, empty object, bad `wall_s`, foreign run id,
unparseable file, no initial heartbeat), and a producer-to-tracker flow that feeds real
`direction_coverage` output into the tracker, including the duplicate-pose rear-obstacle
cancellation. All thresholds are unchanged: 0.25 s sim / 0.5 s wall odometry freshness,
2e-6 s future tolerance, 5 s wall link loss, 0.55 m clearance, 1 m/s ceiling, request and
waypoint TTL, pose-loss timeout.

## 2026-11-10 operator-map fail-closed repair (CPU only)

Three map defects were fixed in `operator_map.py` (untracked new module) and are covered
by 12 new tests in `tests/unit/test_operator_map.py` (59 -> 71 tests). All existing
thresholds (0.25 s sim, 0.5 s wall, 0.10 s pairing, 0.15 m, 0.20 m, 2.5/4.0 m, 200
points) and reason strings are unchanged; no ground-truth floor is used anywhere.

7. Floor evidence. The floor was the low quantile of whatever points fell into the
   search band, so a desk top, a shelf or the floor of the level above could become the
   floor and be carved free. `_assess_floor_evidence()` now requires all three of: a
   candidate strictly below this frame's measured base z and at least one obstacle
   clearance below it, at least 50 supporting points in the free band around it, and
   agreement (0.30 m) with the level's already established floor. On failure `add_frame`
   refuses before writing any cell, with `floor_evidence_missing` /
   `floor_evidence_inconsistent` in the receipt and `frontier.actionable=False`; the
   snapshot exposes the whole evidence record. Reproduction: a frame with only a 0.5 m
   wide elevated surface 0.70 m above the base used to be accepted with floor_offset
   +0.70 m and 400 free cells (actionable); it is now refused (`floor_evidence_missing`,
   no free cells, non-actionable). A real floor patch behind an occluding desk still maps
   normally (390 free, 25 occupied), so the gate does not over-refuse. A level-0 frame
   containing upper-level floor points no longer uses them as level-0 ground and the
   level stays non-actionable until a frame with real floor evidence arrives.
8. Ray origin. Free-space carving started at the base origin, which is wrong by the
   sensor-to-base lever arm and rotation. It now starts at the measured sensor origin,
   `pose_used @ extrinsic.matrix` (`T_map_base @ T_base_sensor`), the same composition
   used for the points. Reproduction with t=(0.5,0,0), R=yaw 90 deg and a measured base
   yaw of 90 deg: the cell between base and sensor was carved FREE before and is UNKNOWN
   now, while the cell on the true sensor-to-obstacle ray is still carved FREE.
9. Stalled streams. Freshness was evaluated once at ingestion and the stored verdict was
   returned by later snapshots, so a stopped pose or cloud stream kept the map actionable
   forever. Ages are now resolved on every `snapshot()` call from the caller's current
   clocks (`now_sim_s` for the source sim clock, `now_wall_s` for the arrival monotonic
   clock; no cached now and no cached age is reused). Negative or rolled-back sim/wall
   ages are refused with `sim_clock_regression` / `wall_clock_regression`, stopped streams
   with `stale_pose_sim_age` / `stale_pose_wall_age` / `stale_cloud_sim_age` /
   `stale_cloud_wall_age`, and a level whose last frame had no ground evidence with
   `level_stale`. Without a caller now the recorded arrival is compared with the current
   monotonic time, so even the existing client (which passes no now) no longer keeps an
   old fresh verdict after both streams stopped.

Not changed on purpose: the range-filter origin in `points_to_map` is still the base
origin (changing it would change which points are dropped); `merge()` still prefers the
target's established floor; the client was not given `now_sim_s` because it has no
authoritative source sim clock - it only knows message stamps, and a stopped stream is
exactly what the wall age detects.

Honest limits: CPU-only synthetic geometry, not validated against a live SLAM run or a
recorded bag. The evidence gate is a fail-closed heuristic: a base origin less than
0.15 m above its floor, or a visible floor patch with fewer than 50 points, is now
refused (safe but coverage-reducing). A horizontal surface below the base by at least
0.15 m with 50+ points that appears before any floor for that level is still
geometrically indistinguishable from a floor and is accepted; the required desk,
mezzanine and shelf cases are above the base and are rejected.

## Control-side repair, sixth batch: global budget, future clocks, lateral coverage, near band

10. The motion budget is global. `motion_end_s` was checked only inside
   `WaypointQueueController.tick()`, so a yaw or a short maneuver action - neither of
   which goes through the follower - kept commanding after the budget had run out.
   Reproduction with the real profile budget: `current_sim_s=301`, `motion_end_s=300`, a
   live heartbeat and a fresh measured cloud. A yaw request returned `[0, 0, 0.15]` with
   `status=tracking` and no terminal; a maneuver request returned `[0.4, 0, 0]`. The
   tracker now evaluates `sim >= controller.motion_end` right after the monotonic-sim
   check, i.e. before the request, link, pose and perception gates, so every path is
   covered: goto, yaw, maneuver, duplicate odometry, cached command and idle. On expiry
   it zeroes the command and latches `interactive_motion_budget`; the existing runtime
   stop-request path then reaches `navigation_terminal_and_parked`. After the fix both
   requests return `[0, 0, 0]` with `status=terminal` and the same terminal reason, and no
   later action can start. Tests: `MotionBudgetTests` (yaw, maneuver, goto, duplicate
   odometry, no action after the budget).
11. The two perception clocks are validated separately. `_perception_gate` compared the
   source age against a symmetric window, which accepted an arrival far in the future
   because the arrival age was only checked from above. Reproduction: with a fresh sim
   stamp and `arrival_wall_s = now + 4000 s` the tracker emitted `[1.0, 0, 0]`
   (`tracking`). The gate now rejects `age_wall < -future_tolerance_s` with
   `perception_arrival_in_future` and `age_sim < -future_tolerance_s` with
   `perception_stamp_in_future`; a negative age is never clipped to zero, and a
   non-finite arrival is unavailable (fail closed) rather than fresh. Both holds keep the
   pose-loss clock and recover on the next valid sample; a duplicate pose cannot excuse a
   future perception. Tests: `PerceptionFutureClockTests` (future arrival, future stamp,
   NaN arrival, recovery, duplicate pose with a future file).
12. A lateral correction is real motion, so it needs lateral observation. The follower
   emitted its cross-track correction without asking whether that side was observed: with
   a lateral offset of 0.1 m and the right side `insufficient_observation` it returned
   `[1.0, -0.05, 0]`. The tracker now runs `_direction_reason` on any non-zero goto lateral
   component and, when that direction is missing, stale or closer than 0.55 m, holds with
   `goto_lateral_uncovered` plus `lateral_direction_reason` (e.g.
   `perception_direction_uncovered:right`) and the original command in the receipt. The
   correction is not silently zeroed: the robot stops, the operator sees the reason, and a
   side observation (for example a yaw scan) releases the hold. A goto that is on its
   segment needs no side coverage and still tracks at 1 m/s. Tests:
   `GotoLateralCoverageTests`.
13. The near band was dropped. `_corridor` selected `along > 0.30` only, so a return that
   survived the body self-echo box but sat inside the near band was not a clearance and a
   healthy frame reported `clear_free` at 4.0 m. Reproduction: a real return at
   `(0.28, 0.30, 0.10)` - `|y| = 0.30` escapes the `|y| < 0.3` self box - is classified
   left, has `along = 0.30` exactly at the corridor start, and the left corridor was empty
   (`clear_free 4.0` with the obstacle 0.30 m away). Surviving near returns now set the
   measured clearance (`near_field_return`), so the same frame reports `measured 0.30`,
   which the tracker refuses with `obstacle_too_close:left`. The published `CORRIDOR`
   record gains `near_field_m` and `near_field_returns_are_obstacles`.

   Correction to the earlier status note: the claim that an obstacle at `x < 0.30`,
   `|y| = 0.30..0.45` is invisible is only true at the boundary. `classify_directions`
   assigns a point by its dominant axis, so such a point belongs to left/right and its
   `along` is `|y| \>= 0.30`, i.e. inside that direction's corridor. The genuinely
   unobservable boundaries are the producer's self-echo box (`|x| < 0.5`,
   `|y| < 0.3`, `z > -0.45`, so an obstacle 0.30 m dead ahead is indistinguishable from
   self echo) and the height band (`z > -0.05` relative to the base, so a 0.15 m kerb
   under a base 0.30 m above the floor never reduces the clearance). Both are pinned by
   `NearFieldBoundaryTests` so the numbers cannot be forgotten; the low-surface case is
   the map floor-evidence gate's job (item 4), not the clearance gate's, because point
   geometry alone cannot separate a kerb from a floor.
14. The screenshot channel cannot be exercised by this session. `read_image` on a real
   capture6 still (`capture6/rgb_7000.png`, sim stamp from `camera_frames.jsonl`) fails
   with "model deepseek-v4.1-flash does not declare image input". The stills and their
   sim stamps exist and are historical method frames, but this model cannot see them, so
   no screenshot was read and no action was chosen from one. This is not silently
   degraded to text: the joint screenshot + local SLAM map decision needs an
   image-capable model on the agent that runs the real exploration, or an explicit
   user-approved non-visual channel. Until the parent decides, this remains open.

Not changed on purpose: the 0.30 m corridor start, the 0.45 m half width, the
`-0.05..1.20` m height band, the producer's 0.5 x 0.3 x -0.45 m self box, the 0.55 m
clearance gate, the 0.25 s / 0.5 s freshness windows, the 2e-6 future tolerance and the
1 m/s ceiling. The lateral gate holds instead of changing the tracking law, so the
cross-track error is not corrected while a side is unobserved; the waypoint queue is kept
and resumes when the side becomes observed.


## Operator-map repair, seventh batch: stalls, low surfaces, refusals, clocks, levels

All five cases were reproduced through the call shape the client actually uses,
`accumulator.snapshot(level_index, frontier_limit=...)` with no `now` arguments, and
re-verified after the fix with the same programs.

10c. Cloud stall in the no-now path. With one accepted frame (cloud stamp 1.0 s), the
   cloud arrival 0.6 wall seconds in the past and the pose stream still live, the
   snapshot returned `actionable=True` with `reasons=[]` and
   `cloud_age_evaluated=False`: the fresh pose stood in for the stalled cloud. Each
   level now owns its last valid cloud source stamp, arrival and ingestion sim age,
   separately from the pose stream, and `_resolve_snapshot_ages` re-evaluates them
   unconditionally on every snapshot for the requested level. The same call now returns
   `actionable=False, reasons=['stale_cloud_wall_age'], cloud_age_evaluated=True,
   cloud_wall_age_s=0.602, pose_wall_age_s=0.004`; with a 0.05 s cloud it still returns
   `actionable=True, reasons=[]`. A missing arrival or a non-finite now makes the age
   unevaluable (`cloud_age_unevaluated` / `clock_invalid`), which is never actionable.
   Integration evidence: `.dsh-subagents/dsh-full-building-exploration-20261110/integration_cli_map.py`
   drives the real `_write_operator_map_snapshots` and reads the written level file.
10d. A low surface is no longer floor. With the base 0.30 m above the floor and the only
   visible surface a 400-point plate 0.15 m high (true floor occluded, first frame of the
   level), the frame used to be `accepted` with `floor_z=0.15`, 400 free cells and
   `actionable=True`. Candidate points plus a 50-point minimum are now only a candidate:
   acceptance needs established-floor agreement with support inside the established free
   band, a caller-measured stand proof within 0.15 m, or a measured standoff of at least
   0.30 m together with a 0.50 m support span on both axes. The plate is refused with
   `floor_evidence_unconfirmed`, 0 free cells and a non-actionable level; the same frame
   plus a visible real floor patch maps normally (route `measured_standoff_and_extent`),
   and a floor with a 0.15 m step strip maps the step as occupied.
10e. A refused frame no longer keeps the map actionable. A valid 40x40 floor frame
   followed by an empty cloud used to leave `actionable=True` with 1600 free cells. The
   refusal is recorded per level and the snapshot is now
   `actionable=False, reasons=['no_points','latest_frame_refused']` while the older
   geometry is kept; it stays non-actionable past the freshness window, and a refusal on
   one level does not block another.
10f. Non-finite clocks. `add_frame(sim_age_s=nan)` used to be accepted and the snapshot
   reported `actionable=True` with `pose_ages.sim_s=nan`; `snapshot(now_sim_s=nan)`
   raised. Ages and caller nows are now finiteness-checked before any comparison:
   `pose_fresh=False` with `pose_age_invalid`, `actionable=False` with
   `clock_invalid` for a NaN/Inf now, `cloud_age_unevaluated` with
   `cloud_wall_age_s=null` for a NaN arrival, and an invalid arrival is not repaired by
   a later fresh sample.
10g. Per-level freshness. A stale level 0 used to be refreshed by level 1's fresh data
   (global `_last_frame` reference) and stayed `actionable=True` with
   `cloud_wall_age_s=None`. The observation store is per level now: level 0 reports
   `actionable=False ['stale_cloud_wall_age']` with `cloud_wall_age_s=1.001` while level
   1 stays actionable, and repeated level-0 snapshots keep ageing.

Clock authority is documented in the module header, `_resolve_snapshot_ages`,
`add_frame` and `snapshot`: arrivals are recorded on `time.monotonic()`; a caller
`now_wall_s` must be that same clock; `now_sim_s` is the source/estimator clock; the
two are never substituted; no cached now or age is reused; and map contents and current
actionability are separate facts.

Honest limits of this batch: an extended flat surface at least 0.30 m below the base
(a wide pallet, a mezzanine edge, a platform) is still indistinguishable from a floor
when no established floor and no measured stand proof exist; the 0.50 m span rule rejects
narrow ledges, not wide plates. The stand-proof route trusts the caller's provenance. The
no-now path can only age the cloud by its recorded arrival, so a frozen source clock with
continuing wall arrivals is invisible to the sim age; a caller with a real clock should
pass `now_sim_s`. The CLI only supplies a sim age when `/clock` messages arrive, so a
run without a clock topic will see a non-actionable operator map by design.

## CLI action results, eighth batch: rejected and cancelled actions stop the wait

`wait_for_action` only matched this sequence's completion, so a `request_rejected`,
`action_cancelled`, `waypoint_dropped` or a global `terminal` fell through to the
timeout: an operator saw a rejected action as still running, and a re-send was invited.
Reproduction with a seeded closing receipt and `timeout_s=0.3`: every case returned
`{'event': 'timeout', 'seq': 1, 'timeout_s': 0.3}` after 0.306 s. The wait now returns the
receipt's own fields plus `status` (`completed | rejected | cancelled | dropped |
expired | held | terminal | timeout`), `success`, `reason`, `seq` and `run_id` in about
0.000 s, and the CLI exit code is `0` completed, `1` rejected/cancelled/dropped/expired/
held/terminal, `2` timeout, `3` local usage. A receipt from another run or an older
sequence is not mistaken for this action, a seq-less `terminal` is global, and a seq-less
`hold` only ends the wait after this sequence was accepted and the hold is newer, so a
startup hold cannot fail a request that is still being consumed. A missing, corrupt or
undecodable receipts file does not raise. `--wait` / `--timeout` were added to `goto`,
`yaw` and `maneuver`, and `scan` propagates each step's outcome. Two deliberate
semantics: a TTL-expired turnaround is now a failure (`expired`), not a completion; and
`waypoint_reached` counts as completion so `goto --wait` can succeed.

## Screenshot channel: blocked by the session model

The image tool exists but this session's model (`deepseek-v4.1-flash`) does not declare
image input, so `read_image` on a real capture6 still fails with
"model deepseek-v4.1-flash does not declare image input". The historical stills and their
sim stamps exist (`capture6/rgb_*.png`, `camera_frames.jsonl`), but no screenshot was
read and no action was chosen from one. This is reported, not silently degraded to text:
the required joint decision on a live screenshot, the local SLAM map and the live pose
needs an image-capable model for the agent that runs the real exploration, or an explicit
user-approved non-visual channel. It stays open until the parent decides.


## Final-review CPU batch: cached goto admission, CLI source clock, public API clocks

10h. Cached goto commands go through the same admission as fresh ones. The duplicate
   odometry goto branch returned `_last_command` before the lateral gate, so a side that
   became unknown or too close kept being strafed through a fresh pose stamp.
   Reproduction: first goto with right measured returned `[1.0, -0.05, 0]`; a duplicate
   fresh odometry sample 100 ms later with right `insufficient_observation` still
   returned `[1.0, -0.05, 0]` (`fresh_duplicate_odometry`), and a 1e-5 m offset kept a
   -5e-6 m/s strafe. Both the follower path and the cached path now call
   `_admit_goto_command` before any command is returned, so the cached command is held
   with `goto_lateral_uncovered` and `command_source=cached_odometry` plus the underlying
   `perception_direction_*` reason, and resumes when the side is observed again. Tests:
   `CachedGotoLateralGateTests` (right unknown, near side obstacle, 1e-5 offset, left
   side, recovery, no-lateral replay).
10i. The CLI feeds the map a real source clock and unclipped ages. `_write_operator_map_snapshots`
   passed no `now_sim_s`, so a source clock far ahead of the cloud stamp could not be
   detected, and the ingestion loop clipped both ages with `max(0, ...)` (a future stamp
   became a fresh zero). Reproduction through the production helper with clock 10 and
   cloud/pose stamps 4010: `actionable=True, reasons=[], cloud_sim_age_s=0.0`. The client
   now records the arrival of every `/clock` sample, passes `now_sim_s` only while that
   sample is current (0.25 s wall), passes the raw source age (`clock - stamp`) and the raw
   arrival age without clipping, and passes the cloud and pose arrivals so the map can
   re-evaluate them. After the fix: future stamps -> `actionable=False`,
   `sim_clock_regression`, `cloud_sim_age_s=-4000`; sim 0.4 s behind the wall clock with a
   fresh wall age -> `stale_cloud_sim_age`; healthy 0.05 s -> `actionable=True`; a stale
   clock sample is not used as now (`source_clock_used=null`, `clock_sample_stale`) and a
   cloud stopped 0.6 wall s with a fresh pose still fails `stale_cloud_wall_age`. The
   manifest records `source_clock_used` / `source_clock_note`. The source sim clock and
   the arrival monotonic clock stay independent. Tests: `OperatorMapSnapshotClockTests`
   (current clock, missing clock, stale clock, future stamp, sim delay, cloud stall,
   per-level ageing, refused empty frame), all through the production writer and the
   written level file.
10j. The public `update_perception` validates its own clocks. The live path reads the
   producer file and already rejects non-finite values, but the public method accepted a
   NaN arrival and the gate then compared NaN ages as false, authorising motion.
   Reproduction: `update_perception(now_wall=nan)` then a goto request returned
   `[1.0, 0, 0]`. The method now raises `ValueError` for a non-finite arrival, stamp,
   clearance, depth ratio or per-direction clearance, and for a negative or non-integer
   point count. Tests: `PerceptionApiValidationTests` (arrival, stamp, clearance,
   direction clearance, valid sample) plus the retained gate-level defence
   `test_non_finite_rear_clearance_never_authorises_reverse`, which injects a NaN sample
   directly and still gets a zero command.
10k. Stand-proof trust boundary. A caller that passes `stand_proof_z_m` can still make a
   surface 0.15 m below the base acceptable (`measured_stand_proof`), even though the
   interactive client never passes it. The module cannot prove that a proof was measured
   rather than synthesised, so the `add_frame` docstring now states the boundary: the
   proof is trusted caller measurement, a navigation caller must never invent one, and a
   real proof must trace to the current stand gate, contact state or a recorded
   calibration, never to ground truth or a static building height.

Image source time correction: `rgb_7000.png` is physical frame index 7000, not 7.000 s;
the paired source time is 35.015 s (`truth_scoring_stream.jsonl` line 7000 and the
`camera_frames.jsonl` frame at 35.015 s). Both the probe record and the status now carry
the correction and the rule that stills are paired by frame index or message source time,
never by filename.

