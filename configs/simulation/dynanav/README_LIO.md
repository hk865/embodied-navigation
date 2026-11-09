# DynaNav RGB-D LIO / RGB-assisted LIVO observer

The default `--mode lio` consumes `/dynanav/lidar_static`, `/dynanav/imu` and
`/clock`; explicit `--mode livo` also consumes raw `/dynanav/camera` RGB.
The cloud comes from camera depth, so it is not an independent LiDAR sensor.
It does not consume GT pose or send motion commands. Its outputs are remapped
under `/dynanav/lio_observer/`. The default GT diagnostic controller does not use
these estimates; explicit `run_dynanav_experiment.py --slam-reference ...`
consumes this observer's measured poses for the repeat-spawn route experiment.
The verified mode-specific results and outstanding same-input comparison are in the
[experiment report](../../../docs/experiments/dynanav-20261109.md).

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
separate `dynanav_lio_lifetime_20261109T043801Z` build and the existing public-dependency
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

## Same-input comparison and raw RGB evidence

Older full-route bags omitted raw RGB. Their lossy, text-overlay MP4 cannot be
relabelled as original 10 Hz images. The first
[dual-stair LIVO run](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-dual-stairs)
completed the GT physical route at 235.415 s and retained full native VIO logs,
but its recorder exited with SQLite database locked after a count query was
made against the live writer. The bag stops near 67.6 s: 675 cloud/CameraInfo messages and 644 RGB
messages. It is preserved as incomplete evidence, not a complete common-input
comparison dataset.

The completed live VIO stream can still be scored as a standalone estimator
run: [scores.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-dual-stairs/single-source-vio-scoring/scores.json)
and [trajectory figure](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-dual-stairs/single-source-vio-scoring/trajectory.png)
report 185 exact native-reference pairs spanning 5.215000153–234.315002441 s.
One initial SE3 is used only for scoring: RMSE 0.079707 m, maximum 0.160615 m,
last matched error 0.152466 m. No GT was supplied to the estimator; GT drove
the robot route, so this is not visually controlled navigation. It is also
not a same-input mode comparison because the bag is incomplete.

Sparse snapshots prove 123 retained `livo` updates and one `lio_vio_skipped`.
The 2,075 raw image headers and 2,037 post-VIO processed states are different
counts; processed does not prove a retained correction. Diagnostic writer
attempted=written=114,250, dropped=0 and I/O failure=false establish logging
integrity for emitted records, not processing of every published camera frame.
The last matched error is not an extrapolated final parking-time error.

The first replacement
[paired run](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-dual-stairs-paired)
feeds the same capture to both modes in domain 104, with the same new lifetime
core. `--observer-namespace` isolates outputs:

| Mode | Namespace | Custom TF output |
| --- | --- | --- |
| lio | `/dynanav/lio_comparison` | `/dynanav/lio_comparison/tf` |
| livo | `/dynanav/livo_comparison` | `/dynanav/livo_comparison/tf` |

The default remains `/dynanav/lio_observer`; custom namespaces must be absolute,
non-root ROS names without trailing slash. They change output/TF/service names,
not shared sensor topics. The LIVO camera parameter service is created inside
its selected namespace. Give each process its own new output directory and
snapshot override path. Final paired-prefix ingress, processed coverage, post-VIO scores and resource
checks are now recorded in the experiment report; simultaneous startup alone
does not establish identical processed-frame coverage or complete-route A/B.

That paired capture ended in an Isaac SDK SIGSEGV with last persisted native truth near 36.11 s. Both
observers and the recorder cooperatively exited 0 and retained the prefix;
RGB archive `capture_completed=false` is not a complete-route receipt. Buffered records/NPZ may extend further, so the precise crash tick is unknown.
This is separate from the diagnosed VIOManager teardown bug. Heavy PhysX material/face
warnings do not yet establish a root cause; archive causation, OOM and GPU Xid
have not been established.

The single clean-instance
[supervised retry](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-dual-stairs-paired-retry)
used domain 105 with the same two modes/new core/WAL/raw archive, but also
ended in SDK SIGSEGV with last persisted native truth near 62.71 s. Final `supervision.json` records
capture=1, LIO=0, LIVO launcher=130 from group stop, recorder=0; group-stop 130
is not evidence of an estimator crash. The supervisor stopped remaining owned
processes. Further SDK retries have stopped. Only the retained prefixes can
be compared on common input; full-route paired quality is not established.
Read-only checks found no OOM or root-USD change, and earlier successful runs
also had the warning storm; the final warning or raw archive is not an
established cause. The complete standalone VIO diagnostics can be scored,
but that earlier incomplete bag cannot establish full common-input A/B.

The saved common exact-keyframe prefixes were scored with one initial SE3 per
source at the same first GT anchor. The first 27-KF prefix has ONLY_LIO/VIO
position RMSE 0.015871/0.015430 m, but VIO maximum error is higher. The retry
49-KF prefix has RMSE 0.020962/0.021943 m. These incomplete route prefixes do
not show a consistent visual advantage. Source archives contain 361/627 RGB
frames, recorder bags 342/587 unique image headers, and VIO callback ingress
339/599 headers; archive existence does not establish complete DDS delivery.
See the experiment report for exact common stamps, protocol, missing frames
and source receipts.

The standalone completed VIO run also produced a
[post-VIO static map](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-dual-stairs/post-vio-static-map/receipt.json)
with 91,449 points from 185 exact keyframes (51 unmatched). Its pose source is
recorded post-VIO estimates represented in a derived offline Odometry bag,
not live-published odometry or the failed recorder bag; no GT pose is used for
mapping. Masks remain simulator oracle. It is a route-observed local
multi-level map, not complete office coverage or a visual navigation loop.

The finalized resource audit reports paired-active mapper CPU means of
9.70/14.08% (LIO/LIVO) for the first prefix and 9.56/13.72% for retry; 100% is
one core. Sampled active-window RSS peaks are about 123/235 and 126/321 MiB.
These exclude the rest of the stack and do not establish full-route peaks.
The first monitor missed about 15.27 s of initial activity; the report retains
its missing-coverage limit rather than reconstructing an unobserved peak.

The derived post-VIO map has also been independently audited: all 2,037
positions are bit-exact to native records, rotation roundtrip differs by at
most about 2.07e-14, and 185-keyframe/91,449-point map output is bit-exact
without reading archived GT transforms. A mutable scratch reader path drifted;
the exact as-run SHA was recovered from an archived copy, with original
receipts unchanged. See the report's independent-audit/source-resolution links.

Capture `--archive-rgb` writes every acquired, pre-overlay renderer RGB sample
with [rgb_archive.py](../../../src/embodied_navigation/simulation/dynanav/rgb_archive.py)
to `capture/rgb_raw/*.npz`. Each ZIP_STORED file retains rgb8 uint8 pixels,
source_ns, consecutive frame_index, frame_id and encoding; timestamps match the
original ROS image header. No video decoding, lossy encoding or resampling is
used. At 512×512 it is about 0.75 MiB/frame, roughly 1.8 GB for this route.
Normal completion checks acquired/published/archive/video counts; interrupted
runs must keep their partial receipt.

The new recorder uses explicit
[storage-wal.yaml](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-dual-stairs-paired/storage-wal.yaml):

```yaml
write:
  pragmas:
    - journal_mode = WAL
    - synchronous = NORMAL
    - busy_timeout = 5000
```

The `write.pragmas` format is documented by
[ROS 2 Jazzy rosbag2_storage_sqlite3](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_storage_sqlite3/README.md).
These values are the recorded experiment settings. No count query is made
against an active writer; database verification waits for normal recorder
shutdown. The standalone VIO score above is available; full-route same-input mode
comparison remains incomplete after both SDK interruptions.

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

The new default prefix is
`build_cache/dynanav_lio_lifetime_20261109T043801Z`. Its
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
uninitialized-pointer path. Both paired attempts used the same new lifetime core for both modes; they
must not silently relabel live old-core results as new-core results. The lifecycle pass does not establish
full-route visual accuracy.
