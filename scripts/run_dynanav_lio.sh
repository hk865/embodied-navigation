#!/usr/bin/env bash
# Start an isolated RGB-D or explicit RTX LiDAR/IMU observer. Never starts a simulator.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: run_dynanav_lio.sh --output-dir NEW_DIRECTORY [--binary-prefix PREFIX] [--parameter-override FILE]
       run_dynanav_lio.sh --mode livo --capture-result RESULT_JSON --output-dir NEW_DIRECTORY
       run_dynanav_lio.sh --check [--mode lio|livo] [--capture-result RESULT_JSON]
       Optional: --observer-namespace /dynanav/lio_observer   # default
       Optional: --observer-transport auto|baseline   # DYNANAV_OBSERVER_TRANSPORT or auto
       Optional: --lidar-topic /dynanav/lidar_perception_filtered
       Optional: --config-dir DIRECTORY   # default: configs/simulation/dynanav

PREFIX contains install/ros. PROJECT_DATA_ROOT can supply the verified, fixed
build-cache default. --parameter-override appends a second ROS parameter file
for estimation experiments; observer input/mode selectors stay fixed.
--observer-namespace defaults to /dynanav/lio_observer. Explicit other namespaces
isolate every native output, including /tf. --lidar-topic explicitly selects the
legacy oracle-static cloud (/dynanav/lidar_static), the online learned-filtered
temporary candidates (/dynanav/lidar_perception_filtered), or the raw static-scene
depth cloud published by the capture (/dynanav/lidar_filtered, IMU frame, no
ground-truth geometry), or real rotating RTX returns (/dynanav/lidar_rtx).
RTX automatically appends fastlivo_rtx_os1_32.yaml: official Ouster OS1 Rev6,
32 channels at 10 Hz, type 3, per-return t/ring and identity IMU extrinsics.
RTX also requires a native core exposing common.allow_ouster_lidar. Select that
build explicitly with --binary-prefix BUILD_ROOT (which contains install/ros,
not install/ros itself); the legacy default prefix is never switched silently.
Other absolute sensor inputs and estimation modes stay fixed.
--mode defaults to lio. livo additionally starts its own parameter_blackboard
in the observer namespace and reads only sensor_installation calibration from
--capture-result. It enables actual /dynanav/camera input; visual updates still
need native diagnostics/snapshots to establish that observations were used.
--dependency-prefix can select the existing ROS dependency
prefix (the directory containing lib and share). ROS_DOMAIN_ID defaults to 91.
--observer-transport auto selects the bundled 4 MiB UDP receive profile only
for LIVO or explicit RTX LiDAR with rmw_fastrtps_cpp/UDPv4 and no existing DDS
configuration. RTX delivery and socket buffers still require runtime verification.
baseline preserves the inherited environment without adding a profile. This
launcher changes only its observer process tree, never the Isaac publisher.
--check inspects files, hashes and dynamic dependencies without creating a run
directory or starting ROS, a mapper, a camera service, or a simulator.
EOF
}

binary_prefix=""
dependency_prefix=""
output_dir=""
parameter_override=""
config_dir=""
mode=lio
observer_namespace=/dynanav/lio_observer
lidar_topic=/dynanav/lidar_static
capture_result=""
camera_blackboard=/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard
observer_transport=${DYNANAV_OBSERVER_TRANSPORT:-auto}
observer_transport_source=default
[[ -z ${DYNANAV_OBSERVER_TRANSPORT:-} ]] || observer_transport_source=environment
check_only=false
while (($#)); do
    case "$1" in
        --binary-prefix|--dependency-prefix|--output-dir|--parameter-override|--config-dir|--mode|--capture-result|--camera-blackboard|--observer-namespace|--lidar-topic|--observer-transport)
            (($# >= 2)) || { printf 'Missing value for %s\n' "$1" >&2; exit 2; }
            case "$1" in
                --binary-prefix) binary_prefix=$2 ;;
                --dependency-prefix) dependency_prefix=$2 ;;
                --output-dir) output_dir=$2 ;;
                --parameter-override) parameter_override=$2 ;;
                --config-dir) config_dir=$2 ;;
                --mode) mode=$2 ;;
                --observer-transport) observer_transport=$2; observer_transport_source=cli ;;
                --observer-namespace) observer_namespace=$2 ;;
                --lidar-topic) lidar_topic=$2 ;;
                --capture-result) capture_result=$2 ;;
                --camera-blackboard) camera_blackboard=$2 ;;
            esac
            shift 2 ;;
        --check) check_only=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done
[[ $observer_namespace =~ ^(/[a-zA-Z_][a-zA-Z0-9_]*)+$ ]] || { printf 'Observer namespace must be an absolute non-root ROS namespace without trailing slash.\n' >&2; exit 2; }
# /dynanav/lidar_filtered is the online depth-derived static cloud published by the
# capture (IMU frame, no ground-truth geometry). /dynanav/lidar_rtx is a distinct
# rotating sensor with native per-return timing; never alias it to a depth cloud.
[[ $lidar_topic == /dynanav/lidar_static || $lidar_topic == /dynanav/lidar_perception_filtered || $lidar_topic == /dynanav/lidar_filtered || $lidar_topic == /dynanav/lidar_rtx ]] || { printf 'Unsupported explicit lidar input topic.\n' >&2; exit 2; }
[[ $mode == lio || $mode == livo ]] || { printf 'Mode must be lio or livo.\n' >&2; exit 2; }
if [[ $mode == livo ]]; then
    [[ -n $capture_result && -r $capture_result ]] || { printf 'LIVO requires readable --capture-result calibration.\n' >&2; exit 2; }
    capture_result=$(realpath -- "$capture_result")
elif [[ -n $capture_result ]]; then
    printf '%s\n' '--capture-result applies only to --mode livo.' >&2
    exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
repo_dir=$(CDPATH= cd -- "$script_dir/.." && pwd -P)
if [[ -n ${PROJECT_DATA_ROOT:-} ]]; then
    project_data="${PROJECT_DATA_ROOT%/}/embodied-navigation"
elif [[ -n $binary_prefix ]]; then
    project_data=$(dirname -- "$(dirname -- "$(realpath -m -- "$binary_prefix")")")
else
    printf 'Set PROJECT_DATA_ROOT or pass --binary-prefix explicitly.\n' >&2
    exit 2
fi
binary_prefix=${binary_prefix:-"$project_data/build_cache/dynanav_post_vio_20261109T052245Z"}
dependency_prefix=${dependency_prefix:-"$project_data/build_cache/native_validation_20261108T124116Z/project/install/ros"}
binary_prefix=$(realpath -m -- "$binary_prefix")
dependency_prefix=$(realpath -m -- "$dependency_prefix")
install_prefix="$binary_prefix/install/ros"
runner="$install_prefix/lib/fast_livo2_ros/fastlivo_mapping"
core="$install_prefix/lib/libfast_livo2_core.so"
config_dir=${config_dir:-"$repo_dir/configs/simulation/dynanav"}
parameters="$config_dir/fastlivo_${mode}.yaml"
camera="$config_dir/camera_rgbd.yaml"
sensor_parameters=""
if [[ $lidar_topic == /dynanav/lidar_rtx ]]; then
    sensor_parameters="$config_dir/fastlivo_rtx_os1_32.yaml"
fi
if [[ -n $parameter_override ]]; then
    [[ -f $parameter_override && -r $parameter_override ]] || { printf 'Unreadable parameter override: %s\n' "$parameter_override" >&2; exit 2; }
    parameter_override=$(realpath -- "$parameter_override")
fi
[[ -r /opt/ros/jazzy/setup.bash ]] || { printf 'Missing ROS Jazzy setup.bash\n' >&2; exit 2; }
# ROS setup scripts do not promise compatibility with nounset.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export LD_LIBRARY_PATH="$install_prefix/lib:$dependency_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$install_prefix:$dependency_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91}

preflight_status=0
preflight_json=$(python3 -B - "$runner" "$core" "$parameters" "$camera" "$dependency_prefix" "$parameter_override" "$mode" "$capture_result" "$camera_blackboard" "$observer_namespace" "$lidar_topic" "$script_dir" "$observer_transport" "$observer_transport_source" "$sensor_parameters" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import math
import yaml

runner, core, parameters, camera, dependencies = map(Path, sys.argv[1:6])
override = Path(sys.argv[6]) if sys.argv[6] else None
mode = sys.argv[7]
capture_result = Path(sys.argv[8]) if sys.argv[8] else None
blackboard = Path(sys.argv[9])
observer_namespace = sys.argv[10]
lidar_input_topic = sys.argv[11]
sys.path.insert(0, sys.argv[12])
from dynanav_observer_transport import select_transport
transport = select_transport(mode, sys.argv[13], os.environ, Path(sys.argv[12]).parent, Path.cwd(), lidar_topic=lidar_input_topic)
transport["policy_source"] = sys.argv[14]
sensor_parameters = Path(sys.argv[15]) if sys.argv[15] else None
sensor_values = (
    yaml.safe_load(sensor_parameters.read_text())["/**"]["ros__parameters"]
    if sensor_parameters is not None else {}
)
base_parameters = yaml.safe_load(parameters.read_text())["/**"]["ros__parameters"]
post_vio_enabled = base_parameters.get("publish", {}).get("post_vio_odometry_en", False)
if type(post_vio_enabled) is not bool or post_vio_enabled != (mode == "livo"):
    raise SystemExit("Observer mode and explicit post-VIO output configuration disagree")
calibration = None
calibration_parameters = {}
if mode == "livo":
    installation = json.loads(capture_result.read_text())["sensor_installation"]
    rotation = installation["camera_optical_R_from_IMU"]
    translation = installation["camera_optical_translation_from_IMU"]
    intrinsics = installation["camera_intrinsics"]
    camera_mount = installation["camera_translation_in_base"]
    imu_mount = installation["imu_translation_in_base"]
    def finite_vector(value, length):
        return (isinstance(value, list) and len(value) == length
                and all(type(x) in (int, float) and math.isfinite(x) for x in value))
    if (not isinstance(rotation, list) or len(rotation) != 3
            or not all(finite_vector(row, 3) for row in rotation)
            or not finite_vector(translation, 3) or not finite_vector(intrinsics, 4)
            or not finite_vector(camera_mount, 3) or not finite_vector(imu_mount, 3)
            or min(intrinsics[:2]) <= 0):
        raise SystemExit("Invalid capture sensor_installation camera calibration")
    import numpy as np
    R = np.array(rotation, dtype=float)
    if (not np.allclose(R.T @ R, np.eye(3), atol=1e-7, rtol=0)
            or not np.isclose(np.linalg.det(R), 1, atol=1e-7, rtol=0)
            or not np.allclose(translation, R @ (np.array(imu_mount) - camera_mount), atol=1e-7, rtol=0)):
        raise SystemExit("Capture camera transform is not rigid or disagrees with recorded mounts")
    camera_parameters = yaml.safe_load(camera.read_text())["/**"]["ros__parameters"]
    calibration_parameters = dict(camera_parameters)
    calibration_parameters.update(cam_fx=float(intrinsics[0]), cam_fy=float(intrinsics[1]), cam_cx=float(intrinsics[2]), cam_cy=float(intrinsics[3]))
    calibration_parameters["extrin_calib"] = {"Rcl": R.reshape(-1).tolist(), "Pcl": [float(x) for x in translation]}
    calibration = {"source": str(capture_result),
                   "source_sha256": hashlib.sha256(capture_result.read_bytes()).hexdigest(),
                   "selected_fields": {key: installation[key] for key in ("camera_optical_R_from_IMU", "camera_optical_translation_from_IMU", "camera_intrinsics", "camera_translation_in_base", "imu_translation_in_base")},
                   "convention": "p_camera_optical = Rcl * p_go2_imu + Pcl; cloud is already go2_imu",
                   "parameters": calibration_parameters}
override_conflicts = []
watermark_requested = False
handoff_requested = sensor_values.get("common", {}).get("same_pair_vio_handoff", False)
if override is not None:
    document = yaml.safe_load(override.read_text())
    fixed = {"common.img_en": int(mode == "livo"), "common.allow_lio_only": mode == "lio",
             "common.lidar_en": 1, "common.use_gps": False,
             "common.lid_topic": "/dynanav/lidar_static",
             "common.imu_topic": "/dynanav/imu", "use_sim_time": True,
             "publish.post_vio_odometry_en": post_vio_enabled}
    if mode == "livo":
        fixed["common.img_topic"] = "/dynanav/camera"
        fixed.update({key: value for key, value in calibration_parameters.items() if key != "extrin_calib"})
        fixed.update({"extrin_calib." + key: value for key, value in calibration_parameters["extrin_calib"].items()})

    def flatten(values, prefix=""):
        for name, value in values.items():
            key = prefix + str(name)
            if isinstance(value, dict):
                yield from flatten(value, key + ".")
            else:
                yield key, value

    def parameter_blocks(values):
        if isinstance(values, dict):
            for name, value in values.items():
                if name == "ros__parameters" and isinstance(value, dict):
                    yield value
                else:
                    yield from parameter_blocks(value)

    # A user override may tune estimation, not switch the selected physical
    # sensor back to instantaneous RGB-D or apply its static transform twice.
    fixed.update(dict(flatten(sensor_values)))
    # This scheduling optimization has an explicit rollback/comparison switch.
    # It does not change the sensor, time contract, IMU coverage or buffer limits.
    fixed.pop("common.same_pair_vio_handoff", None)
    blocks = list(parameter_blocks(document))
    if not blocks:
        raise SystemExit("Override requires a ROS ros__parameters mapping")
    for block in blocks:
        for name, value in flatten(block):
            if name == "common.complete_scan_watermark":
                if type(value) is not bool:
                    raise SystemExit("common.complete_scan_watermark must be a boolean")
                watermark_requested = value
            if name == "common.same_pair_vio_handoff":
                if type(value) is not bool:
                    raise SystemExit("common.same_pair_vio_handoff must be a boolean")
                handoff_requested = value
        override_conflicts.extend(name for name, value in flatten(block)
                                  if name in fixed and value != fixed[name])
missing = [str(path) for path in (runner, core, parameters, camera) if not path.is_file()]
if mode == "livo" and (not blackboard.is_file() or not os.access(blackboard, os.X_OK)):
    missing.append(str(blackboard) + " (camera parameter server missing/not executable)")
if runner.is_file() and not os.access(runner, os.X_OK):
    missing.append(str(runner) + " (not executable)")
unresolved = {}
for path in (runner, core):
    if path.is_file():
        result = subprocess.run(["ldd", str(path)], text=True, capture_output=True)
        failures = [line.strip() for line in result.stdout.splitlines() if "not found" in line]
        if result.returncode:
            failures.append(result.stderr.strip() or "ldd failed")
        if failures:
            unresolved[str(path)] = failures
core_bytes = core.read_bytes() if core.is_file() else b""
capability_marker = b"common.allow_lio_only" in core_bytes
post_vio_capability = b"publish.post_vio_odometry_en" in core_bytes
ouster_capability = b"common.allow_ouster_lidar" in core_bytes
capability_errors = []
if sensor_parameters is not None and not ouster_capability:
    capability_errors.append(
        "Selected native core does not support common.allow_ouster_lidar required "
        "for /dynanav/lidar_rtx. Pass --binary-prefix BUILD_ROOT for an Ouster-capable "
        "build (the directory containing install/ros, not install/ros itself). "
        "The selected prefix has not been changed."
    )
if watermark_requested and (sensor_parameters is None or mode != "livo" or not handoff_requested):
    capability_errors.append("Complete scan watermark requires explicit RTX LIVO with same-pair handoff")
requested_rtx_features = ["common.complete_scan_watermark"] if watermark_requested else []
if sensor_parameters is not None:
    if handoff_requested:
        requested_rtx_features.append("common.same_pair_vio_handoff")
    requested_rtx_features.extend(
        "common." + name for name in
        ("max_buffered_images", "max_buffered_imu", "max_buffered_prop_imu")
        if sensor_values.get("common", {}).get(name, 0) > 0
    )
missing_rtx_features = [name for name in requested_rtx_features
                        if name.encode() not in core_bytes]
if sensor_parameters is not None and ouster_capability and missing_rtx_features:
    capability_errors.append(
        "Selected Ouster core lacks requested RTX handoff/buffer parameters: "
        + ", ".join(missing_rtx_features)
        + ". Pass --binary-prefix BUILD_ROOT for the matching updated build; "
          "a YAML setting alone does not enable missing native code."
    )
post_vio_contract = ({"pose_source": "livo_post_vio", "state_stage": "post_vio",
                      "twist_available": False, "pose_covariance_available": False,
                      "unavailable_covariance_diagonal": 1e12,
                      "visual_update_on_every_image": False,
                      "publication_scope": "one final pose after an actually processed image; per-image status records whether a visual update was retained"}
                     if post_vio_enabled else None)
# The scope text must describe the topic that is actually subscribed: the raw
# depth-derived static-scene cloud is not the learned-person filtered candidate
# cloud, and neither is the oracle-static cloud. Naming them the same would make
# a receipt claim a pipeline that never ran.
lidar_input_scope = {
    "/lidar_static": "oracle-static input (ground-truth geometry; diagnostics only)",
    "/lidar_perception_filtered": "learned-person filtered temporary LIO candidates; background is not certified static",
    "/lidar_filtered": "raw depth-derived static-scene cloud from the capture; no ground-truth geometry, no learned-person filtering; background is not certified static",
    "/lidar_rtx": "official Ouster OS1 Rev6 32-channel 10 Hz RTX range returns; rotating scan with per-return time, no RGB-D backprojection or ground-truth deskew",
}.get("/" + lidar_input_topic.rsplit("/", 1)[-1], "unrecognised lidar input topic")
receipt = {
    "scope": "File/hash/dependency inspection only; no ROS node or estimator executed",
    "mode": mode,
    "observer_transport": transport,
    "observer_namespace": observer_namespace,
    "lidar_input_topic": lidar_input_topic,
    "lidar_input_scope": lidar_input_scope,
    "measurement_source": "RTX LiDAR range sensor; not RGB-D" if sensor_parameters else "RGB-D cloud; not an independent LiDAR sensor",
    "lidar_sensor_parameters": ({"path": str(sensor_parameters),
                                 "sha256": hashlib.sha256(sensor_parameters.read_bytes()).hexdigest(),
                                 "parameters": sensor_values}
                                if sensor_parameters is not None else None),
    "lidar_point_contract": ({"model": "OS1_REV6_32ch10hz1024res", "lidar_type": 3,
                              "frame_id": "go2_imu", "scan_line": 32, "scan_hz": 10.0,
                              "t": "uint32 nanoseconds relative to scan-start header", "ring": "uint8",
                              "sort": "native non-feature preprocessor sorts by t before scan-end calculation",
                              "ground_truth_deskew": False,
                              "capture_render_backend_required": "fabric",
                              "capture_physics_fabric_required": True,
                              "mount_updates_verified": False,
                              "runtime_data_verified": False}
                             if sensor_parameters is not None else None),
    "tf_output_topic": "/tf" if observer_namespace == "/dynanav/lio_observer" else observer_namespace + "/tf",
    "post_vio_odometry_enabled": post_vio_enabled,
    "post_vio_output_capability_present": post_vio_capability,
    "navigation_pose_topic": observer_namespace + ("/post_vio_odometry" if post_vio_enabled else "/aft_mapped_to_init"),
    "navigation_pose_source": "livo_post_vio" if post_vio_enabled else "lio_estimate",
    "post_vio_status_topic": observer_namespace + "/post_vio_odometry_status" if post_vio_enabled else None,
    "post_vio_output_contract": post_vio_contract,
    "parameter_file": str(parameters),
    "camera_calibration": calibration,
    "camera_blackboard": str(blackboard) if mode == "livo" else None,
    "camera_parameter_service": observer_namespace + "/parameter_blackboard/get_parameters" if mode == "livo" else None,
    "ready": not missing and not unresolved and capability_marker and not override_conflicts
             and (not post_vio_enabled or post_vio_capability) and not capability_errors,
    "ouster_opt_in_parameter_present": ouster_capability,
    "capability_errors": capability_errors,
    "observer_contract_conflicts": override_conflicts,
    "rtx_handoff_requested": handoff_requested,
    "complete_scan_watermark_requested": watermark_requested,
    "subscribed_lidar_topic": lidar_input_topic + "/complete_scan" if watermark_requested else lidar_input_topic,
    "requested_rtx_features": requested_rtx_features,
    "missing_rtx_features": missing_rtx_features,
    "parameter_override": ({"path": str(override),
                             "sha256": hashlib.sha256(override.read_bytes()).hexdigest()}
                            if override is not None else None),
    "missing": missing, "unresolved": unresolved,
    "lio_opt_in_parameter_present": capability_marker,
    "runner": str(runner), "core": str(core), "dependency_prefix": str(dependencies),
    "sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in (runner, core) if p.is_file()},
}
print(json.dumps(receipt, indent=2))
if not receipt["ready"]:
    for error in capability_errors:
        print(error, file=sys.stderr)
    raise SystemExit(2)
PY
) || preflight_status=$?
printf '%s\n' "$preflight_json"
((preflight_status == 0)) || exit "$preflight_status"
if $check_only; then
    exit 0
fi
[[ -n $output_dir ]] || { printf 'A new --output-dir is required.\n' >&2; exit 2; }
[[ ! -e $output_dir && ! -L $output_dir ]] || { printf 'Output already exists: %s\n' "$output_dir" >&2; exit 2; }
output_dir=$(realpath -m -- "$output_dir")
project_data=$(realpath -m -- "$project_data")
case "$output_dir/" in
    "$project_data/"*) ;;
    *) printf 'Output must be under project data root: %s\n' "$project_data" >&2; exit 2 ;;
esac
mkdir -p -- "$(dirname -- "$output_dir")"
mkdir -- "$output_dir"
# 只在本启动器内导出，且位于只读 --check 返回之后。
# 不要把此 profile 导出到 supervisor 或 publisher 的环境。
selected_profile=$(python3 -c 'import json,sys; print(json.loads(sys.argv[1])["observer_transport"]["profile"] or "")' "$preflight_json")
if [[ -n $selected_profile ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$selected_profile"
fi
export DEMO_RUN_DIR="$output_dir"
export ROS_LOG_DIR="$output_dir/ros_logs"
# The native per-point binary has no total disk bound. Normal launches retain
# ROS/status logs without enabling this unbounded diagnostic stream.
unset FASTLIVO_DIAGNOSTIC_DIR
export FASTLIVO_PIPELINE_MODE=serial
export FASTLIVO_SERIAL_IMU_PRIORITY=0
mkdir -- "$ROS_LOG_DIR"
printf '%s\n' "$preflight_json" >"$output_dir/preflight.json"

command=("$runner" --ros-args --params-file "$parameters")
calibration_file=""
if [[ $mode == livo ]]; then
    calibration_file="$output_dir/capture_calibration.yaml"
    python3 - "$calibration_file" "$preflight_json" <<'PY'
import json
from pathlib import Path
import sys
import yaml

calibration = json.loads(sys.argv[2])["camera_calibration"]["parameters"]
Path(sys.argv[1]).write_text(yaml.safe_dump({"/**": {"ros__parameters": calibration}}, sort_keys=False))
PY
    command+=(--params-file "$camera")
fi
if [[ -n $sensor_parameters ]]; then
    command+=(--params-file "$sensor_parameters")
fi
if [[ -n $parameter_override ]]; then
    command+=(--params-file "$parameter_override")
fi
if [[ $mode == livo ]]; then
    command+=(--params-file "$calibration_file")
fi
command+=(-r "__node:=dynanav_${mode}_observer" -r "__ns:=$observer_namespace")
# Keep every native absolute output in the observer namespace, including the
# MAVROS pose output. Explicit --slam-reference may consume isolated odometry.
for topic in cloud_registered_full cloud_registered visualization_marker cloud_visual_sub_map_before \
             cloud_effected Laser_map aft_mapped_to_init path planner_normal voxels dyn_obj \
             dyn_obj_removed dyn_obj_dbg_hist mavros/vision_pose/pose rgb_img LIVO2/imu_propagate planes; do
    command+=(-r "/$topic:=$observer_namespace/$topic")
done
if [[ $lidar_topic != /dynanav/lidar_static ]]; then
    command+=(-r "/dynanav/lidar_static:=$lidar_topic")
fi
if [[ $mode == livo ]]; then
    command+=(-r "/post_vio_odometry:=$observer_namespace/post_vio_odometry"
              -r "/post_vio_odometry_status:=$observer_namespace/post_vio_odometry_status")
fi
if [[ $observer_namespace != /dynanav/lio_observer ]]; then
    command+=(-r "/tf:=$observer_namespace/tf")
fi
blackboard_command=("$camera_blackboard" --ros-args --params-file "$camera")
if [[ $mode == livo ]]; then
    blackboard_command+=(--params-file "$calibration_file" -r __node:=parameter_blackboard -r "__ns:=$observer_namespace")
fi
python3 - "$output_dir" "$runner" "$core" "$parameters" "$dependency_prefix" "$parameter_override" "$mode" "$preflight_json" "$camera" "$calibration_file" "${command[@]}" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys

output, runner, core, parameters, dependencies = map(Path, sys.argv[1:6])
override = Path(sys.argv[6]) if sys.argv[6] else None
mode = sys.argv[7]
preflight = json.loads(sys.argv[8])
camera = Path(sys.argv[9])
calibration_file = Path(sys.argv[10]) if sys.argv[10] else None
receipt = {
    "schema": "dynanav_lio_observer_launch/v1",
    "observer_transport": preflight["observer_transport"],
    "command": sys.argv[11:], "ros_domain_id": os.environ["ROS_DOMAIN_ID"],
    "runner": str(runner), "core": str(core), "dependency_prefix": str(dependencies),
    "runner_sha256": hashlib.sha256(runner.read_bytes()).hexdigest(),
    "core_sha256": hashlib.sha256(core.read_bytes()).hexdigest(),
    "parameter_sha256": hashlib.sha256(parameters.read_bytes()).hexdigest(),
    "parameter_override": ({"path": str(override),
                             "sha256": hashlib.sha256(override.read_bytes()).hexdigest()}
                            if override is not None else None),
    "lidar_input_topic": preflight["lidar_input_topic"],
    "lidar_input_scope": preflight["lidar_input_scope"],
    "complete_scan_watermark_requested": preflight["complete_scan_watermark_requested"],
    "subscribed_lidar_topic": preflight["subscribed_lidar_topic"],
    "sensor_inputs": [preflight["subscribed_lidar_topic"], "/dynanav/imu"] + (["/dynanav/camera"] if mode == "livo" else []),
    "clock_input": "/clock", "ground_truth_input": False,
    "navigation_command_output": False,
    "mode": "explicit_opt_in_ONLY_LIO" if mode == "lio" else "explicit_RGB_assisted_LIVO",
    "camera_used": mode == "livo", "VIO_dynamic_mask_implemented": False,
    "camera_usage_scope": "RGB subscription enabled; actual retained visual EKF updates require diagnostics/snapshot evidence" if mode == "livo" else "Image subscription disabled",
    "visual_update_verified": False,
    "camera_calibration": preflight["camera_calibration"],
    "camera_parameter_service": preflight["camera_parameter_service"],
    "observer_namespace": preflight["observer_namespace"],
    "tf_output_topic": preflight["tf_output_topic"],
    "post_vio_odometry_enabled": preflight["post_vio_odometry_enabled"],
    "post_vio_output_capability_present": preflight["post_vio_output_capability_present"],
    "navigation_pose_topic": preflight["navigation_pose_topic"],
    "navigation_pose_source": preflight["navigation_pose_source"],
    "post_vio_status_topic": preflight["post_vio_status_topic"],
    "post_vio_output_contract": preflight["post_vio_output_contract"],
    "camera_config_sha256": hashlib.sha256(camera.read_bytes()).hexdigest() if mode == "livo" else None,
    "capture_calibration_sha256": hashlib.sha256(calibration_file.read_bytes()).hexdigest() if calibration_file else None,
    "visual_parameters_source": "native defaults explicitly selected as experiment settings; no accuracy acceptance claim" if mode == "livo" else None,
    "measurement_source": preflight["measurement_source"],
    "lidar_sensor_parameters": preflight["lidar_sensor_parameters"],
    "lidar_point_contract": preflight["lidar_point_contract"],
    "ouster_opt_in_parameter_present": preflight["ouster_opt_in_parameter_present"],
    "scope": "Launch receipt, not evidence of estimator initialization or route completion",
}
(output / "launch.json").write_text(json.dumps(receipt, indent=2) + "\n")
PY
printf 'Starting %s observer; log: %s/runner.log\n' "$mode" "$output_dir" >&2

# Preserve legacy RGB-D LIO lifecycle. RTX LIO uses the same owned-process
# readiness handshake as LIVO so capture can wait for actual subscriptions.
if [[ $mode == lio && $lidar_topic != /dynanav/lidar_rtx ]]; then
    exec "${command[@]}" >"$output_dir/runner.log" 2>&1
fi

# Only PIDs created below belong to this launcher. Never use a process-name kill.
blackboard_pid=""
mapper_pid=""
cleanup_owned_children() {
    local status=$?
    trap - EXIT
    # Further cancellation must not abort cleanup or the terminal receipt.
    trap '' INT TERM
    for child in "$mapper_pid" "$blackboard_pid"; do
        [[ -n $child ]] || continue
        kill -TERM "$child" 2>/dev/null || true
    done
    local deadline=$((SECONDS + 5))
    while (( SECONDS < deadline )); do
        local alive=false
        for child in "$mapper_pid" "$blackboard_pid"; do
            [[ -n $child ]] || continue
            if kill -0 "$child" 2>/dev/null; then alive=true; fi
        done
        $alive || break
        sleep 0.1
    done
    for child in "$mapper_pid" "$blackboard_pid"; do
        [[ -n $child ]] || continue
        kill -KILL "$child" 2>/dev/null || true
        wait "$child" 2>/dev/null || true
    done
    python3 - "$output_dir" "$status" "$mapper_pid" "$blackboard_pid" <<'PYRECEIPT'
import json
from pathlib import Path
import sys
output = Path(sys.argv[1])
receipt = {"state": "stopped", "exit_code": int(sys.argv[2]),
           "mapper_pid": int(sys.argv[3]) if sys.argv[3] else None,
           "blackboard_pid": int(sys.argv[4]) if sys.argv[4] else None}
(output / "ready.json").unlink(missing_ok=True)
temporary = output / "shutdown.json.tmp"
temporary.write_text(json.dumps(receipt, indent=2) + "\n")
temporary.replace(output / "shutdown.json")
PYRECEIPT
    exit "$status"
}
trap cleanup_owned_children EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ $mode == livo ]]; then
"${blackboard_command[@]}" >"$output_dir/camera_parameters.log" 2>&1 &
blackboard_pid=$!
printf 'Waiting up to 10s for owned camera parameter service.\n' >&2
python3 - "$output_dir/camera_parameter_ready.json" "$preflight_json" <<'PY'
import json
from pathlib import Path
import sys
import rclpy
from rcl_interfaces.srv import GetParameters

preflight = json.loads(sys.argv[2])
expected = preflight["camera_calibration"]["parameters"]
service = preflight["camera_parameter_service"]
names = ["cam_model", "cam_width", "cam_height", "scale", "cam_fx", "cam_fy", "cam_cx", "cam_cy"]
rclpy.init(args=[])
node = rclpy.create_node("dynanav_owned_camera_parameter_check")
try:
    client = node.create_client(GetParameters, service)
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeError("camera parameter service not ready within 8s: " + service)
    request = GetParameters.Request()
    request.names = names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.exception() is not None:
        raise RuntimeError("camera parameter request failed/timed out")
    values = future.result().values
    actual = {}
    for name, value in zip(names, values, strict=True):
        fields = {2: "integer_value", 3: "double_value", 4: "string_value"}
        if value.type not in fields:
            raise RuntimeError("camera parameter has unexpected type: " + name)
        actual[name] = getattr(value, fields[value.type])
        if actual[name] != expected[name]:
            raise RuntimeError("camera parameter value mismatch: " + name)
    Path(sys.argv[1]).write_text(json.dumps({"service": service, "verified_parameters": actual}, indent=2) + "\n")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
kill -0 "$blackboard_pid" 2>/dev/null || { printf 'Owned camera parameter server exited.\n' >&2; exit 1; }
fi
printf 'Starting mapper and verifying sensor subscriptions.\n' >&2
"${command[@]}" >"$output_dir/runner.log" 2>&1 &
mapper_pid=$!
# A provenance receipt is deliberately written before startup. Only this
# atomic handshake, after the owned mapper exposes its input subscriptions,
# allows the supervisor to start capture. No sensor frames are needed here.
python3 - "$output_dir" "$preflight_json" "$mapper_pid" "$blackboard_pid" <<'PYREADY'
import json
import os
from pathlib import Path
import sys
import time
import rclpy
from rclpy.node import NodeNameNonExistentError

output = Path(sys.argv[1])
preflight = json.loads(sys.argv[2])
pids = [int(pid) for pid in sys.argv[3:] if pid]
namespace = preflight["observer_namespace"]
name = "dynanav_" + preflight["mode"] + "_observer"
complete_scan_mode = preflight.get("complete_scan_watermark_requested", False)
if complete_scan_mode and not preflight.get("subscribed_lidar_topic"):
    raise SystemExit("complete scan readiness requires explicit subscribed_lidar_topic")
# Off mode deliberately keeps the original raw-topic lookup, including legacy
# internal preflight fixtures with no certified-subscription metadata.
lidar_subscription = (preflight["subscribed_lidar_topic"] if complete_scan_mode
                      else preflight["lidar_input_topic"])
expected = {lidar_subscription, "/dynanav/imu"}
if preflight["mode"] == "livo":
    expected.add("/dynanav/camera")
rclpy.init(args=[])
node = rclpy.create_node("dynanav_observer_readiness_check")
try:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        for index, pid in enumerate(pids):
            role = "mapper" if index == 0 else "camera parameter server"
            log = "runner.log" if index == 0 else "camera_parameters.log"
            exited = f"owned {role} exited before sensor readiness (pid={pid}); see {output / log}"
            try:
                os.kill(pid, 0)
                # kill(0) also succeeds for a child waiting to be reaped; the
                # process can disappear between this call and the /proc read.
                stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            except (ProcessLookupError, FileNotFoundError):
                raise RuntimeError(exited) from None
            if stat[0] in ("Z", "X"):
                raise RuntimeError(exited)
        try:
            subscription_types = dict(node.get_subscriber_names_and_types_by_node(name, namespace))
        except NodeNameNonExistentError:
            subscription_types = {}
        subscriptions = set(subscription_types)
        # Certified ingress must expose its actual typed DDS subscription. Raw
        # mode keeps the original topic-set check with no new type requirement.
        typed_ready = (not complete_scan_mode
                       or set(subscription_types.get(lidar_subscription, []))
                       == {"dynanav_sensor_msgs/msg/CompleteLidarScan"})
        if expected <= subscriptions and typed_ready:
            receipt = {"state": "ready", "mapper_pid": pids[0],
                       "blackboard_pid": pids[1] if len(pids) > 1 else None,
                       "verified_sensor_subscriptions": sorted(expected),
                       "scope": "input subscriptions available; estimator not initialized"}
            temporary = output / "ready.json.tmp"
            temporary.write_text(json.dumps(receipt, indent=2) + "\n")
            temporary.replace(output / "ready.json")
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise RuntimeError("mapper input subscriptions not ready within 10s")
except RuntimeError as error:
    raise SystemExit(str(error)) from None
finally:
    node.destroy_node()
    rclpy.shutdown()
PYREADY
status=0
if [[ -n $blackboard_pid ]]; then
    # Either owned process dying invalidates the observer, including exit 0.
    wait -n "$mapper_pid" "$blackboard_pid" || status=$?
else
    wait "$mapper_pid" || status=$?
fi
((status != 0)) || status=1
exit "$status"
