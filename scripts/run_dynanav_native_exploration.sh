#!/usr/bin/env bash
# Live RTX + post-VIO + measured PCT global / native SCAN local planning.
# The operator supplies sparse goals; RoboGauge still executes body commands.
set -Eeo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
set -a; source .env.local; set +a
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-219}"
# Fast DDS UDP ports exceed uint16 above domain 232 with the default port map.
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ && "$ROS_DOMAIN_ID" -le 232 ]] || {
    echo "ROS_DOMAIN_ID must be within 0..232 for this Fast DDS port map"; exit 2;
}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export PYTHONDONTWRITEBYTECODE=1 DYNANAV_RTX_COMPLETE_SCAN=1
BUILD="${BUILD_PREFIX:-$PROJECT_DATA_ROOT/embodied-navigation/build_cache/dynanav_complete_scan_20261110T231145Z}"
RUN="${1:-$PROJECT_DATA_ROOT/embodied-navigation/experiments/native_exploration_$(date -u +%Y%m%dT%H%M%SZ)}"
CAL="${CAPTURE_CALIBRATION:-$PROJECT_DATA_ROOT/embodied-navigation/experiments/rtx_lidar_integration_20261110T213000Z/capture14/result.json}"
SCENE="${SIMULATION_SCENE:-$PROJECT_DATA_ROOT/embodied-navigation/build_cache/offline_office_freeze_20261111T030623Z/office_deinstanced_flattened.usdc}"
DURATION="${DURATION_SIM_S:-300}"
SPEED="${SPEED_MPS:-1.0}"
NAVIGATION_BACKEND="${NAVIGATION_BACKEND:-pct_scan}"
SCAN_PREFIX="${SCAN_PREFIX:-$PROJECT_DATA_ROOT/embodied-navigation/build_cache/dynanav_scan_native/install}"
# Nominal Go2 base-support height plus the recorded go2_imu mount z. This is
# a robot offset, not a measured floor elevation or a scene-map input.
PCT_BODY_SUPPORT_OFFSET_M="${PCT_BODY_SUPPORT_OFFSET_M:-0.3148910446837544}"
# The robot's RTX blind ring is modelled only from this generation's surrounding
# measured floor. Provenance labels inferred support; it is not observed FREE.
PCT_START_BLIND_RADIUS_M="${PCT_START_BLIND_RADIUS_M:-1.5}"
[[ "$NAVIGATION_BACKEND" == pct_scan || "$NAVIGATION_BACKEND" == native_route ]] || {
    echo "NAVIGATION_BACKEND must be pct_scan or native_route"; exit 2;
}
# Keep the finite renderer and measured-map wall budgets aligned with the
# requested simulation duration. The default 300 s still receives 900 wall s.
WALL_TIMEOUT_S="$(python3 - "$DURATION" <<'PY_BUDGET'
import math
import sys

duration = float(sys.argv[1])
if not math.isfinite(duration) or not 8 <= duration <= 1800:
    raise SystemExit("Native exploration duration must be finite and within 8..1800 s")
print(max(900, math.ceil(3 * duration)))
PY_BUDGET
)"
[[ ! -e "$RUN" ]] || { echo "Use a new run directory"; exit 2; }
mkdir -p "$RUN"
exec 9>/tmp/embodied-navigation-isaac-exclusive.lock
flock -n 9 || { echo "An experiment already owns the simulator"; exit 3; }
if pgrep -f '^(/home/hyh001/isaacsim/kit/python/bin/python3|/home/hyh001/isaacsim/kit/kit|.*/fastlivo_mapping)' >/dev/null; then
    echo "A simulator or estimator is already running; left it unchanged"; exit 4
fi
export PYTHONPATH="$BUILD/install/ros/lib/python3.12/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$BUILD/install/ros/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$BUILD/install/ros${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
source "$BUILD/capture_message_env.sh"
PIDS=(); CAP=""
cleanup() {
    trap - EXIT
    for pid in "${PIDS[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
    printf 'cleanup %s\n' "$(date -u +%FT%T.%NZ)" >> "$RUN/lifecycle.log"
}
trap cleanup EXIT INT TERM
if [[ "$NAVIGATION_BACKEND" == pct_scan ]]; then
    /usr/bin/python3 scripts/run_dynanav_pct_scan.py --run-dir "$RUN/capture" \
        --scan-prefix "$SCAN_PREFIX" --speed "$SPEED" --build-scan --build-only \
        > "$RUN/planner_build.log" 2>&1
    set +u
    source "$SCAN_PREFIX/setup.bash"
    set -u
fi
printf '%s\n' "$RUN" > /tmp/embodied-navigation-native-exploration-current
# The same override file also enables the bounded native keyframe export that
# the SLAM backend node consumes (and deletes) below.
printf '/**:\n  ros__parameters:\n    common:\n      same_pair_vio_handoff: true\n      complete_scan_watermark: true\n    backend.snapshot_directory: "%s"\n' \
    "$RUN/observer/native_keyframes" > "$RUN/handoff_parameters.yaml"
bash scripts/run_dynanav_lio.sh --binary-prefix "$BUILD" --mode livo \
    --capture-result "$CAL" --observer-namespace /dynanav/rtx_livo \
    --output-dir "$RUN/observer" --lidar-topic /dynanav/lidar_rtx \
    --parameter-override "$RUN/handoff_parameters.yaml" > "$RUN/observer.log" 2>&1 &
OBS=$!; PIDS+=("$OBS")
for _ in $(seq 1 200); do
    [[ -f "$RUN/observer/ready.json" ]] && break
    kill -0 "$OBS" 2>/dev/null || { echo "Estimator exited"; exit 5; }
    sleep .1
done
[[ -f "$RUN/observer/ready.json" ]] || { echo "Estimator startup timed out"; exit 6; }
# Point-cloud SLAM backend: loop closure / prior-map relocalization, publishes
# map->camera_init and /dynanav/backend/corrected_odometry. Navigation below
# still consumes the raw odometry. PRIOR_MAP is an optional saved map directory.
BACKEND_ARGS=(--spool "$RUN/observer/native_keyframes" --namespace /dynanav/backend
    --odometry-topic /dynanav/rtx_livo/post_vio_odometry --save-map "$RUN/map")
if [[ -n "${PRIOR_MAP:-}" ]]; then
    BACKEND_ARGS+=(--prior-map "$PRIOR_MAP")
fi
python3 -m embodied_navigation.localization_mapping.backend.node "${BACKEND_ARGS[@]}" \
    > "$RUN/backend.log" 2>&1 &
BACKEND=$!; PIDS+=("$BACKEND")
# This process renders the world as sensor physics only. No scene geometry or
# simulator truth is passed to the waypoint chooser or route follower.
timeout --signal=TERM --kill-after=20s "$WALL_TIMEOUT_S" /home/hyh001/isaacsim/python.sh \
    scripts/run_dynanav_experiment.py --output "$RUN/capture" --scene "$SCENE" \
    --point-cloud-source rtx_lidar --semantic-provider none --interactive \
    --interactive-pose-source livo_post_vio --interactive-controller "$NAVIGATION_BACKEND" \
    --odometry-topic /dynanav/rtx_livo/post_vio_odometry --render-backend fabric \
    --spawn-yaw "${SPAWN_YAW_RAD:-0}" --speed "$SPEED" --duration "$DURATION" \
    > "$RUN/capture.log" 2>&1 &
CAP=$!; PIDS=("$CAP" "${PIDS[@]}")
printf '%s\n' "${PIDS[@]}" > "$RUN/owned_pids.txt"
for _ in $(seq 1 900); do
    [[ -d "$RUN/capture/interactive" ]] && break
    if ! kill -0 "$CAP" 2>/dev/null; then
        set +e; wait "$CAP"; CODE=$?; set -e
        echo "$CODE" > "$RUN/capture_exit.txt"; exit "$CODE"
    fi
    sleep .1
done
if [[ "$NAVIGATION_BACKEND" == pct_scan ]]; then
    /usr/bin/python3 scripts/run_dynanav_pct_scan.py --run-dir "$RUN/capture" \
        --scan-prefix "$SCAN_PREFIX" --speed "$SPEED" --body-support-offset "$PCT_BODY_SUPPORT_OFFSET_M" --pct-start-blind-radius "$PCT_START_BLIND_RADIUS_M" \
        > "$RUN/planner.log" 2>&1 &
    PLANNER=$!; PIDS+=("$PLANNER")
fi
python3 scripts/explore_waypoint_client.py map --run-dir "$RUN/capture" \
    --cloud-topic /dynanav/rtx_livo/cloud_registered_full \
    --odometry-topic /dynanav/rtx_livo/aft_mapped_to_init \
    --pose-source lio_estimate --cloud-semantics lio_deskewed_map \
    --imu-mount 0.020153064280748367 0 -0.005108955316245556 \
    --extrinsic-translation 0.020153064280748367 0 -0.005108955316245556 \
    --extrinsic-quaternion 0 0 0 1 --sensor-frame go2_imu --base-frame go2_base \
    --binary-prefix "$BUILD" --duration "$WALL_TIMEOUT_S" --snapshot-interval 1 \
    --export-grid "$RUN/operator_grid" > "$RUN/map.log" 2>&1 &
MAP=$!; PIDS+=("$MAP")
printf '%s\n' "${PIDS[@]}" > "$RUN/owned_pids.txt"
echo "Live exploration: $RUN"
set +e; wait "$CAP"; CODE=$?; set -e
echo "$CODE" > "$RUN/capture_exit.txt"
printf 'capture_exit %s %s\n' "$CODE" "$(date -u +%FT%T.%NZ)" >> "$RUN/lifecycle.log"
exit "$CODE"
