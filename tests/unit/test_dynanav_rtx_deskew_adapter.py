"""CPU tests for the deskewed map-frame LiDAR adapter (no ROS, no simulation)."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import dynanav_rtx_deskew_adapter as adapter  # legacy helper, not a production import

MAP_FRAME = "dynanav_lio_map"
MOUNT = [0.020153064280748367, 0.0, -0.005108955316245556]


def ring(radius, count=400, z=0.0):
    angles = np.linspace(-math.pi, math.pi, count, endpoint=False)
    return np.stack([radius * np.cos(angles), radius * np.sin(angles), np.full(count, z)], axis=1)


def walls(radius=2.0, count=400, z_low=0.1, z_high=1.0):
    rows = []
    for z in np.linspace(z_low, z_high, 6):
        rows.append(ring(radius, count=count, z=z))
    return np.vstack(rows)


def test_the_imu_is_at_plus_x_mount_in_the_base_frame():
    # Physical counterexample: the IMU origin sits at base +x = 0.02, so a point
    # at the IMU origin becomes +0.02 in the base frame (+mount, not -mount).
    base = adapter.map_cloud_to_base(np.array([[0.0, 0.0, 0.0]]), [0.0, 0.0, 0.0],
                                     [0.0, 0.0, 0.0, 1.0], MOUNT)
    assert base[0, 0] == pytest.approx(MOUNT[0], abs=1e-12)
    assert base[0, 2] == pytest.approx(MOUNT[2], abs=1e-12)


def test_identity_pose_roundtrip_and_mount_sign():
    points = ring(2.0)
    base = adapter.map_cloud_to_base(points, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], MOUNT)
    assert np.allclose(base, points + np.asarray(MOUNT), atol=1e-12)


def test_full_rpy_and_translation_roundtrip():
    yaw, pitch, roll = 0.7, -0.2, 0.35
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    quat = [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]
    position = [1.5, -0.8, 0.42]
    points = np.array([[2.0, 1.0, 0.5], [-1.0, 0.3, -0.2]])
    base = adapter.map_cloud_to_base(points, position, quat, MOUNT)
    x, y, z, w = quat
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    back = (base - np.asarray(MOUNT)) @ rotation.T + np.asarray(position)
    assert np.allclose(back, points, atol=1e-12)


def test_a_raw_rotating_scan_or_unsupported_topic_is_rejected():
    with pytest.raises(adapter.AdapterError):
        adapter.validate_cloud("/dynanav/lidar_rtx", "go2_imu", MAP_FRAME, deskewed=True, mode="LIO")
    with pytest.raises(adapter.AdapterError):
        adapter.validate_cloud("/cloud_registered", MAP_FRAME, MAP_FRAME, deskewed=False, mode="LIO")
    with pytest.raises(adapter.AdapterError):
        adapter.validate_cloud("/cloud_registered", "go2_imu", MAP_FRAME, deskewed=True, mode="LIO")


def test_the_same_pair_contract_requires_matching_integer_stamps_and_stage():
    good = {"stage": "LIO", "reference_stamp_ns": 10_000_000_000,
            "cloud_stamp_ns": 10_000_000_000, "pose_stamp_ns": 10_000_000_000,
            "pair_id": "lio-42", "binary_sha256": "a" * 64, "deskewed": True}
    assert adapter.validate_same_pair_contract(good)["stage"] == "LIO"
    near = dict(good, pose_stamp_ns=10_050_000_000)
    with pytest.raises(adapter.AdapterError):
        adapter.validate_same_pair_contract(near)
    other_pair = dict(good, pair_id="")
    with pytest.raises(adapter.AdapterError):
        adapter.validate_same_pair_contract(other_pair)
    with pytest.raises(adapter.AdapterError):
        adapter.validate_same_pair_contract(dict(good, stage="RAW"))
    with pytest.raises(adapter.AdapterError):
        adapter.validate_same_pair_contract(dict(good, binary_sha256="short"))


def test_pairing_window_still_rejects_near_but_different_times():
    assert adapter.validate_pairing(10.0, 10.05) == pytest.approx(0.05)
    with pytest.raises(adapter.AdapterError):
        adapter.validate_pairing(10.0, 10.11)


def test_floor_points_are_not_obstacles():
    floor = ring(2.0, z=-0.35)
    evidence = adapter.direction_evidence(floor)
    for name in adapter.DIRECTIONS:
        assert evidence[name]["points_collision"] == 0
        assert evidence[name]["observation_state"] == "insufficient_observation"
        assert evidence[name]["floor_observation_state"] == "supported"


def corridor_coverage(per_bin=8):
    """Collision-height points in every range bin of every direction sector."""
    rows = []
    for centre in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        for radius in (0.4, 1.1, 1.9, 2.7):
            for offset in np.linspace(-0.25, 0.25, per_bin):
                angle = centre + offset
                rows.append([radius * math.cos(angle), radius * math.sin(angle), 0.1])
    return np.asarray(rows)


def test_corridor_points_cover_every_direction_and_range_bin():
    evidence = adapter.direction_evidence(corridor_coverage(), corridor_bins=4, min_points_per_bin=5)
    for name in adapter.DIRECTIONS:
        assert evidence[name]["observation_state"] == "measured"
        assert evidence[name]["clearance_m"] == pytest.approx(0.4, abs=0.05)
        assert all(count >= 5 for count in evidence[name]["bin_points"])


def test_a_dense_narrow_beam_does_not_cover_a_direction():
    angles = np.zeros(400)
    beam = np.stack([2.0 * np.cos(angles), 2.0 * np.sin(angles), np.linspace(0.1, 1.0, 400)], axis=1)
    evidence = adapter.direction_evidence(beam, corridor_bins=4, min_points_per_bin=5)
    assert evidence["front"]["observation_state"] == "insufficient_observation"


def test_a_missing_corridor_bin_is_unknown():
    wall = walls()
    front_only = wall[(wall[:, 0] < 1.5)]
    evidence = adapter.direction_evidence(front_only, corridor_bins=4, min_points_per_bin=5)
    assert evidence["front"]["observation_state"] == "insufficient_observation"


def test_near_front_obstacle_is_the_measured_clearance():
    obstacle = np.vstack([walls(), np.stack([np.linspace(0.6, 0.62, 100), np.zeros(100),
                                             np.full(100, 0.3)], axis=1)])
    evidence = adapter.direction_evidence(obstacle, corridor_bins=4, min_points_per_bin=5)
    assert evidence["front"]["clearance_m"] == pytest.approx(0.6, abs=0.05)


def test_stale_or_future_clouds_never_become_perception():
    evidence = {"front": {"observation_state": "measured", "clearance_m": 2.0,
                          "floor_observation_state": "supported"}}
    for stamp, now in ((10.0, 10.5), (10.5, 10.0), (float("nan"), 10.0)):
        with pytest.raises(adapter.AdapterError):
            adapter.build_perception(direction_evidence_map=evidence, stamp_s=stamp,
                                     now_sim_s=now, max_sim_age_s=0.25)


def test_perception_flags_unknown_directions_and_floor_support():
    evidence = {name: {"observation_state": "measured", "clearance_m": 2.0,
                       "floor_observation_state": "supported"} for name in adapter.DIRECTIONS}
    evidence["left"] = {"observation_state": "insufficient_observation",
                        "clearance_m": None, "floor_observation_state": "unsupported"}
    payload = adapter.build_perception(direction_evidence_map=evidence, stamp_s=10.0,
                                       now_sim_s=10.05, max_sim_age_s=0.25)
    assert payload["valid"] is False
    assert payload["unknown_directions"] == ["left"]
    assert payload["depth_valid_ratio"] is None


def test_the_operator_map_input_rejects_raw_and_mismatched_pairs():
    points = ring(2.0)
    record = adapter.build_operator_map_input(
        points, source_topic="/cloud_registered", header_frame=MAP_FRAME, odom_frame=MAP_FRAME,
        mode="LIO", deskewed=True, cloud_stamp_s=10.0, pose_position=[0.0, 0.0, 0.4],
        pose_quat_xyzw=[0.0, 0.0, 0.0, 1.0], pose_stamp_s=10.05, imu_mount_xyz=MOUNT,
    )
    assert record["provenance"]["source"] == "fastlivo2_deskewed_cloud_registered"
    assert record["provenance"]["deskewed"] is True
    with pytest.raises(adapter.AdapterError):
        adapter.build_operator_map_input(
            points, source_topic="/dynanav/lidar_rtx", header_frame=MAP_FRAME,
            odom_frame=MAP_FRAME, mode="LIO", deskewed=True, cloud_stamp_s=10.0,
            pose_position=[0.0, 0.0, 0.4], pose_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            pose_stamp_s=10.05, imu_mount_xyz=MOUNT)
