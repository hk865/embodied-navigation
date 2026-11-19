"""CPU unit tests for 200 Hz simulation-truth deskew and isolation contracts."""

from __future__ import annotations

import contextlib
import io
import json
import math
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

from embodied_navigation.common.geometry import quaternion_matrix
from embodied_navigation.common.state_contract import FORBIDDEN_POSE_SOURCES
from embodied_navigation.robot.unitree.policy.native.observation import (
    quaternion_rotation,
)
from embodied_navigation.simulation.dynanav import experiment as exp_module
from embodied_navigation.simulation.dynanav import pct_scan_worker
from embodied_navigation.simulation.dynanav import rtx_deskewed_perception as perception
from embodied_navigation.simulation.dynanav.capture.lidar import (
    LIDAR_MOUNT,
    LidarPacket,
    OUSTER_DTYPE,
    pack_lidar_scan,
)
from embodied_navigation.simulation.dynanav.pct_scan_waypoints import (
    PctScanWaypointTracker,
    atomic_json,
)
from embodied_navigation.simulation.dynanav.truth_deskew import (
    TRUTH_CHILD_FRAME_ID,
    TRUTH_DESKEWED_CLOUD_TOPIC,
    TRUTH_FRAME_ID,
    TRUTH_LIO_ODOM_TOPIC,
    TRUTH_POSE_SOURCE,
    TRUTH_VIO_ODOM_TOPIC,
    TruthPoseBuffer,
    deskew_rtx_packet_to_world,
    truth_deskewed_cloud_message,
    truth_odometry_message,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import explore_waypoint_client as client  # noqa: E402


def _quat_from_axis_angle_wxyz(axis, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * float(angle_rad)
    return np.array(
        [math.cos(half), *(math.sin(half) * axis)], dtype=np.float64
    )


def _quat_mul_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


class TruthPoseBufferTests(unittest.TestCase):
    def test_extrinsic_imu_mount_and_wxyz_convention(self):
        imu_mount = np.array([0.020153064280748367, -0.04, 0.085], dtype=np.float64)
        # Non-trivial 3D rotation composed from roll/pitch/yaw.
        q_yaw = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.9)
        q_pitch = _quat_from_axis_angle_wxyz([0.0, 1.0, 0.0], -0.35)
        q_roll = _quat_from_axis_angle_wxyz([1.0, 0.0, 0.0], 0.25)
        quat_wb_wxyz = _quat_mul_wxyz(q_yaw, _quat_mul_wxyz(q_pitch, q_roll))
        pos_wb = np.array([1.25, -2.5, 0.85], dtype=np.float64)

        buf = TruthPoseBuffer(imu_mount)
        pos_wi, quat_wi = buf.record(1_000_000_000, pos_wb, quat_wb_wxyz)

        # Cross-check against observation.quaternion_rotation (wxyz) and
        # rtx_deskewed_perception.rotation_from_quaternion_xyzw (xyzw).
        r_obs = quaternion_rotation(quat_wb_wxyz)
        quat_xyzw = np.array([quat_wb_wxyz[1], quat_wb_wxyz[2], quat_wb_wxyz[3], quat_wb_wxyz[0]])
        r_perc = perception.rotation_from_quaternion_xyzw(quat_xyzw)
        r_geom = quaternion_matrix(*quat_wb_wxyz)
        np.testing.assert_allclose(r_obs, r_geom, atol=1e-12)
        np.testing.assert_allclose(r_perc, r_geom, atol=1e-12)

        expected_pos_wi = pos_wb + r_obs @ imu_mount
        np.testing.assert_allclose(pos_wi, expected_pos_wi, atol=1e-12)
        np.testing.assert_allclose(quat_wi, quat_wb_wxyz, atol=1e-12)

        got_pos, got_quat = buf.interpolate_imu_pose(1_000_000_000)
        np.testing.assert_allclose(got_pos, expected_pos_wi, atol=1e-12)
        np.testing.assert_allclose(got_quat, quat_wb_wxyz, atol=1e-12)

    def test_buffer_validation_slerp_shortest_arc_and_capacity(self):
        buf = TruthPoseBuffer([0.0, 0.0, 0.0], capacity=4, max_clamp_ns=5_000_000)
        with self.assertRaises(ValueError):
            buf.interpolate_imu_pose(100)

        q0 = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.2)
        # Negate q1 (antipodal equivalent) to verify shortest-arc SLERP.
        q1 = -_quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.6)
        buf.record(10_000_000, [0.0, 0.0, 0.0], q0)
        with self.assertRaises(ValueError):
            buf.record(10_000_000, [0.0, 0.0, 0.0], q0)  # non-monotonic
        with self.assertRaises(ValueError):
            buf.record(15_000_000, [np.nan, 0.0, 0.0], q0)
        buf.record(20_000_000, [1.0, 0.0, 0.0], q1)

        pos_mid, quat_mid = buf.interpolate_imu_pose(15_000_000)
        np.testing.assert_allclose(pos_mid, [0.5, 0.0, 0.0], atol=1e-12)
        q_expected = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.4)
        # Compare rotation matrices so sign doesn't matter, and dot > 0.
        self.assertGreater(abs(float(np.dot(quat_mid, q_expected))), 1.0 - 1e-12)
        np.testing.assert_allclose(
            quaternion_matrix(*quat_mid), quaternion_matrix(*q_expected), atol=1e-12
        )

        # Boundary clamping within tolerance works; far outside raises.
        pos_clamp, _ = buf.interpolate_imu_pose(24_000_000)
        np.testing.assert_allclose(pos_clamp, [1.0, 0.0, 0.0], atol=1e-12)
        with self.assertRaises(ValueError):
            buf.interpolate_imu_pose(26_000_000)

        # Capacity ring eviction.
        for k in range(3, 8):
            buf.record(k * 10_000_000, [float(k), 0.0, 0.0], q0)
        self.assertEqual(len(buf), 4)
        with self.assertRaises(ValueError):
            buf.interpolate_imu_pose(10_000_000)


class VectorizedDeskewTests(unittest.TestCase):
    def test_32k_points_stair_climb_motion_deskew_accuracy_and_speed(self):
        imu_mount = np.array([0.020153064280748367, 0.0, -0.005108955316245556])
        buf = TruthPoseBuffer(imu_mount, capacity=400)

        # Constant linear velocity (climbing stairs) + constant angular velocity (yaw + pitch).
        p0 = np.array([-14.5, -7.5, -2.8], dtype=np.float64)
        v_lin = np.array([0.85, -0.25, 0.42], dtype=np.float64)
        # Constant axis rotation in world frame so SLERP matches exact continuous pose!
        omega_axis = np.array([0.0, 0.35, 0.90], dtype=np.float64)
        omega_rate = float(np.linalg.norm(omega_axis))
        omega_unit = omega_axis / omega_rate
        q_init = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.4)

        def base_pose_at(t_s: float):
            p_wb = p0 + v_lin * t_s
            q_delta = _quat_from_axis_angle_wxyz(omega_unit, omega_rate * t_s)
            q_wb = _quat_mul_wxyz(q_delta, q_init)
            return p_wb, q_wb

        # Record 200 Hz poses from t=1.000 s to t=1.300 s (dt=5 ms).
        start_ns = 1_000_000_000
        for step in range(61):
            t_ns = start_ns + step * 5_000_000
            p_wb, q_wb = base_pose_at(step * 0.005)
            buf.record(t_ns, p_wb, q_wb)

        # Build 32,768 static world points on a ground plane + stair steps.
        n_pts = 32_768
        rng = np.random.default_rng(42)
        world_true = np.empty((n_pts, 3), dtype=np.float64)
        world_true[:, 0] = rng.uniform(-18.0, -8.0, size=n_pts)
        world_true[:, 1] = rng.uniform(-10.0, -5.0, size=n_pts)
        # Staircase steps in z along x.
        world_true[:, 2] = -2.8 + 0.16 * np.floor((world_true[:, 0] + 15.0) / 0.30).clip(0, 10)

        # Scan spans [1.100 s, 1.1999 s] (99.9 ms).
        scan_start_ns = 1_100_000_000
        offsets_ns = np.linspace(0, 99_900_000, n_pts, dtype=np.int64)
        records = np.zeros(n_pts, dtype=OUSTER_DTYPE)
        for idx_slice in (slice(None),):
            t_rel_s = (scan_start_ns + offsets_ns[idx_slice] - start_ns) * 1e-9
            # Exact continuous IMU poses at each firing instant.
            p_wb_k = p0[None, :] + t_rel_s[:, None] * v_lin[None, :]
            # Exact R_wb(t_k) per point.
            half = 0.5 * omega_rate * t_rel_s
            q_delta = np.column_stack([
                np.cos(half),
                np.sin(half)[:, None] * omega_unit[None, :],
            ])
            # Vectorized quaternion product q_delta * q_init:
            w1, x1, y1, z1 = q_delta[:, 0], q_delta[:, 1], q_delta[:, 2], q_delta[:, 3]
            w2, x2, y2, z2 = q_init
            q_wb_k = np.column_stack([
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ])
            # R_wb @ imu_mount for each point using quaternion rotation:
            u = q_wb_k[:, 1:4]
            w = q_wb_k[:, 0:1]
            m = np.broadcast_to(imu_mount, (n_pts, 3))
            tc = 2.0 * np.cross(u, m)
            p_wi_k = p_wb_k + m + w * tc + np.cross(u, tc)
            # p_imu_k = R_wi^T (world_true - p_wi_k) = rotate by conjugate quaternion.
            diff = world_true - p_wi_k
            q_conj_u = -u
            tc2 = 2.0 * np.cross(q_conj_u, diff)
            p_imu_k = diff + w * tc2 + np.cross(q_conj_u, tc2)

            records["x"] = p_imu_k[:, 0].astype(np.float32)
            records["y"] = p_imu_k[:, 1].astype(np.float32)
            records["z"] = p_imu_k[:, 2].astype(np.float32)
            records["intensity"] = rng.uniform(0.1, 1.0, size=n_pts).astype(np.float32)
            records["t"] = offsets_ns.astype(np.uint32)

        last_retained_ns = scan_start_ns + int(offsets_ns[-1])
        packet = LidarPacket(
            stamp_ns=scan_start_ns,
            records=records,
            receipt={"last_retained_ns": last_retained_ns, "scan_index": 1},
        )
        now_ns = 1_200_000_000

        t0 = time.perf_counter()
        scan = deskew_rtx_packet_to_world(packet, buf, now_ns)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        # Performance: 32,768 points vectorized must complete well below 50 ms (typically < 15 ms).
        self.assertLess(elapsed_ms, 100.0, f"Vectorized deskew too slow: {elapsed_ms:.2f} ms")

        # Accuracy: recovered world points match ground truth within < 1e-5 m.
        err_f64 = np.linalg.norm(scan.points_world - world_true, axis=1).max()
        self.assertLess(err_f64, 1e-5, f"Max float64 deskew error {err_f64} >= 1e-5 m")
        pts_f32 = np.column_stack([scan.records["x"], scan.records["y"], scan.records["z"]]).astype(np.float64)
        err_f32 = np.linalg.norm(pts_f32 - world_true, axis=1).max()
        self.assertLess(err_f32, 1e-5, f"Max float32 record deskew error {err_f32} >= 1e-5 m")

    def test_cloud_layout_curvature_stamps_and_publish_order(self):
        buf = TruthPoseBuffer([0.02, 0.0, -0.005])
        q = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.5)
        buf.record(1_000_000_000, [0.0, 0.0, 0.0], q)
        buf.record(1_100_000_000, [0.1, 0.0, 0.0], q)

        records = np.zeros(10, dtype=OUSTER_DTYPE)
        records["x"] = np.linspace(1.0, 2.0, 10, dtype=np.float32)
        records["t"] = np.linspace(0, 90_000_000, 10, dtype=np.uint32)
        packet = LidarPacket(
            stamp_ns=1_000_000_000,
            records=records,
            receipt={"last_retained_ns": 1_090_000_000},
        )
        # Case A: now_ns < last_retained_ns MUST refuse (no silent truncation).
        with self.assertRaises(ValueError):
            deskew_rtx_packet_to_world(packet, buf, now_ns=1_089_500_000)
        # Case B: uncovered first/last ray MUST refuse (no silent clamping).
        with self.assertRaises(ValueError):
            deskew_rtx_packet_to_world(
                LidarPacket(
                    stamp_ns=999_000_000,
                    records=records,
                    receipt={"last_retained_ns": 1_089_000_000},
                ),
                buf,
                now_ns=1_090_000_000,
            )
        # Case C: covered and clock-covered -> exact last_retained_ns stamp.
        scan = deskew_rtx_packet_to_world(packet, buf, now_ns=1_095_000_000)
        self.assertEqual(scan.cloud_stamp_ns, 1_090_000_000)
        self.assertLessEqual(scan.cloud_stamp_ns, 1_095_000_000)

        self.assertEqual(scan.records.dtype.itemsize, 20)
        self.assertEqual(
            scan.records.dtype.names, ("x", "y", "z", "intensity", "curvature")
        )
        layout = perception.verify_cloud_layout(list(scan.records.dtype.names))
        self.assertEqual(layout, ("x", "y", "z", "intensity", "curvature"))
        marker = perception.verify_deskew_marker(scan.records["curvature"])
        self.assertGreater(marker["curvature_spread_ms"], 1.0)

        expected_pos, expected_quat = buf.interpolate_imu_pose(scan.cloud_stamp_ns)
        np.testing.assert_allclose(scan.pair_pos_wi, expected_pos, atol=1e-12)
        np.testing.assert_allclose(scan.pair_quat_wi_wxyz, expected_quat, atol=1e-12)

        roles = perception.verify_topic_roles(
            TRUTH_DESKEWED_CLOUD_TOPIC, TRUTH_LIO_ODOM_TOPIC
        )
        self.assertEqual(roles["stage"], perception.DESKEWED_STAGE)
        with self.assertRaises(perception.DeskewedCloudError):
            perception.verify_topic_roles(
                TRUTH_DESKEWED_CLOUD_TOPIC, TRUTH_VIO_ODOM_TOPIC
            )

        # Verify ROS message builders & publish order via mocked rclpy/messages.
        class Message:
            def __init__(self, **kwargs):
                self.header = SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )
                self.clock = SimpleNamespace(sec=0, nanosec=0)
                self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.orientation = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
                self.orientation_covariance = [0.0] * 9
                self.fields = []
                self.child_frame_id = ""
                self.pose = SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
                    )
                )
                self.__dict__.update(kwargs)

        modules = {
            "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
            "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        }
        modules["nav_msgs.msg"].Odometry = Message
        modules["sensor_msgs.msg"].PointCloud2 = Message
        modules["sensor_msgs.msg"].PointField = Message
        with mock.patch.dict(sys.modules, modules):
            cloud_msg = truth_deskewed_cloud_message(scan)
            lio_msg = truth_odometry_message(
                scan.cloud_stamp_ns, scan.pair_pos_wi, scan.pair_quat_wi_wxyz
            )
            self.assertEqual(cloud_msg.header.frame_id, TRUTH_FRAME_ID)
            self.assertEqual(lio_msg.header.frame_id, TRUTH_FRAME_ID)
            self.assertEqual(lio_msg.child_frame_id, TRUTH_CHILD_FRAME_ID)
            self.assertEqual(
                (cloud_msg.header.stamp.sec, cloud_msg.header.stamp.nanosec),
                (lio_msg.header.stamp.sec, lio_msg.header.stamp.nanosec),
            )
            self.assertEqual(cloud_msg.point_step, 20)
            pair = perception.verify_same_stage_pair(
                cloud_frame=cloud_msg.header.frame_id,
                pose_frame=lio_msg.header.frame_id,
                child_frame=lio_msg.child_frame_id,
                cloud_stamp_ns=perception.message_stamp_ns(cloud_msg),
                pose_stamp_ns=perception.message_stamp_ns(lio_msg),
                expected_map_frame=TRUTH_FRAME_ID,
                expected_child_frame=TRUTH_CHILD_FRAME_ID,
                stage=roles["stage"],
            )
            self.assertEqual(pair["reference_stamp_ns"], scan.cloud_stamp_ns)

    def test_alternating_antipodal_quaternions_slerp_shortest_arc(self):
        buf = TruthPoseBuffer([0.0, 0.0, 0.0], capacity=50)
        angles = np.linspace(0.0, 1.2, 25)
        for step, angle in enumerate(angles):
            q = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], float(angle))
            if step % 2 == 1:
                q = -q  # Alternating q / -q antipodal representations.
            buf.record(1_000_000_000 + step * 5_000_000, [0.0, 0.0, 0.0], q)
        queries = np.arange(1_000_000_000, 1_000_000_000 + 24 * 5_000_000 + 1, 1_000_000, dtype=np.int64)
        _, quats = buf.interpolate_imu_poses(queries)
        expected_angles = np.linspace(0.0, 1.2, len(queries))
        for q_interp, theta in zip(quats, expected_angles):
            q_exp = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], float(theta))
            np.testing.assert_allclose(
                quaternion_matrix(*q_interp), quaternion_matrix(*q_exp), atol=1e-12
            )

    def test_pack_lidar_scan_to_deskew_90deg_yaw_nonzero_mounts_roundtrip(self):
        imu_mount = np.array([0.020153064280748367, -0.03, 0.015], dtype=np.float64)
        lidar_mount = np.array(LIDAR_MOUNT, dtype=np.float64)  # [0.0, 0.0, 0.25]
        buf = TruthPoseBuffer(imu_mount, capacity=200)

        p0 = np.array([1.5, -2.0, 0.35], dtype=np.float64)
        v_lin = np.array([0.6, 0.3, 0.15], dtype=np.float64)
        yaw0 = math.pi / 2.0  # 90 degrees yaw
        omega_z = 0.8

        t_start_ns = 2_000_000_000
        for step in range(41):  # 0..200 ms at 200 Hz
            t_s = step * 0.005
            q_wb = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], yaw0 + omega_z * t_s)
            buf.record(t_start_ns + step * 5_000_000, p0 + v_lin * t_s, q_wb)

        # Static world targets far outside Go2 self-filter box.
        world_targets = np.array(
            [
                [5.0, 1.0, 0.0],
                [-2.0, 4.5, 0.8],
                [3.2, -6.5, 1.4],
                [0.5, 2.5, -0.4],
                [4.0, -0.5, 2.1],
            ],
            dtype=np.float64,
        )
        scan_start_ns = t_start_ns + 20_000_000
        offsets_ns = np.array([0, 22_500_000, 47_500_000, 72_500_000, 97_500_000], dtype=np.int64)
        xyz_sensor = np.empty_like(world_targets)
        for idx, (p_world, off_ns) in enumerate(zip(world_targets, offsets_ns)):
            t_s = (20_000_000 + int(off_ns)) * 1e-9
            p_wb = p0 + v_lin * t_s
            q_wb = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], yaw0 + omega_z * t_s)
            r_wb = quaternion_matrix(*q_wb)
            # p_world = p_wb + r_wb @ (xyz_sensor + lidar_mount)
            xyz_sensor[idx] = r_wb.T @ (p_world - p_wb) - lidar_mount

        packet = pack_lidar_scan(
            xyz_sensor=xyz_sensor,
            intensity=np.ones(len(world_targets)),
            offsets_ns=offsets_ns,
            ring=np.arange(len(world_targets), dtype=np.uint8),
            timestamp_ns=scan_start_ns,
            imu_mount=imu_mount,
            mount=lidar_mount,
        )
        scan = deskew_rtx_packet_to_world(
            packet, buf, now_ns=scan_start_ns + int(offsets_ns[-1]) + 2_500_000
        )
        np.testing.assert_allclose(scan.points_world, world_targets, atol=1e-5)
        pts_f32 = np.column_stack(
            [scan.records["x"], scan.records["y"], scan.records["z"]]
        ).astype(np.float64)
        np.testing.assert_allclose(pts_f32, world_targets, atol=1e-5)
        self.assertEqual(scan.cloud_stamp_ns, scan_start_ns + int(offsets_ns[-1]))


class ModeAndIsolationContractTests(unittest.TestCase):
    def test_experiment_check_simulation_truth_profile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / "office.usd"
            scene.write_text("stub")
            robot = root / "runtime_assets/robot/go2_description/go2_description.usda"
            robot.parent.mkdir(parents=True, exist_ok=True)
            robot.write_text("stub")
            output = root / "runs/truth_check"
            stdout = io.StringIO()
            with (
                mock.patch.object(exp_module, "data_root", return_value=root),
                mock.patch(
                    "embodied_navigation.robot.unitree.robogauge.runtime.default_model_path",
                    return_value=root / "model.pt",
                ),
                mock.patch(
                    "embodied_navigation.robot.unitree.robogauge.runtime.verify_model",
                    return_value=None,
                ),
                contextlib.redirect_stdout(stdout),
            ):
                rc = exp_module.main(
                    [
                        "--check",
                        "--output",
                        str(output),
                        "--scene",
                        str(scene),
                        "--point-cloud-source",
                        "rtx_lidar",
                        "--interactive",
                        "--interactive-pose-source",
                        "simulation_truth",
                        "--interactive-controller",
                        "pct_scan",
                    ]
                )
            self.assertEqual(rc, 0)
            payload = json.loads(stdout.getvalue())
            profile = payload["profile"]
            self.assertEqual(profile["slam_pose_source"], "simulation_truth")
            self.assertEqual(profile["navigation_pose_source"], "simulation_truth_imu")
            self.assertEqual(profile["odometry_topic"], TRUTH_VIO_ODOM_TOPIC)
            self.assertIs(profile["slam_twist_available"], False)
            self.assertEqual(profile["slam_speed_source"], "pose_window")
            self.assertEqual(profile["interactive_controller"], "pct_scan")

        # Verify truth_deskew.py is listed in experiment.py source snapshot.
        from tests.unit.test_dynanav_source_snapshot import experiment_sources

        self.assertIn(
            "src/embodied_navigation/simulation/dynanav/truth_deskew.py",
            experiment_sources(),
        )

    def test_pct_scan_tracker_pose_source_isolation(self):
        with tempfile.TemporaryDirectory() as tmp:
            box_truth = Path(tmp) / "run_truth/mailbox"
            truth_tracker = PctScanWaypointTracker(
                dict(
                    slam_pose_source=TRUTH_POSE_SOURCE,
                    speed_mps=1.0,
                    motion_end_s=200.0,
                    interactive_mailbox=str(box_truth),
                )
            )
            # Accepts simulation_truth, rejects livo_post_vio.
            truth_tracker._handle_request(
                dict(
                    version=1,
                    run_id="run_truth",
                    seq=1,
                    action="goto",
                    pose_source="livo_post_vio",
                    goal_xyz_m=[3.0, 0.0, 0.0],
                )
            )
            truth_tracker._handle_request(
                dict(
                    version=1,
                    run_id="run_truth",
                    seq=2,
                    action="goto",
                    pose_source=TRUTH_POSE_SOURCE,
                    goal_xyz_m=[3.0, 0.0, 0.0],
                )
            )
            receipts = [
                json.loads(line)
                for line in truth_tracker.receipts_path.read_text().splitlines()
            ]
            self.assertEqual(receipts[0]["event"], "request_rejected")
            self.assertEqual(receipts[0]["reason"], "pose_source_mismatch")
            self.assertEqual(receipts[1]["event"], "waypoint_accepted")

            # Tick rejects livo_post_vio and accepts simulation_truth.
            cmd, row = truth_tracker.tick(
                current_sim_s=10.0,
                odom_stamp_s=10.0,
                position=[0.0, 0.0, 0.0],
                quat_wxyz=[1.0, 0.0, 0.0, 0.0],
                estimated_velocity=None,
                measured_yaw_rate=0.0,
                ready=True,
                now_wall=10.0,
                arrival_wall=10.0,
                frame_id=TRUTH_FRAME_ID,
                child_frame_id=TRUTH_CHILD_FRAME_ID,
                pose_source="livo_post_vio",
            )
            self.assertEqual(cmd, [0.0, 0.0, 0.0])
            self.assertEqual(row["reason"], "pose_source_mismatch")

            cmd, row = truth_tracker.tick(
                current_sim_s=10.1,
                odom_stamp_s=10.1,
                position=[0.0, 0.0, 0.0],
                quat_wxyz=[1.0, 0.0, 0.0, 0.0],
                estimated_velocity=None,
                measured_yaw_rate=0.0,
                ready=True,
                now_wall=10.1,
                arrival_wall=10.1,
                frame_id=TRUTH_FRAME_ID,
                child_frame_id=TRUTH_CHILD_FRAME_ID,
                pose_source=TRUTH_POSE_SOURCE,
            )
            self.assertEqual(row["pose_source"], TRUTH_POSE_SOURCE)
            self.assertIsNotNone(truth_tracker.active_waypoint)
            req = json.loads((box_truth / "planner_request.json").read_text())
            self.assertEqual(req["pose_source"], TRUTH_POSE_SOURCE)

            # Reverse isolation: livo_post_vio tracker rejects simulation_truth.
            box_vio = Path(tmp) / "run_vio/mailbox"
            vio_tracker = PctScanWaypointTracker(
                dict(
                    slam_pose_source="livo_post_vio",
                    speed_mps=1.0,
                    motion_end_s=200.0,
                    interactive_mailbox=str(box_vio),
                )
            )
            vio_tracker._handle_request(
                dict(
                    version=1,
                    run_id="run_vio",
                    seq=1,
                    action="goto",
                    pose_source=TRUTH_POSE_SOURCE,
                    goal_xyz_m=[3.0, 0.0, 0.0],
                )
            )
            vio_receipts = [
                json.loads(line)
                for line in vio_tracker.receipts_path.read_text().splitlines()
            ]
            self.assertEqual(vio_receipts[0]["reason"], "pose_source_mismatch")

    def test_pct_scan_worker_request_and_cli_isolation(self):
        with tempfile.TemporaryDirectory() as tmp:
            mailbox = Path(tmp)
            obs = dict(run_id="r1", waypoints=dict(active_seq=5, active_action="goto"))
            (mailbox / "observation.json").write_text(json.dumps(obs))
            req_truth = dict(
                version=1,
                run_id="r1",
                seq=5,
                goal_xyz_m=[1.0, 2.0, 3.0],
                frame_id=TRUTH_FRAME_ID,
                child_frame_id=TRUTH_CHILD_FRAME_ID,
                pose_source=TRUTH_POSE_SOURCE,
            )
            (mailbox / "planner_request.json").write_text(json.dumps(req_truth))
            self.assertEqual(
                pct_scan_worker.active_request(
                    mailbox, "r1", expected_pose_source=TRUTH_POSE_SOURCE
                ),
                req_truth,
            )
            self.assertIsNone(
                pct_scan_worker.active_request(
                    mailbox, "r1", expected_pose_source="livo_post_vio"
                )
            )
            # Yaw query isolation
            obs_yaw = dict(run_id="r1", waypoints=dict(active_seq=6, active_action="yaw"))
            (mailbox / "observation.json").write_text(json.dumps(obs_yaw))
            yaw_q = dict(
                version=1,
                run_id="r1",
                seq=6,
                query_id=1,
                body_stamp_ns=1_000_000_000,
                frame_id=TRUTH_FRAME_ID,
                child_frame_id=TRUTH_CHILD_FRAME_ID,
                pose_source=TRUTH_POSE_SOURCE,
                target_yaw_rad=0.5,
            )
            (mailbox / "planner_yaw_query.json").write_text(json.dumps(yaw_q))
            self.assertIsNotNone(
                pct_scan_worker.active_request(
                    mailbox, "r1", expected_pose_source=TRUTH_POSE_SOURCE
                )
            )
            self.assertIsNone(
                pct_scan_worker.active_request(
                    mailbox, "r1", expected_pose_source="livo_post_vio"
                )
            )

        # CLI refuses mixing simulation_truth with default SLAM topics, swapped LIO/VIO, or non-canonical topics.
        for bad_args in (
            ["--run-dir", "/tmp/unused", "--pose-source", TRUTH_POSE_SOURCE],
            [
                "--run-dir", "/tmp/unused", "--pose-source", TRUTH_POSE_SOURCE,
                "--cloud-topic", TRUTH_DESKEWED_CLOUD_TOPIC,
                "--lio-topic", TRUTH_VIO_ODOM_TOPIC,
                "--vio-topic", TRUTH_LIO_ODOM_TOPIC,
            ],
            [
                "--run-dir", "/tmp/unused", "--pose-source", TRUTH_POSE_SOURCE,
                "--cloud-topic", "/dynanav/truth/other_cloud",
                "--lio-topic", TRUTH_LIO_ODOM_TOPIC,
                "--vio-topic", TRUTH_VIO_ODOM_TOPIC,
            ],
        ):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                pct_scan_worker.main(bad_args)
            self.assertNotEqual(caught.exception.code, 0)

    def test_experiment_rejects_wrong_truth_topic_and_rgbd_pct_scan_truth(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / "office.usd"
            scene.write_text("stub")
            robot = root / "runtime_assets/robot/go2_description/go2_description.usda"
            robot.parent.mkdir(parents=True, exist_ok=True)
            robot.write_text("stub")
            output = root / "runs/bad_truth"
            with (
                mock.patch.object(exp_module, "data_root", return_value=root),
                mock.patch(
                    "embodied_navigation.robot.unitree.robogauge.runtime.default_model_path",
                    return_value=root / "model.pt",
                ),
                mock.patch(
                    "embodied_navigation.robot.unitree.robogauge.runtime.verify_model",
                    return_value=None,
                ),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                # Reject rgbd + pct_scan + simulation_truth
                with self.assertRaises(SystemExit) as err_rgbd:
                    exp_module.main(
                        [
                            "--check", "--output", str(output), "--scene", str(scene),
                            "--point-cloud-source", "rgbd", "--interactive",
                            "--interactive-pose-source", "simulation_truth",
                            "--interactive-controller", "pct_scan",
                        ]
                    )
                self.assertNotEqual(err_rgbd.exception.code, 0)
                # Reject non-canonical control_odometry topic
                with self.assertRaises(SystemExit) as err_topic:
                    exp_module.main(
                        [
                            "--check", "--output", str(output), "--scene", str(scene),
                            "--point-cloud-source", "rtx_lidar", "--interactive",
                            "--interactive-pose-source", "simulation_truth",
                            "--interactive-controller", "pct_scan",
                            "--odometry-topic", "/other/control_odometry",
                        ]
                    )
                self.assertNotEqual(err_topic.exception.code, 0)
        with self.assertRaises(ValueError):
            exp_module.slam_odometry_topic(TRUTH_POSE_SOURCE, "/other/control_odometry")

    def test_annotate_policy_receipt_native_route_simulation_truth(self):
        from embodied_navigation.simulation.dynanav.capture.control import (
            annotate_policy_receipt,
        )

        result = {"policy": {}}
        annotate_policy_receipt(
            result,
            {"slam_pose_source": TRUTH_POSE_SOURCE, "interactive_controller": "native_route"},
            False,
            True,
            0.5,
            1.0,
            interactive_control=True,
        )
        self.assertEqual(
            result["speed_test"]["interactive_command_source"],
            "native_route_tracker_over_simulation_truth_pose",
        )
        self.assertTrue(result["policy"]["ground_truth_navigation"])
        self.assertFalse(result["policy"]["SLAM_control_enabled"])

    def test_deferred_truth_packet_waits_one_step_without_truncating_stamp(self):
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        from embodied_navigation.simulation.dynanav.capture.ros_interface import (
            create_ros_interface,
        )

        class Message:
            def __init__(self, **kwargs):
                self.header = SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )
                self.clock = SimpleNamespace(sec=0, nanosec=0)
                self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.orientation = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
                self.orientation_covariance = [0.0] * 9
                self.fields = []
                self.child_frame_id = ""
                self.pose = SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
                    )
                )
                self.__dict__.update(kwargs)

        published_log = []

        def make_pub(kind, topic, qos):
            pub = mock.Mock(topic_name=topic)
            pub.get_subscription_count.return_value = 0
            pub.publish.side_effect = lambda msg, t=topic: published_log.append((t, msg))
            return pub

        node = mock.Mock()
        node.create_publisher.side_effect = make_pub
        rclpy = mock.Mock()
        rclpy.create_node.return_value = node
        modules = {
            "rclpy": rclpy,
            "geometry_msgs.msg": types.SimpleNamespace(TwistStamped=Message),
            "nav_msgs.msg": types.SimpleNamespace(Odometry=Message),
            "rclpy.qos": types.SimpleNamespace(
                qos_profile_sensor_data=object(),
                QoSProfile=types.SimpleNamespace,
                ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1),
            ),
            "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
            "rosgraph_msgs.msg": types.SimpleNamespace(Clock=Message),
            "sensor_msgs.msg": types.SimpleNamespace(
                CameraInfo=Message, Image=Message, Imu=Message,
                PointCloud2=Message, PointField=Message,
            ),
        }
        with mock.patch.dict(sys.modules, modules):
            ros = create_ros_interface(
                {
                    "point_cloud_source": "rtx_lidar",
                    "slam_pose_source": TRUTH_POSE_SOURCE,
                    "odometry_topic": TRUTH_VIO_ODOM_TOPIC,
                },
                None,
                None,
            )
            buf = TruthPoseBuffer([0.02, 0.0, -0.005])
            q = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.2)
            buf.record(1_000_000_000, [0.0, 0.0, 0.0], q)
            buf.record(1_100_000_000, [0.1, 0.0, 0.0], q)
            # Clock at step 1.100 s is 1.0998 s (< last_retained_ns = 1.1005 s) and pose ends at 1.100 s.
            ros.publish_imu_and_clock(
                1.0998,
                {"linear_acceleration": [0, 0, 9.81], "angular_velocity": [0, 0, 0], "orientation": [1, 0, 0, 0]},
                False,
            )
            records = np.zeros(4, dtype=OUSTER_DTYPE)
            records["x"] = 2.0
            records["t"] = np.array([0, 30_000_000, 60_000_000, 100_500_000], dtype=np.uint32)
            packet = LidarPacket(
                stamp_ns=1_000_000_000,
                records=records,
                receipt={"last_retained_ns": 1_100_500_000, "scan_index": 1},
            )
            fake_rtx = mock.Mock()
            fake_rtx.read.side_effect = [packet, None, None, None]
            fake_rtx.scans = 1
            fake_receipts = SimpleNamespace(result={}, journal=mock.Mock())
            with tempfile.TemporaryDirectory() as tmp:
                stream = FrameStream(
                    experiment={"slam_pose_source": TRUTH_POSE_SOURCE},
                    out=Path(tmp), stage=None, gt_diagnostic=False, ros=ros, video=None,
                    rgb_annotators=SimpleNamespace(rgb_ann=None, depth_ann=None, camera_params_ann=None),
                    overview=SimpleNamespace(ann=None),
                    camera_rig=SimpleNamespace(camera=None, sensor_yaw=None, mount=None, imu_mount=[0.02, 0.0, -0.005]),
                    target_speed=1.0, receipts=fake_receipts, rtx_lidar=fake_rtx, truth_pose_buffer=buf,
                )
                # Step i: packet ends at 1.1005 s > actual 1.100 s -> deferred, NOT truncated!
                stream.publish_rtx_pending(1.100)
                self.assertEqual(
                    [t for t, _ in published_log if t in (TRUTH_LIO_ODOM_TOPIC, TRUTH_DESKEWED_CLOUD_TOPIC)],
                    [],
                )
                # Step i+1 (5 ms later): pose at 1.105 s & /clock at 1.105 s cover 1.1005 s -> published!
                buf.record(1_105_000_000, [0.105, 0.0, 0.0], q)
                ros.publish_imu_and_clock(
                    1.105,
                    {"linear_acceleration": [0, 0, 9.81], "angular_velocity": [0, 0, 0], "orientation": [1, 0, 0, 0]},
                    False,
                )
                stream.publish_rtx_pending(1.105)
                truth_pub = [
                    (t, msg) for t, msg in published_log
                    if t in (TRUTH_LIO_ODOM_TOPIC, TRUTH_DESKEWED_CLOUD_TOPIC)
                ]
                self.assertEqual(
                    [t for t, _ in truth_pub],
                    [TRUTH_LIO_ODOM_TOPIC, TRUTH_DESKEWED_CLOUD_TOPIC],
                )
                self.assertEqual(perception.message_stamp_ns(truth_pub[0][1]), 1_100_500_000)
                self.assertEqual(perception.message_stamp_ns(truth_pub[1][1]), 1_100_500_000)

                # If a packet still lacks coverage after > 5 ms, fail closed.
                packet_bad = LidarPacket(
                    stamp_ns=1_100_000_000,
                    records=records,  # ends at 1_200_500_000
                    receipt={"last_retained_ns": 1_200_500_000, "scan_index": 2},
                )
                fake_rtx.read.side_effect = [packet_bad, None, None]
                stream.publish_rtx_pending(1.200)
                with self.assertRaises(ValueError):
                    stream.publish_rtx_pending(1.206)

    def test_ros_interface_publishes_truth_lio_before_cloud_with_identical_stamp(self):
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        from embodied_navigation.simulation.dynanav.capture.ros_interface import (
            create_ros_interface,
        )

        class Message:
            def __init__(self, **kwargs):
                self.header = SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )
                self.clock = SimpleNamespace(sec=0, nanosec=0)
                self.linear_acceleration = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.orientation = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
                self.orientation_covariance = [0.0] * 9
                self.fields = []
                self.child_frame_id = ""
                self.pose = SimpleNamespace(
                    pose=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
                    )
                )
                self.__dict__.update(kwargs)

        published_log = []

        def make_pub(kind, topic, qos):
            pub = mock.Mock(topic_name=topic)
            pub.get_subscription_count.return_value = 0
            pub.publish.side_effect = lambda msg, t=topic: published_log.append((t, msg))
            return pub

        node = mock.Mock()
        node.create_publisher.side_effect = make_pub
        rclpy = mock.Mock()
        rclpy.create_node.return_value = node
        modules = {
            "rclpy": rclpy,
            "geometry_msgs.msg": types.SimpleNamespace(TwistStamped=Message),
            "nav_msgs.msg": types.SimpleNamespace(Odometry=Message),
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
            "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
            "rosgraph_msgs.msg": types.SimpleNamespace(Clock=Message),
            "sensor_msgs.msg": types.SimpleNamespace(
                CameraInfo=Message, Image=Message, Imu=Message,
                PointCloud2=Message, PointField=Message,
            ),
        }
        with mock.patch.dict(sys.modules, modules):
            ros = create_ros_interface(
                {
                    "point_cloud_source": "rtx_lidar",
                    "slam_pose_source": TRUTH_POSE_SOURCE,
                    "odometry_topic": TRUTH_VIO_ODOM_TOPIC,
                },
                None,
                None,
            )
            buf = TruthPoseBuffer([0.02, 0.0, -0.005])
            q = _quat_from_axis_angle_wxyz([0.0, 0.0, 1.0], 0.3)
            pos_wi, quat_wi = buf.record(1_000_000_000, [1.0, 2.0, 0.3], q)
            buf.record(1_100_000_000, [1.1, 2.0, 0.3], q)
            ros.publish_imu_and_clock(
                1.095,
                {
                    "linear_acceleration": [0.0, 0.0, 9.81],
                    "angular_velocity": [0.0, 0.0, 0.0],
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                },
                False,
            )
            ros.publish_truth_vio(1_095_000_000, pos_wi, quat_wi)

            records = np.zeros(5, dtype=OUSTER_DTYPE)
            records["x"] = 2.0
            records["t"] = np.array([0, 20_000_000, 40_000_000, 60_000_000, 90_000_000], dtype=np.uint32)
            packet = LidarPacket(
                stamp_ns=1_000_000_000,
                records=records,
                receipt={"last_retained_ns": 1_090_000_000, "scan_index": 2},
            )
            fake_rtx = mock.Mock()
            fake_rtx.read.side_effect = [packet, None]
            fake_rtx.scans = 1
            fake_annotators = SimpleNamespace(
                rgb_ann=None, depth_ann=None, camera_params_ann=None
            )
            fake_receipts = SimpleNamespace(result={}, journal=mock.Mock())
            with tempfile.TemporaryDirectory() as tmp:
                stream = FrameStream(
                    experiment={"slam_pose_source": TRUTH_POSE_SOURCE},
                    out=Path(tmp),
                    stage=None,
                    gt_diagnostic=False,
                    ros=ros,
                    video=None,
                    rgb_annotators=fake_annotators,
                    overview=SimpleNamespace(ann=None),
                    camera_rig=SimpleNamespace(
                        camera=None, sensor_yaw=None, mount=None, imu_mount=[0.02, 0.0, -0.005]
                    ),
                    target_speed=1.0,
                    receipts=fake_receipts,
                    rtx_lidar=fake_rtx,
                    truth_pose_buffer=buf,
                )
                stream.publish_rtx_pending(1.100)

        truth_events = [
            (topic, msg)
            for topic, msg in published_log
            if topic in (TRUTH_VIO_ODOM_TOPIC, TRUTH_LIO_ODOM_TOPIC, TRUTH_DESKEWED_CLOUD_TOPIC)
        ]
        self.assertEqual(
            [t for t, _ in truth_events],
            [TRUTH_VIO_ODOM_TOPIC, TRUTH_LIO_ODOM_TOPIC, TRUTH_DESKEWED_CLOUD_TOPIC],
        )
        lio_msg = truth_events[1][1]
        cloud_msg = truth_events[2][1]
        self.assertEqual(
            (lio_msg.header.stamp.sec, lio_msg.header.stamp.nanosec),
            (cloud_msg.header.stamp.sec, cloud_msg.header.stamp.nanosec),
        )
        self.assertEqual(perception.message_stamp_ns(lio_msg), 1_090_000_000)
        self.assertEqual(lio_msg.header.frame_id, TRUTH_FRAME_ID)
        self.assertEqual(lio_msg.child_frame_id, TRUTH_CHILD_FRAME_ID)
        self.assertEqual(cloud_msg.header.frame_id, TRUTH_FRAME_ID)

    def test_explore_waypoint_client_accepts_truth_and_refuses_forbidden(self):
        payload = client.build_request(
            seq=1,
            action="goto",
            goal=[2.0, 0.0, 0.0],
            pose_source=TRUTH_POSE_SOURCE,
            run_id="run_truth",
            created_wall_s=1.0,
        )
        self.assertEqual(payload["pose_source"], TRUTH_POSE_SOURCE)
        yaw_payload = client.build_request(
            seq=2,
            action="yaw",
            pose_source=TRUTH_POSE_SOURCE,
            run_id="run_truth",
            created_wall_s=1.0,
            extra={"delta_yaw_rad": 0.5},
        )
        self.assertEqual(yaw_payload["pose_source"], TRUTH_POSE_SOURCE)
        for forbidden in FORBIDDEN_POSE_SOURCES:
            with self.assertRaises(ValueError):
                client.build_request(
                    seq=3,
                    action="goto",
                    goal=[2.0, 0.0, 0.0],
                    pose_source=forbidden,
                    run_id="run_truth",
                    created_wall_s=1.0,
                )
        parser = client.build_parser()
        args = parser.parse_args(
            [
                "map",
                "--run-dir",
                "/tmp/run_truth",
                "--cloud-topic",
                TRUTH_DESKEWED_CLOUD_TOPIC,
                "--odom-topic",
                TRUTH_LIO_ODOM_TOPIC,
                "--pose-source",
                TRUTH_POSE_SOURCE,
                "--cloud-semantics",
                "lio_deskewed_map",
            ]
        )
        self.assertEqual(args.cloud_topic, TRUTH_DESKEWED_CLOUD_TOPIC)
        self.assertEqual(args.odometry_topic, TRUTH_LIO_ODOM_TOPIC)
        self.assertEqual(args.pose_source, TRUTH_POSE_SOURCE)


if __name__ == "__main__":
    unittest.main()
