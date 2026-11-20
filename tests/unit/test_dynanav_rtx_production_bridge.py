"""Offline production-bridge tests: deskewed LIO cloud -> operator map + perception.

CPU only, no ROS runtime: fake messages carry the exact native PointXYZINormal
layout (48-byte point) or a raw wire layout, and the real accumulator is used so
the frame, pair and floor gates are exercised end to end.
"""

import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import explore_waypoint_client as client
from embodied_navigation.simulation.dynanav.operator_map import (
    Extrinsic,
    OccupancyAccumulator,
)

MOUNT = [0.020153064280748367, 0.0, -0.005108955316245556]
NATIVE_FIELDS = [
    ("x", 0), ("y", 4), ("z", 8), ("normal_x", 16), ("normal_y", 20),
    ("normal_z", 24), ("intensity", 32), ("curvature", 36),
]
RAW_FIELDS = [
    ("x", 0), ("y", 4), ("z", 8), ("intensity", 12), ("t", 16),
    ("reflectivity", 20), ("ring", 22), ("ambient", 24), ("range", 28),
]


def native_message(points, *, stamp_ns=9_300_000_000, frame_id="dynanav_lio_map",
                   curvature=None):
    """Build a PointCloud2-like message with the native 48-byte layout."""
    array = np.asarray(points, dtype=float)
    count = array.shape[0]
    buffer = np.zeros(count * 48, dtype=np.uint8)
    view = buffer.view(np.float32)
    for index in range(count):
        base = index * 12
        view[base + 0] = array[index, 0]
        view[base + 1] = array[index, 1]
        view[base + 2] = array[index, 2]
        view[base + 8] = 0.5  # intensity
        view[base + 9] = float(index) if curvature is None else float(curvature)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000,
                                                    nanosec=stamp_ns % 1_000_000_000),
                               frame_id=frame_id),
        fields=[SimpleNamespace(name=name, offset=offset, datatype=7)
                for name, offset in NATIVE_FIELDS],
        width=count, height=1, point_step=48, data=buffer.tobytes(),
    )


def raw_message(points, *, stamp_ns=9_300_000_000, frame_id="go2_imu"):
    """Build a raw rotating wire message (9 fields, 32-byte point)."""
    array = np.asarray(points, dtype=float)
    count = array.shape[0]
    buffer = np.zeros(count * 32, dtype=np.uint8)
    view = buffer.view(np.float32)
    for index in range(count):
        base = index * 8
        view[base + 0] = array[index, 0]
        view[base + 1] = array[index, 1]
        view[base + 2] = array[index, 2]
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000,
                                                    nanosec=stamp_ns % 1_000_000_000),
                               frame_id=frame_id),
        fields=[SimpleNamespace(name=name, offset=offset, datatype=7)
                for name, offset in RAW_FIELDS],
        width=count, height=1, point_step=32, data=buffer.tobytes(),
    )


def odometry_message(position=(0.0, 0.0, 0.4), quat=(0.0, 0.0, 0.0, 1.0),
                     stamp_ns=9_300_000_000, frame_id="dynanav_lio_map",
                     child="go2_imu"):
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000,
                                                     nanosec=stamp_ns % 1_000_000_000),
                               frame_id=frame_id),
        child_frame_id=child,
    )
    msg.pose = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
        orientation=SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    ))
    return msg


def floor_cloud(radius_min=1.6, radius_max=3.0, step=0.06):
    """Map-frame ground plane around the origin plus a wall ring at 2.5 m."""
    rows = []
    for angle in np.arange(-180.0, 180.0, 0.5):
        radians = np.radians(angle)
        for radius in np.arange(radius_min, radius_max, step):
            rows.append([radius * np.cos(radians), radius * np.sin(radians), 0.0])
        for height in np.arange(-0.2, 1.0, 0.08):
            rows.append([2.5 * np.cos(radians), 2.5 * np.sin(radians), height])
    return np.asarray(rows, dtype=float)


class Args(SimpleNamespace):
    """Map command arguments used by the deskewed path."""

    def __init__(self, **overrides):
        values = dict(
            cloud_topic="/dynanav/rtx_livo/cloud_registered_full",
            odometry_topic="/dynanav/rtx_livo/aft_mapped_to_init",
            pose_source="lio_estimate",
            sensor_frame="go2_imu",
            base_frame="go2_base",
            imu_mount=list(MOUNT),
            binary_prefix=None,
            cloud_semantics="lio_deskewed_map",
        )
        values.update(overrides)
        super().__init__(**values)


def state_for(accumulator, *, clock=9.4):
    return {
        "cloud_semantics": "lio_deskewed_map",
        "deskewed_stage": "LIO",
        "pair_required": True,
        "lio_poses": {},
        "source_binary_identity": None,
        "source_binary_sha256": None,
        "last_pair_contract": None,
        "clock": clock,
        "clock_arrival_wall": time.perf_counter(),
        "pose_frame_accepted": 0,
        "pose_frame_rejected": 0,
        "pose_frame_note": None,
        "odometry_map_frame": None,
        "frame_rejections": 0,
        "frame_rejection_reason": None,
        "pose_wait_rejections": 0,
        "last_pose_wait_rejection": None,
        "pose_stamps": [],
        "admission_rejections": 0,
        "pending_overflow_rejections": 0,
        "last_cloud_rejection": None,
        "frames": 0,
        "accepted": 0,
        "last": None,
        "rejected": 0,
        "frame_mismatch": None,
        "declared_frame": None,
    }


class DeskewedBridgeTests(unittest.TestCase):
    """Content, frame, exact-pair and freshness gates of the production bridge."""

    def setUp(self):
        self.args = Args()
        self.accumulator = OccupancyAccumulator(
            extrinsic=Extrinsic(
                translation=list(MOUNT),
                quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
                sensor_frame="go2_imu",
                base_frame="go2_base",
            ),
            resolution_m=0.05,
        )

    def admit(self, message, state, arrival=None, now=None):
        now = time.perf_counter() if now is None else now
        arrival = now if arrival is None else arrival
        return client._deskewed_cloud_admission(
            message, arrival, state, self.args, now
        )

    def seed_pose(self, state, stamp_ns=9_300_000_000):
        message = odometry_message(stamp_ns=stamp_ns)
        client._ingest_odometry(message, self.accumulator, state, self.args, time.perf_counter())
        self.assertEqual("dynanav_lio_map", state["odometry_map_frame"])
        return message

    def test_raw_wire_payload_on_deskewed_topic_is_refused_by_content(self):
        state = state_for(self.accumulator)
        verdict, reason, detail = self.admit(raw_message(floor_cloud()), state)
        self.assertEqual("reject", verdict)
        self.assertEqual("raw_wire_cloud_refused", reason)
        self.assertIn("t", detail["fields"])

    def test_wrong_map_frame_is_refused(self):
        state = state_for(self.accumulator)
        self.seed_pose(state)
        verdict, reason, _ = self.admit(
            native_message(floor_cloud(), frame_id="camera_init"), state
        )
        self.assertEqual("reject", verdict)
        self.assertEqual("cloud_frame_mismatch", reason)

    def test_missing_exact_pose_waits_then_rejects(self):
        state = state_for(self.accumulator)
        self.seed_pose(state)
        arrival = time.perf_counter()
        message = native_message(floor_cloud(), stamp_ns=9_400_000_000)
        verdict, reason, _ = self.admit(message, state, arrival=arrival, now=arrival + 0.05)
        self.assertEqual("wait_pose", verdict)
        verdict, reason, _ = self.admit(message, state, arrival=arrival, now=arrival + 1.0)
        self.assertEqual("reject", verdict)
        self.assertEqual("pose_pair_missing_exact", reason)

    def test_near_stamp_is_not_interpolated(self):
        state = state_for(self.accumulator)
        self.seed_pose(state, stamp_ns=9_300_000_000)
        # 1 ns away: a different pair, no bracket/interpolation.
        message = native_message(floor_cloud(), stamp_ns=9_300_000_001)
        now = time.perf_counter()
        verdict, reason, _ = self.admit(message, state, arrival=now - 1.0, now=now)
        self.assertEqual("reject", verdict)
        self.assertIn(reason, ("pose_pair_missing_exact", "pair_contract_violation"))

    def test_stale_clock_is_refused(self):
        state = state_for(self.accumulator, clock=20.0)
        self.seed_pose(state)
        message = native_message(floor_cloud(), stamp_ns=9_300_000_000)
        verdict, reason, _ = self.admit(message, state, now=time.perf_counter())
        self.assertEqual("reject", verdict)
        self.assertEqual("cloud_sim_expired", reason)

    def test_zero_curvature_payload_is_refused(self):
        from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import (
            DeskewedCloudError,
        )

        message = native_message(floor_cloud(), curvature=0.0)
        with self.assertRaises(DeskewedCloudError):
            client._decode_native_deskewed_cloud(message)

    def test_accepted_deskewed_cloud_enters_map_as_map_frame_once(self):
        state = state_for(self.accumulator)
        self.seed_pose(state)
        message = native_message(floor_cloud())
        receipt = client._ingest_deskewed_cloud(
            message, time.perf_counter(), self.accumulator, state, self.args
        )
        self.assertTrue(receipt.get("accepted"), json.dumps(receipt)[:600])
        self.assertEqual("map", self.accumulator._last_declared_cloud_frame)
        self.assertEqual(0.0, receipt["paired_dt_s"])
        self.assertFalse(receipt["pairing_interpolated"])
        self.assertEqual(state["last_pair_contract"]["reference_stamp_ns"], 9_300_000_000)
        self.assertEqual("measured_standoff_and_extent",
                         receipt["floor_evidence"]["evidence_route"])

    def test_deskewed_default_does_not_carve_from_an_imu_ray_origin(self):
        state = state_for(self.accumulator)
        self.seed_pose(state)
        # The externally supplied accumulator still defaults to both. The
        # production ingest must enforce floor-only rather than trusting it.
        self.assertEqual("both", self.accumulator.free_mode)
        receipt = client._ingest_deskewed_cloud(
            native_message(floor_cloud()), time.perf_counter(),
            self.accumulator, state, self.args,
        )
        self.assertTrue(receipt["accepted"], receipt)
        grid = self.accumulator.grid_for(0)
        self.assertEqual(-1, grid.code_at(*grid.index_of(0.5, 0.0)))
        occupied, free = grid.counts()
        self.assertGreater(occupied, 0)
        self.assertGreater(free, 0)
        self.assertEqual("auto", state["free_space_policy"]["requested"])
        self.assertEqual("floor", state["free_space_policy"]["effective"])
        self.assertIn("ray_origin_contract",
                      state["free_space_policy"]["ray_carving_disabled_reason"])
        self.assertEqual(state["free_space_policy"], receipt["free_space_policy"])

    def test_deskewed_explicit_none_does_not_add_free_cells(self):
        state = state_for(self.accumulator)
        self.seed_pose(state)
        receipt = client._ingest_deskewed_cloud(
            native_message(floor_cloud()), time.perf_counter(),
            self.accumulator, state, Args(free_mode="none"),
        )
        self.assertTrue(receipt["accepted"], receipt)
        occupied, free = self.accumulator.grid_for(0).counts()
        self.assertGreater(occupied, 0)
        self.assertEqual(0, free)

    def test_deskewed_explicit_ray_modes_fail_before_ros_startup(self):
        parser = client.build_parser()
        for mode in ("rays", "both"):
            with self.subTest(mode=mode):
                args = parser.parse_args([
                    "map", "--run-dir", "/tmp/not-a-live-run", "--odometry-topic",
                    "/dynanav/rtx_livo/aft_mapped_to_init",
                    "--cloud-semantics", "lio_deskewed_map", "--free-mode", mode,
                ])
                args.mailbox = None
                with self.assertRaisesRegex(ValueError, "cannot safely satisfy"):
                    client._cmd_map(args)
                state = state_for(self.accumulator)
                with self.assertRaisesRegex(ValueError, "cannot safely satisfy"):
                    client._ingest_deskewed_cloud(
                        native_message(floor_cloud()), time.perf_counter(),
                        self.accumulator, state, Args(free_mode=mode),
                    )
                self.assertEqual((), self.accumulator.levels)

    def test_free_policy_preserves_legacy_modes_and_lio_floor_or_none(self):
        self.assertEqual("both", client._operator_map_free_policy("sensor_imu")["effective"])
        for mode in ("floor", "rays", "both", "none"):
            policy = client._operator_map_free_policy("sensor_imu", mode)
            self.assertEqual(mode, policy["effective"])
            self.assertIsNone(policy["ray_carving_disabled_reason"])
        for mode in ("floor", "none"):
            self.assertEqual(mode, client._operator_map_free_policy("lio_deskewed_map", mode)["effective"])

    def test_default_cloud_semantics_keeps_legacy_path(self):
        parser = client.build_parser()
        args = parser.parse_args([
            "map", "--run-dir", "/tmp/run-20261110", "--odometry-topic",
            "/dynanav/rtx_livo/aft_mapped_to_init",
        ])
        self.assertEqual("sensor_imu", args.cloud_semantics)

    def test_deskewed_semantics_requires_imu_mount_at_startup(self):
        with tempfile.TemporaryDirectory() as temporary:
            run_dir = Path(temporary) / "run-20261110"
            run_dir.mkdir()
            args = Args(imu_mount=None)
            args.run_dir = run_dir
            args.mailbox = None
            args.extrinsic_translation = list(MOUNT)
            with self.assertRaises(ValueError):
                client._cmd_map(args)


if __name__ == "__main__":
    unittest.main()
