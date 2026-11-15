"""CPU-only Writer/queue observations with a synthetic GMO parser, not an RTX pass."""

import hashlib
import itertools
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from embodied_navigation.simulation.dynanav.capture import lidar


class NativeWriterTimingTests(unittest.TestCase):
    def setUp(self):
        # Import and instantiate the actual Writer adapter, never Isaac or ROS.
        names = ["isaacsim", "isaacsim.sensors", "isaacsim.sensors.experimental",
                 "isaacsim.sensors.experimental.rtx", "omni", "omni.replicator",
                 "omni.replicator.core", "sensor_msgs", "sensor_msgs.msg"]
        modules = {name: ModuleType(name) for name in names}
        for name, module in modules.items():
            if "." in name:
                parent, child = name.rsplit(".", 1)
                setattr(modules[parent], child, module)
        registry = {}
        rep = modules["omni.replicator.core"]
        rep.Writer = object
        rep.WriterRegistry = SimpleNamespace(register=lambda cls: registry.update({cls.__name__: cls}))
        rep.annotators = SimpleNamespace(get=lambda name: name)

        class Sensor:
            def __init__(self, *args, **kwargs):
                pass

            def attach_writer(self, name):
                return registry[name]()

        rtx = modules["isaacsim.sensors.experimental.rtx"]
        rtx.Lidar = SimpleNamespace(create=lambda *args, **kwargs: SimpleNamespace())
        rtx.LidarSensor = Sensor
        rtx.parse_generic_model_output_data = self.parse
        msg = modules["sensor_msgs.msg"]
        msg.PointField = lambda **kwargs: SimpleNamespace(**kwargs)
        msg.PointCloud2 = lambda: SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace()), fields=[])
        self.parsed_buffers = []
        self.enterContext(patch.dict(sys.modules, modules))
        self.enterContext(patch.object(lidar.time, "monotonic_ns", side_effect=itertools.count(1_000_000, 1000)))
        self.sensor = lidar.RtxLidar(SimpleNamespace(base_path="/Robot", imu_mount=[.02, 0, -.005]), "unused.usd")

    def raw(self, stamp_ns=100_000_000, frame_id=7, complete=1):
        raw = np.zeros(3, dtype=[("xyz", "<f4", (3,)), ("scalar", "<f4"),
                                  ("offset", "<i8"), ("ring", "<u4"),
                                  ("flags", "u1"), ("stamp", "<i8"),
                                  ("frame", "<u4"), ("complete", "u1")])
        raw["xyz"] = [[2., 0., 0.], [0., 3., 0.], [-4., 0., 0.]]
        raw["scalar"] = [.2, .3, .4]
        raw["offset"] = [1525, 50_000_000, 99_998_450]
        raw["ring"] = [0, 15, 31]
        raw["flags"] = 64
        raw["stamp"], raw["frame"], raw["complete"] = stamp_ns, frame_id, complete
        return raw

    def parse(self, raw):
        self.parsed_buffers.append(raw)
        pose = SimpleNamespace(timestampNs=int(raw["stamp"][0]), posM=[0., 0., 0.],
                               orientation=[0., 0., 0., 1.])
        return SimpleNamespace(
            numElements=len(raw), timestampNs=raw["stamp"][0], frameId=raw["frame"][0],
            scanComplete=raw["complete"][0], frameOfReference=SimpleNamespace(name="SENSOR"),
            motionCompensationState=SimpleNamespace(name="NONCOMPENSATED"),
            elementsCoordsType=SimpleNamespace(name="CARTESIAN"), modelToAppTransform=np.eye(4),
            x=raw["xyz"][:, 0], y=raw["xyz"][:, 1], z=raw["xyz"][:, 2],
            scalar=raw["scalar"], timeOffsetNs=raw["offset"], channelId=raw["ring"],
            flags=raw["flags"], frameStart=pose, frameEnd=pose,
        )

    def write(self, *raws):
        self.sensor.writer.write({"renderProducts": {
            str(index): {"GenericModelOutput": {"data": raw}}
            for index, raw in enumerate(raws)
        }})

    def test_copy_timing_native_pairing_and_ros_wire_are_unchanged(self):
        raw = self.raw()
        expected = lidar.pack_lidar_scan(
            xyz_sensor=raw["xyz"], intensity=raw["scalar"], offsets_ns=raw["offset"],
            ring=raw["ring"], timestamp_ns=100_000_000, imu_mount=self.sensor.imu_mount,
            valid_flags=(raw["flags"] & 64) != 0,
        )
        wrapped = SimpleNamespace(numpy=lambda: raw)
        self.write(wrapped)
        copied = self.parsed_buffers[0]
        self.assertFalse(np.shares_memory(copied, raw))
        original_bytes = raw.tobytes()
        raw["xyz"] = 999
        raw["offset"] = 0
        raw["stamp"] = 0
        self.assertEqual(copied.tobytes(), original_bytes)
        packet = self.sensor.read()
        r = packet.receipt
        self.assertEqual(packet.records.tobytes(), expected.records.tobytes())
        np.testing.assert_array_equal(packet.records["t"], [1525, 50_000_000, 99_998_450])
        np.testing.assert_array_equal(packet.records["ring"], [0, 15, 31])
        self.assertEqual(r["point_sha256"], hashlib.sha256(expected.records.tobytes()).hexdigest())
        self.assertEqual(r["native_timestamp_ns"], 100_000_000)
        self.assertEqual(r["native_frame_id"], 7)
        self.assertEqual(r["scan_index"], 1)
        self.assertFalse(r["motion_compensation"])
        self.assertFalse(r["truth_pose_used"])
        self.assertEqual(r["host_timing_schema"], 1)
        self.assertEqual(r["writer_callback_sequence"], 1)
        self.assertEqual(r["raw_copy_bytes"], raw.nbytes)
        times = [r[key] for key in ("writer_callback_monotonic_ns", "raw_copy_begin_monotonic_ns",
                                   "raw_copy_end_monotonic_ns", "enqueue_monotonic_ns",
                                   "read_monotonic_ns", "dequeue_monotonic_ns")]
        self.assertEqual(times, sorted(times))
        self.assertEqual(r["raw_copy_duration_ns"], times[2] - times[1])
        self.assertEqual(r["queue_residence_ns"], times[-1] - times[3])
        self.assertGreaterEqual(r["queue_residence_ns"], 0)
        actual_msg, expected_msg = lidar.pointcloud_message(packet), lidar.pointcloud_message(expected)
        self.assertEqual(vars(actual_msg), vars(expected_msg))
        self.assertEqual(actual_msg.header.stamp.sec, 0)
        self.assertEqual(actual_msg.header.stamp.nanosec, 100_000_000)
        self.assertEqual(actual_msg.header.frame_id, "go2_imu")
        json.dumps(r)  # Every added observation remains JSON-serializable.

    def test_multiple_products_share_callback_but_have_distinct_enqueues(self):
        self.assertIsNone(self.sensor.read())
        self.write(self.raw(), self.raw(200_000_000, 8))
        first, second = self.sensor.read(), self.sensor.read()
        a, b = first.receipt, second.receipt
        self.assertEqual(a["writer_callback_sequence"], b["writer_callback_sequence"])
        self.assertEqual(a["writer_callback_monotonic_ns"], b["writer_callback_monotonic_ns"])
        self.assertLess(a["enqueue_monotonic_ns"], b["enqueue_monotonic_ns"])
        self.assertEqual((a["read_sequence"], b["read_sequence"]), (2, 3))
        self.assertEqual((a["queue_depth_before_enqueue"], b["queue_depth_before_enqueue"]), (0, 1))
        self.assertEqual((a["queue_depth_after_enqueue"], b["queue_depth_after_enqueue"]), (1, 2))
        self.assertEqual((a["queue_depth_before_dequeue"], b["queue_depth_before_dequeue"]), (2, 1))
        self.assertEqual((a["queue_depth_after_dequeue"], b["queue_depth_after_dequeue"]), (1, 0))
        self.assertEqual((a["queue_peak_depth"], b["queue_peak_depth"]), (1, 2))
        self.assertIsNone(self.sensor.read())

    def test_duplicate_partial_startup_and_empty_callbacks_do_not_add_scans(self):
        self.sensor.minimum_scan_start_ns = 50_000_000
        self.write()
        self.write(self.raw(0))
        self.write(self.raw())
        self.write(self.raw(frame_id=99))  # Same native timestamp is still a duplicate.
        self.write(self.raw(200_000_000, 8, complete=0))
        self.write(self.raw(200_000_000, 8))
        self.assertEqual(len(self.sensor.pending), 2)
        self.assertEqual(self.sensor.scans, 2)
        first, second = self.sensor.read(), self.sensor.read()
        self.assertEqual((first.receipt["writer_callback_sequence"], second.receipt["writer_callback_sequence"]), (3, 6))
        self.assertEqual(second.receipt["native_scan_start_delta_ns"], 100_000_000)

    def test_original_four_scan_limit_remains_fail_closed(self):
        for index in range(4):
            self.sensor.accept_native_output(self.raw((index + 1) * 100_000_000, index))
        self.assertEqual(self.sensor.queue_peak_depth, 4)
        self.assertEqual([p.receipt["queue_capacity_scans"] for p in self.sensor.pending], [4] * 4)
        self.assertEqual([p.receipt["queue_nominal_coverage_ns"] for p in self.sensor.pending], [400_000_000] * 4)
        before = [(p.stamp_ns, p.records.tobytes()) for p in self.sensor.pending]
        with self.assertRaisesRegex(RuntimeError, "overflow"):
            self.sensor.accept_native_output(self.raw(500_000_000, 4))
        self.assertEqual([(p.stamp_ns, p.records.tobytes()) for p in self.sensor.pending], before)
        self.assertEqual(self.sensor.scans, 5)  # Existing parse-before-overflow behavior.
        self.assertEqual(self.sensor.queue_peak_depth, 4)

    def test_callback_errors_still_surface_before_any_dequeue(self):
        self.write(self.raw(200_000_000, 8))
        self.write(self.raw(100_000_000, 7))
        self.assertIsInstance(self.sensor.writer_error, ValueError)
        with self.assertRaisesRegex(RuntimeError, "Native LiDAR writer failed") as error:
            self.sensor.read()
        self.assertIn("backwards", str(error.exception.__cause__))
        self.assertEqual(len(self.sensor.pending), 1)

    def test_receipt_flows_to_existing_journal_with_physics_context_separate(self):
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        self.write(self.raw())
        rows, published = [], []
        stream = FrameStream.__new__(FrameStream)
        stream.rtx_lidar = self.sensor
        stream.receipts = SimpleNamespace(result={}, journal=lambda name, row: rows.append((name, row)))
        stream.ros = SimpleNamespace(publish_rtx_cloud=published.append)
        stream.out = Path("/unused")
        with patch.object(np, "savez_compressed"):
            stream.publish_rtx_pending(.315)
        name, receipt = rows[0]
        self.assertEqual(name, "rtx_lidar_frames.jsonl")
        self.assertEqual(receipt["acquisition_physics_ns"], 315_000_000)
        self.assertEqual(receipt["stamp_ns"], 100_000_000)
        for key in ("writer_callback_monotonic_ns", "enqueue_monotonic_ns", "dequeue_monotonic_ns",
                    "queue_depth_before_dequeue", "queue_residence_ns"):
            self.assertEqual(receipt[key], published[0].receipt[key])
        self.assertEqual(len(rows), 1)


if __name__ == "__main__":
    unittest.main()
