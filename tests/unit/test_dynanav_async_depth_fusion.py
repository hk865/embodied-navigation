"""CPU-only async auxiliary fusion contracts. Never starts ROS/GPU/control."""
import copy
from dataclasses import replace
import hashlib
import io
import struct
import zipfile
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from embodied_navigation.simulation.dynanav.capture import nearfield
from embodied_navigation.simulation.dynanav import async_depth_fusion as async_fusion
from embodied_navigation.simulation.dynanav import depth_safety_fusion as fusion
from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import build_perception_payload
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import explore_waypoint_client as client

STAMP = 10_000_000_000
PAIR = dict(reference_stamp_ns=STAMP, cloud_stamp_ns=STAMP, pose_stamp_ns=STAMP,
            map_frame="lio_map", child_frame="go2_imu", stage="LIO")
CONFIG = dict(imu_mount=[0., 0., 0.], cloud_topic="/rtx/cloud_registered_full",
              odometry_topic="/rtx/aft_mapped_to_init")
META = dict(PAIR, cloud_topic=CONFIG["cloud_topic"], pose_topic=CONFIG["odometry_topic"],
            binary_sha256="a" * 64)


def fixture(directory, stamp=STAMP):
    v, u = np.indices((256, 256))
    rays = np.stack(((u - 128) / (256 * 12 / 42), (v - 128) / (256 * 12 / 42), np.ones_like(u)), -1)
    base = rays @ nearfield.R_BASE_OPTICAL.T
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = (-.344 - nearfield.MOUNT[2]) / base[..., 2]
    depth[depth <= .05] = np.inf
    with np.errstate(invalid="ignore"):
        xyz = base * depth[..., None] + nearfield.MOUNT
    depth[np.abs(np.degrees(np.arctan2(xyz[..., 1], xyz[..., 0]))) >= 44.] = np.inf
    labels = {"2": "/World/environment/mesh", "3": "/World/Go2/Geometry/base"}
    arrays, masks = nearfield.backproject(depth.astype(np.float32),
        {"data": np.full((256, 256), 2, np.uint32), "info": {"idToLabels": labels}})
    record = dict(schema=nearfield.SCHEMA, source=nearfield.SOURCE, stamp_ns=stamp,
                  source_time_s=stamp * 1e-9, arrival_wall_s=100., arrival_clock="perf_counter",
                  frame_id="go2_base", calibration=nearfield.calibration_receipt(),
                  calibration_verified_from_render_parameters=True, depth_type="distance_to_image_plane",
                  native_reference_time=[stamp, 1_000_000_000], depth_reference_time=[stamp, 1_000_000_000],
                  mask_reference_time=[stamp, 1_000_000_000],
                  time_evidence={"event_reference": [stamp, 1_000_000_000], "body_now_substituted": False,
                                 "reference_after_reads": [stamp, 1_000_000_000]},
                  self_mask=dict(masks, method="same_frame_instance_prim_path", same_frame=True,
                                 robot_prim_root=nearfield.ROBOT_ROOT, instance_id_to_prim=labels,
                                 mapping_sha256=hashlib.sha256(json.dumps(labels, sort_keys=True,
                                     separators=(",", ":")).encode()).hexdigest()),
                  environment_ground_truth_used=False, world_pose_used=False, slam_input=False, map_input=False)
    nearfield.write_frame(directory, arrays, record)
    angle, radius = np.meshgrid(np.deg2rad(np.arange(-180., 180., 1.)), np.arange(1.2, 3.6, .04))
    return np.column_stack((np.cos(angle).ravel() * radius.ravel(),
                            np.sin(angle).ravel() * radius.ravel(), np.full(angle.size, -.344)))


def wait_result(worker):
    end = time.perf_counter() + 5
    while time.perf_counter() < end:
        result = worker.poll()
        if result is not None:
            return result
        time.sleep(.001)
    raise AssertionError("worker did not finish within CPU test bound")


class AsyncFusionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        self.points = fixture(self.directory)
        self.pose = dict(position=[0., 0., 0.], quat_xyzw=[0., 0., 0., 1.], arrival_wall_s=100.)
        self.wall = [100.1]
        self.clock = async_fusion.ReceivedClock()
        self.clock.update(10.1, 100.)
        self.path = self.directory / "latest.json"

    def snapshot(self, **changes):
        options = dict(generation=0, sequence=1, points_map=self.points, pose=self.pose,
                       pair=PAIR, config=CONFIG, binary_sha256="a" * 64,
                       cloud_arrival_wall_s=100., depth_manifest=self.path)
        options.update(changes)
        return async_fusion.FusionSnapshot.capture(**options)

    def compute(self, snapshot=None):
        return async_fusion.compute_fusion(snapshot or self.snapshot(), self.clock, lambda: self.wall[0])

    def coordinator(self, **changes):
        options = dict(binary_sha256="a" * 64, wall_clock=lambda: self.wall[0])
        options.update(changes)
        session = async_fusion.FusionCoordinator(**options)
        self.addCleanup(session.close)
        session.on_clock(10.1, 100.)
        return session

    def result(self):
        job = self.snapshot()
        return async_fusion.FusionResult(job, self.compute(job), None)

    def rewrite(self, mutate):
        record = json.loads(self.path.read_bytes())
        mutate(record)
        raw = json.dumps(record).encode()
        self.path.write_bytes(raw)
        (self.directory / ("frame_%019d.json" % STAMP)).write_bytes(raw)

    def test_flag_default_off_and_rejects_legacy_or_missing_enable(self):
        args = client.build_parser().parse_args(["map", "--run-dir", str(self.directory),
                                                "--odometry-topic", CONFIG["odometry_topic"]])
        self.assertFalse(args.enable_async_depth_fusion)
        self.assertFalse(client._async_fusion_requested(args))
        args.enable_async_depth_fusion = True
        with self.assertRaisesRegex(ValueError, "requires native LIO"):
            client._async_fusion_requested(args)
        args.cloud_semantics = "lio_deskewed_map"
        args.nearfield_depth_manifest = self.path
        args.perception_out = self.directory / "perception.json"
        with self.assertRaises(ValueError):
            client._async_fusion_requested(args)
        args.enable_nearfield_depth_fusion = True
        self.assertTrue(client._async_fusion_requested(args))

    def test_snapshot_and_disk_loader_identical_complete_evidence(self):
        snap = self.snapshot()
        kwargs = dict(reference_stamp_ns=STAMP, now_sim_s=10.1, now_wall_s=100.1)
        points, evidence = fusion.load_depth_frame(self.path, **kwargs)
        frozen_points, frozen_evidence = fusion.load_depth_frame(self.path, snapshot=snap.depth, **kwargs)
        np.testing.assert_array_equal(points, frozen_points)
        self.assertEqual(evidence, frozen_evidence)

    def test_geometry_and_source_contract_identical_to_synchronous(self):
        native = fusion.native_source_envelope(self.points, stamp_s=10., arrival_wall_s=100., producer_metadata=META)
        expected = fusion.build_safety_fusion(self.points, native, pair=PAIR, pose_arrival_wall_s=100.,
            depth_manifest=self.path, now_sim_s=10.1, now_wall_s=100.1, enabled=True)
        actual = self.compute()
        for key in ("directions", "safety_geometry", "safety_fusion", "clearance_m", "fail_closed",
                    "stamp_s", "arrival_wall_s", "source", "corridor", "observation_state"):
            self.assertEqual(expected[key], actual[key], key)
        self.assertTrue(fusion.validate_fusion_contract(actual, now_sim_s=10.1, now_wall_s=100.1))
        self.assertEqual(actual["directions"]["left"]["observation_state"], "insufficient_observation")

    def test_snapshot_owns_buffers_and_is_unaffected_by_disk_replacement(self):
        pair, config = copy.deepcopy(PAIR), copy.deepcopy(CONFIG)
        job = self.snapshot(pair=pair, config=config)
        expected = self.compute(job)
        self.points[:] = 999
        self.pose["position"][0] = 900
        pair["pose_stamp_ns"] += 1
        config["imu_mount"][0] = 888
        Path(job.depth.archive_path).write_bytes(b"invalid later disk data")
        self.assertEqual(expected, self.compute(job))
        with self.assertRaises(ValueError):
            job.points.setflags(write=True)
        with self.assertRaises(ValueError):
            job.points[0, 0] = 1.
        with self.assertRaisesRegex(ValueError, "version_changed"):
            job.depth.verify_current()

    def test_normal_new_depth_does_not_invalidate_older_exact_frame(self):
        job = self.snapshot()
        expected = self.compute(job)
        fixture(self.directory, STAMP + 100_000_000)
        self.assertTrue(job.depth.verify_current())
        self.assertEqual(expected, self.compute(job))
        self.assertTrue(self.coordinator().validate(async_fusion.FusionResult(job, expected, None)))

    def test_newer_partial_pointer_cannot_authorize_old_complete_result(self):
        result = self.result()
        session = self.coordinator()
        old = json.loads(self.path.read_bytes())
        self.path.write_text(json.dumps({"schema": old["schema"], "source": old["source"],
                                        "stamp_ns": STAMP + 100_000_000}))
        with self.assertRaises((ValueError, OSError)):
            session.validate(result)

    def test_pointer_rollback_below_capture_latest_refused(self):
        fixture(self.directory, STAMP + 200_000_000)
        result = self.result()
        fixture(self.directory, STAMP + 100_000_000)
        session = self.coordinator()
        session.on_clock(10.21, 100.1)
        with self.assertRaisesRegex(ValueError, "rolled_back"):
            session.validate(result)

    def test_pointer_rollback_below_last_seen_latest_refused(self):
        result = self.result()
        fixture(self.directory, STAMP + 100_000_000)
        earlier = self.path.read_bytes()
        fixture(self.directory, STAMP + 200_000_000)
        session = self.coordinator()
        session.on_clock(10.21, 100.1)
        self.assertTrue(session.validate(result))
        self.path.write_bytes(earlier)
        with self.assertRaisesRegex(ValueError, "rolled_back"):
            session.validate(result)

    def test_newer_pointer_full_evidence_still_requires_original_validation(self):
        result = self.result()
        fixture(self.directory, STAMP + 100_000_000)
        canonical = self.directory / ("frame_%019d.json" % (STAMP + 100_000_000))
        original = self.path.read_bytes()
        cases = [lambda r: r.update(npz_sha256="0" * 64),
                 lambda r: r["calibration"].update(translation_base_m=[9., 0., 0.]),
                 lambda r: r.update(mask_reference_time=[11, 1])]
        for mutate in cases:
            with self.subTest(mutate=mutate):
                record = json.loads(original)
                mutate(record)
                raw = json.dumps(record).encode()
                self.path.write_bytes(raw)
                canonical.write_bytes(raw)
                with self.assertRaises(ValueError):
                    self.coordinator().validate(result)

    def test_newer_corrupt_zip_with_matching_hash_invalidates_cached_file(self):
        result = self.result()
        session = self.coordinator()
        args = SimpleNamespace(perception_out=self.directory / "perception.json")
        state = {"async_fusion": session}
        with mock.patch.object(session.worker, "poll", return_value=result):
            client._publish_async_fusion(args, state)
        fixture(self.directory, STAMP + 100_000_000)
        record = json.loads(self.path.read_bytes())
        archive = Path(record["npz_path"])
        archive.write_bytes(b"PK\x03\x04bad")
        record["npz_sha256"] = hashlib.sha256(archive.read_bytes()).hexdigest()
        raw = json.dumps(record).encode()
        self.path.write_bytes(raw)
        (self.directory / ("frame_%019d.json" % (STAMP + 100_000_000))).write_bytes(raw)
        client._publish_async_fusion(args, state)
        published = json.loads(args.perception_out.read_bytes())
        self.assertTrue(published["fail_closed"])
        self.assertIsNone(published["clearance_m"])
        self.assertEqual(published["arrival_wall_s"], 100.)
        self.assertEqual(session.counters["cached_invalidated"], 1)
        self.assertIn("depth_archive_invalid", session.last_error)

    def test_newer_encrypted_and_unsupported_zip_invalidate_cached_file(self):
        result = self.result()
        fixture(self.directory, STAMP + 100_000_000)
        original_record = json.loads(self.path.read_bytes())
        archive = Path(original_record["npz_path"])
        original_bytes = archive.read_bytes()
        canonical = self.directory / ("frame_%019d.json" % (STAMP + 100_000_000))
        for mode in ("encrypted", "unsupported_compression"):
            with self.subTest(mode=mode):
                archive.write_bytes(original_bytes)
                raw = json.dumps(original_record).encode()
                self.path.write_bytes(raw); canonical.write_bytes(raw)
                session = self.coordinator()
                args = SimpleNamespace(perception_out=self.directory / "perception.json")
                state = {"async_fusion": session}
                with mock.patch.object(session.worker, "poll", return_value=result):
                    client._publish_async_fusion(args, state)
                data = bytearray(original_bytes)
                with zipfile.ZipFile(io.BytesIO(data)) as stored:
                    local = stored.getinfo("stamp_ns.npy").header_offset
                central = data.index(b"PK\x01\x02")
                while True:
                    name_len, extra_len, comment_len = struct.unpack_from("<HHH", data, central + 28)
                    if data[central+46:central+46+name_len] == b"stamp_ns.npy":
                        break
                    central += 46 + name_len + extra_len + comment_len
                if mode == "encrypted":
                    struct.pack_into("<H", data, local + 6, struct.unpack_from("<H", data, local + 6)[0] | 1)
                    struct.pack_into("<H", data, central + 8, struct.unpack_from("<H", data, central + 8)[0] | 1)
                else:
                    struct.pack_into("<H", data, local + 8, 99)
                    struct.pack_into("<H", data, central + 10, 99)
                archive.write_bytes(data)
                record = dict(original_record, npz_sha256=hashlib.sha256(data).hexdigest())
                raw = json.dumps(record).encode()
                self.path.write_bytes(raw); canonical.write_bytes(raw)
                client._publish_async_fusion(args, state)
                published = json.loads(args.perception_out.read_bytes())
                self.assertIsNone(published["clearance_m"])
                self.assertTrue(published["fail_closed"])
                self.assertEqual((published["stamp_s"], published["arrival_wall_s"]), (10., 100.))
                self.assertEqual(session.counters["cached_invalidated"], 1)
                self.assertIn("depth_archive_invalid", session.last_error)

    def test_newer_future_camera_pointer_is_identity_only_and_cached_without_geometry(self):
        result = self.result()
        fixture(self.directory, STAMP + 200_000_000)
        session = self.coordinator()  # Received clock is still 10.1, not 10.2.
        original = fusion._validate_depth_pointer_integrity
        with mock.patch.object(nearfield, "backproject", side_effect=AssertionError("no new geometry on main")), \
                mock.patch.object(fusion, "_validate_depth_pointer_integrity", wraps=original) as check:
            for _ in range(3):
                self.assertTrue(session.validate(result))
        self.assertEqual(check.call_count, 1)
        self.assertEqual(session.stats()["pointer_authority"]["integrity_cache_hits"], 2)
        self.assertEqual(result.payload["safety_fusion"]["sources"]["depth"]["stamp_ns"], STAMP)
        self.assertEqual(session.clock.read(100.1), (10.1, 0))

    def test_wrong_source_schema_calibration_hash_or_rp_still_rejected(self):
        original = self.path.read_bytes()
        cases = [lambda r: r.update(source="unknown"), lambda r: r.update(schema="unknown"),
                 lambda r: r["calibration"].update(translation_base_m=[9., 0., 0.]),
                 lambda r: r.update(npz_sha256="0" * 64),
                 lambda r: r.update(mask_reference_time=[11, 1]),
                 lambda r: r["self_mask"].update(mapping_sha256="0" * 64)]
        for change in cases:
            with self.subTest(change=change):
                self.path.write_bytes(original)
                self.rewrite(change)
                job = self.snapshot()
                result = self.compute(job)
                native = build_perception_payload(self.points, stamp_s=10., arrival_wall_s=100., producer_metadata=META)
                sync = fusion.build_safety_fusion(self.points, native, pair=PAIR, pose_arrival_wall_s=100.,
                    depth_manifest=self.path, now_sim_s=10.1, now_wall_s=100.1, enabled=True)
                self.assertFalse(result["depth_safety_diagnostic"]["accepted"])
                self.assertEqual(sync["depth_safety_diagnostic"]["reason"], result["depth_safety_diagnostic"]["reason"])
                self.assertEqual(sync["directions"], result["directions"])
                with self.assertRaisesRegex(ValueError, "source_unknown"):
                    self.coordinator().validate(async_fusion.FusionResult(job, result, None))

    def test_wrong_exact_pair_binary_or_source_topic_never_publishes(self):
        for changes in (dict(pair=dict(PAIR, pose_stamp_ns=STAMP + 1)),
                        dict(binary_sha256=None),
                        dict(config=dict(CONFIG, cloud_topic="/unknown"))):
            with self.subTest(changes=changes):
                job = self.snapshot(**changes)
                result = self.compute(job)
                self.assertFalse(result["depth_safety_diagnostic"]["accepted"])
                with self.assertRaises(ValueError):
                    self.coordinator().validate(async_fusion.FusionResult(job, result, None))

    def test_old_generation_out_of_order_and_duplicate_result_refused(self):
        result = self.result()
        session = self.coordinator()
        self.assertTrue(session.validate(result))
        session.published(result)
        with self.assertRaisesRegex(ValueError, "out_of_order"):
            session.validate(result)
        newer = replace(result, snapshot=replace(result.snapshot, sequence=2, stamp_ns=STAMP+1))
        session.published(newer)
        with self.assertRaisesRegex(ValueError, "out_of_order"):
            session.validate(result)
        session.on_clock(10.05, 100.1)
        with self.assertRaisesRegex(ValueError, "generation"):
            session.validate(result, cached=True)

    def test_stopped_clock_wall_and_sim_expiry_refused_independently(self):
        result = self.result()
        for wall, clock, arrival in ((100.3, 10.1, 100.), (100.51, 10.1, 100.51),
                                     (100.1, 10.251, 100.1)):
            with self.subTest(wall=wall, clock=clock):
                session = self.coordinator()
                self.wall[0] = wall
                session.on_clock(clock, arrival)
                with self.assertRaises(ValueError):
                    session.validate(result)
                self.wall[0] = 100.1

    def test_expiry_during_geometry_refused_using_new_real_received_clock(self):
        original = fusion.build_perception_payload
        def expire(*args, **kwargs):
            payload = original(*args, **kwargs)
            self.clock.update(10.3, 100.1)
            return payload
        with mock.patch.object(fusion, "build_perception_payload", side_effect=expire):
            with self.assertRaisesRegex(ValueError, "sim_expired"):
                self.compute()

    def test_pending_replaced_bound_one_and_shutdown_is_bounded(self):
        entered, release = threading.Event(), threading.Event()
        seen = []
        def compute(job):
            seen.append(job.sequence)
            entered.set()
            release.wait(3)
            return {"sequence": job.sequence}
        worker = async_fusion.LatestFusionWorker(compute)
        self.addCleanup(release.set)
        self.addCleanup(worker.close)
        job = self.snapshot()
        self.assertTrue(worker.submit(job))
        self.assertTrue(entered.wait(1))
        for seq in range(2, 11):
            self.assertTrue(worker.submit(replace(job, sequence=seq, stamp_ns=STAMP+seq)))
            self.assertEqual(worker.stats()["pending"], 1)
        self.assertEqual(worker.stats()["pending_replaced"], 8)
        start = time.perf_counter()
        self.assertFalse(worker.close(timeout=.01))
        self.assertLess(time.perf_counter() - start, .1)
        release.set()
        self.assertTrue(worker.close(timeout=1.))
        self.assertIsNone(worker.poll())
        self.assertEqual(seen, [1])

    def test_latest_pending_runs_and_old_generation_completion_is_discarded(self):
        entered, release = threading.Event(), threading.Event()
        def compute(job):
            if job.sequence == 1:
                entered.set(); release.wait(2)
            return {"sequence": job.sequence}
        worker = async_fusion.LatestFusionWorker(compute)
        self.addCleanup(worker.close)
        job = self.snapshot()
        worker.submit(job); self.assertTrue(entered.wait(1))
        worker.submit(replace(job, sequence=2, stamp_ns=STAMP+2))
        worker.submit(replace(job, sequence=3, stamp_ns=STAMP+3))
        worker.generation(1)
        self.assertFalse(worker.submit(job))
        worker.submit(replace(job, generation=1, sequence=4, stamp_ns=STAMP+4))
        release.set()
        result = wait_result(worker)
        self.assertEqual(result.snapshot.sequence, 4)
        self.assertEqual(worker.stats()["completion_old_generation_or_closed"], 1)

    def test_expired_queued_job_never_runs_geometry(self):
        self.wall[0] = 100.6
        self.clock.update(10.1, 100.6)
        worker = async_fusion.LatestFusionWorker(lambda job: async_fusion.compute_fusion(
            job, self.clock, lambda: self.wall[0]))
        self.addCleanup(worker.close)
        with mock.patch.object(fusion, "build_perception_payload", side_effect=AssertionError("geometry must not run")):
            worker.submit(self.snapshot())
            result = wait_result(worker)
        self.assertIn("wall_expired", result.error)
        self.assertEqual(worker.stats()["expired_jobs"], 1)

    def test_main_owner_enforced(self):
        session = self.coordinator()
        errors = []
        def other_thread():
            try:
                session.on_clock(10.1, 100.)
            except Exception as error:
                errors.append(str(error))
        thread = threading.Thread(target=other_thread); thread.start(); thread.join()
        self.assertEqual(errors, ["async_fusion_single_owner_violation"])

    def test_partial_deleted_or_replaced_original_sources_never_publish(self):
        result = self.result()
        session = self.coordinator()
        self.path.write_bytes(b'{"partial":')
        with self.assertRaises(ValueError):
            session.validate(result)
        self.path.write_bytes(result.snapshot.depth.latest_bytes)
        for path in (Path(result.snapshot.depth.canonical_path), Path(result.snapshot.depth.archive_path)):
            raw = path.read_bytes()
            path.unlink()
            with self.assertRaises((OSError, ValueError)):
                session.validate(result)
            path.write_bytes(raw)
            with self.assertRaisesRegex(ValueError, "version_changed"):
                session.validate(result)

    def test_source_code_version_change_refused(self):
        source = self.directory / "consumer.py"; source.write_text("original")
        session = self.coordinator(source_paths=[source])
        result = self.result()
        source.write_text("modified")
        with self.assertRaisesRegex(ValueError, "consumer_source_version_changed"):
            session.validate(result)

    def test_read_race_refused_without_retry(self):
        original = async_fusion.file_version
        calls = [0]
        def raced(path):
            version = original(path)
            if Path(path) == self.path:
                calls[0] += 1
                if calls[0] > 1:
                    return version[:-1] + (version[-1] + 1,)
            return version
        with mock.patch.object(async_fusion, "file_version", side_effect=raced):
            with self.assertRaisesRegex(ValueError, "changed_during_read"):
                self.snapshot()

    def test_publish_rechecks_after_serialization_then_cache_expires_without_retime(self):
        result = self.result()
        session = self.coordinator()
        state = {"async_fusion": session}
        args = SimpleNamespace(perception_out=self.directory / "perception.json")
        with mock.patch.object(session.worker, "poll", return_value=result):
            client._publish_async_fusion(args, state)
        self.assertEqual(json.loads(args.perception_out.read_bytes())["clearance_m"], result.payload["clearance_m"])
        self.wall[0] = 100.6
        session.on_clock(10.1, 100.6)
        client._publish_async_fusion(args, state)
        payload = json.loads(args.perception_out.read_bytes())
        self.assertIsNone(payload["clearance_m"])
        self.assertTrue(payload["fail_closed"])
        self.assertEqual(payload["stamp_s"], 10.)
        self.assertEqual(payload["arrival_wall_s"], 100.)
        self.assertEqual(session.counters["cached_invalidated"], 1)
        client._publish_async_fusion(args, state)
        self.assertEqual(session.counters["cached_invalidated"], 1)

    def test_expiry_between_poll_and_atomic_replace_cannot_publish(self):
        result = self.result()
        session = self.coordinator()
        args = SimpleNamespace(perception_out=self.directory / "perception.json")
        original = Path.write_text
        def delayed_write(path, text, *a, **kw):
            written = original(path, text, *a, **kw)
            session.on_clock(10.3, 100.1)
            return written
        with mock.patch.object(session.worker, "poll", return_value=result), \
                mock.patch.object(Path, "write_text", delayed_write), \
                mock.patch.object(client.os, "replace", wraps=client.os.replace) as replaced:
            client._publish_async_fusion(args, {"async_fusion": session})
        replaced.assert_not_called()
        self.assertFalse(args.perception_out.exists())
        self.assertEqual(session.counters["pre_replace_rejected"], 1)

    def _native_bridge_inputs(self, session=None):
        fields = ["x", "y", "z", "normal_x", "normal_y", "normal_z", "intensity", "curvature"]
        raw = np.zeros(len(self.points), dtype=[(name, "<f4") for name in fields])
        for axis, name in enumerate(("x", "y", "z")):
            raw[name] = self.points[:, axis]
        raw["curvature"] = np.linspace(0., 82., len(raw))
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0), frame_id="lio_map"),
            fields=[SimpleNamespace(name=name, datatype=7, offset=i*4) for i, name in enumerate(fields)],
            data=raw.tobytes(), point_step=32, width=len(raw), height=1)
        pose = dict(self.pose, stamp_ns=STAMP, stamp_s=10., map_pose=np.eye(4),
                    frame_id="lio_map", child_frame_id="go2_imu")
        state = dict(clock=10.1, clock_arrival_wall=100., source_binary_sha256="a"*64,
                     cloud_semantics="lio_deskewed_map", odometry_map_frame="lio_map",
                     lio_poses={STAMP: pose}, deskewed_stage="LIO")
        if session is not None:
            state["async_fusion"] = session
        args = SimpleNamespace(**CONFIG, sensor_frame="go2_imu", pose_source="lio_estimate",
                               free_mode="auto", perception_out=self.directory/"perception.json",
                               nearfield_depth_manifest=self.path)
        accumulator = SimpleNamespace(add_frame=mock.Mock(return_value={"accepted": True}))
        return message, state, args, accumulator

    def test_bridge_opt_in_enqueues_native_only_map_and_main_alone_publishes(self):
        session = self.coordinator()
        message, state, args, accumulator = self._native_bridge_inputs(session)
        with mock.patch.object(client.time, "perf_counter", return_value=100.1), \
                mock.patch.object(client, "_write_deskewed_perception", side_effect=AssertionError("sync must not run")):
            receipt = client._ingest_deskewed_cloud(message, 100., accumulator, state, args)
        self.assertTrue(receipt["async_fusion_enqueued"])
        self.assertNotIn("perception", receipt)
        self.assertFalse(args.perception_out.exists())
        native_map_points = accumulator.add_frame.call_args.args[0]
        np.testing.assert_allclose(native_map_points, self.points, atol=2e-7)
        self.assertEqual(len(native_map_points), len(self.points))
        result = wait_result(session.worker)
        self.assertIsNone(result.error)
        evidence = result.payload["depth_safety_diagnostic"]["native_cloud_evidence"]
        self.assertEqual(evidence["wire_sha256"], hashlib.sha256(message.data).hexdigest())
        self.assertGreater(evidence["deskew_marker"]["curvature_spread_ms"], 80.)
        with mock.patch.object(session.worker, "poll", return_value=result):
            client._publish_async_fusion(args, state)
        self.assertEqual(json.loads(args.perception_out.read_bytes())["source"], fusion.FUSION_SOURCE)
        self.assertEqual(state["perception_samples"], 1)

    def test_bridge_off_retains_synchronous_call_and_raw_layout_still_rejected(self):
        message, state, args, accumulator = self._native_bridge_inputs()
        with mock.patch.object(client.time, "perf_counter", return_value=100.1), \
                mock.patch.object(client, "_write_deskewed_perception", return_value={"original_sync": True}) as write:
            receipt = client._ingest_deskewed_cloud(message, 100., accumulator, state, args)
        write.assert_called_once()
        self.assertEqual(receipt["perception"], {"original_sync": True})
        self.assertNotIn("async_fusion_enqueued", receipt)
        session = self.coordinator()
        message, state, args, accumulator = self._native_bridge_inputs(session)
        message.fields.append(SimpleNamespace(name="t", datatype=7, offset=0))
        with mock.patch.object(client.time, "perf_counter", return_value=100.1):
            receipt = client._ingest_deskewed_cloud(message, 100., accumulator, state, args)
        self.assertFalse(receipt["accepted"])
        accumulator.add_frame.assert_not_called()
        self.assertEqual(session.worker.stats().get("submitted", 0), 0)

    def _service_probe(self, real_geometry):
        entered, finished = threading.Event(), threading.Event()
        original = fusion.build_perception_payload
        def geometry(*args, **kwargs):
            entered.set()
            try:
                if real_geometry:
                    return original(*args, **kwargs)
                # Pure Python CPU load intentionally holds the GIL between the
                # interpreter's ordinary thread switches, not time.sleep.
                until = time.perf_counter() + .2
                n = 0
                while time.perf_counter() < until:
                    n = (n + 7) % 100_003
                return original(*args, **kwargs)
            finally:
                finished.set()
        session = self.coordinator()
        job = self.snapshot()
        fixture(self.directory, STAMP + 100_000_000)
        next_pair = dict(PAIR, reference_stamp_ns=STAMP+100_000_000,
                         cloud_stamp_ns=STAMP+100_000_000, pose_stamp_ns=STAMP+100_000_000)
        next_job = self.snapshot(sequence=2, pair=next_pair)
        latencies, callbacks, service_gaps = [], [], []
        with mock.patch.object(fusion, "build_perception_payload", side_effect=geometry):
            session.worker.submit(job)
            self.assertTrue(entered.wait(2))
            previous = time.perf_counter()
            while not finished.is_set():
                start = time.perf_counter()
                service_gaps.append(start - previous)
                previous = start
                session.on_clock(10.1, 100.1)
                # Simulate arrival of the next native complete pair at the
                # owner, not another worker spin or a shared ROS executor.
                callbacks.append("clock+nextcloud")
                if len(callbacks) == 1:
                    self.assertTrue(session.worker.submit(next_job))
                latencies.append(time.perf_counter() - start)
                time.sleep(.001)
            result = wait_result(session.worker)
        self.assertGreater(len(callbacks), 0)
        self.assertLess(max(latencies), .1)
        self.assertLess(max(service_gaps), .1)
        self.assertLessEqual(session.worker.stats()["pending"], 1)
        self.assertIsNone(result.error)
        print("ASYNC_CPU_SERVICE", json.dumps({"real_geometry": real_geometry,
              "callbacks_during_geometry": len(callbacks), "max_service_call_ms": max(latencies)*1000,
              "max_inter_service_gap_ms": max(service_gaps)*1000,
              "pending_replaced": session.worker.stats().get("pending_replaced", 0)}))

    def test_pure_python_gil_load_allows_clock_and_nextcloud_service(self):
        self._service_probe(False)

    def test_real_numpy_geometry_allows_clock_and_nextcloud_service(self):
        self._service_probe(True)


if __name__ == "__main__":
    unittest.main()
