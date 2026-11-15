"""Pure CPU safety contract tests; synthetic pixels are never a GPU acceptance."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from embodied_navigation.simulation.dynanav.capture import nearfield
from embodied_navigation.simulation.dynanav.depth_safety_fusion import (
    FUSION_SOURCE, build_safety_fusion, load_depth_frame, validate_fusion_contract,
)
from embodied_navigation.simulation.dynanav.exploration_waypoints import InteractiveWaypointTracker
from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import build_perception_payload
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import explore_waypoint_client as client

STAMP = 10_000_000_000
PAIR = dict(reference_stamp_ns=STAMP, cloud_stamp_ns=STAMP, pose_stamp_ns=STAMP,
            map_frame="lio_map", child_frame="go2_imu", stage="LIO")
META = dict(PAIR, cloud_topic="/rtx/cloud_registered_full", pose_topic="/rtx/aft_mapped_to_init",
            binary_sha256="a" * 64)


def json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def image_floor(floor_z=-.344):
    v, u = np.indices((256, 256))
    rays = np.stack(((u - 128) / (256 * 12 / 42), (v - 128) / (256 * 12 / 42), np.ones_like(u)), -1)
    base = rays @ nearfield.R_BASE_OPTICAL.T
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = (floor_z - nearfield.MOUNT[2]) / base[..., 2]
    depth[depth <= .05] = np.inf
    # A front-only camera fixture. Side/rear may never inherit front evidence.
    with np.errstate(invalid="ignore"):
        xyz = base * depth[..., None] + nearfield.MOUNT
    depth[np.abs(np.degrees(np.arctan2(xyz[..., 1], xyz[..., 0]))) >= 44.] = np.inf
    return depth.astype(np.float32)


def archive(directory, *, stamp=STAMP, arrival=100., depth=None, ids=None):
    labels = {"2": "/World/environment/mesh", "3": "/World/Go2/Geometry/base"}
    segmentation = {"data": np.full((256, 256), 2, np.uint32) if ids is None else ids,
                    "info": {"idToLabels": labels}}
    arrays, masks = nearfield.backproject(image_floor() if depth is None else depth, segmentation)
    record = dict(schema=nearfield.SCHEMA, source=nearfield.SOURCE, stamp_ns=stamp,
                  source_time_s=stamp * 1e-9, arrival_wall_s=arrival, arrival_clock="perf_counter",
                  frame_id="go2_base", calibration=nearfield.calibration_receipt(),
                  calibration_verified_from_render_parameters=True, depth_type="distance_to_image_plane",
                  native_reference_time=[stamp, 1_000_000_000], depth_reference_time=[stamp, 1_000_000_000],
                  mask_reference_time=[stamp, 1_000_000_000],
                  time_evidence={"event_reference": [stamp, 1_000_000_000], "body_now_substituted": False, "reference_after_reads": [stamp, 1_000_000_000]},
                  self_mask=dict(masks, method="same_frame_instance_prim_path", same_frame=True,
                                 robot_prim_root=nearfield.ROBOT_ROOT, instance_id_to_prim=labels,
                                 mapping_sha256=json_sha(labels)),
                  environment_ground_truth_used=False, world_pose_used=False, slam_input=False, map_input=False)
    return nearfield.write_frame(directory, arrays, record)


def native_floor():
    angle, radius = np.meshgrid(np.deg2rad(np.arange(-180., 180., 1.)), np.arange(1.2, 3.6, .04))
    return np.column_stack((np.cos(angle).ravel() * radius.ravel(),
                            np.sin(angle).ravel() * radius.ravel(), np.full(angle.size, -.344)))


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.path = self.directory / "latest.json"
        self.record = archive(self.directory)
        self.points = native_floor()
        self.native = build_perception_payload(self.points, stamp_s=10., arrival_wall_s=100., producer_metadata=META)
        self.assertIsNone(self.native["clearance_m"])

    def tearDown(self):
        self.temp.cleanup()

    def fuse(self, **changes):
        options = dict(pair=PAIR, pose_arrival_wall_s=100., depth_manifest=self.path,
                       now_sim_s=10.1, now_wall_s=100.1, enabled=True)
        options.update(changes)
        return build_safety_fusion(self.points, self.native, **options)

    def rewrite(self, mutate):
        record = json.loads(self.path.read_text()); mutate(record)
        self.path.write_text(json.dumps(record))

    def test_default_is_diagnostic_only(self):
        result = self.fuse(enabled=False)
        self.assertEqual(result["source"], self.native["source"])
        self.assertIsNone(result["clearance_m"])
        self.assertTrue(result["depth_safety_diagnostic"]["accepted"])
        self.assertFalse(result["depth_safety_diagnostic"]["control_used"])
        self.assertEqual(result["directions"], self.native["directions"])

    def test_valid_exact_frame_supports_front_not_unknown_sides(self):
        result = self.fuse()
        self.assertEqual(result["source"], FUSION_SOURCE, result.get("depth_safety_diagnostic"))
        self.assertEqual(result["directions"]["front"]["observation_state"], "measured")
        for name in ("rear", "left", "right"):
            self.assertEqual(result["directions"][name]["observation_state"], "insufficient_observation")
        self.assertEqual(result["corridor"]["forward_m"], [.35, 3.])
        self.assertEqual(result["corridor"]["radial_bins"], 4)
        self.assertTrue(all(n >= 10 for n in result["directions"]["front"]["radial_bin_floor_points"]))
        self.assertFalse(result["safety_fusion"]["depth_enters_slam"])
        self.assertTrue(result["safety_fusion"]["operator_map_native_only"])
        self.assertNotIn("lidar_geometry", result)
        self.assertEqual(result["safety_fusion"]["native_cloud_points"], len(self.points))
        self.assertTrue(validate_fusion_contract(result, now_sim_s=10.1, now_wall_s=100.1))
        self.assertNotIn("depth_safety_diagnostic", self.native)

    def test_missing_depth_preserves_original_clock_and_unknown(self):
        result = self.fuse(depth_manifest=self.directory / "missing.json")
        self.assertEqual(result["stamp_s"], 10.)
        self.assertEqual(result["arrival_wall_s"], 100.)
        self.assertEqual(result["directions"], self.native["directions"])
        self.assertFalse(result["depth_safety_diagnostic"]["accepted"])

    def test_latest_newer_uses_only_exact_immutable_frame(self):
        archive(self.directory, stamp=STAMP + 100_000_000, arrival=100.02)
        result = self.fuse()
        self.assertEqual(result["source"], FUSION_SOURCE)
        depth = result["safety_fusion"]["sources"]["depth"]
        self.assertEqual(depth["stamp_ns"], STAMP)
        self.assertEqual(depth["arrival_wall_s"], 100.)
        self.assertEqual(depth["latest_minus_lio_ns"], 100_000_000)

    def test_one_nanosecond_mismatch_is_not_interpolated(self):
        self.rewrite(lambda r: r.update(stamp_ns=STAMP + 1))
        (self.directory / ("frame_%019d.json" % STAMP)).unlink()
        result = self.fuse()
        self.assertIn("latest_minus_lio_ns=1", result["depth_safety_diagnostic"]["reason"])
        self.assertEqual(result["directions"], self.native["directions"])

    def test_each_invalid_provenance_field_fails_closed(self):
        original = self.path.read_text()
        cases = [lambda r: r.update(frame_id="map"), lambda r: r.update(source="rgb_guess"),
                 lambda r: r.update(arrival_clock="time.time"),
                 lambda r: r.update(environment_ground_truth_used=True),
                 lambda r: r.update(world_pose_used=True),
                 lambda r: r.update(source_time_s=9.9),
                 lambda r: r.update(mask_reference_time=[11, 1]),
                 lambda r: r["time_evidence"].update(body_now_substituted=True),
                 lambda r: r["calibration"].update(translation_base_m=[.9, 0., .18]),
                 lambda r: r["self_mask"].update(legacy_self_box_used=True),
                 lambda r: r["self_mask"].update(mapping_sha256="f" * 64),
                 lambda r: r.update(npz_sha256="0" * 64),
                 lambda r: r.update(arrival_wall_s=99.), lambda r: r.update(arrival_wall_s=101.)]
        for mutate in cases:
            with self.subTest(mutate=mutate):
                self.path.write_text(original); self.rewrite(mutate)
                result = self.fuse()
                self.assertFalse(result["depth_safety_diagnostic"]["accepted"])
                self.assertEqual(result["source"], self.native["source"])
                self.assertEqual(result["arrival_wall_s"], 100.)

    def test_raw_pixel_mask_and_endpoint_tampering_refused_even_with_new_sha(self):
        path = Path(self.record["npz_path"])
        with np.load(path, allow_pickle=False) as z:
            original = {k: z[k] for k in z.files}
        for key in ("points_base_m", "pixel_valid", "pixel_unknown", "point_pixel_flat_indices"):
            with self.subTest(key=key):
                arrays = copy.deepcopy(original)
                arrays[key].flat[0] = 123 if arrays[key].dtype.kind != "b" else not arrays[key].flat[0]
                np.savez(path, **arrays)
                self.rewrite(lambda r: r.update(npz_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
                result = self.fuse()
                self.assertIn("reprojection_or_mask", result["depth_safety_diagnostic"]["reason"])

    def test_unknown_no_returns_and_robot_occlusion_never_supply_floor(self):
        for label in (3, 99):
            with tempfile.TemporaryDirectory() as tmp:
                archive(tmp, ids=np.full((256, 256), label, np.uint32))
                result = self.fuse(depth_manifest=Path(tmp) / "latest.json")
                self.assertFalse(result["depth_safety_diagnostic"]["accepted"])
                self.assertIsNone(result["clearance_m"])
        with tempfile.TemporaryDirectory() as tmp:
            archive(tmp, depth=np.full((256, 256), np.inf, np.float32))
            result = self.fuse(depth_manifest=Path(tmp) / "latest.json")
            self.assertFalse(result["depth_safety_diagnostic"]["accepted"])

    def test_native_future_expired_pose_or_binary_identity_cannot_be_rescued(self):
        for change in (dict(now_sim_s=9.9), dict(now_sim_s=10.251), dict(now_wall_s=100.501),
                       dict(pose_arrival_wall_s=99.), dict(pose_arrival_wall_s=101.)):
            with self.subTest(change=change):
                self.assertFalse(self.fuse(**change)["depth_safety_diagnostic"]["accepted"])
        self.native["producer_metadata"]["binary_sha256"] = None
        self.assertIn("binary_identity", self.fuse()["depth_safety_diagnostic"]["reason"])

    def test_tracker_checks_source_age_not_just_relabelled_top_level_age(self):
        result = self.fuse()
        result["safety_fusion"]["sources"]["depth"]["arrival_wall_s"] = 90.
        with self.assertRaisesRegex(ValueError, "expired"):
            validate_fusion_contract(result, now_sim_s=10.1, now_wall_s=100.1)

    def test_tracker_rechecks_fusion_contract_and_original_floor_bins(self):
        result = self.fuse()
        tracker = object.__new__(InteractiveWaypointTracker)
        tracker._perception = None
        tracker.config = dict(require_perception=True, future_tolerance_s=2e-6,
                              perception_max_wall_age_s=.5, perception_max_sim_age_s=.3,
                              perception_base_frame="go2_base", min_cloud_points=1000,
                              min_clearance_m=.8, min_depth_valid_ratio=.9)
        sample = dict(result, points=result["corridor_points"])
        tracker._read_perception_file = lambda now: sample
        self.assertIsNone(tracker._perception_gate(100.1, 10.1))
        sample["directions"]["front"]["radial_bin_floor_points"][0] = 0
        self.assertEqual(tracker._perception_gate(100.1, 10.1)[0], "perception_floor_unsupported")
        sample["safety_fusion"] = None
        self.assertEqual(tracker._perception_gate(100.1, 10.1)[0], "perception_fusion_invalid")
        self.assertFalse(tracker._floor_supported({}, FUSION_SOURCE))

    def test_source_name_or_corridor_change_cannot_bypass_checks(self):
        result = self.fuse()
        for mutate in (lambda p: p.update(source="rtx_depth_safety_fusion_v2"),
                       lambda p: p["corridor"].update(forward_m=[1., 3.]),
                       lambda p: p["directions"]["front"].update(azimuth_coverage_fraction=.1),
                       lambda p: p["safety_fusion"]["sources"]["depth"].update(stamp_ns=STAMP - 1)):
            p = copy.deepcopy(result); mutate(p)
            with self.assertRaises(ValueError):
                validate_fusion_contract(p, now_sim_s=10.1, now_wall_s=100.1)

    def test_cli_has_two_independent_opt_ins_and_rejects_bad_combination_before_ros(self):
        base = ["map", "--run-dir", str(self.directory), "--cloud-topic", "/rtx/cloud_registered_full",
                "--odometry-topic", "/rtx/aft_mapped_to_init"]
        args = client.build_parser().parse_args(base)
        self.assertIsNone(args.nearfield_depth_manifest)
        self.assertFalse(args.enable_nearfield_depth_fusion)
        args = client.build_parser().parse_args(base + ["--enable-nearfield-depth-fusion"])
        args.mailbox = None
        with self.assertRaisesRegex(ValueError, "requires --nearfield"):
            client._cmd_map(args)

    def bridge_fusion(self, *, enabled, manifest=None, pump_action=None):
        output = self.directory / "bridge-perception.json"
        pose = dict(position=[0., 0., 0.], quat_xyzw=[0., 0., 0., 1.], arrival_wall_s=100.)
        args = SimpleNamespace(imu_mount=[0., 0., 0.], perception_out=output,
                               cloud_topic=META["cloud_topic"], odometry_topic=META["pose_topic"],
                               nearfield_depth_manifest=self.path if manifest is None else manifest,
                               enable_nearfield_depth_fusion=enabled)
        state = dict(source_binary_sha256="a" * 64, clock=10.1, clock_arrival_wall=100.)
        wall = [100.1]
        if pump_action is not None:
            state["pump_callbacks"] = lambda: pump_action(state, wall)
        from embodied_navigation.simulation.dynanav import rtx_deskewed_perception as rtx
        with mock.patch.object(rtx, "fit_floor_plane", wraps=rtx.fit_floor_plane) as fit:
            with mock.patch.object(client.time, "perf_counter", side_effect=lambda: wall[0]):
                client._write_deskewed_perception(self.points, pose, args, state,
                                                 stamp_s=10., arrival_wall=100., pair=PAIR)
        return json.loads(output.read_text()), fit.call_count

    def test_enabled_valid_depth_fits_joint_once_and_never_native_fallback(self):
        result, calls = self.bridge_fusion(enabled=True)
        self.assertEqual(calls, 1)
        self.assertEqual(result["source"], FUSION_SOURCE)
        self.assertEqual(result["safety_fusion"]["native_cloud_points"], len(self.points))
        self.assertEqual(result["directions"]["front"]["observation_state"], "measured")

    def test_diagnostic_only_keeps_native_output_and_both_independent_fits(self):
        result, calls = self.bridge_fusion(enabled=False)
        self.assertEqual(calls, 2)
        self.assertEqual(result["source"], self.native["source"])
        self.assertEqual(result["directions"], self.native["directions"])
        self.assertFalse(result["depth_safety_diagnostic"]["control_used"])

    def test_enabled_invalid_depth_lazily_fits_native_once(self):
        result, calls = self.bridge_fusion(enabled=True, manifest=self.directory / "missing.json")
        self.assertEqual(calls, 1)
        self.assertEqual(result["source"], self.native["source"])
        self.assertEqual(result["directions"], self.native["directions"])
        self.assertFalse(result["depth_safety_diagnostic"]["accepted"])
        self.assertEqual(result["arrival_wall_s"], 100.)

    def test_new_depth_second_level_stays_joint_unknown_without_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive(tmp, depth=image_floor(-.65))
            result, calls = self.bridge_fusion(enabled=True, manifest=Path(tmp) / "latest.json")
        self.assertEqual(calls, 1)
        self.assertEqual(result["source"], FUSION_SOURCE)
        self.assertEqual(result["directions"]["front"]["observation_state"], "insufficient_observation")
        self.assertIsNone(result["clearance_m"])
        self.assertTrue(result["fail_closed"])

    def test_independent_broad_layers_remain_ambiguous_in_single_joint_fit(self):
        lower = self.points.copy()
        lower[:, 2] -= .30
        self.points = np.concatenate([self.points, lower])
        result, calls = self.bridge_fusion(enabled=True)
        self.assertEqual(calls, 1)
        self.assertEqual(result["source"], FUSION_SOURCE)
        self.assertEqual(result["safety_geometry"]["floor_evidence"]["reason"], "floor_ambiguous_surfaces")
        self.assertIsNone(result["clearance_m"])
        self.assertTrue(result["fail_closed"])

    def test_postcompute_real_new_clock_exposes_expiry_without_native_refit(self):
        def pump(state, wall):
            wall[0] = 100.15
            state.update(clock=10.4, clock_arrival_wall=100.15)
            return {"spin_calls": 8, "callback_count": 2, "max_spin_calls": 8, "timeout_sec": 0.}
        result, calls = self.bridge_fusion(enabled=True, pump_action=pump)
        self.assertEqual(calls, 1)
        self.assertEqual(result["source"], FUSION_SOURCE)
        self.assertIsNone(result["clearance_m"])
        self.assertTrue(result["fail_closed"])
        diagnostic = result["depth_safety_diagnostic"]
        self.assertTrue(diagnostic["accepted_input"])
        self.assertFalse(diagnostic["accepted_post_compute"])
        self.assertFalse(diagnostic["control_used"])
        self.assertEqual(diagnostic["reason"], "post_compute:cloud_sim_expired")
        self.assertEqual(result["stamp_s"], 10.)
        self.assertEqual(result["arrival_wall_s"], 100.)
        self.assertEqual(diagnostic["clock_before_compute"]["source_time_s"], 10.1)
        self.assertEqual(diagnostic["clock_after_pump"]["source_time_s"], 10.4)
        self.assertEqual(diagnostic["post_compute_pump"]["spin_calls"], 8)

    def test_postcompute_wall_expiry_cannot_be_renewed_by_fresh_clock_callback(self):
        def pump(state, wall):
            wall[0] = 100.6
            state.update(clock=10.1, clock_arrival_wall=100.6)
            return {"spin_calls": 8, "callback_count": 1, "max_spin_calls": 8, "timeout_sec": 0.}
        result, calls = self.bridge_fusion(enabled=True, pump_action=pump)
        self.assertEqual(calls, 1)
        self.assertEqual(result["depth_safety_diagnostic"]["reason"], "post_compute:cloud_wall_expired")
        self.assertFalse(result["depth_safety_diagnostic"]["accepted_post_compute"])
        for source in result["safety_fusion"]["sources"].values():
            self.assertEqual(source["arrival_wall_s"], 100.)
            self.assertEqual(source["stamp_ns"], STAMP)
        self.assertTrue(result["fail_closed"])

    def test_valid_callback_clock_records_postcompute_without_extrapolation(self):
        def pump(state, wall):
            wall[0] = 100.2
            state.update(clock=10.2, clock_arrival_wall=100.2)
            return {"spin_calls": 8, "callback_count": 1, "max_spin_calls": 8, "timeout_sec": 0.}
        result, calls = self.bridge_fusion(enabled=True, pump_action=pump)
        self.assertEqual(calls, 1)
        diagnostic = result["depth_safety_diagnostic"]
        self.assertTrue(diagnostic["accepted_post_compute"])
        self.assertEqual(diagnostic["post_compute_wall_s"], 100.2)
        self.assertFalse(diagnostic["clock_after_pump"]["clock_extrapolated"])
        self.assertIn("not physics-now", diagnostic["clock_after_pump"]["scope"])
        self.assertEqual(result["stamp_s"], 10.)
        self.assertEqual(result["arrival_wall_s"], 100.)

    def test_callback_pump_is_exactly_bounded_and_never_blocks(self):
        state = {"callback_count": 0}
        calls = []
        def spin_once(node, *, timeout_sec):
            calls.append((node, timeout_sec))
            state["callback_count"] += 1
        receipt = client._bounded_map_callback_pump(state, spin_once=spin_once, node="node", clock=lambda: 100.)
        self.assertEqual(calls, [("node", 0.)] * 8)
        self.assertEqual(receipt["callback_count"], 8)
        self.assertEqual(receipt["spin_calls"], 8)

    def test_callback_arrivals_do_not_extend_fixed_consumption_batch(self):
        pending = ["old1", "old2"]
        seen = []
        def consume(batch):
            self.assertIsInstance(batch, tuple)
            for cloud in batch:
                seen.append(cloud)
                pending.append("new:" + cloud)
            return ["old2"]
        receipt = client._consume_fixed_cloud_batch(pending, consume)
        self.assertEqual(seen, ["old1", "old2"])
        self.assertEqual(pending, ["old2", "new:old1", "new:old2"])
        self.assertEqual(receipt["batch_count"], 2)
        self.assertEqual(receipt["received_during_batch"], 2)

    def test_clock_snapshot_refuses_stale_future_or_missing_callback_receipt(self):
        for state in ({}, {"clock": 10., "clock_arrival_wall": 99.},
                      {"clock": 10., "clock_arrival_wall": 101.}):
            snapshot = client._map_clock_snapshot(state, 100.)
            self.assertFalse(snapshot["usable"])
            self.assertFalse(snapshot["clock_extrapolated"])

    def test_compact_mask_drops_no_pixel_verification_and_keeps_mapping_identity(self):
        result = self.fuse()
        depth = result["safety_fusion"]["sources"]["depth"]
        self.assertNotIn("instance_id_to_prim", depth["self_mask"])
        self.assertEqual(depth["self_mask"]["mapping_sha256"], depth["segmentation_mapping_sha256"])
        self.assertIn("instance_id_to_prim", json.loads(self.path.read_text())["self_mask"])
        depth["segmentation_mapping_sha256"] = "1" * 64
        with self.assertRaisesRegex(ValueError, "depth_contract"):
            validate_fusion_contract(result, now_sim_s=10.1, now_wall_s=100.1)

    def test_bridge_default_does_not_read_depth_and_preserves_native_map_input(self):
        output = self.directory / "perception.json"
        pose = dict(position=[0., 0., 0.], quat_xyzw=[0., 0., 0., 1.], arrival_wall_s=100.)
        args = SimpleNamespace(imu_mount=[0., 0., 0.], perception_out=output,
                               cloud_topic=META["cloud_topic"], odometry_topic=META["pose_topic"])
        state = {"source_binary_sha256": "a" * 64}
        before = self.points.copy()
        with mock.patch("embodied_navigation.simulation.dynanav.depth_safety_fusion.load_depth_frame",
                        side_effect=AssertionError("default must not read depth")):
            client._write_deskewed_perception(self.points, pose, args, state, stamp_s=10., arrival_wall=100., pair=PAIR)
        result = json.loads(output.read_text())
        self.assertEqual(result["source"], "rtx_lidar_cloud_registered_full")
        np.testing.assert_array_equal(self.points, before)
        self.assertNotIn("safety_fusion", result)


if __name__ == "__main__":
    unittest.main()
