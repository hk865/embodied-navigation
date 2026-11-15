"""Synthetic CPU contract tests; no simulated sensor/GPU pass is claimed."""
from collections import deque
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

from embodied_navigation.simulation.dynanav.capture.nearfield import (
    MOUNT, R_BASE_OPTICAL, RESOLUTION, backproject, calibration_receipt,
    point_coverage, validate_camera_parameters, verify_reference_pair, write_frame)
from embodied_navigation.simulation.dynanav.capture.nearfield_sensor import NearfieldDepthRender


def seg(ids=None, labels=None):
    return {"data": np.full((256, 256), 2, np.uint32) if ids is None else ids,
            "info": {"idToLabels": labels or {"2": "/World/environment/mesh", "3": "/World/Go2/Geometry/base"}}}


def parameters():
    return {"renderProductResolution": [256, 256], "cameraAperture": [42., 42.],
            "cameraFocalLength": 12., "cameraApertureOffset": [0., 0.],
            "metersPerSceneUnit": 1., "cameraModel": "pinhole", "cameraViewTransform": np.eye(4)}


def plane_image(floor_z=-.344):
    v, u = np.indices((256, 256))
    rays = np.stack(((u - 128) / (256 * 12 / 42), (v - 128) / (256 * 12 / 42), np.ones_like(u)), axis=-1)
    rays_base = rays @ R_BASE_OPTICAL.T
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = (floor_z - MOUNT[2]) / rays_base[..., 2]
    depth[depth <= .05] = np.inf
    return depth.astype(np.float32)


class NearfieldGeometryTests(unittest.TestCase):
    def test_calibration_is_right_handed_and_explicit_extra_hardware(self):
        np.testing.assert_allclose(R_BASE_OPTICAL.T @ R_BASE_OPTICAL, np.eye(3), atol=1e-15)
        self.assertAlmostEqual(np.linalg.det(R_BASE_OPTICAL), 1.)
        self.assertTrue(calibration_receipt()["simulated_hardware"])
        self.assertTrue(calibration_receipt()["physical_robot_requires_corresponding_downward_sensor"])

    def test_valid_floor_inside_legacy_self_box_survives(self):
        arrays, receipt = backproject(plane_image(), seg())
        xyz = arrays["points_base_m"]
        np.testing.assert_allclose(xyz[:, 2], -.344, atol=1e-6)
        self.assertTrue(np.any((xyz[:, 0] < .5) & (abs(xyz[:, 1]) < .3)))
        self.assertFalse(receipt["legacy_self_box_used"])

    def test_self_pixels_and_no_returns_stay_unknown(self):
        ids = seg()["data"]
        ids[120:135, 120:135] = 3
        depth = plane_image(); depth[:20, :] = np.inf
        arrays, receipt = backproject(depth, seg(ids))
        self.assertEqual(receipt["self_pixels"], 225)
        self.assertTrue(arrays["pixel_unknown"][120:135, 120:135].all())
        self.assertFalse(arrays["pixel_valid"][:20].any())
        self.assertFalse(np.any(arrays["pixel_self"].flat[arrays["point_pixel_flat_indices"]]))

    def test_missing_background_and_unlabelled_ids_are_unknown(self):
        ids = np.full((256, 256), 99, np.uint32)
        for label in ("BACKGROUND", "UNLABELLED", {"class": "floor"}, None):
            arrays, _ = backproject(plane_image(), seg(ids, {"99": label}))
            self.assertEqual(len(arrays["points_base_m"]), 0)
            self.assertTrue(arrays["pixel_unknown"].all())

    def test_robot_root_does_not_mask_similarly_named_environment(self):
        arrays, _ = backproject(plane_image(), seg(labels={"2": "/World/Go2_floor"}))
        self.assertGreater(len(arrays["points_base_m"]), 100)
        self.assertFalse(arrays["pixel_self"].any())

    def test_mask_shape_and_mapping_fail_closed(self):
        for segmentation in ({"data": np.zeros((1, 1)), "info": {}},
                             {"data": np.zeros((256, 256), np.uint32), "info": {}},
                             {"data": np.zeros((256, 256), float), "info": {"idToLabels": {"0": "/World/x"}}}):
            with self.assertRaises(ValueError):
                backproject(plane_image(), segmentation)

    def test_intrinsics_not_world_pose_drive_projection(self):
        native = parameters(); validate_camera_parameters(native)
        native["cameraViewTransform"] = np.full((4, 4), np.nan)
        validate_camera_parameters(native)
        native["cameraFocalLength"] = 24
        with self.assertRaises(ValueError): validate_camera_parameters(native)

    def test_intrinsics_wrong_model_resolution_scale_or_offset_fail(self):
        for key, value in (("cameraModel", "fisheye"), ("renderProductResolution", [512, 512]),
                           ("metersPerSceneUnit", .01), ("cameraApertureOffset", [0, .5])):
            native = parameters(); native[key] = value
            with self.assertRaises(ValueError): validate_camera_parameters(native)

    def test_ideal_plane_covers_nearfield_all_nine_front_angular_cells(self):
        # Ideal plane is test fixture, never evidence about the real basement.
        arrays, _ = backproject(plane_image(), seg())
        coverage = point_coverage(arrays["points_base_m"])
        counts = np.asarray(coverage["endpoint_counts"])
        self.assertTrue((counts[0] >= 10).all(), counts)
        self.assertTrue((counts[1:] >= 10).all(), counts)
        self.assertFalse(coverage["clearance_supported"])

    def test_center_ray_cannot_claim_full_front_sector(self):
        coverage = point_coverage(np.array([[.3, 0, -.344]] * 100))
        self.assertEqual(np.count_nonzero(coverage["endpoint_counts"][0]), 1)

    def test_exact_pixel_point_mapping_is_preserved(self):
        arrays, _ = backproject(plane_image(), seg())
        index = arrays["point_pixel_flat_indices"]
        np.testing.assert_array_equal(index, np.flatnonzero(arrays["pixel_valid"]))
        self.assertEqual(arrays["points_base_m"].shape, (len(index), 3))


class NearfieldClockArchiveTests(unittest.TestCase):
    def render(self):
        r = NearfieldDepthRender(None, None, None, SimpleNamespace(get_data=plane_image),
                                 SimpleNamespace(get_data=parameters))
        r.instance_ann = SimpleNamespace(get_data=seg)
        r.reference_ann = SimpleNamespace(get_data=lambda: {"referenceTimeNumerator": 2, "referenceTimeDenominator": 10})
        r.rtx_cache = deque()
        return r

    def accept(self, r, stamp=.2, reference=(2, 10)):
        # Exercise the archive/calibration layer with an explicitly owned,
        # pre-bound bundle. SDK callback binding has separate CPU tests.
        native = r.reference_ann.get_data()
        r._accept_native_render_frame(
            bound_data={"depth_ann": r.depth_ann.get_data().copy(),
                        "instance_ann": r.instance_ann.get_data(),
                        "camera_params_ann": r.camera_params_ann.get_data()},
            source_s=stamp, reference_time=reference, acquisition_physics_s=stamp + .005,
            time_evidence={"body_now_substituted": False, "original_arrival_wall_s": 42.,
                           "binding_after": {"reference_time": [native["referenceTimeNumerator"],
                                                                 native["referenceTimeDenominator"]]}})

    def test_native_reference_rational_equality(self):
        verify_reference_pair((2, 10), (1, 5))
        for after in ((0, 0), (3, 10), (1.0, 5), (None, 5)):
            with self.assertRaises(ValueError): verify_reference_pair((2, 10), after)

    def test_clock_is_native_and_no_control_or_slam_payload(self):
        r = self.render(); self.accept(r)
        arrays, record = r.rtx_cache[0]
        self.assertEqual(record["stamp_ns"], 200_000_000)
        self.assertEqual(record["source_time_s"], .2)
        self.assertFalse(record["control_enabled"])
        self.assertFalse(record["slam_input"])
        self.assertFalse(record["world_pose_used"])
        self.assertNotIn("cameraViewTransform", record)
        self.assertEqual(record["depth_reference_time"], record["mask_reference_time"])
        self.assertEqual(record["self_mask"]["instance_id_to_prim"], seg()["info"]["idToLabels"])

    def test_duplicate_and_before_minimum_ignored_backwards_rejected(self):
        r = self.render(); r.minimum_stamp_ns = 150_000_000
        self.accept(r, .1); self.assertFalse(r.rtx_cache)
        self.accept(r); self.accept(r); self.assertEqual(len(r.rtx_cache), 1)
        with self.assertRaisesRegex(ValueError, "backwards"): self.accept(r, .18)

    def test_mid_read_clock_mismatch_rejected(self):
        r = self.render()
        with self.assertRaisesRegex(ValueError, "crossed"): self.accept(r, reference=(3, 10))
        self.assertFalse(r.rtx_cache)

    def test_queue_full_refuses_silent_loss(self):
        r = self.render(); r.rtx_cache.extend([None] * 4)
        with self.assertRaisesRegex(RuntimeError, "overflow"): self.accept(r)

    def test_copies_pixels_and_archive_is_immutable_with_hash(self):
        r = self.render(); self.accept(r); arrays, record = r.rtx_cache[0]
        with TemporaryDirectory() as d:
            receipt = write_frame(d, arrays, record)
            data = Path(receipt["npz_path"])
            self.assertEqual(receipt["npz_sha256"], hashlib.sha256(data.read_bytes()).hexdigest())
            self.assertEqual(json.loads((Path(d) / "latest.json").read_text())["stamp_ns"], 200_000_000)
            self.assertTrue(data.with_suffix(".json").exists())
            with np.load(data, allow_pickle=False) as archive:
                self.assertEqual(int(archive["stamp_ns"]), 200_000_000)
                np.testing.assert_array_equal(archive["pixel_unknown"], ~archive["pixel_valid"])
            with self.assertRaises(ValueError): write_frame(d, arrays, record)

    def test_opt_in_default_does_not_import_isaac(self):
        from embodied_navigation.simulation.dynanav.capture.nearfield_sensor import create_nearfield_depth
        self.assertIsNone(create_nearfield_depth(None, None, {}))
        with self.assertRaises(ValueError):
            create_nearfield_depth(None, None, {"simulated_nearfield_depth": True})


if __name__ == "__main__":
    unittest.main()
