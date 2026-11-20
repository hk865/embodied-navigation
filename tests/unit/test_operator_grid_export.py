"""Optional operator grid export and read-only PNG renderer (CPU only)."""

import hashlib
import importlib.util
import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

import explore_waypoint_client as client
from embodied_navigation.simulation.dynanav import operator_map as om

REPO = Path(__file__).resolve().parents[2]
RENDERER_PATH = REPO / "scripts" / "render_dynanav_operator_grid.py"
RES = 0.1


def load_renderer():
    spec = importlib.util.spec_from_file_location("grid_renderer", RENDERER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(position=(0.0, 0.0, 0.0)):
    return om.transform_from_position_quaternion(position, (0.0, 0.0, 0.0, 1.0))


def block(i0, j0, nx, ny, z, step=RES):
    xs = ((i0 + np.arange(nx)) + 0.5) * step
    ys = ((j0 + np.arange(ny)) + 0.5) * step
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, float(z))], axis=1)


def floor_with_hole():
    """A free patch with one missing cell (an unknown hole) plus one obstacle."""
    points = block(-40, -40, 40, 40, 0.0)
    keep = ~((points[:, 0] > 0.0) & (points[:, 0] < 0.1) & (points[:, 1] > 0.0) & (points[:, 1] < 0.1))
    return np.vstack([points[keep], block(80, 0, 1, 1, 0.20)])


class OperatorGridExportTests(unittest.TestCase):
    def _accumulator(self, points=None):
        accumulator = om.OccupancyAccumulator(
            extrinsic=om.Extrinsic(np.eye(4)), resolution_m=RES, free_mode="floor"
        )
        receipt = accumulator.add_frame(
            floor_with_hole() if points is None else points,
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.4)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=time.monotonic(),
        )
        self.assertTrue(receipt["accepted"])
        return accumulator

    def _export(self, accumulator, directory, *, event="operator_map_partial", **overrides):
        snapshot = accumulator.snapshot(0, frontier_limit=5)
        kwargs = dict(
            event=event,
            written_wall_s=1234.5,
            map_frame=accumulator.map_frame,
            odometry_map_frame="dynanav_lio_map",
            source_clock_used=10.0,
            source_clock_note="current_clock_sample",
            frontier=snapshot["frontier"],
            partial=event != "operator_map_accumulated",
        )
        kwargs.update(overrides)
        return client._export_operator_grid(accumulator, 0, 0.4, directory, **kwargs)

    def test_export_copies_the_real_cells_without_changing_the_map(self):
        accumulator = self._accumulator()
        before = dict(accumulator.grid_for(0).cells())
        before_snapshot = accumulator.snapshot(0, frontier_limit=5)
        with tempfile.TemporaryDirectory() as temporary:
            info = self._export(accumulator, temporary)
            with np.load(info["npz"]) as data:
                cells = accumulator.grid_for(0).cells()
                expected = sorted(cells)
                self.assertEqual([int(v) for v in data["ix"]], [k[0] for k in expected])
                self.assertEqual([int(v) for v in data["iy"]], [k[1] for k in expected])
                self.assertEqual([int(v) for v in data["code"]], [cells[k] for k in expected])
            manifest = json.loads(Path(info["manifest"]).read_text())
            self.assertEqual("operator_grid_export/v1", manifest["schema"])
            self.assertEqual("dynanav_lio_map", manifest["odometry_map_frame"])
            self.assertAlmostEqual(0.0, manifest["classification_floor_z_m"])
            self.assertEqual(manifest["counts"]["total"], len(expected))
            self.assertGreaterEqual(manifest["counts"]["occupied"], 1)
            self.assertLess(manifest["bounds"]["min_ix"], 0)
            self.assertEqual(
                hashlib.sha256(Path(info["npz"]).read_bytes()).hexdigest(),
                manifest["npz_sha256"],
            )
        self.assertEqual(before, accumulator.grid_for(0).cells())
        after_snapshot = accumulator.snapshot(0, frontier_limit=5)
        self.assertEqual(
            before_snapshot["frontier"]["actionable"], after_snapshot["frontier"]["actionable"]
        )

    def test_export_records_actual_lio_sources_without_rgbd_or_vio_claims(self):
        from types import SimpleNamespace

        accumulator = self._accumulator()
        args = SimpleNamespace(
            cloud_topic="/dynanav/rtx_livo/cloud_registered_full",
            odometry_topic="/dynanav/rtx_livo/aft_mapped_to_init",
            pose_source="lio_estimate", sensor_frame="go2_imu",
            base_frame="go2_base", frontier_limit=5,
        )
        pair = dict(stage="LIO", cloud_stamp_ns=10000000000,
                    pose_stamp_ns=10000000000, map_frame="dynanav_lio_map")
        state = dict(cloud_semantics="lio_deskewed_map", deskewed_stage="LIO",
                     last_pair_contract=pair, odometry_map_frame="dynanav_lio_map",
                     free_space_policy=client._operator_map_free_policy("lio_deskewed_map"),
                     declared_frame=None, frame_mismatch=None, frames=1,
                     accepted=1, rejected=0, last=None)
        with tempfile.TemporaryDirectory() as temporary:
            args.export_grid = Path(temporary) / "grid"
            result = client._write_operator_map_snapshots(
                accumulator, Path(temporary) / "map", args, state,
                event="operator_map_partial",
            )
            level = next(iter(result["levels"].values()))
            manifest = json.loads(Path(level["grid_export"]["manifest"]).read_text())
        self.assertIn("native deskewed LIO map-frame", manifest["provenance"])
        self.assertNotIn("RGB-D", manifest["provenance"])
        self.assertNotIn("post-VIO", manifest["provenance"])
        sources = manifest["source_contract"]
        self.assertEqual(args.cloud_topic, sources["cloud_topic"])
        self.assertEqual(args.odometry_topic, sources["odometry_topic"])
        self.assertEqual(pair, sources["last_pair"])
        self.assertEqual(state["free_space_policy"], manifest["free_space_semantics"]["policy"])
        self.assertEqual(state["free_space_policy"], result["free_space_policy"])
        self.assertEqual("floor", manifest["free_space_semantics"]["free_mode"])
        # Synthetic accumulator defaults are deliberately different from the
        # configured topics. Preserve them as observations, never overwrite them.
        self.assertEqual(accumulator.snapshot(0)["sources"], sources["snapshot_sources"])
        self.assertNotEqual(sources["cloud_topic"], sources["snapshot_sources"]["cloud_source"])

    def test_sensor_coordinates_do_not_claim_rgbd_or_post_vio_provenance(self):
        from types import SimpleNamespace

        accumulator = self._accumulator()
        args = SimpleNamespace(cloud_topic="/some/cloud", odometry_topic="/some/pose",
                               pose_source="estimator")
        contract = client._operator_grid_source_contract(
            args, {"cloud_semantics": "sensor_imu"}, accumulator.snapshot(0))
        with tempfile.TemporaryDirectory() as temporary:
            result = self._export(accumulator, temporary, source_contract=contract)
            manifest = json.loads(Path(result["manifest"]).read_text())
        self.assertIn("modality/stage not inferred", manifest["provenance"])
        self.assertNotIn("RGB-D", manifest["provenance"])
        self.assertNotIn("post-VIO", manifest["provenance"])
        self.assertIsNone(manifest["source_contract"]["last_pair"])

    def test_export_without_source_evidence_does_not_invent_provenance(self):
        accumulator = self._accumulator()
        with tempfile.TemporaryDirectory() as temporary:
            result = self._export(accumulator, temporary)
            manifest = json.loads(Path(result["manifest"]).read_text())
        self.assertEqual({}, manifest["source_contract"])
        self.assertIn("modality/stage not inferred", manifest["provenance"])
        self.assertFalse(manifest["free_space_semantics"]["free_is_traversability_certificate"])
        self.assertEqual("floor", manifest["free_space_semantics"]["free_mode"])

    def test_projected_ray_free_is_not_ground_support(self):
        # A distant floor patch establishes the level, while an elevated hit
        # carves an XY ray over the unobserved near field. No floor point hits
        # the queried cell. This reproduces why FREE alone is not traversability.
        points = np.vstack([block(20, 20, 10, 10, 0.0), [[3.05, 0.05, 0.8]]])
        grids = {}
        for mode in ("floor", "both"):
            acc = om.OccupancyAccumulator(
                extrinsic=om.Extrinsic(np.eye(4)), resolution_m=RES,
                free_mode=mode, min_points=50,
            )
            receipt = acc.add_frame(
                points, frame="map", cloud_stamp_s=10.0,
                pose=pose((0.0, 0.0, 0.4)), pose_stamp_s=10.0,
                sim_age_s=0.0, wall_age_s=0.0,
            )
            self.assertTrue(receipt["accepted"], receipt)
            grids[mode] = acc.grid_for(0)
        self.assertEqual(om.UNKNOWN, grids["floor"].code_at(10, 0))
        self.assertEqual(om.FREE, grids["both"].code_at(10, 0))
        self.assertEqual(om.UNKNOWN, grids["both"].code_at(10, 5))

    def test_the_unknown_hole_is_grey_and_not_free(self):
        accumulator = self._accumulator()
        with tempfile.TemporaryDirectory() as temporary:
            info = self._export(accumulator, temporary)
            png = Path(temporary) / "grid.png"
            renderer = load_renderer()
            render = renderer.render_grid(info["npz"], info["manifest"], png)
            manifest = json.loads(Path(info["manifest"]).read_text())
            image = Image.open(png).convert("RGB")
            holes = [key for key in ((0, 0), (0, 1), (1, 0), (1, 1))
                     if key not in accumulator.grid_for(0).cells()]
            self.assertTrue(holes)
            hole = holes[0]
            row, col = renderer.cell_pixel(
                hole[0], hole[1],
                manifest["bounds"]["min_ix"], manifest["bounds"]["max_ix"],
                manifest["bounds"]["min_iy"], manifest["bounds"]["max_iy"],
            )
            self.assertEqual(renderer.UNKNOWN_COLOR, image.getpixel((col + 1, row + 1)))
            self.assertNotEqual(renderer.FREE_COLOR, image.getpixel((col + 1, row + 1)))
            cells = accumulator.grid_for(0).cells()
            obstacle = next(key for key, value in cells.items() if value == om.OCCUPIED)
            free_cell = next(key for key, value in cells.items() if value == om.FREE)
            row, col = renderer.cell_pixel(
                obstacle[0], obstacle[1],
                manifest["bounds"]["min_ix"], manifest["bounds"]["max_ix"],
                manifest["bounds"]["min_iy"], manifest["bounds"]["max_iy"],
            )
            self.assertEqual(renderer.OCCUPIED_COLOR, image.getpixel((col + 1, row + 1)))
            row, col = renderer.cell_pixel(
                free_cell[0], free_cell[1],
                manifest["bounds"]["min_ix"], manifest["bounds"]["max_ix"],
                manifest["bounds"]["min_iy"], manifest["bounds"]["max_iy"],
            )
            self.assertEqual(renderer.FREE_COLOR, image.getpixel((col + 1, row + 1)))
            self.assertEqual(renderer.BACKGROUND, image.getpixel((2, 2)))
            self.assertLess(row, image.height - renderer.FOOTER_H)
            self.assertGreater(row, renderer.HEADER_H)

    def test_the_renderer_labels_the_real_odometry_frame_and_keeps_the_declared_one(self):
        accumulator = self._accumulator()
        with tempfile.TemporaryDirectory() as temporary:
            info = self._export(accumulator, temporary)
            png = Path(temporary) / "grid.png"
            render = load_renderer().render_grid(info["npz"], info["manifest"], png)
            self.assertEqual("dynanav_lio_map", render["frame_label"])
            self.assertEqual(accumulator.map_frame, render["declared_map_frame"])

    def test_the_renderer_refuses_a_stale_or_malformed_pair(self):
        accumulator = self._accumulator()
        with tempfile.TemporaryDirectory() as temporary:
            info = self._export(accumulator, temporary)
            renderer = load_renderer()
            png = Path(temporary) / "grid.png"
            manifest = json.loads(Path(info["manifest"]).read_text())
            # stale pair: modify the npz after the manifest hash was written
            np.savez(info["npz"], ix=np.array([0]), iy=np.array([0]), code=np.array([0]))
            with self.assertRaises(RuntimeError):
                renderer.render_grid(info["npz"], info["manifest"], png)
            # wrong schema
            broken = dict(manifest, schema="wrong/v0")
            Path(info["manifest"]).write_text(json.dumps(broken))
            with self.assertRaises(RuntimeError):
                renderer.render_grid(info["npz"], info["manifest"], png)
            # unknown code is not free
            np.savez(info["npz"], ix=np.array([0]), iy=np.array([0]), code=np.array([-1]))
            fixed = dict(manifest, npz_sha256=hashlib.sha256(Path(info["npz"]).read_bytes()).hexdigest())
            Path(info["manifest"]).write_text(json.dumps(fixed))
            with self.assertRaises(RuntimeError):
                renderer.render_grid(info["npz"], info["manifest"], png)
            # mismatched lengths
            np.savez(info["npz"], ix=np.array([0, 1]), iy=np.array([0]), code=np.array([0, 0]))
            fixed["npz_sha256"] = hashlib.sha256(Path(info["npz"]).read_bytes()).hexdigest()
            Path(info["manifest"]).write_text(json.dumps(fixed))
            with self.assertRaises(RuntimeError):
                renderer.render_grid(info["npz"], info["manifest"], png)

    def test_a_oversized_export_or_image_is_refused(self):
        accumulator = self._accumulator()
        original = client.OPERATOR_GRID_MAX_CELLS
        client.OPERATOR_GRID_MAX_CELLS = 5
        try:
            with tempfile.TemporaryDirectory() as temporary:
                with self.assertRaises(ValueError):
                    self._export(accumulator, temporary)
                self.assertTrue(accumulator.grid_for(0).cells())
        finally:
            client.OPERATOR_GRID_MAX_CELLS = original
        with tempfile.TemporaryDirectory() as temporary:
            info = self._export(accumulator, temporary)
            with self.assertRaises(RuntimeError):
                load_renderer().render_grid(
                    info["npz"], info["manifest"], Path(temporary) / "big.png", max_side_px=64
                )

    def test_an_empty_grid_exports_no_cells_and_renders_no_observed_map(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=om.Extrinsic(np.eye(4)), resolution_m=RES, free_mode="floor"
        )
        with tempfile.TemporaryDirectory() as temporary:
            info = self._export(accumulator, temporary)
            with np.load(info["npz"]) as data:
                self.assertEqual(0, int(data["ix"].size))
            manifest = json.loads(Path(info["manifest"]).read_text())
            self.assertEqual(0, manifest["counts"]["total"])
            png = Path(temporary) / "grid.png"
            render = load_renderer().render_grid(info["npz"], info["manifest"], png)
            self.assertEqual(0, render["cells"])
            self.assertEqual("no observed map", render["note"])

    def test_the_export_flag_defaults_to_off(self):
        parser = client.build_parser()
        parsed = parser.parse_args(
            ["map", "--duration", "1", "--odometry-topic", "/dynanav/livo_navigation/post_vio_odometry"]
        )
        self.assertIsNone(parsed.export_grid)
