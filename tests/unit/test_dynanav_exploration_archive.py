"""CPU-only provenance and pure-helper checks for recovered historical tools.

Never imports the top-level historical observer/collector scripts, starts ROS,
launches Isaac, or sends a navigation request.
"""

import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np


REPO = Path(__file__).resolve().parents[2]
MANIFEST = REPO / "docs/experiments/dynanav-exploration-tools-20261111.manifest.json"
ARCHIVE = REPO / "scripts/dynanav_exploration/archive_20261111"
RENDERERS = ARCHIVE / "coverage_111150"


class ExplorationArchiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads(MANIFEST.read_text())
        cls.cache = tempfile.TemporaryDirectory(prefix="dynanav-archive-tests-")
        cls.old_mpl = os.environ.get("MPLCONFIGDIR")
        os.environ["MPLCONFIGDIR"] = cls.cache.name
        spec = importlib.util.spec_from_file_location(
            "archived_dynanav_render_generation", RENDERERS / "render_generation.py"
        )
        cls.renderer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.renderer)

    @classmethod
    def tearDownClass(cls):
        if cls.old_mpl is None:
            os.environ.pop("MPLCONFIGDIR", None)
        else:
            os.environ["MPLCONFIGDIR"] = cls.old_mpl
        cls.cache.cleanup()

    def test_archive_bytes_match_manifest(self):
        self.assertEqual(len(self.manifest["files"]), 28)
        for entry in self.manifest["files"]:
            with self.subTest(path=entry["repository_path"]):
                path = (REPO / entry["repository_path"]).resolve()
                self.assertTrue(path.is_relative_to(ARCHIVE.resolve()))
                self.assertEqual(path.stat().st_size, entry["bytes"])
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                self.assertEqual(digest, entry["source_sha256"])
                self.assertEqual(digest, entry["archived_sha256"])

    def test_all_sources_parse_without_executing(self):
        count = 0
        for entry in self.manifest["files"]:
            path = REPO / entry["repository_path"]
            if path.suffix == ".py":
                ast.parse(path.read_text(), filename=str(path))
                count += 1
            else:
                self.assertEqual(path.suffix, ".json")
                json.loads(path.read_text())
        self.assertEqual(count, 20)

    def test_no_runtime_data_copied_or_model_authorship_inferred(self):
        self.assertFalse(self.manifest["source_thread"]["model_sku_verified"])
        self.assertTrue(self.manifest["source_thread"]["actual_invocations_verified"])
        self.assertTrue(all(not p["copied"] for p in self.manifest["external_evidence"]))
        self.assertTrue(all(p["bytes"] < 200000 for p in self.manifest["files"]))
        self.assertTrue(all(Path(p["repository_path"]).suffix in {".py", ".json"}
                            for p in self.manifest["files"]))

    def test_path_binding_rejects_cross_generation_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            allowed = root / "current"
            allowed.mkdir()
            binding = {"allowed_input_roots": [str(allowed)]}
            self.assertEqual(self.renderer.bound_path(allowed / "frame.json", binding),
                             allowed / "frame.json")
            with self.assertRaises(ValueError):
                self.renderer.bound_path(root / "previous/frame.json", binding)

    def test_trace_filters_source_frame_truth_and_future(self):
        base = dict(odom_stamp_s=1.0, aligned_position_xyz_m=[0, 0, 0],
                    aligned_yaw_rad=0.0, pose_source="livo_post_vio", no_gt_pose_used=True,
                    frame_id="dynanav_lio_map", child_frame_id="go2_imu")
        rows = [base, dict(base, odom_stamp_s=1.2, aligned_position_xyz_m=[1, 0, 0])]
        rows += [dict(base, odom_stamp_s=1.1, **change) for change in (
            {"pose_source": "truth"}, {"no_gt_pose_used": False},
            {"frame_id": "other"}, {"child_frame_id": "other"})]
        rows += [dict(base, odom_stamp_s=3.0)]
        binding = dict(pose_source="livo_post_vio", map_frame="dynanav_lio_map",
                       pose_child_frame="go2_imu")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "navigation.jsonl"
            path.write_text("malformed\n" + "\n".join(json.dumps(row) for row in rows))
            trace = self.renderer.load_trace(path, binding, cutoff=2.0)
        np.testing.assert_array_equal(trace[:, 0], [1.0, 1.2])
        np.testing.assert_array_equal(trace[:, 1], [0.0, 1.0])

    def test_trajectory_length_does_not_bridge_missing_intervals(self):
        trace = np.array([[0, 0, 0, 0, 0], [.2, 1, 0, 0, 0],
                          [2, 100, 0, 0, 0], [2.2, 101, 0, 0, 0]], dtype=float)
        self.assertEqual(len(self.renderer.segments(trace)), 2)
        self.assertAlmostEqual(self.renderer.measured_length(trace), 2.0)

    def test_receipt_prefix_does_not_claim_later_reach(self):
        accepted = dict(event="waypoint_accepted", seq=1, goal_xyz_m=[1, 2, 0])
        reached = dict(event="waypoint_reached", seq=1)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "receipts.jsonl"
            path.write_text("\n".join(json.dumps(row) for row in [accepted, reached]))
            audit = self.renderer.receipts(path, {"receipts": [accepted]})
        self.assertEqual(audit["goto_accepted"], 1)
        self.assertEqual(audit["goto_reached"], 0)
        self.assertEqual(audit["active_seq"], 1)

    def test_receipt_ambiguous_anchor_rejected(self):
        stop = dict(event="stop_accepted", seq=2)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "receipts.jsonl"
            path.write_text("\n".join(json.dumps(stop) for _ in range(2)))
            with self.assertRaisesRegex(ValueError, "Ambiguous"):
                self.renderer.receipts(path, {"receipts": [stop]})

    def test_full_bounds_retain_outliers_and_sampling_is_repeatable(self):
        points = np.array([[0, 0, 0], [1, 1, 1], [100, -100, 10]], dtype=float)
        bounds = self.renderer.bounds_from_points(points)
        self.assertTrue(np.all(bounds[:, 0] <= points.min(axis=0)))
        self.assertTrue(np.all(bounds[:, 1] >= points.max(axis=0)))
        cloud = np.arange(300).reshape(100, 3)
        np.testing.assert_array_equal(self.renderer.sample(cloud, 10),
                                      self.renderer.sample(cloud, 10))

    def test_only_guarded_renderer_clis_have_no_action_help(self):
        for name in ("render_generation.py", "render_observed_roi.py", "render_upper_region.py"):
            with self.subTest(script=name), tempfile.TemporaryDirectory() as temporary:
                env = dict(os.environ, MPLCONFIGDIR=temporary, PYTHONDONTWRITEBYTECODE="1")
                result = subprocess.run([sys.executable, "-B", str(RENDERERS / name), "--help"],
                                        cwd=temporary, env=env, text=True,
                                        capture_output=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("--binding", result.stdout)
                self.assertIn("--snapshot", result.stdout)


if __name__ == "__main__":
    unittest.main()
