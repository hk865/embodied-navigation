import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    DYNAMIC,
    STATIC,
    UNKNOWN,
    LayeredMap,
    Observation,
    PointMask,
    SemanticScheduler,
    read_snapshot,
)


def observation(sequence=1, stamp=100, points=None, transform=None, epoch=1):
    return Observation(
        epoch,
        sequence,
        stamp,
        stamp,
        np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.0, 0.0, 0.0]])
        if points is None
        else points,
        np.eye(4) if transform is None else transform,
        f"kf:{sequence}",
        "imu",
        "calibration:v1",
    )


def mask(obs, states, *, instances=(), labels=None, kind="sim_oracle", produced=None):
    return PointMask(
        obs.epoch,
        obs.sequence,
        obs.source_ns,
        obs.source_ns if produced is None else produced,
        obs.frame_id,
        obs.calibration_id,
        np.array(states, dtype=np.uint8),
        {"kind": kind, "producer": "fixture"},
        instances,
        labels or {},
    )


class LayeredMapTests(unittest.TestCase):
    def test_dynamic_unknown_excluded_but_full_navigation_cloud_retained(self):
        store = LayeredMap(map_id="office")
        obs = observation()
        result = store.integrate(obs, mask(obs, [STATIC, DYNAMIC, UNKNOWN]))
        np.testing.assert_array_equal(result["navigation_points"], obs.points_sensor)
        np.testing.assert_array_equal(result["static_indices"], [0])
        np.testing.assert_array_equal(store.static_points(), obs.points_sensor[:1])
        self.assertEqual(len(result["dynamic_points"]), 1)
        self.assertEqual(len(result["unknown_points"]), 1)
        self.assertFalse(store.snapshot()["semantic_coverage_complete"])
        # Producer reuse cannot mutate the owned observation or admitted map.
        with self.assertRaises(ValueError):
            obs.points_sensor[0, 0] = 99

    def test_missing_expired_and_wrong_identity_masks_quarantine(self):
        for failure in (
            "missing",
            "expired",
            "sequence",
            "calibration",
            "point_count",
            "future",
        ):
            with self.subTest(failure=failure):
                store = LayeredMap(max_mask_age_ns=10)
                obs = observation()
                candidate = mask(obs, [STATIC] * 3)
                now = obs.source_ns
                if failure == "missing":
                    candidate = None
                elif failure == "expired":
                    now += 11
                elif failure == "sequence":
                    candidate = mask(observation(sequence=2), [STATIC] * 3)
                elif failure == "calibration":
                    candidate = PointMask(
                        1,
                        1,
                        100,
                        100,
                        "imu",
                        "other",
                        np.ones(3, dtype=int),
                        {"kind": "learned", "producer": "fixture"},
                    )
                elif failure == "point_count":
                    candidate = mask(obs, [STATIC])
                else:
                    candidate = mask(obs, [STATIC] * 3, produced=101)
                result = store.integrate(obs, candidate, now_ns=now)
                self.assertNotEqual(result["mask_status"], "accepted")
                self.assertEqual(len(result["unknown_points"]), 3)
                self.assertEqual(len(result["navigation_points"]), 3)
                self.assertEqual(len(store.static_points()), 0)
                self.assertFalse(store.snapshot()["complete"])

    def test_late_mask_replaces_only_its_own_static_contribution(self):
        store = LayeredMap()
        first = observation(points=[[0, 0, 0]])
        second = observation(2, 200, points=[[0, 0, 0], [5, 0, 0]])
        store.integrate(first, mask(first, [STATIC]))
        store.integrate(second, mask(second, [STATIC, STATIC]))
        self.assertTrue(
            store.revise_mask(
                first.observation_id, mask(first, [DYNAMIC], produced=250), now_ns=250
            )
        )
        # Shared static voxel from another observation must survive revocation.
        np.testing.assert_array_equal(store.static_points(), [[0, 0, 0], [5, 0, 0]])
        self.assertFalse(
            store.revise_mask(second.observation_id, mask(first, [DYNAMIC]), now_ns=250)
        )

    def test_standing_person_is_dynamic_and_ids_survive_detection_index_changes(self):
        store = LayeredMap(map_id="office")
        first = observation(points=[[0, 0, 0], [0.1, 0, 0], [4, 0, 0]])
        store.integrate(
            first,
            mask(
                first,
                [STATIC] * 3,
                instances=("x", "x", "wall"),
                labels={"x": "person", "wall": "wall"},
            ),
        )
        ident = store.query(label="person")["objects"][0]["object_id"]
        second = observation(
            2, 1_000_000_100, points=[[0.2, 0, 0], [0.3, 0, 0], [4, 0, 0]]
        )
        store.integrate(
            second,
            mask(
                second,
                [DYNAMIC, DYNAMIC, STATIC],
                instances=("new", "new", "w2"),
                labels={"new": "person", "w2": "wall"},
                kind="learned",
            ),
        )
        person = store.query(label="person")["objects"][0]
        self.assertEqual(person["object_id"], ident)
        self.assertEqual(len(person["history"]), 2)
        self.assertAlmostEqual(person["velocity"][0], 0.2)
        np.testing.assert_array_equal(store.static_points(), [[4, 0, 0]])
        self.assertEqual(person["provenance"]["kind"], "learned")
        self.assertEqual(
            store.snapshot()["provenance_summary"]["mask_kinds"],
            ["learned", "sim_oracle"],
        )

    def test_becoming_dynamic_revokes_older_object_contributions(self):
        store = LayeredMap()
        first = observation(points=[[0, 0, 0]])
        store.integrate(
            first,
            mask(first, [STATIC], instances=("chair0",), labels={"chair0": "chair"}),
        )
        second = observation(2, 200, points=[[0.1, 0, 0]])
        store.integrate(
            second,
            mask(second, [DYNAMIC], instances=("chair1",), labels={"chair1": "chair"}),
        )
        self.assertEqual(len(store.query()["objects"]), 1)
        self.assertEqual(len(store.static_points()), 0)

    def test_revision_reprojects_geometry_and_object_history_without_id_change(self):
        store = LayeredMap(map_id="office")
        obs = observation(points=[[0, 0, 0]])
        store.integrate(
            obs, mask(obs, [STATIC], instances=("desk",), labels={"desk": "desk"})
        )
        before = store.query()["objects"][0]["object_id"]
        changed = np.eye(4)
        changed[:3, 3] = [2, 0, 3]
        store.update_anchor(obs.anchor_id, changed, revision=1)
        result = store.query(center=[2, 0, 3], radius=0.1)["objects"]
        self.assertEqual(result[0]["object_id"], before)
        np.testing.assert_array_equal(result[0]["history"][0]["position"], [2, 0, 3])
        np.testing.assert_array_equal(store.static_points(), [[2, 0, 3]])
        with self.assertRaises(ValueError):
            store.update_anchor(obs.anchor_id, changed, revision=1)

    def test_same_xy_different_floors_do_not_merge_objects(self):
        store = LayeredMap()
        for seq, height in [(1, 0), (2, 3)]:
            obs = observation(seq, seq * 100, points=[[0, 0, height]])
            store.integrate(
                obs, mask(obs, [STATIC], instances=("door",), labels={"door": "door"})
            )
        self.assertEqual(len(store.query(label="door")["objects"]), 2)
        self.assertEqual(len(store.query(center=[0, 0, 3], radius=0.5)["objects"]), 1)

    def test_epoch_scoped_oracle_paths_override_ttl_centroid_and_class_association(
        self,
    ):
        store = LayeredMap(map_id="office", object_ttl_ns=100, association_distance=0.5)

        def integrate(seq, stamp, paths, points):
            obs = observation(seq, stamp, points=points)
            proof = PointMask(
                1,
                seq,
                stamp,
                stamp,
                "imu",
                "calibration:v1",
                np.full(len(points), DYNAMIC, dtype=np.uint8),
                {
                    "kind": "sim_oracle",
                    "producer": "isaac.instances",
                    "instance_id_scope": "epoch",
                },
                tuple(paths),
                dict.fromkeys(paths, "person"),
            )
            store.integrate(obs, proof)

        integrate(1, 100, ["/World/A", "/World/B"], [[0, 0, 0], [1, 0, 0]])
        before = {
            x["source_instance_id"]: x["object_id"] for x in store.query()["objects"]
        }
        # More than TTL elapsed; visible centroids exchange and shift far beyond gate.
        integrate(2, 1000, ["/World/B", "/World/A"], [[0, 0, 0], [10, 0, 0]])
        after = {
            x["source_instance_id"]: x["object_id"] for x in store.query()["objects"]
        }
        self.assertEqual(before, after)
        self.assertNotEqual(after["/World/A"], after["/World/B"])
        for obj in store.query()["objects"]:
            self.assertIn("not_rigid_body", obj["velocity_semantics"])
            entry = obj["history"][-1]
            self.assertEqual(entry["source_instance_id"], obj["source_instance_id"])
            self.assertEqual(
                (entry["epoch"], entry["frame_id"], entry["calibration_id"]),
                (1, "imu", "calibration:v1"),
            )

    def test_unknown_label_can_be_refined_without_allocating_a_new_id(self):
        store = LayeredMap()
        first = observation(points=[[0, 0, 0]])
        store.integrate(first, mask(first, [STATIC], instances=("old",)))
        ident = store.query()["objects"][0]["object_id"]
        second = observation(2, 200, points=[[0.1, 0, 0]])
        store.integrate(
            second, mask(second, [STATIC], instances=("new",), labels={"new": "desk"})
        )
        self.assertEqual(store.query(label="desk")["objects"][0]["object_id"], ident)

    def test_compact_metadata_and_save_do_not_materialize_static_map(self):
        store = LayeredMap()
        obs = observation(points=[[0, 0, 0]])
        proof = mask(obs, [STATIC], instances=("desk",), labels={"desk": "desk"})
        store.integrate(obs, proof)
        # Caller mutation must not rewrite already captured provenance.
        proof.provenance["kind"] = "learned"
        with patch.object(
            store,
            "static_points",
            side_effect=AssertionError("expensive full-map path"),
        ):
            result = store.summary()
            self.assertNotIn("static_points", result)
            self.assertNotIn("history", result["objects"][0])
            self.assertEqual(
                result["observation_point_counts"],
                {"static": 1, "dynamic": 0, "unknown": 0},
            )
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "summary.json"
                store.save(path, include_points=False)
                snapshot = read_snapshot(path)
                self.assertFalse(snapshot["geometry_included"])
                self.assertNotIn("static_points", snapshot)
                self.assertTrue(snapshot["provenance_summary"]["sim_oracle_used"])

    def test_capacity_is_explicit_and_navigation_is_not_dropped(self):
        store = LayeredMap(max_observations=1)
        first, second = observation(), observation(2, 200)
        store.integrate(first, mask(first, [STATIC] * 3))
        result = store.integrate(second, mask(second, [DYNAMIC] * 3))
        self.assertFalse(result["persisted"])
        self.assertEqual(len(result["navigation_points"]), 3)
        self.assertFalse(store.snapshot()["capacity_complete"])

    def test_mask_coverage_is_distinct_from_semantic_label_coverage(self):
        store = LayeredMap()
        obs = observation(points=[[0, 0, 0]])
        store.integrate(obs, mask(obs, [STATIC], instances=("geometry",)))
        summary = store.summary()
        self.assertTrue(summary["mask_coverage_complete"])
        self.assertFalse(summary["semantic_label_coverage_complete"])
        self.assertTrue(summary["semantic_coverage_complete_is_legacy_mask_alias"])
        store.revise_mask(
            obs.observation_id,
            mask(
                obs,
                [STATIC],
                instances=("geometry",),
                labels={"geometry": "scene_geometry"},
            ),
            now_ns=100,
        )
        summary = store.summary()
        self.assertTrue(summary["semantic_label_coverage_complete"])
        self.assertTrue(summary["provenance_summary"]["sim_oracle_used"])

    def test_snapshot_is_queryable_and_oracle_is_not_claimed_as_learned(self):
        store = LayeredMap(map_id="office")
        obs = observation(points=[[2, 0, 0]])
        store.integrate(
            obs, mask(obs, [STATIC], instances=("desk",), labels={"desk": "desk"})
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene_map.json"
            store.save(path)
            result = read_snapshot(path)
            self.assertEqual(
                result["objects"][0]["object_id"],
                store.query()["objects"][0]["object_id"],
            )
            self.assertTrue(result["provenance_summary"]["sim_oracle_used"])
            self.assertEqual(result["static_points"], [[2, 0, 0]])
            self.assertEqual(list(path.parent.glob(".scene_map_*")), [])

    def test_epoch_pose_and_monotonicity_contracts(self):
        store = LayeredMap()
        obs = observation()
        store.integrate(obs)
        for invalid in [obs, observation(2, 99), observation(2, 200, epoch=2)]:
            with self.assertRaises(ValueError):
                store.integrate(invalid)
        with self.assertRaises(ValueError):
            Observation(1, 2, 200, 199, [[0, 0, 0]], np.eye(4), "a", "imu", "cal")
        self.assertEqual(LayeredMap().query()["objects"], [])


class SchedulerTests(unittest.TestCase):
    def test_periodic_event_busy_and_late_completion(self):
        scheduler = SemanticScheduler(interval_ns=100, event_min_interval_ns=30)
        self.assertTrue(scheduler.request(0, epoch=1))
        self.assertFalse(scheduler.request(100, epoch=1))
        self.assertTrue(scheduler.complete(0, epoch=1))
        self.assertTrue(scheduler.request(100, epoch=1, event=False))
        self.assertTrue(scheduler.complete(100, epoch=1))
        self.assertFalse(scheduler.request(120, epoch=1, event=True))
        self.assertTrue(scheduler.request(130, epoch=1, event=True))
        self.assertTrue(scheduler.request(0, epoch=2))
        self.assertFalse(scheduler.complete(130, epoch=1))
        self.assertEqual(scheduler.pending, (2, 0))
        self.assertTrue(scheduler.complete(0, epoch=2))
        with self.assertRaises(ValueError):
            scheduler.request(0, epoch=1)

    def test_semantic_calls_are_not_per_frame(self):
        scheduler = SemanticScheduler(interval_ns=100, event_min_interval_ns=30)
        admitted = []
        for stamp in range(1000):
            if scheduler.request(stamp, epoch=1):
                admitted.append(stamp)
                scheduler.complete(stamp, epoch=1)
        self.assertEqual(admitted, list(range(0, 1000, 100)))


class QueryCliTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "scene_map.json"
        store = LayeredMap(map_id="office", object_ttl_ns=100)
        obs = observation(points=[[0, 0, 0], [0, 0, 3], [2, 0, 0]])
        store.integrate(
            obs,
            mask(
                obs,
                [STATIC, STATIC, DYNAMIC],
                instances=("floor0", "floor1", "walker"),
                labels={"floor0": "door", "floor1": "door", "walker": "person"},
            ),
        )
        store.save(self.path, now_ns=1000, include_points=False)
        self.script = Path(__file__).resolve().parents[2] / "scripts/query_scene_map.py"

    def cli(self, *args):
        return subprocess.run(
            [sys.executable, str(self.script), str(self.path), *args],
            check=False,
            capture_output=True,
            text=True,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
        )

    def test_label_spatial_filter_keeps_coordinate_time_and_provenance(self):
        result = self.cli("--label", "door", "--near", "0", "0", "3", "--radius", ".5")
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertEqual(len(data["objects"]), 1)
        self.assertEqual(data["objects"][0]["source_instance_id"], "floor1")
        self.assertEqual(data["map"]["frame_id"], "map")
        self.assertEqual(data["map"]["query_clock_ns"], 1000)
        self.assertTrue(data["map"]["provenance_summary"]["sim_oracle_used"])
        self.assertEqual(data["objects"][0]["pose_source_ns"], 100)

    def test_expired_filter_is_explicit_and_context_is_deterministic(self):
        normal = json.loads(self.cli("--label", "person").stdout)
        self.assertEqual(normal["objects"], [])
        expired = self.cli("--label", "person", "--include-expired", "--context")
        self.assertEqual(expired.returncode, 0, expired.stderr)
        self.assertEqual(
            expired.stdout,
            self.cli("--label", "person", "--include-expired", "--context").stdout,
        )
        for text in [
            "matched_objects=1",
            "not rigid-body velocity",
            "mask_coverage_complete",
            "sim_oracle",
            "capacity_complete",
            "unknown",
            "no inferred floor names",
        ]:
            self.assertIn(text, expired.stdout)

    def test_invalid_filter_arguments_fail_without_traceback(self):
        cases = [
            ("--near", "0", "0", "0"),
            ("--radius", "1"),
            ("--near", "0", "0", "0", "--radius", "-1"),
            ("--near", "nan", "0", "0", "--radius", "1"),
        ]
        for args in cases:
            with self.subTest(args=args):
                result = self.cli(*args)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn("Traceback", result.stderr)

    def test_unknown_snapshot_schema_fails(self):
        self.path.write_text('{"schema":"not_a_scene_map","objects":[]}')
        result = self.cli()
        self.assertEqual(result.returncode, 2)
        self.assertIn("unknown map snapshot schema", result.stderr)


if __name__ == "__main__":
    unittest.main()
