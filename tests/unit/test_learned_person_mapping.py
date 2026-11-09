"""Direct RGB-only masks with explicit inherited pose-preprocessing provenance."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    LayeredMap,
    Observation,
    PointMask,
)
from embodied_navigation.perception.learned_person_mapping import (
    learned_person_evidence,
    load_geometry,
)


class LearnedPersonMappingTests(unittest.TestCase):
    def test_overlap_owned_once_by_higher_score_and_background_unknown(self):
        masks = np.zeros((2, 2, 3), bool)
        masks[0].flat[[0, 1]] = True
        masks[1].flat[[1, 2]] = True
        states, ids, labels, instances = learned_person_evidence(
            masks, np.array([0.8, 0.9]), np.array([2, 1, 0, 5]), 100
        )
        np.testing.assert_array_equal(states, [2, 2, 2, 0])
        self.assertEqual(
            ids, ("rgb-person:100:1", "rgb-person:100:1", "rgb-person:100:0", "")
        )
        self.assertEqual(set(labels.values()), {"person"})
        self.assertEqual(sum(x["owned_point_count"] for x in instances), 3)
        self.assertFalse(np.any(states == 1))

    def test_geometry_allowlist_does_not_read_oracle_arrays_or_pose(self):
        with tempfile.TemporaryDirectory() as t:
            p = Path(t) / "frame.npz"
            bomb = np.array([object()], dtype=object)
            np.savez(
                p,
                source_ns=np.int64(100),
                points_sensor=np.ones((3, 3)),
                pixel_flat_indices=np.array([5, 0, 3]),
                states=bomb,
                instance_paths=bomb,
                T_map_sensor=bomb,
            )
            ns, points, pixels = load_geometry(p, (2, 3))
            self.assertEqual(ns, 100)
            self.assertEqual(points.shape, (3, 3))
            np.testing.assert_array_equal(pixels, [5, 0, 3])

    def test_learned_frame_ids_use_existing_geometric_association_and_no_static(self):
        store = LayeredMap(map_id="learned", frame_id="estimated")
        mask_pixels = np.ones((1, 1, 2), bool)
        for sequence, ns in [(1, 100), (2, 200)]:
            states, ids, labels, _ = learned_person_evidence(
                mask_pixels, np.array([0.9]), np.array([0, 1]), ns
            )
            obs = Observation(
                epoch=1,
                sequence=sequence,
                source_ns=ns,
                pose_source_ns=ns,
                points_sensor=np.array([[1.0, 0.0, 0.0], [1.0, 0.1, 0.0]]),
                T_map_sensor=np.eye(4),
                anchor_id=str(sequence),
                frame_id="imu",
                calibration_id="rgb",
                pose_source="slam",
            )
            mask = PointMask(
                epoch=1,
                sequence=sequence,
                source_ns=ns,
                produced_ns=ns,
                frame_id="imu",
                calibration_id="rgb",
                states=states,
                instance_ids=ids,
                labels=labels,
                provenance={
                    "kind": "learned",
                    "producer": "rgb-test",
                    "instance_id_scope": "frame",
                },
            )
            store.integrate(obs, mask)
        self.assertEqual(len(store.objects), 1)
        self.assertEqual(store.static_points().shape, (0, 3))
        summary = store.summary()
        self.assertFalse(summary["provenance_summary"]["sim_oracle_used"])
        self.assertEqual(summary["provenance_summary"]["mask_kinds"], ["learned"])


class PoseDependencyProvenanceTests(unittest.TestCase):
    def test_unknown_dependency_never_defaults_to_oracle_free(self):
        from embodied_navigation.perception.learned_person_mapping import (
            forward_pose_provenance,
            pose_dependency_provenance,
        )

        provenance = pose_dependency_provenance(bag="bag", topic="/estimated")
        scene = forward_pose_provenance(
            {"provenance_summary": {"sim_oracle_used": False}}, provenance
        )
        self.assertEqual(scene["pose_preprocessing_mask_source"], "unknown")
        self.assertFalse(scene["end_to_end_oracle_free"])
        self.assertIsNone(scene["provenance_summary"]["sim_oracle_used"])

    def test_linked_receipts_disclose_oracle_preprocessing_in_objects_and_context(self):
        import importlib.util
        import json

        from embodied_navigation.perception.learned_person_mapping import (
            forward_pose_provenance,
            pose_dependency_provenance,
        )

        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            launch = folder / "launch.json"
            producer = folder / "producer.json"
            launch.write_text(
                json.dumps(
                    {
                        "sensor_inputs": ["/static", "/imu"],
                        "ground_truth_input": False,
                        "core_sha256": "core",
                        "command": [],
                    }
                )
            )
            producer.write_text(
                json.dumps(
                    {"static_frontend_topic": "/static", "mask_source": "sim_oracle"}
                )
            )
            provenance = pose_dependency_provenance(
                bag=folder / "bag",
                topic="/estimated",
                launch_receipt=launch,
                preprocessing_receipt=producer,
            )
            original = {
                "objects": [
                    {
                        "object_id": "unchanged",
                        "provenance": {"kind": "learned"},
                        "history": [
                            {"source_ns": 10, "provenance": {"kind": "learned"}}
                        ],
                    }
                ],
                "provenance_summary": {
                    "mask_kinds": ["learned"],
                    "pose_sources": ["slam"],
                    "sim_oracle_used": False,
                },
            }
            scene = forward_pose_provenance(original, provenance)
            self.assertFalse(original["provenance_summary"]["sim_oracle_used"])
            self.assertTrue(scene["provenance_summary"]["sim_oracle_used"])
            self.assertFalse(
                scene["provenance_summary"]["direct_observation_sim_oracle_used"]
            )
            self.assertEqual(scene["objects"][0]["object_id"], "unchanged")
            self.assertEqual(scene["objects"][0]["provenance"]["kind"], "learned")
            self.assertEqual(
                scene["objects"][0]["history"][0]["pose_preprocessing_mask_source"],
                "sim_oracle",
            )
            spec = importlib.util.spec_from_file_location(
                "pose_provenance_context",
                Path(__file__).resolve().parents[2] / "scripts/query_scene_map.py",
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            query = module.select(scene, label=None, include_expired=True)
            context = module.context_text(query)
            self.assertIn('"pose_preprocessing_mask_source": "sim_oracle"', context)
            self.assertIn('"end_to_end_oracle_free": false', context)

    def test_input_remap_prevents_mislabeling_different_pose_input(self):
        import json

        from embodied_navigation.perception.learned_person_mapping import (
            pose_dependency_provenance,
        )

        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            launch = folder / "launch.json"
            producer = folder / "producer.json"
            launch.write_text(
                json.dumps({"sensor_inputs": ["/static"], "command": ["/static:=/raw"]})
            )
            producer.write_text(
                json.dumps(
                    {"static_frontend_topic": "/static", "mask_source": "sim_oracle"}
                )
            )
            with self.assertRaisesRegex(ValueError, "effective estimator"):
                pose_dependency_provenance(
                    bag=folder / "bag",
                    topic="/estimated",
                    launch_receipt=launch,
                    preprocessing_receipt=producer,
                )


if __name__ == "__main__":
    unittest.main()
