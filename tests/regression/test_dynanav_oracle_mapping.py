"""CPU oracle adapter checks with fake renderer/scene, never a simulator test."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    DYNAMIC,
    STATIC,
    UNKNOWN,
)
from embodied_navigation.simulation.dynanav.oracle_mapping import OracleMapping

RIGID_BODY_API = object()


class Attribute:
    def __init__(self, name, value=None, time_samples=0):
        self.name, self.value, self.time_samples = name, value, time_samples

    def GetName(self):
        return self.name

    def Get(self):
        return self.value

    def GetNumTimeSamples(self):
        return self.time_samples


class Prim:
    def __init__(
        self, *, parent=None, rigid=False, attributes=(), pseudo=False, prototype=False
    ):
        self.parent, self.rigid, self.attributes = parent, rigid, list(attributes)
        self.pseudo, self.prototype = pseudo, prototype

    def HasAPI(self, api):
        return api is RIGID_BODY_API and self.rigid

    def GetAttributes(self):
        return self.attributes

    def GetParent(self):
        return self.parent

    def IsPseudoRoot(self):
        return self.pseudo

    def IsPrototype(self):
        return self.prototype

    def IsInPrototype(self):
        return self.prototype


class Stage:
    def __init__(self, prims):
        self.prims = prims

    def GetPrimAtPath(self, path):
        return self.prims.get(path)


class Renderer:
    def __init__(self, image, paths):
        self.image, self.paths = image, paths

    def get_data(self):
        return {"data": self.image.copy(), "info": {"idToLabels": dict(self.paths)}}


class OracleAdapter(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.count = 0
        self.profile = {
            "semantic_interval_ns": 2_000_000_000,
            "map_keyframe_interval_ns": 500_000_000,
            "pedestrian_roots": ["/World/pedestrian"],
        }
        root = Prim(pseudo=True)
        rigid = Prim(parent=root, rigid=True)
        animated = Prim(
            parent=root, attributes=[Attribute("xformOp:translate", time_samples=2)]
        )
        self.semantic = Attribute("semantic:Semantics:params:semanticData", "wall")
        self.stage = Stage(
            {
                "/World/floor": Prim(parent=root, attributes=[self.semantic]),
                "/World/rigid/mesh": Prim(parent=rigid),
                "/World/animated/mesh": Prim(parent=animated),
                "/World/pedestrian/mesh": Prim(parent=root),
                "/__Prototype_1/mesh": Prim(parent=root, prototype=True),
            }
        )
        pxr = SimpleNamespace(UsdPhysics=SimpleNamespace(RigidBodyAPI=RIGID_BODY_API))
        self.pxr = patch.dict(sys.modules, {"pxr": pxr})
        self.pxr.start()
        self.addCleanup(self.pxr.stop)
        self.points = np.array([[2.0, 1.0, 0.0], [4.0, 0.0, 1.0], [6.0, 2.0, 3.0]])
        # Deliberately nonmonotonic: final point order must own the correspondence.
        self.packet = SimpleNamespace(
            points=self.points.copy(), pixel_flat_indices=np.array([5, 0, 3])
        )
        self.depth = np.ones((2, 3), dtype=np.float32)
        self.rgb = np.zeros((2, 3, 3), dtype=np.uint8)
        self.transform = np.array(
            [
                [0.0, -1.0, 0.0, 10.0],
                [1.0, 0.0, 0.0, 20.0],
                [0.0, 0.0, 1.0, 30.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def mapper(self):
        self.count += 1
        return OracleMapping(
            self.stage, self.directory / f"mapping_{self.count}", self.profile
        )

    def process(self, mapper, segmentation, stamp=1_000_000_000):
        return mapper.process(
            packet=self.packet,
            segmentation=segmentation,
            source_ns=stamp,
            transform=self.transform,
            rgb=self.rgb,
            depth=self.depth,
        )

    def test_missing_or_wrong_shape_mask_quarantines_without_reusing_previous_frame(
        self,
    ):
        for segmentation in (
            None,
            {
                "data": np.ones((1, 2), dtype=np.uint32),
                "info": {"idToLabels": {"1": "/World/floor"}},
            },
        ):
            with self.subTest(segmentation=segmentation):
                mapper = self.mapper()
                states, counts = self.process(mapper, segmentation)
                np.testing.assert_array_equal(states, [UNKNOWN] * 3)
                self.assertEqual(counts["mask_status"], "missing_mask")
                self.assertEqual(counts["navigation"], 3)
                self.assertEqual(len(mapper.store.static_points()), 0)
        mapper = self.mapper()
        renderer = Renderer(np.ones((2, 3), dtype=np.uint32), {"1": "/World/floor"})
        states, _ = self.process(mapper, renderer.get_data())
        np.testing.assert_array_equal(states, [STATIC] * 3)
        states, counts = self.process(mapper, None, stamp=1_100_000_000)
        np.testing.assert_array_equal(states, [UNKNOWN] * 3)
        self.assertEqual(counts["unknown"], 3)
        self.assertEqual(mapper.mask_failures, 1)

    def test_pixel_order_pose_and_complete_navigation_survive_static_dynamic_unknown_split(
        self,
    ):
        mapper = self.mapper()
        renderer = Renderer(
            np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint32),
            {"10": "/World/floor", "40": "/World/absent", "60": "/World/rigid/mesh"},
        )
        results = []
        integrate = mapper.store.integrate

        def observe(*args, **kwargs):
            result = integrate(*args, **kwargs)
            results.append(result)
            return result

        with patch.object(mapper.store, "integrate", side_effect=observe):
            states, counts = self.process(mapper, renderer.get_data())
        np.testing.assert_array_equal(states, [DYNAMIC, STATIC, UNKNOWN])
        self.assertEqual(
            {
                name: counts[name]
                for name in ("static", "dynamic", "unknown", "navigation")
            },
            {"static": 1, "dynamic": 1, "unknown": 1, "navigation": 3},
        )
        result = results[0]
        world = self.points @ self.transform[:3, :3].T + self.transform[:3, 3]
        np.testing.assert_allclose(result["navigation_points"], world)
        np.testing.assert_array_equal(result["navigation_points_sensor"], self.points)
        np.testing.assert_array_equal(self.packet.points, self.points)
        np.testing.assert_allclose(mapper.store.static_points(), world[[1]])
        observation = next(iter(mapper.store.observations.values()))["observation"]
        self.assertEqual(
            (observation.source_ns, observation.pose_source_ns), (1_000_000_000,) * 2
        )
        self.assertEqual(observation.frame_id, "go2_imu")
        self.assertEqual(observation.pose_source, "sim_oracle")

    def test_rigid_animated_and_pedestrian_ownership_never_enters_static_map(self):
        mapper = self.mapper()
        renderer = Renderer(
            np.array([[2, 0, 0], [3, 0, 1]], dtype=np.uint32),
            {
                "1": "/World/rigid/mesh",
                "2": "/World/animated/mesh",
                "3": "/World/pedestrian/mesh",
            },
        )
        states, counts = self.process(mapper, renderer.get_data())
        np.testing.assert_array_equal(states, [DYNAMIC] * 3)
        self.assertEqual(counts["navigation"], 3)
        self.assertEqual(len(mapper.store.static_points()), 0)

    def test_authored_person_is_dynamic_in_returned_mask_and_saved_map(self):
        self.semantic.value = "person"
        mapper = self.mapper()
        renderer = Renderer(np.ones((2, 3), dtype=np.uint32), {"1": "/World/floor"})
        states, counts = self.process(mapper, renderer.get_data())
        np.testing.assert_array_equal(states, [DYNAMIC] * 3)
        self.assertEqual(counts["dynamic"], 3)
        self.assertEqual(len(mapper.store.static_points()), 0)
        with np.load(next(mapper.frames.glob("*.npz"))) as saved:
            np.testing.assert_array_equal(saved["states"], [DYNAMIC] * 3)

    def test_internal_prototype_path_is_unknown_despite_valid_usd_prim(self):
        mapper = self.mapper()
        renderer = Renderer(
            np.ones((2, 3), dtype=np.uint32), {"1": "/__Prototype_1/mesh"}
        )
        states, counts = self.process(mapper, renderer.get_data())
        np.testing.assert_array_equal(states, [UNKNOWN] * 3)
        self.assertEqual(counts["navigation"], 3)
        self.assertEqual(len(mapper.store.static_points()), 0)

    def test_semantic_refresh_interval_does_not_reuse_masks_or_gate_geometry_keyframes(
        self,
    ):
        mapper = self.mapper()
        renderer = Renderer(np.ones((2, 3), dtype=np.uint32), {"1": "/World/floor"})
        self.process(mapper, renderer.get_data(), stamp=0)
        self.assertEqual(mapper.labels["/World/floor"], "wall")
        self.semantic.value = "partition"
        # A new rigid instance must be dynamic even between semantic refreshes.
        renderer.paths = {"1": "/World/rigid/mesh"}
        states, _ = self.process(mapper, renderer.get_data(), stamp=100_000_000)
        np.testing.assert_array_equal(states, [DYNAMIC] * 3)
        renderer.paths = {"1": "/World/floor"}
        self.process(mapper, renderer.get_data(), stamp=1_999_999_999)
        self.assertEqual(mapper.labels["/World/floor"], "wall")
        self.assertEqual(mapper.scheduler.requested, 1)
        self.process(mapper, renderer.get_data(), stamp=2_000_000_000)
        self.assertEqual(mapper.labels["/World/floor"], "partition")
        self.assertEqual(mapper.scheduler.requested, 2)
        self.assertEqual(mapper.keyframes, 2)
        receipt = mapper.finish()
        self.assertEqual(receipt["learned_model_calls"], 0)
        self.assertEqual(receipt["oracle_semantic_refreshes"], 2)
        self.assertFalse(receipt["SLAM_dynamic_filter_verified"])
        self.assertFalse(receipt["camera_visual_frontend_masked"])
        lines = [
            json.loads(line)
            for line in (mapper.output / "frames.jsonl").read_text().splitlines()
        ]
        self.assertEqual(len(lines), 4)


if __name__ == "__main__":
    unittest.main()
