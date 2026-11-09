"""Path checks and optional no-app USD checks against installed local human assets."""

import hashlib
import math
import subprocess
import sys
import unittest
from pathlib import Path

from embodied_navigation.simulation.dynanav.pedestrian import (
    PedestrianPath,
    create_pedestrian,
    inspect_walk_support,
)


class PedestrianPathTests(unittest.TestCase):
    def test_sine_path_is_bounded_and_obeys_maximum_speed(self):
        path = PedestrianPath((1, 2, 3), (5, 5, 3), 0.6)
        previous = path.pose(0)[0]
        for step in range(1, 2001):
            position, _ = path.pose(step * 0.01)
            fraction = (position[0] - 1.0) / 4.0
            self.assertGreaterEqual(fraction, -1e-12)
            self.assertLessEqual(fraction, 1.0 + 1e-12)
            self.assertAlmostEqual(position[1], 2.0 + fraction * 3.0)
            self.assertEqual(position[2], 3.0)
            self.assertLessEqual(math.dist(previous, position) / 0.01, 0.600001)
            previous = position

    def test_linear_path_reverses_at_endpoints(self):
        path = PedestrianPath((0, 0, 1), (2, 0, 1), 1.0, "linear")
        self.assertEqual(path.pose(0)[0], (0, 0, 1))
        self.assertEqual(path.pose(2)[0], (2, 0, 1))
        self.assertEqual(path.pose(3)[0], (1, 0, 1))
        self.assertAlmostEqual(path.pose(3)[1], math.pi)
        self.assertEqual(path.pose(4)[0], (0, 0, 1))

    def test_one_way_is_bounded_and_constant_speed_until_endpoint(self):
        path = PedestrianPath((-1, 2, 3), (2, 6, 3), 0.6, "one_way")
        previous = path.pose(0)[0]
        for step in range(1, 251):
            time = step * 0.1
            position, heading = path.pose(time)
            fraction = (position[0] + 1.0) / 3.0
            self.assertGreaterEqual(fraction, 0.0)
            self.assertLessEqual(fraction, 1.0)
            self.assertAlmostEqual(position[1], 2.0 + fraction * 4.0)
            self.assertEqual(position[2], 3.0)
            measured_speed = math.dist(previous, position) / 0.1
            self.assertLessEqual(measured_speed, 0.600001)
            if time < path.length / path.speed_mps:
                self.assertAlmostEqual(measured_speed, path.speed_mps)
            self.assertAlmostEqual(heading, math.atan2(4, 3))
            previous = position

    def test_one_way_reaches_exact_endpoint_and_never_returns(self):
        path = PedestrianPath((1, -2, 0.4), (-2, 2, 0.4), 0.6, "one_way")
        arrival = path.length / path.speed_mps
        self.assertEqual(path.pose(0)[0], path.start)
        before, heading = path.pose(arrival - 0.001)
        self.assertAlmostEqual(math.dist(before, path.end), path.speed_mps * 0.001)
        for time in (arrival, arrival + 1, 2 * arrival, 1e6):
            position, final_heading = path.pose(time)
            self.assertEqual(position, path.end)
            self.assertEqual(final_heading, heading)

    def test_invalid_motion_rejected(self):
        for end, speed, mode in [
            ((0, 0, 0), 1, "sine"),
            ((1, 0, 1), 1, "sine"),
            ((1, 0, 0), 0, "sine"),
            ((1, 0, 0), 1, "unknown"),
        ]:
            with self.assertRaises(ValueError):
                PedestrianPath((0, 0, 0), end, speed, mode)
        with self.assertRaises(ValueError):
            PedestrianPath((0, 0, 0), (1, 0, 0)).pose(float("nan"))

    def test_module_import_does_not_import_isaac(self):
        subprocess.run(
            [
                sys.executable,
                "-B",
                "-c",
                "import sys; import embodied_navigation.simulation.dynanav.pedestrian; assert 'pxr' not in sys.modules; assert 'isaacsim' not in sys.modules; assert 'omni.usd' not in sys.modules",
            ],
            check=True,
        )


ISAAC = Path("/home/hyh001/isaacsim/extscache")
HUMAN = (
    ISAAC
    / "isaacsim.replicator.agent.core-1.6.8+110.1.2/data/test_scenes/test_biped/biped_demo_meters.usd"
)
WALK = (
    ISAAC
    / "omni.anim.retarget.core-110.1.0+110.1.1.lx64.r.cp312.u7f4/data/rigs/Human/assets/walk.skelanim.usd"
)
try:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdSkel, Vt
except ImportError:
    Usd = None


@unittest.skipUnless(
    Usd is not None and HUMAN.is_file() and WALK.is_file(),
    "requires installed local USD runtime/human assets; no simulator app",
)
class LocalHumanUSDTests(unittest.TestCase):
    def stage(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, "Z")
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        return stage

    def test_walk_deforms_visible_human_and_collision_shares_root_motion(self):
        stage = self.stage()
        before = stage.GetRootLayer().ExportToString()
        checksums = {
            path: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (HUMAN, WALK)
        }
        actor = create_pedestrian(
            stage, HUMAN, start=(1, 2, 3), end=(5, 2, 3), animation_path=WALK
        )
        self.assertEqual(actor.report["animation"]["mode"], "local_skeletal_walk")
        self.assertEqual(actor.report["animation"]["translation_unit_scale"], 0.01)
        self.assertTrue(
            UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(actor.root_path))
            .GetKinematicEnabledAttr()
            .Get()
        )
        capsule = stage.GetPrimAtPath(actor.root_path + "/CollisionProxy")
        self.assertTrue(
            UsdPhysics.CollisionAPI(capsule).GetCollisionEnabledAttr().Get()
        )
        self.assertEqual(UsdGeom.Imageable(capsule).ComputeVisibility(), "invisible")
        human = stage.GetPrimAtPath(actor.root_path + "/VisualFrame/Human")
        skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath(str(human.GetPath()) + "/Root"))
        mesh = stage.GetPrimAtPath(str(human.GetPath()) + "/Body_Mesh")

        def skin_points():
            cache = UsdSkel.Cache()
            cache.Populate(UsdSkel.Root(human), Usd.PrimDefaultPredicate)
            transforms = cache.GetSkelQuery(skeleton).ComputeSkinningTransforms(
                Usd.TimeCode.Default()
            )
            points = Vt.Vec3fArray(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
            self.assertTrue(
                cache.GetSkinningQuery(mesh).ComputeSkinnedPoints(
                    transforms, points, Usd.TimeCode.Default()
                )
            )
            return points

        first = skin_points()
        actor.update(1.0 / 3.0)
        second = skin_points()
        self.assertGreater(max((a - b).GetLength() for a, b in zip(first, second)), 0.1)
        xforms = UsdGeom.XformCache()
        root_position = xforms.GetLocalToWorldTransform(
            stage.GetPrimAtPath(actor.root_path)
        ).ExtractTranslation()
        collider_position = xforms.GetLocalToWorldTransform(
            capsule
        ).ExtractTranslation()
        self.assertAlmostEqual(root_position[0], collider_position[0])
        self.assertAlmostEqual(root_position[1], collider_position[1])
        self.assertEqual(stage.GetRootLayer().ExportToString(), before)
        for path, checksum in checksums.items():
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), checksum)
        self.assertIs(stage.GetEditTarget().GetLayer(), stage.GetRootLayer())

    def test_one_way_holds_root_and_skeletal_pose_after_single_crossing(self):
        stage = self.stage()
        actor = create_pedestrian(
            stage,
            HUMAN,
            start=(1, 2, 3),
            end=(1.6, 2, 3),
            speed_mps=0.6,
            path_mode="one_way",
            animation_path=WALK,
        )
        arrival = actor.path.length / actor.path.speed_mps
        first = actor.update(arrival)
        animation = UsdSkel.Animation(
            stage.GetPrimAtPath(actor.root_path + "/VisualFrame/Human/LocalWalk")
        )
        rotations = animation.GetRotationsAttr().Get()
        translations = animation.GetTranslationsAttr().Get()
        later = actor.update(arrival + 1.0)
        self.assertEqual(first["position_m"], later["position_m"])
        self.assertEqual(rotations, animation.GetRotationsAttr().Get())
        self.assertEqual(translations, animation.GetTranslationsAttr().Get())
        self.assertEqual(
            actor.report["path_scope"], "single_crossing_then_endpoint_hold"
        )
        self.assertTrue(actor.report["endpoint_skeletal_pose_held"])

    def test_static_pose_and_failed_creation_are_explicit_and_session_only(self):
        stage = self.stage()
        actor = create_pedestrian(stage, HUMAN, start=(0, 0, 0), end=(1, 0, 0))
        self.assertEqual(actor.report["animation"]["mode"], "translating_static_pose")
        with self.assertRaises(ValueError):
            create_pedestrian(stage, HUMAN, start=(0, 0, 0), end=(1, 0, 0))
        other = self.stage()
        before = other.GetSessionLayer().ExportToString()
        with self.assertRaises(ValueError):
            create_pedestrian(
                other, HUMAN, start=(0, 0, 0), end=(1, 0, 0), animation_path=HUMAN
            )
        self.assertEqual(before, other.GetSessionLayer().ExportToString())
        self.assertFalse(other.GetPrimAtPath("/World/DynamicPedestrian"))


class WalkSupportInterfaceTests(unittest.TestCase):
    """Fake callbacks test admission only; they are not Office support evidence."""

    def setUp(self):
        from types import SimpleNamespace
        from unittest.mock import patch

        self.namespace = SimpleNamespace
        self.mesh_type, self.rigid_api = object(), object()

        class CollisionAPI:
            def __init__(self, prim):
                self.prim = prim

            def GetCollisionEnabledAttr(self):
                return SimpleNamespace(Get=lambda: self.prim.enabled)

        self.collision_api = CollisionAPI
        pxr = SimpleNamespace(
            UsdGeom=SimpleNamespace(Mesh=self.mesh_type),
            UsdPhysics=SimpleNamespace(
                CollisionAPI=CollisionAPI, RigidBodyAPI=self.rigid_api
            ),
        )
        self.patches = [
            patch.dict(
                sys.modules,
                {"carb": SimpleNamespace(Float3=lambda *xyz: xyz), "pxr": pxr},
            ),
            patch(
                "embodied_navigation.simulation.dynanav.session_collision.load_config",
                return_value={
                    "floor_paths": ["/Root/original_floor"],
                    "stair_paths": ["/Root/original_stair"],
                },
            ),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        outer = self

        class Prim:
            enabled = True
            rigid = False

            def __bool__(self):
                return True

            def IsActive(self):
                return True

            def IsA(self, schema):
                return schema is outer.mesh_type

            def HasAPI(self, api):
                return (
                    api is outer.collision_api or api is outer.rigid_api and self.rigid
                )

            def IsPseudoRoot(self):
                return False

            def GetParent(self):
                return None

        self.prim = Prim()
        self.actor = SimpleNamespace(
            path=PedestrianPath((-22, -8.7, -1.49), (-22, -6.3, -1.49), 0.3, "one_way"),
            root_path="/World/DynamicPedestrian",
            stage=SimpleNamespace(GetPrimAtPath=lambda path: self.prim),
        )

    def query(self, *, missing_first=False, extra=(), surface_error=0.0):
        outer = self

        class Query:
            def __init__(self):
                self.calls = []

            def raycast_all(self, origin, direction, length, callback, both_sides):
                first = not self.calls
                self.calls.append((origin, direction, length, both_sides))
                if not (first and missing_first):
                    callback(
                        outer.namespace(
                            collision="/Root/original_floor",
                            rigid_body="",
                            position=(
                                origin[0],
                                origin[1],
                                origin[2] - 0.12 + surface_error,
                            ),
                            normal=(0, 0, 1),
                            distance=0.12 - surface_error,
                        )
                    )
                if first:
                    for collision, rigid_body, distance in extra:
                        callback(
                            outer.namespace(
                                collision=collision,
                                rigid_body=rigid_body,
                                position=(origin[0], origin[1], origin[2] - distance),
                                normal=(0, 0, 1),
                                distance=distance,
                            )
                        )

        return Query()

    def test_three_real_query_calls_keep_all_hits_and_only_exclude_declared_ownership(
        self,
    ):
        query = self.query(
            extra=[
                ("/World/Go2/foot", "", 0.02),
                ("/shape_owned_by_actor", self.actor.root_path, 0.03),
            ]
        )
        report = inspect_walk_support(query, self.actor)
        self.assertTrue(report["passed"])
        self.assertEqual(len(query.calls), 3)
        self.assertEqual(query.calls[1][0][:2], (-22.0, -7.5))
        self.assertEqual(len(report["checks"][0]["all_hits"]), 3)
        self.assertEqual(len(report["checks"][0]["excluded_hits"]), 2)
        self.assertFalse(report["physics_advanced"])
        self.assertFalse(report["human_volume_clearance_verified"])

    def test_nearer_unknown_obstacle_is_rejected_instead_of_selecting_floor_below(self):
        report = inspect_walk_support(
            self.query(extra=[("/Root/unknown_obstacle", "", 0.04)]), self.actor
        )
        self.assertFalse(report["passed"])
        first = report["checks"][0]
        self.assertEqual(
            first["nearest_scene_hit"]["collision"], "/Root/unknown_obstacle"
        )
        self.assertIn(
            "nearest_hit_is_not_original_static_support", first["failure_reasons"]
        )
        self.assertEqual(len(first["all_hits"]), 2)

    def test_missing_support_is_not_filled_and_other_points_still_recorded(self):
        report = inspect_walk_support(self.query(missing_first=True), self.actor)
        self.assertFalse(report["passed"])
        first = report["checks"][0]
        self.assertEqual(first["all_hits"], [])
        self.assertIsNone(first["nearest_scene_hit"])
        self.assertFalse(report["synthetic_or_filled_hits"])
        self.assertEqual(len(report["checks"]), 3)

    def test_unknown_tied_nearest_hit_cannot_be_ignored_by_callback_order(self):
        report = inspect_walk_support(
            self.query(extra=[("/Root/unknown_obstacle", "", 0.12)]), self.actor
        )
        self.assertFalse(report["passed"])
        self.assertEqual(len(report["checks"][0]["nearest_scene_hits"]), 2)

    def test_enabled_original_mesh_with_rigid_body_ownership_is_not_static_support(
        self,
    ):
        self.prim.rigid = True
        report = inspect_walk_support(self.query(), self.actor)
        self.assertFalse(report["passed"])
        self.assertIn(
            "nearest_hit_is_not_original_static_support",
            report["checks"][0]["failure_reasons"],
        )

    def test_surface_must_match_actual_authored_foot_height(self):
        report = inspect_walk_support(
            self.query(surface_error=0.03), self.actor, tolerance_m=0.02
        )
        self.assertFalse(report["passed"])
        self.assertIn(
            "nearest_surface_xyz_outside_tolerance",
            report["checks"][0]["failure_reasons"],
        )


if __name__ == "__main__":
    unittest.main()
