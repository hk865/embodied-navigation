"""CPU checks of scene-query admission, not simulated contact/traversal tests."""

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from embodied_navigation.simulation.dynanav.session_collision import (
    SupportPreflightError,
    load_config,
    preflight_support,
)


class Query:
    def __init__(self, config, *, extra=None, extra_distance=0.04, missing=False):
        self.config = config
        self.extra = extra
        self.extra_distance = extra_distance
        self.missing = missing
        self.calls = 0

    def raycast_all(self, origin, direction, length, callback, both_sides):
        sample = self.config["support_preflight"]["samples"][self.calls]
        first = self.calls == 0
        self.calls += 1
        if not (first and self.missing):
            callback(
                SimpleNamespace(
                    collision=sample["expected_colliders"][0],
                    rigid_body="",
                    position=sample["surface_xyz_m"],
                    normal=(0, 0, 1),
                    distance=0.12,
                )
            )
        # Return the nearer obstacle last: query callbacks need not be sorted.
        if first and self.extra:
            callback(
                SimpleNamespace(
                    collision=self.extra,
                    rigid_body="",
                    position=(origin[0], origin[1], origin[2] - self.extra_distance),
                    normal=(0, 0, 1),
                    distance=self.extra_distance,
                )
            )
        return 2


class SessionSupportAdmission(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.carb = patch.dict(
            sys.modules, {"carb": SimpleNamespace(Float3=lambda *xyz: xyz)}
        )
        self.carb.start()
        self.addCleanup(self.carb.stop)

    def test_original_first_flight_selection_and_explicit_robot_exclusion(self):
        self.assertEqual(len(self.config["floor_paths"]), 119)
        self.assertIn("/Root/SM_Floor_6m_17/SM_Floor_6m/SM_Floor_6m", self.config["floor_paths"])
        self.assertEqual(len(self.config["stair_paths"]), 4)
        samples = self.config["support_preflight"]["samples"]
        self.assertEqual(
            [s["source_waypoint_index"] for s in samples[1:]], list(range(3, 16))
        )
        self.assertTrue(all(s["surface_xyz_m"][1] == -7.5 for s in samples))
        query = Query(self.config, extra="/World/Go2/foot")
        report = preflight_support(query)
        self.assertTrue(report["passed"])
        self.assertEqual(query.calls, len(samples))
        self.assertEqual(len(report["checks"][0]["all_hits"]), 2)
        self.assertEqual(len(report["checks"][0]["robot_hits_excluded"]), 1)
        self.assertFalse(report["physics_advanced"])

    def test_nearer_unexpected_collider_cannot_select_floor_below_it(self):
        query = Query(self.config, extra="/Root/unexpected_obstacle")
        with self.assertRaises(SupportPreflightError) as caught:
            preflight_support(query)
        report = caught.exception.report
        self.assertFalse(report["passed"])
        self.assertEqual(
            len(report["checks"]), len(self.config["support_preflight"]["samples"])
        )
        first = report["checks"][0]
        self.assertEqual(
            first["nearest_scene_hit"]["collision"], "/Root/unexpected_obstacle"
        )
        self.assertEqual(len(first["all_hits"]), 2)

    def test_missing_hit_remains_missing(self):
        with self.assertRaises(SupportPreflightError) as caught:
            preflight_support(Query(self.config, missing=True))
        first = caught.exception.report["checks"][0]
        self.assertFalse(first["passed"])
        self.assertEqual(first["all_hits"], [])
        self.assertIsNone(first["nearest_scene_hit"])
        self.assertFalse(caught.exception.report["synthetic_or_filled_hits"])

    def test_tied_nearest_unknown_collider_is_not_callback_order_dependent(self):
        query = Query(
            self.config, extra="/Root/unexpected_obstacle", extra_distance=0.12
        )
        with self.assertRaises(SupportPreflightError) as caught:
            preflight_support(query)
        first = caught.exception.report["checks"][0]
        self.assertEqual(len(first["nearest_scene_hits"]), 2)
        self.assertFalse(first["passed"])



class FloorSessionRepair(unittest.TestCase):
    """Real USD in-memory tests; no SimulationApp or PhysX runtime required."""

    def setUp(self):
        try:
            from pxr import Usd, UsdGeom, UsdPhysics
        except ImportError:
            self.skipTest('CPU USD bindings unavailable')
        self.Usd, self.Geom, self.Physics = Usd, UsdGeom, UsdPhysics
        self.config = load_config()
        self.stage = Usd.Stage.CreateInMemory()
        for path in self.config['floor_paths']:
            mesh = UsdGeom.Mesh.Define(self.stage, path)
            mesh.CreatePointsAttr([(0, 0, 0), (3, 0, 0), (3, 3, 0), (0, 3, 0)])
            mesh.CreateFaceVertexCountsAttr([4])
            mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        for path in self.config['stair_paths']:
            prim = UsdGeom.Mesh.Define(self.stage, path).GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        ground = UsdGeom.Mesh.Define(self.stage, self.config['ground_plane_path']).GetPrim()
        UsdPhysics.CollisionAPI.Apply(ground).CreateCollisionEnabledAttr(True)

    def test_all_floors_session_only_idempotent_and_source_unchanged(self):
        from embodied_navigation.simulation.dynanav.session_collision import repair_basement_session
        root = self.stage.GetRootLayer()
        before = root.ExportToString()
        target = self.stage.GetEditTarget().GetLayer()
        report = repair_basement_session(self.stage)
        self.assertEqual(report['floor_paths_enabled'], self.config['floor_paths'])
        self.assertEqual(root.ExportToString(), before)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), target)
        for path in self.config['floor_paths']:
            prim = self.stage.GetPrimAtPath(path)
            self.assertTrue(self.Physics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
            self.assertEqual(self.Physics.MeshCollisionAPI(prim).GetApproximationAttr().Get(), 'none')
        self.assertFalse(self.Physics.CollisionAPI(self.stage.GetPrimAtPath(self.config['ground_plane_path'])).GetCollisionEnabledAttr().Get())
        session = self.stage.GetSessionLayer().ExportToString()
        repair_basement_session(self.stage)
        self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)
        self.assertEqual(root.ExportToString(), before)

    def test_hidden_tilted_degenerate_or_missing_floor_rejected_before_edits(self):
        from embodied_navigation.simulation.dynanav.session_collision import repair_basement_session
        path = '/Root/SM_Floor_6m_17/SM_Floor_6m/SM_Floor_6m'
        mesh = self.Geom.Mesh(self.stage.GetPrimAtPath(path))
        original = self.stage.GetRootLayer().ExportToString()
        for kind in ['hidden', 'tilted', 'degenerate', 'inactive']:
            with self.subTest(kind=kind):
                self.stage.GetRootLayer().ImportFromString(original)
                mesh = self.Geom.Mesh(self.stage.GetPrimAtPath(path))
                if kind == 'hidden':
                    mesh.MakeInvisible()
                elif kind == 'tilted':
                    mesh.GetPointsAttr().Set([(0,0,0), (3,0,0), (3,3,1), (0,3,1)])
                elif kind == 'degenerate':
                    mesh.GetPointsAttr().Set([(0,0,0)] * 4)
                else:
                    mesh.GetPrim().SetActive(False)
                source = self.stage.GetRootLayer().ExportToString()
                session = self.stage.GetSessionLayer().ExportToString()
                with self.assertRaises(RuntimeError):
                    repair_basement_session(self.stage)
                self.assertEqual(self.stage.GetSessionLayer().ExportToString(), session)
                self.assertEqual(self.stage.GetRootLayer().ExportToString(), source)

    def test_failed_authoring_rolls_back_session_and_restores_edit_target(self):
        from embodied_navigation.simulation.dynanav.session_collision import repair_basement_session
        session = self.stage.GetSessionLayer()
        before = session.ExportToString()
        source = self.stage.GetRootLayer().ExportToString()
        with patch.object(self.Physics.MeshCollisionAPI, 'Apply', side_effect=RuntimeError('injected')):
            with self.assertRaisesRegex(RuntimeError, 'injected'):
                repair_basement_session(self.stage)
        self.assertEqual(session.ExportToString(), before)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), source)
        self.assertEqual(self.stage.GetEditTarget().GetLayer(), self.stage.GetRootLayer())


if __name__ == "__main__":
    unittest.main()
