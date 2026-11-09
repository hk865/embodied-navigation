"""Restore configured original office colliders in the USD session only.

Importing this module does not load Isaac or edit a stage. The optional live
PhysX ray check certifies sampled scene support, never foot contact or traversal.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from embodied_navigation.project import project_root

CONFIG_PATH = project_root() / "configs/simulation/dynanav_basement_collision.json"


def load_config(path=None):
    """Read the tracked geometry selection without importing historical files."""
    config = json.loads(Path(CONFIG_PATH if path is None else path).read_text())
    if config.get("schema") != "dynanav_basement_collision/v1":
        raise ValueError("Unsupported basement collision configuration")
    floors, stairs = config["floor_paths"], config["stair_paths"]
    if (
        len(floors) != 7
        or len(set(floors)) != 7
        or len(stairs) != 4
        or len(set(stairs)) != 4
    ):
        raise ValueError(
            "Expected the seven original floors and four original stair meshes"
        )
    return config


def repair_basement_session(stage, config_path=None):
    """Enable original floor meshes and disable GroundPlane in the session layer.

    Existing stair collision must already be active and use the exact mesh.
    No mesh/platform is created and no source layer is saved. The edit target is
    restored even on failure; a failed repair also restores the session content.
    """
    from pxr import UsdGeom, UsdPhysics

    config = load_config(config_path)
    floors = [stage.GetPrimAtPath(path) for path in config["floor_paths"]]
    ground = stage.GetPrimAtPath(config["ground_plane_path"])
    for path, prim in zip(config["floor_paths"], floors):
        if (
            not prim
            or not prim.IsA(UsdGeom.Mesh)
            or not prim.IsActive()
            or prim.IsInstanceProxy()
        ):
            raise RuntimeError("Original editable floor unavailable: " + path)
    if not ground or not ground.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError("Original GroundPlane collision unavailable")
    stairs = {}
    for path in config["stair_paths"]:
        prim = stage.GetPrimAtPath(path)
        if (
            not prim
            or not prim.IsA(UsdGeom.Mesh)
            or not prim.IsActive()
            or not prim.HasAPI(UsdPhysics.CollisionAPI)
            or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            or str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())
            != "none"
        ):
            raise RuntimeError("Original enabled exact stair mesh unavailable: " + path)
        stairs[path] = {"collision_enabled": True, "approximation": "none"}

    session = stage.GetSessionLayer()
    if not session:
        raise RuntimeError("A USD session layer is required")
    sources = [
        (layer, layer.ExportToString())
        for layer in stage.GetUsedLayers()
        if layer != session
    ]
    session_before = session.ExportToString()
    previous_target = stage.GetEditTarget()
    root_dirty_before = stage.GetRootLayer().dirty
    try:
        stage.SetEditTarget(session)
        UsdPhysics.CollisionAPI(ground).CreateCollisionEnabledAttr(False)
        for prim in floors:
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
        if UsdPhysics.CollisionAPI(ground).GetCollisionEnabledAttr().Get() is not False:
            raise RuntimeError("GroundPlane collision was not disabled")
        for path, prim in zip(config["floor_paths"], floors):
            if (
                not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                or str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())
                != "none"
            ):
                raise RuntimeError("Exact floor collision was not enabled: " + path)
        if not all(layer.ExportToString() == text for layer, text in sources):
            raise RuntimeError("A non-session source layer changed")
    except Exception:
        session.ImportFromString(session_before)
        raise
    finally:
        stage.SetEditTarget(previous_target)

    return {
        "scope": config["scope"],
        "floor_paths_enabled": list(config["floor_paths"]),
        "existing_lower_stair_colliders": stairs,
        "ground_plane_collision_enabled": False,
        "root_dirty_before": bool(root_dirty_before),
        "root_dirty_after": bool(stage.GetRootLayer().dirty),
        "source_layers_unchanged": True,
        "new_geometry": False,
        "source_saved": False,
        "provenance": config["provenance"],
    }


class SupportPreflightError(RuntimeError):
    """Failed live scene check with the complete collected report attached."""

    def __init__(self, report):
        self.report = report
        super().__init__("Original office scene support preflight failed")


def preflight_support(query, config_path=None):
    """Check actual PhysX hits at the configured original-scene samples.

    ``query`` is ``omni.physx.get_physx_scene_query_interface()`` from the live,
    initialized stage. All hits are recorded. Robot-owned hits are explicitly
    excluded from this static-scene check; every other nearer collider remains
    authoritative, including an unexpected obstacle. A lower floor cannot hide
    such an obstacle. This does not measure contacts or advance physics.
    """
    import carb

    config = load_config(config_path)
    settings = config["support_preflight"]
    targets = (
        set(config["floor_paths"])
        | set(config["stair_paths"])
        | set(config.get("walk_surface_paths", []))
    )
    checks = []
    for sample in settings["samples"]:
        expected = sample["surface_xyz_m"]
        origin = [
            expected[0],
            expected[1],
            expected[2] + settings["ray_origin_above_surface_m"],
        ]
        hits = []

        def record(hit, hits=hits):
            hits.append(
                {
                    "collision": str(hit.collision),
                    "rigid_body": str(getattr(hit, "rigid_body", "")),
                    "position": list(map(float, hit.position)),
                    "normal": list(map(float, hit.normal)),
                    "distance": float(hit.distance),
                }
            )
            return True

        query.raycast_all(
            carb.Float3(*origin),
            carb.Float3(0.0, 0.0, -1.0),
            settings["ray_length_m"],
            record,
            True,
        )
        ignored, scene, invalid = [], [], []
        for hit in hits:
            values = [*hit["position"], *hit["normal"], hit["distance"]]
            if (
                len(hit["position"]) != 3
                or len(hit["normal"]) != 3
                or not all(math.isfinite(value) for value in values)
                or not 0 <= hit["distance"] <= settings["ray_length_m"]
            ):
                invalid.append(hit)
            elif any(
                hit["collision"] == prefix or hit["collision"].startswith(prefix + "/")
                for prefix in settings["robot_paths_excluded_from_static_check"]
            ):
                ignored.append(hit)
            else:
                scene.append(hit)
        scene.sort(key=lambda hit: hit["distance"])
        nearest = scene[0] if scene else None
        nearest_hits = (
            [hit for hit in scene if hit["distance"] == nearest["distance"]]
            if nearest is not None
            else []
        )
        tolerance = settings["surface_tolerance_m"]
        passed = not invalid and nearest is not None
        if nearest is not None:
            passed = (
                passed
                and all(
                    hit["collision"] in targets
                    and hit["collision"] in sample["expected_colliders"]
                    and max(abs(hit["position"][i] - expected[i]) for i in range(3))
                    <= tolerance
                    and abs(hit["position"][2] - (origin[2] - hit["distance"]))
                    <= tolerance
                    for hit in nearest_hits
                )
                and not any(
                    hit["collision"] == config["ground_plane_path"] for hit in scene
                )
            )
        checks.append(
            {
                "label": sample["label"],
                "expected_surface_xyz_m": expected,
                "ray_origin_m": origin,
                "ray_direction": [0.0, 0.0, -1.0],
                "ray_length_m": settings["ray_length_m"],
                "all_hits": hits,
                "robot_hits_excluded": ignored,
                "invalid_hits": invalid,
                "nearest_scene_hit": nearest,
                "nearest_scene_hits": nearest_hits,
                "passed": bool(passed),
            }
        )
    report = {
        "scope": "live PhysX sampled static-scene support; not robot contact or traversal",
        "passed": bool(checks) and all(check["passed"] for check in checks),
        "checks": checks,
        "physics_advanced": False,
        "synthetic_or_filled_hits": False,
    }
    if not report["passed"]:
        raise SupportPreflightError(report)
    return report
