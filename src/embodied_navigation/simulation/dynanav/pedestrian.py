"""Optional local human actor, authored only in the live USD session.

No Isaac/pxr imports or simulator startup occur at import time.  The rendered
human and its explicitly approximate capsule collider share one kinematic root.
The helper adds no scene support geometry and never saves an asset or stage.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

ROOT_PATH = "/World/DynamicPedestrian"


@dataclass(frozen=True)
class PedestrianPath:
    """Bounded horizontal motion; ``speed_mps`` is the maximum path speed.

    ``sine`` and ``linear`` repeat between endpoints. ``one_way`` travels once
    at constant speed, then holds the endpoint and forward heading indefinitely.
    That mode defines a single-crossing experiment, not general avoidance.
    """

    start: tuple[float, float, float]
    end: tuple[float, float, float]
    speed_mps: float = 0.6
    mode: str = "sine"

    def __post_init__(self):
        for name in ("start", "end"):
            value = tuple(map(float, getattr(self, name)))
            if len(value) != 3 or not all(map(math.isfinite, value)):
                raise ValueError("Pedestrian endpoints must be finite XYZ coordinates")
            object.__setattr__(self, name, value)
        if abs(self.start[2] - self.end[2]) > 1e-6:
            raise ValueError("Pedestrian path must be horizontal")
        if self.length <= 1e-6:
            raise ValueError("Pedestrian path needs distinct endpoints")
        if not math.isfinite(self.speed_mps) or self.speed_mps <= 0:
            raise ValueError("Pedestrian speed must be positive and finite")
        if self.mode not in {"sine", "linear", "one_way"}:
            raise ValueError("Pedestrian path mode must be sine, linear or one_way")

    @property
    def length(self):
        return math.dist(self.start, self.end)

    def pose(self, sim_time):
        """Return foot position and heading within the declared path mode.

        ``linear`` reverses at endpoints; ``one_way`` stops at its first arrival.
        """
        sim_time = float(sim_time)
        if not math.isfinite(sim_time) or sim_time < 0:
            raise ValueError("Simulation time must be finite and nonnegative")
        travel = sim_time * self.speed_mps / self.length
        if self.mode == "sine":
            phase = 2.0 * travel
            fraction = 0.5 * (1.0 - math.cos(phase))
            forward = math.sin(phase) >= 0
        elif self.mode == "one_way":
            fraction = min(travel, 1.0)
            forward = True
        else:
            phase = travel % 2.0
            fraction = phase if phase <= 1.0 else 2.0 - phase
            forward = phase < 1.0
        position = (
            self.end
            if self.mode == "one_way" and sim_time >= self.length / self.speed_mps
            else tuple(a + fraction * (b - a) for a, b in zip(self.start, self.end))
        )
        heading = math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])
        return position, heading if forward else heading + math.pi


class PedestrianActor:
    """Live actor handle. Call ``update(sim_time)`` before each physics step.

    In ``one_way`` mode both root motion and the skeletal sample freeze at first
    arrival. The visible human remains a collision obstacle at that endpoint;
    the final gait pose is held, rather than replaced with an idle animation.
    """

    def __init__(self, stage, path, translate, rotate, walk_update, report):
        self.stage = stage
        self.path = path
        self.root_path = ROOT_PATH
        self.report = report
        self._translate = translate
        self._rotate = rotate
        self._walk_update = walk_update

    def update(self, sim_time):
        from pxr import Gf

        position, heading = self.path.pose(sim_time)
        previous_target = self.stage.GetEditTarget()
        try:
            self.stage.SetEditTarget(self.stage.GetSessionLayer())
            self._translate.Set(Gf.Vec3d(*position))
            # The installed biped walks along +Z before Y-up -> Z-up correction.
            self._rotate.Set(math.degrees(heading) + 90.0)
            if self._walk_update is not None:
                sample_time = float(sim_time)
                if self.path.mode == "one_way":
                    sample_time = min(
                        sample_time, self.path.length / self.path.speed_mps
                    )
                self._walk_update(sample_time)
        finally:
            self.stage.SetEditTarget(previous_target)
        return {
            "sim_time_s": float(sim_time),
            "position_m": list(position),
            "heading_rad": heading,
        }


def _local_usd(path):
    from pxr import Usd

    path = Path(path).expanduser().resolve(strict=True)
    if not path.is_file() or path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise ValueError("A local USD file is required")
    stage = Usd.Stage.Open(str(path))
    if not stage or not stage.GetDefaultPrim():
        raise ValueError("Pedestrian USD needs a valid default prim")
    return path, stage


def _configure_walk(stage, human, skeleton, animation_path, source_units):
    from pxr import Gf, UsdGeom, UsdSkel, Vt

    path, source_stage = _local_usd(animation_path)
    source = UsdSkel.Animation(source_stage.GetDefaultPrim())
    if not source or source.GetJointsAttr().Get() != skeleton.GetJointsAttr().Get():
        raise ValueError(
            "Walk animation must exactly match the local human skeleton joints"
        )
    samples = source.GetTranslationsAttr().GetTimeSamples()
    rate = source_stage.GetTimeCodesPerSecond()
    units = UsdGeom.GetStageMetersPerUnit(source_stage) / source_units
    if len(samples) < 2 or samples[-1] <= samples[0] or rate <= 0:
        raise ValueError("Walk animation needs a nonempty time-sampled cycle")
    joints = source.GetJointsAttr().Get()
    if not joints or str(joints[0]) != "Root":
        raise ValueError("Walk animation requires the installed biped Root joint")
    for time in (samples[0], samples[-1]):
        for attribute in (
            source.GetTranslationsAttr(),
            source.GetRotationsAttr(),
            source.GetScalesAttr(),
        ):
            if len(attribute.Get(time) or []) != len(joints):
                raise ValueError("Walk animation has incomplete joint transforms")
    animation = UsdSkel.Animation.Define(stage, str(human.GetPath()) + "/LocalWalk")
    animation.CreateJointsAttr(joints)
    translation = animation.CreateTranslationsAttr()
    rotation = animation.CreateRotationsAttr()
    scale = animation.CreateScalesAttr()
    UsdSkel.BindingAPI.Apply(skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets(
        [animation.GetPath()]
    )

    def update(sim_time):
        # Hold source_stage alive, and sample without changing the scene timeline.
        time = samples[0] + (sim_time * rate) % (samples[-1] - samples[0])
        values = Vt.Vec3fArray(
            [
                Gf.Vec3f(*(float(v) * units for v in xyz))
                for xyz in source.GetTranslationsAttr().Get(time)
            ]
        )
        root = values[0]
        values[0] = Gf.Vec3f(0.0, root[1], 0.0)
        translation.Set(values)
        rotation.Set(source.GetRotationsAttr().Get(time))
        scale.Set(source.GetScalesAttr().Get(time))
        _ = source_stage

    update(0.0)
    return update, {
        "mode": "local_skeletal_walk",
        "asset": str(path),
        "joint_count": len(joints),
        "source_frames": [samples[0], samples[-1]],
        "source_time_codes_per_second": rate,
        "translation_unit_scale": units,
        "root_horizontal_motion_removed": True,
        "playback_speed": "source cadence independent of actor translation",
    }


def create_pedestrian(
    stage,
    asset_path,
    *,
    start,
    end,
    speed_mps=0.6,
    path_mode="sine",
    animation_path=None,
    collision_radius_m=0.3,
):
    """Reference a local Y-up human and return its session-only actor handle.

    ``start`` and ``end`` are world-space foot positions in a Z-up meter stage.
    The optional walk must use exactly the same skeleton joints; its horizontal
    root travel is removed. A capsule approximates the human collision volume,
    including when its arms/legs extend beyond the capsule. It is not a mesh
    collider or a contact/avoidance validation. Default pose is explicitly static.
    ``path_mode="one_way"`` specifies a single crossing followed by an endpoint
    hold of root and skeletal pose; it does not test repeated or blind-side
    approaches, or establish general avoidance.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdSkel, Vt

    path = PedestrianPath(start, end, float(speed_mps), path_mode)
    if (
        str(UsdGeom.GetStageUpAxis(stage)) != "Z"
        or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9
    ):
        raise ValueError("Pedestrian requires a Z-up meter scene")
    if not math.isfinite(collision_radius_m) or collision_radius_m <= 0:
        raise ValueError("Collision proxy radius must be positive and finite")
    asset_path, asset_stage = _local_usd(asset_path)
    if str(UsdGeom.GetStageUpAxis(asset_stage)) != "Y":
        raise ValueError("Pedestrian helper expects the installed Y-up human")
    units = UsdGeom.GetStageMetersPerUnit(asset_stage)
    meshes = [
        p
        for p in Usd.PrimRange(asset_stage.GetDefaultPrim())
        if p.IsA(UsdGeom.Mesh)
        and len(UsdGeom.Mesh(p).GetPointsAttr().Get() or []) > 0
        and len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get() or []) > 0
        and UsdGeom.Imageable(p).ComputeVisibility() != UsdGeom.Tokens.invisible
        and UsdGeom.Imageable(p).ComputePurpose()
        in {UsdGeom.Tokens.default_, UsdGeom.Tokens.render}
    ]
    if not meshes:
        raise ValueError("Local pedestrian asset has no visible renderable mesh")
    if stage.GetPrimAtPath(ROOT_PATH):
        raise ValueError("Dynamic pedestrian root already exists")
    session = stage.GetSessionLayer()
    if not session:
        raise ValueError("Pedestrian requires a USD session layer")
    before = session.ExportToString()
    previous_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(session)
        root = UsdGeom.Xform.Define(stage, ROOT_PATH)
        translate = root.AddTranslateOp()
        rotate = root.AddRotateZOp()
        rigid = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        rigid.CreateRigidBodyEnabledAttr(True)
        rigid.CreateKinematicEnabledAttr(True)
        visual = UsdGeom.Xform.Define(stage, ROOT_PATH + "/VisualFrame")
        foot_offset = visual.AddTranslateOp()
        visual.AddRotateXOp().Set(90.0)
        visual.AddScaleOp().Set(Gf.Vec3d(units))
        human = UsdSkel.Root.Define(stage, ROOT_PATH + "/VisualFrame/Human")
        human.GetPrim().GetReferences().AddReference(str(asset_path))
        skeletons = [
            UsdSkel.Skeleton(p)
            for p in Usd.PrimRange(human.GetPrim())
            if p.IsA(UsdSkel.Skeleton)
        ]
        walk_update, animation_report = None, {"mode": "translating_static_pose"}
        if animation_path is not None:
            if len(skeletons) != 1:
                raise ValueError("Walk requires exactly one local human skeleton")
            walk_update, animation_report = _configure_walk(
                stage, human, skeletons[0], animation_path, units
            )
        else:
            # The installed biped has a dangling animationSource relationship.
            for skeleton in skeletons:
                UsdSkel.BindingAPI.Apply(
                    skeleton.GetPrim()
                ).CreateAnimationSourceRel().SetTargets([])
        if walk_update is not None:
            cache = UsdSkel.Cache()
            cache.Populate(human, Usd.PrimDefaultPredicate)
            transforms = cache.GetSkelQuery(skeletons[0]).ComputeSkinningTransforms(
                Usd.TimeCode.Default()
            )
            low, high = math.inf, -math.inf
            for prim in Usd.PrimRange(human.GetPrim()):
                if prim.IsA(UsdGeom.Mesh):
                    points = Vt.Vec3fArray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
                    query = cache.GetSkinningQuery(prim)
                    if not query or not query.ComputeSkinnedPoints(
                        transforms, points, Usd.TimeCode.Default()
                    ):
                        raise ValueError("Local walk failed actual mesh skinning")
                    low = min(low, min(float(p[1]) for p in points))
                    high = max(high, max(float(p[1]) for p in points))
        else:
            bounds = Gf.Range3d()
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
            for mesh in meshes:
                bounds.UnionWith(cache.ComputeWorldBound(mesh).ComputeAlignedRange())
            low, high = bounds.GetMin()[1], bounds.GetMax()[1]
        height = (high - low) * units
        if not 0.5 < height < 3.0 or 2 * collision_radius_m >= height:
            raise ValueError(
                "Local human dimensions are incompatible with capsule proxy"
            )
        foot_offset.Set(Gf.Vec3d(0.0, 0.0, -low * units))
        capsule = UsdGeom.Capsule.Define(stage, ROOT_PATH + "/CollisionProxy")
        capsule.CreateAxisAttr("Z")
        capsule.CreateRadiusAttr(collision_radius_m)
        capsule.CreateHeightAttr(height - 2 * collision_radius_m)
        capsule.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
        capsule.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim()).CreateCollisionEnabledAttr(
            True
        )
        report = {
            "root_path": ROOT_PATH,
            "asset": str(asset_path),
            "asset_mesh_count": len(meshes),
            "source_up_axis": "Y",
            "source_meters_per_unit": units,
            "animation": animation_report,
            "path_mode": path_mode,
            "path_scope": "single_crossing_then_endpoint_hold"
            if path_mode == "one_way"
            else "repeating_bounded_path",
            "path_arrival_time_s": path.length / path.speed_mps
            if path_mode == "one_way"
            else None,
            "endpoint_skeletal_pose_held": path_mode == "one_way"
            and walk_update is not None,
            "start_m": list(path.start),
            "end_m": list(path.end),
            "max_speed_mps": path.speed_mps,
            "collision": {
                "type": "kinematic_capsule_approximation",
                "radius_m": collision_radius_m,
                "total_height_m": height,
                "rendered": False,
                "root_shared_with_human": True,
            },
            "foot_offset_m": -low * units,
            "session_only": True,
            "source_saved": False,
        }
        actor = PedestrianActor(stage, path, translate, rotate, walk_update, report)
        actor.update(0.0)
    except Exception:
        session.ImportFromString(before)
        raise
    finally:
        stage.SetEditTarget(previous_target)
    return actor


def inspect_walk_support(query, actor, tolerance_m=0.02):
    """Inspect real static PhysX support at path start, midpoint and end only.

    Call after the live scene's physics setup. ``query`` must be the actual
    PhysX scene-query interface; this function neither advances physics nor
    creates geometry. All callback hits are retained. Only Go2-owned and this
    actor's own hits are excluded, using collision/rigid-body ownership paths.
    The nearest remaining hit(s) must be an original configured floor/stair mesh
    with no rigid-body ancestor, enabled collision, and XYZ near the authored
    foot position. Unknown nearer obstacles remain authoritative and fail.

    The returned ``passed`` is three-point static support admission, not whole
    path coverage, human volume collision clearance, contact or traversal proof.
    A failed report must cause the caller to stop the experiment case.
    """
    import carb
    from pxr import UsdGeom, UsdPhysics

    from embodied_navigation.simulation.dynanav.session_collision import load_config

    tolerance_m = float(tolerance_m)
    if not math.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ValueError("Walk support tolerance must be positive and finite")
    config = load_config()
    allowed = set(config["floor_paths"]) | set(config["stair_paths"])
    excluded_roots = ["/World/Go2", str(actor.root_path)]
    above = max(0.12, 2.0 * tolerance_m)
    ray_length = 2.0 * above
    start, end = actor.path.start, actor.path.end
    samples = [
        ("start", start),
        ("midpoint", tuple((a + b) / 2 for a, b in zip(start, end))),
        ("end", end),
    ]
    checks = []
    for label, expected in samples:
        expected = list(map(float, expected))
        origin = [expected[0], expected[1], expected[2] + above]
        hits, errors = [], []

        def record(hit, *, hits=hits):
            try:
                hits.append(
                    {
                        "collision": str(hit.collision),
                        "rigid_body": str(getattr(hit, "rigid_body", "")),
                        "position": list(map(float, hit.position)),
                        "normal": list(map(float, hit.normal)),
                        "distance": float(hit.distance),
                    }
                )
            except Exception as error:  # noqa: BLE001 - native scene queries can raise SDK-specific errors.  # noqa: BLE001 - retain malformed native SDK hits as failed evidence.
                hits.append({"decode_error": repr(error)})
            return True

        try:
            query.raycast_all(
                carb.Float3(*origin),
                carb.Float3(0.0, 0.0, -1.0),
                ray_length,
                record,
                True,
            )
        except Exception as error:  # noqa: BLE001 - native scene queries can raise SDK-specific errors.
            errors.append("query_error: " + repr(error))
        excluded, scene, invalid = [], [], []
        for hit in hits:
            if "decode_error" in hit:
                invalid.append(hit)
                continue
            values = [*hit["position"], *hit["normal"], hit["distance"]]
            if (
                len(hit["position"]) != 3
                or len(hit["normal"]) != 3
                or not all(map(math.isfinite, values))
                or not 0 <= hit["distance"] <= ray_length
            ):
                invalid.append(hit)
                continue
            owner = next(
                (
                    root
                    for root in excluded_roots
                    if any(
                        path == root or path.startswith(root + "/")
                        for path in (hit["collision"], hit["rigid_body"])
                    )
                ),
                None,
            )
            if owner is not None:
                excluded.append({"excluded_owner_path": owner, "hit": hit})
            else:
                scene.append(hit)
        scene.sort(key=lambda hit: hit["distance"])
        nearest = scene[0] if scene else None
        nearest_hits = (
            [hit for hit in scene if hit["distance"] == nearest["distance"]]
            if nearest
            else []
        )
        if invalid:
            errors.append("invalid_hit_record")
        if not nearest:
            errors.append("no_nonexcluded_hit")
        for hit in nearest_hits:
            prim = actor.stage.GetPrimAtPath(hit["collision"])
            static = bool(
                prim
                and prim.IsActive()
                and prim.IsA(UsdGeom.Mesh)
                and prim.HasAPI(UsdPhysics.CollisionAPI)
                and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            )
            cursor = prim
            while cursor and not cursor.IsPseudoRoot():
                if cursor.HasAPI(UsdPhysics.RigidBodyAPI):
                    static = False
                cursor = cursor.GetParent()
            if hit["collision"] not in allowed or not static:
                errors.append("nearest_hit_is_not_original_static_support")
            if (
                max(abs(hit["position"][axis] - expected[axis]) for axis in range(3))
                > tolerance_m
            ):
                errors.append("nearest_surface_xyz_outside_tolerance")
            if abs(hit["position"][2] - (origin[2] - hit["distance"])) > tolerance_m:
                errors.append("nearest_hit_ray_geometry_inconsistent")
        checks.append(
            {
                "label": label,
                "expected_foot_xyz_m": expected,
                "ray_origin_m": origin,
                "ray_direction": [0.0, 0.0, -1.0],
                "ray_length_m": ray_length,
                "all_hits": hits,
                "excluded_hits": excluded,
                "invalid_hits": invalid,
                "nearest_scene_hit": nearest,
                "nearest_scene_hits": nearest_hits,
                "failure_reasons": sorted(set(errors)),
                "passed": not errors,
            }
        )
    return {
        "scope": "three_point_live_PhysX_static_walk_support_only",
        "passed": all(check["passed"] for check in checks),
        "checks": checks,
        "surface_tolerance_m": tolerance_m,
        "allowed_original_support_paths": sorted(allowed),
        "excluded_ownership_paths": excluded_roots,
        "all_callback_hits_recorded": True,
        "physics_advanced": False,
        "new_geometry": False,
        "source_saved": False,
        "synthetic_or_filled_hits": False,
        "whole_path_support_verified": False,
        "human_volume_clearance_verified": False,
        "contact_or_traversal_verified": False,
    }
