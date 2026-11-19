"""Isaac Sim scene session for the DynaNav RGB-D capture probe.

This module owns everything that mutates the USD stage/session or the physics
application: SimulationApp creation, headless viewport display delegates, USD
stage open plus world/axes setup, basement collision repair, the Go2 articulation
spawn (asset reference, drive-ownership check, training spawn properties),
contact-report sensor installation, physics and articulation initialisation, and
the static support preflights. It is only called from the explicit ``run()``
entry; importing it has no side effects.
"""

from collections import OrderedDict
import os
import hashlib
from pathlib import Path
import time

import numpy as np

from embodied_navigation.project import data_root


def start_simulation_app(use_fabric, *, rtx_lidar=False, log_to_stdout=False):
    """Create the legacy app, with required FSD/multi-tick only for explicit RTX."""
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "width": 512,
            "height": 512,
            "renderer": "RaytracedLighting",
            "enable_motion_bvh": rtx_lidar,
            "create_new_stage": False,
            "disable_viewport_updates": True,
            "multi_gpu": False,
            "sync_loads": False,
            "limit_cpu_threads": 8,
            "extra_args": [
                # The native launcher owns the bounded stdout/stderr sink. Keep
                # Info (the original Kit file threshold), without a second file.
                *(["--/log/file=", "--/log/enableStandardStreamOutput=true",
                   "--/log/outputStreamLevel=info"] if log_to_stdout else []),
                "--/app/settings/persistent=0",
                "--/app/extensions/registryEnabled=false",
                "--/app/fastShutdown=true",
                f"--/rtx/hydra/supportMultiTickRate={'true' if rtx_lidar else 'false'}",
                f"--/app/useFabricSceneDelegate={'true' if (use_fabric or rtx_lidar) else 'false'}",
                *(["--/rtx/rendering/perSensorTickTlas=true",
                   "--/app/settings/fabricDefaultSimPeriodNumerator=1",
                   "--/app/settings/fabricDefaultSimPeriodDenominator=200"] if rtx_lidar else []),
            ],
        }
    )
    if rtx_lidar:
        import omni.usd
        from isaacsim.core.rendering_manager import RenderingManager
        omni.usd.get_context().new_stage()
        # Configure history before the sensor stage/render products are created.
        RenderingManager.set_dt(0.005)
    return app


def install_headless_display_delegates():
    """Replace viewport display delegates so the headless app does not render UI."""
    from omni.kit.viewport.window import get_viewport_window_instances
    from omni.kit.widget.viewport.display_delegate import ViewportDisplayDelegate

    class HeadlessFixedDisplayDelegate(ViewportDisplayDelegate):
        def create(self, ui_frame, previous, *args, **kwargs):
            return None

        def destroy(self):
            pass

        @property
        def size(self):
            return (512, 512)

        def update(self, *args, **kwargs):
            return (512, 512)

    _headless_previous_delegates = []
    _headless_delegates = []
    for _viewport_window in list(get_viewport_window_instances()):
        _headless_previous_delegates.append(
            _viewport_window.viewport_widget.display_delegate
        )
        _delegate = HeadlessFixedDisplayDelegate(_viewport_window.viewport_api)
        _headless_delegates.append(_delegate)
        _viewport_window.viewport_widget.display_delegate = _delegate
    return _headless_previous_delegates, _headless_delegates


def resolve_spawn(experiment):
    """Resolve the spawn pose and real-time factor from experiment/env."""
    spawn = np.array(
        [
            float(os.environ.get("GO2_SPAWN_X", "-10.6")),
            float(os.environ.get("GO2_SPAWN_Y", "-14.5")),
            float(experiment.get("spawn", [0, 0, 0.4])[2]),
        ]
    )
    if experiment.get("spawn"):
        spawn = np.asarray(experiment["spawn"], dtype=float)
    spawn_yaw = float(experiment.get("spawn_yaw", os.environ.get("GO2_SPAWN_YAW", "0")))
    sim_rtf = float(os.environ.get("PROBE_RTF", ".5"))
    return spawn, spawn_yaw, sim_rtf


def prepare_rtx_scene_stage(scene):
    """Populate static scene geometry before Fabric observes the stage.

    RC7 Fabric can omit Office instance-proxy geometry when the original stage
    is opened first. Changing instanceable after renderer population is too
    late. Expand only the source scene's instances in its anonymous session
    layer, before attachment; the robot and sensors do not exist yet. No source
    geometry, transforms, materials, collision flags or asset files are edited.
    Keep FSD and physics Fabric enabled for real sensor motion/history.
    """
    from pxr import Sdf, Usd, UsdUtils

    source_hash = hashlib.sha256(Path(scene).read_bytes()).hexdigest()
    root = Sdf.Layer.FindOrOpen(str(scene))
    if root is None:
        raise RuntimeError("Cannot open the RTX source layer")
    session = Sdf.Layer.CreateAnonymous("dynanav-rtx-scene-session.usda")
    stage = Usd.Stage.Open(root, session)
    if stage is None:
        raise RuntimeError("Cannot prepare the RTX source scene")
    expanded = []
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        # Expanding an outer instance can expose nested instance roots.
        while True:
            instances = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsInstance()]
            if not instances:
                break
            if any(path in expanded for path in instances):
                raise RuntimeError("RTX scene instance expansion made no progress")
            for path in instances:
                # Editing an outer instance can rebuild descendant prim handles.
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.SetInstanceable(False):
                    raise RuntimeError(f"Cannot expand RTX scene instance {path}")
                expanded.append(path)
    after_hash = hashlib.sha256(Path(scene).read_bytes()).hexdigest()
    if after_hash != source_hash:
        raise RuntimeError("RTX scene source changed during session preparation")
    stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    return stage_id, {
        "method": "expand source scene instances in anonymous session before renderer attachment",
        "expanded_instance_count": len(expanded),
        "expanded_instance_paths": expanded,
        "remaining_instance_count": 0,
        "source_sha256_before": source_hash,
        "source_sha256_after": after_hash,
        "source_saved": False,
        "geometry_authored": False,
        "collision_attributes_authored": False,
        "robot_and_sensor_instances_included": False,
    }


def attach_prepared_rtx_stage(context, stage_id, *, update=None, timeout_s=120.0):
    """Wait for the SDK's asynchronous attachment completion, not acceptance.

    Clean up only this preparation's cache entry if attachment fails. Never
    clear the global cache or close a stage belonging to another caller.
    """
    from pxr import Usd, UsdUtils

    completed = []
    def on_finish(success, error):
        completed.append((bool(success), str(error)))

    try:
        if not context.attach_stage_with_callback(stage_id, on_finish):
            raise RuntimeError("Cannot request attachment of the prepared RTX scene")
        if update is None:
            import omni.kit.app
            update = omni.kit.app.get_app().update
        deadline = time.monotonic() + timeout_s
        while not completed:
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out attaching the prepared RTX scene")
            update()
        success, error = completed[0]
        if not success:
            raise RuntimeError(f"Prepared RTX scene attachment failed: {error}")
        if context.get_stage_id() != stage_id:
            raise RuntimeError("Prepared RTX scene stage is not the active stage")
    except BaseException:
        try:
            if context.get_stage_id() == stage_id:
                context.close_stage()
        finally:
            UsdUtils.StageCache.Get().Erase(Usd.StageCache.Id.FromLongInt(stage_id))
        raise


def open_stage(scene, experiment, result, save):
    """Open the USD scene and establish Z-up metre world axes."""
    import omni.usd
    from pxr import UsdGeom

    context = omni.usd.get_context()
    if experiment.get("point_cloud_source", "rgbd") == "rtx_lidar":
        stage_id, preparation = prepare_rtx_scene_stage(scene)
        attach_prepared_rtx_stage(context, stage_id)
        result["rtx_scene_preparation"] = preparation
        save()
    else:
        assert context.open_stage(scene)
    stage = context.get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if experiment.get("basement_collision"):
        from embodied_navigation.simulation.dynanav.session_collision import (
            repair_basement_session,
        )

        result["session_collision"] = repair_basement_session(
            stage, config_path=experiment.get("collision_config_path")
        )
        save()
    return stage


def spawn_articulation_asset(stage, spawn, experiment, result, save):
    """Reference the Go2 asset, verify drive ownership and apply spawn properties."""
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    asset = str(
        data_root() / "runtime_assets/robot/go2_description/go2_description.usda"
    )
    prim = stage.DefinePrim("/World/Go2")
    prim.GetReferences().AddReference(asset)
    result["stage"] = "asset_loading"
    save()
    x = UsdGeom.Xformable(prim)
    ops = x.GetOrderedXformOps()
    translate = next(
        (o for o in ops if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None
    )
    (translate or x.AddTranslateOp()).Set(Gf.Vec3d(*spawn))
    roots = [
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    result["articulation_roots"] = roots
    result["asset"] = asset
    save()
    assert len(roots) == 1
    # Force drive gains are expected zero in this converted asset; refuse competing drive ownership.
    drives = []
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(p, "angular")
            stiff = drive.GetStiffnessAttr().Get() if drive else 0.0
            damp = drive.GetDampingAttr().Get() if drive else 0.0
            drives.append(
                {"path": str(p.GetPath()), "stiffness": stiff, "damping": damp}
            )
            assert float(stiff or 0) == 0 and float(damp or 0) == 0, (
                "Existing nonzero joint drive; refuse second actuator"
            )
    result["existing_drives"] = drives
    # Match documented training spawn properties; no gain/torque tuning.
    for p in stage.Traverse():
        if str(p.GetPath()).startswith("/World/Go2") and p.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
            api.CreateLinearDampingAttr(0.0)
            api.CreateAngularDampingAttr(0.0)
            api.CreateMaxDepenetrationVelocityAttr(1.0)
    for path in roots:
        api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateEnabledSelfCollisionsAttr(False)
        api.CreateSolverPositionIterationCountAttr(8)
        api.CreateSolverVelocityIterationCountAttr(4)
    result["training_spawn_properties"] = {
        "solver_position_iterations": 8,
        "solver_velocity_iterations": 4,
        "self_collisions": False,
        "linear_and_angular_damping": 0.0,
        "max_depenetration_velocity": 1.0,
        "source": "RL_for_unitree/assets/robots.py _common_spawn",
    }
    return roots


class ContactMonitor:
    """Own the PhysX full-contact-report subscription and its bounded evidence.

    ``contact_state`` is the five-slot [body, FR, FL, RR, RL] counter reset every
    physics step; ``contact_events``/``dynamic_contact_events``/``contact_paths``
    are the receipts appended by the same callback the original script used.
    """

    def __init__(self):
        self.contact_state = [0, 0, 0, 0, 0]
        self.contact_events = []
        self.dynamic_contact_events = []
        self.contact_paths = {}
        self.callback_count = 0
        self.contact_pair_summary = {
            "version": 1, "pair_capacity": 128, "pair_evictions": 0,
            "robot_contact_headers": 0, "robot_contact_points": 0,
            "evicted_contact_headers": 0, "evicted_contact_points": 0,
            "observation_only": True,
            "scope": "All pairs with a Go2 collider or actor and positive contact-data count",
            "time_semantics": "First/last positive contact report, not contact start/end",
            "pair_statistics_scope": "Since latest LRU retention; evicted pair history is not preserved",
            "impulse_metric": "Maximum native point impulse Euclidean norm, not force",
            "pairs": [],
        }
        self._contact_pairs = OrderedDict()
        self._subscription = None

    def install(self, stage):
        """Subscribe and return the live bounded receipt for existing result saves."""
        from pxr import PhysxSchema, UsdPhysics

        self.stage = stage
        for p in stage.Traverse():
            if str(p.GetPath()).startswith("/World/Go2") and p.HasAPI(
                UsdPhysics.RigidBodyAPI
            ):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)

        import omni.physx

        self._subscription = (
            omni.physx.get_physx_simulation_interface()
            .subscribe_full_contact_report_events(self.on_contact)
        )
        return self.contact_pair_summary

    def _summarize_pair(self, paths, actors, header, data, physics_s):
        # Sort associated collider/actor tuples so reversed reports share a pair.
        pair = tuple(sorted(zip(paths, actors)))
        summary = self.contact_pair_summary
        summary["robot_contact_headers"] += 1
        summary["robot_contact_points"] += int(header.num_contact_data)
        if pair not in self._contact_pairs:
            if len(self._contact_pairs) == summary["pair_capacity"]:
                _, evicted = self._contact_pairs.popitem(last=False)
                summary["pair_evictions"] += 1
                summary["evicted_contact_headers"] += evicted["header_count"]
                summary["evicted_contact_points"] += evicted["contact_point_count"]
            self._contact_pairs[pair] = {
                "colliders": [p[0] for p in pair], "actors": [p[1] for p in pair],
                "first_physics_s": physics_s, "last_physics_s": physics_s,
                "header_count": 0, "contact_point_count": 0,
                "max_point_impulse_norm": 0.0,
            }
            summary["pairs"] = list(self._contact_pairs.values())
        row = self._contact_pairs[pair]
        self._contact_pairs.move_to_end(pair)
        row["last_physics_s"] = physics_s
        row["header_count"] += 1
        row["contact_point_count"] += int(header.num_contact_data)
        for point in data[header.contact_data_offset:
                          header.contact_data_offset + header.num_contact_data]:
            row["max_point_impulse_norm"] = max(
                row["max_point_impulse_norm"], float(np.linalg.norm(point.impulse)))

    def begin_step(self):
        self.contact_state[:] = [0, 0, 0, 0, 0]

    def on_contact(self, headers, data, anchors=None):
        from pxr import PhysicsSchemaTools, UsdGeom
        from isaacsim.core.simulation_manager import SimulationManager

        stage = self.stage
        self.callback_count += 1
        for h in headers:
            if h.num_contact_data <= 0:
                continue
            paths = [
                str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
            ]
            actors = [
                str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
            ]
            if any(path.startswith("/World/Go2") for path in paths + actors):
                self._summarize_pair(paths, actors, h, data,
                                     float(SimulationManager.get_simulation_time()))
            for j, path in enumerate(paths):
                if not path.startswith("/World/Go2"):
                    continue
                key = None
                if actors[j] == "/World/Go2/Geometry/base":
                    key = 0
                for k, leg in enumerate(["FR", "FL", "RR", "RL"], 1):
                    if actors[j].split("/")[-1] == leg + "_calf" and path.endswith(
                        "/sphere"
                    ):
                        sphere = UsdGeom.Sphere(stage.GetPrimAtPath(path))
                        assert (
                            abs(float(sphere.GetRadiusAttr().Get()) - 0.022) < 1e-6
                        ), "Foot shape differs"
                        key = k
                self.contact_paths[path] = {"actor": actors[j], "group": key}
                if key is not None:
                    self.contact_state[key] += 1
            if len(self.contact_events) < 3000:
                ratios = []
                for pnt in data[
                    h.contact_data_offset : h.contact_data_offset + h.num_contact_data
                ]:
                    normal = np.array(pnt.normal)
                    impulse = np.array(pnt.impulse)
                    fn = float(abs(impulse @ normal))
                    ft = float(np.linalg.norm(impulse - (impulse @ normal) * normal))
                    ratios.append(
                        {
                            "normal_impulse": fn,
                            "tangent_impulse": ft,
                            "ratio": ft / fn if fn > 1e-8 else None,
                        }
                    )
                anchor_impulses = (
                    [
                        list(map(float, x.impulse))
                        for x in anchors[
                            h.friction_anchors_offset : h.friction_anchors_offset
                            + h.num_friction_anchors_data
                        ]
                    ]
                    if anchors is not None
                    else []
                )
                self.contact_events.append(
                    {
                        "physics_time": float(SimulationManager.get_simulation_time()),
                        "friction_anchor_impulses": anchor_impulses,
                        "colliders": paths,
                        "actors": actors,
                        "count": int(h.num_contact_data),
                        "impulses": ratios,
                    }
                )


def setup_physics(roots, use_fabric, app, result, save, *, rtx_lidar=False):
    """Create the articulation/physics views and install the 0.005 s clock."""
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager
    import isaacsim.core.experimental.utils.app as au
    import omni.replicator.core as rep

    art = Articulation(roots[0])
    SimulationManager.enable_fabric(use_fabric)
    SimulationManager.setup_simulation(dt=0.005, device="cpu")
    import carb.settings

    settings = carb.settings.get_settings()
    if rtx_lidar:
        # RTX uses Fabric motion history. Keep the native USD result mirror
        # current too for the existing RGB render-product pipeline.
        settings.set_bool("/physics/updateToUsd", True)
    result["runtime_render_path"] = {
        "requested": "fabric" if use_fabric else "usd",
        "physics_fabric_enabled": SimulationManager.is_fabric_enabled(),
        "fabric_scene_delegate": settings.get("/app/useFabricSceneDelegate"),
        "physics_update_to_usd": settings.get("/physics/updateToUsd"),
        "physics_suppress_readback": settings.get("/physics/suppressReadback"),
        "support_multi_tick_rate": settings.get("/rtx/hydra/supportMultiTickRate"),
        "per_sensor_tick_tlas": settings.get("/rtx/rendering/perSensorTickTlas"),
    }
    from isaacsim.core.rendering_manager import RenderingManager

    RenderingManager.set_dt(0.005)
    au.play()
    app.update()
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True, wait_for_render=not rtx_lidar)
    if rtx_lidar:
        RenderingManager.render()
        RenderingManager.render()
    result["dof_names"] = art.dof_names
    result["link_names"] = art.link_names
    save()
    return art


def initialize_articulation(art, stage, spawn, spawn_yaw, out, imu_mount, result, save):
    """Check the joint contract and initialise the articulation state."""
    import json

    import numpy as np
    from isaacsim.core.experimental.prims import RigidPrim
    from embodied_navigation.project import project_root

    contract = json.loads(
        (
            project_root()
            / "src/embodied_navigation/robot/unitree/policy/native/contract.json"
        ).read_text()
    )
    canonical = contract["joint_names"]
    assert len(art.dof_names) == 12 and set(art.dof_names) == set(canonical), (
        "Asset joint identity differs from navigation contract"
    )
    indices = [art.dof_names.index(n) for n in canonical]
    result["canonical_to_isaac_dof_indices"] = indices
    target = np.zeros(12, dtype=np.float32)
    for name, q in zip(canonical, contract["default_joint_positions"]):
        target[art.dof_names.index(name)] = q
    result["root_pose_before_spawn"] = [
        a.numpy().tolist() for a in art.get_world_poses()
    ]
    art.set_world_poses(
        positions=[spawn.tolist()],
        orientations=[[np.cos(spawn_yaw / 2), 0.0, 0.0, np.sin(spawn_yaw / 2)]],
    )
    result["root_pose_after_spawn"] = [
        a.numpy().tolist() for a in art.get_world_poses()
    ]
    result["root_velocity_before_reset"] = [
        a.numpy().tolist() for a in art.get_velocities()
    ]
    art.set_velocities(
        linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]]
    )
    result["root_velocity_after_reset"] = [
        a.numpy().tolist() for a in art.get_velocities()
    ]
    save()
    art.set_dof_positions(target[None, :])
    art.set_dof_velocities(np.zeros((1, 12), np.float32))
    art.set_dof_max_efforts(np.full((1, 12), 23.5))
    max_velocities = np.full((1, 12), 30.0)
    art.set_dof_max_velocities(max_velocities)
    result["dof_max_velocities_radps"] = {
        "requested": max_velocities[0].tolist(),
        "readback": art.get_dof_max_velocities().numpy()[0].tolist(),
        "physics_tensor_view_valid": art.is_physics_tensor_entity_valid(),
    }
    calf_views = [
        RigidPrim(
            "/World/Go2/Geometry/base/"
            + leg
            + "_hip/"
            + leg
            + "_thigh/"
            + leg
            + "_calf",
            reset_xform_op_properties=False,
        )
        for leg in ["FR", "FL", "RR", "RL"]
    ]
    from embodied_navigation.simulation.dynanav.collision_bounds import export_bounds

    export_bounds(stage, out)
    result["link_coms_local"] = [v.numpy().tolist() for v in art.get_link_coms()]
    assert np.allclose(
        np.array(result["link_coms_local"][0][0][0]), imu_mount, atol=1e-6
    ), "USD and physical COM disagree"
    result["implicit_gains_runtime"] = [v.numpy().tolist() for v in art.get_dof_gains()]
    assert all(np.allclose(v.numpy(), 0) for v in art.get_dof_gains()), (
        "Competing drive after initialization"
    )
    qdefault = np.array(contract["default_joint_positions"])
    return contract, canonical, indices, target, qdefault


def run_preflights(experiment, result, save):
    """Run basement static support preflights before walking."""
    if experiment.get("basement_collision"):
        import omni.physx

        from embodied_navigation.simulation.dynanav.session_collision import (
            preflight_support,
        )

        result["support_preflight"] = preflight_support(
            omni.physx.get_physx_scene_query_interface(),
            config_path=experiment.get("collision_config_path"),
        )



def play():
    """Play the timeline (``au.play()``) without importing Isaac at module load."""
    import isaacsim.core.experimental.utils.app as au

    au.play()


def stop(app, previous_delegates):
    """Stop and close the app in the original success-cleanup order."""
    import isaacsim.core.experimental.utils.app as au

    au.stop()
    previous_delegates.clear()
    app.close()
