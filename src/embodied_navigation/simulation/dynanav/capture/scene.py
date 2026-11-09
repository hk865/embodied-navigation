"""Isaac Sim scene session for the DynaNav RGB-D capture probe.

This module owns everything that mutates the USD stage/session or the physics
application: SimulationApp creation, headless viewport display delegates, USD
stage open plus world/axes setup, basement collision repair, optional pedestrian
actor setup, the Go2 articulation spawn (asset reference, drive-ownership check,
training spawn properties), contact-report sensor installation, physics and
articulation initialisation, and the static support preflights. It is only
called from the explicit ``run()`` entry; importing it has no side effects.
"""

import os

import numpy as np

from embodied_navigation.project import data_root


def start_simulation_app(use_fabric):
    """Create the headless SimulationApp with the exact original settings."""
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "width": 512,
            "height": 512,
            "renderer": "RaytracedLighting",
            "create_new_stage": False,
            "disable_viewport_updates": True,
            "multi_gpu": False,
            "sync_loads": False,
            "limit_cpu_threads": 8,
            "extra_args": [
                "--/app/settings/persistent=0",
                "--/app/extensions/registryEnabled=false",
                "--/app/fastShutdown=true",
                "--/rtx/hydra/supportMultiTickRate=false",
                f"--/app/useFabricSceneDelegate={'true' if use_fabric else 'false'}",
            ],
        }
    )
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


def open_stage(scene, experiment, result, save):
    """Open the USD scene and establish Z-up metre world axes."""
    import omni.usd
    from pxr import UsdGeom

    assert omni.usd.get_context().open_stage(scene)
    stage = omni.usd.get_context().get_stage()
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


def setup_pedestrian(stage, experiment, result):
    """Create the optional session-only pedestrian actor."""
    pedestrian = None
    if experiment.get("pedestrian"):
        from embodied_navigation.simulation.dynanav.pedestrian import create_pedestrian

        pedestrian_config = experiment["pedestrian"]
        pedestrian = create_pedestrian(
            stage,
            pedestrian_config["asset"],
            start=pedestrian_config["start"],
            end=pedestrian_config["end"],
            speed_mps=pedestrian_config["speed_mps"],
            path_mode=pedestrian_config["path_mode"],
            animation_path=pedestrian_config["walk"],
        )
        result["pedestrian"] = pedestrian.report
    return pedestrian


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
        self._subscription = None

    def install(self, stage):
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
            if (
                any(path.startswith("/World/DynamicPedestrian") for path in paths)
                and any(path.startswith("/World/Go2") for path in paths)
                and len(self.dynamic_contact_events) < 1000
            ):
                self.dynamic_contact_events.append(
                    {
                        "physics_s": float(SimulationManager.get_simulation_time()),
                        "colliders": paths,
                        "actors": actors,
                        "contact_count": int(h.num_contact_data),
                    }
                )
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


def setup_physics(roots, use_fabric, app, result, save):
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
    result["runtime_render_path"] = {
        "requested": "fabric" if use_fabric else "usd",
        "physics_fabric_enabled": SimulationManager.is_fabric_enabled(),
        "fabric_scene_delegate": settings.get("/app/useFabricSceneDelegate"),
        "physics_update_to_usd": settings.get("/physics/updateToUsd"),
        "physics_suppress_readback": settings.get("/physics/suppressReadback"),
    }
    from isaacsim.core.rendering_manager import RenderingManager

    RenderingManager.set_dt(0.005)
    au.play()
    app.update()
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
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
    art.set_dof_max_velocities(np.full((1, 12), 30.0))
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


def create_pedestrian_view(stage, pedestrian):
    """Create the native rigid-prim view used for pedestrian readback."""
    from isaacsim.core.experimental.prims import RigidPrim

    pedestrian_view = None
    if pedestrian is not None:
        previous_edit_target = stage.GetEditTarget()
        try:
            stage.SetEditTarget(stage.GetSessionLayer())
            pedestrian_view = RigidPrim(
                pedestrian.root_path, reset_xform_op_properties=False
            )
        finally:
            stage.SetEditTarget(previous_edit_target)
        assert pedestrian_view.is_physics_tensor_entity_valid(), (
            "Pedestrian native rigid-body view unavailable"
        )
    return pedestrian_view


def update_pedestrian(pedestrian, experiment):
    """Advance the optional pedestrian actor for this physics step."""
    if pedestrian is None:
        return None
    from isaacsim.core.simulation_manager import SimulationManager

    actor_time = max(
        0.0,
        float(SimulationManager.get_simulation_time())
        - experiment["pedestrian"]["start_delay_s"],
    )
    return pedestrian.update(actor_time)


def run_preflights(experiment, pedestrian, result, save):
    """Run basement/pedestrian static support preflights before walking."""
    if experiment.get("basement_collision"):
        import omni.physx

        from embodied_navigation.simulation.dynanav.session_collision import (
            preflight_support,
        )

        result["support_preflight"] = preflight_support(
            omni.physx.get_physx_scene_query_interface(),
            config_path=experiment.get("collision_config_path"),
        )
    if (
        pedestrian is not None
        and experiment["pedestrian"].get("scenario") == "landing-crossing"
    ):
        from embodied_navigation.simulation.dynanav.pedestrian import (
            inspect_walk_support,
        )

        result["pedestrian_support"] = inspect_walk_support(
            omni.physx.get_physx_scene_query_interface(), pedestrian
        )
        save(force=True)
        if not result["pedestrian_support"]["passed"]:
            raise RuntimeError("Pedestrian three-point static support preflight failed")



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
