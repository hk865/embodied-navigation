"""单次 0.005 s 物理步进及其时钟、力矩与安全断言。

本模块负责每步的执行顺序：写入并回读关节力矩、执行一步物理（Fabric 或 USD
更新）、校验仿真时钟与 IMU 时钟、执行姿态/接触/速度/位置安全门，并返回
供帧处理与控制阶段使用的 IMU/base 测量。所有阈值与探针默认值保持原样；
0.005 s 步长对应 200 Hz 物理频率。
"""

from dataclasses import dataclass

from embodied_navigation.common.time import float32_sensor_time_tolerance

import numpy as np


def _native_static_exploration(experiment):
    """Scope prototype acceptance envelopes to static, simulation-only native runs."""
    return (
        experiment.get("control_mode") == "interactive_waypoint_exploration"
        and experiment.get("interactive_controller") in ("native_route", "pct_scan")
        and experiment.get("interactive_simulation_only") is True
    )


def _record_configured_bound(result, name, observation, save):
    """Retain threshold events as evidence; never send them to navigation."""
    if observation.get("startup_exemption", False) or observation["within_guard"]:
        return
    count_key = name + "_out_of_band_steps"
    count = result.get(count_key, 0) + 1
    result[count_key] = count
    result.setdefault(name + "_first_out_of_band", dict(observation))
    result[name + "_last_out_of_band"] = dict(observation)
    if count == 1:
        save(force=True)


@dataclass
class Measurement:
    """步进后测量值，供控制、帧处理与收据阶段共享。

    ``tau`` 为本步写入的关节力矩；``actual_time`` 为仿真时间（s），
    ``it`` 为 IMU 采样时间（s）；``pos``/``quat`` 为 base 世界位姿，
    ``v``/``w`` 为 base 世界速度/角速度；``Rb`` 为 body→world 旋转；
    ``sensor_position``/``camera_position`` 已按安装外参变换到世界系。
    """

    tau: np.ndarray
    actual_time: float
    pos: np.ndarray
    quat: np.ndarray
    v: object
    w: object
    it: float
    reading: dict
    Rb: np.ndarray
    sensor_position: np.ndarray
    camera_position: np.ndarray
    previous_completed_tau: np.ndarray


class PhysicsStepper:
    """拥有单步 0.005 s 步进及跨步保留的“上一 IMU 样本”状态。

    ``use_fabric`` 决定物理步后是否走 Fabric 更新路径；``physical_speed_guard``
    为允许的 base 线速度上限（m/s）；``last_imu_time`` 与
    ``previous_completed_tau`` 跨步保留，用于检查 IMU 采样连续性。
    """

    def __init__(
        self,
        *,
        art,
        experiment,
        indices,
        use_fabric,
        imu_sensor,
        receipts,
        spawn,
        physical_speed_guard,
        imu_mount,
        mount,
        sync_usd_for_camera=False,
        capture_render_outputs=None,
    ):
        self.capture_render_outputs = capture_render_outputs
        self.sync_usd_for_camera = sync_usd_for_camera
        self.art = art
        self.experiment = experiment
        self.indices = indices
        self.use_fabric = use_fabric
        self.imu_sensor = imu_sensor
        self.receipts = receipts
        self.spawn = spawn
        self.physical_speed_guard = physical_speed_guard
        self.imu_mount = imu_mount
        self.mount = mount
        self.last_imu_time = None
        self.previous_completed_tau = np.zeros(12)

    def step(self, *, iteration, tau, q, qd, contact_state, sensor_rows):
        """执行一步物理并返回测量状态。

        ``iteration`` 是本次采集的步序号（从 0 起）；``tau`` 为本步关节力矩，
        ``q``/``qd`` 为关节位置/速度，``contact_state`` 为接触状态，``sensor_rows``
        为追加写入的 IMU 行。任何断言失败都直接抛异常终止采集，不做降级重试。
        """
        import os

        import omni.physx
        import omni.replicator.core as rep
        from isaacsim.core.simulation_manager import SimulationManager
        from embodied_navigation.robot.unitree.policy.native.observation import (
            quaternion_rotation,
        )

        i = iteration
        art = self.art
        experiment = self.experiment
        indices = self.indices
        use_fabric = self.use_fabric
        imu_sensor = self.imu_sensor
        imu_mount = self.imu_mount
        mount = self.mount
        spawn = self.spawn
        physical_speed_guard = self.physical_speed_guard
        result = self.receipts.result
        save = self.receipts.save
        last_imu_time = self.last_imu_time
        # 力矩写入后立刻回读，前后各校验一次（atol=1e-5），确保物理引擎实际
        # 接受了本步命令，而不是只更新了缓存。
        art.set_dof_efforts(tau[None, :])
        effort_before = art.get_dof_efforts().numpy()[0].copy()
        assert np.allclose(effort_before, tau, atol=1e-5), (
            "Effort readback differs before step"
        )
        time_before = float(SimulationManager.get_simulation_time())
        steps_before = SimulationManager.get_num_physics_steps()
        # Keep one 0.005 s physical step per iteration. Legacy RGB-D renders
        # every 10 steps; tick-controlled RTX/RGB sensors render every 20,
        # matching their real 10 Hz period instead of waiting for nonexistent
        # camera frames at 20 Hz.
        render_stride = 20 if self.sync_usd_for_camera else 10
        render_tick = (i + 1) % render_stride == 0
        if render_tick and not self.sync_usd_for_camera:
            rep.orchestrator.step(
                rt_subframes=1, delta_time=0.005, pause_timeline=False,
                wait_for_render=True,
            )
        else:
            # RTX由原生管理器推进；避免Replicator再次调度异步物理/渲染帧。
            SimulationManager.step(steps=1, update_fabric=use_fabric)
            if not use_fabric or self.sync_usd_for_camera:
                omni.physx.get_physx_simulation_interface().fetch_results()
                omni.physx.get_physx_interface().update_transformations(
                    False, True, True, False
                )
        if render_tick and self.sync_usd_for_camera:
            from isaacsim.core.rendering_manager import RenderingManager
            if self.capture_render_outputs is not None:
                self.capture_render_outputs()
            RenderingManager.render()
            if self.capture_render_outputs is not None:
                self.capture_render_outputs()
            RenderingManager.render()
            if self.capture_render_outputs is not None:
                self.capture_render_outputs()
        actual_time = float(SimulationManager.get_simulation_time())
        effort_after = art.get_dof_efforts().numpy()[0].copy()
        assert np.allclose(effort_after, tau, atol=1e-5), (
            "Effort readback differs after step"
        )
        previous_completed_tau = tau[indices].copy()
        result["last_clock_check"] = {
            "iteration": i,
            "before_s": time_before,
            "after_s": actual_time,
            "step_count_delta": int(
                SimulationManager.get_num_physics_steps() - steps_before
            ),
        }
        save()
        # 时钟契约：一步必须恰好推进 0.005 s 且物理步计数恰好 +1；
        # 1e-6 是允许的浮点误差，超差说明掉步或重复步。
        assert (
            abs(actual_time - time_before - 0.005) < 1e-6
            and SimulationManager.get_num_physics_steps() - steps_before == 1
        ), "Physics skipped or duplicate step"
        p, r = art.get_world_poses()
        v, w = art.get_velocities()
        pos = p.numpy()[0]
        quat = r.numpy()[0]
        result["last_state_before_guard"] = {
            "pos": pos.tolist(),
            "quat": quat.tolist(),
            "time": float(SimulationManager.get_simulation_time()),
            "iteration": i,
        }
        save()
        wq, xq, yq, zq = quat
        roll = np.arctan2(2 * (wq * xq + yq * zq), 1 - 2 * (xq * xq + yq * yq))
        pitch = np.arcsin(np.clip(2 * (wq * yq - zq * xq), -1, 1))
        native_static_diagnostics = _native_static_exploration(experiment)
        # Independent diagnostics from existing readbacks; no new motion gate.
        result["contact_observation"] = {
            "physics_s": actual_time, "iteration": i,
            "counts_body_FR_FL_RR_RL": list(map(int, contact_state)),
            "body_contact_detected": bool(contact_state[0]),
            "body_contact_report_only": bool(experiment.get("body_contact_report_only", False)),
            "startup_exemption": bool(i < 300), "navigation_input": False,
        }
        result["attitude_observation"] = {
            "physics_s": actual_time, "iteration": i,
            "roll_rad": float(roll), "pitch_rad": float(pitch),
            "limit_rad": 0.5,
            "within_guard": bool(max(abs(roll), abs(pitch)) < 0.5),
            "diagnostic_only": native_static_diagnostics,
            "startup_exemption": bool(i < 300), "navigation_input": False,
        }
        result["physical_state_observation"] = {
            "physics_s": actual_time, "iteration": i,
            "position_finite": bool(np.isfinite(pos).all()),
            "quaternion_finite": bool(np.isfinite(quat).all()),
            "linear_velocity_finite": bool(np.isfinite(v.numpy()[0]).all()),
            "angular_velocity_finite": bool(np.isfinite(w.numpy()[0]).all()),
            "joint_position_finite": bool(np.isfinite(q).all()),
            "joint_velocity_finite": bool(np.isfinite(qd).all()),
            "joint_readback_stage": "pre_step",
            "base_readback_stage": "post_step",
            "navigation_input": False,
        }
        # Static native exploration records the prototype attitude envelope;
        # other modes retain its original startup exemption and threshold.
        # Preserve the old threshold's implicit rejection of non-finite poses.
        # Contact handling still follows body_contact_report_only in every mode.
        if native_static_diagnostics:
            _record_configured_bound(
                result, "attitude_threshold", result["attitude_observation"], save
            )
        if i >= 300 and contact_state[0]:
            result["body_contact_steps"] = result.get("body_contact_steps", 0) + 1
        assert i < 300 or (
            np.isfinite(quat).all()
            and np.isfinite([roll, pitch]).all()
            and (
                native_static_diagnostics
                or result["attitude_observation"]["within_guard"]
            )
            and (
                contact_state[0] == 0
                or experiment.get("body_contact_report_only", False)
            )
        ), "Fall/body-contact gate"
        # After startup, the configured base-speed threshold is diagnostic
        # only. Contacts, attitude and state validity remain separate evidence;
        # strict contact handling, finite-state, motor and clock checks remain.
        # Diagnostic only: retain the same already-read physics sample and
        # existing contact counters; never feed these scalars to navigation.
        linear_velocity_readback = v.numpy()[0]
        speed_norm = float(np.linalg.norm(linear_velocity_readback))
        speed_observation = {
            "physics_s": actual_time,
            "iteration": i,
            "linear_speed_norm_mps": speed_norm,
            "horizontal_speed_norm_mps": float(
                np.linalg.norm(linear_velocity_readback[:2])
            ),
            "abs_vertical_speed_mps": float(abs(linear_velocity_readback[2])),
            "limit_mps": float(physical_speed_guard),
            "within_guard": bool(speed_norm < physical_speed_guard),
            "startup_exemption": bool(i < 400),
            "contact_counts_body_FR_FL_RR_RL": list(map(int, contact_state)),
            "navigation_input": False,
            "diagnostic_only": True,
        }
        result["physical_speed_guard_observation"] = speed_observation
        new_speed_peak = speed_norm > result.get("physical_speed_guard_peak", {}).get(
            "linear_speed_norm_mps", -1.0
        )
        if new_speed_peak:
            result["physical_speed_guard_peak"] = dict(speed_observation)
        if i >= 400 and not speed_observation["within_guard"]:
            steps = result.get("physical_speed_guard_out_of_band_steps", 0) + 1
            result["physical_speed_guard_out_of_band_steps"] = steps
            result.setdefault("physical_speed_guard_first_out_of_band", dict(speed_observation))
            result["physical_speed_guard_last_out_of_band"] = dict(speed_observation)
            # Force-save the first event; existing periodic saves retain
            # subsequent peaks without per-step threshold-event disk I/O.
            if steps == 1:
                save(force=True)
        # 位置安全：base 高度下限取 experiment.minimum_base_z_m（默认 0.15 m），
        # 相对出生点的水平位移上限取 experiment.maximum_displacement_m，
        # 缺省回退到环境变量 MAX_DISPLACEMENT（默认 ".25" m）。
        minimum_base_z = float(experiment.get("minimum_base_z_m", 0.15))
        maximum_displacement = float(experiment.get(
            "maximum_displacement_m", os.environ.get("MAX_DISPLACEMENT", ".25")
        ))
        horizontal_displacement = float(np.linalg.norm(pos[:2] - spawn[:2]))
        position_within_guard = bool(
            pos[2] > minimum_base_z and horizontal_displacement < maximum_displacement
        )
        result["position_bound_observation"] = {
            "physics_s": actual_time, "iteration": i,
            "base_z_m": float(pos[2]), "minimum_base_z_m": minimum_base_z,
            "horizontal_displacement_m": horizontal_displacement,
            "maximum_displacement_m": maximum_displacement,
            "within_guard": position_within_guard,
            "diagnostic_only": native_static_diagnostics, "navigation_input": False,
        }
        if native_static_diagnostics:
            _record_configured_bound(
                result, "position_bound", result["position_bound_observation"], save
            )
        assert np.isfinite(pos).all() and (
            native_static_diagnostics or position_within_guard
        ), "Position safety bound"
        reading = imu_sensor.get_data()
        it = float(reading["time"])
        result["last_imu_clock_check"] = {
            "imu_s": it,
            "physics_s": actual_time,
            "previous_imu_s": last_imu_time,
        }
        # IMU 时间戳来自 float32，容差按 float32 在当前时间值上的量化间距
        # 取一半（至少 2e-6 s），再加 1e-7 s 余量；不能用固定容差替代。
        time_tolerance = float32_sensor_time_tolerance(actual_time)
        result["last_imu_clock_check"]["quantization_aware_tolerance_s"] = (
            time_tolerance
        )
        save()
        assert abs(it - actual_time) < time_tolerance, (
            "IMU time differs from physical state beyond float32 quantization"
        )
        # 除连续时间外还要求二者落在同一个 0.005 s tick 上，防止时间接近
        # 但跨步采样。
        assert round(it / 0.005) == round(actual_time / 0.005), (
            "IMU tick differs from physics"
        )
        assert (
            last_imu_time is None
            or round(it / 0.005) - round(last_imu_time / 0.005) == 1
        ), "IMU skipped or repeated sample tick"
        last_imu_time = it
        # IMU 世界速度含杠杆臂项：base 速度 + 角速度 × 质心到 IMU 的力臂；
        # link_coms_local 是收据阶段写入的本地质心，不能省略该项。
        Rb = quaternion_rotation(quat)
        com_local = np.array(result["link_coms_local"][0][0][0])
        sensor_velocity = v.numpy()[0] + np.cross(
            w.numpy()[0], Rb @ (imu_mount - com_local)
        )
        sensor_position = pos + Rb @ imu_mount
        camera_position = pos + Rb @ mount
        sensor_rows.append(
            {
                "time": it,
                "accel_body": np.asarray(reading["linear_acceleration"]).tolist(),
                "gyro_body": np.asarray(reading["angular_velocity"]).tolist(),
                "base_R_world": Rb.tolist(),
                "sensor_velocity_world": sensor_velocity.tolist(),
                "sensor_position_world": sensor_position.tolist(),
            }
        )

        self.last_imu_time = last_imu_time
        self.previous_completed_tau = previous_completed_tau
        return Measurement(
            tau=tau,
            actual_time=actual_time,
            pos=pos,
            quat=quat,
            v=v,
            w=w,
            it=it,
            reading=reading,
            Rb=Rb,
            sensor_position=sensor_position,
            camera_position=camera_position,
            previous_completed_tau=previous_completed_tau,
        )
