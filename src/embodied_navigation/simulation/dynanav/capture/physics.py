"""One 0.005 s physics step plus its clock, effort and safety assertions.

This module owns the per-step actuation sequence: effort write/readback checks,
the single physics step (Fabric or USD update), simulation-clock and IMU-clock
assertions, attitude/contact/speed/position safety gates, and the IMU/base
measurement returned to the frame and control stages. Every threshold and probe
default is unchanged.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Measurement:
    """Post-step measurements shared by the control, frame and receipt stages."""

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
    """Own one 0.005 s step and the persistent last-IMU-sample state."""

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
    ):
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
        """Execute one physics step and return the measured state."""
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
        art.set_dof_efforts(tau[None, :])
        effort_before = art.get_dof_efforts().numpy()[0].copy()
        assert np.allclose(effort_before, tau, atol=1e-5), (
            "Effort readback differs before step"
        )
        time_before = float(SimulationManager.get_simulation_time())
        steps_before = SimulationManager.get_num_physics_steps()
        if (i + 1) % 10:
            SimulationManager.step(steps=1, update_fabric=use_fabric)
            if not use_fabric:
                omni.physx.get_physx_simulation_interface().fetch_results()
                omni.physx.get_physx_interface().update_transformations(
                    False, True, True, False
                )
        else:
            rep.orchestrator.step(
                rt_subframes=1, delta_time=0.005, pause_timeline=False
            )
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
        if i >= 300 and contact_state[0]:
            result["body_contact_steps"] = result.get("body_contact_steps", 0) + 1
        assert i < 300 or (
            max(abs(roll), abs(pitch)) < 0.5
            and (
                contact_state[0] == 0
                or experiment.get("body_contact_report_only", False)
            )
        ), "Fall/body-contact gate"
        assert i < 400 or np.linalg.norm(v.numpy()[0]) < physical_speed_guard, (
            "Configured experiment physical overspeed bound"
        )
        assert pos[2] > float(
            experiment.get("minimum_base_z_m", 0.15)
        ) and np.linalg.norm(pos[:2] - spawn[:2]) < float(
            experiment.get(
                "maximum_displacement_m", os.environ.get("MAX_DISPLACEMENT", ".25")
            )
        ), "Position safety bound"
        reading = imu_sensor.get_data()
        it = float(reading["time"])
        result["last_imu_clock_check"] = {
            "imu_s": it,
            "physics_s": actual_time,
            "previous_imu_s": last_imu_time,
        }
        time_tolerance = (
            max(2e-6, 0.5 * float(abs(np.spacing(np.float32(actual_time))))) + 1e-7
        )
        result["last_imu_clock_check"]["quantization_aware_tolerance_s"] = (
            time_tolerance
        )
        save()
        assert abs(it - actual_time) < time_tolerance, (
            "IMU time differs from physical state beyond float32 quantization"
        )
        assert round(it / 0.005) == round(actual_time / 0.005), (
            "IMU tick differs from physics"
        )
        assert (
            last_imu_time is None
            or round(it / 0.005) - round(last_imu_time / 0.005) == 1
        ), "IMU skipped or repeated sample tick"
        last_imu_time = it
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
