"""Gazebo native64 -> explicitly privileged simulation safety facts.

No decision thresholds and no navigation/SLAM publication live in this adapter.
The transport reads the previous physics step at PreUpdate: sample=frame-.005s.
"""

import math
import numpy as np
from embodied_navigation.common.state_contract import (
    GAZEBO_NATIVE64_SOURCE,
    GAZEBO_SIMULATION_CLOCK,
    PhysicalSafetyState,
    StateContractError,
    StateOrigin,
    StateProvenance,
    StatePurpose,
    validate_state_provenance,
)

PHYSICS_STEP_S = 0.005


def native_rpy(quaternion_wxyz):
    """Exact original native-loop formula, including its asin clipping."""
    w, x, y, z = quaternion_wxyz
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(np.clip(2 * (w * y - z * x), -1, 1)),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def from_gazebo_native64(native_state, *, first_world_time_s, ground_height_m, run_id):
    """Copy actual state plus the actual downward safety-ray result.

    Caller supplies the first frame of this run, never an unrelated wall clock.
    The state must be from the Gazebo native request64 peer. A real sensor cannot
    use this factory to acquire simulated-state identity.
    """
    state = np.asarray(native_state, dtype=np.float64)
    if state.shape != (64,) or not np.isfinite(state).all():
        raise StateContractError("invalid_gazebo_native64")
    if not math.isfinite(first_world_time_s) or not math.isfinite(ground_height_m):
        raise StateContractError("invalid_simulation_start_or_ground_height")
    frame_time = float(state[0])
    angles = native_rpy(state[4:8])
    provenance = StateProvenance(
        origin=StateOrigin.SIMULATION_TRUTH,
        source=GAZEBO_NATIVE64_SOURCE,
        clock_domain=GAZEBO_SIMULATION_CLOCK,
        run_id=run_id,
        sample_stamp_ns=round(float(state[0] - PHYSICS_STEP_S) * 1e9),
        frame_stamp_ns=round(frame_time * 1e9),
    )
    validate_state_provenance(
        provenance,
        purpose=StatePurpose.SIMULATION_SAFETY,
        expected_source=GAZEBO_NATIVE64_SOURCE,
        expected_clock_domain=GAZEBO_SIMULATION_CLOCK,
        expected_run_id=run_id,
    )
    result = PhysicalSafetyState(
        provenance=provenance,
        frame_time_s=frame_time,
        elapsed_s=float(state[0] - first_world_time_s),
        roll_rad=angles[0],
        pitch_rad=angles[1],
        body_clearance_m=float(state[3] - ground_height_m),
        contacts=tuple(float(value) for value in state[50:55]),
        actuator_fault_code=float(state[56]),
    )
    result.validate_measurements()
    return result
