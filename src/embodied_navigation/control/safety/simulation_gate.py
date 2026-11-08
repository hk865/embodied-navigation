"""Unified RoboGauge simulation safety decisions; no simulator imports.

Preserves the archived thresholds, latching order and grace times. Body contact
is diagnostic here; existing contact-duration protections elsewhere still apply.
This is not a real-hardware safety controller and cannot admit real sensors.
"""
from dataclasses import dataclass
from embodied_navigation.common.state_contract import (
    GAZEBO_NATIVE64_SOURCE, GAZEBO_SIMULATION_CLOCK, PhysicalSafetyState,
    StateContractError, StatePurpose, validate_state_provenance,
)


@dataclass(frozen=True)
class SimulationSafetyResult:
    fault: str | None
    stop: bool
    reason: str | None
    observed_fall_or_low: bool
    observed_body_contact: bool
    persistent_fall: bool
    persistent_fall_observation_end: bool
    fall_started_s: float | None
    state_contract_valid: bool = True
    state_contract_error: str | None = None

    @property
    def request_fault_damping(self):
        return self.fault is not None


class SimulationSafetyGate:
    def __init__(self, *, run_id):
        if not isinstance(run_id, str) or not run_id.strip():
            raise StateContractError('missing_run_id')
        self.run_id = run_id
        self.fault = None
        self.fall_started = None

    def latch_runtime_faults(self, *, policy_finite, actuator_fault_code, prior_fault=None):
        """Original priority, before the potentially failing terrain query.

        Repeating this call in evaluate is idempotent. The two-phase hook keeps
        already latched policy/actuator faults even if the ray provider throws.
        """
        self.fault = self.fault or prior_fault
        if not policy_finite:
            self.fault = self.fault or 'nonfinite_policy'
        if actuator_fault_code > 0:
            self.fault = self.fault or f'actuator_fault_{int(actuator_fault_code)}'
        return self.fault

    def reject_state(self, error, *, prior_fault=None):
        """Fail closed on a new source/clock/measurement contract violation."""
        self.fault = self.fault or prior_fault or 'simulation_safety_state_contract_rejected'
        return SimulationSafetyResult(
            fault=self.fault, stop=True, reason=self.fault,
            observed_fall_or_low=False, observed_body_contact=False,
            persistent_fall=False, persistent_fall_observation_end=False,
            fall_started_s=self.fall_started, state_contract_valid=False,
            state_contract_error=str(error),
        )

    def evaluate(self, state, *, policy_finite=True, prior_fault=None):
        if not isinstance(state, PhysicalSafetyState):
            return self.reject_state('missing_typed_physical_safety_state', prior_fault=prior_fault)
        try:
            validate_state_provenance(state.provenance, purpose=StatePurpose.SIMULATION_SAFETY,
                                      expected_source=GAZEBO_NATIVE64_SOURCE,
                                      expected_clock_domain=GAZEBO_SIMULATION_CLOCK,
                                      expected_run_id=self.run_id)
            state.validate_measurements()
            if state.provenance.frame_stamp_ns != round(state.frame_time_s*1e9):
                raise StateContractError('frame_time_stamp_mismatch')
            if state.provenance.sample_stamp_ns != round((state.frame_time_s-.005)*1e9):
                raise StateContractError('native_preupdate_sample_time_mismatch')
        except (StateContractError, TypeError, ValueError, OverflowError) as error:
            return self.reject_state(error, prior_fault=prior_fault)
        self.latch_runtime_faults(policy_finite=policy_finite,
                                  actuator_fault_code=state.actuator_fault_code,
                                  prior_fault=prior_fault)
        t = state.elapsed_s
        tilt = max(abs(state.roll_rad), abs(state.pitch_rad))
        observed_fall_or_low = bool(t > 1.5 and (tilt > .8 or state.body_clearance_m < .15))
        observed_body_contact = bool(t > 1.5 and state.contacts[0] > 0)
        if observed_fall_or_low:
            self.fault = self.fault or 'navigation_physical_fall_or_low_clearance'
        persistent = bool(t > 2. and (tilt > 1.1 or state.body_clearance_m < .10))
        if not persistent:
            self.fall_started = None
        elif self.fall_started is None:
            self.fall_started = t
        observation_end = bool(self.fall_started is not None and t-self.fall_started >= 4.)
        stop = bool(observation_end or (self.fault is not None and t >= 2))
        return SimulationSafetyResult(
            fault=self.fault, stop=stop, reason=self.fault,
            observed_fall_or_low=observed_fall_or_low,
            observed_body_contact=observed_body_contact, persistent_fall=persistent,
            persistent_fall_observation_end=observation_end, fall_started_s=self.fall_started,
        )
