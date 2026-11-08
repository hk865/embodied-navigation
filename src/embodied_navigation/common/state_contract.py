"""Typed source/time provenance shared by state producers and consumers.

Provenance is an in-process contract, not cryptographic authentication. Producers
must copy their real measurement metadata, never relabel truth as localization.
There is no real-hardware backend here; REAL_SENSOR only names a future source.
"""
from dataclasses import dataclass
from enum import Enum
import math
from numbers import Integral


class StateContractError(ValueError):
    """A state cannot be admitted for the requested purpose."""


class StateOrigin(str, Enum):
    SIMULATION_TRUTH = 'simulation_truth'
    SLAM_ESTIMATE = 'slam_estimate'
    REAL_SENSOR = 'real_sensor'  # Reserved contract, not an implemented backend.


class StatePurpose(str, Enum):
    SIMULATION_SAFETY = 'simulation_safety'
    EVALUATION = 'evaluation'
    NAVIGATION = 'navigation'


GAZEBO_NATIVE64_SOURCE = 'gazebo.native64.preupdate'
GAZEBO_SIMULATION_CLOCK = 'gazebo_simulation'


@dataclass(frozen=True)
class StateProvenance:
    origin: StateOrigin
    source: str
    clock_domain: str
    run_id: str
    sample_stamp_ns: int
    frame_stamp_ns: int

    def as_dict(self):
        return dict(origin=self.origin.value if isinstance(self.origin, StateOrigin) else self.origin,
                    source=self.source, clock_domain=self.clock_domain, run_id=self.run_id,
                    sample_stamp_ns=self.sample_stamp_ns, frame_stamp_ns=self.frame_stamp_ns)


def validate_state_provenance(provenance, *, purpose, expected_source,
                              expected_clock_domain, expected_run_id):
    """Reject missing identity, cross-run/time-domain input and truth navigation.

    Consumers must name an exact expected producer, clock and run. A source label
    alone never grants navigation authority; only SLAM_ESTIMATE is eligible.
    Timestamp freshness/clock progression belongs to each consumer's clock rules.
    """
    if not isinstance(provenance, StateProvenance):
        raise StateContractError('missing_typed_state_provenance')
    try:
        purpose = StatePurpose(purpose)
        origin = StateOrigin(provenance.origin)
    except (ValueError, TypeError) as error:
        raise StateContractError('unknown_state_origin_or_purpose') from error
    for name, actual, expected in (
        ('source', provenance.source, expected_source),
        ('clock_domain', provenance.clock_domain, expected_clock_domain),
        ('run_id', provenance.run_id, expected_run_id),
    ):
        if not isinstance(actual, str) or not actual.strip() or not isinstance(expected, str) or not expected.strip():
            raise StateContractError('missing_' + name)
        if actual != expected:
            raise StateContractError('mismatched_' + name)
    for name in ('sample_stamp_ns', 'frame_stamp_ns'):
        stamp = getattr(provenance, name)
        if isinstance(stamp, bool) or not isinstance(stamp, Integral):
            raise StateContractError('invalid_' + name)
    if provenance.sample_stamp_ns > provenance.frame_stamp_ns:
        raise StateContractError('measurement_after_frame')
    if provenance.source == GAZEBO_NATIVE64_SOURCE and origin != StateOrigin.SIMULATION_TRUTH:
        raise StateContractError('gazebo_native64_origin_cannot_be_relabelled')
    if purpose == StatePurpose.NAVIGATION and origin != StateOrigin.SLAM_ESTIMATE:
        raise StateContractError('navigation_requires_slam_estimate')
    if purpose == StatePurpose.SIMULATION_SAFETY and origin != StateOrigin.SIMULATION_TRUTH:
        raise StateContractError('simulation_safety_requires_simulation_truth')


@dataclass(frozen=True)
class PhysicalSafetyState:
    """Physical facts only, not a pose provider or navigation observation.

    elapsed_s is relative to the producer's run start in its declared clock.
    Contacts retain the native body/FR/FL/RR/RL order. No stop policy lives here.
    """
    provenance: StateProvenance
    frame_time_s: float
    elapsed_s: float
    roll_rad: float
    pitch_rad: float
    body_clearance_m: float
    contacts: tuple[float, float, float, float, float]
    actuator_fault_code: float

    def validate_measurements(self):
        values = (self.frame_time_s, self.elapsed_s, self.roll_rad, self.pitch_rad,
                  self.body_clearance_m, self.actuator_fault_code)
        try:
            valid = (all(math.isfinite(value) for value in values)
                     and len(self.contacts) == 5
                     and all(math.isfinite(value) for value in self.contacts))
        except (TypeError, ValueError):
            valid = False
        if not valid:
            raise StateContractError('invalid_physical_safety_measurements')
