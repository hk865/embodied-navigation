"""Original producer/source gates plus a frame-local ordinary parking adapter.

This module accepts velocity requests, never policy actions or native truth.
It performs no IO and never caches a nonzero command on a rejected input.
"""

from embodied_navigation.common.observation import api as _observer_api

_r118_source_age = _observer_api("r118_source_age_probe")
import hashlib
import json
import math
import numpy as np
from embodied_navigation.common.integration_limits import (
    COMMAND_SOURCE_FRESHNESS_NS,
    COMMAND_SOURCE_FRESHNESS_S,
)
from embodied_navigation.control.parking.parking_zero_contract import (
    authorize_frame,
    before_actor_observation,
)
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


class CommandReader:
    def __init__(self, run_id, scope_sha, limits, profile=None):
        self.minimal_baseline = minimal_baseline_enabled(profile) if profile is not None else False
        self.last_native_clock = None
        self.run_id, self.scope_sha = run_id, scope_sha
        self.limits = np.asarray(limits, float)
        self.last_sequence = 0
        self.last_hash = None
        self.serial = 0
        self.authorization = None
        self.frame = None

    def revoke(self):
        self.authorization = None

    def reject_frame(self, clock, reason):
        self.serial += 1
        self.revoke()
        self.frame = None
        return np.zeros(3), True, dict(read_clock_ns=clock, status="rejected", reason=reason)

    def read(self, raw, clock, wall):
        self.serial += 1
        self.revoke()
        self.frame = None
        record = dict(
            read_clock_ns=clock,
            read_monotonic_wall=wall,
            raw_utf8=None,
            raw_bytes_sha256=None,
            decoded_envelope=None,
        )
        try:
            if self.minimal_baseline:
                if type(clock) is not int or clock < 0 or not math.isfinite(wall):
                    raise ValueError("Actual native clock/wall unavailable")
                if self.last_native_clock is not None and clock <= self.last_native_clock:
                    raise ValueError("Actual native clock repeated or reversed")
                self.last_native_clock = clock
            if not isinstance(raw, bytes):
                raise ValueError("Command bytes unavailable")
            record["raw_bytes_sha256"] = hashlib.sha256(raw).hexdigest()
            record["raw_utf8"] = raw.decode("utf-8")
            data = json.loads(record["raw_utf8"])
            record["decoded_envelope"] = data
            if (
                data.get("schema_version") != 1
                or data.get("mode") != "scan_slam"
                or data.get("source") != "scan_slam"
            ):
                raise ValueError("Unknown command contract")
            seq = data["sequence"]
            digest = hashlib.sha256(canonical(data).encode()).hexdigest()
            if (
                type(seq) is not int
                or seq < 1
                or seq < self.last_sequence
                or seq == self.last_sequence
                and digest != self.last_hash
            ):
                raise ValueError("Command sequence changed or reversed")
            values = np.asarray(data["command"], float)
            if (
                values.shape != (3,)
                or not np.isfinite(values).all()
                or (abs(values) > self.limits + 1e-10).any()
            ):
                raise ValueError("Malformed or out-of-range velocity")
            sim_age = clock / 1e9 - float(data["sim_time"])
            wall_age = wall - float(data["monotonic_wall"])
            if self.minimal_baseline:
                if not (
                    math.isfinite(sim_age)
                    and math.isfinite(wall_age)
                    and -0.05 <= sim_age <= COMMAND_SOURCE_FRESHNESS_S
                    and wall_age >= 0
                ):
                    raise ValueError("Producer simulation age or wall clock invalid")
                record["simulation_wall_age_observation"] = {
                    "producer_wall_age_s": wall_age,
                    "original_wall_limit_s": COMMAND_SOURCE_FRESHNESS_S,
                    "producer_wall_limit_exceeded": wall_age > COMMAND_SOURCE_FRESHNESS_S,
                    "original_timestamps_preserved": True,
                }
            elif not (
                -0.05 <= sim_age <= COMMAND_SOURCE_FRESHNESS_S
                and 0 <= wall_age <= COMMAND_SOURCE_FRESHNESS_S
            ):
                raise ValueError("Producer dual300ms expired")
            lineage = data.get("controller_source")
            marker = data.get("ordinary_terminal_zero_event")
            if np.any(values) or marker is not None:
                if (
                    not isinstance(lineage, dict)
                    or lineage.get("navigation_ground_truth_used") is not False
                ):
                    raise ValueError("No original controller source")
                from embodied_navigation.control.parking.terminal_reference import validate_fields

                validate_fields(lineage, self.run_id)
                pose, gyro, control = (
                    lineage[key]
                    for key in ("source_pose_stamp_ns", "paired_imu_stamp_ns", "control_stamp_ns")
                )
                if not all(type(x) is int for x in (pose, gyro, control)):
                    raise ValueError("Invalid source integer stamps")
                if not (
                    0 <= clock - pose <= COMMAND_SOURCE_FRESHNESS_NS
                    and 0 <= pose - gyro <= 20_000_000
                    and pose <= control <= clock + 50_000_000
                ):
                    raise ValueError("Source age/causality rejected")
                if self.minimal_baseline:
                    source_ages = [
                        wall - lineage[key]
                        for key in (
                            "source_pose_received_monotonic_wall",
                            "paired_imu_received_monotonic_wall",
                        )
                    ]
                    if not all(math.isfinite(age) and age >= 0 for age in source_ages):
                        raise ValueError("Original source wall clock invalid")
                    record["simulation_wall_age_observation"].update(
                        source_pose_wall_age_s=source_ages[0],
                        source_imu_wall_age_s=source_ages[1],
                        source_wall_limit_exceeded=any(
                            age > COMMAND_SOURCE_FRESHNESS_S for age in source_ages
                        ),
                    )
                elif not (
                    0
                    <= wall - lineage["source_pose_received_monotonic_wall"]
                    <= COMMAND_SOURCE_FRESHNESS_S
                    and 0
                    <= wall - lineage["paired_imu_received_monotonic_wall"]
                    <= COMMAND_SOURCE_FRESHNESS_S
                ):
                    raise ValueError("Original source wall age expired")
                if not np.allclose(values, lineage["command_after_slew"], atol=1e-12, rtol=0):
                    raise ValueError("Command lost guarded source binding")
            self.last_sequence, self.last_hash = seq, digest
            if data.get("healthy") is not True:
                record.update(status="rejected", reason="SLAM/SCAN input unhealthy")
                return np.zeros(3), True, record
            if data.get("stop_requested") is True:
                values = np.zeros(3)
            self.frame = dict(
                frame_serial=self.serial,
                input_clock_ns=clock,
                producer_sequence=seq,
                producer_raw_bytes_sha256=record["raw_bytes_sha256"],
            )
            if marker is not None:
                if marker != lineage.get("ordinary_terminal_zero_event"):
                    raise ValueError("Producer zero event lost exact source binding")
                self.authorization = authorize_frame(
                    marker,
                    lineage,
                    self.run_id,
                    values,
                    self.frame,
                    source_max_age_ns=COMMAND_SOURCE_FRESHNESS_NS,
                )
            record.update(
                status="accepted",
                reason=(
                    "Simulation baseline: original wall ages observed; actual source binding retained"
                    if self.minimal_baseline
                    else "Fresh actual SLAM/IMU/SCAN velocity"
                ),
                producer_sim_age_s=sim_age,
                producer_wall_age_s=wall_age,
                ordinary_zero_authorized=self.authorization is not None,
            )
            return values, False, record
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
            self.revoke()
            self.frame = None
            record.update(status="rejected", reason=type(error).__name__ + ": " + str(error))
            return np.zeros(3), True, record

        finally:
            if _r118_source_age.ENABLED:
                _r118_source_age.reader_result(record)

    def before_observation(self, command, request, clock):
        try:
            return before_actor_observation(
                command, request, self.authorization, clock, frame=self.frame
            )
        finally:
            self.revoke()  # Exactly one attempted observation consumes it.
