"""Bridge veto reasons in original evaluation order; authority is unchanged.

Predicates and their state changes stay in CommandGate.snapshot. This catalogue
adds stable codes to the existing reason strings. It has no bypass mode.
"""
from dataclasses import dataclass

@dataclass(frozen=True)
class GateSpec:
    code: str
    effect: str = "original_bridge_zero_command"
    predicate_owner: str = "CommandGate.snapshot"

GATE_SPECS = tuple(GateSpec(code) for code in ('latched_failure', 'cascade_provenance', 'champ_feedback_freshness', 'champ_failed', 'sensor_anchor_publisher_chain', 'sensor_warmup', 'immutable_slam_anchor', 'route_fence', 'publisher_graph_freshness', 'slam_publisher_identity', 'command_publisher_identity', 'slam_pose_validity', 'two_valid_slam_poses', 'command_validity', 'dual_source_age'))
GATES = {spec.code: spec for spec in GATE_SPECS}

class GateReasons(list):
    """Keep the original list truth value, order, duplicates and strings."""
    def __init__(self):
        super().__init__()
        self.events = []

    def reject(self, code, reason):
        spec = GATES[code]
        self.append(reason)
        self.events.append(dict(code=code, reason=reason, effect=spec.effect))
