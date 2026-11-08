"""Read-only evidence surrounding actual command publication; no control inputs.

Records keep the original command/time assignments and their exact ROS anchor.
Every append uses the existing fail-closed asynchronous EvidenceWriter.
"""
from __future__ import annotations
import sys
import time

SCHEMA = 'teacher_actual_command_publication/v1'


def _flags(node):
    hold = getattr(node, 'control_clock_hold', None)
    return dict(obstacle_hold=bool(getattr(node, 'obstacle_hold', False)),
                alignment_hold=bool(getattr(node, 'alignment_hold', False)),
                tilt_hold=bool(getattr(node, 'tilt_hold', False)),
                tilt_source=getattr(node, 'tilt_source', None),
                bridge_state=getattr(node, 'bridge_safety', {}).get('state'),
                evidence_error=getattr(node.evidence, 'error', None),
                clock_hold_holding=None if hold is None else hold.holding,
                clock_hold_guard_required=None if hold is None else hold.guard_required,
                pid_guard_needs_evaluation=bool(getattr(node, 'pid_guard_needs_evaluation', False)))


class PublicationLedger:
    @staticmethod
    def begin(node, stopped):
        """Snapshot before the original publish method changes any state."""
        entry_wall_ns = time.monotonic_ns()
        row = getattr(node, 'pid_row', None)
        gate = getattr(node, 'heading_gate', None)
        return dict(entry_monotonic_wall_ns=entry_wall_ns, command_before=list(node.command),
                    last_command_time_before=getattr(node, 'last_command_time', None),
                    last_publication_ros_clock_ns_before=getattr(node, 'last_publication_ros_clock_ns', None),
                    associated_pid_sequence=None if row is None else row['sequence'],
                    trigger=sys._getframe(2).f_code.co_name,
                    stopped_argument=bool(stopped),
                    state_before=node.state,
                    heading_phase=None if gate is None else gate.phase,
                    heading_reference=None if gate is None else gate.heading,
                    protection_flags_before=_flags(node))

    @staticmethod
    def publication_started():
        return time.monotonic_ns()

    @staticmethod
    def completed(node, before, publish_ros_clock_ns, started_monotonic_wall_ns):
        """Called only after the existing cmd_pub.publish successfully returns."""
        completed_wall_ns = time.monotonic_ns()
        node.publication_records = getattr(node, 'publication_records', 0) + 1
        row = dict(schema=SCHEMA, sequence=node.publication_records,
                   monotonic_wall_ns=completed_wall_ns,
                   publish_started_monotonic_wall_ns=started_monotonic_wall_ns,
                   publish_ros_clock_ns=publish_ros_clock_ns,
                   command_after=list(node.command),
                   commands_published_count=node.counts['commands'],
                   last_command_time_after=node.last_command_time,
                   state_after=node.state, protection_flags_after=_flags(node),
                   navigation_ground_truth_used=False, **before)
        # Serialization/queue/IO failures propagate through the original writer
        # error and protection/watchdog pathways, never silently skip a record.
        node.evidence.append(node.run/'navigation_command_publications.jsonl', row)
