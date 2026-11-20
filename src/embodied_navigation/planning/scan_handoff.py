"""Per-request SCAN reference handoff, scoped to the owned child and sent odometry."""
from collections import OrderedDict

import numpy as np

CONSUMPTION_ACK_SCHEMA = "r85_scan_child_odometry_consumed/v1"
SCHEMA = CONSUMPTION_ACK_SCHEMA
ODOMETRY_ACK_SCHEMA = CONSUMPTION_ACK_SCHEMA


def reference_waypoints(route, consumed_position):
    """PCT point zero is its old planning origin, not a waypoint to revisit."""
    previous = np.asarray(consumed_position, dtype=float)
    result = []
    for value in route[1:]:
        point = np.asarray(value, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('pct_nonfinite_reference')
        if np.linalg.norm(point - previous) <= 1e-6:
            continue
        result.append(point)
        previous = point
    return result


class ReferenceHandoff:
    """A failed handoff cannot revive; only a new request creates a new instance.

    The deadline covers child startup and the reference response, never route
    execution. Records on ROS topics are external inputs, so every receipt is
    matched to the request, child PID, frame and actual outgoing source stamp.
    """
    def __init__(self, request, pid, nonce, now, timeout_s):
        self.request = request
        self.pid = pid
        self.nonce = nonce
        self.timeout_s = timeout_s
        self.deadline = now + timeout_s
        self.sent_odometry = OrderedDict()
        self.consumed = None
        self.consumed_position = None
        self.reference_stamp = None
        self.reference_odometry_ack = None
        self.commit = None
        self.failure = None
        self.failure_event = None

    def sent_body_pose(self, stamp_ns, position):
        self.sent_odometry[stamp_ns] = tuple(position)
        while len(self.sent_odometry) > 256:
            self.sent_odometry.popitem(last=False)

    def consume_ack(self, info):
        expected = dict(schema=ODOMETRY_ACK_SCHEMA, run_id=self.request['run_id'],
                        planner_epoch=self.request['seq'], planner_pid=self.pid,
                        localization_generation=0, startup_nonce=self.nonce,
                        source_frame_id=self.request['frame_id'], have_odom=True)
        if self.failure or not isinstance(info, dict):
            return False
        if any(type(info.get(k)) is not type(v) or info[k] != v for k, v in expected.items()):
            return False
        stamp = info.get('source_stamp_ns')
        if type(stamp) is not int or stamp not in self.sent_odometry:
            return False
        if self.consumed is not None and stamp < self.consumed['source_stamp_ns']:
            return False
        self.consumed = dict(info)
        self.consumed_position = self.sent_odometry[stamp]
        return True

    def sent_reference(self, stamp, now):
        self.reference_stamp = list(stamp)
        self.reference_odometry_ack = dict(self.consumed)
        self.deadline = now + self.timeout_s

    def matches_reference(self, info):
        return (not self.failure and self.reference_stamp is not None and isinstance(info, dict)
                and info.get('run_id') == self.request['run_id']
                and type(info.get('reference_epoch')) is int
                and info['reference_epoch'] == self.request['seq']
                and type(info.get('planner_process_id')) is int
                and info['planner_process_id'] == self.pid
                and isinstance(info.get('reference_stamp'), list)
                and all(type(part) is int for part in info['reference_stamp'])
                and info['reference_stamp'] == self.reference_stamp)

    def accept_commit(self, info):
        if (self.commit is not None or not self.matches_reference(info) or
                info.get('schema') != 'scan_route_definition/v1'
                or not isinstance(info.get('reference_content'), dict)
                or info['reference_content'].get('frame_id') != self.request['frame_id']
                or type(info.get('committed_reference_revision')) is not int
                or info['committed_reference_revision'] <= 0):
            return False
        self.commit = dict(info)
        return True

    def receive_status(self, info):
        if (not self.matches_reference(info) or info.get('schema') != 'scan_reference_status/v1'
                or info.get('frame_id') != self.request['frame_id']):
            return False
        event, reason = info.get('event'), info.get('reason')
        if event not in ('reference_rejected', 'target_abandoned') or not isinstance(reason, str):
            return False
        # Abandonment is emitted only after SCAN clears its committed target.
        # It may arrive before the definition on the other ROS topic.
        self.fail('native_scan_' + event + ':' + reason, info)
        return True

    def fail(self, reason, event=None):
        if self.failure is None:
            self.failure = reason
            self.failure_event = event

    def check_timeout(self, now):
        if not self.failure and self.commit is None and now >= self.deadline:
            stage = 'reference_response'
            if self.reference_stamp is None:
                stage = 'odometry_ack' if self.consumed is None else 'reference_reader'
            self.fail('native_scan_' + stage + '_timeout')
        return self.failure

    def accepts_candidate(self, info):
        return (self.commit is not None and self.matches_reference(info)
                and info.get('committed_reference_revision') ==
                self.commit['committed_reference_revision'])
