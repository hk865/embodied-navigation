"""Preloaded simulation-truth collection route; never an estimator input."""
import json
import math
from pathlib import Path

from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker


class TruthCollectionRoute:
    """Reuse native safety/stall tracking while preserving per-segment reverse."""
    controller_name = 'truth_collection'

    def __init__(self, experiment, start_xy):
        self.points = experiment['collection_route']
        if not self.points:
            raise ValueError('collection_route must not be empty')
        for row in self.points:
            xyz = row['goal_xyz_m']
            if len(xyz) != 3 or not all(math.isfinite(float(x)) for x in xyz):
                raise ValueError('collection goals must be finite XYZ')
            if type(row.get('reverse', False)) is not bool:
                raise ValueError('reverse must be boolean')
        self.mailbox = Path(experiment['interactive_mailbox'])
        self.mailbox.mkdir(parents=True, exist_ok=True)
        self.speed = experiment['speed_mps']
        self.motion_end = experiment['motion_end_s']
        self.stage = 0
        self.events = []
        self.failure = None
        self.arrived = False
        self.terminal_reason = None
        self.tracker = None
        self.start_xy = start_xy

    @property
    def phase(self):
        return 'terminal' if self.failure or self.arrived or self.terminal_reason else self.tracker.phase if self.tracker else 'idle'

    def invalidate_measurement(self):
        if self.tracker:
            self.tracker.invalidate_measurement()

    def poll_request(self):
        # Only safe stop is accepted here; route changes require a new frozen run.
        try:
            row = json.loads((self.mailbox / 'request.json').read_text())
        except FileNotFoundError:
            return
        if row.get('action') in ('stop', 'cancel'):
            self.terminal_reason = 'collection_operator_stop'

    def tick(self, t, p, yaw, speed, omega, ready=True):
        self.poll_request()
        if self.arrived or self.failure or self.terminal_reason:
            return [0., 0., 0.]
        if self.tracker is None:
            goal = self.points[self.stage]
            if math.dist(p[:2], goal['goal_xyz_m'][:2]) < 1e-6:
                self.failure = 'collection_zero_length_segment'
                return [0., 0., 0.]
            self.tracker = RouteTracker([goal['goal_xyz_m']], self.start_xy,
                speed=self.speed, motion_end=self.motion_end, stop_on_stall=True,
                reverse=goal.get('reverse', False))
        command = self.tracker.tick(t, p, yaw, speed, omega, ready=ready)
        if self.tracker.failure:
            self.failure = self.tracker.failure
            self.events.append(dict(event='collection_failed', stage=self.stage, time=t, reason=self.failure))
            return [0., 0., 0.]
        if self.tracker.arrived:
            self.events.append(dict(event='collection_reached', stage=self.stage, time=t, position=list(p)))
            self.stage += 1
            self.start_xy = list(p[:2])
            self.tracker = None
            self.arrived = self.stage == len(self.points)
            return [0., 0., 0.]
        return command
