"""Associate an actual SCAN payload with its committed current reference.

This contract establishes provenance, not collision freedom or route progress.
Both topics are reliable and volatile; caches are bounded and cleared at every
new reference. Full numeric payload equality prevents pairing a reused id/time
with different control points. No ROS imports are needed for unit tests.
"""
from collections import OrderedDict
import math


def payload(msg):
    return dict(order=int(msg.order), traj_id=int(msg.traj_id),
                start_time=[int(msg.start_time.sec), int(msg.start_time.nanosec)],
                pos_pts=[[float(p.x),float(p.y),float(p.z)] for p in msg.pos_pts],
                knots=list(map(float,msg.knots)))


def key(value):
    return (value['traj_id'], *value['start_time'])


class TrajectoryAssociation:
    def __init__(self, capacity=32):
        self.capacity = capacity
        self.reference_stamp = None
        self.goal = None
        self.splines, self.metadata = OrderedDict(), OrderedDict()
        self.rejected = 0
        self.last_rejected = None
        self.accepted = None
        self.accepted_highwater_id = -1

    def reset(self):
        self.reference_stamp = self.goal = self.accepted = None
        self.accepted_highwater_id = -1
        self.splines.clear()
        self.metadata.clear()

    def request(self, stamp, goal):
        self.reset()
        self.reference_stamp = list(map(int,stamp))
        self.goal = list(map(float,goal))

    def reject(self, reason):
        self.rejected += 1
        self.last_rejected = reason

    def put(self, cache, identity, value):
        cache[identity] = value
        cache.move_to_end(identity)
        while len(cache) > self.capacity:
            cache.popitem(last=False)

    def add_spline(self, msg):
        if self.reference_stamp is None:
            self.reject('current waypoint has not requested a reference')
            return None
        value = payload(msg)
        identity = key(value)
        self.put(self.splines, identity, (msg,value))
        return self.match(identity)

    def add_metadata(self, value):
        if not isinstance(value,dict):
            raise ValueError('trajectory metadata must be an object')
        if self.reference_stamp is None:
            self.reject('current waypoint has not requested a reference')
            return None
        if value.get('schema') != 1 or value.get('reference_stamp') != self.reference_stamp:
            self.reject('trajectory belongs to a different reference')
            return None
        goal = value.get('body_goal', [])
        if len(goal) != 3 or any(not math.isfinite(float(x)) for x in goal) or any(
                abs(float(x)-y) > 1e-8 for x,y in zip(goal,self.goal)):
            self.reject('trajectory reference goal differs from current body goal')
            return None
        trajectory = value['trajectory']
        identity = key(trajectory)
        self.put(self.metadata, identity, value)
        return self.match(identity)

    def match(self, identity):
        if identity not in self.splines or identity not in self.metadata:
            return None
        msg,value = self.splines.pop(identity)
        meta = self.metadata.pop(identity)
        if value != meta['trajectory']:
            self.reject('metadata and B-spline payload differ')
            return None
        # SCAN increments id for every newly generated local trajectory. Its
        # mutable freeze start_time cannot establish chronological order.
        if identity[0] <= self.accepted_highwater_id:
            self.reject('trajectory id does not advance current reference')
            return None
        self.accepted = value
        self.accepted_highwater_id = identity[0]
        self.last_rejected = None
        return msg,meta
