"""Geometry and safety policy shared by the live ROS controller and its tests.

No simulator pose API is available in this module. Navigation receives SLAM pose,
SCAN's sampled trajectory and a contemporaneous registered LiDAR scan.
"""
from __future__ import annotations

import math
import numpy as np

MAX_TURN_RATE = .12
MAX_WALK_YAW_RATE = .08


def rotation_xyzw(q):
    x, y, z, w = map(float, q)
    n = x*x + y*y + z*z + w*w
    if n < 1e-10:
        raise ValueError("invalid zero quaternion")
    s = 2.0 / n
    return np.array([[1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
                     [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
                     [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]])


class RawImuTilt:
    """Only body tilt safety; raw attitude never alters SLAM pose or heading."""
    def __init__(self, body_imu_rotation, reference_rotation=None, timeout=.25):
        self.body_imu_rotation = np.asarray(body_imu_rotation)
        self.reference_rotation = np.eye(3) if reference_rotation is None else np.asarray(reference_rotation)
        self.timeout = timeout
        self.stamp = -math.inf
        self.updated = -math.inf
        self.tilt = None
        self.max_tilt = 0.
        self.last_rejected = None

    def update(self, quaternion, stamp, wall_now, ros_now, orientation_available=True):
        q = np.asarray(quaternion, dtype=float)
        if not orientation_available or q.shape != (4,) or not np.isfinite(q).all() or np.dot(q,q)<1e-10:
            self.last_rejected = 'invalid IMU orientation'
            return False
        if not math.isfinite(stamp) or stamp <= self.stamp:
            self.last_rejected = 'repeated or out-of-order IMU stamp'
            return False
        if stamp < ros_now-self.timeout or stamp > ros_now+.10:
            self.last_rejected = 'IMU stamp outside current clock window'
            return False
        body_rotation = self.reference_rotation @ rotation_xyzw(q) @ self.body_imu_rotation.T
        self.tilt = body_tilt(body_rotation)
        self.stamp, self.updated = stamp, wall_now
        self.max_tilt = max(self.max_tilt, self.tilt)
        self.last_rejected = None
        return True

    def fresh(self, wall_now, ros_now):
        return (math.isfinite(self.updated) and 0 <= wall_now-self.updated <= self.timeout
                and -.10 <= ros_now-self.stamp <= self.timeout)


def validate_request(data):
    if not isinstance(data, dict):
        raise ValueError("request must be a JSON object")
    request_id = data.get("request_id")
    if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
        raise ValueError("request_id must be a nonempty string of at most 128 characters")
    if data.get("frame_id") != "camera_init":
        raise ValueError("frame_id must be camera_init; transform world coordinates first")
    points = np.asarray(data.get("waypoints"), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 500:
        raise ValueError("waypoints must be between 1 and 500 body-center [x,y,z] positions")
    if not np.isfinite(points).all() or np.abs(points).max() > 1000:
        raise ValueError("waypoints must be finite and within 1000 m")
    return request_id, points


def remove_go2_self_returns(cloud, pose, rotation):
    """Remove only the Go2 occupied body/leg envelope, in measured body frame.

    Body/hips and legs are separate boxes; this is not a radial blindness filter.
    The lower box covers swinging legs, with a small margin for the 10 Hz scan's
    residual motion. Objects outside these robot-volume boxes are never removed.
    """
    local = (cloud-pose) @ rotation
    body = ((np.abs(local[:, 0]) < .30) & (np.abs(local[:, 1]) < .23)
            & (local[:, 2] > -.10) & (local[:, 2] < .20))
    legs = ((np.abs(local[:, 0]) < .45) & (np.abs(local[:, 1]) < .28)
            & (local[:, 2] > -.36) & (local[:, 2] <= -.07))
    keep = ~(body | legs)
    return cloud[keep], int((~keep).sum())


class TrackingPositionFilter:
    """Average measured body sway for route direction, never for arrival checks.

    Go2's gait and the visual/LiDAR update cause short position excursions. Near
    an endpoint those excursions amplify into large lookahead heading changes.
    The filter uses sensor timestamps, so its lag is independent of real-time
    factor. Raw pose remains the source for obstacle, arrival and tilt guards.
    """
    def __init__(self, time_constant=0.4):
        self.time_constant = time_constant
        self.position = None
        self.stamp = None

    def update(self, position, stamp):
        position = np.asarray(position, dtype=float)
        if self.position is None or stamp < self.stamp:
            self.position = position.copy()
        elif stamp > self.stamp:
            alpha = -math.expm1(-(stamp-self.stamp)/self.time_constant)
            self.position += alpha*(position-self.position)
        self.stamp = stamp
        return self.position.copy()


def follow_trajectory(pose, rotation, samples, goal, max_speed=0.12, lookahead=0.80,
                      tracking_pose=None, gate_translation=True, return_steering=False):
    """Position feedback onto SCAN geometry, with heading alignment before walking.

    Returns body planar velocity, yaw rate, and chosen world-space target. Height
    is a planning/arrival constraint, never an XYZ pose command to the simulator.
    """
    tracking_pose = pose if tracking_pose is None else tracking_pose
    distance = np.linalg.norm(samples[:, :2] - tracking_pose[:2], axis=1)
    nearest = int(np.argmin(distance))
    target_index = nearest
    length = 0.0
    while target_index + 1 < len(samples) and length < lookahead:
        length += float(np.linalg.norm(samples[target_index + 1, :2] - samples[target_index, :2]))
        target_index += 1
    target = samples[target_index]
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    projection = samples[nearest, :2]
    tangent = target[:2]-projection
    if np.linalg.norm(tangent) < 1e-6:
        # The lookahead reaches the verified SCAN endpoint. Recover its actual
        # terminal tangent, rather than amplify a tiny body-to-endpoint vector.
        previous = nearest-1
        while previous > 0 and np.linalg.norm(projection-samples[previous,:2]) < .10:
            previous -= 1
        if previous >= 0:
            tangent = projection-samples[previous,:2]
    exhausted = False
    if np.linalg.norm(tangent) < 1e-6:
        direction = np.zeros(2)
        desired = yaw
        exhausted = True
    else:
        unit = tangent/np.linalg.norm(tangent)
        offset = projection-tracking_pose[:2]
        cross_track = offset-unit*float(offset@unit)
        # Keep the correction's geometric scale at the same .8 m used along a
        # long path. Endpoint proximity must not magnify a .1 m lateral sway.
        direction = lookahead*unit+cross_track
        desired = math.atan2(direction[1],direction[0])
        # Never follow a virtual tangent past the collision-checked trajectory
        # when raw measured arrival has not happened. Ask SCAN for a new path.
        exhausted = nearest == len(samples)-1 and float(offset@unit) <= .025
    info = dict(target=target.tolist(), projection=projection.tolist(),
                direction=direction.tolist(), heading=float(desired),
                error=HeadingGate.error(desired,yaw), exhausted=bool(exhausted),
                correction_scale=float(lookahead), nearest_index=nearest,
                target_index=target_index)
    def result(velocity, yaw_rate):
        values=(velocity,yaw_rate,target)
        return (*values,info) if return_steering else values
    if exhausted:
        return result(np.zeros(2),0.0)
    yaw_error = math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))
    yaw_rate = float(np.clip(1.3*yaw_error, -MAX_TURN_RATE, MAX_TURN_RATE))
    if gate_translation and abs(yaw_error) > 0.20:
        return result(np.zeros(2),yaw_rate)
    speed = min(max_speed, 0.70*np.linalg.norm(target[:2]-tracking_pose[:2]),
                0.60*np.linalg.norm(goal[:2]-pose[:2]))
    # CHAMP's tested gait handles forward motion and modest yaw corrections.
    # Lateral velocity combined with turns destabilized the physical Go2. Cross
    # track error is corrected by heading to the measured lookahead point.
    return result(np.array([speed*max(0.,math.cos(yaw_error)),0.]),
                  float(np.clip(yaw_rate,-MAX_WALK_YAW_RATE,MAX_WALK_YAW_RATE)))


def body_tilt(rotation):
    return max(abs(math.atan2(rotation[2,1],rotation[2,2])),
               abs(math.asin(float(np.clip(-rotation[2,0],-1.,1.)))))


def trajectory_has_progress(samples, pose, goal, arrival_radius=.22):
    """Reject SCAN emergency-stop splines while the measured goal is still far.

    A constant parking spline is a valid upstream stop command, but it is not a
    route to an outstanding mission waypoint. Never execute the previous route
    after receiving such a stop, or infer arrival from its elapsed duration.
    """
    if pose is None or np.linalg.norm(goal-pose) < arrival_radius:
        return True
    arc_length = float(np.linalg.norm(np.diff(samples,axis=0),axis=1).sum())
    extent = float(np.linalg.norm(samples-samples[0],axis=1).max())
    return arc_length >= .05 and extent >= .025


class HeadingGate:
    """Latch one heading through a turn, then settle without chasing replans.

    CHAMP resets its gait phase on an all-zero command. Repeated tiny alignment
    corrections caused backward drift despite correctly signed forward commands.
    A turn therefore has exactly one latched target and one uninterrupted stop.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.phase = 'pre_turn'
        self.heading = None
        self.settle_until = None
        self.outside_since = None

    @staticmethod
    def error(target, yaw):
        return math.atan2(math.sin(target-yaw), math.cos(target-yaw))

    def resume_after_stop(self, yaw, fresh_path_heading, stopped_duration):
        """A measured aligned new SCAN path may resume after a full actual stop.

        The caller guarantees a fresh committed trajectory and continuous zero
        commands during obstacle clearance. Real turns retain the original
        pre-turn/align/settle sequence and unchanged angular limits.
        """
        self.reset()
        if stopped_duration >= 1.0 and abs(self.error(fresh_path_heading,yaw)) <= .20:
            self.phase='drive'
            return True
        return False

    def update(self, sim_time, yaw, current_path_heading):
        current_error = self.error(current_path_heading, yaw)
        if self.phase == 'drive':
            # A short gait/SLAM sway must not restart CHAMP before a full stride.
            # Persistent misalignment and large route changes still stop motion.
            if abs(current_error) <= .20:
                self.outside_since = None
            elif self.outside_since is None:
                self.outside_since = sim_time
            if abs(current_error) < .55 and (self.outside_since is None
                                              or sim_time-self.outside_since < .5):
                return True, float(np.clip(.5*current_error,-MAX_WALK_YAW_RATE,MAX_WALK_YAW_RATE))
        if self.phase == 'drive':
            self.reset()
        if self.phase == 'pre_turn':
            if self.settle_until is None:
                self.settle_until = sim_time+1.0
            if sim_time < self.settle_until:
                return False, 0.
            self.phase = 'align'
            self.settle_until = None
        if self.phase == 'settle':
            if sim_time < self.settle_until:
                return False, 0.
            if abs(current_error) <= .20:
                self.phase = 'drive'
                self.heading = None
                return True, float(np.clip(.5*current_error,-MAX_WALK_YAW_RATE,MAX_WALK_YAW_RATE))
            self.reset()
            self.settle_until = sim_time+1.0
            return False, 0.
        if self.heading is None:
            self.heading = current_path_heading
        locked_error = self.error(self.heading,yaw)
        if abs(locked_error) < .10:
            self.phase = 'settle'
            self.settle_until = sim_time+1.0
            return False, 0.
        return False, float(np.clip(1.3*locked_error,-MAX_TURN_RATE,MAX_TURN_RATE))


def limit_acceleration(previous, target, dt, linear_acc=.15, yaw_acc=.25):
    limits = np.array([linear_acc, linear_acc, yaw_acc])*max(0.,min(dt,.10))
    command = np.asarray(previous)+np.clip(np.asarray(target)-np.asarray(previous),-limits,limits)
    # Entering heading alignment must stop translation immediately.
    if np.linalg.norm(target[:2]) < 1e-9:
        command[:2] = 0.
    command[1] = 0.
    return command


def obstacle_ahead(cloud, pose, target, route, *, stop_distance=0.95, half_width=0.42,
                   min_points=3):
    """Find actual LiDAR returns in the swept body corridor.

    The route supplies expected *body-center* elevation along ramps. Returns below
    that elevation minus 0.12 m are supporting ground and excluded. The remaining
    corridor is conservatively 0.84 m wide; no semantic obstacle truth is used.
    """
    if cloud is None or len(cloud) == 0:
        return False, None, 0
    direction = target[:2] - pose[:2]
    norm = np.linalg.norm(direction)
    if norm < 0.02:
        return False, None, 0
    direction = direction / norm
    relative = cloud[:, :2] - pose[:2]
    along = relative @ direction
    lateral = relative[:, 0]*direction[1] - relative[:, 1]*direction[0]
    corridor = (along > 0.30) & (along < stop_distance) & (np.abs(lateral) < half_width)
    points = cloud[corridor]
    if not len(points):
        return False, None, 0
    # Route-segment interpolation preserves the expected ramp elevation instead
    # of removing ground with a fixed global-height slice.
    body_z = np.full(len(points), pose[2])
    best = np.full(len(points), np.inf)
    for a, b in zip(route[:-1], route[1:]):
        segment = b[:2]-a[:2]
        denom = float(segment @ segment)
        t = np.clip(((points[:, :2]-a[:2]) @ segment) / max(denom, 1e-12), 0, 1)
        projection = a[:2] + t[:, None]*segment
        error = np.linalg.norm(points[:, :2]-projection, axis=1)
        use = error < best
        body_z[use] = a[2] + t[use]*(b[2]-a[2])
        best[use] = error[use]
    above = (points[:, 2] > body_z-0.12) & (points[:, 2] < body_z+0.60)
    candidates = points[above]
    if len(candidates) < min_points:
        return False, None, len(candidates)
    clearance = float(np.min((candidates[:, :2]-pose[:2]) @ direction))
    return True, clearance, len(candidates)


def steering_obstacle_ahead(cloud, pose, checked_target, steering_direction, route):
    """Conservative union of checked-path and actual steering corridors.

    Both evaluate identical raw measurements and the same footprint/elevation
    rules. Changing the steering geometry must not rotate the obstacle guard
    away from the actual requested motion, or remove its original protection.
    """
    original=obstacle_ahead(cloud,pose,checked_target,route)
    direction=np.asarray(steering_direction)
    motion_target=np.asarray(pose).copy()
    motion_target[:2] += .8*direction/max(float(np.linalg.norm(direction)),1e-9)
    motion=obstacle_ahead(cloud,pose,motion_target,route)
    if not motion[0]:return original
    return (True,motion[1] if not original[0] else min(original[1],motion[1]),
            max(original[2],motion[2]))
