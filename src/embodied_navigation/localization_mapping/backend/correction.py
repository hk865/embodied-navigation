"""Geometry proposals are not navigation authority.

Temporal admission lives on the backend; the existing localization_runtime is
still the only publisher applying nav_world @ C_world_odom to pose AND cloud.
"""
from __future__ import annotations
import json
import time
from pathlib import Path
import numpy as np
from .geometry import checked_transform, inverse, pose_distance, transform_points


class TemporalAdmission:
    def __init__(self, confirmations=2, minimum_dt_ns=150_000_000,
                 maximum_age_ns=3_000_000_000, translation_tolerance=.15,
                 rotation_tolerance=.08):
        if confirmations < 1 or minimum_dt_ns <= 0 or maximum_age_ns < minimum_dt_ns:
            raise ValueError('invalid temporal admission limits')
        if translation_tolerance <= 0 or rotation_tolerance <= 0:
            raise ValueError('invalid correction consistency limits')
        self.confirmations = confirmations; self.minimum_dt_ns = minimum_dt_ns
        self.maximum_age_ns = maximum_age_ns
        self.translation_tolerance = translation_tolerance; self.rotation_tolerance = rotation_tolerance
        self.reset()

    def reset(self):
        self.epoch = None; self.previous = None; self.count = 0; self.last_accepted_ns = 0

    def consider(self, keyframe, registration, latest_stamp_ns, T_world_map=None):
        metric = lambda value: float(value) if np.isfinite(value) else None
        result = dict(schema='livo_backend_correction/v1', accepted=False, epoch=keyframe.epoch,
                      sequence=keyframe.sequence, source_ns=keyframe.stamp_ns,
                      T_odom_body_source_ns=keyframe.stamp_ns,
                      T_odom_body=keyframe.T_odom_body.tolist(), reason=registration.reason,
                      registration_elapsed_s=registration.elapsed_s,
                      geometry=dict(fitness=metric(registration.fitness), rmse=metric(registration.rmse),
                                    observability=metric(registration.observability)))
        if keyframe.epoch != self.epoch:
            self.reset(); self.epoch = keyframe.epoch
        if not registration.accepted or registration.T_map_lidar is None:
            self.previous = None; self.count = 0; return result
        if not 0 <= latest_stamp_ns-keyframe.stamp_ns <= self.maximum_age_ns:
            result['reason'] = 'stale_backend_result'; return result
        if keyframe.stamp_ns <= self.last_accepted_ns:
            result['reason'] = 'out_of_order_result'; return result
        world_map = np.eye(4) if T_world_map is None else checked_transform(T_world_map)
        matched_body = world_map @ checked_transform(registration.T_map_lidar) @ inverse(keyframe.T_body_lidar)
        correction = matched_body @ inverse(keyframe.T_odom_body)
        if self.previous is None:
            self.count = 1
        else:
            stamp, previous = self.previous
            distance, angle = pose_distance(correction, previous)
            if keyframe.stamp_ns-stamp < self.minimum_dt_ns:
                result['reason'] = 'confirmation_too_close'; return result
            if keyframe.stamp_ns-stamp > self.maximum_age_ns or distance > self.translation_tolerance or angle > self.rotation_tolerance:
                self.count = 1
            else: self.count += 1
        self.previous = (keyframe.stamp_ns, correction)
        result.update(C_world_odom=correction.tolist(), T_world_body=matched_body.tolist(),
                      independent_confirmations=self.count)
        if self.count < self.confirmations:
            result['reason'] = 'awaiting_temporal_confirmation'; return result
        self.last_accepted_ns = keyframe.stamp_ns
        result.update(accepted=True, reason='geometry_and_temporal_verified')
        return result


def apply_bundle(correction, pose_odom, cloud_odom, T_navigation_world=None):
    """One captured correction for both outputs, never two independently polled C's."""
    c = checked_transform(correction)
    if T_navigation_world is not None: c = checked_transform(T_navigation_world) @ c
    return c @ checked_transform(pose_odom), transform_points(cloud_odom, c)


class BackendResultReader:
    """Read opt-in backend proposals, not a second corrected odometry stream.

    Results created before this reader started are deliberately ignored. Epoch
    changes require restarting the localization owner, just as frontend reset
    requires rebuilding its raw pose/cloud association.
    """
    def __init__(self, path, maximum_age_ns=3_000_000_000):
        self.path = Path(path); self.maximum_age_ns = maximum_age_ns
        self.started_wall_ns = time.time_ns(); self.last_file_ns = 0
        self.epoch = None; self.last_sequence = -1

    def poll(self, clock_ns, parent_generation):
        try:
            info = self.path.stat()
            if info.st_mtime_ns <= max(self.started_wall_ns, self.last_file_ns): return None
            self.last_file_ns = info.st_mtime_ns
            result = json.loads(self.path.read_text())
        except (OSError, ValueError): return None
        if result.get('schema') != 'livo_backend_correction/v1': return None
        if result.get('accepted') is not True: return result
        source = result.get('source_ns'); epoch = result.get('epoch'); sequence = result.get('sequence')
        if any(type(v) is not int for v in (source, epoch, sequence)): return None
        if not -50_000_000 <= clock_ns-source <= self.maximum_age_ns: return None
        wall = result.get('published_wall_ns')
        if type(wall) is not int or not 0 <= time.time_ns()-wall <= self.maximum_age_ns: return None
        if self.epoch is None: self.epoch = epoch
        if epoch != self.epoch or sequence <= self.last_sequence: return None
        try:
            c = checked_transform(result['C_world_odom']); body = checked_transform(result['T_odom_body'])
            matched = checked_transform(result['T_world_body'])
        except (KeyError, ValueError): return None
        if result.get('T_odom_body_source_ns') != source or not np.allclose(c @ body, matched, rtol=0, atol=1e-7): return None
        self.last_sequence = sequence
        result['parent_generation'] = parent_generation
        return result
