"""V33 planning guides and fail-closed direction coherence on checked SCAN curves.

Guides are a registered mission-leg prior, not a collision/support certificate.
They never authorize movement or arrival. Actual SCAN samples, native guards and
original region/deadline/stop contracts remain the execution authority.
"""

import copy
import math
import numpy as np
from embodied_navigation.control.tracking.cascade_core import Controller
from embodied_navigation.planning.reference.spatial_reference import point_at


def contract(profile):
    """Resolve and bound the optional global route-reference settings from the profile."""
    value = profile.get("global_route_reference_contract", {"enabled": False})
    if not isinstance(value, dict) or type(value.get("enabled", False)) is not bool:
        raise ValueError("global_route_reference_contract.enabled must be boolean")
    result = dict(
        enabled=value.get("enabled", False),
        guide_spacing_m=1.0,
        guide_start_ahead_m=0.35,
        direction_audit_arc_m=0.8,
        max_guide_points=64,
    )
    result.update(value)
    if result["enabled"]:
        for name in ("guide_spacing_m", "guide_start_ahead_m", "direction_audit_arc_m"):
            number = result[name]
            if (
                isinstance(number, bool)
                or not isinstance(number, (int, float))
                or not math.isfinite(number)
                or number <= 0
            ):
                raise ValueError("Invalid global route reference " + name)
        if (
            not 0.2 <= result["guide_spacing_m"] <= 2.0
            or not 0.2 <= result["direction_audit_arc_m"] <= 1.2
        ):
            raise ValueError("Global reference geometry outside bounded experiment range")
        if (
            type(result["max_guide_points"]) is not int
            or not 2 <= result["max_guide_points"] <= 128
        ):
            raise ValueError("Invalid bounded guide point count")
    return result


def vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("Global reference requires finite camera_init xyz")
    return result


class SegmentGuides:
    """Freeze one measured activation-to-original-goal line per goal identity.

    Progress selects which planning-only guides to submit; it never updates the
    measured arrival window. Guide locations remain fixed rather than being
    redrawn through each instantaneous localization position.
    """

    def __init__(self):
        self.identity = None
        self.start = None
        self.goal = None
        self.progress = 0.0

    def remaining(self, identity, start, goal, pose, settings):
        start, goal, pose = vector(start), vector(goal), vector(pose)
        identity = tuple(identity)
        if self.identity != identity:
            self.identity = identity
            self.start = start.copy()
            self.goal = goal.copy()
            self.progress = 0.0
        elif not np.array_equal(start, self.start) or not np.array_equal(goal, self.goal):
            raise ValueError("Frozen mission leg changed under same reference identity")
        delta = self.goal - self.start
        length = float(np.linalg.norm(delta[:2]))
        terminal_coalesced = 0
        if length <= 1e-6:
            points = [self.goal.tolist()]
            projection = 0.0
        else:
            projection = float(
                np.clip((pose[:2] - self.start[:2]) @ delta[:2] / length, 0.0, length)
            )
            self.progress = max(self.progress, projection)
            spacing = float(settings["guide_spacing_m"])
            first = (
                math.floor((self.progress + settings["guide_start_ahead_m"]) / spacing) + 1
            ) * spacing
            arcs = np.arange(first, length - 1e-8, spacing)
            guides = [self.start + delta * (float(s) / length) for s in arcs]
            # A helper very near the mandatory original goal creates a short
            # minSnap time-allocation segment. Remove only planning helpers;
            # keep every remaining grid location and the exact original goal.
            while guides and float(np.linalg.norm(guides[-1] - self.goal)) < 0.5 * spacing:
                guides.pop()
                terminal_coalesced += 1
            if len(guides) + 1 > settings["max_guide_points"]:
                raise ValueError("Planning guide count exceeds bounded contract")
            points = [point.tolist() for point in guides] + [self.goal.tolist()]
        return points, dict(
            schema="mission_segment_planning_guides/v1",
            identity=list(identity),
            frame_id="camera_init",
            start_xyz=self.start.tolist(),
            original_goal_xyz=self.goal.tolist(),
            measured_projection_m=projection,
            planning_progress_m=self.progress,
            guide_count=len(points),
            guide_points_xyz=copy.deepcopy(points),
            guide_spacing_m=settings["guide_spacing_m"],
            terminal_helper_min_distance_m=0.5 * settings["guide_spacing_m"],
            terminal_helpers_coalesced=terminal_coalesced,
            guide_source="fixed original goal and measured SLAM segment activation",
            guide_only_not_arrival=True,
            collision_or_support_certified=False,
            navigation_ground_truth_used=False,
        )


class StableProjectionController(Controller):
    """Use the same checked-curve projection for preview and actual control.

    The lower arc bound cannot move behind already measured progress. A new
    trajectory identity starts a new projection ledger; no previous trajectory
    progress or arrival is transferred. No curve points or gains are replaced.
    """

    def set_path(self, *args, **kwargs):
        changed = super().set_path(*args, **kwargs)
        if changed:
            self.progress = 0.0
            self.new_path = True
            if self.path_handoff is not None:
                self.path_handoff = dict(
                    self.path_handoff, projection_progress_reset_for_new_identity=True
                )
        return changed

    def _project(self, position):
        low = 0.0 if self.new_path else self.progress
        high = (
            self.cumulative[-1] if self.new_path else min(self.cumulative[-1], self.progress + 0.8)
        )
        best = None
        for i, (a, d, length) in enumerate(zip(self.path[:-1], self.path_delta, self.lengths)):
            if self.cumulative[i + 1] < low or self.cumulative[i] > high:
                continue
            u0 = max(0.0, (low - self.cumulative[i]) / length)
            u1 = min(1.0, (high - self.cumulative[i]) / length)
            u = float(np.clip((position - a) @ d / (d @ d), u0, u1))
            point = a + u * d
            along = float(self.cumulative[i] + u * length)
            distance2 = float((position - point) @ (position - point))
            candidate = (distance2, along, i, point)
            if best is None or candidate[:3] < best[:3]:
                best = candidate
        if best is None:
            raise ValueError("No causal checked-curve projection remains")
        _, nearest_s, i, point = best
        self.progress = max(self.progress, nearest_s)
        self.new_path = False
        tangent = self.path_delta[i, :2] / self.lengths[i]
        normal = np.array([-tangent[1], tangent[0]])
        cross = float((position[:2] - point[:2]) @ normal)
        self.last_projection_position = np.asarray(position, float).copy()
        self.last_projection_tangent = tangent.copy()
        return (
            i,
            point,
            tangent,
            normal,
            cross,
            float(self.path_delta[i, 2] / self.lengths[i]),
            nearest_s,
        )


def direction_check(cascade, reference, max_error_rad, settings):
    """Audit two headings of an existing checked subarc; never execute its chord."""
    if not math.isfinite(max_error_rad) or not 0 < max_error_rad <= 0.2:
        raise ValueError("Direction check must use the original drive heading limit")
    begin = float(reference["nearest_s_m"])
    end = min(float(cascade.cumulative[-1]), begin + settings["direction_audit_arc_m"])
    p0 = point_at(cascade.path, cascade.cumulative, begin)
    p1 = point_at(cascade.path, cascade.cumulative, end)
    chord = p1[:2] - p0[:2]
    length = float(np.linalg.norm(chord))
    tail = reference["heading_source"] == "fixed_goal_tail"
    far_heading = math.atan2(chord[1], chord[0]) if length > 1e-6 else None
    error = (
        None
        if far_heading is None
        else math.atan2(
            math.sin(reference["heading_rad"] - far_heading),
            math.cos(reference["heading_rad"] - far_heading),
        )
    )
    allowed = tail or error is not None and abs(error) <= max_error_rad
    return dict(
        schema="checked_scan_direction_coherence/v1",
        allowed=bool(allowed),
        reason="fixed_goal_tail_preserved"
        if tail
        else "checked_subarc_directions_coherent"
        if allowed
        else "checked_subarc_direction_incoherent_requires_replan",
        near_heading_rad=float(reference["heading_rad"]),
        audit_heading_rad=far_heading,
        difference_rad=error,
        maximum_difference_rad=max_error_rad,
        start_s_m=begin,
        end_s_m=end,
        path_id=cascade.path_id,
        path_sha256=cascade.path_sha,
        audit_chord_is_execution_path=False,
        original_path_replaced=False,
        collision_or_support_certified=False,
        navigation_ground_truth_used=False,
    )


def admission_direction_check(
    samples, pose, goal, goal_yaw, cascade_config, max_error_rad, settings
):
    points = np.asarray(samples, dtype=float)
    indices = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i, :2] - points[indices[-1], :2]) >= 1e-5:
            indices.append(i)
    cascade = StableProjectionController(
        cascade_config, vector(goal).tolist(), float(goal_yaw), "direction_admission"
    )
    cascade.set_path(points[indices].tolist(), "pending_checked_scan", 0, 0, position=vector(pose))
    return direction_check(
        cascade, cascade.preview_reference(vector(pose)), max_error_rad, settings
    )
