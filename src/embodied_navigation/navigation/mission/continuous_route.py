"""Fixed original46 route progress and explicit ordered pass evidence.

This changes ordinary region completion to measured passage, never changes a
region volume or height bound. Connector and phase-end ArrivalWindow contracts
remain intact. This module has no ROS, simulator pose or control authority.
"""

import copy
import hashlib
import json
import math
import numpy as np

from embodied_navigation.navigation.mission.goal_regions import (
    contains,
    contains_control,
    parse_goal,
)
from embodied_navigation.navigation.mission.state_machine import Mission
from embodied_navigation.planning.reference.global_route_reference import StableProjectionController
from embodied_navigation.control.tracking.cascade_core import Controller

SCHEMA = "original46_continuous_route/v1"
PASS = "original_region_ordered_pass/v1"
CONNECTORS = {"exploration:11", "return_origin:5", "navigation_f1_f3:7"}


def sha(value):
    """SHA256 over the canonical JSON form of one route/request record."""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def configuration(profile):
    """Read the optional continuous-route contract; enabled requires original46.

    Disabled (absent or ``enabled: false``) leaves the ordinary region path in
    charge. Enabled requires the reviewed bounded guide settings and the
    original46 mission flag, so no other mission can select this route.
    """
    value = profile.get("continuous_route_contract", {"enabled": False})
    if not isinstance(value, dict) or type(value.get("enabled", False)) is not bool:
        raise ValueError("continuous_route_contract.enabled must be boolean")
    result = dict(
        enabled=value.get("enabled", False),
        guide_spacing_m=1.0,
        max_guide_points=128,
        max_projection_advance_m=0.8,
    )
    result.update(value)
    if result["enabled"]:
        if (
            result["guide_spacing_m"] != 1.0
            or result["max_guide_points"] != 128
            or result["max_projection_advance_m"] != 0.8
        ):
            raise ValueError("V34 uses the reviewed bounded original46 settings")
        if profile.get("mission46_required") is not True:
            raise ValueError("continuous route is limited to original46 mission")
    return result


def hard_indices(goals):
    if not goals or any(g.legacy for g in goals):
        raise ValueError("continuous route requires explicit original regions")
    names = {g.goal_id.split(":")[0] for g in goals}
    counts = {"exploration": 18, "return_origin": 14, "navigation_f1_f3": 14}
    if len(names) != 1 or next(iter(names)) not in counts:
        raise ValueError("unknown original route")
    name = next(iter(names))
    if len(goals) != counts[name] or [g.goal_id for g in goals] != [
        f"{name}:{i}" for i in range(counts[name])
    ]:
        raise ValueError("original route identity/order/count required")
    return tuple(i for i, g in enumerate(goals) if g.goal_id in CONNECTORS or i == len(goals) - 1)


class RouteProgress:
    def __init__(self, request_id, goals, goal_hash, settings):
        self.request_id = request_id
        self.goals = tuple(goals)
        self.goal_hash = goal_hash
        self.settings = settings
        self.hard = hard_indices(self.goals)
        self.target_index = self.hard[0]
        self.next_index = 0
        self.points = None
        self.arc = None
        self.route_sha = None
        self.progress = 0.0
        self.last_stamp = None
        self.last_position = None
        self.activated_ns = None
        self.next_activated_ns = None
        self.last_projection = None
        self.receipts = []

    def activate(self, start, stamp_ns, clock_ns):
        if self.points is not None:
            return
        start = np.asarray(start, dtype=float)
        if (
            start.shape != (3,)
            or not np.isfinite(start).all()
            or type(stamp_ns) is not int
            or type(clock_ns) is not int
        ):
            raise ValueError("actual finite measured route activation required")
        self.points = np.vstack([start, [g.center for g in self.goals]])
        lengths = np.linalg.norm(np.diff(self.points, axis=0)[:, :2], axis=1)
        if np.any(lengths <= 1e-6):
            raise ValueError("original route has a vertical/duplicate leg")
        self.arc = np.r_[0.0, np.cumsum(lengths)]
        self.route_sha = sha(self.points.tolist())
        self.activated_ns = self.next_activated_ns = clock_ns
        self.last_stamp = stamp_ns
        self.last_position = start.copy()

    def deadline_exceeded(self, clock_ns):
        return (
            self.next_index < len(self.goals)
            and self.next_activated_ns is not None
            and clock_ns - self.next_activated_ns
            > round(self.goals[self.next_index].timeout_sim_s * 1e9)
        )

    def project(self, pose):
        """Bounded nearest3D projection; uncovered sequence bounds the search.

        The ledger is independent of local SCAN trajectory arc coordinates.
        Full xyz distance, causal arc window and the next uncovered region keep
        overlapping route positions/floors from granting forward passage.
        """
        pose = np.asarray(pose, dtype=float)
        low = max(0.0, self.progress - 0.15)
        high = min(
            self.arc[self.target_index + 1],
            self.progress + 0.8,
            self.arc[min(self.next_index + 1, len(self.arc) - 1)] + 0.25,
        )
        candidates = []
        for i, (a, b) in enumerate(zip(self.points[:-1], self.points[1:])):
            length = self.arc[i + 1] - self.arc[i]
            if self.arc[i + 1] < low or self.arc[i] > high:
                continue
            d = b - a
            u = float(
                np.clip(
                    (pose - a) @ d / (d @ d),
                    max(0.0, (low - self.arc[i]) / length),
                    min(1.0, (high - self.arc[i]) / length),
                )
            )
            point = a + u * d
            s = float(self.arc[i] + u * length)
            candidates.append((float((pose - point) @ (pose - point)), s, i, point))
        if not candidates:
            raise ValueError("no causal original-route projection")
        distance, s, segment, point = min(candidates, key=lambda x: (x[0], x[1], x[2]))
        self.progress = max(self.progress, s)
        self.last_projection = dict(
            arc_m=s,
            monotonic_progress_m=self.progress,
            distance_xyz_m=math.sqrt(distance),
            segment=segment,
            projected_xyz=point.tolist(),
        )
        return self.last_projection

    def observe(self, pose, stamp_ns, clock_ns, protected=False):
        if self.points is None:
            self.activate(pose, stamp_ns, clock_ns)
            return None
        if self.next_index >= len(self.goals) or stamp_ns <= self.last_stamp:
            return None
        old_stamp = self.last_stamp
        self.last_stamp = stamp_ns
        self.last_position = np.asarray(pose, dtype=float).copy()
        if protected or stamp_ns - old_stamp > 200_000_000:
            return None
        projection = self.project(pose)
        index = self.next_index
        goal = self.goals[index]
        if index in self.hard or not contains_control(goal, pose):
            return None
        # A measured point must be near its ordered route vertex in both original
        # 3D volume and route arc. Passing another coincident layer cannot count.
        if abs(projection["arc_m"] - float(self.arc[index + 1])) > 0.5:
            return None
        value = dict(
            schema=PASS,
            completion_semantics="ordered_pass_without_dwell",
            request_id=self.request_id,
            goal_id=goal.goal_id,
            waypoint_index=index,
            goals_definition_sha256=self.goal_hash,
            route_sha256=self.route_sha,
            stamp_ns=stamp_ns,
            start_stamp_ns=stamp_ns,
            dwell_ns=0,
            raw_position=np.asarray(pose, float).tolist(),
            region_inside=bool(contains(goal, pose)),
            control_region_inside=True,
            protected=False,
            reason="passed",
            max_observation_gap_ns=200_000_000,
            previous_observation_stamp_ns=old_stamp,
            arrival_definition=goal.definition()["arrival"],
            control_arrival_definition=goal.control_arrival_definition(),
            original_route_surface_id=goal.route_surface_id,
            route_vertex_arc_m=float(self.arc[index + 1]),
            route_projection=copy.deepcopy(projection),
            navigation_ground_truth_used=False,
            original_dwell_satisfied=False,
        )
        self.receipts.append(value)
        self.next_index += 1
        self.next_activated_ns = clock_ns
        return value

    def accept_hard_receipt(self, value, clock_ns):
        if (
            self.next_index != self.target_index
            or value.get("goal_id") != self.goals[self.next_index].goal_id
        ):
            raise ValueError("hard arrival cannot skip ordered original regions")
        if value.get("reason") != "arrived" or value.get("dwell_ns", -1) < round(
            self.goals[self.next_index].dwell_sim_s * 1e9
        ):
            raise ValueError("original hard arrival dwell required")
        self.receipts.append(copy.deepcopy(value))
        self.next_index += 1
        self.next_activated_ns = clock_ns
        # target_index remains at the connector until independent mission_hold
        # confirms its original terrain-provider acknowledgment.

    def release_hard_target(self):
        if self.next_index > self.target_index and self.next_index < len(self.goals):
            self.target_index = next(i for i in self.hard if i >= self.next_index)
        return self.target_index

    def terminal_heading(self, index):
        d = self.points[index + 1, :2] - self.points[index, :2]
        return math.atan2(d[1], d[0])

    def reference_points(self, end_index=None):
        """Original centers and fixed-leg helpers to next mandatory hard stop."""
        end = self.target_index if end_index is None else int(end_index)
        if not self.next_index <= end <= self.target_index:
            raise ValueError("local guide endpoint outside ordered current span")
        points = []
        for i in range(self.next_index, end + 1):
            a, b = self.points[i], self.points[i + 1]
            length = self.arc[i + 1] - self.arc[i]
            offsets = np.arange(1.0, length - 0.5 + 1e-8, 1.0)
            for offset in offsets:
                if self.arc[i] + offset > self.progress + 0.35:
                    points.append((a + (b - a) * offset / length).tolist())
            points.append(b.tolist())
        if not points or len(points) > 128:
            raise ValueError("bounded continuous guide count exceeded")
        return points

    def missed_pass_requires_replan(self):
        """Never grant skipped coverage; request a checked route through it.

        This is recovery evidence, not a synthetic path-exhaustion receipt. The
        local execution path can be discarded while route progress is retained.
        """
        if (
            self.last_projection is None
            or self.last_position is None
            or self.next_index in self.hard
            or self.next_index >= len(self.goals)
        ):
            return False
        vertex = float(self.arc[self.next_index + 1])
        return self.last_projection["arc_m"] >= vertex + 0.20 and not contains_control(
            self.goals[self.next_index], self.last_position
        )

    def guard_route(self):
        return self.points[: self.target_index + 2].copy()

    def status(self):
        return dict(
            schema=SCHEMA,
            request_id=self.request_id,
            goals_definition_sha256=self.goal_hash,
            route_sha256=self.route_sha,
            route_points_xyz=None if self.points is None else self.points.tolist(),
            route_activation_stamp_ns=self.activated_ns,
            next_activated_ns=self.next_activated_ns if self.next_index < len(self.goals) else None,
            next_timeout_ns=round(self.goals[self.next_index].timeout_sim_s * 1e9)
            if self.next_index < len(self.goals)
            else None,
            next_coverage_index=self.next_index,
            hard_target_index=self.target_index,
            hard_stop_indices=list(self.hard),
            monotonic_progress_m=self.progress,
            measured_projection=copy.deepcopy(self.last_projection),
            ordinary_pass_count=sum(r.get("schema") == PASS for r in self.receipts),
            original_dwell_count=sum(r.get("reason") == "arrived" for r in self.receipts),
            completion_semantics="ordinary_ordered_pass_and_original_hard_stop_dwell",
            original_goal_geometry_unchanged=True,
            navigation_ground_truth_used=False,
        )


def completion_valid(status, raw_goals, request_id, goal_hash, pose):
    """New completion gate; explicitly does not award legacy all-dwell PASS."""
    try:
        goals = tuple(parse_goal(g) for g in raw_goals)
        if (
            status.get("request_id") != request_id
            or status.get("goals_definition_sha256") != goal_hash
            or status.get("goals_definitions") != raw_goals
            or status.get("waypoint_index") != len(goals)
            or status.get("total") != len(goals)
        ):
            return False
        route = status.get("continuous_route") or {}
        points = np.asarray(route.get("route_points_xyz"), dtype=float)
        if (
            route.get("schema") != SCHEMA
            or route.get("request_id") != request_id
            or route.get("goals_definition_sha256") != goal_hash
            or route.get("navigation_ground_truth_used") is not False
            or points.ndim != 2
            or points.shape[1] != 3
            or len(points) < len(goals) + 1
            or not np.isfinite(points).all()
            or sha(points.tolist()) != route.get("route_sha256")
            or not np.array_equal(points[1 : len(goals) + 1], np.asarray([g.center for g in goals]))
        ):
            return False
        # Prefix validation uses the same full frozen route; the prefix ends at
        # the mandatory connector, not a newly declared ordinary hard stop.
        hard = set(route.get("hard_stop_indices", []))
        full_count = len(points) - 1
        expected_hard = {
            i
            for i in range(full_count)
            if f"{goals[0].goal_id.split(':')[0]}:{i}" in CONNECTORS or i == full_count - 1
        }
        if hard != expected_hard or len(goals) - 1 not in hard:
            return False
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0)[:, :2], axis=1))]
        receipts = status.get("region_arrivals")
        if not isinstance(receipts, list) or len(receipts) != len(goals):
            return False
        previous = -1
        previous_arc = 0.0
        for index, (g, r) in enumerate(zip(goals, receipts)):
            if (
                not isinstance(r, dict)
                or r.get("goal_id") != g.goal_id
                or r.get("request_id") != request_id
                or r.get("goals_definition_sha256") != goal_hash
                or r.get("protected") is not False
                or r.get("region_inside") is not True
                or r.get("control_region_inside") is not True
                or r.get("arrival_definition") != g.definition()["arrival"]
                or r.get("control_arrival_definition") != g.control_arrival_definition()
                or r.get("max_observation_gap_ns") != 200_000_000
                or not contains_control(g, r.get("raw_position"))
            ):
                return False
            stamp, start, dwell = (r.get(k) for k in ("stamp_ns", "start_stamp_ns", "dwell_ns"))
            if (
                any(type(v) is not int for v in (stamp, start, dwell))
                or start <= previous
                or stamp < start
                or dwell != stamp - start
            ):
                return False
            if index in hard:
                if (
                    r.get("schema") == PASS
                    or r.get("reason") != "arrived"
                    or dwell < round(g.dwell_sim_s * 1e9)
                ):
                    return False
            else:
                p = r.get("route_projection") or {}
                observed = r.get("previous_observation_stamp_ns")
                if (
                    r.get("schema") != PASS
                    or r.get("completion_semantics") != "ordered_pass_without_dwell"
                    or r.get("reason") != "passed"
                    or dwell != 0
                    or r.get("original_dwell_satisfied") is not False
                    or r.get("route_sha256") != route["route_sha256"]
                    or r.get("waypoint_index") != index
                    or type(observed) is not int
                    or not 0 < stamp - observed <= 200_000_000
                    or r.get("original_route_surface_id") != g.route_surface_id
                    or r.get("navigation_ground_truth_used") is not False
                    or not math.isclose(
                        r.get("route_vertex_arc_m", -1.0), arc[index + 1], abs_tol=1e-8
                    )
                    or abs(p.get("arc_m", -1.0) - arc[index + 1]) > 0.5
                    or p.get("monotonic_progress_m", -1.0) < previous_arc
                ):
                    return False
                previous_arc = p["monotonic_progress_m"]
            previous = stamp
        return pose is not None and contains_control(goals[-1], pose)
    except (TypeError, ValueError, KeyError, IndexError, AttributeError):
        return False


class ContinuousMission(Mission):
    def region_completion_valid(self, status, pose):
        return completion_valid(
            status, self.current_goals, self.current_request, self.current_goals_sha256, pose
        )


class CausalHandoffProjectionController(StableProjectionController):
    """Use existing measured3D local-path handoff without V33's forced reset.

    Local arc changes coordinate system on a replan; the independent fixed-route
    ledger never does. PI, filters, command and nearest3D handoff remain those of
    the existing cascade, and the bounded causal projection remains unchanged.
    """

    def set_path(self, *args, **kwargs):
        return Controller.set_path(self, *args, **kwargs)
