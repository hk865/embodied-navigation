"""Route execution and planning adapters for ``PIDNavigation``.

Covers follow/continuous region progress, planner goal/request/reference points,
segment activation and the final bundle reference.
"""

from .pid_support import (
    math,
    observe_business_gate,
    early_handoff_request,
    _final_path_receipt_snapshot,
    pending_path_waiting,
    np,
    copy,
    direction_check,
    follow_trajectory,
    _r96_cpu,
    ArrivalWindow,
)


class RouteMixin:
    """Checked-trajectory execution, continuous progress, planning and bundle references."""

    def follow_checked_trajectory(self, *args, **kwargs):
        kwargs["tracking_pose"] = self.pose
        result = follow_trajectory(*args, **kwargs)
        if not kwargs.get("return_steering"):
            return result
        self.ensure_cascade()
        # The gate and PD/PI use one spatial reference, including its short-tail
        # rule. Preview neither advances progress nor integrates a held sample.
        if _r96_cpu.PROBE is None:
            reference = self.cascade.preview_reference(self.pose)
        else:
            with _r96_cpu.span("embodied_navigation.control.preview_reference", self):
                reference = self.cascade.preview_reference(self.pose)
        yaw = reference["heading_rad"]
        if getattr(self, "global_reference_settings", {"enabled": False})["enabled"]:
            self.reference_consistency_check = direction_check(
                self.cascade,
                reference,
                self.profile["teacher_transition"]["drive_heading_max_rad"],
                self.global_reference_settings,
            )
            observed = observe_business_gate(
                self,
                "scan_runtime_direction_coherence",
                self.reference_consistency_check,
                {"checked_subarc_direction_incoherent_requires_replan"},
            )
            self.reference_consistency_hold = not observed["allowed"]
            if self.reference_consistency_hold:
                self.reference_replan_pending = True
        if not self.reference_consistency_hold:
            early_handoff_request(self, reference)
        velocity, rate, target, steering = result
        original = copy.deepcopy(steering)
        steering = dict(
            steering,
            heading=yaw,
            error=math.atan2(
                math.sin(yaw - math.atan2(self.rotation[1, 0], self.rotation[0, 0])),
                math.cos(yaw - math.atan2(self.rotation[1, 0], self.rotation[0, 0])),
            ),
            heading_reference="same actual SCAN finite-arc spatial reference as cascade",
            original_lookahead_steering=original,
            cascade_projection=dict(
                path_id=self.cascade.path_id,
                path_sha256=self.cascade.path_sha,
                reference=copy.deepcopy(reference),
                actual_pose_stamp_ns=int(self.pose_stamp),
                navigation_ground_truth_used=False,
            ),
            direction_coherence=copy.deepcopy(getattr(self, "reference_consistency_check", None)),
        )
        self.evidence.append(
            self.run / "navigation_heading_projection.jsonl",
            dict(
                request_id=self.request_id,
                waypoint_index=self.waypoint_index,
                source_pose_stamp_ns=int(self.pose_stamp),
                control_stamp_ns=self.get_clock().now().nanoseconds,
                steering=copy.deepcopy(steering),
                feedback=copy.deepcopy(self.feedback),
                navigation_ground_truth_used=False,
            ),
        )
        return velocity, rate, target, steering

    def continuous_before_region(self, now, clock_ns):
        if self.automatic_reference is not None:
            return False
        ledger = self.continuous_route
        if ledger is None:
            return
        if ledger.points is None:
            ledger.activate(self.region_raw_pose, int(self.pose_stamp), clock_ns)
            self.waypoint_index = ledger.target_index
            self.region_arrival = ArrivalWindow(self.goals[self.waypoint_index])
        receipt = ledger.observe(
            self.region_raw_pose,
            int(self.pose_stamp),
            clock_ns,
            protected=self.obstacle_hold
            or self.tilt_hold
            or self.bridge_safety.get("state") != "ready",
        )
        if receipt is not None:
            self.region_arrivals.append(copy.deepcopy(receipt))
            self.evidence.append(self.run / "navigation_region_passes.jsonl", receipt)
            self.path_admission_replan_pending = True
            self.last_reference = -math.inf
        if ledger.missed_pass_requires_replan() and self.coverage_replan_index != ledger.next_index:
            self.coverage_replan_index = ledger.next_index
            self.evidence.append(
                self.run / "navigation_coverage_replans.jsonl",
                dict(
                    ledger.status(),
                    reason="ordinary_region_not_observed_replan_through_original_volume",
                    source_pose_stamp_ns=int(self.pose_stamp),
                    control_stamp_ns=clock_ns,
                    old_trajectory_id=self.active_trajectory_id,
                    safety_gates_preserved=True,
                ),
            )
            self.samples = None
            self.execution_path_exhaustion = None
            self.trajectory_association.reset()
            self.planning_start_state = None
            self.active_trajectory_id = None
            self.active_reference_stamp = None
            self.last_reference = -math.inf
            return True
        return False

    def continuous_record_hard(self, receipt, clock_ns):
        if self.continuous_route is not None:
            self.continuous_route.accept_hard_receipt(receipt, clock_ns)

    def continuous_release_target(self):
        if self.automatic_reference is not None:
            return
        ledger = self.continuous_route
        if ledger is None or self.state != "running" or ledger.points is None:
            return
        target = ledger.release_hard_target()
        if target != self.waypoint_index and ledger.next_index <= target:
            self.waypoint_index = target
            self.region_arrival = ArrivalWindow(self.goals[target])
            self.region_arrival.last_stamp = int(self.pose_stamp)
            self.region_arrival_evidence = None

    def execution_guard_route(self, start, goal):
        if self.automatic_reference is not None:
            bound = getattr(self, "_program_route_fence", None)
            if bound is not None and bound[0] == self._program_route_key():
                return self._program_guard_route
            return np.asarray(self.automatic_reference.points(), dtype=float)
        ledger = self.continuous_route
        return (
            ledger.guard_route()
            if ledger is not None and ledger.points is not None
            else np.vstack([start, goal])
        )

    def fixed_terminal_heading(self, index, start):
        if self.automatic_reference is not None:
            return self.automatic_reference.terminal_heading()
        ledger = self.continuous_route
        if ledger is not None and ledger.points is not None:
            return ledger.terminal_heading(index)
        delta = self.waypoints[index][:2] - start[:2]
        if np.linalg.norm(delta) < 1e-6:
            raise ValueError("Frozen goal heading requires nondegenerate actual SLAM segment")
        return math.atan2(delta[1], delta[0])

    def planner_goal(self):
        if self.automatic_reference is not None:
            return np.asarray(self.automatic_reference.planner_goal(), dtype=float)
        ledger = self.continuous_route
        return (
            self.waypoints[ledger.next_index]
            if ledger is not None and ledger.points is not None
            else super().planner_goal()
        )

    def request_plan(self):
        if pending_path_waiting(self):
            return
        if self.automatic_reference is not None:
            return super().request_plan()
        ledger = self.continuous_route
        index = None if ledger is None else ledger.next_index
        before = self.reference_requests
        result = super().request_plan()
        if self.reference_requests > before:
            self.scan_reference_coverage_index = index
        return result

    def planner_reference_points(self, goal):
        if self.automatic_reference is not None:
            return self.automatic_reference.points()
        if self.continuous_route is not None:
            points = self.continuous_route.reference_points(
                end_index=self.continuous_route.next_index
            )
            self.evidence.append(
                self.run / "navigation_global_reference_requests.jsonl",
                dict(
                    self.continuous_route.status(),
                    guide_points_xyz=copy.deepcopy(points),
                    guide_count=len(points),
                    local_body_goal=list(map(float, goal)),
                    local_endpoint_coverage_index=self.continuous_route.next_index,
                    fixed_hard_target_index=self.waypoint_index,
                    frame_id="camera_init",
                    guide_only_not_arrival=True,
                    collision_or_support_certified=False,
                    source_pose_stamp_ns=int(self.pose_stamp),
                    control_stamp_ns=self.get_clock().now().nanoseconds,
                ),
            )
            return points
        if not self.global_reference_settings["enabled"]:
            return [goal]
        if self.segment_start is None:
            raise ValueError("Global planning reference lacks original measured segment activation")
        points, receipt = self.global_route_guides.remaining(
            (self.request_id, self.waypoint_index),
            self.segment_start,
            goal,
            self.pose,
            self.global_reference_settings,
        )
        self.evidence.append(
            self.run / "navigation_global_reference_requests.jsonl",
            dict(
                receipt,
                source_pose_stamp_ns=int(self.pose_stamp),
                control_stamp_ns=self.get_clock().now().nanoseconds,
                goals_definition_sha256=self.goals_definition_sha256,
            ),
        )
        return points

    def record_segment_activation(self):
        key = (self.request_id, self.waypoint_index)
        if key not in self.segment_references:
            if self.segment_start is None or not np.array_equal(self.segment_start, self.pose):
                raise RuntimeError("Actual segment activation lacks its original SLAM pose")
            self.segment_references[key] = dict(
                segment_start=self.segment_start.tolist(),
                source_pose_stamp_ns=int(self.segment_start_pose_stamp_ns),
                control_stamp_ns=int(self.segment_started_ros_ns),
                feedback=copy.deepcopy(self.feedback),
                request_id=self.request_id,
                waypoint_index=self.waypoint_index,
            )
            self.evidence.append(
                self.run / "navigation_segment_activation.jsonl",
                dict(**self.segment_references[key], navigation_ground_truth_used=False),
            )

    def measured_region_arrival(self):
        self.record_segment_activation()
        if (
            self.continuous_route is not None
            and self.continuous_route.next_index != self.waypoint_index
        ):
            self.reset_region_arrival("ordered_pass_coverage_pending")
            return False, False
        return super().measured_region_arrival()

    def final_bundle_reference(self):
        """Build the last-waypoint bundle from a read-only receipt snapshot.

        Only self.path_receipt is replaced elsewhere; it is never mutated in
        place, so the bundle keeps every original key/value and identity without
        a per-waypoint points_xyz/metadata copy. The R96 CPU probe stays an
        observation-only hook: it times the same construction under its own
        stage label but never forces a deep copy.
        """
        receipt = self.path_receipt
        if _r96_cpu.PROBE is None:
            path_receipt = _final_path_receipt_snapshot(receipt)
        else:
            path_receipt = _r96_cpu.receipt_copy(
                _final_path_receipt_snapshot,
                receipt,
                self,
                stage="embodied_navigation.control.final_bundle_receipt_snapshot",
            )
        return dict(
            index=self.waypoint_index,
            start=self.segment_start.copy(),
            trajectory_id=self.active_trajectory_id,
            array=self.trajectory_archive_reference,
            path_receipt=path_receipt,
            reference_stamp=tuple(self.active_reference_stamp),
        )
