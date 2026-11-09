#!/usr/bin/env python3
"""Live navigation controller entry for the default native launch.

Process role ``controller`` (``control/launch/components.py``) -> ``main`` ->
``IntegratedNavigation`` -> ``PIDNavigation`` -> ``ros_node.TeacherNavigation``
-> ``base_node.Navigation``.

The task state machine (idle/running/succeeded/failed/stopped and the
drive/pre_turn/align/settle heading phases) lives in
``base_node.Navigation.control()``; this class adds path admission, cascade
command limits, parking ownership and evidence plumbing.

Layout (the class is composed from verbatim method mixins):
- ``pid_ingress``: ROS callbacks for requests, corridor, IMU, odometry, cloud.
- ``pid_admission`` / ``pid_admission_apply``: admission lifecycle and the
  large spline-vs-route decision body.
- ``pid_route``: checked-trajectory execution, continuous progress, planning.
- ``pid_control``: cascade/PID velocity selection, ACK gating, parking,
  terminal protection and command publication.
- ``pid_support``: shared evidence/probe helpers and imports.
- ``pid_entrypoint``: rclpy lifecycle, executor drain and process ``main``.
"""

# Absolute imports keep the legacy "run this file directly" entry working: the
# module may be executed as ``__main__`` (no package) as well as imported as
# ``embodied_navigation.navigation.pid_navigation``.  All implementation stays in
# the responsibility modules; this file is only the compatibility facade.
from embodied_navigation.navigation.pid_support import (
    math,
    continuous_configuration,
    FrozenFence,
    ResetBookkeeping,
    CausalImuBuffer,
    ControlClockHold,
    TERMINAL_PARKING_OWNER_ENABLED,
    deque,
    global_reference_contract,
    RawImuTilt,
    TeacherNavigation,
    imu_buffer_enabled,
    SegmentGuides,
    _r47_initialize,
)

# Re-export the original module-level helpers (pre-split callers could import
# them from this path) plus the process entry helpers.  They are imported, not
# copied, so the implementation still lives in ``pid_support``/``pid_entrypoint``.
from embodied_navigation.navigation.pid_support import (
    _plain_json_evidence_graph,
    _enqueue_navigation_imu_record,
    _copy_path_receipt_metadata,
    _final_path_receipt_snapshot,
    _pid_receipt_json_bytes,
    _append_pid_row_evidence,
    _TeacherSubstageTiming,
    _r47_begin,
    _r47_guard_dimensions,
    _r47_publication_anchor,
    _r47_mark,
    _r47_bind,
    _r47_end,
    _r56_observe_admission_body,
    _r47_flush,
    _pre_turn_external_hold_seed_wait,
    _r67_capture_known_frame_false,
    _r67_attach_known_frame_failure,
    _r67_observe_known_generation_ready,
    observe_business_gate,
    finite_retention_reference,
    diagnostic_profile_view,
    admission_callback_rejected,
    _admission_metadata_fragment,
    _receipt_json_with_metadata_fragment,
)
from embodied_navigation.navigation.pid_entrypoint import (
    _drain_executor,
    _main as _entry_main,
    main as _entrypoint_main,
)

from embodied_navigation.navigation.pid_admission import AdmissionMixin
from embodied_navigation.navigation.pid_admission_apply import AdmissionApplyMixin
from embodied_navigation.navigation.pid_control import ControlMixin
from embodied_navigation.navigation.pid_ingress import IngressMixin
from embodied_navigation.navigation.pid_route import RouteMixin


class PIDNavigation(
    IngressMixin,
    AdmissionMixin,
    AdmissionApplyMixin,
    RouteMixin,
    ControlMixin,
    TeacherNavigation,
):
    """Default navigation controller; state lives here, behavior in the mixins."""

    def __init__(self, run):
        super().__init__(run)
        self.pid = ResetBookkeeping()  # no legacy controller mathematics
        self._r105_terminal_owner_enabled = TERMINAL_PARKING_OWNER_ENABLED
        self._r105_terminal_context = None
        _r47_initialize(self, run)
        self.pid_velocity = None
        self.pid_pose_stamp = None
        self.pid_row = None
        self._pid_row_receipt_snapshot = None
        self._pid_receipt_bytes_cache = None
        self.pid_records = 0
        self.pid_guard_needs_evaluation = False
        self.pid_prepared_output = None
        self.cloud_callback_records = 0
        self.pending_cloud_callback_row = None
        self.imu_history = deque(maxlen=512)
        self.feedback = None
        self.paired_imu = None
        self.imu_source_buffer = None
        self.causal_imu_subscription = None
        if imu_buffer_enabled(self.profile):
            self.imu_source_buffer = CausalImuBuffer(self, RawImuTilt)
            self.causal_imu_subscription = self.create_causal_imu_subscription(
                self.imu_source_buffer.callback
            )
        self.cascade = None
        self.cascade_key = None
        self.cascade_last_row = None
        self.path_receipt = None
        self.cascade_path_id = None
        self.final_bundle = None
        self.imu_records = 0
        self.cloud_archive_records = 0
        self.segment_references = {}
        self.global_reference_settings = global_reference_contract(self.profile)
        self.global_route_guides = SegmentGuides()
        self.continuous_settings = continuous_configuration(self.profile)
        self.continuous_route = None
        self.coverage_replan_index = None
        self.scan_reference_coverage_index = None
        self.reference_consistency_hold = False
        self.reference_consistency_check = None
        self.registered_cloud_archive_mode = self.profile.get("engineering_recording", {}).get(
            "registered_cloud_archive", "legacy_all"
        )
        if self.registered_cloud_archive_mode not in ("legacy_all", "disabled"):
            raise ValueError(
                "Unsupported registered_cloud_archive mode; bounded_event is not yet implemented"
            )
        self.reference_replan_pending = False
        self.path_handoff_early_requested_for = None
        self.path_handoff_last_check = None
        self.pending_path_commit = None
        from embodied_navigation.planning.reference.automatic_reference_runtime import (
            enabled as r85_enabled,
            AutomaticReferenceRuntime,
            AutomaticRouteFence,
        )

        self.automatic_reference = (
            AutomaticReferenceRuntime(self.run, self.profile, self.trajectory_association)
            if r85_enabled(self.profile)
            else None
        )
        if self.automatic_reference is not None:
            # Original46 observation must be separate; it cannot drive this task.
            self.continuous_settings = dict(self.continuous_settings, enabled=False)
        self.path_admission_fence = (
            AutomaticRouteFence(self.automatic_reference)
            if self.automatic_reference is not None
            else FrozenFence(self.run, self.profile)
        )
        self.r85_physical_gate = None
        if self.automatic_reference is not None:
            from embodied_navigation.planning.admission.physical_gate import R85PhysicalPathGate

            self.r85_physical_gate = R85PhysicalPathGate(self.run)
        self._r88_admission = None
        self.path_admission_active = None
        self.path_admission_hold = False
        self.path_admission_replan_pending = False
        self.path_admission_sequence = 0
        self.path_admission_last_decision = None
        self.control_clock_hold = ControlClockHold()
        self.corridor_observer = None
        self.corridor_last_certificate = None
        self.corridor_last_request_wall = -math.inf
        self.corridor_snapshot_count = 0
        corridor_config = self.profile.get("corridor", {"mode": "off"})
        if corridor_config.get("mode") not in ("off", "shadow") or corridor_config.get(
            "retain_valid_plan", False
        ):
            raise ValueError(
                "Unvalidated corridor control authority is not enabled in this revision"
            )
        if corridor_config.get("mode") == "shadow":
            from embodied_navigation.common.observation import api as _observer_api

            CorridorObserver = _observer_api("corridor_runtime").CorridorObserver
            from nav_msgs.msg import Path as PathMsg
            from std_msgs.msg import String

            self.corridor_observer = CorridorObserver(
                dict(
                    horizon_m=1.2,
                    max_half_width_m=0.30,
                    width_step_m=0.04,
                    max_speed_mps=float(self.profile["max_speed_mps"]),
                    brake_decel_mps2=0.15,
                    latency_s=0.3,
                    stop_margin_m=0.10,
                    body_radius_m=0.25,
                    body_offset_m=0.18,
                    body_below_m=0.12,
                    body_above_m=0.12,
                    position_uncertainty_m=0.08,
                    yaw_half_range_rad=math.pi,
                    map_max_age_ns=300_000_000,
                    cell_max_age_ns=300_000_000,
                    support_max_age_ns=300_000_000,
                    body_ground_height_m=0.32,
                    support_height_tolerance_m=0.10,
                    max_support_slope_rad=0.35,
                )
            )
            self.corridor_request_pub = self.create_publisher(
                PathMsg, "/demo/teacher/corridor/request", 1
            )
            self.corridor_snapshot_sub = self.create_subscription(
                String, "/demo/teacher/corridor/snapshot", self.on_corridor_snapshot, 1
            )
        # PUBLICATION_OBSERVER_BEGIN init_counter
        self.publication_records = 0


def _main(shutdown_signals):
    """Keep the original private entry signature (used by older callers/tests).

    The implementation now lives in :mod:`pid_entrypoint`; this thin wrapper
    simply binds the composed class and the controller docstring.
    """
    return _entry_main(shutdown_signals, PIDNavigation, __doc__)


def main():
    """Run the controller process (same entry as before the split)."""
    _entrypoint_main(PIDNavigation, __doc__)


if __name__ == "__main__":
    main()
