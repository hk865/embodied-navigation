"""Result receipts and evidence journals for one DynaNav capture run.

This module owns the ``<output>/result.json`` contract: the initial result
dictionary and stage names, the throttled atomic save (the 200 Hz physics loop
must not write the data volume every tick), the append-only JSONL journals, and
the failure receipt that records the traceback before the caller re-raises and
the process exits non-zero. It imports no Isaac, ROS, torch or project runtime
modules at import time.
"""

import json
import time
import traceback
from dataclasses import dataclass, field


def initial_result(experiment, gt_diagnostic, slam_control):
    """Build the initial result dict written before Isaac starts."""
    result = {
        "stage": "startup",
        "outcome": "running",
        "experiment": experiment,
        "ground_truth_navigation": gt_diagnostic,
        "SLAM_control_enabled": not gt_diagnostic,
        "stair_speed_cap_enabled": False,
        "truth_usage": "policy proprioception, physics guards, diagnostic mapping and native-pose tracking"
        if gt_diagnostic
        else "policy proprioception, physical guards, optional isolated identity scoring; measured estimator poses drive the route and online mapping"
        if slam_control and experiment.get("semantic_provider") == "learned_rgb_person"
        else "policy proprioception, physical guards, diagnostic oracle map and scoring; measured estimator poses alone drive the route"
        if slam_control
        else "policy proprioception, COM velocity PID and scoring; navigation consumes SLAM",
        "navigation_running": False,
        "policy_loaded": False,
        "rgb_rendered": False,
    }
    return result


class Receipts:
    """Own ``result.json`` plus the append-only JSONL journals for one run.

    ``save`` is throttled to 0.25 s unless forced, and writes atomically through
    ``result.json.tmp`` so a crash never leaves a truncated receipt.
    """

    def __init__(self, output, experiment, gt_diagnostic, slam_control):
        self.out = output
        self.result = initial_result(experiment, gt_diagnostic, slam_control)
        self._last_receipt_wall = -float("inf")
        self._journals = {}

    def save(self, force=False):
        # Receipt I/O must not run at the 200 Hz physics rate on the data volume.
        now = time.perf_counter()
        if not force and now - self._last_receipt_wall < 0.25:
            return
        temporary = self.out / "result.json.tmp"
        temporary.write_text(json.dumps(self.result, indent=2))
        temporary.replace(self.out / "result.json")
        self._last_receipt_wall = now

    def journal(self, name, payload, *, flush=False):
        if name not in self._journals:
            self._journals[name] = (self.out / name).open("a")
        self._journals[name].write(json.dumps(payload) + "\n")
        if flush:
            self._journals[name].flush()

    def close_journals(self):
        """Attempt every stream, then propagate the first close/flush failure."""
        errors = []
        for name, stream in self._journals.items():
            try:
                stream.close()
            except BaseException as error:
                errors.append((name, error))
        if errors:
            first = errors[0][1]
            for name, error in errors[1:]:
                BaseException.add_note(first, f"journal {name}: {_safe_repr(error)}")
            raise first


@dataclass
class CaptureContext:
    """Optional runtime components referenced by the failure receipt.

    The original script inspected ``locals()`` in its ``except`` block; this
    object records the same components explicitly as each one is constructed.
    """

    receipts: Receipts
    app: object = None
    rgb_archive: object = None
    stream: object = None
    video: object = None
    mapping: object = None
    ros: object = None
    art: object = None
    published: dict = field(default_factory=dict)
    latest_odom: list | None = None
    rows: list | None = None
    sensor_rows: list | None = None
    camera_rows: list | None = None
    policy_frames: list | None = None
    dynamic_contact_events: list | None = None
    q: object = None
    qd: object = None
    target: object = None


def _safe_repr(error):
    """Exception formatting itself must not interrupt resource cleanup."""
    try:
        return repr(error)
    except BaseException:
        return f"<{type(error).__name__}: repr unavailable>"


def record_failure(receipts, error, context, *, interrupted_error=None):
    """Best-effort failure receipt and cleanup that never masks the primary error.

    The original top-level ``except`` block did one thing at a time and a failure
    in any receipt/cleanup step (for example a second ``app.close`` error) could
    replace the real simulation exception.  Here every receipt/cleanup step is
    attempted independently, cleanup errors are recorded under
    ``failure_cleanup_errors`` when the receipt can still be written, and the
    primary exception is left untouched for the caller to re-raise.
    """
    result = receipts.result
    cleanup_errors = []

    def attempt(label, action):
        try:
            action()
        except BaseException as cleanup_error:  # noqa: BLE001 - cleanup must not mask the primary error
            cleanup_errors.append(f"{label}: {_safe_repr(cleanup_error)}")

    # Put the primary error in the receipt first so it survives a later
    # receipt-write or cleanup failure.
    result.update(
        error=_safe_repr(error),
        outcome="cancelled" if isinstance(error, (KeyboardInterrupt, SystemExit)) else "failed",
        traceback="<traceback unavailable>",
    )
    attempt("traceback", lambda: result.update(
        traceback="".join(traceback.format_exception(type(error), error, error.__traceback__))
    ))

    if interrupted_error is not None:
        result["interrupted_operation_error"] = _safe_repr(interrupted_error)
        attempt("interrupted_operation_traceback", lambda: result.update(
            interrupted_operation_traceback="".join(traceback.format_exception(
                type(interrupted_error), interrupted_error, interrupted_error.__traceback__
            ))
        ))

    attempt("journals", receipts.close_journals)
    if context.rgb_archive is not None:
        def archive_receipt():
            acquired_frames = (
                context.stream.rgb_frames_acquired
                if context.stream is not None
                else 0
            )
            result["rgb_archive"] = context.rgb_archive.receipt(
                acquired_frames=acquired_frames,
                published_frames=context.published.get("rgb"),
                video_frames=context.video.frames if context.video is not None else None,
            )

        attempt("rgb_archive", archive_receipt)
    if context.qd is not None:
        def actuator_failure_state():
            from isaacsim.core.simulation_manager import SimulationManager

            result["actuator_failure_state"] = {
                "q_isaac": context.q.tolist(),
                "qd_isaac": context.qd.tolist(),
                "dof_names": context.art.dof_names,
                "target_isaac": context.target.tolist(),
                "physics_s": float(SimulationManager.get_simulation_time()),
            }

        attempt("actuator_failure_state", actuator_failure_state)
    if context.mapping is not None:
        attempt("mapping", lambda: context.mapping.finalize_failure(result))

    def failure_report():
        report = getattr(error, "report", None)
        if report is not None:
            # Reject unserializable diagnostics before they poison result.json.
            json.dumps(report)
            result["failure_report"] = report

    attempt("failure_report", failure_report)
    if context.video is not None:
        attempt("video", lambda: context.video.close(timeout=10))
    if context.ros is not None:
        attempt("ros", context.ros.destroy)

    def runtime_evidence():
        result.update(
            rows=context.rows or [],
            sensor_rows=context.sensor_rows or [],
            camera_rows=context.camera_rows or [],
            policy_frames=context.policy_frames or [],
            published=context.published,
            latest_odom=(context.latest_odom or [None])[0],
        )
        result["dynamic_contact_events"] = context.dynamic_contact_events or []

    attempt("runtime_evidence", runtime_evidence)

    if context.app is not None:
        result.update(app_close_requested=True, app_close_returned=False)
    if cleanup_errors:
        result["failure_cleanup_errors"] = cleanup_errors
    # This is the last guaranteed Python write: native fast shutdown may exit
    # the process directly, without returning to our cleanup handler.
    attempt("receipt", lambda: receipts.save(force=True))
    attempt("traceback_output", lambda: print(result["traceback"], flush=True))
    if context.app is not None:
        def close_app():
            context.app.close(exit_code=1)
            result["app_close_returned"] = True

        attempt("app_close", close_app)

    if cleanup_errors:
        result["failure_cleanup_errors"] = cleanup_errors
    # Best effort on returning paths; terminal outcome already precedes close.
    attempt("receipt_after_cleanup", lambda: receipts.save(force=True))
