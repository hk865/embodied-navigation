"""Process entrypoint for the live navigation controller.

rclpy init, the ``IntegratedNavigation`` composition, executor/spinner lifecycle
and the final drain/receipt sequence. ``pid_navigation`` re-exports ``main`` and
passes the class explicitly so ``python -m ...pid_navigation`` stays compatible
without importing the module twice.
"""

import argparse
import json
import os
from pathlib import Path

from embodied_navigation.common.shutdown import (
    ShutdownRequested as _ShutdownRequested,
    ShutdownSignals as _ShutdownSignals,
)

from .pid_support import (
    _r67_observe_known_generation_ready,
    Path,
    _r107_scan_take_probe,
    _r114_clock_service,
    json,
    _r99_clock_probe,
    _r47_flush,
    _callback_cost_flush,
    _r118_source_age,
    _corrected_delivery_flush,
    _sensor_hold_flush,
    argparse,
    _r96_cpu,
    _r56_observe_admission_body,
    os,
)


def _drain_executor(executor):
    """Join callbacks before context/node destruction and retrieve real errors."""
    try:
        executor.shutdown()
    finally:
        executor._executor.shutdown(wait=True)
    # Jazzy retains submitted Tasks after the spinner stops. Fetch every done
    # exception, including a callback that finished during pool shutdown.
    tasks = set(getattr(executor, "_futures", ()))
    tasks.update(getattr(executor, "_pending_tasks", {}))
    errors = []
    for task in tasks:
        if task.done() and not task.cancelled():
            try:
                task.result()
            except BaseException as error:
                errors.append(error)
    if errors:
        raise BaseExceptionGroup("Controller callbacks failed while draining", errors)


def _main(shutdown_signals, navigation_class, description=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import MultiThreadedExecutor
    from embodied_navigation.common.time.cooperative_executor import (
        make_executor,
        receipt as r90_executor_receipt,
        finish as r90_executor_finish,
    )

    p = argparse.ArgumentParser(
        description=description if description is not None else __doc__
    )
    p.add_argument("--run", type=Path, required=True)
    args, ros = p.parse_known_args()
    run = args.run.resolve()
    os.environ["DEMO_RUN_DIR"] = str(run)
    from embodied_navigation.localization_mapping.known_scene_inputs import KnownSceneInputMixin

    class IntegratedNavigation(KnownSceneInputMixin, navigation_class):
        pass

    IntegratedNavigation._known_generation_ready = _r67_observe_known_generation_ready(
        KnownSceneInputMixin._known_generation_ready,
        KnownSceneInputMixin._control_candidate.__code__,
    )
    IntegratedNavigation._admit_matched_spline = _r56_observe_admission_body(
        IntegratedNavigation._admit_matched_spline
    )
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    node = IntegratedNavigation(run)
    executor = None
    try:
        # Start the new child only after the complete ROS node was constructed.
        from embodied_navigation.planning.admission.async_admission import (
            enabled as r88_enabled,
            Admission,
        )

        if r88_enabled() and node.r85_physical_gate is not None:
            eager_send = os.environ.get("R89_EAGER_PHYSICAL_SEND", "0")
            if eager_send not in ("0", "1"):
                raise ValueError("R89_EAGER_PHYSICAL_SEND must be 0 or 1")
            node._r88_admission = Admission(node, eager_submit=eager_send == "1")
        from embodied_navigation.common.observation import api as _observer_api

        install_r89_runtime_trace = _observer_api("r89_runtime_trace").install
        node._r89_trace_receipt = install_r89_runtime_trace(node)
        node._r96_cpu_receipt = _r96_cpu.install(node)
        executor = make_executor(MultiThreadedExecutor, node.profile, num_threads=2)
        node._r99_clock_probe_receipt = _r99_clock_probe.install(node, executor)
        node._r107_take_receipt = _r107_scan_take_probe.install(node, executor)
        node._r114_clock_service = _r114_clock_service.install(node, executor)
        node._r90_executor_receipt = r90_executor_receipt(executor)
        r91_selection = os.environ.get("R91_EMPTY_RETRY_YIELD", "0")
        if r91_selection not in ("0", "1"):
            raise ValueError("R91_EMPTY_RETRY_YIELD must be 0 or 1")
        node._r91_empty_retry_yield = None
        if r91_selection == "1":
            if (
                node._r90_executor_receipt.get("enabled") is not True
                or node._r90_executor_receipt.get("worker_threads") != 2
            ):
                raise ValueError(
                    "R91 requires the selected R90 sole-owner executor with two workers"
                )
            from embodied_navigation.common.time.empty_retry_yield import (
                install as install_r91_empty_retry_yield,
            )

            node._r91_empty_retry_yield = install_r91_empty_retry_yield(executor, enabled=True)
        node._r91_executor_receipt = dict(
            environment_key="R91_EMPTY_RETRY_YIELD",
            environment_value=r91_selection,
            candidate_commit=os.environ.get("R87_CANDIDATE_COMMIT"),
            candidate=None
            if node._r91_empty_retry_yield is None
            else node._r91_empty_retry_yield.snapshot(),
        )
        node._r118_source_age_binding = _r118_source_age.bind("controller", run)
        executor.add_node(node)
    except BaseException:
        shutdown_signals.begin_cleanup()
        try:
            if executor is not None:
                try:
                    try:
                        executor.shutdown()
                    finally:
                        executor._executor.shutdown(wait=True)
                except Exception:
                    pass
            _r96_cpu.detach()
            _r107_scan_take_probe.detach()
            _r99_clock_probe.detach()
            try:
                from embodied_navigation.common.observation import api as _observer_api

                finish_r89_runtime_trace = _observer_api("r89_runtime_trace").finish
                try:
                    finish_r89_runtime_trace(node)
                finally:
                    try:
                        _r96_cpu.finish(node)
                    finally:
                        _r99_clock_probe.finish(node)
            except Exception:
                pass
        finally:
            if node._r88_admission is not None:
                node._r88_admission.close()
        raise
    try:
        executor.spin()
    except _ShutdownRequested:
        pass
    finally:
        shutdown_signals.begin_cleanup()
        try:
            try:
                try:
                    _drain_executor(executor)
                finally:
                    try:
                        _r96_cpu.detach()
                    finally:
                        try:
                            _r107_scan_take_probe.detach()
                        finally:
                            _r99_clock_probe.detach()
            finally:
                try:
                    handoff = getattr(node, "_bridge_safety_handoff", None)
                    if handoff is not None:
                        handoff.close(node, service_negative=rclpy.ok())
                    if rclpy.ok():
                        node.publish_command()
                finally:
                    try:
                        if node.corridor_observer is not None:
                            remaining = node.corridor_observer.close()
                            if remaining is not None:
                                node.evidence.append(run / "corridor_certificates.jsonl", remaining)
                            node.evidence.append(
                                run / "corridor_worker_receipt.jsonl",
                                dict(
                                    submitted=node.corridor_observer.submitted,
                                    completed=node.corridor_observer.completed,
                                    busy_diagnostic_snapshots=node.corridor_observer.busy_snapshots,
                                    control_authority=False,
                                    navigation_ground_truth_used=False,
                                ),
                            )
                    except Exception as error:
                        node.evidence.append(
                            run / "corridor_input_rejections.jsonl",
                            dict(
                                reason="shadow_close:" + type(error).__name__ + ":" + str(error),
                                control_authority=False,
                            ),
                        )
                    try:
                        try:
                            if node._r88_admission is not None:
                                exit_receipt = node._r88_admission.close()
                                if exit_receipt is not None:
                                    (run / "r88_async_physical_exit.json").write_text(
                                        json.dumps(exit_receipt, allow_nan=False) + "\n"
                                    )
                        finally:
                            # New-resource cleanup cannot skip the existing
                            # source/archive drain when its own receipt fails.
                            if node.imu_source_buffer is not None:
                                node.imu_source_buffer.finish()
                            node.event_archive.close()
                            node.evidence.close()
                    finally:
                        (run / "navigation_pid_writer_receipt.json").write_text(
                            json.dumps(
                                dict(
                                    schema=1,
                                    expected_pid_records=node.pid_records,
                                    expected_guard_records=node.guard_sequence,
                                    expected_imu_records=node.imu_records,
                                    expected_cloud_archive_records=node.cloud_archive_records,
                                    expected_cloud_callback_records=node.cloud_callback_records,
                                    queue_error=node.evidence.error,
                                    expected_control_clock_hold_records=node.control_clock_hold.records,
                                    # PUBLICATION_OBSERVER_BEGIN writer_expected_count
                                    expected_command_publication_records=node.publication_records,
                                    final_commands_published_count=node.counts["commands"],
                                    # PUBLICATION_OBSERVER_END writer_expected_count
                                    status="failed" if node.evidence.error else "drained",
                                    navigation_verified=False,
                                )
                            )
                            + "\n"
                        )
                        node.destroy_node()
                        rclpy.try_shutdown()
        finally:
            try:
                try:
                    try:
                        None
                    finally:
                        r90_executor_finish(node, executor)
                finally:
                    from embodied_navigation.common.observation import api as _observer_api

                    finish_r89_runtime_trace = _observer_api("r89_runtime_trace").finish
                    try:
                        finish_r89_runtime_trace(node)
                    finally:
                        try:
                            _r96_cpu.finish(node)
                        finally:
                            _r99_clock_probe.finish(node)
            finally:
                try:
                    _r118_source_age.finish(run)
                    _r114_clock_service.finish(node)
                    _r47_flush(node)
                finally:
                    try:
                        _sensor_hold_flush(node)
                    finally:
                        try:
                            _corrected_delivery_flush(
                                getattr(node, "_r74_corrected_delivery_witness", None)
                            )
                        finally:
                            _callback_cost_flush(getattr(node, "_r74_callback_cost_witness", None))


def main(navigation_class=None, description=None):
    """Run the controller process; import the class lazily to avoid an import cycle."""
    if navigation_class is None:
        from .pid_navigation import PIDNavigation as navigation_class
    with _ShutdownSignals() as shutdown_signals:
        _main(shutdown_signals, navigation_class, description)


if __name__ == "__main__":
    main()
