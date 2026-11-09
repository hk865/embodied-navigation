"""Process entrypoint for the live navigation controller.

rclpy init, the ``IntegratedNavigation`` composition, executor/spinner lifecycle
and the final drain/receipt sequence. ``pid_navigation`` re-exports ``main`` and
passes the class explicitly so ``python -m ...pid_navigation`` stays compatible
without importing the module twice.
"""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# 现役导航控制器的进程入口。
#
# 职责：初始化 rclpy、把 ``KnownSceneInputMixin`` 与导航类组合成
# ``IntegratedNavigation``、装配 ``MultiThreadedExecutor`` 与各观测探针、进入
# ``spin``，最后按固定顺序 drain 回调、关闭节点并写出收据。``pid_navigation``
# 重新导出 ``main`` 并显式传入类，使 ``python -m ...pid_navigation`` 在不再
# import 一次模块的前提下保持兼容。
#
# 边界：本模块不做任务状态机、准入或速度决策（这些在
# ``base_node.Navigation.control()`` 与 ``pid_*`` mixin 中），只负责进程生命周期
# 与收尾顺序。收尾必须先停命令、再关资源、最后写收据，顺序见 ``_main`` 的
# ``finally`` 段。


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
    """在销毁 context/node 前汇合回调线程，并取回真实回调异常。

    先停 executor 的 spin，再让底层线程池 ``shutdown(wait=True)`` 等已提交回调
    结束。Jazzy 会把已提交的 Task 留在 executor 上，因此这里显式收集
    ``_futures`` 与 ``_pending_tasks`` 中已完成的结果：任一回调抛错就抛
    ``BaseExceptionGroup``，避免关停路径吞掉业务异常。
    """
    try:
        executor.shutdown()
    finally:
        executor._executor.shutdown(wait=True)
    # Jazzy 在 spinner 停止后仍保留已提交的 Task；这里连线程池关闭期间刚结束的
    # 回调一起取回异常，而不是只依赖 executor 自身的日志。
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
    """构造并运行一个 controller 进程，返回 ``None``。

    参数：``shutdown_signals`` 是 ``main`` 打开的 ``ShutdownSignals``；
    ``navigation_class`` 是要组合的导航基类；``description`` 仅在该函数被直接调用时
    作为 ``argparse`` 的帮助文本（``pid_navigation._main`` 传入自己的模块
    docstring）。``--run`` 是唯一必填参数（run 目录），其余参数原样交给 rclpy，
    ROS remap 等仍生效。返回前执行固定的收尾序列；构造/装配阶段抛出的异常会在
    清理已安装资源后原样抛给调用方。
    """
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
    # 只认 --run，其余参数留给 rclpy（parse_known_args），launch 传入的 ROS
    # remap/参数不会被 argparse 拒绝。
    args, ros = p.parse_known_args()
    run = args.run.resolve()
    # 节点构造与证据路径都从环境变量取 run 目录；必须在实例化节点前写入。
    os.environ["DEMO_RUN_DIR"] = str(run)
    from embodied_navigation.localization_mapping.known_scene_inputs import KnownSceneInputMixin

    # 组合类不新增行为，只把 known-scene 输入混入现役导航类。
    class IntegratedNavigation(KnownSceneInputMixin, navigation_class):
        pass

    # 两处包装只在本进程生效、只加诊断标记，不复制业务判定（实现见 pid_support）。
    IntegratedNavigation._known_generation_ready = _r67_observe_known_generation_ready(
        KnownSceneInputMixin._known_generation_ready,
        KnownSceneInputMixin._control_candidate.__code__,
    )
    IntegratedNavigation._admit_matched_spline = _r56_observe_admission_body(
        IntegratedNavigation._admit_matched_spline
    )
    # SignalHandlerOptions.NO：信号转正常关闭由 ShutdownSignals 自己处理，避免
    # rclpy 自动 shutdown 在回调或收尾中途销毁 context（见
    # tests/unit/test_ros_shutdown_lifecycle.py）。
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    node = IntegratedNavigation(run)
    executor = None
    try:
        # 等整个 ROS 节点构造完成后再启动 R88 准入（其几何 worker 是子进程）。
        from embodied_navigation.planning.admission.async_admission import (
            enabled as r88_enabled,
            Admission,
        )

        # R88 异步物理准入：模块默认关闭（"0"），生产由 launch features 注入 1
        # （docs/review/validation-and-threshold-audit.md E5）；且必须已有 R85 物理门。
        if r88_enabled() and node.r85_physical_gate is not None:
            # 代码默认 "0"：物理检查请求提交后不立即 flush，等下一次轮询再送出。
            eager_send = os.environ.get("R89_EAGER_PHYSICAL_SEND", "0")
            if eager_send not in ("0", "1"):
                raise ValueError("R89_EAGER_PHYSICAL_SEND must be 0 or 1")
            node._r88_admission = Admission(node, eager_submit=eager_send == "1")
        from embodied_navigation.common.observation import api as _observer_api

        # R89/R96/R99/R107 是观测端口：本检出经 common.observation.api 恒返回禁用实现
        # （common/observation.py:86-88），install 为空操作。R114_CLOCK_SERVICE 则是真实
        # 开关（代码默认 "0"，configs/launch/settings.json 设为 "1"），启用时会包装原始
        # /clock 订阅，前置校验不符直接报错。
        install_r89_runtime_trace = _observer_api("r89_runtime_trace").install
        node._r89_trace_receipt = install_r89_runtime_trace(node)
        node._r96_cpu_receipt = _r96_cpu.install(node)
        # 硬编码 2 个 worker：R91 空轮询 yield 只允许 R90 sole-owner executor 恰为两个
        # 线程，下面会校验该收据，不满足则拒绝启动。
        executor = make_executor(MultiThreadedExecutor, node.profile, num_threads=2)
        node._r99_clock_probe_receipt = _r99_clock_probe.install(node, executor)
        node._r107_take_receipt = _r107_scan_take_probe.install(node, executor)
        node._r114_clock_service = _r114_clock_service.install(node, executor)
        node._r90_executor_receipt = r90_executor_receipt(executor)
        # R91 空轮询 yield：代码默认 "0"（关闭）；configs/launch/settings.json 的
        # features 将其设为 "1"。它只在上面那个 sole-owner 双 worker executor 上可用。
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
        # 收据同时记录环境值、R87 候选 commit 与实际安装快照；candidate 为 None 表示
        # 未启用。R87_CANDIDATE_COMMIT 只作记录，不参与这里的分支。
        node._r91_executor_receipt = dict(
            environment_key="R91_EMPTY_RETRY_YIELD",
            environment_value=r91_selection,
            candidate_commit=os.environ.get("R87_CANDIDATE_COMMIT"),
            candidate=None
            if node._r91_empty_retry_yield is None
            else node._r91_empty_retry_yield.snapshot(),
        )
        # 把源年龄探针绑定到本 run 的 controller 角色；禁用端口下为空操作。
        node._r118_source_age_binding = _r118_source_age.bind("controller", run)
        # 节点完全装配成功后才加入 executor，此前任何一步失败都不会进入 spin。
        executor.add_node(node)
    except BaseException:
        # 装配失败也要反向关闭已安装资源：先禁止重复信号打断清理，再停 executor、
        # 卸载探针、结束 R89 trace、关闭 R88 准入，最后原样抛出原异常（不吞错）。
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
        # 只有进程信号会转成这个异常；其他回调异常继续向外抛。
        pass
    finally:
        # 进入清理后，重复的 SIGINT/SIGTERM 不再中断收尾。
        shutdown_signals.begin_cleanup()
        try:
            # 先 drain 回调（等所有已提交 Task 结束）再卸载探针；顺序反过来会在
            # 回调仍写证据时破坏其观测状态。
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
                        # 先关闭执行桥接交接并处理其中挂起的负向（保护）事件，再补发命令。
                        handoff.close(node, service_negative=rclpy.ok())
                    if rclpy.ok():
                        # 不带 velocity 的 publish_command = 零速停车并撤销命令源。
                        node.publish_command()
                finally:
                    try:
                        # 走廊观察者收尾：写回剩余证书与 worker 计数；失败只追加一条
                        # 拒绝证据，绝不阻断后续的资源关闭。
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
                            # R88 准入关闭返回非空收据时写 r88_async_physical_exit.json。
                            if node._r88_admission is not None:
                                exit_receipt = node._r88_admission.close()
                                if exit_receipt is not None:
                                    (run / "r88_async_physical_exit.json").write_text(
                                        json.dumps(exit_receipt, allow_nan=False) + "\n"
                                    )
                        finally:
                            # 新增资源（IMU 源缓冲、事件归档）的清理即使自身收据失败，
                            # 也不能跳过既有的 source/archive drain 与证据关闭。
                            if node.imu_source_buffer is not None:
                                node.imu_source_buffer.finish()
                            node.event_archive.close()
                            node.evidence.close()
                    finally:
                        # 写入本 run 的期望计数与队列错误；status 只在证据队列出错时为
                        # failed。navigation_verified 恒为 False：这表示数据已 drain，
                        # 不构成导航闭环验证。
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
                        # 先销毁节点、再关 rclpy context。
                        node.destroy_node()
                        rclpy.try_shutdown()
        finally:
            # 诊断收据 flush：嵌套 finally 保证前一步失败时仍继续执行后续 flush。
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
    """运行 controller 进程；延迟导入类以避免 import 环。

    ``navigation_class`` 为空时（``-m`` / console script 路径）才导入
    ``pid_navigation.PIDNavigation``，因为该模块在顶层反过来 import 本模块的
    ``_main``/``main``。``description`` 原样交给 ``argparse``。函数在
    ``ShutdownSignals`` 作用域内调用 ``_main``，退出时恢复原信号处理器；异常
    从作用域向外传播。
    """
    if navigation_class is None:
        from .pid_navigation import PIDNavigation as navigation_class
    with _ShutdownSignals() as shutdown_signals:
        _main(shutdown_signals, navigation_class, description)


if __name__ == "__main__":
    main()
