"""单次 DynaNav 采集运行的结果收据与证据 journal。

本模块定义 ``<output>/result.json`` 契约：初始结果字典与阶段名、限频的原子
保存（200 Hz 物理循环不能每 tick 写数据卷）、只追加的 JSONL journal，以及
在调用方重新抛异常、进程非零退出之前记录 traceback 的失败收据。import 本模块
不会加载 Isaac、ROS、torch 或项目运行时模块。
"""

import json
import time
import traceback
from dataclasses import dataclass, field


def initial_result(experiment, gt_diagnostic, slam_control):
    """构造 Isaac 启动前写入的初始结果字典。

    ``stage`` 从 ``startup`` 开始，``outcome`` 为 ``running``；
    ``SLAM_control_enabled`` 与 ``ground_truth_navigation`` 互斥。
    ``truth_usage`` 按运行模式分四支：真值诊断、SLAM + learned 行人语义、
    普通 SLAM、以及 COM 速度 PID（导航消费 SLAM）。分支由本函数内的条件
    表达式决定，不是运行时再判断。
    """
    result = {
        "stage": "startup",
        "outcome": "running",
        "experiment": experiment,
        "ground_truth_navigation": gt_diagnostic,
        "SLAM_control_enabled": not gt_diagnostic,
        "stair_speed_cap_enabled": False,
        # 真值用途声明会写入 result.json，供离线审计区分“策略本体感知/物理保护/
        # 诊断建图”与“实测估计位姿驱动路线”两类运行。
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
    """持有一次运行的 ``result.json`` 与只追加 JSONL journal。

    ``save`` 默认限频 0.25 s（force=True 可绕过），并通过 ``result.json.tmp``
    原子替换，崩溃时不会留下被截断的收据；journal 首次写入时才打开文件，
    由 ``close_journals`` 统一收尾。
    """

    def __init__(self, output, experiment, gt_diagnostic, slam_control):
        self.out = output
        self.result = initial_result(experiment, gt_diagnostic, slam_control)
        self._last_receipt_wall = -float("inf")
        self._journals = {}

    def save(self, force=False):
        # 收据 I/O 不能按 200 Hz 物理频率写数据卷：非强制调用在 0.25 s 内
        # 直接返回。force 用于失败收尾与终态，保证最后一次写入不被限频吞掉。
        now = time.perf_counter()
        if not force and now - self._last_receipt_wall < 0.25:
            return
        temporary = self.out / "result.json.tmp"
        temporary.write_text(json.dumps(self.result, indent=2))
        temporary.replace(self.out / "result.json")
        self._last_receipt_wall = now

    def journal(self, name, payload, *, flush=False):
        # 每个 journal 是独立追加流，按 name 懒打开；payload 必须可 JSON 序列化，
        # flush=True 供需要立刻可读的边界事件使用（不改变持久性语义）。
        if name not in self._journals:
            self._journals[name] = (self.out / name).open("a")
        self._journals[name].write(json.dumps(payload) + "\n")
        if flush:
            self._journals[name].flush()

    def close_journals(self):
        """逐个尝试关闭所有 journal，最后抛出第一个关闭/刷新异常。

    其余流的异常以 note 形式附在第一个异常上，避免清理阶段互相掩盖。
    """
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
    """失败收据可能引用的可选运行时组件。

    原脚本在 ``except`` 块里检查 ``locals()``；这里改为在组件构造时显式
    登记同一批对象，未构造成功的组件保持 None，收据阶段据此跳过。
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
    """尽力写失败收据并清理，且绝不掩盖主异常。

    原顶层 ``except`` 块逐步执行，任何收据/清理步骤再抛错（例如第二次
    ``app.close`` 失败）都会替换真正的仿真异常。这里每个步骤独立尝试，
    清理错误在收据仍可写时记入 ``failure_cleanup_errors``，主异常保持原样
    交给调用方重新抛出。``interrupted_error`` 用于记录“收尾时又被中断”的
    次要异常。
    """
    result = receipts.result
    cleanup_errors = []

    def attempt(label, action):
        # 每个收尾动作单独隔离：任何 BaseException（含 SystemExit）都只记录
        # 到 cleanup_errors，不向上传播，确保后续收尾仍会执行。
        try:
            action()
        except BaseException as cleanup_error:  # noqa: BLE001 - cleanup must not mask the primary error
            cleanup_errors.append(f"{label}: {_safe_repr(cleanup_error)}")

    # 先把主错误写进收据，后续收据写入或清理再失败也不会丢掉它；
    # KeyboardInterrupt/SystemExit 记为 cancelled，其余为 failed。
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
            # 先剔除不可序列化的诊断值，避免它们污染 result.json 的写入。
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

    # 先声明“已请求关闭、尚未返回”：若随后 native 快速关闭直接结束进程，
    # result.json 仍能证明关闭请求发生过。
    if context.app is not None:
        result.update(app_close_requested=True, app_close_returned=False)
    if cleanup_errors:
        result["failure_cleanup_errors"] = cleanup_errors
    # 这是 Python 侧最后一次有保证的写入：native 快速关闭可能直接结束进程，
    # 不再回到本清理函数；因此必须在 app.close 之前强制保存收据。
    attempt("receipt", lambda: receipts.save(force=True))
    attempt("traceback_output", lambda: print(result["traceback"], flush=True))
    if context.app is not None:
        def close_app():
            context.app.close(exit_code=1)
            result["app_close_returned"] = True

        attempt("app_close", close_app)

    if cleanup_errors:
        result["failure_cleanup_errors"] = cleanup_errors
    # 若 app.close 正常返回，再补存一次，把 app_close_returned 等终态落盘；
    # 终态 outcome 已在关闭前写入，这一步只是尽力而为。
    attempt("receipt_after_cleanup", lambda: receipts.save(force=True))
