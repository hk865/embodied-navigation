"""只在一次新鲜短重复上保留既有的 R85 typed 零字节。

职责：当控制时钟出现一次短重复、且当前处于“无保护的 ordinary 零持有”状态时，
核对源文件 `cascade_command_source.json` 的字节、时间戳与绑定身份，决定是否允许
沿用这份既有零源而不重写它。
边界：不制造来源、事件、frame 授权、收据时钟或命令；bridge/executor 仍会每帧
独立校验这份未变来源。本模块只读源文件、不做 I/O 写；失败返回拒绝回执而非放行。
"""

import hashlib
import json
import math
from pathlib import Path
import time

from embodied_navigation.control.parking.parking_zero_contract import exact_zero
from embodied_navigation.robot.unitree.bridge import validate_controller_source

# 300 ms（硬编码）：短重复上下文与来源收据的有效期，与 clock_hold 的 TTL 一致。
TTL_NS = 300_000_000
CONTEXT_SCHEMA = "r85_short_duplicate_ordinary_source_context/v1"
RECEIPT_SCHEMA = "r85_short_duplicate_ordinary_source_preservation/v1"


def canonical_sha(value):
    """规范化 JSON 的 SHA-256（键排序、紧凑分隔符、拒绝 NaN）；用于字节身份比对。"""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def preserve_existing_source(node, context, clock_ns, *, command_before):
    """在原始 terminal 保护观察之后调用；不做 I/O 写。

    参数：`node` 为控制节点；`context` 是 `clock_hold` 在短重复分支构造的上下文
    （schema 与 `decision` 必须匹配）；`clock_ns` 是本帧控制时钟（ns）；
    `command_before` 是发布前的命令（必须精确为零）。返回允许/拒绝回执：
    允许时附源文件字节与时间证据；拒绝时 `allowed=False` 并带 `reason`/`error`。
    捕获到的读取/校验异常都会转成拒绝回执（fail closed），不抛给控制路径。
    """
    result = dict(
        schema=RECEIPT_SCHEMA,
        allowed=False,
        source_file_written=False,
        source_timestamps_refreshed=False,
        event_recreated=False,
        frame_authorization_reused=False,
    )
    try:
        # 第一组前置条件：必须是一次显式 R85 短重复，且发布前命令精确为零。
        # 任一不满足都说明这不是“同一份零源”的场景，直接拒绝。
        if (
            getattr(node, "automatic_reference", None) is None
            or not isinstance(context, dict)
            or context.get("schema") != CONTEXT_SCHEMA
            or context.get("decision") != "duplicate"
            or context.get("protected_or_stale_reset") is not False
            or context.get("source_stale_reasons") != []
            or not exact_zero(context.get("command_before", []))
            or not exact_zero(command_before)
            or list(command_before) != context["command_before"]
            or context.get("compute_ros_clock_ns") != clock_ns
            or type(context.get("compute_monotonic_wall_ns")) is not int
        ):
            raise ValueError("not_explicit_R85_fresh_short_duplicate")
        # 第二组：时钟保持状态必须仍是本帧那个短重复（holding/guard_required 都置位，
        # last clock/wall 与上下文一致，且最后一次进度在 300 ms 内）。
        hold = node.control_clock_hold
        if (
            hold.holding is not True
            or hold.guard_required is not True
            or hold.last_clock_ns != clock_ns
            or hold.last_wall_ns != context["compute_monotonic_wall_ns"]
            or hold.last_progress_wall_ns is None
            or not 0 <= context["compute_monotonic_wall_ns"] - hold.last_progress_wall_ns < TTL_NS
        ):
            raise ValueError("not_current_short_duplicate_clock_hold")
        # 上下文本身不能过期或来自未来；墙钟用 monotonic_ns（ns）。
        now = time.monotonic_ns()
        if not 0 <= now - context["compute_monotonic_wall_ns"] < TTL_NS:
            raise ValueError("duplicate_context_expired_or_future")
        # owner failure 只是诊断观察：仅在 diagnostic_observation 开启时才做额外判定，
        # 不因此放宽正式业务失败。
        cascade = node.cascade
        ordinary = cascade.ordinary_parking
        owner = getattr(node, "terminal_parking_owner", None)
        observed_owner_failure = False
        if ordinary.failure:
            from embodied_navigation.common.observation import api as _observer_api

            diagnostic_enabled = _observer_api("diagnostic_observation").enabled
            if diagnostic_enabled(node.profile):
                from embodied_navigation.control.parking.ordinary_parking import (
                    diagnostic_business_failure,
                )

                observed_owner_failure = diagnostic_business_failure(
                    ordinary.failure
                ) and ordinary.diagnostic_failure_is_observed(
                    dict(run=node.run, profile=node.profile)
                )
        # 第三组：当前必须是无保护的 ordinary 零 owner。这里逐项核对状态、锁存、
        # 交接阶段、请求身份与精确零命令；任一不符即拒绝（宁拒绝不误放行）。
        if (
            node.state not in ("running", "succeeded")
            or node.evidence.error
            or node.pid_row is not None
            or node.pending_guard_row is not None
            or node.obstacle_hold
            or node.tilt_hold
            or node.bridge_safety.get("state") != "ready"
            or getattr(node, "ordinary_terminal_failure_latched", None) is not None
            or cascade.protected
            or cascade.failure_latched
            or ordinary.failure
            and not observed_owner_failure
            or not ordinary.enabled
            or ordinary.stage not in ("handoff_zero", "zero_hold")
            or ordinary.ready_stamp is None
            or ordinary.handoff_clock is None
            or owner is not ordinary
            or getattr(node, "terminal_parking_request", None) != node.request_id
            or cascade.row.get("mode") != "ordinary_zero_hold"
            or cascade.row.get("protection_active") is not False
            or cascade.ordinary_zero_event is None
            or node.cascade_key[0] != node.request_id
            or not exact_zero(node.command)
            or not exact_zero(cascade.command)
            or not node._known_generation_ready()
        ):
            raise ValueError("not_current_unprotected_ordinary_zero_owner")
        # 第四组：SCAN epoch 必须属于当前 run 且 ready，planner epoch、planner 身份与
        # localization generation 必须与本次自动参考 scope 完全一致。
        epoch = node.known_scene_scan_epoch
        scope = node.automatic_reference.scope
        if (
            not isinstance(epoch, dict)
            or len(scope) != 3
            or any(type(v) is not int for v in scope)
            or type(epoch.get("generation")) is not int
            or type(epoch.get("localization_generation")) is not int
            or type(node.known_scene_inputs.get("generation")) is not int
            or epoch.get("schema") != "known_scene_SCAN_epoch_ready/v1"
            or epoch.get("run_id") != node.run.name
            or epoch.get("ready") is not True
            or (
                epoch.get("planner_epoch"),
                epoch.get("planner_identity", {}).get("pid"),
                epoch.get("localization_generation"),
            )
            != tuple(scope)
            or node.known_scene_inputs["generation"] is None
            or epoch.get("generation") != node.known_scene_inputs["generation"]
            or epoch.get("generation") != scope[2]
        ):
            raise ValueError("ordinary_source_current_planner_scope_not_ready")
        # 读取既有源文件并记录发布前 stat；256 KiB 是硬编码大小预算，超限即拒绝。
        path = Path(node.run) / "cascade_command_source.json"
        before_stat = path.stat()
        raw = path.read_bytes()
        if len(raw) > 256 * 1024:
            raise ValueError("original_source_size_budget")
        source = json.loads(raw)
        # 第五组：源字节必须是同一次 ordinary 零事件的 typed 来源，且各项身份
        # （request/goal/path/cascade sequence）与当前节点一致。
        marker = source.get("ordinary_terminal_zero_event")
        if (
            source.get("schema")
            not in (
                "actual_SLAM_SCAN_cascade_command_source/v1",
                "actual_SLAM_terminal_fixed_goal_command_source/v1",
            )
            or source.get("navigation_ground_truth_used") is not False
            or source.get("healthy") is False
            or not exact_zero(source.get("command_after_slew", []))
            or source.get("cascade_mode") != "ordinary_zero_hold"
            or source.get("ordinary_parking_stage") not in ("handoff_zero", "zero_hold")
            or marker != cascade.ordinary_zero_event
            or source.get("request_id") != node.request_id
            or source.get("goal_sha256") != cascade.goal_sha
            or source.get("path_sha256") != cascade.path_sha
            or source.get("cascade_sequence") != node.pid_records
        ):
            raise ValueError("not_same_existing_typed_ordinary_zero_source")
        # 源文件本身没有 epoch 字段，需要通过“这条已准入的物理路径”来绑定；
        # 这样新重启的 planner 不能只靠更新 ready 元组就为旧 ordinary 源背书。
        observed_physical_prediction = False
        # 两个分支：terminal 参考身份直接比对；否则核对 automatic route fence 与
        # 物理路径准入绑定（anchor/physical/identity/receipt 四组哈希与 scope）。
        if getattr(cascade, "terminal_reference", None) is not None:
            from embodied_navigation.control.parking.terminal_owner import validate_context
            from embodied_navigation.control.parking.terminal_reference import (
                validate_fields,
                validate_reference,
            )

            record = validate_context(node)
            if validate_reference(validate_fields(source, node.run.name)) != validate_reference(
                record
            ):
                raise ValueError("duplicate_terminal_reference_identity_changed")
            reference_evidence = dict(
                reference_kind=source["reference_kind"],
                reference_sha256=source["reference_sha256"],
                active_physical_path_id=None,
                active_physical_raw_payload_sha256=None,
                active_physical_reference_file_sha256=None,
                active_physical_planner_scope=None,
                current_terminal_planner_scope=list(scope),
                path_geometry_present=False,
            )
        else:
            active = node.path_admission_active
            receipt = node.path_receipt
            anchor = active["anchor_binding"]
            physical = active["proof"]["r85_physical"]
            physical_binding = physical["source_binding"]
            certificate = physical["physical_certificate"]
            identity = active["identity"]
            runtime = node.automatic_reference
            # 物理证书只接受 CERTIFIED_STATIC_SPACE_ONLY；否则仅记录诊断观察。
            certificate_passed = (
                certificate.get("allowed") is True
                and certificate.get("status") == "CERTIFIED_STATIC_SPACE_ONLY"
            )
            if not certificate_passed:
                from embodied_navigation.common.observation import api as _observer_api

                diagnostic_enabled = _observer_api("diagnostic_observation").enabled
                if diagnostic_enabled(node.profile):
                    from embodied_navigation.planning.admission.physical_gate import (
                        finite_observed_prediction,
                    )

                    observed_physical_prediction = (
                        physical.get("diagnostic_observe_only") is True
                        and physical.get("diagnostic_original_allowed") is False
                        and physical.get("diagnostic_original_reason")
                        == "actual_curve_physical_space_not_certified"
                        and finite_observed_prediction(
                            certificate, physical["physical_payload"], physical_binding
                        )
                    )
            if (
                anchor.get("schema") != "r85_automatic_route_fence_binding/v1"
                or (
                    anchor.get("planner_epoch"),
                    anchor.get("planner_process_id"),
                    anchor.get("localization_generation"),
                )
                != tuple(scope)
                or anchor.get("reference_file_sha256") != runtime.file_sha
                or anchor.get("reference_id") != runtime.doc["reference_id"]
                or (
                    physical_binding.get("planner_epoch"),
                    physical_binding.get("planner_process_id"),
                    physical_binding.get("localization_generation"),
                )
                != tuple(scope)
                or physical_binding.get("reference_file_sha256") != runtime.file_sha
                or physical_binding.get("reference_id") != runtime.doc["reference_id"]
                or physical_binding.get("run_id") != node.run.name
                or physical_binding.get("request_id") != node.request_id
                or physical.get("allowed") is not True
                or not (certificate_passed or observed_physical_prediction)
                or physical.get("raw_payload_sha256") != canonical_sha(active["payload"])
                or physical.get("metadata_sha256") != canonical_sha(active["metadata"])
                or certificate.get("payload_sha256") != canonical_sha(physical["physical_payload"])
                or physical_binding.get("static_sources") != node.r85_physical_gate.static_sources
                or identity.get("request_id") != node.request_id
                or identity.get("waypoint_index") != source.get("waypoint_index")
                or identity.get("waypoint_index") != node.cascade_key[1]
                or receipt.get("path_id") != cascade.path_id
                or canonical_sha(cascade.path_record) != cascade.path_sha
                or receipt.get("request_id") != node.request_id
                or receipt.get("waypoint_index") != source.get("waypoint_index")
                or active.get("trajectory_id") != source.get("trajectory_id")
                or active.get("trajectory_id") != receipt.get("trajectory_id")
                or active.get("trajectory_id") != node.active_trajectory_id
                or tuple(active.get("active_reference_stamp", ()))
                != tuple(node.active_reference_stamp)
                or tuple(active["metadata"].get("reference_stamp", ()))
                != tuple(node.active_reference_stamp)
            ):
                raise ValueError("ordinary_source_not_current_same_physical_admitted_path_scope")
            reference_evidence = dict(
                active_physical_path_id=receipt["path_id"],
                active_physical_raw_payload_sha256=physical["raw_payload_sha256"],
                active_physical_reference_file_sha256=anchor["reference_file_sha256"],
                active_physical_planner_scope=list(scope),
            )
        # 绑定核对可能耗时：之后必须再次确认 known generation 与短重复都未过期。
        now = time.monotonic_ns()
        if not node._known_generation_ready():
            raise ValueError("original_known_source_ready_expired_during_binding_check")
        if not 0 <= now - hold.last_progress_wall_ns < TTL_NS:
            raise ValueError("short_duplicate_expired_during_binding_check")
        for stamp, wall in (
            (node.pose_stamp, node.pose_updated),
            (node.cloud_stamp, node.cloud_updated),
        ):
            if (
                type(stamp) is not int
                or not 0 <= clock_ns - stamp < TTL_NS
                or not isinstance(wall, (int, float))
                or not math.isfinite(wall)
                or not 0 <= now - round(wall * 1e9) < TTL_NS
            ):
                raise ValueError("original_current_pose_or_cloud_expired")
        if not node.raw_imu.fresh(now / 1e9, clock_ns / 1e9):
            raise ValueError("original_current_raw_IMU_expired")
        # 复用原 bridge validator（双 TTL、别名、事件归属与精确 guard 命令绑定），
        # 绝不绕过它。
        validate_controller_source(source, [0.0, 0.0, 0.0], clock_ns, now, node.run.name)
        # 源时间字段必须是整数且非负；cascade_sequence 至少为 1（0 表示没有来源）。
        keys = (
            "source_pose_stamp_ns",
            "paired_imu_stamp_ns",
            "control_stamp_ns",
            "source_pose_received_wall_ns",
            "source_imu_received_wall_ns",
            "cascade_sequence",
        )
        if (
            any(type(source.get(k)) is not int or source[k] < 0 for k in keys)
            or source["cascade_sequence"] < 1
        ):
            raise ValueError("original_source_integer_stamps_required")
        pose, gyro, control = (source[k] for k in keys[:3])
        # 因果与龄期：pose 源龄 < 300 ms；pose 与 IMU 之间 <= 20 ms（硬编码，配对窗）；
        # pose <= control <= 当前控制时钟。
        if not (
            0 <= clock_ns - pose < TTL_NS
            and 0 <= pose - gyro <= 20_000_000
            and pose <= control <= clock_ns
        ):
            raise ValueError("original_source_sim_age_or_causality_expired")
        if not all(0 <= now - source[k] < TTL_NS for k in keys[3:5]):
            raise ValueError("original_source_wall_age_expired")
        for alias, nanos in (
            ("source_pose_received_monotonic_wall", "source_pose_received_wall_ns"),
            ("paired_imu_received_monotonic_wall", "source_imu_received_wall_ns"),
        ):
            # 浮点秒别名必须与整数 ns 值一致，容差 1e-9 s（1 ns）。
            if (
                not isinstance(source.get(alias), (int, float))
                or not math.isfinite(source[alias])
                or abs(source[alias] - source[nanos] / 1e9) > 1e-9
            ):
                raise ValueError("original_source_wall_alias_mismatch")
        # 事件 marker 里的 ready pose / handoff 时钟必须与当前 ordinary owner 一致，
        # 防止把旧事件套到新交接上。
        if (
            marker["ready_pose_stamp_ns"] != ordinary.ready_stamp
            or marker["handoff_clock_ns"] != ordinary.handoff_clock
        ):
            raise ValueError("ordinary_owner_handoff_identity_changed")
        # 诊断观察必须真的被记录；记录失败同样拒绝，避免“观察到了却没留下证据”。
        if observed_physical_prediction or observed_owner_failure:
            from embodied_navigation.common.observation import api as _observer_api

            record = _observer_api("diagnostic_observation").record
            terminal_note = (
                {
                    k: reference_evidence[k]
                    for k in ("reference_kind", "reference_sha256", "path_geometry_present")
                }
                if "reference_kind" in reference_evidence
                else {}
            )
            if (
                record(
                    node.run,
                    node.profile,
                    "ordinary_zero_duplicate_business_prediction",
                    "original_formal_physical_or_parking_business_permission_false",
                    dict(
                        observed_physical_prediction=observed_physical_prediction,
                        observed_owner_failure=observed_owner_failure,
                        original_owner_failure=ordinary.failure,
                        payload_sha256=reference_evidence["active_physical_raw_payload_sha256"],
                        existing_source_bytes_unchanged=True,
                        actual_source_and_zero_event_fresh=True,
                        formal_parking_pass=False,
                        **terminal_note,
                    ),
                )
                is not True
            ):
                raise ValueError("ordinary_duplicate_business_observation_not_recorded")
        # 允许回执：完整记录发布前源文件路径/字节数/inode/mtime/sha 与原始 UTF-8 内容，
        # 供发布后 `observe_unchanged_after_publish` 断言字节未变。
        result.update(
            allowed=True,
            reason="same_fresh_typed_zero_source_bytes_preserved",
            decision=context["decision"],
            protected_or_stale_reset=False,
            source_stale_reasons=[],
            compute_ros_clock_ns=clock_ns,
            compute_monotonic_wall_ns=context["compute_monotonic_wall_ns"],
            source_checked_monotonic_wall_ns=now,
            original_command_before=list(command_before),
            source_file_path=str(path),
            source_file_bytes=len(raw),
            source_file_inode_before=before_stat.st_ino,
            source_file_mtime_ns_before=before_stat.st_mtime_ns,
            source_file_sha256_before=hashlib.sha256(raw).hexdigest(),
            source_original_raw_utf8=raw.decode("utf-8"),
            event_id=marker["event_id"],
            run_id=node.run.name,
            request_id=node.request_id,
            goal_sha256=source["goal_sha256"],
            path_sha256=source["path_sha256"],
            source_original_stamps={k: source[k] for k in keys},
            **reference_evidence,
            source_pose_sim_age_ns=clock_ns - pose,
            source_pose_wall_age_ns=now - source["source_pose_received_wall_ns"],
            source_imu_wall_age_ns=now - source["source_imu_received_wall_ns"],
            original_source_max_age_ns=TTL_NS,
            native_guard_required_before_nonzero_resume=hold.guard_required,
        )
        return result
    # 任何读取/校验失败都返回拒绝回执（reason 固定），不把异常抛给控制路径。
    except (
        OSError,
        ValueError,
        TypeError,
        KeyError,
        AttributeError,
        IndexError,
        OverflowError,
    ) as error:
        return dict(
            result,
            reason="ordinary_duplicate_source_preservation_refused",
            error=type(error).__name__ + ":" + str(error),
        )


def observe_unchanged_after_publish(node, receipt):
    """实际零发布之后的只读断言；源文件字节或元数据变化即 fail closed。

    只在 `receipt["allowed"] is True` 时执行；重新读取源文件，把发布后的 sha/
    inode/mtime/size 写入回执。任意一项变化都抛 `RuntimeError`，表示“保留的零源”
    不再可信。
    """
    if receipt.get("allowed") is not True:
        return receipt
    path = Path(node.run) / "cascade_command_source.json"
    raw = path.read_bytes()
    after_stat = path.stat()
    after = hashlib.sha256(raw).hexdigest()
    receipt.update(
        source_file_sha256_after=after,
        source_bytes_unchanged_after_publish=after == receipt["source_file_sha256_before"],
        source_file_inode_after=after_stat.st_ino,
        source_file_mtime_ns_after=after_stat.st_mtime_ns,
        source_file_metadata_unchanged_after_publish=(
            after_stat.st_ino == receipt["source_file_inode_before"]
            and after_stat.st_mtime_ns == receipt["source_file_mtime_ns_before"]
            and after_stat.st_size == receipt["source_file_bytes"]
        ),
    )
    if (
        not receipt["source_bytes_unchanged_after_publish"]
        or not receipt["source_file_metadata_unchanged_after_publish"]
    ):
        raise RuntimeError("Preserved ordinary zero source bytes changed during actual publication")
    return receipt
