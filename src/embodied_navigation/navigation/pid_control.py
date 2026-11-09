"""``PIDNavigation`` 的速度控制与命令发布。

负责 cascade 的建立与复用、ACK 读取与选择、PID 速度选择、native 几何 guard 的
执行、终端停车所有权与命令发布。基类任务状态机仍在
``base_node.Navigation.control()``；本 mixin 覆写 ``control``，在进入基类状态机
前插入准入、任务保持、时钟保持与终点捕获分支。

边界：不实现控制律数学（core 在 ``control/tracking``），不解析请求或传感器消息
（在 ``pid_ingress``），也不决定路线（在 ``pid_admission``/``pid_route``）。

命令语义：``publish_command()`` 不带速度 = 零速停车并撤销命令源；带速度 = 一次
正常发布，且必须与 native guard 之后准备好的输出一致，否则宁可不发并报错。
"""

from .pid_support import (
    constrain_exactzero_launch,
    np,
    time,
    _r86_probe_guard_begin,
    json,
    ordinary_parking_enabled,
    _r47_guard_dimensions,
    _copy_path_receipt_metadata,
    _r47_publication_anchor,
    Controller,
    StableProjectionController,
    _pre_turn_external_hold_seed_wait,
    _append_pid_row_evidence,
    _pid_receipt_json_bytes,
    retain_native_context,
    _sensor_hold_end,
    _r47_bind,
    _r47_end,
    make_event,
    _r86_probe_guard_end,
    _r118_source_age,
    OrderedDict,
    slew,
    _r67_attach_known_frame_failure,
    _r96_cpu,
    CausalHandoffProjectionController,
    os,
    _r105_terminal,
    _r47_begin,
    remember_native_guard,
    diagnostic_profile_view,
    _callback_cost_observe,
    copy,
    commit_pending_path,
    minimal_baseline_enabled,
    _sensor_hold_begin,
    _sensor_parking_protection,
    _r47_mark,
)


class ControlMixin:
    """级联/速度选择、ACK 门控、停车、发布与终端保护的 mixin。"""

    def ensure_cascade(self):
        """按 ``(request_id, waypoint_index)`` 建立或复用本段的 cascade 控制器。

        控制器类型是硬编码的三选一：连续路线启用时用
        ``CausalHandoffProjectionController``，否则全局参考启用时用
        ``StableProjectionController``，否则 ``Controller``；配置取自 run profile 的
        ``cascade`` 节（由 prepare 生成并 pin，见 navigation/README.md 配置节）。
        段切换（key 变化）时清空本段缓存并写固定目标收据；同一段内只在收到新
        ``path_id`` 且尚未捕获路径时装入路径。副作用是追加
        ``navigation_fixed_goal_references.jsonl``/``navigation_cascade_paths.jsonl``
        证据（默认 writer 下为空操作）。
        """
        if getattr(self, "_r105_terminal_context", None) is not None:
            _r105_terminal.validate_context(self)
            self.cascade.diagnostic_observe_only = dict(
                run=self.run, profile=diagnostic_profile_view(self.profile)
            )
            return
        key = (self.request_id, self.waypoint_index)
        if self.cascade_key != key:
            self.reference_replan_pending = False
            self.pid_prepared_output = None
            self.pid_row = None
            self._pid_row_receipt_snapshot = None
            goal = self.waypoints[self.waypoint_index]
            yaw = self.fixed_terminal_heading(self.waypoint_index, self.segment_start)
            core = (
                CausalHandoffProjectionController
                if self.continuous_route is not None
                else StableProjectionController
                if getattr(self, "global_reference_settings", {"enabled": False})["enabled"]
                else Controller
            )
            self.cascade = core(
                self.profile["cascade"], goal.tolist(), yaw, ":".join(map(str, key))
            )
            self.cascade.minimal_simulation_baseline = minimal_baseline_enabled(self.profile)
            if "r85_neutral_rate_brake_capture" in self.profile:
                self.cascade.configure_neutral_rate_brake_capture(self.profile)
            if "r85_teacher_pre_zero_heading_p_taper" in self.profile:
                self.cascade.configure_teacher_pre_zero_heading_p_taper(
                    self.profile, self.run.name, self.handoff_scope_sha256
                )
            self.reference_consistency_hold = False
            self.reference_consistency_check = None
            self.cascade_key = key
            self.cascade_path_id = None
            self.evidence.append(
                self.run / "navigation_fixed_goal_references.jsonl",
                dict(
                    request_id=self.request_id,
                    waypoint_index=self.waypoint_index,
                    segment_start=self.segment_start.tolist(),
                    goal=goal.tolist(),
                    heading_rad=yaw,
                    source_pose_stamp_ns=self.segment_references[key]["source_pose_stamp_ns"],
                    control_stamp_ns=self.segment_references[key]["control_stamp_ns"],
                    goals_definition_sha256=self.goals_definition_sha256,
                    fixed_goal=self.cascade.goal_record,
                    fixed_goal_sha256=self.cascade.goal_sha,
                    navigation_ground_truth_used=False,
                ),
            )
        if (
            self.cascade.capture_stamp is None
            and self.path_receipt
            and (self.path_receipt["request_id"], self.path_receipt["waypoint_index"]) == key
            and self.path_receipt["path_id"] != self.cascade_path_id
        ):
            r = self.path_receipt
            self.cascade.set_path(
                r["points_xyz"],
                r["path_id"],
                r["stamp_ns"],
                r["received_wall_ns"],
                r["frame_id"],
                position=self.pose,
            )
            self.cascade_path_id = r["path_id"]
            self.evidence.append(
                self.run / "navigation_cascade_paths.jsonl",
                dict(
                    **self.cascade.path_record,
                    path_sha256=self.cascade.path_sha,
                    trajectory_id=r["trajectory_id"],
                    array_file=r["array_file"],
                    source_sample_indices=r["source_sample_indices"],
                    source_samples_count=r["source_samples_count"],
                    source_samples_float64_sha256=r["source_samples_float64_sha256"],
                    points_float64_sha256=r["points_float64_sha256"],
                    navigation_ground_truth_used=False,
                ),
            )
        if self.cascade is not None:
            self.cascade.diagnostic_observe_only = dict(
                run=self.run, profile=diagnostic_profile_view(self.profile)
            )

    def _decode_ack_entries_reusing_lines(self, raw):
        """按行复用已解析的 ACK JSON，避免每个 tick 重解析整份文件。

        只缓存解析树：因果选择、新鲜度与选中行的深拷贝仍在 ``get_ack`` 每次执行。
        输入必须是本模块约定的固定前缀/后缀格式；任何不符合（超过 32 行、行内换行、
        非 dict）都回退到整文件 ``json.loads``。行缓存最多 32 项、按完整行字节索引并
        按 LRU 淘汰；新文件版本中字节完全相同的行会复用已解析的 row，因此未变的行
        可以跨文件版本复用。
        """
        prefix = b'{"entries":['
        suffix = b'],"schema":"teacher_known_velocity_ack/v1"}\n'
        if not (raw.startswith(prefix) and raw.endswith(suffix)):
            return json.loads(raw)["entries"]
        body = raw[len(prefix) : -len(suffix)]
        if not body:
            return []
        lines = body.split(b",\n", 32)
        if len(lines) > 32 or any(
            not (line.startswith(b"{") and line.endswith(b"}")) or b"\n" in line or b"\r" in line
            for line in lines
        ):
            return json.loads(raw)["entries"]
        cache = getattr(self, "_ack_parsed_line_lru", None)
        if cache is None:
            cache = OrderedDict()
        entries = []
        try:
            for line in lines:
                if line in cache:
                    row = cache[line]
                    cache.move_to_end(line)
                else:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        return json.loads(raw)["entries"]
                    cache[line] = row
                    if len(cache) > 32:
                        cache.popitem(last=False)
                entries.append(row)
        except (ValueError, TypeError, RecursionError):
            return json.loads(raw)["entries"]
        self._ack_parsed_line_lru = cache
        return entries

    def _read_ack_entries_cached(self, path):
        """读取 ACK 文件，并按 ``(inode, mtime_ns, size)`` 缓存解析结果。

        缓存只省解析，不省新鲜度判定；每次 ``get_ack`` 仍用自己的真实位姿 header 选择。
        读取时在同一打开 fd 上比较读取前后的元数据，并核对 raw 长度与读取后 size 一致，
        变化或长度不符即报错；通过后缓存整份文件的解析结果。该检查只覆盖同一 fd 的读取
        过程：路径被原子替换不会改变已打开的 fd，因此不声称能检测所有替换或并发变化。
        条目类型必须是 list。
        """
        def key(stat):
            return (stat.st_ino, stat.st_mtime_ns, stat.st_size)

        observed = key(path.stat())
        if getattr(self, "_ack_entries_cache_key", None) == observed and hasattr(
            self, "_ack_entries_cache"
        ):
            return self._ack_entries_cache
        with path.open("rb") as stream:
            before = key(os.fstat(stream.fileno()))
            raw = stream.read()
            after = key(os.fstat(stream.fileno()))
        if before != after or len(raw) != after[2]:
            raise ValueError("Actual ACK changed during same-FD read")
        entries = self._decode_ack_entries_reusing_lines(raw)
        if not isinstance(entries, list):
            raise ValueError("Actual ACK entries must be a list")
        self._ack_entries_cache_key = after
        self._ack_entries_cache = entries
        return entries

    @_r96_cpu.observe("ack.get_ack")
    def get_ack(self, pose_stamp_ns=None):
        """选择与给定位姿时间戳因果一致的最新一条执行 ACK；失败返回 ``None``。

        ``pose_stamp_ns`` 缺省取当前 ``feedback["stamp_ns"]``；没有反馈时也返回
        ``None``。ACK 文件是 run 目录的 ``closed_loop_executor_ack.json``（由
        ``robot/unitree/policy/worker.py`` 写入，navigation/README.md:85），schema 为
        ``teacher_known_velocity_ack/v1``。选中条件：条目 ``stamp_ns <= pose_stamp_ns``；
        若 profile 配置了 ``r85_student_lower_loop_ack``，还要求 ``run_id`` 等于本次
        交接作用域 ``handoff_scope_sha256``。文件缺失或格式错误一律返回 ``None``，
        由调用方转成保护停车，不在这里重试。返回深拷贝，调用方改不到缓存。
        """
        if pose_stamp_ns is None:
            if not self.feedback:
                return None
            pose_stamp_ns = self.feedback["stamp_ns"]
        try:
            entries = self._read_ack_entries_cached(self.run / "closed_loop_executor_ack.json")
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                configuration as lower_ack_configuration,
            )

            if (
                lower_ack_configuration(self.profile["cascade"].get("r85_student_lower_loop_ack"))
                is not None
            ):
                expected_scope = self.handoff_scope_sha256
                selected = next(
                    (
                        r
                        for r in reversed(entries)
                        if r["stamp_ns"] <= pose_stamp_ns and r.get("run_id") == expected_scope
                    ),
                    None,
                )
            else:
                selected = next(
                    (r for r in reversed(entries) if r["stamp_ns"] <= pose_stamp_ns), None
                )
            # 缓存解析树保持私有；调用方只拿到这一小份 ACK 深拷贝。
            return copy.deepcopy(selected)
        except (OSError, KeyError, ValueError, TypeError):
            return None

    @_r96_cpu.observe("select")
    def select_pid_velocity(
        self, velocity, yaw_rate, target, allow_translation, mode_override=None
    ):
        """计算一次身体系速度命令，并写入本 tick 的 PID 收据行。

        参数：``velocity``/``yaw_rate`` 是任务状态机给的期望（body 系，m/s 与
        rad/s）；``target`` 是当前检查目标；``allow_translation`` 决定能否平移；
        ``mode_override`` 用于停车/保持等强制模式。返回 ``(vx, vy), yaw_rate``。
        模式优先级硬编码为：hold（障碍/参考一致性/转向预段/未主动制动的 settle）>
        brake（受控制动）> drive（允许平移）> turn；交接失败会强制 hold 并把
        ``state`` 置 ``failed``。
        缺 ``feedback``/``paired_imu``/ACK 时以
        ``actual_causal_feedback_or_ack_missing`` 转保护；``failure_latched`` 同样置
        failed。最后按硬编码的每轴加速度上限 [0.6, 0.6, 0.8] 调 ``slew``（m/s²、
        m/s²、rad/s²）；停车/保持模式直接给零，``ordinary_zero_hold`` 再强制精确零。
        真正的发布由调用方完成。
        """
        if _r96_cpu.PROBE is None:
            ns = self.get_clock().now().nanoseconds
            wall = time.monotonic_ns()
        else:
            with _r96_cpu.span("select.ros_clock", self):
                ns = self.get_clock().now().nanoseconds
                wall = time.monotonic_ns()
        if _r118_source_age.ENABLED:
            _r118_source_age.controller(self, "select.enter", ns, feedback=True)
        _r47_call = _r47_begin(self, "select", ns, wall)
        try:
            _r47_mark(self, _r47_call, "ensure_cascade_before")
            self.ensure_cascade()
            _r47_mark(self, _r47_call, "ensure_cascade_after")
            controlled_settle = bool(getattr(self.heading_gate, "controlled_settle", False))
            controlled_braking = bool(getattr(self.heading_gate, "controlled_braking", False))
            active_settle = controlled_settle or controlled_braking
            # 模式优先级：障碍/参考一致性/转向预段 hold > 受控制动 brake > 允许平移
            # drive > turn（硬编码，不来自 profile）。
            mode = mode_override or (
                "hold"
                if self.obstacle_hold
                or getattr(self, "reference_consistency_hold", False)
                or self.heading_gate.phase == "pre_turn"
                or self.heading_gate.phase == "settle"
                and not active_settle
                else "brake"
                if controlled_braking
                else "drive"
                if allow_translation
                else "turn"
            )
            handoff_failure = getattr(self.heading_gate, "handoff_failure", None)
            if handoff_failure:
                self.state = "failed"
                self.message = "主动转向交接失败：" + str(handoff_failure)
                mode = "hold"
            heading_reference = copy.deepcopy(self.steering)
            gate_heading = self.heading_gate.heading
            if (
                self.imu_source_buffer is not None
                and self.feedback is not None
                and self.paired_imu is None
            ):
                paired = self.imu_source_buffer.pair(self.feedback["stamp_ns"], None, ns)
                if paired is not None:
                    self.paired_imu = copy.deepcopy(paired)
                    self.imu_source_buffer.event(
                        self.feedback["stamp_ns"], ns, wall, "current_pose_pair_from_fifo", paired
                    )
            _r47_mark(self, _r47_call, "get_ack_before")
            ack = self.get_ack()
            _r47_mark(self, _r47_call, "get_ack_after")
            guard = None
            # 缺当前反馈、缺配对 IMU 或缺 ACK 都表示本 tick 没有可用的因果输入。
            if self.feedback is None or self.paired_imu is None or ack is None:
                guard = "actual_causal_feedback_or_ack_missing"
            if handoff_failure:
                guard = "active_turn_handoff_failure:" + str(handoff_failure)
            context = (self.request_id, self.waypoint_index)
            previous = getattr(self, "_pre_turn_hold_seed_context", None)
            if previous is not None:
                previous = dict(
                    previous,
                    uninterrupted=bool(
                        self.cascade.protected
                        and self.cascade.protection_reason == "external_hold"
                        and not self.cascade.recovery
                        and self.cascade.last_math_stamp is None
                        and self.cascade.last_clock == previous["clock_ns"]
                        and self.cascade.last_wall == previous["wall_ns"]
                    ),
                )
            unload_event = None
            if (
                getattr(self.heading_gate, "integral_unload_config", None) is not None
                and mode == "brake"
                and guard is None
            ):
                unload_event = self.heading_gate.bind_integral_unload(
                    self.run.name,
                    self.handoff_scope_sha256,
                    self.cascade.path_sha,
                    self.cascade.goal_sha,
                    ns,
                )
                if self.heading_gate.handoff_failure:
                    guard = "active_turn_handoff_failure:" + self.heading_gate.handoff_failure
            update_args = dict(
                guard_reason=guard,
                mode_override=mode,
                external_turn_heading=gate_heading
                if mode in ("turn", "brake")
                and (self.heading_gate.phase == "align" or active_settle)
                else None,
            )
            if unload_event is not None:
                update_args["pre_zero_unload_event"] = unload_event
            _r47_mark(self, _r47_call, "core_update_before")
            cmd, diagnostic = self.cascade.update(
                self.feedback, self.paired_imu, ack, ns, wall, **update_args
            )
            _r47_mark(self, _r47_call, "core_update_after")
            if (
                diagnostic.get("controller_updated")
                and diagnostic.get("kind") == "ordinary_preT0_recapture"
            ):
                self.evidence.append(
                    self.run / "ordinary_preT0_attempts.jsonl",
                    diagnostic["preT0_attempt_disqualified"],
                )
            seed_wait = _pre_turn_external_hold_seed_wait(
                previous,
                self.cascade,
                self.heading_gate,
                context,
                mode,
                guard,
                diagnostic,
                cmd,
                ns,
                wall,
            )
            # 任何非 pre-turn 调用都会消费掉这个种子令牌；重复调用或另一种保护不能
            # 复用同一次“单一有效来源”的例外。
            self._pre_turn_hold_seed_context = (
                dict(
                    cascade=self.cascade,
                    context=context,
                    clock_ns=ns,
                    wall_ns=wall,
                    gate_context=copy.deepcopy(self.heading_gate.transition_context),
                    purpose=self.heading_gate.transition_purpose,
                )
                if mode == "hold"
                and self.heading_gate.phase == "pre_turn"
                and guard is None
                and not getattr(self.heading_gate, "handoff_active", False)
                and diagnostic.get("mode") == "protect"
                and diagnostic.get("reason") == "external_hold"
                and diagnostic.get("failure_latched") is None
                else None
            )
            if seed_wait:
                diagnostic["handoff_seed_recovery_wait"] = True
            if (
                getattr(self.heading_gate, "handoff_active", False)
                and (
                    guard is not None
                    or diagnostic.get("mode") in ("protect", "recovering")
                    and not (mode == "hold" and diagnostic.get("reason") == "external_hold")
                )
                and not seed_wait
            ):
                self.heading_gate.fail_handoff("cascade_source_or_control_protection", ns / 1e9)
                self.state = "failed"
                self.message = "主动转向交接失败：" + str(self.heading_gate.handoff_failure)
            if diagnostic.get("failure_latched"):
                self.state = "failed"
                self.message = "串级真实输入保护锁定：" + str(diagnostic["failure_latched"])
            self.cascade_last_row = diagnostic
            self.pid_guard_needs_evaluation = (
                bool(diagnostic.get("controller_updated")) or self.control_clock_hold.guard_required
            )
            self.reference_replan_pending = (
                getattr(self, "reference_consistency_hold", False)
                or diagnostic["mode"] == "reference_constraint_hold"
            )
            _r47_mark(self, _r47_call, "postcore_protection_logic_complete")
            desired = cmd.copy()
            # 只采样进入最终 slew 决策前已经流逝的时间；之后 native guard 的耗时既
            # 不预测也不计入。每轴上限 [0.6, 0.6, 0.8] 为硬编码（m/s²、m/s²、rad/s²），
            # slew 内部把 dt 截断到 [0, 0.1] s。
            cmd = (
                np.zeros(3)
                if diagnostic["mode"]
                in (
                    "protect",
                    "recovering",
                    "pre_turn",
                    "settle",
                    "path_end_hold",
                    "reference_constraint_hold",
                    "ordinary_zero_hold",
                )
                else slew(
                    self.command,
                    desired,
                    self.get_clock().now().nanoseconds / 1e9 - self.last_command_time,
                    [0.6, 0.6, 0.8],
                )
            )
            if diagnostic["mode"] == "ordinary_zero_hold":
                # 普通终端零速：直接给精确零，不再走 launch 约束。
                cmd[:] = 0.0
                launch_constraint = dict(
                    reason="identity_bound_ordinary_terminal_zero", exact_body_zero=True
                )
            else:
                cmd, launch_constraint = constrain_exactzero_launch(
                    self.command, desired, cmd, diagnostic["mode"], self.rotation[:2, :2]
                )
            self.pid_prepared_output = cmd.copy()
            world_direction = self.rotation[:2, :2] @ cmd[:2]
            if np.linalg.norm(world_direction) > 1e-9:
                self.steering["direction"] = world_direction.tolist()
            self.pid_records += 1
            pid_receipt_bytes = _pid_receipt_json_bytes(self)
            self.pid_row = dict(
                schema="teacher_closed_loop_cascade_tick/v1",
                sequence=self.pid_records,
                cascade_sequence=self.pid_records,
                control_pose_stamp_ns=int(self.pose_stamp),
                source_pose_stamp_ns=int(self.pose_stamp),
                paired_imu_stamp_ns=None
                if self.paired_imu is None
                else self.paired_imu["stamp_ns"],
                control_stamp_ns=ns,
                compute_ros_clock_ns=ns,
                compute_monotonic_wall=wall / 1e9,
                feedback=copy.deepcopy(self.feedback),
                imu=copy.deepcopy(self.paired_imu),
                ack=copy.deepcopy(ack),
                heading_gate_reference=dict(
                    phase=self.heading_gate.phase,
                    locked_heading=gate_heading,
                    steering=heading_reference,
                    controlled_settle=controlled_settle,
                    controlled_braking=controlled_braking,
                    settle_control_stage=getattr(self.heading_gate, "settle_control_stage", None),
                    zero_handoff_count=getattr(self.heading_gate, "zero_handoff_count", None),
                    last_zero_handoff=copy.deepcopy(
                        getattr(self.heading_gate, "last_zero_handoff", None)
                    ),
                ),
                path_receipt=None
                if minimal_baseline_enabled(self.profile)
                or pid_receipt_bytes is not None
                or self.path_receipt is None
                else {
                    k: (_copy_path_receipt_metadata(v) if k == "metadata" else copy.deepcopy(v))
                    for k, v in self.path_receipt.items()
                    if k != "points_xyz"
                },
                cascade=diagnostic,
                control_pose=self.pose.tolist(),
                control_quaternion=self.pose_quaternion,
                control_rotation=self.rotation.tolist(),
                request_id=self.request_id,
                waypoint_index=self.waypoint_index,
                trajectory_id=self.active_trajectory_id,
                trajectory_archive_file=self.trajectory_archive_reference,
                mode=diagnostic["mode"],
                checked_target=np.asarray(target).tolist(),
                goal=self.waypoints[self.waypoint_index].tolist(),
                actual_pid_world_direction=world_direction.tolist(),
                steering_direction=self.steering["direction"],
                desired_body_command=desired.tolist(),
                prepared_after_slew_command=cmd.tolist(),
                launch_output_constraint=launch_constraint,
                navigation_ground_truth_used=False,
            )
            self._pid_row_receipt_snapshot = (
                (self.pid_row, pid_receipt_bytes) if pid_receipt_bytes is not None else None
            )
            if getattr(self.heading_gate, "quiet_target_enabled", False):
                self.pid_row["quiet_handoff_binding"] = self.heading_gate._quiet_binding(
                    self.heading_gate.transition_context,
                    self.heading_gate.transition_purpose,
                    self.handoff_scope_sha256,
                    diagnostic.get("path_id"),
                    diagnostic.get("path_sha256"),
                    diagnostic.get("fixed_goal_sha256"),
                )
            _r47_bind(self, _r47_call, self.pid_row)
            _r47_mark(self, _r47_call, "postcore_row_complete")
            if _r118_source_age.ENABLED:
                _r118_source_age.controller(self, "select.row_complete", ns, self.pid_row)
            return cmd[:2], float(cmd[2])
        finally:
            _r47_end(self, _r47_call)

    def evaluate_motion_guard(self, *args):
        """执行 native 几何 guard，并把结果挂到本 tick 的 PID/guard 收据上。

        ``*args`` 原样交给 ``record_native_guard``（实测点云、位姿、目标、航向与
        路线）；本方法只加观测包裹：默认关闭的慢 guard 窗口探针、保持原生上下文、
        记住 guard 结果，并在 ``pending_guard_row`` 中回填点云归档与 PID 序号。
        返回 ``record_native_guard`` 的原始结果；副作用是清空
        ``pid_guard_needs_evaluation`` 并通知 ``control_clock_hold`` 本轮 guard
        已完成。
        """
        _r47_call = _r47_begin(self, "guard")
        _r47_bind(self, _r47_call, getattr(self, "pid_row", None))
        _r47_guard_dimensions(self, _r47_call, args)
        try:
            _r47_mark(self, _r47_call, "native_geometry_guard_before")
            # 默认关闭的慢 guard 窗口：两端采集每线程 CPU/调度与 gc 差值，只有 guard
            # 运行过久时才抓 C watchdog 栈块；guard 调用本身不变。
            _guard_window = _r86_probe_guard_begin()
            try:
                result = self.record_native_guard(*args)
            finally:
                _r86_probe_guard_end(_guard_window, _r47_call)
            _r47_mark(self, _r47_call, "native_geometry_guard_after")
            _r47_mark(self, _r47_call, "retain_native_context_before")
            retain_native_context(self, args)
            _r47_mark(self, _r47_call, "retain_native_context_after")
            _r47_mark(self, _r47_call, "remember_native_guard_before")
            remember_native_guard(self)
            _r47_mark(self, _r47_call, "remember_native_guard_after")
            if self.pending_guard_row is not None:
                self.pending_guard_row.update(
                    cloud_array_file=self.guard_cloud_receipt["cloud_array_file"],
                    cloud_raw_payload_file=self.guard_cloud_receipt["cloud_raw_payload_file"],
                    cloud_archive_sequence=self.guard_cloud_receipt["cloud_archive_sequence"],
                )
                if self.pid_row:
                    self.pending_guard_row.update(
                        pid_sequence=self.pid_row["sequence"],
                        cascade_sequence=self.pid_row["sequence"],
                        pid_compute_ros_clock_ns=self.pid_row["compute_ros_clock_ns"],
                        pid_control_pose_stamp_ns=self.pid_row["control_pose_stamp_ns"],
                    )
            self.pid_guard_needs_evaluation = False
            self.control_clock_hold.guard_completed()
            _r47_mark(self, _r47_call, "guard_row_complete")
            return result
        finally:
            _r47_end(self, _r47_call)

    def observe_ordinary_terminal_protection(self, ns):
        """发布零速时检查终端停车所有者的失败，并锁存本任务的失败状态。

        只在 ``terminal_parking_owner`` 属于当前 request、且已就绪
        （``ready_stamp`` 非空）时生效。失败原因优先取 owner 自身 ``failure``；否则
        依次检查已知场景来源/epoch、动态障碍 hold、倾角 hold 与 ``state == failed``。
        若 ``mission46_status.json`` 显示本 run 已 completed 且 final_parking_verified，
        则不再用回退原因把完成态判成失败（owner 自报的 failure 仍会锁存）。开启
        诊断观测且失败可被观测时，先验证来源与 ACK 全部新鲜
        （位姿/点云/桥接 0.3 s 墙钟窗、ACK 300 ms 内且不晚于位姿）再记诊断；不满足时
        改记实际来源保护。最终调用 ``owner.guard(reason)``，把 ``state`` 置 failed，并
        只锁存一次 ``ordinary_terminal_failure_history.jsonl``。
        """
        owner = getattr(self, "terminal_parking_owner", None)
        if (
            owner is None
            or getattr(self, "terminal_parking_request", None) != self.request_id
            or owner.ready_stamp is None
        ):
            return
        reason = owner.failure
        # 正常完成是独立的生命周期边界；即使之后有正式收据，已有的物理/控制失败
        # 仍要记录。
        completed = False
        try:
            final = json.loads((self.run / "mission46_status.json").read_text())
            completed = (
                final.get("run_id") == self.run.name
                and final.get("stage") == "completed"
                and final.get("final_parking_verified") is True
            )
        except (OSError, ValueError, TypeError):
            pass
        if reason is None and not completed:
            if hasattr(self, "_known_generation_ready") and not self._known_generation_ready():
                reason = "known_scene_source_or_epoch_protection"
            elif self.obstacle_hold:
                reason = "dynamic_obstacle_protection"
            elif getattr(self, "tilt_hold", False):
                reason = "tilt_protection"
            elif self.state == "failed":
                reason = "terminal_controller_failure"
        if reason is not None:
            from embodied_navigation.common.observation import api as _observer_api

            diagnostic_enabled = _observer_api("diagnostic_observation").enabled
            diagnostic_record = _observer_api("diagnostic_observation").record
            observed_business = False
            if diagnostic_enabled(self.profile):
                from embodied_navigation.control.parking.ordinary_parking import (
                    diagnostic_business_failure,
                )

                observed_business = (
                    diagnostic_business_failure(reason)
                    and owner.diagnostic_failure_is_observed(
                        dict(run=self.run, profile=diagnostic_profile_view(self.profile))
                    )
                    and getattr(self, "ordinary_terminal_failure_latched", None) is None
                )
            if observed_business:
                # 诊断观测只在来源与 ACK 全部新鲜时成立：位姿/点云/桥接在 0.3 s
                # 墙钟内、原始 IMU 新鲜、ACK 在 300 ms 内且不晚于当前位姿。
                now = time.monotonic()
                source_ready = False
                try:
                    from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                        input_binding,
                    )

                    ack = self.get_ack()
                    input_binding(ack)
                    source_ready = bool(
                        self._known_generation_ready()
                        and not self.obstacle_hold
                        and not getattr(self, "tilt_hold", False)
                        and self.state != "failed"
                        and self.raw_imu.fresh(now, ns / 1e9)
                        and all(
                            0 <= now - stamp < 0.3
                            for stamp in (
                                self.pose_updated,
                                self.cloud_updated,
                                self.bridge_updated,
                            )
                        )
                        and self.bridge_safety.get("state") == "ready"
                        and type(ack["stamp_ns"]) is int
                        and 0 <= ns - ack["stamp_ns"] < 300_000_000
                        and ack["stamp_ns"] <= self.pose_stamp
                        and 0 <= now - ack["received_wall_ns"] / 1e9 < 0.3
                    )
                except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
                    source_ready = False
                if source_ready:
                    if (
                        diagnostic_record(
                            self.run,
                            self.profile,
                            "ordinary_terminal_business_failure_latch",
                            reason,
                            dict(
                                control_stamp_ns=int(ns),
                                request_id=self.request_id,
                                original_business_failure_retained=True,
                                original_owner_failure=owner.failure,
                                actual_source_and_ACK_fresh=True,
                                normal_control_and_zero_phases_unchanged=True,
                                formal_parking_pass=False,
                            ),
                        )
                        is not True
                    ):
                        raise RuntimeError("ordinary_terminal_business_observation_not_recorded")
                    return
                reason = "actual_source_or_ACK_protection_during_observed_parking_business_failure"
            owner.guard(reason)
            # 失败原因只锁存一次；锁存后本任务的后续 tick 都直接按 failed 发零速。
            if getattr(self, "ordinary_terminal_failure_latched", None) is None:
                self.ordinary_terminal_failure_latched = dict(
                    request_id=self.request_id,
                    reason=owner.failure,
                    clock_ns=ns,
                    ready_pose_stamp_ns=owner.ready_stamp,
                    ordinary_parking=owner.evidence(),
                    navigation_ground_truth_used=False,
                )
                self.evidence.append(
                    self.run / "ordinary_terminal_failure_history.jsonl",
                    self.ordinary_terminal_failure_latched,
                )
            self.state = "failed"
            self.message = "普通停车失败锁定：" + str(owner.failure)

    @_callback_cost_observe("control")
    @_r96_cpu.observe("embodied_navigation.navigation.pid_navigation")
    def control(self, now):
        """每个控制 tick 的入口：先处理已锁定的停车失败与主动交接，再进入候选控制。

        已锁定普通停车失败时直接发零速并保持 ``failed``；交接超时或失败同样置
        failed 并发布停车。候选控制抛出的异常在此转成 failed + 零速命令 + 错误证据，
        不让异常逃到 executor 去依赖下游的 300 ms watchdog。
        """
        failed = getattr(self, "ordinary_terminal_failure_latched", None)
        if failed is not None and failed.get("request_id") == self.request_id:
            self.state = "failed"
            self.publish_command()
            return
        gate = self.heading_gate
        clock_ns = self.get_clock().now().nanoseconds
        if getattr(gate, "integral_unload_config", None) is not None:
            gate.check_integral_unload_deadline(clock_ns)
        if (
            getattr(gate, "handoff_active", False)
            and gate.handoff_deadline_s is not None
            and clock_ns / 1e9 > gate.handoff_deadline_s
        ):
            gate.fail_handoff("active_handoff_deadline_exceeded", clock_ns / 1e9)
        if getattr(gate, "handoff_failure", None):
            self.state = "failed"
            self.message = "主动转向交接失败：" + str(gate.handoff_failure)
            self.publish_command()
            return
        try:
            return self._control_candidate(now)
        except Exception as error:
            # 参考/交接异常必须在本 tick 内停车，不能逃出 executor 只依赖下游
            # 300 ms watchdog。
            self.state = "failed"
            self.message = "V33控制参考异常，停车：" + type(error).__name__ + ": " + str(error)
            self.reference_replan_pending = False
            self.publish_command()
            self.evidence.append(
                self.run / "navigation_controller_errors.jsonl",
                dict(
                    error_type=type(error).__name__,
                    error=str(error),
                    command_after_stop=list(self.command),
                    control_stamp_ns=self.get_clock().now().nanoseconds,
                    source_pose_stamp_ns=int(self.pose_stamp),
                    navigation_ground_truth_used=False,
                ),
            )

    @_r96_cpu.observe("embodied_navigation.control.candidate")
    def _control_candidate(self, now):
        """按固定顺序推进一个 tick：走廊 → 时钟保持 → 终端 → 准入 → 任务保持 → 基类状态机。

        顺序即安全语义：``control_clock_hold`` 在同一 ROS clock 重复 tick 时先发精确
        零，且要求新的实测几何检查后才能恢复非零命令；``_r105_terminal`` 与
        ``path_admission_hold`` 可以各自先接管；``mission46_required`` 时按 mission
        保持发零速（``succeeded`` 且已有 final_bundle 则转入停车）。最后才调用
        ``super().control(now)``（``base_node`` 的任务状态机）。
        """
        self.poll_corridor(now)
        # 真实 ROS /clock 未前进时，墙钟定时器可能在一个时钟周期内跑两次。这里在
        # 任何 PI、航向、到点或几何运算之前发精确零，并不改动原始来源收据；只有
        # 时钟前进后重新做一次实测几何检查，才允许恢复非零命令。
        if self.control_clock_hold.before_control(
            self, self.get_clock().now().nanoseconds, round(now * 1e9)
        ):
            return
        if _r105_terminal.control(self, now):
            return
        commit_pending_path(self)
        self.request_admission_replan_if_due(now)
        if self.path_admission_hold:
            if self.interrupt_active_handoff("active_handoff_path_admission_hold"):
                self.publish_command()
                return
            return self.control_admission_hold(now)
        if self.profile.get("mission46_required"):
            from embodied_navigation.navigation.mission.mission46_guard import mission_hold

            hold, reason = mission_hold(
                self.run,
                self.request_id,
                self.region_arrivals,
                self.goals_definition_sha256,
                self.get_clock().now().nanoseconds,
                now,
            )
            if hold:
                if self.interrupt_active_handoff("active_handoff_mission_hold"):
                    self.publish_command()
                    return
                self.message = "Teacher46 transition: " + reason
                # 已到达相位终点时保留既有反馈保持；连接段暂停是向现役 Teacher
                # 发零速度请求，绝不是零动作或另起一个关节控制器。
                if self.state == "succeeded" and self.final_bundle is not None:
                    return self.control_parking(now)
                self.apply_tilt_guard(now, self.get_clock().now().nanoseconds / 1e9)
                self.publish_command()
                return
        self.continuous_release_target()
        if self.state == "succeeded" and self.final_bundle is not None:
            return self.control_parking(now)
        # 末航点且采样路径已就绪时，先冻结最终路径收据，停车路径随后按它重建目标。
        if (
            self.state == "running"
            and self.samples is not None
            and self.waypoint_index == len(self.waypoints) - 1
            and self.segment_start is not None
        ):
            self.ensure_cascade()
            self.final_bundle = self.final_bundle_reference()
        return super().control(now)

    def control_parking(self, now):
        """终点捕获/停车：按最终路径收据重建目标，再由实测点云 guard 决定走或停。

        进入时把 ``waypoint_index`` 临时切到 ``final_bundle["index"]``，并恢复其参考
        时间戳、轨迹与路径；``ordinary_parking_enabled(profile)`` 时把终端停车所有权
        交给 cascade 的 ordinary parking。来源新鲜度阈值硬编码为 0.3 s（位姿/点云
        墙钟），任一项 stale、缺配对 IMU、倾角保护或障碍 hold 都只发 hold 零速。
        native 几何 guard 报保护时立即发零速停车、把受检候选归档，再以 hold 模式选一次
        零命令；否则发布捕获命令。``finally`` 恢复原 ``waypoint_index`` 并结束本 tick 的
        传感器保持记录。
        """
        ns = self.get_clock().now().nanoseconds
        bundle = self.final_bundle
        index = self.waypoint_index
        # 用 final_bundle 的索引、参考时间戳与路径临时切换本 tick 的目标；finally 恢复。
        self.waypoint_index = bundle["index"]
        self.active_reference_stamp = bundle["reference_stamp"]
        self.active_trajectory_id = bundle["trajectory_id"]
        self.trajectory_archive_reference = bundle["array"]
        self.path_receipt = bundle["path_receipt"]
        self.segment_start = bundle["start"]
        goal = self.waypoints[self.waypoint_index]
        if ordinary_parking_enabled(self.profile):
            # 普通停车启用时，把终端停车所有权交给 cascade 的 ordinary parking，
            # 并登记到当前 request；收据由 publish_command 侧的观察路径读取。
            self.cascade.begin_ordinary_terminal(
                int(ns), round(float(self.goals[self.waypoint_index].timeout_sim_s) * 1e9)
            )
            self.terminal_parking_owner = self.cascade.ordinary_parking
            self.terminal_parking_request = self.request_id
        self.steering = dict(
            direction=(goal - self.pose)[:2].tolist(),
            heading=self.cascade.goal_yaw,
            heading_error=0.0,
            exhausted=False,
        )
        _parking_sensor_row = None
        _parking_stale = None
        try:
            _parking_sensor_row = _sensor_hold_begin(self, "ordinary_parking", now, ns)
            # 停车只依赖实测来源：位姿/点云墙钟超过 0.3 s 或没有配对 IMU 都按 stale。
            stale = (
                now - self.pose_updated > 0.3
                or now - self.cloud_updated > 0.3
                or self.paired_imu is None
            )
            _parking_stale = stale
            _parking_tilt = self.apply_tilt_guard(now, ns / 1e9)
            _sensor_parking_protection(self, _parking_sensor_row, _parking_tilt)
            if _parking_tilt or stale or self.obstacle_hold:
                # stale、倾角保护或障碍 hold 时只发 hold 零速。
                self.select_pid_velocity(np.zeros(2), 0.0, goal, False, mode_override="hold")
                self.publish_command()
                return
            velocity, yawrate = self.select_pid_velocity(
                np.zeros(2), 0.0, goal, True, mode_override="capture"
            )
            result = self.evaluate_motion_guard(
                self.cloud,
                self.pose,
                goal,
                self.steering["direction"],
                self.execution_guard_route(bundle["start"], goal),
            )
            if result[0]:
                self.message = "终点实际点云保护停车"
                # guard 报保护：立即发零速停车，并归档被检查过的候选。
                self.publish_command()
                self.select_pid_velocity(np.zeros(2), 0.0, goal, False, mode_override="hold")
                self.publish_command()
            else:
                self.message = "实际SLAM固定终点主动捕获/位置朝向保持"
                self.publish_command(velocity, yawrate)
            if self.pending_guard_row:
                self.pending_guard_row.update(
                    command_after_control=list(self.command),
                    zero_requested_after_control=not any(self.command),
                    state_after_control=self.state,
                    after_control_ros_clock_ns=self.get_clock().now().nanoseconds,
                    obstacle_hold_after=self.obstacle_hold,
                    evidence_queue_error=self.evidence.error,
                )
                self.evidence.append(
                    self.run / "navigation_guard_history.jsonl", self.pending_guard_row
                )
                self.pending_guard_row = None
        finally:
            # 恢复原航点索引并结束本 tick 的传感器保持记录。
            self.waypoint_index = index
            _sensor_hold_end(self, _parking_sensor_row, _parking_stale)

    def publish_command(self, velocity=None, yaw_rate=0.0, *, duplicate_ordinary_hold=None):
        """发布一条命令；``velocity is None`` 表示零速停车，否则发布正常速度命令。

        参数：``velocity`` 为 body 系平面速度（m/s）或 ``None``；``yaw_rate`` 为
        rad/s；``duplicate_ordinary_hold`` 只供普通停车重复零命令的源文件保留事务使用。
        发布前：执行桥接交接存在时先 ``reserve`` 申请发布权，抢不到就交给
        ``cancel_publication``（走原零速保护路径）；正常速度必须与
        ``pid_prepared_output`` 在 1e-12 绝对容差内一致，否则抛 RuntimeError（防止
        guard 之后命令被改）。零速路径会 reset PID、撤销命令源，并在主动交接中把非权威零速判为交接
        失败。副作用：写 ``cascade_command_source.json``（临时文件 + replace 原子替换）、
        PID 行证据、``/demo/cmd_vel`` Twist 与 ``/demo/navigation/execution_frozen`` 开关
        （state 非 running、障碍 hold 或 |v_xy| < 0.015 m/s 时为 True，阈值硬编码）。
        返回 source_preservation（仅重复零命令保留事务时可能非 None）。
        """
        handoff = getattr(self, "_bridge_safety_handoff", None)
        reservation = False
        if velocity is not None and handoff is not None:
            # 有执行桥接交接时先申请写租约；抢不到就把这次发布交给原零速保护路径。
            reservation = handoff.reserve(self)
            if not reservation:
                return handoff.cancel_publication(self)
        try:
            join = getattr(self, "_r104_parking_causal_join", None)
            prior_join_source = None if join is None else join.published_source
            # 任何未成功的发布或命令源替换都会让 join 缓存失效；只有成功的原始重复
            # 零命令保留事务才能把它恢复。
            if join is not None:
                join.published(None)
            _r47_call = _r47_begin(self, "publication", stopped=velocity is None)
        except BaseException:
            if reservation:
                handoff.end_reservation(self)
            raise
        try:
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Bool

            # PUBLICATION_OBSERVER_BEGIN entry_snapshot
            from embodied_navigation.control.publication_ledger import PublicationLedger

            publication_before = PublicationLedger.begin(self, velocity is None)
            _r47_publication_anchor(self, _r47_call, publication_before)
            # PUBLICATION_OBSERVER_END entry_snapshot
            msg = Twist()
            ns = self.get_clock().now().nanoseconds
            if velocity is None:
                # 零速发布是检查终端停车所有者失败的时机。
                self.observe_ordinary_terminal_protection(ns)
            self.last_publication_ros_clock_ns = ns
            if velocity is not None:
                output = np.r_[velocity, yaw_rate]
                if self.pid_prepared_output is None or not np.allclose(
                    output, self.pid_prepared_output, rtol=0, atol=1e-12
                ):
                    # 正常速度必须与 native guard 之前准备的输出完全一致；不一致说明
                    # 命令在 guard 之后被改，宁可抛错也不发布未受检的命令。
                    raise RuntimeError("Cascade command changed after native guard")
                msg.linear.x, msg.linear.y, msg.angular.z = map(float, output)
            elif hasattr(self, "pid"):
                # 零速路径重置计数器并清空本轮 prepared 输出，避免下一次正常发布复用它。
                self.pid.reset("exact_zero_stop_or_protection")
                self.pid_prepared_output = None
            self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
            self.last_command_time = ns / 1e9
            gate = self.heading_gate
            if getattr(gate, "handoff_active", False) and velocity is None:
                # 主动交接期间的显式零速：来源/状态不新鲜，或零速不是权威零阶段，都判
                # 为交接失败并置 failed。
                now = time.monotonic()
                protected = (
                    self.state != "running"
                    or self.obstacle_hold
                    or self.tilt_hold
                    or getattr(self, "reference_consistency_hold", False)
                    or self.samples is None
                    or getattr(self, "path_admission_hold", False)
                    or now - self.pose_updated > self.pose_timeout
                    or now - self.cloud_updated > self.cloud_timeout
                    or not self.raw_imu.fresh(now, ns / 1e9)
                    or now - self.bridge_updated > 0.30
                    or self.bridge_safety.get("state") != "ready"
                )
                if protected or getattr(gate, "settle_control_stage", None) != "authoritative_zero":
                    gate.fail_handoff("explicit_zero_protection_or_stop", ns / 1e9)
                    self.state = "failed"
                    self.message = "主动转向交接失败：" + str(gate.handoff_failure)
            # 只把“权威零阶段 + 交接仍活跃 + 未失败”的零发布记为最终零。
            note_final_zero = (
                velocity is None
                and not any(self.command)
                and getattr(gate, "settle_control_stage", None) == "authoritative_zero"
                and getattr(gate, "handoff_active", False)
                and not getattr(gate, "handoff_failure", None)
            )
            row = getattr(self, "pid_row", None)
            _r47_bind(self, _r47_call, row)
            if row is not None:
                # 命令源快照与该行 PID 收据同源；时间戳与来源回执都取实际值。
                row.update(
                    publish_ros_clock_ns=ns,
                    command_after_slew=list(self.command),
                    state=self.state,
                    obstacle_hold=self.obstacle_hold,
                    alignment_hold=self.alignment_hold,
                    stopped=velocity is None,
                    reset_count_after_publish=self.pid.resets,
                    queue_error=self.evidence.error,
                )
                f = row["feedback"]
                i = row["imu"]
                source = dict(
                    schema="actual_SLAM_SCAN_cascade_command_source/v1",
                    cascade_sequence=row["sequence"],
                    source_pose_stamp_ns=row["source_pose_stamp_ns"],
                    paired_imu_stamp_ns=row["paired_imu_stamp_ns"],
                    control_stamp_ns=row["control_stamp_ns"],
                    command_after_slew=list(self.command),
                    navigation_ground_truth_used=False,
                    source_pose_received_wall_ns=None if f is None else f["received_wall_ns"],
                    source_imu_received_wall_ns=None if i is None else i["received_wall_ns"],
                    source_pose_received_monotonic_wall=None
                    if f is None
                    else f["received_wall_ns"] / 1e9,
                    paired_imu_received_monotonic_wall=None
                    if i is None
                    else i["received_wall_ns"] / 1e9,
                    request_id=row["request_id"],
                    trajectory_id=row["trajectory_id"],
                    waypoint_index=row["waypoint_index"],
                    goal_sha256=row["cascade"]["fixed_goal_sha256"],
                    path_sha256=row["cascade"]["path_sha256"],
                    cascade_mode=row["cascade"]["mode"],
                    parking_hold_declared_pose_stamp_ns=row["cascade"][
                        "parking_hold_declared_pose_stamp_ns"
                    ],
                    heading_transition_stage=getattr(gate, "settle_control_stage", None),
                    heading_transition_context=copy.deepcopy(
                        getattr(gate, "transition_context", {})
                    ),
                    heading_zero_handoff_count=getattr(gate, "zero_handoff_count", None),
                    heading_last_zero_handoff=copy.deepcopy(
                        getattr(gate, "last_zero_handoff", None)
                    ),
                )
                if getattr(self.cascade, "terminal_reference", None) is not None:
                    from embodied_navigation.control.parking.terminal_reference import fields

                    source.update(
                        fields(self.cascade.terminal_reference),
                        schema="actual_SLAM_terminal_fixed_goal_command_source/v1",
                        path_geometry_present=False,
                    )
                if (
                    getattr(gate, "integral_unload_config", None) is not None
                    and row["cascade"]["mode"] == "brake"
                ):
                    source["pre_zero_integral_unload"] = copy.deepcopy(gate.integral_unload_event)
                source["ordinary_parking_stage"] = row["cascade"].get("ordinary_parking_stage")
                source["ordinary_terminal_zero_event"] = None
                if (
                    velocity is not None
                    and row["cascade"]["mode"] == "ordinary_zero_hold"
                    and not row["cascade"]["protection_active"]
                ):
                    ordinary = self.cascade.ordinary_parking
                    if self.cascade.ordinary_zero_event is None:
                        self.cascade.ordinary_zero_event = make_event(
                            self.run.name,
                            self.request_id,
                            source["goal_sha256"],
                            source["path_sha256"],
                            row["sequence"],
                            ordinary.ready_stamp,
                            ordinary.handoff_clock,
                            row["sequence"],
                            terminal_reference=getattr(self.cascade, "terminal_reference", None),
                        )
                    marker = self.cascade.ordinary_zero_event
                    source.update(
                        ordinary_terminal_zero_event=dict(marker),
                        ordinary_zero_event_id=marker["event_id"],
                        ordinary_zero_event_sequence=marker["event_sequence"],
                    )
                    if ordinary.first_zero_publication_clock is None:
                        ordinary.first_zero_publication_clock = ns
                elif velocity is None:
                    source["ordinary_parking_stage"] = "revoked"
                # 临时文件 + replace 原子替换，保证下游读到与本 tick 命令一致的来源。
                tmp = self.run / "cascade_command_source.tmp"
                _r47_mark(self, _r47_call, "source_tmp_write_before")
                tmp.write_text(json.dumps(source, allow_nan=False) + "\n")
                _r47_mark(self, _r47_call, "source_tmp_write_after")
                _r47_mark(self, _r47_call, "source_replace_before")
                tmp.replace(self.run / "cascade_command_source.json")
                _r47_mark(self, _r47_call, "source_replace_after")
                if _r118_source_age.ENABLED:
                    _r118_source_age.controller(self, "source.atomic_replace", ns, row)
                _r47_mark(self, _r47_call, "PID_evidence_append_before")
                _append_pid_row_evidence(self, row)
                _r47_mark(self, _r47_call, "PID_evidence_append_after")
                self.pid_row = None
                self._pid_row_receipt_snapshot = None
            source_preservation = None
            if velocity is None and row is None:
                # 重复零命令尝试保留既有命令源；不允许保留时写一条显式撤销来源记录。
                if duplicate_ordinary_hold is not None:
                    from embodied_navigation.control.safety.duplicate_source import (
                        preserve_existing_source,
                        observe_unchanged_after_publish,
                    )

                    source_preservation = preserve_existing_source(
                        self,
                        duplicate_ordinary_hold,
                        ns,
                        command_before=publication_before["command_before"],
                    )
                if source_preservation is None or not source_preservation["allowed"]:
                    revoked = dict(
                        schema="cascade_command_explicit_zero_revocation/v1",
                        navigation_ground_truth_used=False,
                        command_after_slew=[0.0, 0.0, 0.0],
                        ordinary_terminal_zero_event=None,
                        ordinary_parking_stage="revoked",
                        request_id=self.request_id,
                        publish_ros_clock_ns=ns,
                    )
                    tmp = self.run / "cascade_command_source.tmp"
                    _r47_mark(self, _r47_call, "revoked_source_tmp_write_before")
                    tmp.write_text(json.dumps(revoked, allow_nan=False) + "\n")
                    _r47_mark(self, _r47_call, "revoked_source_tmp_write_after")
                    _r47_mark(self, _r47_call, "revoked_source_replace_before")
                    tmp.replace(self.run / "cascade_command_source.json")
                    _r47_mark(self, _r47_call, "revoked_source_replace_after")
                    if source_preservation is not None:
                        source_preservation.update(
                            source_file_written=True, preservation_performed=False
                        )
                else:
                    source_preservation["preservation_performed"] = True
            # PUBLICATION_OBSERVER_BEGIN actual_publish_start
            publication_started_wall_ns = PublicationLedger.publication_started()
            # PUBLICATION_OBSERVER_END actual_publish_start
            _r47_mark(self, _r47_call, "DDS_publish_call_before")
            self.cmd_pub.publish(msg)
            join = getattr(self, "_r104_parking_causal_join", None)
            if join is not None:
                if row is not None:
                    join.published(source)
                elif source_preservation is not None and source_preservation["allowed"]:
                    join.published(prior_join_source)
            _r47_mark(self, _r47_call, "DDS_publish_return")
            if _r118_source_age.ENABLED:
                _r118_source_age.controller(self, "command.DDS_return", ns, row)
            self.counts["commands"] += 1
            if getattr(gate, "quiet_target_enabled", False):
                record = None
                core = {} if row is None else row.get("cascade", {})
                ref = {} if row is None else row.get("heading_gate_reference", {})
                f = None if row is None else row.get("feedback")
                i = None if row is None else row.get("imu")
                binding = gate._quiet_binding(
                    gate.transition_context,
                    gate.transition_purpose,
                    self.handoff_scope_sha256,
                    self.cascade_path_id,
                    None if self.cascade is None else self.cascade.path_sha,
                    None if self.cascade is None else self.cascade.goal_sha,
                )
                normal = bool(
                    row is not None
                    and isinstance(f, dict)
                    and isinstance(i, dict)
                    and "received_wall_ns" in f
                    and "received_wall_ns" in i
                    and velocity is not None
                    and self.state == "running"
                    and gate.controlled_braking
                    and gate.handoff_active
                    and core.get("mode") == "brake"
                    and core.get("controller_updated") is True
                    and core.get("protection_active") is False
                    and ref.get("phase") == "settle"
                    and ref.get("controlled_braking") is True
                    and ref.get("settle_control_stage") == "controlled_braking"
                    and row.get("quiet_handoff_binding") == binding
                    and row.get("request_id") == self.request_id
                    and row.get("waypoint_index") == self.waypoint_index
                    and not self.obstacle_hold
                    and not self.tilt_hold
                    and not getattr(self, "reference_consistency_hold", False)
                    and not getattr(self, "path_admission_hold", False)
                    and self.samples is not None
                    and not self.control_clock_hold.holding
                    and not self.control_clock_hold.guard_required
                    and not self.pid_guard_needs_evaluation
                    and self.evidence.error is None
                    and not row.get("launch_output_constraint", {}).get("applied", False)
                )
                if normal:
                    record = dict(
                        sequence=self.counts["commands"],
                        publish_clock_ns=self.get_clock().now().nanoseconds,
                        publish_wall_ns=time.monotonic_ns(),
                        control_clock_ns=row["compute_ros_clock_ns"],
                        control_wall_ns=core["compute_wall_ns"],
                        source_pose_stamp_ns=row["source_pose_stamp_ns"],
                        paired_imu_stamp_ns=row["paired_imu_stamp_ns"],
                        source_pose_received_wall_ns=f["received_wall_ns"],
                        source_imu_received_wall_ns=i["received_wall_ns"],
                        command_body=list(self.command),
                        normal_braking=True,
                        **binding,
                    )
                if record is not None and getattr(gate, "integral_unload_config", None) is not None:
                    pi = core.get("velocity_PI", {})
                    record["pre_zero_integral_unload"] = copy.deepcopy(
                        pi.get("pre_zero_integral_unload")
                    )
                    record["yaw_integral_state"] = pi.get("integral_state", [None] * 3)[2]
                    record["integral_dt_s"] = pi.get("integral_dt_s")
                gate.note_quiet_publication(record)
            if note_final_zero:
                gate.note_zero_publication(self.get_clock().now().nanoseconds)
            _r47_mark(self, _r47_call, "quiet_and_zero_note_after")
            # PUBLICATION_OBSERVER_BEGIN actual_publish_completed
            PublicationLedger.completed(self, publication_before, ns, publication_started_wall_ns)
            _r47_publication_anchor(self, _r47_call, completed=True)
            _r67_attach_known_frame_failure(self, _r47_call, publication_before)
            # PUBLICATION_OBSERVER_END actual_publish_completed
            # 执行冻结开关：非 running、障碍 hold 或平面速度 < 0.015 m/s（硬编码，
            # 命令单位 m/s）时为 True。
            self.freeze_pub.publish(
                Bool(
                    data=bool(
                        self.state != "running"
                        or self.obstacle_hold
                        or np.linalg.norm(self.command[:2]) < 0.015
                    )
                )
            )
            if source_preservation is not None and source_preservation["allowed"]:
                observe_unchanged_after_publish(self, source_preservation)
            # 返回保留事务结果（仅重复零命令路径可能非 None），供上层核对。
            return source_preservation
        finally:
            if reservation:
                handoff.end_reservation(self)
            _r47_end(self, _r47_call)
