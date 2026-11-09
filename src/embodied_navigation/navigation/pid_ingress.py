"""``PIDNavigation`` 的传感器与命令入口。

这里的回调只做 ROS 输入的翻译、校验与节点状态更新，准入、路线与速度决策留在
其他 mixin。入口对来源的限制沿用基类：``TeacherNavigation`` 先要求请求与本 run
冻结的 ``navigation_request.json`` 一致、SLAM 位姿/点云 header 合法且前进，
``base_node`` 再按 frame、时间戳与相邻姿态过滤，所以下文说的“基类接受”指这些
检查全部通过。

边界：本模块不主动产生运动命令；换任务时会调用 ``publish_command()`` 发零速并
撤销命令源（见 ``on_request``），其余停车由基类保护路径触发。默认 writer 下
``evidence.append``/``enqueue`` 均为空操作，除非替换为记录型 writer
（navigation/README.md“日志与输出”节），否则这里写的证据不落盘。
"""

from .pid_support import (
    math,
    _sensor_callback_odom,
    Path,
    np,
    time,
    copy,
    RouteProgress,
    _r99_clock_probe,
    json,
    CLOCK_SERVICE_ENABLED,
    _sensor_callback_cloud,
    _enqueue_navigation_imu_record,
    hashlib,
)


class IngressMixin:
    """订阅回调集合：请求/目标、走廊快照、IMU、里程计、点云与点云归档。"""

    def on_request(self, msg):
        """接受/拒绝一条新请求，并在 request_id 真正切换时清空按阶段状态。

        ``super().on_request(msg)`` 先做准入：``TeacherNavigation`` 要求请求内容与本
        run 冻结的 ``navigation_request.json`` 一致，``base_node`` 要求目标可解析、
        同一 request_id 不得改目标、运行中不接收新任务；被拒绝时 request_id 不变，
        这里也不会重置任何状态。切换成功后清掉 cascade、路径收据、准入 hold 与连续
        路线进度，并发一条零速命令；IMU/SLAM 历史保持有效。
        """
        previous = self.request_id
        super().on_request(msg)
        if self.request_id != previous and hasattr(self, "cascade"):
            self._r105_terminal_context = None
            asynchronous = getattr(self, "_r88_admission", None)
            if asynchronous is not None:
                asynchronous.invalidate("accepted_request_changed")
            # 每个阶段各自持有一条实测路径与 PI 状态；来源/IMU 历史与不可变的原始
            # 注册仍然有效，故不在这里清空。
            self.cascade = None
            self.cascade_key = None
            self.cascade_last_row = None
            self.cascade_path_id = None
            self.path_receipt = None
            self.final_bundle = None
            self.reference_replan_pending = False
            self.pending_path_commit = None
            self.path_admission_active = None
            self.path_admission_hold = False
            self.path_admission_replan_pending = False
            self.pid_row = None
            self._pid_row_receipt_snapshot = None
            self.pid_prepared_output = None
            self.reference_consistency_hold = False
            self.reference_consistency_check = None
            # 先发零速并撤销旧命令源，避免上一任务的末速度被带入新任务。
            self.publish_command()
            self.continuous_route = (
                RouteProgress(
                    self.request_id,
                    self.goals,
                    self.goals_definition_sha256,
                    self.continuous_settings,
                )
                if self.continuous_settings["enabled"]
                else None
            )
            self.coverage_replan_index = None

    def on_corridor_snapshot(self, msg):
        """把一条走廊快照交给 shadow 观察者，并归档输入与提交结果。

        ``corridor_observer`` 只在 ``corridor.mode=shadow`` 时由
        ``pid_navigation.__init__`` 创建；正常启动（``mode=off``）为 ``None``，回调
        直接返回。快照解码与摘要来自 ``common.observation.api("corridor_runtime")``，
        本检出该端口恒为禁用实现（common/observation.py:86-88），其
        ``CorridorObserver`` 构造即抛 ``ValueError``，所以这些路径只有外部实验包接管
        该端口时才可用。时间戳均为 ns：``wall`` 为 monotonic，``clock`` 为 ROS clock。
        回调无控制权，证据固定标 ``control_authority=False``；任何异常只写拒绝证据。
        """
        from embodied_navigation.common.observation import api as _observer_api

        decode_snapshot = _observer_api("corridor_runtime").decode_snapshot
        sha_text = _observer_api("corridor_runtime").sha_text
        wall = time.monotonic_ns()
        clock = self.get_clock().now().nanoseconds
        if self.corridor_observer is None:
            return
        try:
            snapshot = decode_snapshot(msg.data)
            self.corridor_snapshot_count += 1
            if self.cascade is None or self.feedback is None or self.path_receipt is None:
                return
            reference = self.cascade.preview_reference(self.pose)
            # layer_id 用“任务号:航点号”区分重叠的地面路径；实际可通行表面仍须由
            # 实测点云建立，不能只看路线身份。
            layer_id = str(self.request_id) + ":" + str(self.waypoint_index)
            path = dict(
                points_xyz=self.cascade.path.tolist(),
                path_id=self.cascade.path_id,
                path_sha256=self.cascade.path_sha,
                frame_id="camera_init",
                layer_id=layer_id,
                stamp_ns=self.cascade.path_record["stamp_ns"],
            )
            state = dict(
                position_world_xyz=self.pose.tolist(),
                yaw_rad=math.atan2(self.rotation[1, 0], self.rotation[0, 0]),
                speed_mps=float(np.linalg.norm(self.feedback["origin_velocity_body"][:2])),
                progress_m=reference["nearest_s_m"],
                stamp_ns=int(self.pose_stamp),
            )
            digest = sha_text(msg.data)
            if self.corridor_observer.submit(snapshot, path, state, clock, wall, digest):
                # 观察者接受该快照后才归档原始字节；写盘放在 enqueue 闭包里，默认
                # writer 不执行该闭包。
                import gzip

                raw = msg.data.encode("utf-8")
                relative = Path("corridor_snapshots") / (
                    str(self.corridor_snapshot_count).zfill(6) + ".json.gz"
                )

                def archive():
                    target = self.run / relative
                    target.parent.mkdir(exist_ok=True)
                    target.write_bytes(gzip.compress(raw, compresslevel=1, mtime=0))

                self.evidence.enqueue(archive)
                self.evidence.append(
                    self.run / "corridor_inputs.jsonl",
                    dict(
                        snapshot_file=str(relative),
                        snapshot_sha256=digest,
                        path=path,
                        actual_state=state,
                        limits=self.corridor_observer.limits,
                        received_wall_ns=wall,
                        compute_request_clock_ns=clock,
                        navigation_ground_truth_used=False,
                        control_authority=False,
                    ),
                )
        except Exception as error:
            # 解码/提交失败只记拒绝证据，不改变控制状态。
            self.evidence.append(
                self.run / "corridor_input_rejections.jsonl",
                dict(
                    reason=type(error).__name__ + ":" + str(error),
                    received_wall_ns=wall,
                    clock_ns=clock,
                    control_authority=False,
                ),
            )

    def poll_corridor(self, now):
        """轮询走廊观察结果；异常只写拒绝证据，不向控制链路抛出。"""
        try:
            return self._poll_corridor(now)
        except Exception as error:
            self.evidence.append(
                self.run / "corridor_input_rejections.jsonl",
                dict(
                    reason="shadow_poll:" + type(error).__name__ + ":" + str(error),
                    received_wall_ns=time.monotonic_ns(),
                    control_authority=False,
                ),
            )

    def _poll_corridor(self, now):
        """归档走廊证书，并按 profile 周期请求一条未来走廊路径。

        证书写入 ``corridor_certificates.jsonl``。仅当任务 running、已有级联路径、
        且距上次请求超过 ``profile["corridor"]["snapshot_request_period_s"]``（经
        prepare pin 进 profile，单位 s）时才发请求：从当前位姿起沿路径取 13 个采样点
        （连当前位姿共 14 个位姿），前瞻 1.2 m（硬编码，与 pid_navigation 中
        ``horizon_m=1.2`` 一致），frame 为 ``camera_init``。
        """
        if self.corridor_observer is None:
            return
        result = self.corridor_observer.poll()
        if result is not None:
            result["recorded_clock_ns"] = self.get_clock().now().nanoseconds
            result["active_path_id_at_receipt"] = (
                None if self.cascade is None else self.cascade.path_id
            )
            self.corridor_last_certificate = result
            self.evidence.append(self.run / "corridor_certificates.jsonl", result)
        if (
            self.state != "running"
            or self.cascade is None
            or self.cascade.path is None
            or now - self.corridor_last_request_wall
            < float(self.profile["corridor"]["snapshot_request_period_s"])
        ):
            return
        from nav_msgs.msg import Path as PathMsg
        from geometry_msgs.msg import PoseStamped
        from embodied_navigation.planning.reference.spatial_reference import point_at

        reference = self.cascade.preview_reference(self.pose)
        start = reference["nearest_s_m"]
        end = min(start + 1.2, float(self.cascade.cumulative[-1]))
        msg = PathMsg()
        msg.header.frame_id = "camera_init"
        msg.header.stamp = self.get_clock().now().to_msg()
        for point in [
            self.pose,
            *[
                point_at(self.cascade.path, self.cascade.cumulative, s)
                for s in np.linspace(start, end, 13)
            ],
        ]:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.corridor_request_pub.publish(msg)
        self.corridor_last_request_wall = now

    def on_imu(self, msg):
        """先让基类校验并接受 IMU，再补一条配对历史与安全缓冲注记。

        只有 ``super().on_imu`` 接受（gyro 时间戳前进）且 header 不晚于当前 ROS
        clock 时才写历史；R85 自动参考运行时，未来 header 也保留为历史但不参与因果
        配对（行内标 ``r85_original_future_header_history_only``）。单位：stamp/clock
        为 ns，``received_wall_ns`` 为 monotonic ns；角速度已转到 body 系。
        ``source_topic`` 固定 ``/livox/imu``。
        """
        before = self.gyro_stamp
        before_records = self.imu_records
        wall = time.monotonic_ns()
        clock = self.get_clock().now().nanoseconds
        super().on_imu(msg)
        stamp = int(self.stamp(msg))
        q = msg.orientation
        v = msg.angular_velocity
        # R85 会保留安全接受过的原始 IMU 字节，直到其 header 变成因果；未改动的
        # 里程计配对仍要求 IMU <= pose <= 控制时钟，并沿用原来的 20 ms 配对窗。
        r85_history_only = getattr(self, "automatic_reference", None) is not None
        if (
            self.gyro_stamp is not None
            and round(self.gyro_stamp * 1e9) == stamp
            and self.gyro_stamp != before
            and (stamp <= clock or r85_history_only)
        ):
            row = dict(
                stamp_ns=stamp,
                received_wall_ns=wall,
                frame_id="body",
                angular_velocity_body=list(self.gyro_body),
                source_frame_id=msg.header.frame_id,
                gyro_source_xyz=[v.x, v.y, v.z],
                orientation_xyzw=[q.x, q.y, q.z, q.w],
                orientation_covariance=list(msg.orientation_covariance),
                imu_to_body_rotation=self.imu_body_rotation.tolist(),
                callback_ros_clock_ns=clock,
                source_topic="/livox/imu",
                navigation_ground_truth_used=False,
            )
            if stamp > clock:
                row["r85_original_future_header_history_only"] = True
            self.imu_history.append(row)
            self.imu_records += 1
            _enqueue_navigation_imu_record(self, row, self.imu_records)
        if self.imu_source_buffer is not None:
            intake = self.imu_source_buffer.safety_intake
            original_raw = self.raw_imu if intake is None else intake.original_raw_imu
            self.imu_source_buffer.note_latest(
                stamp, wall, clock, self.imu_records > before_records, original_raw.last_rejected
            )

    def on_odom(self, msg):
        """校验一条 SLAM 里程计，配对同刻 IMU，并在基类接受后重建反馈。

        header 晚于当前 ROS clock 时先交给 ``_sensor_callback_odom(early_future=True)``
        记账后直接返回，不更新位姿。否则优先走 R104 因果配对（环境变量
        ``R104_PARKING_CAUSAL_JOIN``，代码默认 "0"，configs/launch/settings.json 设为
        "1"），否则在 IMU 历史里取最近一条满足 ``0 <= stamp - imu_stamp <= 20 ms`` 的
        记录；配对失败仍交给基类，但 ``paired_imu`` 为空，若随后仍无法从 IMU 缓冲补配，
        ``select_pid_velocity`` 会因缺因果 IMU 而 hold。
        只有 ``pose_stamp`` 前进（基类真正接受）时才重建 ``feedback`` 与
        ``pid_velocity``；位置单位 m、速度 m/s，姿态为 world 系 WXYZ。
        """
        stamp = int(self.stamp(msg))
        if CLOCK_SERVICE_ENABLED:
            self._r114_clock_service.service(stamp)
        _clock_read_before = (
            _r99_clock_probe.read_before(self) if _r99_clock_probe.READ_OPTION == "1" else None
        )
        clock = self.get_clock().now().nanoseconds
        wall = time.monotonic_ns()
        _r99_clock_probe.note(self, stamp, clock, wall, self.pose_stamp, _clock_read_before)
        if stamp > clock:
            # header 在未来：只记账，不参与位姿/配对。
            _sensor_callback_odom(
                self, stamp, clock, wall, self.pose_stamp, self.pose_stamp, early_future=True
            )
            return
        # R104 因果配对优先；未安装（None，模块默认关闭）时回退到 IMU 历史。
        join = getattr(self, "_r104_parking_causal_join", None)
        bound, paired = (False, None) if join is None else join.bound_pair(self, stamp, clock, wall)
        if not bound:
            paired = next(
                (r for r in reversed(self.imu_history) if 0 <= stamp - r["stamp_ns"] <= 20_000_000),
                None,
            )
            if self.imu_source_buffer is not None:
                original = paired
                paired = self.imu_source_buffer.pair(stamp, paired, clock)
                if original is None:
                    self.imu_source_buffer.event(
                        stamp,
                        clock,
                        wall,
                        "odom_pair_missing" if paired is None else "odom_pair_from_fifo",
                        paired,
                    )
        # 先保存基类回调前的 gyro 值；给基类一个与位姿配对的因果 IMU，回调结束再
        # 还原为最新值。
        latest = (self.gyro_body, self.gyro_stamp, self.gyro_wall)
        if paired:
            # 有配对才替换 gyro；否则清空，避免旧 IMU 冒充当前配对。
            self.gyro_body = paired["angular_velocity_body"]
            self.gyro_stamp = paired["stamp_ns"] / 1e9
            self.gyro_wall = paired["received_wall_ns"] / 1e9
        else:
            self.gyro_body = None
        before = self.pose_stamp
        quiet = getattr(self.heading_gate, "quiet_target_enabled", False)
        if quiet:
            # 安静目标模式下，位姿回调要包进 begin/finish，用于交接期的静默判定。
            self.heading_gate.begin_quiet_pose(
                stamp,
                clock,
                wall,
                self.heading_gate.transition_context,
                self.heading_gate.transition_purpose,
                self.handoff_scope_sha256,
                self.cascade_path_id,
                None if self.cascade is None else self.cascade.path_sha,
                None if self.cascade is None else self.cascade.goal_sha,
            )
        try:
            super().on_odom(msg)
        finally:
            self.gyro_body, self.gyro_stamp, self.gyro_wall = latest
            if quiet:
                self.heading_gate.finish_quiet_pose(stamp, self.pose_stamp > before)
        if self.pose_stamp > before:
            # 只有基类真正前移位姿时间戳时才更新反馈；否则保留上一条已接受反馈。
            q = msg.pose.pose.orientation
            v = msg.twist.twist.linear
            from embodied_navigation.perception.sensors.feedback_mailbox import receipt_ns
            from embodied_navigation.localization_mapping.feedback_state import (
                accepted_slam_feedback,
            )

            self.feedback = accepted_slam_feedback(
                stamp_ns=stamp,
                received_wall_ns=receipt_ns(self, "body", wall),
                position_world_xyz=self.pose.tolist(),
                quaternion_wxyz=[q.w, q.x, q.y, q.z],
                origin_velocity_body=[v.x, v.y, v.z],
                callback_ros_clock_ns=clock,
            )
            self.paired_imu = copy.deepcopy(paired)
            self.pid_velocity = self.rotation @ np.array([v.x, v.y, v.z])
            self.pid_pose_stamp = stamp
            self.evidence.append(
                self.run / "navigation_feedback_history.jsonl",
                dict(
                    self.feedback,
                    paired_imu=copy.deepcopy(paired),
                    navigation_ground_truth_used=False,
                ),
            )
        _sensor_callback_odom(self, stamp, clock, wall, before, self.pose_stamp, message=msg)

    def on_cloud(self, msg):
        """记录一次点云回调收据，并在基类接受（``cloud_stamp`` 前进）时归档。

        无论接受与否都写一行 ``navigation_cloud_callbacks.jsonl``；``reason`` 由基类
        ``note_cloud_decision`` 的分支填写（帧、解码、无有限点、邻近姿态等拒绝原因），
        若回调返回时仍未填写则记为 ``callback_returned_without_branch_receipt``。
        归档只对接受的点云执行（``archive_cloud``）。时间戳同时记录 ROS clock ns 与
        monotonic wall 秒。
        """
        wall = time.monotonic()
        clock = self.get_clock().now().nanoseconds
        before = self.cloud_stamp
        # 先把待填的收据行挂到节点上，基类的 note_cloud_decision 会就地补 reason。
        self.cloud_callback_records += 1
        self.pending_cloud_callback_row = dict(
            schema="pid_actual_cloud_callback/v1",
            sequence=self.cloud_callback_records,
            producer_stamp_ns=int(self.stamp(msg)),
            frame_id=msg.header.frame_id,
            received_monotonic_wall=wall,
            received_ros_clock_ns=clock,
            source_topic="/cloud_registered_full",
            input_width=int(msg.width),
            input_height=int(msg.height),
            previous_accepted_stamp_ns=int(before),
            reason=None,
            navigation_ground_truth_used=False,
        )
        try:
            super().on_cloud(msg)
            if self.cloud_stamp > before:
                self.archive_cloud(msg)
        finally:
            # 无论接受/拒绝都补齐耗时与接受标志并写证据；归档只在接受时进行。
            row = self.pending_cloud_callback_row
            end = time.monotonic()
            row.update(
                finished_monotonic_wall=end,
                callback_duration_wall_s=end - wall,
                finished_ros_clock_ns=self.get_clock().now().nanoseconds,
                accepted=row["reason"] == "accepted",
                accepted_stamp_ns=int(self.cloud_stamp),
                accepted_cloud_received_monotonic_wall=self.cloud_updated
                if math.isfinite(self.cloud_updated)
                else None,
            )
            if row["reason"] is None:
                row["reason"] = "callback_returned_without_branch_receipt"
            _sensor_callback_cloud(self, row)
            self.evidence.append(self.run / "navigation_cloud_callbacks.jsonl", row)
            self.pending_cloud_callback_row = None

    def archive_cloud(self, msg):
        """归档一条被接受的点云；``disabled`` 模式只留收据，``legacy_all`` 另存数组。

        归档模式来自 profile 的 ``engineering_recording.registered_cloud_archive``，
        由 ``pid_navigation.__init__`` 限定为 ``legacy_all``/``disabled``。``disabled``
        不复制点、不写数组文件，只写一行收据并清空 ``guard_cloud_receipt`` 的归档字段；
        ``legacy_all`` 把过滤后的 ``<f8`` xyz 与原始 payload 写到
        ``navigation_cloud_arrays/``，真正的写盘在 ``evidence.enqueue`` 闭包里（默认
        writer 不执行）。``cloud_archive_records`` 只统计被接受并进入归档的点云。
        """
        self.cloud_archive_records += 1
        seq = self.cloud_archive_records
        if getattr(self, "registered_cloud_archive_mode", "legacy_all") == "disabled":
            # disabled 模式保留实时几何与精确的 native guard 收据，但不额外复制点，
            # 也不写 payload 文件。
            self.guard_cloud_receipt.update(
                cloud_array_file=None,
                cloud_raw_payload_file=None,
                cloud_archive_sequence=None,
                cloud_archive_collected=False,
            )
            self.evidence.append(
                self.run / "navigation_cloud_xyz.jsonl",
                dict(
                    schema="actual_registered_cloud_archive_disabled/v1",
                    sequence=seq,
                    stamp_ns=int(self.cloud_stamp),
                    frame_id=msg.header.frame_id,
                    filtered_xyz_float64_sha256=self.guard_cloud_receipt[
                        "filtered_xyz_float64_sha256"
                    ],
                    filtered_points=int(len(self.guard_cloud)),
                    input_payload_bytes=len(msg.data),
                    array_file=None,
                    raw_payload_file=None,
                    archive_mode="disabled",
                    cloud_archive_collected=False,
                    cloud_replay_verified=False,
                    historical_full_cloud_evidence_pass=False,
                    live_guard_input_preserved=True,
                    navigation_ground_truth_used=False,
                ),
            )
            return
        # legacy_all：保留过滤后的 float64 xyz 与原始 payload 字节，供回放核对。
        xyz = np.array(self.guard_cloud, dtype="<f8", copy=True)
        raw = bytes(msg.data)
        directory = self.run / "navigation_cloud_arrays"
        name = f"{seq:06d}_{self.cloud_stamp}"
        array = directory / (name + ".npy")
        payload = directory / (name + ".bin")
        record = dict(
            schema="actual_registered_cloud_xyz_archive/v1",
            sequence=seq,
            stamp_ns=int(self.cloud_stamp),
            frame_id=msg.header.frame_id,
            received_monotonic_wall=self.guard_cloud_receipt["received_monotonic_wall"],
            array_file=str(array.relative_to(self.run)),
            raw_payload_file=str(payload.relative_to(self.run)),
            filtered_xyz_float64_sha256=hashlib.sha256(xyz.tobytes()).hexdigest(),
            raw_payload_sha256=hashlib.sha256(raw).hexdigest(),
            dtype="<f8",
            shape=list(xyz.shape),
            fields=[
                dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                for f in msg.fields
            ],
            width=int(msg.width),
            height=int(msg.height),
            point_step=int(msg.point_step),
            row_step=int(msg.row_step),
            is_bigendian=bool(msg.is_bigendian),
            is_dense=bool(msg.is_dense),
            filtering_body_stamp_ns=self.guard_cloud_receipt["nearest_slam_pose_stamp_ns"],
            filtering_body_pose=self.guard_cloud_receipt["filtering_body_pose"],
            filtering_body_rotation=self.guard_cloud_receipt["filtering_body_rotation"],
            navigation_ground_truth_used=False,
        )
        self.guard_cloud_receipt.update(
            cloud_array_file=record["array_file"],
            cloud_raw_payload_file=record["raw_payload_file"],
            cloud_archive_sequence=seq,
        )

        # 真正的文件写入延后到 writer 的 enqueue 闭包；默认 writer 直接丢弃该闭包。
        def archive():
            directory.mkdir(exist_ok=True)
            np.save(array, xyz, allow_pickle=False)
            payload.write_bytes(raw)
            document = {**record, "array_sha256": hashlib.sha256(array.read_bytes()).hexdigest()}
            with (self.run / "navigation_cloud_xyz.jsonl").open("a") as out:
                out.write(json.dumps(document, allow_nan=False) + "\n")

        self.evidence.enqueue(archive)
