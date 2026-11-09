"""Sensors and command ingress for ``PIDNavigation``.

The callbacks here only translate/validate ROS inputs and update the node's
state; admission, routing and velocity decisions stay in the other mixins.
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
    """Inbound ROS callbacks: requests/goals, corridor snapshots, IMU, odometry, cloud and archives."""

    def on_request(self, msg):
        previous = self.request_id
        super().on_request(msg)
        if self.request_id != previous and hasattr(self, "cascade"):
            self._r105_terminal_context = None
            asynchronous = getattr(self, "_r88_admission", None)
            if asynchronous is not None:
                asynchronous.invalidate("accepted_request_changed")
            # Each stage has its own measured path and PI state. Source/IMU
            # histories and the immutable original registration remain valid.
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
            # The route identity disambiguates overlapping floor paths; the
            # actual surface still has to be established by measured points.
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
        before = self.gyro_stamp
        before_records = self.imu_records
        wall = time.monotonic_ns()
        clock = self.get_clock().now().nanoseconds
        super().on_imu(msg)
        stamp = int(self.stamp(msg))
        q = msg.orientation
        v = msg.angular_velocity
        # R85 retains original safety-accepted IMU bytes until their headers
        # become causal. The unchanged odometry pairing still requires
        # IMU <= pose <= control clock and the original 20ms pair window.
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
            _sensor_callback_odom(
                self, stamp, clock, wall, self.pose_stamp, self.pose_stamp, early_future=True
            )
            return
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
        latest = (self.gyro_body, self.gyro_stamp, self.gyro_wall)
        if paired:
            self.gyro_body = paired["angular_velocity_body"]
            self.gyro_stamp = paired["stamp_ns"] / 1e9
            self.gyro_wall = paired["received_wall_ns"] / 1e9
        else:
            self.gyro_body = None
        before = self.pose_stamp
        quiet = getattr(self.heading_gate, "quiet_target_enabled", False)
        if quiet:
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
        wall = time.monotonic()
        clock = self.get_clock().now().nanoseconds
        before = self.cloud_stamp
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
        self.cloud_archive_records += 1
        seq = self.cloud_archive_records
        if getattr(self, "registered_cloud_archive_mode", "legacy_all") == "disabled":
            # Keep the accepted live geometry and exact native guard receipt.
            # Do not allocate an additional point copy or write payload files.
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

        def archive():
            directory.mkdir(exist_ok=True)
            np.save(array, xyz, allow_pickle=False)
            payload.write_bytes(raw)
            document = {**record, "array_sha256": hashlib.sha256(array.read_bytes()).hexdigest()}
            with (self.run / "navigation_cloud_xyz.jsonl").open("a") as out:
                out.write(json.dumps(document, allow_nan=False) + "\n")

        self.evidence.enqueue(archive)
