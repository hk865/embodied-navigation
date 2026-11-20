"""10 Hz RGB-D 帧发布与逐帧证据收据。

本模块负责传感器帧流水线：读取 RGB/深度/CameraParams 标注器、归档原始 RGB、
运行建图 sink、按原样发布 ROS 点云/深度/内参/RGB 消息、记录相机渲染位姿与
自过滤收据、定期保存 RGB-D 快照，并绘制 overview/RGB 视频画面。所有节奏、
journal 字段与断言均保持原样；“10 Hz”是由上层每 20 个 0.005 s 物理步调用
一次本模块形成的，本文件不自行计时。
"""

from collections import deque
import json

import numpy as np

from embodied_navigation.perception.depth import depth_cloud_packet


class FrameStream:
    """持有 10 Hz 帧计数与逐帧证据写入。

    ``video_frames`` 由视频写入器返回的下一帧序号；跨帧保留，用于阶段收据。
    """

    def __init__(
        self,
        *,
        experiment,
        out,
        stage,
        gt_diagnostic,
        ros,
        video,
        rgb_annotators,
        overview,
        camera_rig,
        target_speed,
        receipts,
        rtx_lidar=None,
        truth_pose_buffer=None,
    ):
        self.experiment = experiment
        self.rtx_lidar = rtx_lidar
        self.truth_pose_buffer = truth_pose_buffer
        self._pending_truth_packets = deque()
        self.rgb_outputs = rgb_annotators
        if rgb_annotators is not None:
            rgb_annotators.quarantine_output = out
            if hasattr(rgb_annotators, "_binding_quarantine"):
                rgb_annotators._flush_native_quarantine()
        self.out = out
        self.stage = stage
        self.gt_diagnostic = gt_diagnostic
        self.ros = ros
        self.video = video
        self.rgb_ann = rgb_annotators.rgb_ann
        self.depth_ann = rgb_annotators.depth_ann
        self.camera_params_ann = rgb_annotators.camera_params_ann
        self.rgb_time_ann = getattr(rgb_annotators, "time_ann", None)
        self.last_rgb_native_ns = None
        self.overview_ann = overview.ann
        self.camera = camera_rig.camera
        self.sensor_yaw = camera_rig.sensor_yaw
        self.mount = camera_rig.mount
        self.imu_mount = camera_rig.imu_mount
        self.target_speed = target_speed
        self.receipts = receipts
        self.video_frames = 0

    def _write_interactive_still(self, source_time, rgb):
        """Low-rate real camera still; cadence is wall/sim time based.

        The image is operator evidence only: it carries its own source time;
        never a pose or ground truth.
        """
        import json as _json
        import pathlib as _pathlib

        mailbox = self.experiment.get("interactive_mailbox")
        if not mailbox:
            return None
        last_still = getattr(self, "_interactive_last_still_s", None)
        still_interval_s = 2.0
        if last_still is not None and source_time - last_still < still_interval_s:
            return None
        self._interactive_last_still_s = source_time
        frame_index = int(getattr(self, "_interactive_still_count", 0))
        self._interactive_still_count = frame_index + 1
        frames_dir = _pathlib.Path(mailbox).parent / "interactive_frames"
        try:
            frames_dir.mkdir(parents=True, exist_ok=True)
            from PIL import Image as _PILImage

            image_path = frames_dir / ("rgb_%06d.png" % frame_index)
            _PILImage.fromarray(np.asarray(rgb, dtype=np.uint8)).save(image_path)
            record = {
                "frame": frame_index,
                "stamp_s": source_time,
                "source_time_s": source_time,
                "interval_s": still_interval_s,
                "cadence": "time_based",
                "image_frame_is_a_timestamp": True,
                "path": str(image_path),
                "image": "rgb_from_this_simulation",
                "pose_included": False,
                "ground_truth_included": False,
            }
            with (frames_dir / "index.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(_json.dumps(record, sort_keys=True) + "\n")
            return record
        except (OSError, ImportError, ValueError):
            return None

    def publish_rtx_pending(self, actual_time):
        """LiDAR publication is independent of RGB buffer/camera cadence."""
        packet = self.rtx_lidar.read()
        result = self.receipts.result
        result["point_cloud_source"] = "isaac_rtx_lidar"
        result["depth_backprojection_enabled"] = False
        actual_ns = round(actual_time * 1e9)
        truth_pose_buffer = getattr(self, "truth_pose_buffer", None)
        pending_truth = None
        if truth_pose_buffer is not None:
            pending_truth = getattr(self, "_pending_truth_packets", None)
            if pending_truth is None:
                pending_truth = deque()
                self._pending_truth_packets = pending_truth
        while packet is not None:
            # Physics clock must already cover the final native firing time.
            last_retained_ns = packet.receipt["last_retained_ns"]
            if last_retained_ns is not None and last_retained_ns > actual_ns + 1_000_000:
                raise ValueError("RTX scan contains rays later than the current physics clock")
            self.ros.publish_rtx_cloud(packet)
            if truth_pose_buffer is not None:
                if len(packet.records) == 0 or last_retained_ns is None:
                    raise ValueError("Cannot deskew empty RTX packet")
                first_ns = int(packet.stamp_ns) + int(packet.records["t"].min())
                last_ns = int(packet.stamp_ns) + int(packet.records["t"].max())
                pending_truth.append((packet, first_ns, last_ns, actual_ns))
            evidence = dict(packet.receipt, acquisition_physics_ns=actual_ns)
            self.receipts.journal("rtx_lidar_frames.jsonl", evidence)
            result["rtx_lidar_last_scan"] = evidence
            result["rtx_lidar_scans"] = self.rtx_lidar.scans
            result["rtx_lidar_connected"] = True
            scan_index = packet.receipt["scan_index"]
            if scan_index in (1, 10, 50):
                np.savez_compressed(self.out / ("rtx_scan_%04d.npz" % scan_index),
                                    points=packet.records, stamp_ns=packet.stamp_ns,
                                    metadata_json=json.dumps(evidence))
            packet = self.rtx_lidar.read()
        if truth_pose_buffer is not None and pending_truth:
            from embodied_navigation.simulation.dynanav.truth_deskew import (
                deskew_rtx_packet_to_world,
            )

            clock_ref = getattr(self.ros, "last_clock_ns", None)
            clock_ns = (
                clock_ref[0]
                if isinstance(clock_ref, list) and clock_ref
                else None
            )
            while pending_truth:
                pkt, first_ns, last_ns, enqueued_actual_ns = pending_truth[0]
                ready = (
                    clock_ns is not None
                    and last_ns <= clock_ns
                    and truth_pose_buffer.covers(first_ns, last_ns)
                )
                if ready:
                    pending_truth.popleft()
                    deskewed = deskew_rtx_packet_to_world(
                        pkt, truth_pose_buffer, clock_ns
                    )
                    self.ros.publish_truth_deskewed_and_lio(deskewed)
                    continue
                if actual_ns > enqueued_actual_ns + 5_000_000:
                    deskew_rtx_packet_to_world(
                        pkt,
                        truth_pose_buffer,
                        clock_ns if clock_ns is not None else -1,
                    )
                break
        if not result.get("rtx_lidar_connected"):
            result["rtx_lidar_reads_without_new_scan"] = result.get("rtx_lidar_reads_without_new_scan", 0) + 1
    def _process_rtx_frame(
        self, it, actual_time, rgb, overview_ann, camera_rows, *, publish_to_slam=True
    ):
        """RGB stays visual input; the only primary cloud is native RTX LiDAR."""
        if publish_to_slam:
            self.publish_rtx_pending(actual_time)
        result = self.receipts.result
        if publish_to_slam:
            still = self._write_interactive_still(float(it), rgb)
            if still is not None:
                result["interactive_still"] = still
        overview = np.asarray(overview_ann.get_data())[..., :3]
        assert overview.shape == (512, 512, 3), "Invalid overview frame"
        self.video_frames = self.video.write(overview, rgb, it, self.target_speed, self.gt_diagnostic)
        if publish_to_slam:
            self.ros.publish_rgb(it, rgb)
            camera_rows.append({"time": actual_time, "published_time": it,
                                "point_cloud_source": "isaac_rtx_lidar", "depth_backprojection_enabled": False})
            self.receipts.journal("camera_frames.jsonl", camera_rows[-1])

    def process(
        self,
        *,
        iteration,
        measurement,
        camera_rows,
        publish_to_slam=True,
    ):
        """按原循环块处理一帧 10 Hz RGB-D 数据。

        ``measurement`` 提供本帧对应的物理时间与位姿；``camera_rows`` 记录
        逐帧诊断日志。断言失败即终止采集。
        """
        from pxr import UsdGeom

        i = iteration
        actual_time = measurement.actual_time
        it = measurement.it
        pos = measurement.pos
        Rb = measurement.Rb
        sensor_position = measurement.sensor_position
        camera_position = measurement.camera_position
        out = self.out
        stage = self.stage
        gt_diagnostic = self.gt_diagnostic
        journal = self.receipts.journal
        result = self.receipts.result
        ros = self.ros
        rgb_ann = self.rgb_ann
        depth_ann = self.depth_ann
        camera_params_ann = self.camera_params_ann
        overview_ann = self.overview_ann
        camera = self.camera
        sensor_yaw = self.sensor_yaw
        mount = self.mount
        imu_mount = self.imu_mount
        target_speed = self.target_speed
        video_frames = self.video_frames
        # 标注器返回的数据是渲染缓冲视图：深度为 distance_to_image_plane
        # （米），RGB 只取前三通道，避免把 alpha 通道带进 512x512x3 契约。
        cached_rgb = None
        if self.rtx_lidar is not None:
            cached_rgb = self.rgb_outputs.take_rtx_frame()
            if cached_rgb is None:
                result["rtx_rgb_frames_skipped_stale_or_startup"] = result.get("rtx_rgb_frames_skipped_stale_or_startup", 0) + 1
                return
            image_time = cached_rgb["time_s"]
            image_ns = round(image_time * 1e9)
            if image_ns > round(actual_time * 1e9) + 1_000_000:
                raise RuntimeError("RTX RGB frame is later than physics clock")
            self.last_rgb_native_ns = image_ns
            it = image_time
            result["rtx_rgb_native_stamp_ns"] = image_ns
            result["rtx_rgb_acquisition_lag_ns"] = round(actual_time * 1e9) - image_ns
        zz = None if self.rtx_lidar is not None else np.asarray(depth_ann.get_data())
        rgb = cached_rgb["rgb"] if cached_rgb is not None else np.asarray(rgb_ann.get_data())[..., :3]
        result["rgb_rendered"] = True
        # 只在第 0/49 帧落渲染阶段收据，便于对照首帧与预热结束时的状态；
        # 其余帧不重复写，避免每帧产生小文件。
        if video_frames in (0, 49):
            from embodied_navigation.simulation.dynanav.render_diagnostics import (
                capture_stage_receipt,
            )

            (out / f"render_stage_{video_frames:04d}.json").write_text(
                json.dumps(capture_stage_receipt(stage), indent=2)
            )
        # CameraParams 给出的是 world→camera 的 view 矩阵，取逆才得到相机
        # 世界位姿；renderer_forward 用 -row2 是因为相机朝 -Z 看。
        camera_params = cached_rgb["camera_parameters"] if cached_rgb is not None else camera_params_ann.get_data()
        render_view = np.asarray(camera_params["cameraViewTransform"]).reshape(4, 4)
        render_world = np.linalg.inv(render_view)
        camera_pose_receipt = {
            "physics_s": actual_time,
            "rgb_source_s": it,
            "native_rgb_reference_time": cached_rgb.get("reference_time") if cached_rgb is not None else None,
            "native_rgb_time_evidence": cached_rgb.get("time_evidence") if cached_rgb is not None else None,
            "native_rgb_acquisition_physics_s": cached_rgb.get("acquisition_physics_s") if cached_rgb is not None else None,
            "renderer_view_matrix": render_view.tolist(),
            "renderer_camera_world_matrix": render_world.tolist(),
            "renderer_position": render_world[3, :3].tolist(),
            "renderer_forward": (-render_world[2, :3]).tolist(),
            "native_expected_position": camera_position.tolist(),
            "native_expected_forward": (Rb @ sensor_yaw[:, 0]).tolist(),
            "authored_usd_camera_world_matrix": np.asarray(
                UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(0)
            ).tolist(),
        }
        result["camera_render_pose"] = camera_pose_receipt
        journal("camera_render_pose.jsonl", camera_pose_receipt)
        if self.rtx_lidar is not None:
            native_route = getattr(self, "experiment", {}).get("interactive_controller") in ("native_route", "pct_scan")
            position_error = np.linalg.norm(render_world[3, :3] - camera_position)
            forward_cos = float(np.dot(-render_world[2, :3], Rb @ sensor_yaw[:, 0]))
            result["rtx_rgb_render_pose_check"] = {"position_error_m": float(position_error),
                                                   "forward_cosine": forward_cos,
                "renderer_source_s": float(it), "current_physics_s": float(actual_time),
                "comparison": "historical_renderer_metadata_vs_current_body_mount",
                "same_epoch_extrinsic_error_certified": False,
                "used_for_control": not native_route}
            # Native static exploration reuses the original route follower. This
            # later RTX check compares two different epochs: valid render latency
            # becomes an apparent mount error as soon as the body moves. Keep the
            # diagnostic and original event/buffer binding checks, but do not turn
            # a cross-epoch residual into an additional navigation stop.
            if (publish_to_slam
                    and not native_route
                    and actual_time > 2.0
                    and (position_error > 0.03 or forward_cos < np.cos(0.04))):
                error = RuntimeError("RTX RGB renderer pose does not follow the physical camera mount")
                try:
                    self.rgb_outputs.quarantine_native_frame(error, {
                        "valid": False, "rgb": rgb, "camera_parameters": camera_params,
                        "pose_check": camera_pose_receipt, "cached_frame": cached_rgb,
                    })
                except BaseException as secondary:
                    result["camera_quarantine_error"] = repr(secondary)
                raise error
        if self.rtx_lidar is not None:
            self._process_rtx_frame(
                it, actual_time, rgb, overview_ann, camera_rows, publish_to_slam=publish_to_slam
            )
            return
        # 这里的内参/外参与 sensors.create_camera_rig 的 USD 相机同源；
        # keep 是深度有效掩码，self_mask 是机身自回波掩码（按有效点计）。
        packet = depth_cloud_packet(zz, sensor_yaw, mount, imu_mount)
        xyz, keep, self_mask = (
            packet.points,
            packet.depth_keep_mask,
            packet.self_mask,
        )
        # 自回波盒为 base 系下的硬编码包围盒（m），与 perception/depth.py 的
        # 过滤盒一致；改动这里不会改实际过滤结果，只是收据字段。
        result["self_filter"] = {
            "body_box_min": [-0.5, -0.3, -0.45],
            "body_box_max": [0.5, 0.3, 0.3],
            "excluded_points_last": int(self_mask.sum()),
            "coordinates": "actual base axes, known fixed IMU offset",
        }
        # 1000 点是深度覆盖率的参考下限；gt_diagnostic 场景允许低覆盖，
        # 其余场景点数不足会触发下面的断言而不是静默继续。
        result["depth_quality"] = {
            "points": len(xyz),
            "finite_fraction": float(keep.mean()),
            "reference_min_points": 1000,
            "diagnostic_allows_low_coverage": gt_diagnostic,
        }
        if len(xyz) <= 1000:
            result["low_coverage_rgbd_frames"] = (
                result.get("low_coverage_rgbd_frames", 0) + 1
            )
        assert np.isfinite(xyz).all(), "Nonfinite RGB-D point"
        assert gt_diagnostic or len(xyz) > 1000, "Insufficient valid Go2 depth"
        if publish_to_slam:
            ros.publish_cloud(
                it,
                xyz,
                None,
                None,
                None,
                None,
            )
            ros.publish_depth_and_info(it, zz)
            if (
                self.experiment.get("control_mode") == "interactive_waypoint_exploration"
                and self.experiment.get("interactive_mailbox")
            ):
                self._write_interactive_still(float(actual_time), rgb)
        # 快照节奏：第 1000/1400/1600 帧是速度阶梯实验的固定检查点，
        # 另外每 1000 帧落一组；文件名用 1 基帧号。
        if (i + 1) in [1000, 1400, 1600] or (i + 1) % 1000 == 0:
            from PIL import Image as PILImage

            PILImage.fromarray(rgb).save(out / f"rgb_{i + 1}.png")
            np.savez_compressed(
                out / f"depth_{i + 1}.npz",
                depth=zz,
                base_R_world=Rb,
                base_position_world=pos,
                camera_local_to_world=np.array(
                    UsdGeom.Xformable(
                        camera.GetPrim()
                    ).ComputeLocalToWorldTransform(0)
                ),
                imu_mount=imu_mount,
                mount=mount,
                imu_points=xyz,
            )
        overview = np.asarray(overview_ann.get_data())[..., :3]
        assert overview.shape == (512, 512, 3), "Invalid overview frame"
        video_frames = self.video.write(
            overview, rgb, actual_time, target_speed, gt_diagnostic
        )
        if publish_to_slam:
            ros.publish_rgb(it, rgb)
            camera_rows.append(
                {
                    "time": actual_time,
                    "published_time": it,
                    "valid_depth_points": len(xyz),
                    "finite_depth_fraction": float(keep.mean()),
                    # 世界系高度 >0.12 m 的非地面点数，仅用于离线评分诊断，
                    # 不参与导航或安全判定。
                    "world_nonground_points_scoring_only": int(
                        (((xyz @ Rb.T + sensor_position)[:, 2]) > 0.12).sum()
                    ),
                }
            )
            journal("camera_frames.jsonl", camera_rows[-1])

        self.video_frames = video_frames
