"""10 Hz RGB-D 帧发布与逐帧证据收据。

本模块负责传感器帧流水线：读取 RGB/深度/CameraParams 标注器、归档原始 RGB、
运行建图 sink、按原样发布 ROS 点云/深度/内参/RGB 消息、记录相机渲染位姿与
自过滤收据、定期保存 RGB-D 快照，并绘制 overview/RGB 视频画面。所有节奏、
journal 字段与断言均保持原样；“10 Hz”是由上层每 20 个 0.005 s 物理步调用
一次本模块形成的，本文件不自行计时。
"""

import json

import numpy as np

from embodied_navigation.perception.depth import depth_cloud_packet


class FrameStream:
    """持有 10 Hz 帧计数与逐帧证据写入。

    ``rgb_frames_acquired`` 统计归档的 RGB 帧，``video_frames`` 由视频写入器
    返回的下一帧序号；两者跨帧保留，用于快照与阶段收据。
    """

    def __init__(
        self,
        *,
        experiment,
        out,
        stage,
        gt_diagnostic,
        mapping,
        ros,
        video,
        rgb_annotators,
        overview,
        camera_rig,
        target_speed,
        receipts,
        rgb_archive,
        rtx_lidar=None,
        nearfield_depth=None,
        multicamera=None,
    ):
        self.experiment = experiment
        self.rtx_lidar = rtx_lidar
        self.nearfield_depth = nearfield_depth
        self.multicamera = multicamera
        self.rgb_outputs = rgb_annotators
        for output in (rgb_annotators, nearfield_depth):
            if output is not None:
                output.quarantine_output = out
                if hasattr(output, "_binding_quarantine"):
                    output._flush_native_quarantine()
        self.out = out
        self.stage = stage
        self.gt_diagnostic = gt_diagnostic
        self.mapping = mapping
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
        self.rgb_archive = rgb_archive
        self.rgb_frames_acquired = 0
        self.video_frames = 0

    def _publish_interactive_perception(self, packet, actual_time, rgb, depth):
        """Publish live static-perception clearance for the interactive operator.

        Only the explicit interactive mode produces this file. The producer
        packet carries IMU-frame points together with their original pixel
        indices; the payload builder translates them into the base frame and
        gathers the validity/self masks through those indices. A pairing or
        calibration failure becomes a fail-closed sample, never a clearance.
        """
        experiment = self.experiment
        mailbox = experiment.get("interactive_mailbox")
        if (
            experiment.get("control_mode") != "interactive_waypoint_exploration"
            or not mailbox
        ):
            return
        import json as _json
        import os as _os
        import pathlib as _pathlib
        import time as _time

        depth_array = np.asarray(depth, dtype=float)
        valid_pixels = depth_array[np.isfinite(depth_array) & (depth_array > 0.05)]
        depth_valid_ratio = float(valid_pixels.size) / float(max(1, depth_array.size))
        # The payload builder owns the IMU->base translation and the pixel-index
        # gather. It never guesses: an unpaired packet becomes a fail-closed
        # sample whose observation state makes the tracker hold.
        from .perception import interactive_perception_payload

        payload = interactive_perception_payload(
            packet,
            imu_mount=self.imu_mount,
            stamp_s=float(actual_time),
            arrival_wall_s=_time.perf_counter(),
            depth_valid_ratio=depth_valid_ratio,
        )
        path = _pathlib.Path(mailbox) / "perception.json"
        temporary = path.with_name(path.name + ".tmp")
        try:
            temporary.write_text(_json.dumps(payload, sort_keys=True))
            _os.replace(temporary, path)
        except OSError:
            pass
        self._write_interactive_still(float(actual_time), rgb)

    def _write_interactive_still(self, source_time, rgb, *, perception=None):
        """Low-rate real camera still; cadence is wall/sim time based.

        The image is operator evidence only: it carries its own source time and,
        optionally, the current perception sample; never a pose or ground truth.
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
            if perception is not None:
                record["perception"] = perception
            with (frames_dir / "index.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(_json.dumps(record, sort_keys=True) + "\n")
            return record
        except (OSError, ImportError, ValueError):
            return None

    def _read_external_perception(self, actual_time):
        """Report the external perception sample ages; never refresh or rewrite it.

        When the native deskewed-cloud producer owns perception.json, this capture
        process only records what it currently sees. A stopped producer leaves an
        old arrival and source stamp, so the tracker age gates keep holding; the
        capture never re-stamps a sample to keep it alive.
        """
        import json as _json
        import pathlib as _pathlib
        import time as _time

        mailbox = self.experiment.get("interactive_mailbox")
        if not mailbox:
            return None
        path = _pathlib.Path(mailbox) / "perception.json"
        try:
            payload = _json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, ValueError):
            return None
        if not isinstance(payload, dict):
            return None
        stamp = payload.get("stamp_s")
        arrival = payload.get("arrival_wall_s")
        geometry = payload.get("lidar_geometry") or {}
        summary = {
            "source": payload.get("source"),
            "stamp_s": stamp,
            "arrival_wall_s": arrival,
            "observation_state": payload.get("observation_state"),
            "clearance_m": payload.get("clearance_m"),
            "fail_closed": payload.get("fail_closed"),
            "producer_metadata": payload.get("producer_metadata"),
            "blind_azimuth_sectors_deg": geometry.get("blind_azimuth_sectors_deg"),
            "near_field_floor_unobserved_m": geometry.get("near_field_floor_unobserved_m"),
        }
        try:
            summary["sim_age_s"] = (
                None if stamp is None else float(actual_time) - float(stamp)
            )
        except (TypeError, ValueError):
            summary["sim_age_s"] = None
        try:
            summary["wall_age_s"] = (
                None if arrival is None else _time.perf_counter() - float(arrival)
            )
        except (TypeError, ValueError):
            summary["wall_age_s"] = None
        return summary
    def publish_rtx_pending(self, actual_time):
        """LiDAR publication is independent of RGB buffer/camera cadence."""
        multicamera = getattr(self, "multicamera", None)
        if multicamera is not None:
            multicamera.write_pending(self.receipts, actual_time)
        nearfield = getattr(self, "nearfield_depth", None)
        if nearfield is not None:
            nearfield.write_pending(self.out, self.receipts, actual_time)
        packet = self.rtx_lidar.read()
        result = self.receipts.result
        result["point_cloud_source"] = "isaac_rtx_lidar"
        result["depth_backprojection_enabled"] = False
        while packet is not None:
            # Physics clock must already cover the final native firing time.
            last_retained_ns = packet.receipt["last_retained_ns"]
            if last_retained_ns is not None and last_retained_ns > round(actual_time * 1e9) + 1_000_000:
                raise ValueError("RTX scan contains rays later than the current physics clock")
            self.ros.publish_rtx_cloud(packet)
            evidence = dict(packet.receipt, acquisition_physics_ns=round(actual_time * 1e9))
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
        if not result.get("rtx_lidar_connected"):
            result["rtx_lidar_reads_without_new_scan"] = result.get("rtx_lidar_reads_without_new_scan", 0) + 1
    def _process_rtx_frame(self, it, actual_time, rgb, overview_ann, camera_rows):
        """RGB stays visual input; the only primary cloud is native RTX LiDAR."""
        import os
        import time
        import pathlib
        self.publish_rtx_pending(actual_time)
        result = self.receipts.result
        # The perception sample is owned by either this capture (legacy fail-closed
        # payload) or the native deskewed-cloud producer. A silent producer must
        # never be turned into a fresh sample: the capture does not rewrite or
        # re-stamp perception.json for the RTX deskewed-cloud owner, so a missing or
        # old file keeps the tracker original age gates holding.
        mailbox = self.experiment.get("interactive_mailbox")
        external_perception = None
        if (
            self.experiment.get("control_mode") == "interactive_waypoint_exploration"
            and mailbox
            and self.experiment.get("interactive_controller") not in ("native_route", "pct_scan")
        ):
            owner = self.experiment.get("interactive_perception_owner", "fail_closed")
            result["interactive_perception_owner"] = owner
            if owner == "rtx_deskewed_cloud":
                external_perception = self._read_external_perception(actual_time)
                result["interactive_external_perception"] = external_perception
            else:
                from .perception import fail_closed_perception_payload
                payload = fail_closed_perception_payload(
                    stamp_s=actual_time, arrival_wall_s=time.perf_counter(),
                    error="Raw rotating LiDAR requires deskewed perception; RGB-D perception is disabled",
                )
                payload["source"] = "rtx_lidar_perception_not_connected"
                path = pathlib.Path(mailbox) / "perception.json"
                temporary = path.with_name(path.name + ".tmp")
                temporary.write_text(json.dumps(payload, sort_keys=True))
                os.replace(temporary, path)
        still = self._write_interactive_still(float(it), rgb, perception=external_perception)
        if still is not None:
            result["interactive_still"] = still
        overview = np.asarray(overview_ann.get_data())[..., :3]
        assert overview.shape == (512, 512, 3), "Invalid overview frame"
        self.video_frames = self.video.write(overview, rgb, it, self.target_speed, self.gt_diagnostic)
        self.ros.publish_rgb(it, rgb)
        camera_rows.append({"time": actual_time, "published_time": it,
                            "point_cloud_source": "isaac_rtx_lidar", "depth_backprojection_enabled": False})
        self.receipts.journal("camera_frames.jsonl", camera_rows[-1])

    def process(
        self,
        *,
        iteration,
        measurement,
        pedestrian_view,
        pedestrian_target,
        dynamic_guard,
        camera_rows,
    ):
        """按原循环块处理一帧 10 Hz RGB-D 数据。

        ``measurement`` 提供本帧对应的物理时间与位姿；``pedestrian_view``
        非空时额外读取原生行人位姿并写入收据；``dynamic_guard`` 与
        ``camera_rows`` 分别传给建图与逐帧诊断日志。断言失败即终止采集。
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
        oracle_mapper = self.mapping.oracle_mapper
        learned_mapper = self.mapping.learned_mapper
        journal = self.receipts.journal
        result = self.receipts.result
        ros = self.ros
        latest_odom = ros.latest_odom
        rgb_ann = self.rgb_ann
        depth_ann = self.depth_ann
        camera_params_ann = self.camera_params_ann
        overview_ann = self.overview_ann
        camera = self.camera
        sensor_yaw = self.sensor_yaw
        mount = self.mount
        imu_mount = self.imu_mount
        target_speed = self.target_speed
        rgb_archive = self.rgb_archive
        rgb_frames_acquired = self.rgb_frames_acquired
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
        if rgb_archive is not None:
            rgb_frames_acquired += 1
            self.rgb_frames_acquired = rgb_frames_acquired
            rgb_archive.write(
                rgb,
                source_ns=round(it * 1e9),
                frame_index=video_frames,
                frame_id="go2_camera_optical",
            )
            result["rgb_archive"] = rgb_archive.receipt(
                acquired_frames=rgb_frames_acquired
            )
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
            if (not native_route
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
            self._process_rtx_frame(it, actual_time, rgb, overview_ann, camera_rows)
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
        # 完整障碍点云仍对导航可见（包含行人）；语义过滤只影响建图/评分消费方。
        states, frontend_keep = self.mapping.process_frame(
            packet=packet,
            xyz=xyz,
            rgb=rgb,
            zz=zz,
            it=it,
            actual_time=actual_time,
            Rb=Rb,
            sensor_position=sensor_position,
            latest_odom=latest_odom,
            dynamic_guard=dynamic_guard,
            result=result,
            journal=journal,
        )
        if pedestrian_view is not None:
            from isaacsim.core.experimental.utils import (
                backend as backend_utils,
            )

            # 行人原生位姿必须走 tensor 后端读回；不支持或回退时直接报错，
            # 避免用另一种后端的结果冒充同源证据。
            with backend_utils.use_backend(
                "tensor", raise_on_unsupported=True, raise_on_fallback=True
            ):
                person_position, person_orientation = (
                    pedestrian_view.get_world_poses()
                )
            native_person = person_position.numpy()[0].copy()
            person_receipt = {
                "physics_s": actual_time,
                "target": pedestrian_target,
                "native_position": native_person.tolist(),
                "native_target_error_m": float(
                    np.linalg.norm(native_person - pedestrian_target["position_m"])
                ),
                "readback_backend": "tensor",
                "physics_step_delta": result["last_clock_check"][
                    "step_count_delta"
                ],
            }
            result["pedestrian_readback"] = person_receipt
            journal("pedestrian_native.jsonl", person_receipt)
        ros.publish_cloud(
            it,
            xyz,
            states,
            frontend_keep,
            oracle_mapper,
            learned_mapper,
        )
        ros.publish_depth_and_info(it, zz)
        self._publish_interactive_perception(packet, actual_time, rgb, zz)
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

        self.rgb_frames_acquired = rgb_frames_acquired
        self.video_frames = video_frames
