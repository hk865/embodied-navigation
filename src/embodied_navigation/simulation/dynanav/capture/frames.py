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
    ):
        self.experiment = experiment
        self.out = out
        self.stage = stage
        self.gt_diagnostic = gt_diagnostic
        self.mapping = mapping
        self.ros = ros
        self.video = video
        self.rgb_ann = rgb_annotators.rgb_ann
        self.depth_ann = rgb_annotators.depth_ann
        self.camera_params_ann = rgb_annotators.camera_params_ann
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
        zz = np.asarray(depth_ann.get_data())
        rgb = np.asarray(rgb_ann.get_data())[..., :3]
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
        camera_params = camera_params_ann.get_data()
        render_view = np.asarray(camera_params["cameraViewTransform"]).reshape(4, 4)
        render_world = np.linalg.inv(render_view)
        camera_pose_receipt = {
            "physics_s": actual_time,
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
