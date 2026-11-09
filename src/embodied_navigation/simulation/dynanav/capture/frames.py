"""10 Hz RGB-D frame publication and per-frame evidence receipts.

This module owns the sensor-frame pipeline: read the RGB/depth/CameraParams
annotators, archive raw RGB, run the mapping sinks, publish the unchanged ROS
clouds/depth/info/RGB messages, record camera render-pose and self-filter
receipts, save periodic RGB-D snapshots, and draw the overview/RGB video panel.
All cadences, journal payloads and assertions are unchanged.
"""

import json

import numpy as np

from embodied_navigation.perception.depth import depth_cloud_packet


class FrameStream:
    """Own the 10 Hz frame cadence counters and per-frame evidence writes."""

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
        """Process one 10 Hz RGB-D frame exactly as the original loop block."""
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
        if video_frames in (0, 49):
            from embodied_navigation.simulation.dynanav.render_diagnostics import (
                capture_stage_receipt,
            )

            (out / f"render_stage_{video_frames:04d}.json").write_text(
                json.dumps(capture_stage_receipt(stage), indent=2)
            )
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
        packet = depth_cloud_packet(zz, sensor_yaw, mount, imu_mount)
        xyz, keep, self_mask = (
            packet.points,
            packet.depth_keep_mask,
            packet.self_mask,
        )
        result["self_filter"] = {
            "body_box_min": [-0.5, -0.3, -0.45],
            "body_box_max": [0.5, 0.3, 0.3],
            "excluded_points_last": int(self_mask.sum()),
            "coordinates": "actual base axes, known fixed IMU offset",
        }
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
        # Complete obstacle cloud remains visible to navigation, including pedestrians.
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
                "world_nonground_points_scoring_only": int(
                    (((xyz @ Rb.T + sensor_position)[:, 2]) > 0.12).sum()
                ),
            }
        )
        journal("camera_frames.jsonl", camera_rows[-1])

        self.rgb_frames_acquired = rgb_frames_acquired
        self.video_frames = video_frames
