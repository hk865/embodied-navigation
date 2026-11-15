"""Optional native-render depth diagnostic. Not connected to SLAM or control."""
from __future__ import annotations

import copy
import hashlib
import json
import time

import numpy as np

from .nearfield import (APERTURE_MM, FOCAL_MM, MOUNT, R_BASE_OPTICAL, RESOLUTION,
                        SCHEMA, SOURCE, backproject, calibration_receipt,
                        point_coverage, reference_pair, validate_camera_parameters,
                        verify_reference_pair, write_frame)
from .sensors import RgbdRender


class NearfieldDepthRender(RgbdRender):
    """Reuse the already checked native RP event clock, not the RGB timestamp."""
    native_observer_name = "dynanav.native_nearfield_depth_frame"

    def _accept_native_render_frame(self, *, bound_data, source_s, reference_time,
                                    acquisition_physics_s=None, time_evidence=None):
        begin = time.perf_counter_ns()
        reference_pair(reference_time)
        if not np.isfinite(source_s) or source_s < 0:
            raise ValueError("Invalid nearfield native simulation time")
        ns = round(source_s * 1e9)
        if ns < getattr(self, "minimum_stamp_ns", 0) or ns == self.rtx_last_native_ns:
            return
        if self.rtx_last_native_ns is not None and ns < self.rtx_last_native_ns:
            raise ValueError("Native nearfield time moved backwards")
        depth = np.asarray(bound_data['depth_ann'])
        instances = bound_data['instance_ann']
        parameters = bound_data['camera_params_ann']
        if depth.size == 0 or not isinstance(instances, dict) or not parameters:
            self.startup_skipped = getattr(self, "startup_skipped", 0) + 1
            return
        # These owned arrays passed the shared before/after RP binding checks.
        # CameraParams is renderer metadata, not an asserted pixel-tick pose.
        time_evidence = dict(time_evidence or {})
        verify_reference_pair(reference_time, time_evidence["binding_after"]["reference_time"])
        time_evidence.update(
            conversion="get_simulation_time_at_time(native_reference_time); round(seconds*1e9)",
            reference_after_reads=list(time_evidence["binding_after"]["reference_time"]),
            annotator_pairing="owned copies after automatic SDK SDG; native RP time and render results checked before/after",
        )
        validate_camera_parameters(parameters)
        arrays, masks = backproject(depth, instances)
        labels = {str(k): v for k, v in instances["info"]["idToLabels"].items()}
        mapping_text = json.dumps(labels, sort_keys=True, separators=(",", ":"))
        if len(self.rtx_cache) >= 4:
            raise RuntimeError("Native nearfield queue overflow; refusing silent frame loss")
        arrival = float(time_evidence["original_arrival_wall_s"])
        self.rtx_cache.append((arrays, {
            "schema": SCHEMA, "source": SOURCE, "stamp_ns": ns,
            "source_time_s": float(source_s), "arrival_wall_s": arrival,
            "arrival_clock": "perf_counter", "frame_id": "go2_base",
            "native_reference_time": list(reference_time),
            "depth_reference_time": list(reference_time), "mask_reference_time": list(reference_time),
            "time_evidence": copy.deepcopy(time_evidence),
            "acquisition_physics_s": acquisition_physics_s,
            "calibration": calibration_receipt(), "calibration_verified_from_render_parameters": True,
            "depth_type": "distance_to_image_plane",
            "self_mask": dict(masks, method="same_frame_instance_prim_path", same_frame=True,
                              robot_prim_root="/World/Go2", instance_id_to_prim=labels,
                              mapping_sha256=hashlib.sha256(mapping_text.encode()).hexdigest()),
            "diagnostic_point_coverage": point_coverage(arrays["points_base_m"]),
            "control_enabled": False, "slam_input": False, "map_input": False,
            "environment_ground_truth_used": False, "world_pose_used": False,
            "queue_depth_after_enqueue": len(self.rtx_cache) + 1,
            "capture_processing_ns": time.perf_counter_ns() - begin,
            "simulated_hardware": True,
        }))
        # Kept in a separate diagnostic journal, not in the navigation NPZ/JSON.
        self.renderer_poses = getattr(self, "renderer_poses", {})
        self.renderer_poses[ns] = np.asarray(parameters["cameraViewTransform"]).reshape(4, 4).tolist()
        self.rtx_last_native_ns = ns

    def reset_start(self, minimum_stamp_ns):
        self.minimum_stamp_ns = int(minimum_stamp_ns)
        self.rtx_cache.clear()
        self.rtx_last_native_ns = None
        if hasattr(self, '_pending_native_frames'):
            self.reset_native_binding()
        self.renderer_poses = {}

    def write_pending(self, output, receipts, physics_s):
        self.quarantine_output = output
        if hasattr(self, '_binding_quarantine'):
            self._flush_native_quarantine()
        if self.rtx_error is not None:
            raise RuntimeError("Native nearfield capture failed") from self.rtx_error
        while self.rtx_cache:
            arrays, record = self.rtx_cache.popleft()
            renderer_view = self.renderer_poses.pop(record["stamp_ns"])
            receipts.journal("nearfield_renderer_pose_diagnostic.jsonl", {
                "stamp_ns": record["stamp_ns"], "cameraViewTransform": renderer_view,
                "diagnostic_only": True, "navigation_input": False,
                "expected_local_calibration": calibration_receipt(),
            })
            lag = round(physics_s * 1e9) - record["stamp_ns"]
            if lag < -1_000_000:
                raise ValueError("Native nearfield frame later than physics clock")
            record["drain_physics_lag_ns"] = lag
            record["queue_residence_ns"] = round((time.perf_counter() - record["arrival_wall_s"]) * 1e9)
            record = write_frame(output / "nearfield_depth", arrays, record)
            receipts.journal("nearfield_depth_frames.jsonl", record)
            receipts.result["nearfield_depth_last_frame"] = record
            receipts.result["nearfield_depth_frames"] = receipts.result.get("nearfield_depth_frames", 0) + 1


def create_nearfield_depth(stage, rig, experiment):
    if not experiment.get("simulated_nearfield_depth", False):
        return None
    if experiment.get("point_cloud_source") != "rtx_lidar":
        raise ValueError("Nearfield diagnostic requires the unchanged RTX primary pipeline")
    from pxr import Gf, UsdGeom
    from isaacsim.sensors.experimental.rtx import RtxCamera
    import omni.replicator.core as rep

    if not np.isclose(float(UsdGeom.GetStageMetersPerUnit(stage)), 1.):
        raise ValueError("Nearfield static calibration requires metre stage units")
    path = rig.base_path + "/SimulatedNearfieldDepthCamera"
    camera = UsdGeom.Camera.Define(stage, path)
    rotation_rows = (R_BASE_OPTICAL @ np.diag([1., -1., -1.])).T
    camera.AddTransformOp().Set(Gf.Matrix4d(Gf.Matrix3d(*rotation_rows.flatten().tolist()), Gf.Vec3d(*MOUNT)))
    camera.GetHorizontalApertureAttr().Set(APERTURE_MM)
    camera.GetVerticalApertureAttr().Set(APERTURE_MM)
    camera.GetFocalLengthAttr().Set(FOCAL_MM)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(.05, 5.0))
    camera.GetPrim().ApplyAPI("OmniSensorAPI")
    # FSD multi-tick scheduler registration is mandatory, just like front RGB.
    scheduler_sensor = RtxCamera(path, tick_rate=10.0, reset_xform_op_properties=False)
    product = rep.create.render_product(path, RESOLUTION)

    def annotator(name):
        result = rep.AnnotatorRegistry.get_annotator(name)
        result.attach([product])
        return result

    # Legacy adapter wraps the SDK fast instance-ID image and exposes idToLabels.
    # It does NOT require semantic labels on any environment or robot prim.
    instance_ann = annotator("instance_id_segmentation")
    result = NearfieldDepthRender(rep=rep, product=product, rgb_ann=None,
                                 depth_ann=annotator("distance_to_image_plane"),
                                 camera_params_ann=annotator("CameraParams"),
                                 time_ann=annotator("IsaacReadSimulationTime"))
    result.instance_ann = instance_ann
    result.camera = camera
    result.scheduler_sensor = scheduler_sensor
    result.attach_rtx_writer()
    return result
