"""Opt-in count-selectable native RGB/depth capture. Primary estimator stays single-camera."""
from __future__ import annotations
from collections import deque
import copy
import json
import time
import numpy as np
from .sensors import RgbdRender, _reference_pair
from ..multicamera import camera_calibration, validate_intrinsics, RESOLUTION, APERTURE_MM


class MultiViewRender(RgbdRender):
    def _accept_native_render_frame(self, *, bound_data, source_s, reference_time,
                                    acquisition_physics_s=None, time_evidence=None):
        _reference_pair(reference_time)
        if not np.isfinite(source_s) or source_s < 0:
            raise ValueError('Invalid multi-view native timestamp')
        ns = round(source_s * 1e9)
        if ns < getattr(self, 'minimum_stamp_ns', 0) or ns == self.rtx_last_native_ns:
            return
        if self.rtx_last_native_ns is not None and ns < self.rtx_last_native_ns:
            raise ValueError('Multi-view native time moved backwards')
        validate_intrinsics(bound_data['camera_params_ann'], self.calibration)
        rgb = np.asarray(bound_data['rgb_ann'])
        depth = np.asarray(bound_data['depth_ann'])
        if rgb.shape not in ((512, 512, 3), (512, 512, 4)) or depth.shape != (512, 512):
            raise ValueError('Unpaired multi-view RGB/depth dimensions')
        if len(self.rtx_cache) >= 4:
            raise RuntimeError('Multi-view pending queue overflow; no silent reuse')
        self.rtx_cache.append(dict(stamp_ns=ns, source_time_s=float(source_s),
                                  native_reference_time=list(reference_time),
                                  acquisition_physics_s=acquisition_physics_s,
                                  time_evidence=copy.deepcopy(time_evidence),
                                  rgb=rgb[..., :3].copy(), depth=depth.astype(np.float32, copy=True),
                                  native_camera_parameters={key: np.asarray(bound_data['camera_params_ann'][key]).tolist()
                                      for key in ('renderProductResolution', 'cameraAperture', 'cameraFocalLength',
                                                  'cameraApertureOffset', 'metersPerSceneUnit')},
                                  native_camera_model=bound_data['camera_params_ann']['cameraModel']))
        self.rtx_last_native_ns = ns
        self.accepted_frames = getattr(self, 'accepted_frames', 0) + 1

    def reset_start(self, stamp_ns):
        self.minimum_stamp_ns = stamp_ns
        self.accepted_frames = 0
        self.rtx_cache.clear()
        self.rtx_last_native_ns = None
        if hasattr(self, '_pending_native_frames'):
            self.reset_native_binding()


class MultiCameraCapture:
    def __init__(self, views, output, primary):
        self.views, self.output, self.primary = views, output, primary
        self.last_saved = {}
        self.counts = {view.calibration['camera_id']: 0 for view in views}
        self.output.mkdir(parents=True, exist_ok=True)
        manifest = dict(schema='dynanav-multicamera/v1', generation=str(output.parent.resolve()),
                        cameras=[v.calibration for v in views], camera_count=len(views), archive_interval_s=1.,
                        primary_slam_input='camera_0 RGB only; explicit experimental optics apply to all selected cameras',
                        additional_camera_slam_input=False, independent_vio_implemented=False,
                        pose_fusion_implemented=False, all_direction_visibility_guaranteed=False,
                        angular_coverage=f"{len(views)} {views[0].calibration['horizontal_fov_deg']:.6g} degree horizontal pinholes at {360./len(views):g} degree intervals; body/scene occlusions remain",
                        scene_geometry_or_renderer_world_pose_used=False)
        (output / 'calibration.json').write_text(json.dumps(manifest, indent=2))
        for view in views:
            view.quarantine_output = output.parent

    def reset_start(self, stamp_ns):
        for view in self.views:
            view.reset_start(stamp_ns)

    def capture(self, minimum_stamp_ns):
        for view in self.views[1:]:
            view.capture_rtx_frame(minimum_stamp_ns)

    def write_pending(self, receipts, physics_s):
        from PIL import Image
        for view in self.views:
            if view.rtx_error is not None:
                raise RuntimeError('Multi-view capture failed: ' + view.calibration['camera_id']) from view.rtx_error
            camera_id = view.calibration['camera_id']
            while view.rtx_cache:
                frame = view.rtx_cache.popleft()
                ns = frame['stamp_ns']
                if ns > round(physics_s * 1e9) + 1_000_000:
                    raise ValueError('Multi-view frame is later than physics clock')
                if ns - self.last_saved.get(camera_id, -10**18) < 1_000_000_000:
                    continue
                self.last_saved[camera_id] = ns
                directory = self.output / camera_id
                directory.mkdir(exist_ok=True)
                name = f'{ns:018d}'
                Image.fromarray(frame.pop('rgb').astype(np.uint8)).save(directory / (name + '.png'))
                np.savez_compressed(directory / (name + '.npz'), axial_depth_m=frame.pop('depth'))
                frame.update(camera_id=camera_id, calibration_id=camera_id, rgb_path=f'{camera_id}/{name}.png',
                             depth_path=f'{camera_id}/{name}.npz', generation=str(self.output.parent.resolve()),
                             frame_id=view.calibration['optical_frame'], depth_type='distance_to_image_plane',
                             world_pose_used=False, slam_input=(camera_id == 'camera_0'),
                             extra_depth_slam_or_planner_input=False, queue_drained_physics_s=physics_s,
                             saved_wall_perf_counter_s=time.perf_counter())
                with (self.output / 'index.jsonl').open('a') as handle:
                    handle.write(json.dumps(frame, default=lambda x: x.item() if isinstance(x, np.generic) else str(x)) + '\n')
                self.counts[camera_id] += 1
        receipts.result['multicamera_frames_archived'] = dict(self.counts)

    def close(self):
        self.primary.bound_frame_observer = None
        summary = {}
        for index, view in enumerate(self.views):
            native = self.primary if index == 0 else view
            summary[view.calibration['camera_id']] = dict(
                accepted_native_frames=getattr(view, 'accepted_frames', 0),
                archived_frames=self.counts[view.calibration['camera_id']],
                callbacks=getattr(native, 'rtx_callback_count', 0),
                invalid_frame_count=getattr(native, 'native_frame_drop_count', 0),
                capture_error=None if native.rtx_error is None else repr(native.rtx_error),
                pending_frames=len(view.rtx_cache))
        for view in self.views[1:]:
            view.stop_native_capture()
        return summary


def create_multicamera(stage, rig, primary, experiment, output):
    if not experiment.get('multicamera_observation', False):
        return None
    if not rig.rtx_enabled or primary.depth_ann is None:
        raise ValueError('Multi-camera observations require native RTX and primary paired depth')
    count = experiment.get('multicamera_count', 6)
    if type(count) is not int or count not in range(3, 7):
        raise ValueError('Multi-view camera count must be an integer within 3..6')
    from pxr import Gf, UsdGeom
    from isaacsim.sensors.experimental.rtx import RtxCamera
    rep = primary.rep
    views = []
    for index in range(count):
        calibration = camera_calibration(index, rig.imu_mount, primary_yaw=rig.theta, primary_mount=rig.mount, camera_count=count,
                                         horizontal_fov_deg=experiment.get('multicamera_horizontal_fov_deg'))
        if index == 0:
            # Use exactly the primary render bundle, not a second front camera.
            view = MultiViewRender(rep, primary.product, primary.rgb_ann, primary.depth_ann, primary.camera_params_ann)
            view.rtx_cache = deque()
            primary.bound_frame_observer = view._accept_native_render_frame
        else:
            path = rig.base_path + f'/MultiViewCamera{index}'
            camera = UsdGeom.Camera.Define(stage, path)
            R_BC = np.asarray(calibration['R_base_from_optical'])
            rows = (R_BC @ np.diag([1., -1., -1.])).T
            camera.AddTransformOp().Set(Gf.Matrix4d(Gf.Matrix3d(*rows.flatten().tolist()),
                                       Gf.Vec3d(*calibration['translation_base_m'])))
            camera.GetHorizontalApertureAttr().Set(APERTURE_MM)
            camera.GetVerticalApertureAttr().Set(APERTURE_MM)
            camera.GetFocalLengthAttr().Set(calibration['authored_focal_length'])
            camera.GetClippingRangeAttr().Set(Gf.Vec2f(.05, 100.))
            camera.GetPrim().ApplyAPI('OmniSensorAPI')
            scheduler = RtxCamera(path, tick_rate=10., reset_xform_op_properties=False)
            product = rep.create.render_product(path, RESOLUTION)
            def annotator(name):
                value = rep.AnnotatorRegistry.get_annotator(name)
                value.attach([product])
                return value
            view = MultiViewRender(rep, product, annotator('rgb'), annotator('distance_to_image_plane'),
                                   annotator('CameraParams'), annotator('IsaacReadSimulationTime'),
                                   recover_invalid_native_frames=True)
            view.native_observer_name = f'dynanav.multiview_{index}'
            view.scheduler_sensor, view.camera = scheduler, camera
            view.attach_rtx_writer()
        view.calibration = calibration
        views.append(view)
    return MultiCameraCapture(views, output / 'multicamera', primary)
