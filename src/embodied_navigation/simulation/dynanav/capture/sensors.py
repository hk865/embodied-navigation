"""采集探针的相机、IMU 与渲染产品安装。

本模块负责物理传感器装置本身：相机安装几何与 authored USD 相机、Isaac IMU
传感器、replicator 的 RGB/深度/CameraParams 标注器，以及第三人称 overview
相机与渲染产品。只返回普通值对象（``CameraRig``/``RgbdRender``/
``OverviewRender``）与安装收据，不启动应用、不打开 stage。
"""

from dataclasses import dataclass

import numpy as np


def _reference_pair(value):
    import numbers
    if (not isinstance(value, (tuple, list)) or len(value) != 2 or
            any(isinstance(v, bool) or not isinstance(v, numbers.Integral) for v in value) or
            value[0] < 0 or value[1] <= 0):
        raise ValueError('Missing or invalid native camera reference time')
    return int(value[0]), int(value[1])


def _owned_camera_value(value, budget=None):
    """Own SDK buffers and metadata; bounded even on the quarantine path."""
    import copy
    if budget is None:
        budget = [32 * 1024 * 1024]
    if isinstance(value, np.ndarray):
        budget[0] -= value.nbytes
        if budget[0] < 0:
            raise ValueError('Native camera snapshot exceeds 32 MiB budget')
        return value.copy()
    if isinstance(value, dict):
        return {k: _owned_camera_value(v, budget) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_owned_camera_value(v, budget) for v in value]
    return copy.deepcopy(value)


def _camera_binding_attributes(rp_time_ann, annotators):
    """Resolve public OmniGraph data dependencies, not guessed node names.

    Each pixel/metadata producer must consume the same RP dispatcher result as
    rpFabricTime. Exec edges are not data provenance and are not traversed.
    Producer exec is only an auxiliary enabled-state check; frame freshness
    still requires the original event, RP rational time and result binding.
    """
    rp = rp_time_ann.get_node().get_attribute('inputs:rp')
    connections = rp.get_upstream_connections()
    if len(connections) != 1 or connections[0].get_name() != 'outputs:renderResults':
        raise ValueError('rpFabricTime has no unique render-product result source')
    dispatcher = connections[0].get_node()
    product = dispatcher.get_attribute('outputs:renderProductPath')
    if not product.is_valid():
        raise ValueError('Native camera dispatcher has no render product identity')
    expected_source = connections[0].get_path()
    dispatcher_path = str(dispatcher.get_prim_path())
    result = {rp.get_path(): rp}
    execution = {}
    def completed_outputs(node, *, required=False):
        outputs = [attr for attr in node.get_attributes()
                   if attr.get_name() in ('outputs:exec', 'outputs:execOut')]
        if required and not outputs:
            raise ValueError('Camera data producer has no completion output')
        execution.update({attr.get_path(): attr for attr in outputs})
    completed_outputs(rp_time_ann.get_node(), required=True)
    for name, annotator in annotators.items():
        root = annotator.get_node()
        completed_outputs(root, required=True)
        pending, visited, found = [root], set(), 0
        while pending:
            node = pending.pop()
            path = str(node.get_prim_path())
            if path in visited:
                continue
            # The verified RP dispatcher is the source boundary, not an image
            # producer. Its scalar productPath/cudaStream edges also reach the
            # global dispatcher. Their exec outputs are per-event flow pulses,
            # not persistent image readiness, so never walk beyond this source.
            # Keep its product identity and renderResults checks independently.
            if path == dispatcher_path:
                continue
            visited.add(path)
            completed_outputs(node)
            if len(visited) > 128:
                raise ValueError('Native camera data dependency graph exceeds budget')
            for attr in node.get_attributes():
                attr_name = attr.get_name()
                if not attr_name.startswith('inputs:') or 'exec' in attr_name.lower():
                    continue
                upstream = attr.get_upstream_connections()
                if attr_name == 'inputs:renderResults':
                    completed_outputs(node, required=True)
                    if len(upstream) != 1 or upstream[0].get_path() != expected_source:
                        raise ValueError('Camera annotator depends on a different render product')
                    result[attr.get_path()] = attr
                    found += 1
                else:
                    pending.extend(edge.get_node() for edge in upstream)
        if not found:
            raise ValueError('Camera annotator has no render-result data provenance: ' + name)
    return product, result, execution


class _CameraListeners:
    """Dropping the existing rtx_listener releases every new observer too."""
    def __init__(self, guards):
        self.guards = guards

    def reset(self):
        guards, self.guards = self.guards, []
        first_error = None
        for guard in guards:
            try:
                guard.reset()
            except BaseException as error:
                first_error = first_error or error
        if first_error is not None:
            raise first_error

    def __del__(self):
        try:
            self.reset()
        except BaseException:
            pass


@dataclass
class CameraRig:
    """相机安装几何与诊断用 USD 相机。

    ``theta`` 为绕 base z 轴的相机偏航（rad），``sensor_yaw`` 是其旋转矩阵，
    ``Rcl`` 为相机光学系相对 IMU 系的旋转；``mount`` 为相机在 base 系的位置，
    ``imu_mount`` 取 articulation base 的质心。
    """

    theta: float
    sensor_yaw: np.ndarray
    Rcl: np.ndarray
    base_path: str
    mount: np.ndarray
    imu_mount: np.ndarray
    camera: object
    rtx_enabled: bool = False
    focal_length: float = 24.0


@dataclass
class RgbdRender:
    """机器人 RGB/深度/CameraParams 三路输出的 replicator 图。"""

    rep: object
    product: object
    rgb_ann: object
    depth_ann: object
    camera_params_ann: object
    time_ann: object = None
    rtx_cache: object = None
    rtx_writer: object = None
    rtx_listener: object = None
    rtx_callback_count: int = 0
    rtx_invalid_reference_count: int = 0
    rtx_error: object = None
    rtx_last_native_ns: int | None = None
    recover_invalid_native_frames: bool = False

    def accept_rtx_rgb_frame(self, *, source_s, reference_time, rgb, parameters,
                             acquisition_physics_s=None, time_evidence=None):
        """Copy one native frame, never substituting the current body clock."""
        import copy
        import numbers
        if (not isinstance(reference_time, (tuple, list)) or len(reference_time) != 2
                or not all(isinstance(v, numbers.Integral) for v in reference_time)
                or reference_time[0] < 0 or reference_time[1] <= 0):
            raise ValueError("Missing or invalid native RGB reference time")
        source_s = float(source_s)
        if not np.isfinite(source_s) or source_s < 0:
            raise ValueError("Nonfinite or negative native RGB frame time")
        ns = round(source_s * 1e9)
        if ns < getattr(self, "minimum_stamp_ns", 0) or ns == self.rtx_last_native_ns:
            return
        if self.rtx_last_native_ns is not None and ns < self.rtx_last_native_ns:
            raise ValueError("Native RGB time moved backwards")
        rgb = np.asarray(rgb)
        if rgb.shape not in ((512,512,3), (512,512,4)) or not parameters:
            return
        if len(self.rtx_cache) >= 4:
            raise RuntimeError("Native RGB queue overflow; refusing silent frame loss")
        self.rtx_cache.append({"time_s": source_s, "rgb": rgb[..., :3].copy(),
                              "camera_parameters": copy.deepcopy(parameters),
                              "reference_time": [int(v) for v in reference_time],
                              "acquisition_physics_s": acquisition_physics_s,
                              "time_evidence": copy.deepcopy(time_evidence),
                              "arrival_wall_s": (time_evidence or {}).get("original_arrival_wall_s")})
        self.rtx_last_native_ns = ns

    def _native_annotators(self):
        return {name: getattr(self, name) for name in
                ('rgb_ann', 'depth_ann', 'instance_ann', 'camera_params_ann')
                if getattr(self, name, None) is not None}

    def _accept_native_render_frame(self, *, bound_data, **kwargs):
        self.accept_rtx_rgb_frame(rgb=bound_data['rgb_ann'],
                                 parameters=bound_data['camera_params_ann'], **kwargs)
        observer = getattr(self, 'bound_frame_observer', None)
        if observer is not None:
            observer(bound_data=bound_data, **kwargs)

    def _init_native_binding(self):
        from collections import deque
        self._pending_native_frames = deque()
        self._binding_last_event = None
        self._binding_minimum_stamp_ns = getattr(self, 'minimum_stamp_ns', 0)
        self._binding_stopped = False
        self._binding_quarantine = None
        self._binding_quarantine_written = False
        self._binding_drop_records = deque(maxlen=8)
        self.native_frame_drop_count = 0
        self.last_native_frame_drop = None

    def _remember_native_event(self, event):
        """Keep the first arrival, never turn a repeated read into fresh input."""
        ref = _reference_pair(event['reference_time'])
        if event['source_ns'] < getattr(self, 'minimum_stamp_ns', 0):
            return
        last = self._binding_last_event
        if last is not None:
            cmp = ref[0] * last['reference_time'][1] - last['reference_time'][0] * ref[1]
            if cmp < 0:
                raise ValueError('Native camera event time moved backwards')
            if cmp == 0:
                if (event['results'], event['frame_number']) != (last['results'], last['frame_number']):
                    raise ValueError('Ambiguous native camera events for one reference time')
                return
        if len(self._pending_native_frames) >= 4:
            raise RuntimeError('Native camera pending event overflow')
        self._pending_native_frames.append(event)
        self._binding_last_event = event

    def _binding_state(self):
        reference = self.rp_time_ann.get_data(do_array_copy=True)
        return {
            'reference_time': [reference.get('fabricFrameTimeNumerator'),
                               reference.get('fabricFrameTimeDenominator')],
            'product': str(self._binding_product_attr.get()),
            'render_results': {name: int(attr.get()) for name, attr in self._binding_result_attrs.items()},
            'completed_outputs': {name: int(attr.get()) for name, attr in self._binding_output_exec_attrs.items()},
        }

    def _verify_binding(self, event, state):
        if state['product'] != str(self.product.path) or event['product'] != state['product']:
            raise ValueError('Native camera render product mismatch')
        a, b = _reference_pair(event['reference_time']), _reference_pair(state['reference_time'])
        if a[0] * b[1] != b[0] * a[1]:
            raise ValueError('Native camera RP time does not match its completed event')
        if (not state['render_results'] or
                any(value != event['results'] for value in state['render_results'].values())):
            raise ValueError('Native camera annotators do not share the completed render results')
        if not state['completed_outputs'] or any(value != 1 for value in state['completed_outputs'].values()):
            raise ValueError('Native camera data producer has not completed its outputs')

    def _consume_after_sdk_update(self):
        """One read-only pass after SDK SDG evaluation; no retry or extra tick."""
        if self._binding_stopped or self.rtx_error is not None:
            return
        while self._pending_native_frames:
            event = self._pending_native_frames[0]
            if event['source_ns'] < getattr(self, 'minimum_stamp_ns', 0):
                self._pending_native_frames.popleft()
                continue
            bundle = {'event': event, 'valid': False, 'arrays': {}}
            try:
                # Read the state before pixels, but keep raw pixels quarantined
                # even when that state proves unpaired. Never enqueue until both
                # states have been checked against the SAME original event.
                bundle['before'] = self._binding_state()
                budget = [32 * 1024 * 1024]
                for name, ann in self._native_annotators().items():
                    bundle['arrays'][name] = _owned_camera_value(ann.get_data(do_array_copy=True), budget)
                global_ref = _owned_camera_value(self.reference_ann.get_data(do_array_copy=True))
                time_info = _owned_camera_value(self.time_ann.get_data(do_array_copy=True))
                bundle['after'] = self._binding_state()
                self._verify_binding(event, bundle['before'])
                self._verify_binding(event, bundle['after'])
                if bundle['before'] != bundle['after']:
                    raise ValueError('Native camera binding changed during its data reads')
                evidence = {
                    'binding_version': 2,
                    'source': 'original native RP completion event, verified after automatic SDK SDG update',
                    'event_reference': list(event['reference_time']),
                    'event_payload': event['payload'],
                    'global_reference_at_original_callback': event['global_reference'],
                    'global_reference_after_sdk_update': global_ref,
                    'auxiliary_simulation_time': time_info,
                    'original_arrival_wall_s': event['arrival_wall_s'],
                    'original_arrival_physics_s': event['acquisition_physics_s'],
                    'binding_before': bundle['before'], 'binding_after': bundle['after'],
                    'camera_params_pose_epoch': 'renderer metadata; physical endpoint and pixel-tick pose not certified',
                    'body_now_substituted': False,
                }
                if self.recover_invalid_native_frames and (
                    np.asarray(bundle['arrays'].get('rgb_ann')).shape not in
                    ((512, 512, 3), (512, 512, 4)) or
                    not bundle['arrays'].get('camera_params_ann')
                ):
                    raise ValueError('Missing or invalid native RGB frame data')
                self._accept_native_render_frame(
                    bound_data=bundle['arrays'], source_s=event['source_s'],
                    reference_time=event['reference_time'],
                    acquisition_physics_s=event['acquisition_physics_s'],
                    time_evidence=evidence,
                )
                self._pending_native_frames.popleft()
            except BaseException as error:
                self._fail_native_binding(error, bundle)
                return

    def quarantine_native_frame(self, error, bundle):
        """At most one bounded failed sample per RP, never a valid frame."""
        if self._binding_quarantine is None:
            self._binding_quarantine = {'error': repr(error), 'valid': False,
                                        'bundle': _owned_camera_value(bundle)}
        self._flush_native_quarantine()

    def _flush_native_quarantine(self):
        if self._binding_quarantine_written or self._binding_quarantine is None:
            return
        output = getattr(self, 'quarantine_output', None)
        if output is None:
            return  # Retained in memory until FrameStream supplies its output.
        try:
            import json
            from pathlib import Path
            directory = Path(output) / 'camera_quarantine' / str(self.product.path).rsplit('/', 1)[-1]
            directory.mkdir(parents=True, exist_ok=True)
            arrays = {}
            def encode(value, name='sample'):
                if isinstance(value, np.ndarray):
                    arrays[name] = value
                    return {'array_key': name, 'shape': list(value.shape), 'dtype': str(value.dtype)}
                if isinstance(value, dict):
                    return {str(k): encode(v, name + '_' + str(k)) for k, v in value.items()}
                if isinstance(value, (list, tuple)):
                    return [encode(v, name + '_' + str(i)) for i, v in enumerate(value)]
                if isinstance(value, np.generic):
                    return value.item()
                return value
            metadata = encode(self._binding_quarantine)
            np.savez(directory / 'first_failed_sample.npz', **arrays)
            (directory / 'first_failed_sample.json').write_text(json.dumps(metadata, sort_keys=True))
            self._binding_quarantine_written = True
        except BaseException as error:
            self.quarantine_write_error = repr(error)  # Never replace the original failure.

    def _flush_native_frame_drops(self):
        output = getattr(self, 'quarantine_output', None)
        if output is None or not self._binding_drop_records:
            return
        try:
            import json
            from pathlib import Path
            path = Path(output) / 'camera_frame_validation.jsonl'
            with path.open('a', encoding='utf-8') as handle:
                while self._binding_drop_records:
                    handle.write(json.dumps(self._binding_drop_records[0], default=str) + '\n')
                    self._binding_drop_records.popleft()
        except BaseException as secondary:
            self.quarantine_write_error = repr(secondary)

    def _fail_native_binding(self, error, bundle):
        if self.recover_invalid_native_frames:
            # Static exploration drops an unverified frame, not the simulator.
            # Retain its original clocks and let the next real RP event retry;
            # neither old pixels nor current-body timestamps are substituted.
            event = bundle.get('event', bundle)
            self.native_frame_drop_count += 1
            self.rtx_invalid_reference_count += 1
            self.last_native_frame_drop = {
                'event': 'native_rgb_frame_dropped', 'valid': False,
                'reason': repr(error), 'product': str(self.product.path),
                'source_ns': event.get('source_ns'),
                'reference_time': event.get('reference_time', event.get('parsed_reference')),
                'global_reference': event.get('global_reference'),
                'original_arrival_wall_s': event.get('arrival_wall_s'),
                'acquisition_physics_s': event.get('acquisition_physics_s'),
                'frame_number': event.get('frame_number'),
                'render_results': event.get('results'),
                'unverified_pending_discarded': len(self._pending_native_frames),
                'drop_count': self.native_frame_drop_count,
                'simulator_termination_requested': False,
                'current_body_stamp_substituted': False,
                'next_read': 'next real native RP completion event',
            }
            self._binding_drop_records.append(dict(self.last_native_frame_drop))
            self._pending_native_frames.clear()
            try:
                self.quarantine_native_frame(error, bundle)
            except BaseException as secondary:
                self.quarantine_write_error = repr(secondary)
            self._flush_native_frame_drops()
            return
        if self.rtx_error is None:
            self.rtx_error = error
            try:
                self.quarantine_native_frame(error, bundle)
            except BaseException as secondary:
                self.quarantine_write_error = repr(secondary)
        try:
            self.stop_native_capture()
        except BaseException as secondary:
            self.capture_cleanup_error = repr(secondary)

    def stop_native_capture(self):
        self._binding_stopped = True
        if hasattr(self, '_pending_native_frames'):
            self._pending_native_frames.clear()
        listener = self.rtx_listener
        self.rtx_listener = None
        if listener is not None:
            listener.reset()

    def reset_native_binding(self):
        """Drop pre-reset events, not capture failures or original source clocks."""
        self._pending_native_frames.clear()
        self._binding_last_event = None
        self._binding_minimum_stamp_ns = getattr(self, 'minimum_stamp_ns', 0)

    def attach_rtx_writer(self):
        """Record completion immediately; copy its data only after automatic SDG.

        RC7 updates global ReferenceTime before the per-RP data nodes. Immediate
        evaluate remains for existing scheduling compatibility, but no image is
        accepted from that callback. No waitIdle, extra evaluate or physics step.
        """
        import copy
        import time
        from collections import deque
        import carb
        import omni.usd
        import omni.kit.app
        import omni.timeline
        import omni.graph.core as og
        from omni.syntheticdata import SyntheticData, _syntheticdata
        from isaacsim.core.simulation_manager import SimulationManager, _simulation_manager

        self.rtx_cache = deque()
        self._init_native_binding()
        self.reference_ann = self.rep.AnnotatorRegistry.get_annotator('ReferenceTime')
        self.reference_ann.attach([self.product])
        self.rp_time_ann = self.rep.AnnotatorRegistry.get_annotator('rpFabricTime')
        self.rp_time_ann.attach([self.product])
        self._binding_product_attr, self._binding_result_attrs, self._binding_output_exec_attrs = _camera_binding_attributes(
            self.rp_time_ann, self._native_annotators())
        sd = _syntheticdata.acquire_syntheticdata_interface()
        controller = og.Controller()
        graph = controller.graph('/Render/PostProcess/SDGPipeline')
        manager = _simulation_manager.acquire_simulation_manager_interface()
        product_path = str(self.product.path)

        def on_native_frame(event):
            if self._binding_stopped or self.rtx_error is not None:
                return
            record = {'valid': False}
            arrival = time.perf_counter()
            try:
                parsed = sd.parse_rendered_simulation_event(event['product_path_handle'], event['results'])
                if str(parsed[0]) != product_path:
                    return
                acquisition = float(SimulationManager.get_simulation_time())
                self.rtx_callback_count += 1
                record = {'product': product_path, 'parsed_reference': list(parsed[1:]),
                          'payload': copy.deepcopy(event.payload), 'arrival_wall_s': arrival,
                          'acquisition_physics_s': acquisition, 'valid': False}
                controller.evaluate_sync(graph_id=graph)  # The original single evaluate.
                global_reference = _owned_camera_value(self.reference_ann.get_data())
                record['global_reference'] = global_reference
                reference = _reference_pair((parsed[1], parsed[2]))
                global_pair = _reference_pair((global_reference.get('referenceTimeNumerator'),
                                               global_reference.get('referenceTimeDenominator')))
                if reference[0] * global_pair[1] != global_pair[0] * reference[1]:
                    raise ValueError('Native camera completion and global reference disagree')
                native_time = float(manager.get_simulation_time_at_time(reference))
                if not np.isfinite(native_time) or native_time < 0:
                    raise ValueError('Invalid native camera simulation time')
                record = {'product': product_path, 'reference_time': list(reference),
                          'source_s': native_time, 'source_ns': round(native_time * 1e9),
                          'results': int(event['results']), 'frame_number': int(event['frame_number']),
                          'payload': copy.deepcopy(event.payload), 'global_reference': global_reference,
                          'arrival_wall_s': arrival, 'acquisition_physics_s': acquisition}
                if record['results'] <= 0 or record['frame_number'] < 0:
                    raise ValueError('Invalid native camera render identity')
                self._remember_native_event(record)
            except BaseException as error:
                self._fail_native_binding(error, record)

        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        name = getattr(self, 'native_observer_name', 'dynanav.native_rgb_frame')
        self.rtx_listener = _CameraListeners([
            dispatcher.observe_event(
                event_name=omni.usd.get_context().stage_rendering_event_name(omni.usd.StageRenderingEventType.NEW_FRAME, True),
                on_event=on_native_frame, observer_name=name),
            dispatcher.observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                order=SyntheticData.post_process_graph_tick_order() + 1,
                on_event=lambda event: self._consume_after_sdk_update(), observer_name=name + '.after_sdg'),
            dispatcher.observe_event(event_name=omni.timeline.GLOBAL_EVENT_STOP,
                                     on_event=lambda event: self.stop_native_capture(), observer_name=name + '.stop'),
        ])

    def capture_rtx_frame(self, minimum_stamp_ns):
        self.minimum_stamp_ns = minimum_stamp_ns
        if hasattr(self, '_binding_minimum_stamp_ns') and self._binding_minimum_stamp_ns != minimum_stamp_ns:
            self.reset_native_binding()
        if hasattr(self, '_binding_quarantine'):
            self._flush_native_quarantine()
        if hasattr(self, '_binding_drop_records'):
            self._flush_native_frame_drops()
        if self.rtx_error is not None:
            raise RuntimeError("Native RGB frame capture failed") from self.rtx_error

    def take_rtx_frame(self):
        self.capture_rtx_frame(getattr(self, "minimum_stamp_ns", 0))
        return self.rtx_cache.popleft() if self.rtx_cache else None

    def has_rtx_frame(self):
        self.capture_rtx_frame(getattr(self, "minimum_stamp_ns", 0))
        return bool(self.rtx_cache)


@dataclass
class OverviewRender:
    """第三人称 overview 相机及其 RGB 标注器（仅用于录像/诊断）。"""

    spectator: object
    product: object
    ann: object


def create_camera_rig(stage, experiment):
    """把相机装到真实 articulation base 上，返回其安装几何。

    偏航角优先取 experiment 的 ``camera_yaw_rad``，否则取环境变量
    ``GO2_CAMERA_YAW``，默认 π/2 rad（90°）。安装位置 [0.3, 0, 0.13] 与
    默认焦距/aperture 保持原值；多相机实验可显式设置 HFOV，并同步输出内参收据。
    """
    import os

    from pxr import Gf, UsdGeom, UsdPhysics

    # 传感器装在真实 articulation 的 base 上，不另建独立 rig；偏航角决定相机
    # 相对机身的朝向（默认 90°，与 RGB-D 前视安装一致）。
    theta = float(
        experiment.get(
            "camera_yaw_rad", os.environ.get("GO2_CAMERA_YAW", "1.5707963267948966")
        )
    )
    sensor_yaw = np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1.0],
        ]
    )
    Rcl = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]]) @ sensor_yaw.T
    # base 路径、相机安装偏移（m）与诊断相机名都是硬编码契约；下游按同名路径
    # 查找相机/IMU，改名会同时影响 frames 与 receipts。
    base_path = "/World/Go2/Geometry/base"
    mount = np.array([0.3, 0.0, 0.13])
    imu_mount = np.array(
        UsdPhysics.MassAPI(stage.GetPrimAtPath(base_path)).GetCenterOfMassAttr().Get(),
        dtype=float,
    )

    camera_path = base_path + "/DiagnosticCamera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.AddTransformOp().Set(
        Gf.Matrix4d(
            Gf.Matrix3d(
                *(np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]]) @ sensor_yaw.T)
                .flatten()
                .tolist()
            ),
            Gf.Vec3d(*mount),
        )
    )
    # 默认比例 24/42 保持原值。广视场只在 RTX 多相机实验中显式开启，
    # 不经过固定内参的 RGB-D 反投影路径；裁剪面仍为 0.05..100 场景单位。
    camera.GetHorizontalApertureAttr().Set(42.0)
    camera.GetVerticalApertureAttr().Set(42.0)
    from ..multicamera import focal_length_for_hfov
    focal_length = focal_length_for_hfov(experiment.get("multicamera_horizontal_fov_deg"))
    camera.GetFocalLengthAttr().Set(focal_length)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    rtx_enabled = experiment.get("point_cloud_source", "rgbd") == "rtx_lidar"
    if rtx_enabled:
        # Register the camera with the same multi-tick sensor scheduler as LiDAR.
        # A legacy unregistered camera can retain its startup view under FSD.
        from isaacsim.sensors.experimental.rtx import RtxCamera
        camera.GetPrim().ApplyAPI("OmniSensorAPI")
        RtxCamera(camera_path, tick_rate=10.0, reset_xform_op_properties=False)
    return CameraRig(theta, sensor_yaw, Rcl, base_path, mount, imu_mount, camera, rtx_enabled, focal_length)


def create_imu_sensor(rig):
    """在 articulation base 质心处安装 IMU 传感器。

    三个 filter_size 均为 1，表示不再做额外滑动平均；下游拿到的是单步
    物理量（加速度 m/s²、角速度 rad/s）。
    """
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor

    base_path = rig.base_path
    imu_mount = rig.imu_mount
    imu_sensor = IMUSensor(
        IMU.create(
            base_path + "/DiagnosticIMU",
            translations=[imu_mount.tolist()],
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )
    )

    return imu_sensor


def create_rgbd_annotators(rig, *, include_depth=True, recover_invalid_native_frames=False):
    """创建 RGB/深度/CameraParams 渲染产品与标注器。

    渲染分辨率为 512×512；深度使用 ``distance_to_image_plane``（沿光轴
    的米制距离，与 depth.py 的针孔反投影约定对应）。``capture_on_play=False``
    禁止随播放自动抓帧，采集节奏由上层逐步驱动。
    """
    import omni.replicator.core as rep

    camera_path = rig.base_path + "/DiagnosticCamera"
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(camera_path, (512, 512))
    rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_ann.attach([product])
    depth_ann = None
    if include_depth:
        depth_ann = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        depth_ann.attach([product])
    camera_params_ann = rep.AnnotatorRegistry.get_annotator("CameraParams")
    camera_params_ann.attach([product])

    time_ann = None
    if not include_depth or getattr(rig, "rtx_enabled", False):
        time_ann = rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime")
        time_ann.attach([product])
    result = RgbdRender(
        rep=rep,
        product=product,
        rgb_ann=rgb_ann,
        depth_ann=depth_ann,
        camera_params_ann=camera_params_ann,
        time_ann=time_ann,
        recover_invalid_native_frames=bool(recover_invalid_native_frames),
    )
    if not include_depth or getattr(rig, "rtx_enabled", False):
        result.attach_rtx_writer()
    return result


def create_overview(rig, stage, rep):
    """创建录像用的第三人称 overview 渲染产品。

    相机的位置/朝向（eye/focus）、焦距 22 与 aperture 35 是硬编码常量，
    只影响录像视角，不参与导航或标定计算；分辨率同为 512×512。
    """
    from pxr import Gf, UsdGeom

    base_path = rig.base_path
    spectator = UsdGeom.Camera.Define(stage, base_path + "/PIDOverviewCamera")
    eye = np.array([-1.8, -1.35, 1.15])
    focus = np.array([0.15, 0.0, -0.1])
    spectator.AddTransformOp().Set(
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*focus), Gf.Vec3d(0, 0, 1))
        .GetInverse()
    )
    spectator.GetFocalLengthAttr().Set(22.0)
    spectator.GetHorizontalApertureAttr().Set(35.0)
    spectator.GetVerticalApertureAttr().Set(35.0)
    spectator.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    if getattr(rig, "rtx_enabled", False):
        from isaacsim.sensors.experimental.rtx import RtxCamera
        spectator.GetPrim().ApplyAPI("OmniSensorAPI")
        RtxCamera(base_path + "/PIDOverviewCamera", tick_rate=10.0, reset_xform_op_properties=False)
    overview_product = rep.create.render_product(
        base_path + "/PIDOverviewCamera", (512, 512)
    )
    overview_ann = rep.AnnotatorRegistry.get_annotator("rgb")
    overview_ann.attach([overview_product])

    return OverviewRender(
        spectator=spectator, product=overview_product, ann=overview_ann
    )


def sensor_installation_receipt(rig, truth_usage, sim_rtf):
    """返回传感器安装收据：安装变换、内参、仿真限制与真值用途。

    收据里的 ``camera_optical_translation_from_IMU`` 由 Rcl 把 base 系下的
    位移旋转到相机光学系；``camera_intrinsics`` 为 [fx, fy, cx, cy]，
    由本次 authored focal/aperture 比例计算；默认 ``focal_px=512*24/42``。
    ``truth_only_for_scoring=False`` 表明该采集不以真值作为导航输入。
    """
    base_path = rig.base_path
    imu_mount = rig.imu_mount
    Rcl = rig.Rcl
    mount = rig.mount
    theta = rig.theta
    focal = getattr(rig, "focal_length", 24.0)

    return {
        "parent": base_path,
        "imu_translation_in_base": imu_mount.tolist(),
        "camera_optical_translation_from_IMU": (Rcl @ (imu_mount - mount)).tolist(),
        "camera_translation_in_base": mount.tolist(),
        "camera_optical_R_from_IMU": Rcl.tolist(),
        "custom_sensor_yaw_degrees": float(np.rad2deg(theta)),
        "camera_intrinsics": [512 * focal / 42, 512 * focal / 42, 256, 256],
        "camera_model": "pinhole",
        "authored_focal_length": focal,
        "authored_aperture": [42., 42.],
        "modality": "global-shutter RGB-D, not rotating lidar",
        "truth_only_for_scoring": False,
        "truth_usage": truth_usage,
        "max_simulation_real_time_factor": sim_rtf,
        "pacing_changes_physics_dt": False,
    }
