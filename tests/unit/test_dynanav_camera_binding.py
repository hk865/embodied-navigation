"""CPU-only frame binding fixtures. No claim of RTX or moving-camera acceptance."""
import copy
from collections import deque
import gc
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from embodied_navigation.simulation.dynanav.capture import sensors
from embodied_navigation.simulation.dynanav.capture.nearfield_sensor import NearfieldDepthRender
from test_dynanav_nearfield_depth import parameters, plane_image, seg


class Attr:
    def __init__(self, node, name, value=None, upstream=()):
        self.node, self.name, self.value, self.upstream = node, name, value, list(upstream)
        node.attrs[name] = self
    def get(self): return self.value() if callable(self.value) else self.value
    def get_name(self): return self.name
    def get_path(self): return self.node.path + '.' + self.name
    def get_node(self): return self.node
    def is_valid(self): return True
    def get_upstream_connections(self): return self.upstream


class Node:
    def __init__(self, path): self.path, self.attrs = path, {}
    def get_prim_path(self): return self.path
    def get_attribute(self, name): return self.attrs[name]
    def get_attributes(self): return list(self.attrs.values())


class Ann:
    def __init__(self, value=None, node=None):
        self.value, self.node, self.calls, self.hook = value, node, [], None
    def get_node(self): return self.node
    def attach(self, products): self.products = products
    def get_data(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if self.hook: self.hook()
        return self.value  # Deliberately mutable, even when SDK copy is requested.


def rig(path='/RP/front', nearfield=False):
    dispatch = Node(path + '/dispatch')
    ptr = Attr(dispatch, 'outputs:renderResults', 10)
    product = Attr(dispatch, 'outputs:renderProductPath', path)
    # Real SDK scalar provenance edge: camera productPath reaches the RP
    # dispatcher, whose path/pointer lists reach the global event dispatcher.
    # Neither dispatcher pulse is a persistent image-data readiness flag.
    Attr(dispatch, 'outputs:exec', 0)
    global_dispatch = Node('/global_event_source')
    Attr(global_dispatch, 'outputs:exec', 0)
    paths = Attr(global_dispatch, 'outputs:renderProductPaths', [path])
    Attr(dispatch, 'inputs:renderProductPaths', upstream=[paths])
    rpnode = Node(path + '/rp_time'); rpattr = Attr(rpnode, 'inputs:rp', lambda: ptr.value, [ptr])
    Attr(rpnode, 'outputs:exec', 1)
    rptime = Ann({'fabricFrameTimeNumerator': 1, 'fabricFrameTimeDenominator': 10}, rpnode)
    anns = {}
    values = {'depth_ann': plane_image(), 'instance_ann': seg(), 'camera_params_ann': parameters()} if nearfield else {
        'rgb_ann': np.zeros((512, 512, 4), np.uint8), 'camera_params_ann': {'cameraViewTransform': np.eye(4)}}
    for name, value in values.items():
        producer = Node(path + '/' + name + '_producer')
        Attr(producer, 'outputs:exec', 1)
        Attr(producer, 'inputs:renderProductPath', upstream=[product])
        Attr(producer, 'inputs:renderResults', lambda: ptr.value, [ptr])
        data = Attr(producer, 'outputs:data')
        node = Node(path + '/' + name); Attr(node, 'inputs:data', upstream=[data])
        Attr(node, 'outputs:exec', 1)
        if name == 'instance_ann':
            token = Node(path + '/instance_token_producer')
            Attr(token, 'inputs:renderResults', lambda: ptr.value, [ptr])
            Attr(token, 'outputs:exec', 1)
            labels = Attr(token, 'outputs:labels')
            Attr(node, 'inputs:labels', upstream=[labels])
        # A global execution input is deliberately unrelated and is not provenance.
        globalnode = Node('/global'); execution = Attr(globalnode, 'outputs:exec')
        Attr(node, 'inputs:exec', upstream=[execution])
        anns[name] = Ann(value, node)
    cls = NearfieldDepthRender if nearfield else sensors.RgbdRender
    obj = cls(None, SimpleNamespace(path=path), anns.get('rgb_ann'), anns.get('depth_ann'), anns['camera_params_ann'])
    if nearfield: obj.instance_ann = anns['instance_ann']
    obj.reference_ann = Ann({'referenceTimeNumerator': 1, 'referenceTimeDenominator': 10})
    obj.time_ann = Ann({'execOut': 1, 'simulationTime': .1})
    obj.rp_time_ann = rptime
    obj.rtx_cache = deque(); obj._init_native_binding()
    obj._binding_product_attr, obj._binding_result_attrs, obj._binding_output_exec_attrs = sensors._camera_binding_attributes(rptime, anns)
    obj.test_ptr, obj.test_product, obj.test_anns = ptr, product, anns
    obj.test_dispatch, obj.test_global_dispatch = dispatch, global_dispatch
    return obj


def event(obj, tick=1, ptr=10, arrival=42.):
    return {'product': str(obj.product.path), 'reference_time': [tick, 10],
            'source_s': tick / 10, 'source_ns': tick * 100_000_000,
            'results': ptr, 'frame_number': tick, 'payload': {'frame_number': tick},
            'arrival_wall_s': arrival, 'acquisition_physics_s': tick / 10 + .115,
            'global_reference': {'referenceTimeNumerator': tick, 'referenceTimeDenominator': 10}}


def ready(obj, tick, ptr=10):
    obj.test_ptr.value = ptr
    obj.rp_time_ann.value = {'fabricFrameTimeNumerator': tick, 'fabricFrameTimeDenominator': 10}
    obj.reference_ann.value = {'referenceTimeNumerator': tick, 'referenceTimeDenominator': 10}
    obj.time_ann.value = {'execOut': 1, 'simulationTime': tick / 10}


class BindingTests(unittest.TestCase):
    def test_immediate_old_pixels_never_enter_queue_then_current_pixels_are_copied(self):
        r = rig(); r._remember_native_event(event(r, 2, 20))
        self.assertFalse(r.rtx_cache); self.assertFalse(r.rgb_ann.calls)
        ready(r, 2, 20); r.rgb_ann.value[:] = 27
        r._consume_after_sdk_update(); frame = r.take_rtx_frame()
        self.assertEqual(frame['time_s'], .2); self.assertEqual(frame['reference_time'], [2, 10])
        self.assertEqual(frame['arrival_wall_s'], 42.); self.assertEqual(frame['acquisition_physics_s'], .315)
        self.assertEqual(frame['time_evidence']['binding_version'], 2)
        r.rgb_ann.value[:] = 99; r.camera_params_ann.value['cameraViewTransform'][:] = 0
        self.assertEqual(int(frame['rgb'].max()), 27)
        np.testing.assert_array_equal(frame['camera_parameters']['cameraViewTransform'], np.eye(4))
        self.assertEqual(r.rgb_ann.calls, [((), {'do_array_copy': True})])

    def test_stale_time_even_reused_pointer_fails_and_quarantines_original_bytes(self):
        r = rig(); r._remember_native_event(event(r, 2, 10))
        with tempfile.TemporaryDirectory() as d:
            r.quarantine_output = Path(d); r._consume_after_sdk_update()
            self.assertFalse(r.rtx_cache); self.assertIn('RP time', str(r.rtx_error))
            q = Path(d) / 'camera_quarantine/front'
            self.assertFalse(json.loads((q / 'first_failed_sample.json').read_text())['valid'])
            with np.load(q / 'first_failed_sample.npz') as z:
                self.assertEqual(z['sample_bundle_arrays_rgb_ann'].shape, (512, 512, 4))
            first = (q / 'first_failed_sample.npz').read_bytes()
            r.quarantine_native_frame(ValueError('second'), {'rgb': np.ones((2, 2))})
            self.assertEqual(first, (q / 'first_failed_sample.npz').read_bytes())

    def test_reused_pointer_with_matching_new_native_time_is_allowed(self):
        r = rig()
        for tick in (1, 2):
            ready(r, tick); r._remember_native_event(event(r, tick)); r._consume_after_sdk_update()
        self.assertEqual([f['time_s'] for f in r.rtx_cache], [.1, .2])

    def test_pointer_or_product_change_before_or_during_read_never_enqueues(self):
        for change in ('pointer_before', 'pointer_during', 'time_during', 'product_before', 'product_during'):
            r = rig(); r._remember_native_event(event(r))
            if change == 'pointer_before': r.test_ptr.value = 20
            if change == 'product_before': r.test_product.value = '/other'
            if change == 'pointer_during': r.rgb_ann.hook = lambda: setattr(r.test_ptr, 'value', 20)
            if change == 'time_during': r.rgb_ann.hook = lambda: ready(r, 2)
            if change == 'product_during': r.rgb_ann.hook = lambda: setattr(r.test_product, 'value', '/other')
            with self.subTest(change=change):
                r._consume_after_sdk_update(); self.assertFalse(r.rtx_cache); self.assertIsNotNone(r.rtx_error)

    def test_two_products_cannot_borrow_each_others_clock_or_results(self):
        a, b = rig('/RP/a'), rig('/RP/b'); a._remember_native_event(event(a, 2, 20)); b._remember_native_event(event(b))
        ready(a, 2, 20); a._consume_after_sdk_update(); b._consume_after_sdk_update()
        self.assertEqual(a.rtx_cache[0]['time_s'], .2); self.assertEqual(b.rtx_cache[0]['time_s'], .1)
        c = rig('/RP/c'); borrowed = event(a); c._remember_native_event(borrowed); c._consume_after_sdk_update()
        self.assertFalse(c.rtx_cache); self.assertIn('product', str(c.rtx_error))

    def test_duplicate_keeps_first_arrival_and_does_not_reread_after_accept(self):
        r = rig(); r._remember_native_event(event(r)); r._remember_native_event(event(r, arrival=99))
        self.assertEqual(len(r._pending_native_frames), 1)
        r._consume_after_sdk_update(); r._remember_native_event(event(r, arrival=100)); r._consume_after_sdk_update()
        self.assertEqual(len(r.rtx_cache), 1); self.assertEqual(r.rtx_cache[0]['arrival_wall_s'], 42.)
        self.assertEqual(len(r.rgb_ann.calls), 1)

    def test_same_time_different_frame_or_pointer_is_ambiguous(self):
        for key in ('results', 'frame_number'):
            r = rig(); r._remember_native_event(event(r)); second = event(r); second[key] += 1
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'Ambiguous'):
                r._remember_native_event(second)

    def test_zero_one_multiple_callbacks_and_unmatched_first_update_expires(self):
        r = rig(); r._consume_after_sdk_update(); self.assertFalse(r.rgb_ann.calls)
        r._remember_native_event(event(r)); r._remember_native_event(event(r, 2, 20)); ready(r, 2, 20)
        r._consume_after_sdk_update(); self.assertFalse(r.rtx_cache); self.assertTrue(r._binding_stopped)
        self.assertFalse(r._pending_native_frames)
        ready(r, 1); r._consume_after_sdk_update(); self.assertFalse(r.rtx_cache)

    def test_pending_capacity_and_time_order_are_fail_closed(self):
        r = rig()
        for t in range(1, 5): r._remember_native_event(event(r, t))
        with self.assertRaisesRegex(RuntimeError, 'overflow'): r._remember_native_event(event(r, 5))
        self.assertEqual(len(r._pending_native_frames), 4)
        with self.assertRaisesRegex(ValueError, 'backwards'): r._remember_native_event(event(r, 1))

    def test_startup_minimum_and_reset_discard_only_old_pending(self):
        r = rig(); r.minimum_stamp_ns = 150_000_000
        r._remember_native_event(event(r)); self.assertFalse(r._pending_native_frames)
        r._remember_native_event(event(r, 2)); r.capture_rtx_frame(150_000_000)
        self.assertFalse(r._pending_native_frames)
        ready(r, 2); r._remember_native_event(event(r, 2)); r._consume_after_sdk_update()
        self.assertEqual(len(r.rtx_cache), 1)

    def test_original_failure_survives_secondary_quarantine_and_cleanup_errors(self):
        r = rig(); original = ValueError('original'); r.rtx_listener = SimpleNamespace(reset=lambda: (_ for _ in ()).throw(RuntimeError('cleanup')))
        with patch.object(r, 'quarantine_native_frame', side_effect=OSError('disk')):
            r._fail_native_binding(original, {})
        self.assertIs(r.rtx_error, original); self.assertIn('disk', r.quarantine_write_error)
        self.assertIn('cleanup', r.capture_cleanup_error)

    def test_bounded_snapshot_and_read_exception_never_queue(self):
        r = rig(); r.rgb_ann.value = np.zeros(32 * 1024 * 1024 + 1, np.uint8)
        r._remember_native_event(event(r)); r._consume_after_sdk_update()
        self.assertIn('budget', str(r.rtx_error)); self.assertFalse(r.rtx_cache)
        r = rig(); error = RuntimeError('SDK read'); r.rgb_ann.hook = lambda: (_ for _ in ()).throw(error)
        r._remember_native_event(event(r)); r._consume_after_sdk_update()
        self.assertIs(r.rtx_error, error)

    def test_nearfield_depth_ids_parameters_share_bundle_and_original_arrival(self):
        r = rig('/RP/near', True); r._remember_native_event(event(r)); r._consume_after_sdk_update()
        arrays, receipt = r.rtx_cache[0]
        self.assertEqual(receipt['arrival_wall_s'], 42.); self.assertEqual(receipt['stamp_ns'], 100_000_000)
        self.assertFalse(receipt['control_enabled']); self.assertFalse(receipt['world_pose_used'])
        old = arrays['points_base_m'].copy(); r.depth_ann.value[:] = 99; r.instance_ann.value['data'][:] = 0
        np.testing.assert_array_equal(arrays['points_base_m'], old)
        self.assertEqual(receipt['time_evidence']['binding_before'], receipt['time_evidence']['binding_after'])

    def test_nearfield_stale_ids_pointer_is_rejected_even_when_other_data_match(self):
        r = rig('/RP/near', True)
        ptrs = r._binding_result_attrs
        name = next(name for name in ptrs if 'instance_ann' in name)
        ptrs[name].value = 999
        r._remember_native_event(event(r)); r._consume_after_sdk_update()
        self.assertFalse(r.rtx_cache); self.assertIn('annotators', str(r.rtx_error))

    def test_dependency_graph_rejects_wrong_product_or_missing_data_provenance(self):
        r = rig(); other = Node('/other'); pointer = Attr(other, 'outputs:renderResults', 10)
        producer = r.rgb_ann.node.attrs['inputs:data'].upstream[0].node
        producer.attrs['inputs:renderResults'].upstream = [pointer]
        with self.assertRaisesRegex(ValueError, 'different render product'):
            sensors._camera_binding_attributes(r.rp_time_ann, r._native_annotators())
        r = rig(); r.rgb_ann.node.attrs = {'outputs:exec': r.rgb_ann.node.attrs['outputs:exec']}
        with self.assertRaisesRegex(ValueError, 'no render-result'):
            sensors._camera_binding_attributes(r.rp_time_ann, r._native_annotators())

    def test_unexecuted_output_is_rejected_but_zero_input_exec_is_not_freshness(self):
        for target in ('root', 'producer', 'rp_time'):
            r = rig(); r._remember_native_event(event(r))
            node = r.rgb_ann.node if target == 'root' else (r.rp_time_ann.node if target == 'rp_time' else r.rgb_ann.node.attrs['inputs:data'].upstream[0].node)
            node.attrs['outputs:exec'].value = 0
            with self.subTest(target=target):
                r._consume_after_sdk_update(); self.assertFalse(r.rtx_cache)
                self.assertIn('not completed', str(r.rtx_error))
        r = rig(); r.rgb_ann.node.attrs['inputs:exec'].value = 0
        r._remember_native_event(event(r)); r._consume_after_sdk_update()
        self.assertEqual(len(r.rtx_cache), 1)

    def test_nearfield_mask_and_token_execution_both_required(self):
        for name in ('instance_ann_producer', 'instance_token_producer'):
            r = rig('/RP/near', True)
            attr = next(v for k, v in r._binding_output_exec_attrs.items() if name + '.' in k)
            attr.value = 0; r._remember_native_event(event(r)); r._consume_after_sdk_update()
            with self.subTest(name=name):
                self.assertFalse(r.rtx_cache); self.assertIn('not completed', str(r.rtx_error))

    def test_execution_change_during_copy_is_rejected(self):
        r = rig(); output = r.rgb_ann.node.attrs['outputs:exec']
        r.rgb_ann.hook = lambda: setattr(output, 'value', 0)
        r._remember_native_event(event(r)); r._consume_after_sdk_update()
        self.assertFalse(r.rtx_cache); self.assertIn('not completed', str(r.rtx_error))

    def test_verified_rp_source_boundary_excludes_only_dispatch_flow_pulses(self):
        for nearfield in (False, True):
            r = rig('/unusual/product17', nearfield)
            # Exact node identity, not the words 'PostProcessDispatch' in a path.
            r.test_dispatch.path = '/unusual/verified_source91'
            r._binding_product_attr, r._binding_result_attrs, r._binding_output_exec_attrs = sensors._camera_binding_attributes(r.rp_time_ann, r._native_annotators())
            self.assertEqual(r.test_dispatch.attrs['outputs:exec'].get(), 0)
            self.assertEqual(r.test_global_dispatch.attrs['outputs:exec'].get(), 0)
            execution = r._binding_output_exec_attrs
            self.assertNotIn(r.test_dispatch.attrs['outputs:exec'].get_path(), execution)
            self.assertNotIn(r.test_global_dispatch.attrs['outputs:exec'].get_path(), execution)
            self.assertIn(r.camera_params_ann.node.attrs['outputs:exec'].get_path(), execution)
            r._remember_native_event(event(r)); r._consume_after_sdk_update()
            with self.subTest(nearfield=nearfield):
                self.assertIsNone(r.rtx_error); self.assertEqual(len(r.rtx_cache), 1)

    def test_zero_dispatcher_does_not_bypass_wrong_product_result_or_data_execution(self):
        for fault in ('product', 'result', 'root', 'token'):
            r = rig('/near', True)
            if fault == 'product': r.test_product.value = '/wrong'
            if fault == 'result': r.test_ptr.value = 99
            if fault == 'root': r.camera_params_ann.node.attrs['outputs:exec'].value = 0
            if fault == 'token':
                key = next(k for k in r._binding_output_exec_attrs if 'instance_token_producer.' in k)
                r._binding_output_exec_attrs[key].value = 0
            r._remember_native_event(event(r)); r._consume_after_sdk_update()
            with self.subTest(fault=fault):
                self.assertIsNotNone(r.rtx_error); self.assertFalse(r.rtx_cache)

    def test_invalid_rational_and_exact_fraction_equivalence(self):
        for ref in ([0, 0], [-1, 1], [True, 1], [1., 10], [1], None):
            with self.subTest(ref=ref), self.assertRaises(ValueError): sensors._reference_pair(ref)
        r = rig(); r.rp_time_ann.value = {'fabricFrameTimeNumerator': 100_000_000, 'fabricFrameTimeDenominator': 1_000_000_000}
        r._remember_native_event(event(r)); r._consume_after_sdk_update(); self.assertEqual(len(r.rtx_cache), 1)

    def test_stop_releases_all_guards_even_if_one_reset_fails(self):
        resets = []
        def bad(): resets.append('bad'); raise RuntimeError('reset')
        group = sensors._CameraListeners([SimpleNamespace(reset=bad), SimpleNamespace(reset=lambda: resets.append('ok'))])
        with self.assertRaisesRegex(RuntimeError, 'reset'): group.reset()
        self.assertEqual(resets, ['bad', 'ok']); group.reset()
        r = rig(); r.rtx_listener = sensors._CameraListeners([SimpleNamespace(reset=lambda: resets.append('dropped'))])
        r.rtx_listener = None; gc.collect(); self.assertIn('dropped', resets)
        r.stop_native_capture(); r._consume_after_sdk_update(); self.assertFalse(r.rgb_ann.calls)


class SdkCallbackTests(unittest.TestCase):
    def sdk(self, obj):
        names = ['carb', 'omni', 'omni.usd', 'omni.kit', 'omni.kit.app', 'omni.timeline',
                 'omni.graph', 'omni.graph.core', 'omni.syntheticdata', 'omni.replicator',
                 'omni.replicator.core', 'isaacsim', 'isaacsim.core', 'isaacsim.core.simulation_manager']
        mods = {name: ModuleType(name) for name in names}
        for name, mod in mods.items():
            if '.' in name:
                parent, child = name.rsplit('.', 1); setattr(mods[parent], child, mod)
        handlers, evaluated = {}, []
        class Guard:
            def __init__(self, key): self.key, self.resets = key, 0
            def reset(self): self.resets += 1
        guards = []
        def observe_event(**kwargs):
            handlers[kwargs['event_name']] = kwargs
            guard = Guard(kwargs['event_name']); guards.append(guard); return guard
        mods['carb'].eventdispatcher = SimpleNamespace(get_eventdispatcher=lambda: SimpleNamespace(observe_event=observe_event))
        mods['omni.usd'].get_context = lambda: SimpleNamespace(stage_rendering_event_name=lambda value, immediate: 'native')
        mods['omni.usd'].StageRenderingEventType = SimpleNamespace(NEW_FRAME=1)
        mods['omni.kit.app'].GLOBAL_EVENT_UPDATE = 'update'
        mods['omni.timeline'].GLOBAL_EVENT_STOP = 'stop'
        mods['omni.graph.core'].Controller = lambda: SimpleNamespace(graph=lambda path: path, evaluate_sync=lambda **kwargs: evaluated.append(kwargs))
        mods['omni.syntheticdata'].SyntheticData = SimpleNamespace(post_process_graph_tick_order=lambda: 100)
        parsed = [str(obj.product.path), 2, 10]
        mods['omni.syntheticdata']._syntheticdata = SimpleNamespace(acquire_syntheticdata_interface=lambda: SimpleNamespace(parse_rendered_simulation_event=lambda *args: parsed))
        mods['isaacsim.core.simulation_manager'].SimulationManager = SimpleNamespace(get_simulation_time=lambda: .315)
        mods['isaacsim.core.simulation_manager']._simulation_manager = SimpleNamespace(acquire_simulation_manager_interface=lambda: SimpleNamespace(get_simulation_time_at_time=lambda pair: pair[0] / pair[1]))
        registry = {'ReferenceTime': obj.reference_ann, 'rpFabricTime': obj.rp_time_ann}
        obj.rep = SimpleNamespace(AnnotatorRegistry=SimpleNamespace(get_annotator=lambda name: registry[name]))
        self.enterContext(patch.dict(sys.modules, mods))
        return mods, handlers, evaluated, guards, parsed

    def native(self, results=20, frame=2):
        class Event(dict):
            @property
            def payload(self): return dict(self)
        return Event(product_path_handle=7, results=results, frame_number=frame)

    def test_real_adapter_one_immediate_evaluate_no_pixels_until_scheduled_update(self):
        r = rig(); mods, callbacks, evaluated, guards, parsed = self.sdk(r)
        r.attach_rtx_writer(); self.assertEqual(callbacks['update']['order'], 101)
        r.reference_ann.value = {'referenceTimeNumerator': 2, 'referenceTimeDenominator': 10}
        with patch('time.perf_counter', return_value=42.): callbacks['native']['on_event'](self.native())
        self.assertEqual(len(evaluated), 1); self.assertFalse(r.rtx_cache); self.assertFalse(r.rgb_ann.calls)
        ready(r, 2, 20); r.rgb_ann.value[:] = 13
        callbacks['update']['on_event']({})
        self.assertEqual(len(evaluated), 1); self.assertEqual(int(r.rtx_cache[0]['rgb'].max()), 13)
        self.assertEqual(r.rtx_cache[0]['arrival_wall_s'], 42.)
        callbacks['stop']['on_event']({}); self.assertTrue(r._binding_stopped)
        self.assertEqual([g.resets for g in guards], [1, 1, 1])

    def test_adapter_other_rp_zero_and_duplicate_callbacks_preserve_first_arrival(self):
        r = rig(); mods, c, evaluated, guards, parsed = self.sdk(r); r.attach_rtx_writer()
        c['update']['on_event']({}); self.assertFalse(r.rgb_ann.calls)
        parsed[0] = '/other'; c['native']['on_event'](self.native()); self.assertFalse(evaluated)
        parsed[0] = str(r.product.path); ready(r, 2, 20)
        with patch('time.perf_counter', return_value=42.): c['native']['on_event'](self.native())
        with patch('time.perf_counter', return_value=99.): c['native']['on_event'](self.native())
        c['update']['on_event']({}); self.assertEqual(len(r.rtx_cache), 1)
        self.assertEqual(r.rtx_cache[0]['arrival_wall_s'], 42.); self.assertEqual(len(evaluated), 2)

    def test_original_callback_exception_is_latched_and_all_observers_stop(self):
        r = rig(); mods, c, evaluated, guards, parsed = self.sdk(r); r.attach_rtx_writer()
        parsed[2] = 0; c['native']['on_event'](self.native())
        self.assertIsInstance(r.rtx_error, ValueError); self.assertFalse(r.rtx_cache)
        self.assertEqual([g.resets for g in guards], [1, 1, 1])
        c['native']['on_event'](self.native()); self.assertEqual(len(evaluated), 1)

    def test_callback_multiple_events_before_update_cannot_label_latest_pixels_as_first(self):
        r = rig(); mods, c, evaluated, guards, parsed = self.sdk(r); r.attach_rtx_writer()
        ready(r, 2, 20); c['native']['on_event'](self.native())
        parsed[1] = 3; ready(r, 3, 30); c['native']['on_event'](self.native(30, 3))
        c['update']['on_event']({}); self.assertFalse(r.rtx_cache); self.assertIn('RP time', str(r.rtx_error))
        self.assertEqual(len(evaluated), 2)

    def test_legacy_rgbd_creation_does_not_install_native_binding(self):
        r = rig(); mods, c, evaluated, guards, parsed = self.sdk(r)
        rep = mods['omni.replicator.core']; created = []
        rep.orchestrator = SimpleNamespace(set_capture_on_play=lambda value: None)
        rep.create = SimpleNamespace(render_product=lambda *args: SimpleNamespace(path='/legacy'))
        rep.AnnotatorRegistry = SimpleNamespace(get_annotator=lambda name: (created.append(name), Ann())[1])
        with patch.object(sensors.RgbdRender, 'attach_rtx_writer', side_effect=AssertionError('legacy binding changed')):
            output = sensors.create_rgbd_annotators(SimpleNamespace(base_path='/Robot'), include_depth=True)
        self.assertEqual(created, ['rgb', 'distance_to_image_plane', 'CameraParams'])
        self.assertIsNone(output.rtx_cache); self.assertIsNone(output.rtx_listener)

    def test_original_mount_guard_quarantines_before_raising_without_publishing(self):
        self._exercise_mount_residual(native_route=False)

    def test_native_route_records_cross_epoch_residual_without_extra_control_gate(self):
        self._exercise_mount_residual(native_route=True)

    def _exercise_mount_residual(self, *, native_route):
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        r = rig()
        world = np.array([[0., -1., 0., 0.], [0., 0., 1., 0.], [-1., 0., 0., 0.], [0., 0., 0., 1.]])
        r.camera_params_ann.value['cameraViewTransform'] = np.linalg.inv(world)
        r._remember_native_event(event(r)); r._consume_after_sdk_update()
        with tempfile.TemporaryDirectory() as d:
            r.quarantine_output = Path(d)
            stream = FrameStream.__new__(FrameStream)
            stream.experiment = {"interactive_controller": "native_route"} if native_route else {}
            stream.rtx_lidar = object(); stream.rgb_outputs = r; stream.out = Path(d)
            stream.stage = None; stream.gt_diagnostic = False
            stream.mapping = SimpleNamespace(oracle_mapper=None, learned_mapper=None)
            stream.receipts = SimpleNamespace(result={}, journal=lambda *args: None)
            stream.ros = SimpleNamespace(latest_odom=None)
            stream.rgb_ann = r.rgb_ann; stream.depth_ann = None; stream.camera_params_ann = r.camera_params_ann
            stream.overview_ann = None; stream.camera = SimpleNamespace(GetPrim=lambda: None)
            stream.sensor_yaw = np.eye(3); stream.mount = np.zeros(3); stream.imu_mount = np.zeros(3)
            stream.target_speed = .5; stream.rgb_archive = None; stream.rgb_frames_acquired = 0
            stream.video_frames = 1; stream.last_rgb_native_ns = None
            measurement = SimpleNamespace(actual_time=3.315, it=3.315, pos=np.zeros(3), Rb=np.eye(3),
                                          sensor_position=np.zeros(3), camera_position=np.array([.031, 0, 0]))
            pxr = ModuleType('pxr'); pxr.UsdGeom = SimpleNamespace(Xformable=lambda prim: SimpleNamespace(ComputeLocalToWorldTransform=lambda t: np.eye(4)))
            with patch.dict(sys.modules, {'pxr': pxr}), patch.object(stream, '_process_rtx_frame') as publish:
                if native_route:
                    stream.process(iteration=0, measurement=measurement, pedestrian_view=None,
                                   pedestrian_target=None, dynamic_guard=None, camera_rows=[])
                    publish.assert_called_once()
                else:
                    with self.assertRaisesRegex(RuntimeError, 'does not follow'):
                        stream.process(iteration=0, measurement=measurement, pedestrian_view=None,
                                       pedestrian_target=None, dynamic_guard=None, camera_rows=[])
                    publish.assert_not_called()
            self.assertEqual(stream.receipts.result['rtx_rgb_render_pose_check']['forward_cosine'], 1.)
            self.assertEqual(stream.receipts.result['rtx_rgb_render_pose_check']['used_for_control'], not native_route)
            self.assertEqual((Path(d) / 'camera_quarantine/front/first_failed_sample.npz').exists(), not native_route)


if __name__ == '__main__':
    unittest.main()
