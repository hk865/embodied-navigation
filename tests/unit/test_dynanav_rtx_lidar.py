"""CPU contract checks; these do not claim an RTX runtime pass."""
import unittest
import numpy as np
from embodied_navigation.simulation.dynanav.capture.lidar import (
    OUSTER_DTYPE, pack_lidar_scan,
)


class LidarWireTests(unittest.TestCase):
    def packet(self, **kwargs):
        data = dict(xyz_sensor=np.array([[2., 0., 0.], [0., 3., 0.], [-4., 0., 0.]]),
                    intensity=[.2, .3, .4], offsets_ns=np.array([-50_000_000, 0, 49_000_000]),
                    ring=np.array([0, 15, 31], dtype=np.uint32), timestamp_ns=150_000_000,
                    imu_mount=[.02, 0., -.005])
        data.update(kwargs)
        return pack_lidar_scan(**data)

    def test_native_absolute_times_survive_signed_offset_normalization(self):
        p = self.packet()
        self.assertEqual(p.stamp_ns, 100_000_000)
        np.testing.assert_array_equal(p.records['t'], [0, 50_000_000, 99_000_000])
        np.testing.assert_array_equal(p.records['ring'], [0, 15, 31])
        self.assertEqual(p.receipt['last_retained_ns'], 199_000_000)
        self.assertFalse(p.receipt['motion_compensation'])
        self.assertFalse(p.receipt['truth_pose_used'])

    def test_only_fixed_installation_translation_is_applied(self):
        p = self.packet()
        np.testing.assert_allclose(p.points[0], [1.98, 0, .255])
        self.assertEqual(OUSTER_DTYPE.itemsize, 32)
        self.assertEqual(OUSTER_DTYPE['t'], np.dtype('<u4'))
        self.assertEqual(OUSTER_DTYPE['ring'], np.dtype('u1'))

    def test_sort_keeps_native_time_ring_intensity_pairing(self):
        p = self.packet(offsets_ns=np.array([99_000_000, 0, 50_000_000]))
        np.testing.assert_array_equal(p.records['ring'], [15, 31, 0])
        np.testing.assert_allclose(p.records['intensity'], [.3, .4, .2])

    def test_bad_native_fields_fail_instead_of_fabricating_scan(self):
        cases = [dict(offsets_ns=np.array([0., 1., 2.])),
                 dict(offsets_ns=np.array([0, 0, 0])),
                 dict(offsets_ns=np.array([0, 50_000_000, 200_000_000])),
                 dict(ring=np.array([0, 1, 32])),
                 dict(imu_mount=[float('nan'), 0, 0]),
                 dict(intensity=[0, 1])]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                self.packet(**case)

    def test_invalid_returns_do_not_change_scan_header_origin(self):
        p = self.packet(xyz_sensor=np.array([[float('nan'), 0, 0], [0, 3, 0], [-4, 0, 0]]))
        self.assertEqual(p.stamp_ns, 100_000_000)
        np.testing.assert_array_equal(p.records['t'], [50_000_000, 99_000_000])
        self.assertEqual(p.receipt['invalid_returns'], 1)

    def test_sensor_self_return_filtered_without_point_budget_decimation(self):
        p = self.packet(xyz_sensor=np.array([[0.2, 0, 0], [0, 3, 0], [-4, 0, 0]]))
        self.assertEqual(len(p.records), 2)
        self.assertFalse(p.receipt['decimation'])

    def test_nonnegative_native_offsets_preserve_authoritative_scan_start(self):
        p = self.packet(offsets_ns=np.array([1525, 50_000_000, 99_998_450]))
        self.assertEqual(p.stamp_ns, 150_000_000)
        self.assertEqual(p.receipt['native_offset_min_ns'], 1525)
        self.assertEqual(p.receipt['header_rebase_ns'], 0)
        np.testing.assert_array_equal(p.records['t'], [1525, 50_000_000, 99_998_450])

    def test_native_validity_flag_rejects_finite_invalid_return(self):
        p = self.packet(valid_flags=np.array([False, True, True]))
        self.assertEqual(len(p.records), 2)
        np.testing.assert_array_equal(p.records['ring'], [15, 31])
        self.assertEqual(p.receipt['invalid_returns'], 1)


class RtxFrameTests(unittest.TestCase):
    def test_rtx_frame_publishes_native_scan_and_rgb_without_depth_or_truth(self):
        from types import SimpleNamespace
        from pathlib import Path
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        packet = LidarWireTests().packet()
        packet.receipt['scan_index'] = 2
        queue = [packet, None]
        published = []
        journal = []
        stream = FrameStream.__new__(FrameStream)
        stream.rtx_lidar = SimpleNamespace(read=lambda: queue.pop(0), scans=2)
        stream.receipts = SimpleNamespace(result={}, journal=lambda n,p: journal.append((n,p)))
        stream.experiment = {}
        stream.ros = SimpleNamespace(publish_rtx_cloud=lambda p: published.append(('scan', p.stamp_ns)),
                                     publish_rgb=lambda t,p: published.append(('rgb', t)))
        stream.video = SimpleNamespace(write=lambda *a: 1)
        stream.target_speed = .1
        stream.gt_diagnostic = False
        stream.out = Path('/unused')
        rows = []
        stream._process_rtx_frame(.2, .2, np.zeros((512,512,3), dtype=np.uint8),
                                  SimpleNamespace(get_data=lambda: np.zeros((512,512,4), dtype=np.uint8)), rows)
        self.assertEqual(published, [('scan', 100_000_000), ('rgb', .2)])
        self.assertFalse(stream.receipts.result['depth_backprojection_enabled'])
        self.assertTrue(stream.receipts.result['rtx_lidar_connected'])
        self.assertEqual(rows[0]['point_cloud_source'], 'isaac_rtx_lidar')

    def test_future_native_ray_fails_before_publishing(self):
        from types import SimpleNamespace
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        stream = FrameStream.__new__(FrameStream)
        stream.rtx_lidar = SimpleNamespace(read=lambda: LidarWireTests().packet())
        stream.receipts = SimpleNamespace(result={})
        stream.ros = SimpleNamespace(publish_rtx_cloud=lambda p: self.fail('future scan published'))
        with self.assertRaisesRegex(ValueError, 'later than'):
            stream._process_rtx_frame(.1, .1, None, None, [])


class LidarCapacityTests(unittest.TestCase):
    def test_native_wire_capacity_is_fail_closed_not_truncated(self):
        from embodied_navigation.simulation.dynanav.capture.lidar import MAX_SCAN_POINTS
        self.assertEqual(MAX_SCAN_POINTS, 65536)
        n = MAX_SCAN_POINTS + 1
        with self.assertRaisesRegex(ValueError, 'over-budget'):
            pack_lidar_scan(xyz_sensor=np.zeros((n,3)), intensity=np.zeros(n),
                            offsets_ns=np.arange(n,dtype=np.int64),
                            ring=np.zeros(n,dtype=np.uint8), timestamp_ns=0, imu_mount=[0,0,0])


class NativeRgbFrameTests(unittest.TestCase):
    def output(self):
        from collections import deque
        from embodied_navigation.simulation.dynanav.capture.sensors import RgbdRender
        result = RgbdRender(None, None, None, None, None)
        result.rtx_cache = deque()
        result.minimum_stamp_ns = 15_000_000
        return result

    def accept(self, output, t=.1, ref=None, rgb=None, params=None):
        output.accept_rtx_rgb_frame(
            source_s=t, reference_time=ref if ref is not None else (round(t*1e9), 1_000_000_000),
            rgb=rgb if rgb is not None else np.zeros((512,512,4), dtype=np.uint8),
            parameters=params if params is not None else {'cameraViewTransform': np.eye(4)},
            acquisition_physics_s=t+.115,
        )

    def test_native_image_time_is_not_replaced_by_body_or_arrival_time(self):
        out=self.output();self.accept(out,.2)
        frame=out.take_rtx_frame()
        self.assertEqual(frame['time_s'], .2)
        self.assertEqual(frame['reference_time'], [200_000_000,1_000_000_000])
        self.assertAlmostEqual(frame['acquisition_physics_s'], .315)

    def test_invalid_reference_never_uses_sdk_current_time_fallback(self):
        for ref in [(0,0), (1,-1), (-1,1), (1.,1), (1,), None]:
            out=self.output()
            with self.subTest(ref=ref), self.assertRaises(ValueError):
                out.accept_rtx_rgb_frame(source_s=.1, reference_time=ref,
                    rgb=np.zeros((512,512,4),dtype=np.uint8),parameters={'valid':True})
            self.assertFalse(out.rtx_cache)

    def test_frame_and_parameters_are_owned_copies(self):
        out=self.output();rgb=np.zeros((512,512,4),dtype=np.uint8);params={'matrix':np.eye(4)}
        self.accept(out,rgb=rgb,params=params);rgb[:]=255;params['matrix'][:]=0
        frame=out.take_rtx_frame();self.assertEqual(frame['rgb'].shape,(512,512,3))
        self.assertEqual(int(frame['rgb'].max()),0)
        np.testing.assert_array_equal(frame['camera_parameters']['matrix'],np.eye(4))

    def test_duplicates_startup_and_backwards_time_are_explicit(self):
        out=self.output();self.accept(out,0);self.assertFalse(out.rtx_cache)
        self.accept(out,.2);self.accept(out,.2);self.assertEqual(len(out.rtx_cache),1)
        with self.assertRaisesRegex(ValueError,'backwards'):self.accept(out,.1)

    def test_bounded_queue_refuses_silent_frame_loss(self):
        out=self.output()
        for t in [.1,.2,.3,.4]:self.accept(out,t)
        with self.assertRaisesRegex(RuntimeError,'overflow'):self.accept(out,.5)
        self.assertEqual([x['time_s'] for x in out.rtx_cache],[.1,.2,.3,.4])
