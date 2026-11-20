"""CPU-only certificate contracts with mocked GMO/ROS; not a runtime RTX pass."""
from pathlib import Path
from types import SimpleNamespace as NS
import os
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from embodied_navigation.simulation.dynanav.capture import lidar


def gmo(start=100_000_000, frame=7, offsets=(1525, 50_000_000), valid=True):
    n = len(offsets)
    pose = lambda t: NS(timestampNs=t, posM=[0., 0., 0.], orientation=[0., 0., 0., 1.])
    return NS(numElements=n, timestampNs=start, frameId=frame, scanComplete=1,
              frameStart=pose(start + lidar.COMPLETE_SCAN_COORD_FRAME_START_PHASE_NS), frameEnd=pose(start + lidar.COMPLETE_SCAN_PERIOD_NS),
              timeOffsetNs=np.array(offsets, dtype=np.int64), channelId=np.zeros(n, dtype=np.uint32),
              x=np.full(n, 2.), y=np.zeros(n), z=np.zeros(n), scalar=np.ones(n),
              flags=np.full(n, 64 if valid else 0), modelToAppTransform=np.eye(4),
              frameOfReference=NS(name='SENSOR'), motionCompensationState=NS(name='NONCOMPENSATED'),
              elementsCoordsType=NS(name='CARTESIAN'))


def scanner(enabled=True):
    s = lidar.RtxLidar.__new__(lidar.RtxLidar)
    s.complete_certificate = lidar.CompleteScanCertificate(lidar.COMPLETE_SCAN_CONFIG_ID) if enabled else None
    s.imu_mount = np.array([.02, 0, -.005])
    s.minimum_scan_start_ns = 0
    s.last_key = None
    s.scans = 0
    return s


def parse(s, data):
    with patch.dict(sys.modules, {'isaacsim.sensors.experimental.rtx': NS(parse_generic_model_output_data=lambda raw: data)}):
        return s._parse_native(np.ones(1, dtype=np.uint8))


class CompleteScanTests(unittest.TestCase):
    def test_early_last_return_does_not_define_completion_and_old_bytes_unchanged(self):
        data = gmo(offsets=(1525, 20_000_000))
        old, certified = parse(scanner(False), data), parse(scanner(), data)
        self.assertEqual(old.records.tobytes(), certified.records.tobytes())
        self.assertEqual(old.stamp_ns, certified.stamp_ns)
        self.assertNotIn('complete_scan_certificate', old.receipt)
        cert = certified.receipt['complete_scan_certificate']
        self.assertEqual(cert['source_end_ns'], 200_000_000)
        self.assertEqual(cert['sequence'], 1)
        self.assertEqual(cert['sensor_id'], 'go2_rtx_os1')
        self.assertEqual(cert['config_id'], lidar.COMPLETE_SCAN_CONFIG_ID)
        self.assertEqual(certified.receipt['last_retained_ns'], 120_000_000)

    def test_partial_never_certified_but_next_full_can_complete_same_scan(self):
        s = scanner()
        partial = gmo(); partial.scanComplete = 0
        self.assertIsNone(parse(s, partial))
        self.assertEqual(parse(s, gmo()).receipt['complete_scan_certificate']['sequence'], 1)
        self.assertEqual(parse(s, gmo(200_000_000, 20)).receipt['complete_scan_certificate']['sequence'], 2)

    def test_wrong_boundaries_signed_rebased_offsets_and_wrong_complete_fail_closed(self):
        for change in [lambda x: setattr(x.frameStart, 'timestampNs', x.timestampNs-1),
                       lambda x: setattr(x.frameEnd, 'timestampNs', x.timestampNs+99_999_999),
                       lambda x: setattr(x, 'scanComplete', 2),
                       lambda x: setattr(x, 'timeOffsetNs', np.array([-1, 50_000_000])),
                       lambda x: setattr(x, 'timeOffsetNs', np.array([0, 100_000_000])),
                       lambda x: setattr(x, 'timeOffsetNs', np.array([0., 1.]))]:
            s = scanner(); data = gmo(); change(data)
            with self.subTest(data=data), self.assertRaises(ValueError): parse(s, data)
            with self.assertRaisesRegex(ValueError, 'session is failed'): parse(s, gmo())

    def test_duplicate_gap_backward_and_repeated_frame_fail_closed(self):
        for start, frame in [(100_000_000, 7), (300_000_000, 8), (50_000_000, 9), (200_000_000, 7)]:
            s = scanner(); parse(s, gmo())
            with self.subTest(start=start, frame=frame), self.assertRaises(ValueError):
                parse(s, gmo(start, frame))

    def test_all_filtered_and_zero_native_returns_are_explicit_empty_clouds(self):
        for data in [gmo(valid=False), gmo(offsets=()), gmo(offsets=(0,), valid=False)]:
            packet = parse(scanner(), data)
            self.assertEqual(packet.stamp_ns, 100_000_000)
            self.assertEqual(packet.records.dtype, lidar.OUSTER_DTYPE)
            self.assertEqual(packet.records.tobytes(), b'')
            self.assertEqual(packet.receipt['header_rebase_ns'], 0)
            self.assertIsNone(packet.receipt['last_retained_ns'])
            self.assertEqual(packet.receipt['complete_scan_certificate']['source_end_ns'], 200_000_000)
        with self.assertRaises(ValueError): parse(scanner(False), gmo(valid=False))
        self.assertIsNone(parse(scanner(False), gmo(offsets=())))

    def test_restart_uses_new_session_not_old_identity(self):
        a, b = scanner(), scanner()
        ca = parse(a, gmo()).receipt['complete_scan_certificate']
        cb = parse(b, gmo()).receipt['complete_scan_certificate']
        self.assertNotEqual(ca['session_id'], cb['session_id'])
        self.assertEqual((ca['sequence'], cb['sequence']), (1, 1))

    def test_atomic_envelope_uses_same_cloud_without_join_or_mutation(self):
        packet = parse(scanner(), gmo())
        def cloud_message(packet):
            return NS(header=NS(stamp=NS(sec=0, nanosec=packet.stamp_ns), frame_id='go2_imu'),
                      height=1, width=len(packet.records), point_step=32, row_step=32*len(packet.records),
                      is_dense=True, is_bigendian=False, fields=[], data=packet.records.tobytes())
        cloud = cloud_message(packet)
        with patch.dict(sys.modules, {'dynanav_sensor_msgs.msg': NS(CompleteLidarScan=NS)}), patch.object(lidar, 'pointcloud_message', side_effect=cloud_message):
            msg = lidar.complete_scan_message(packet, cloud)
            self.assertIs(msg.cloud, cloud)
            self.assertEqual(msg.source_start_ns, packet.stamp_ns)
            self.assertEqual(msg.coordinate_frame_start_ns, packet.stamp_ns + 15_000_000)
            self.assertEqual(msg.coordinate_frame_end_ns, packet.stamp_ns + 100_000_000)
            for mutate in [lambda c: setattr(c, 'data', b'wrong'),
                           lambda c: setattr(c.header.stamp, 'nanosec', 0),
                           lambda c: setattr(c.header, 'frame_id', 'wrong'),
                           lambda c: setattr(c, 'row_step', 0),
                           lambda c: setattr(c, 'is_bigendian', True),
                           lambda c: setattr(c, 'fields', [NS(name='x', offset=0, datatype=7, count=1)])]:
                wrong = cloud_message(packet); mutate(wrong)
                with self.assertRaisesRegex(ValueError, 'Supplied cloud'):
                    lidar.complete_scan_message(packet, wrong)
            packet.records['t'][0] = 100_000_000
            with self.assertRaises(ValueError): lidar.complete_scan_message(packet, cloud)
        self.assertIsNone(lidar.complete_scan_message(parse(scanner(False), gmo()), cloud))

    def test_config_attribute_attestation_rejects_mutation_and_animation(self):
        def prim(values, samples=0):
            return NS(IsValid=lambda: True, GetAttribute=lambda n: NS(Get=lambda: values[n], GetNumTimeSamples=lambda: samples))
        lidar.attest_complete_scan_prim(prim(lidar._COMPLETE_ATTRIBUTES))
        for key in lidar._COMPLETE_ATTRIBUTES:
            changed = dict(lidar._COMPLETE_ATTRIBUTES); changed[key] = -1
            with self.assertRaises(ValueError): lidar.attest_complete_scan_prim(prim(changed))
        with self.assertRaises(ValueError): lidar.attest_complete_scan_prim(prim(lidar._COMPLETE_ATTRIBUTES, 1))

    def test_empty_certificate_passes_capture_without_invented_last_firing_time(self):
        from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
        packet = parse(scanner(), gmo(valid=False))
        packet.receipt['scan_index'] = 2  # Avoid unrelated capture fixture files.
        pending = iter([packet, None])
        stream = FrameStream.__new__(FrameStream)
        stream.rtx_lidar = NS(read=lambda: next(pending), scans=2)
        stream.ros = NS(publish_rtx_cloud=Mock())
        stream.receipts = NS(result={}, journal=Mock())
        stream.publish_rtx_pending(.2)
        stream.ros.publish_rtx_cloud.assert_called_once_with(packet)
        self.assertIsNone(stream.receipts.result['rtx_lidar_last_scan']['last_retained_ns'])

    def test_ros_opt_in_publishes_atomic_envelope_and_identical_legacy_cloud(self):
        from embodied_navigation.simulation.dynanav.capture.ros_interface import create_ros_interface
        class Message:
            def __init__(self, **kwargs):
                self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
                self.fields = []
                self.__dict__.update(kwargs)
        for enabled in ('0', '1'):
            publishers = {}
            def publisher(kind, topic, qos):
                publishers[topic] = Mock()
                return publishers[topic]
            node = Mock(); node.create_publisher.side_effect = publisher
            rclpy = Mock(); rclpy.create_node.return_value = node
            modules = {
                'rclpy': rclpy, 'rclpy.qos': NS(qos_profile_sensor_data=object()),
                'rclpy.signals': NS(SignalHandlerOptions=NS(NO=0)),
                'geometry_msgs.msg': NS(TwistStamped=Message), 'nav_msgs.msg': NS(Odometry=Message),
                'rosgraph_msgs.msg': NS(Clock=Message),
                'sensor_msgs.msg': NS(**dict.fromkeys(['CameraInfo', 'Image', 'Imu', 'PointCloud2', 'PointField'], Message)),
                'dynanav_sensor_msgs.msg': NS(CompleteLidarScan=Message),
            }
            packet = parse(scanner(), gmo(valid=False))
            trace = []
            with patch.dict(sys.modules, modules), patch.dict(os.environ, {'DYNANAV_RTX_COMPLETE_SCAN': enabled, 'DYNANAV_PUBLISH_TRACE': enabled}):
                ros = create_ros_interface({'point_cloud_source': 'rtx_lidar'}, None, None, journal=lambda name, row: trace.append((name, row)))
                ros.publish_rtx_cloud(packet)
                cloud = publishers[lidar.LIDAR_TOPIC].publish.call_args.args[0]
                self.assertEqual((cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data), (0, 1, 32, 0, b''))
                if enabled == '1':
                    envelope = publishers[lidar.COMPLETE_SCAN_TOPIC].publish.call_args.args[0]
                    self.assertIs(envelope.cloud, cloud)
                    self.assertEqual(ros.published['rtx_complete_scan'], 1)
                    publishers[lidar.COMPLETE_SCAN_TOPIC].publish.assert_called_once_with(envelope)
                    self.assertEqual([row['event'] for _, row in trace], ['publish_call', 'publish_returned'])
                    for filename, row in trace:
                        self.assertEqual(filename, 'sensor_publish_trace.jsonl')
                        self.assertEqual(row['topic'], lidar.COMPLETE_SCAN_TOPIC)
                        self.assertEqual(row['source_ns'], packet.stamp_ns)
                        self.assertEqual(row['data_bytes'], len(cloud.data))
                        self.assertEqual(row['point_count'], cloud.width * cloud.height)
                        self.assertFalse(row['delivery_confirmed'])
                else:
                    self.assertNotIn(lidar.COMPLETE_SCAN_TOPIC, publishers)
                    self.assertEqual(trace, [])
                ros.destroy()

    def test_runtime_firing_schedule_must_match_frozen_asset_not_only_rate(self):
        fire = 'omni:sensor:Core:emitterState:s001:fireTimeNs'
        values = dict(lidar._COMPLETE_ATTRIBUTES, **{fire: [1525, 4576]})
        def prim(data, usd_types=None):
            def attr(name):
                return NS(Get=lambda: data[name], GetName=lambda: name, GetNumTimeSamples=lambda: 0,
                          GetTypeName=lambda: (usd_types or {}).get(name, "token")) if name in data else None
            return NS(IsValid=lambda: True, GetAttribute=attr,
                      GetAttributes=lambda: [attr(name) for name in data])
        frozen = prim(values)
        stage = NS(GetPrimAtPath=lambda _: frozen)
        with patch.dict(sys.modules, {'pxr': NS(Usd=NS(Stage=NS(Open=lambda _: stage)))}), patch.object(Path, 'read_bytes', return_value=b'fixture'), patch.object(lidar.hashlib, 'sha256', return_value=NS(hexdigest=lambda: lidar.COMPLETE_SCAN_ASSET_SHA256)):
            self.assertEqual(lidar.attest_complete_scan_asset('fixture', prim(dict(values, **lidar._COMPLETE_RUNTIME_OVERRIDES))), lidar.COMPLETE_SCAN_CONFIG_ID)
            for changed in [dict(values, **{fire: [0, 4576]}),
                            dict(values, **{fire.replace('s001', 's002'): [1525, 4576]})]:
                with self.assertRaises(ValueError): lidar.attest_complete_scan_asset('fixture', prim(dict(changed, **lidar._COMPLETE_RUNTIME_OVERRIDES)))

    def test_explicit_runtime_overrides_are_exact_even_without_frozen_schema_defaults(self):
        def prim(data, usd_types=None):
            def attr(name):
                return NS(Get=lambda: data[name], GetName=lambda: name, GetNumTimeSamples=lambda: 0,
                          GetTypeName=lambda: (usd_types or {}).get(name, "token")) if name in data else None
            return NS(IsValid=lambda: True, GetAttribute=attr,
                      GetAttributes=lambda: [attr(name) for name in data])
        for with_schema in (False, True):
            values = dict(lidar._COMPLETE_ATTRIBUTES)
            if with_schema:
                values.update(lidar._COMPLETE_RUNTIME_OVERRIDES)
                values['omni:sensor:Core:elementsCoordsType'] = 'SPHERICAL'
            frozen = prim(values)
            stage = NS(GetPrimAtPath=lambda _: frozen)
            runtime = dict(values, **lidar._COMPLETE_RUNTIME_OVERRIDES)
            with patch.dict(sys.modules, {'pxr': NS(Usd=NS(Stage=NS(Open=lambda _: stage)))}), patch.object(Path, 'read_bytes', return_value=b'fixture'), patch.object(lidar.hashlib, 'sha256', return_value=NS(hexdigest=lambda: lidar.COMPLETE_SCAN_ASSET_SHA256)):
                self.assertEqual(lidar.attest_complete_scan_asset('fixture', prim(runtime)), lidar.COMPLETE_SCAN_CONFIG_ID)
                for name in lidar._COMPLETE_RUNTIME_OVERRIDES:
                    with self.assertRaisesRegex(ValueError, "actual_usd_type='string' expected_usd_type='token'"):
                        lidar.attest_complete_scan_asset('fixture', prim(runtime, {name: 'string'}))
                    for bad in (None, 0, b'CARTESIAN', 'SPHERICAL', 'WORLD', 'COMPENSATED'):
                        changed = dict(runtime); changed[name] = bad
                        with self.subTest(name=name, bad=bad, schema=with_schema), self.assertRaisesRegex(ValueError, 'actual=.*expected='):
                            lidar.attest_complete_scan_asset('fixture', prim(changed))
                    missing = dict(runtime); missing.pop(name)
                    with self.assertRaises(ValueError): lidar.attest_complete_scan_asset('fixture', prim(missing))

    def test_native_gmo_output_semantics_still_reject_other_frames_and_encodings(self):
        for field, value in [('elementsCoordsType', 'SPHERICAL'),
                             ('frameOfReference', 'WORLD'),
                             ('motionCompensationState', 'COMPENSATED')]:
            data = gmo(); setattr(data, field, NS(name=value))
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'sensor-local, Cartesian and noncompensated'):
                parse(scanner(), data)

    def test_unknown_asset_hash_rejected_before_usd_open(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {'pxr': NS(Usd=NS(Stage=NS(Open=lambda _: self.fail('must not open untrusted hash'))))}):
            asset = Path(tmp)/'sensor.usd'; asset.write_bytes(b'wrong config')
            with self.assertRaisesRegex(ValueError, 'attested official'):
                lidar.attest_complete_scan_asset(asset)


if __name__ == '__main__':
    unittest.main()
