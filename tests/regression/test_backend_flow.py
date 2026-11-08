"""Cross-module backend acceptance, not a ROS/Gazebo performance substitute."""
from pathlib import Path
import shutil, subprocess, sys, tempfile, threading, time, unittest
import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.localization_mapping.backend.external_map import ExternalMap
from embodied_navigation.localization_mapping.backend.registration import Relocalizer, RegistrationConfig
from embodied_navigation.localization_mapping.backend.history import Keyframe, History
from embodied_navigation.localization_mapping.backend.worker import BackendWorker
from embodied_navigation.localization_mapping.backend.correction import BackendResultReader, TemporalAdmission, apply_bundle
from embodied_navigation.localization_mapping.backend.geometry import inverse, pose_distance, transform_points
from embodied_navigation.localization_mapping.backend.run import decode
from test_backend_registration import scene, transform, partial_scan


def matcher():
    return Relocalizer(ExternalMap(scene(), block_size=20, overlap=3, voxel_size=.10),
                      RegistrationConfig(voxel_size=.10, ransac_iterations=2200))


def await_result(worker, timeout=20):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        result = worker.poll()
        if result is not None: return result
        time.sleep(.005)
    raise AssertionError('backend worker did not finish')


class BackendBusinessFlows(unittest.TestCase):
    def test_external_map_async_correction_one_pose_cloud_transform(self):
        # Two different sensor poses, nonzero IMU/LiDAR extrinsics. Ground truth
        # is used only in assertions after the backend's own admission decision.
        C = transform(); C[:3, 3] = [.7, -.3, .2]
        extrinsic = np.eye(4); extrinsic[:3, 3] = [.12, -.08, .2]
        extrinsic[:3, :3] = Rotation.from_euler('xyz', [.02, -.03, .04]).as_matrix()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); reader = BackendResultReader(root/'result.json')
            worker = BackendWorker(matcher(), history_directory=root/'history', result_path=root/'result.json')
            try:
                for i in (1, 2):
                    actual_lidar = transform(); actual_lidar[:3, 3] += [i*.08, -i*.03, 0]
                    actual_body = actual_lidar @ inverse(extrinsic); raw_body = inverse(C) @ actual_body
                    scan = partial_scan(T_map_lidar=actual_lidar, seed=90+i)
                    k = Keyframe.owned(25, i, 1_000_000_000+i*250_000_000, raw_body, extrinsic, scan, 'livo')
                    self.assertTrue(worker.try_submit(k)); result = await_result(worker)
                    if i == 1:
                        self.assertFalse(result['accepted']); self.assertEqual(result['reason'], 'awaiting_temporal_confirmation')
                    else: self.assertTrue(result['accepted'], result)
                proposal = reader.poll(k.stamp_ns, 9)
                self.assertIsNotNone(proposal); self.assertEqual(proposal['parent_generation'], 9)
                self.assertIsNone(reader.poll(k.stamp_ns, 9))
                error = pose_distance(np.array(proposal['C_world_odom']), C)
                self.assertLess(error[0], .04); self.assertLess(error[1], .01)
                raw_points = transform_points(scan, raw_body @ extrinsic)
                nav = np.eye(4); nav[:3, 3] = [-.4, .2, .1]
                pose, cloud = apply_bundle(proposal['C_world_odom'], raw_body, raw_points, nav)
                np.testing.assert_allclose(pose, nav @ np.array(proposal['C_world_odom']) @ raw_body, atol=1e-12)
                np.testing.assert_allclose(cloud, transform_points(raw_points, nav @ np.array(proposal['C_world_odom'])), atol=1e-12)
                # Same captured correction, and LiDAR extrinsic applied ONCE.
                np.testing.assert_allclose(cloud, transform_points(scan, pose @ extrinsic), atol=1e-11)
                saved = History.load(root/'history/epoch_25')
                self.assertEqual(len(saved.keyframes), 2); self.assertEqual(len(saved.edges), 1)
                self.assertEqual(saved.keyframes[-1].final_stage, 'livo')
                print('backend_two_scan_flow', worker.stats)
            finally: self.assertTrue(worker.close())

    def test_cpp_spool_bounded_overload_and_stale_rejection(self):
        if shutil.which('g++') is None: self.skipTest('g++ required for real C++/Python spool flow')
        repo = Path(__file__).resolve().parents[2]
        core = repo/'third_party/fast_livo2/fast_livo2_core'
        with tempfile.TemporaryDirectory() as build:
            executable = Path(build)/'snapshot_flow'
            subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread',
                '-I'+str(core/'include'),str(core/'src/backend/snapshot_bridge.cpp'),
                str(core/'tests/backend_snapshot_flow.cpp'),'-o',str(executable)], check=True)
            subprocess.run([str(executable),str(Path(build)/'fixture')], check=True)
            k = decode(Path(build)/'fixture/paired/kf_73_1')
        self.assertEqual(k.final_stage, 'livo'); self.assertFalse(k.points_lidar.flags.writeable)
        h = History(); h.add(k)
        np.testing.assert_allclose(h.cloud()[0], [4, 1.2, 2], atol=1e-7)
        with tempfile.TemporaryDirectory() as directory:
            h.save(directory); np.testing.assert_allclose(History.load(directory).cloud(), h.cloud())
        registration = matcher().localize(partial_scan())
        self.assertTrue(registration.accepted)
        entered, release = threading.Event(), threading.Event()
        class HeldMatcher:
            def localize(self, points):
                entered.set()
                if not release.wait(10): raise TimeoutError('test barrier timeout')
                return registration
        worker = BackendWorker(HeldMatcher(), capacity=1, admission=TemporalAdmission(confirmations=1))
        source = Keyframe.owned(26, 1, 1_000_000_000, np.eye(4), np.eye(4), partial_scan())
        self.assertTrue(worker.try_submit(source)); self.assertTrue(entered.wait(2))
        next_frame = Keyframe.owned(26, 2, 5_000_000_000, np.eye(4), np.eye(4), partial_scan())
        self.assertTrue(worker.try_submit(next_frame)); samples = []
        for _ in range(2000):
            start = time.perf_counter_ns(); accepted = worker.try_submit(next_frame)
            samples.append(time.perf_counter_ns()-start); self.assertFalse(accepted)
        self.assertEqual(worker.stats['queue_high_water'], 1); self.assertEqual(worker.stats['dropped_full'], 2000)
        wrong = Keyframe.owned(27, 1, 6_000_000_000, np.eye(4), np.eye(4), partial_scan())
        self.assertFalse(worker.try_submit(wrong)); release.set(); self.assertTrue(worker.close())
        self.assertEqual(worker.stats['processed'], 2)
        stale = TemporalAdmission(confirmations=1).consider(source, registration, next_frame.stamp_ns)
        self.assertFalse(stale['accepted']); self.assertEqual(stale['reason'], 'stale_backend_result')
        print('python_queue_submit_ns', dict(p50=float(np.quantile(samples,.5)), p99=float(np.quantile(samples,.99)), max=max(samples), drops=worker.stats['dropped_full']))

    def test_verified_geometry_loop_optimizes_and_rebuilds_history(self):
        landmarks = scene(); truth_last = np.eye(4); truth_last[:3, 3] = [.15, -.08, .05]
        scan_last = transform_points(landmarks, inverse(truth_last))
        h = History(submap_size=3)
        for i, position in enumerate([[0,0,0],[2,0,0],[2,2,0],[0,2,0],[.65,.12,.05]]):
            raw = np.eye(4); raw[:3, 3] = position
            h.add(Keyframe.owned(28, i+1, (i+1)*1_000_000_000, raw, np.eye(4), landmarks if i == 0 else scan_last))
        before_cloud = h.cloud().copy(); before = np.linalg.norm(h.poses[-1][:3,3]-truth_last[:3,3])
        registration = h.verify_and_add_loop(0, len(h.keyframes)-1,
            RegistrationConfig(voxel_size=.10, ransac_iterations=2200))
        self.assertTrue(registration.accepted); optimized = h.optimize()
        self.assertTrue(optimized['optimized'], optimized)
        after = np.linalg.norm(h.poses[-1][:3,3]-truth_last[:3,3]); self.assertLess(after, before*.3)
        self.assertFalse(np.allclose(before_cloud, h.cloud()))
        np.testing.assert_array_equal(h.keyframes[-1].points_lidar, scan_last)
        with tempfile.TemporaryDirectory() as directory:
            h.save(directory); loaded = History.load(directory)
            self.assertEqual(loaded.revision, 1); self.assertEqual(loaded.submaps, [[0,1,2],[3,4]])
            self.assertEqual(loaded.edges[-1].kind, 'loop'); np.testing.assert_allclose(loaded.cloud(), h.cloud(), atol=1e-12)
        print('verified_loop_graph', dict(before_m=before, after_m=after, **optimized))


if __name__ == '__main__': unittest.main()
