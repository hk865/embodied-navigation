"""Synthetic CPU contracts only. Real Isaac results live in separate run reports."""
from collections import deque
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import numpy as np
from PIL import Image
from embodied_navigation.simulation.dynanav.multicamera import (
    camera_calibration, validate_intrinsics, project_shared_map, nearest_pose, focal_length_for_hfov)
from embodied_navigation.simulation.dynanav.capture.multicamera_sensor import MultiViewRender, MultiCameraCapture

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('multi_snapshot', REPO / 'scripts/dynanav_exploration/multicamera_snapshot.py')
snapshot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(snapshot)


def parameters():
    return dict(renderProductResolution=[512, 512], cameraAperture=[42., 42.], cameraFocalLength=24.,
                cameraApertureOffset=[0., 0.], metersPerSceneUnit=1., cameraModel='pinhole')


def render(index=0):
    r = MultiViewRender(None, None, None, None, None)
    r.calibration = camera_calibration(index, [0., 0., 0.])
    r.rtx_cache = deque()
    return r


def accept(r, time=.2):
    r._accept_native_render_frame(bound_data=dict(rgb_ann=np.zeros((512, 512, 4), np.uint8),
        depth_ann=np.full((512, 512), 3., np.float32), camera_params_ann=parameters()),
        source_s=time, reference_time=(round(time*10), 10),
        time_evidence=dict(body_now_substituted=False, original_arrival_wall_s=42.))


class MultiCameraGeometryTests(unittest.TestCase):
    def test_full_circle_and_real_mounts(self):
        calibrations = [camera_calibration(i, [.02, 0., -.005]) for i in range(6)]
        self.assertGreater(calibrations[0]['horizontal_fov_deg'], 60.)
        for c in calibrations:
            R = np.array(c['R_base_from_optical'])
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-15)
            self.assertAlmostEqual(np.linalg.det(R), 1.)
            self.assertAlmostEqual(np.linalg.norm(np.array(c['translation_base_m'])[:2]), .3)
        # Sample every horizontal bearing: all lie in >=1 field of view.
        for theta in np.linspace(-np.pi, np.pi, 720):
            covered = any(abs(np.arctan2(np.sin(theta-c['yaw_base_rad']), np.cos(theta-c['yaw_base_rad']))) < np.deg2rad(c['horizontal_fov_deg']/2) for c in calibrations)
            self.assertTrue(covered)

    def test_same_map_point_can_belong_to_two_views(self):
        angle = np.deg2rad(30.)
        points = np.array([[5*np.cos(angle), 5*np.sin(angle), .13]])
        found = []
        for i in range(6):
            c = camera_calibration(i, [0., 0., 0.])
            z = (np.asarray(c['R_camera_from_imu']) @ points[0] + c['translation_camera_from_imu_m'])[2]
            d = np.full((512, 512), z)
            r = project_shared_map(points, [0,0,0,0,0,0,1], c, d)
            if len(r['point_indices']): found.append(i)
        self.assertEqual(found, [0, 1])

    def test_occluded_and_unknown_depth_do_not_pass(self):
        c = camera_calibration(0, [0., 0., 0.])
        points = np.array([[3.3, 0., .13], [5.3, 0., .13], [-1., 0., .13], [np.nan, 0., 0.]])
        for d, count in ((np.full((512,512),3.),1), (np.full((512,512),np.inf),0), (np.zeros((512,512)),0)):
            r = project_shared_map(points, [0,0,0,0,0,0,1], c, d)
            self.assertEqual(len(r['point_indices']), count)
            if count: np.testing.assert_array_equal(r['point_indices'], [0])

    def test_estimator_pose_inverse_and_camera_mount_once(self):
        c = camera_calibration(0, [.02, 0., -.005])
        # IMU translated +10 x, yaw90: camera centre = [10, .28, .135].
        pose = [10, 0, 0, 0, 0, np.sqrt(.5), np.sqrt(.5)]
        points = np.array([[10., 3.28, .135]])
        r = project_shared_map(points, pose, c, np.full((512,512),3.))
        np.testing.assert_allclose(r['pixels_uv'], [[256,256]], atol=1e-10)

    def test_pose_timestamp_and_quaternion_fail_closed(self):
        pose = np.array([[0,0,0,0,0,0,1.]])
        p, receipt = nearest_pose(np.array([100_000_000]), pose, 100_000_001)
        self.assertEqual(receipt['signed_pose_camera_delta_ns'], -1)
        self.assertFalse(receipt['source_timestamps_rewritten'])
        with self.assertRaises(ValueError): nearest_pose(np.array([100_000_000]), pose, 140_000_000)
        with self.assertRaises(ValueError): nearest_pose(np.array([.1]), pose, 1)
        with self.assertRaises(ValueError):
            project_shared_map(np.zeros((1,3)), [0]*7, camera_calibration(0,[0,0,0]), np.ones((512,512)))

    def test_unsigned_pose_epochs_choose_the_actual_nearest(self):
        poses = np.array([[1,0,0,0,0,0,1], [2,0,0,0,0,0,1]], dtype=float)
        pose, receipt = nearest_pose(np.array([100, 200], dtype=np.uint64), poses, 110)
        self.assertEqual(pose[0], 1)
        self.assertEqual(receipt['signed_pose_camera_delta_ns'], -10)
        with self.assertRaises(ValueError):
            nearest_pose(np.array([2**64-1], dtype=np.uint64), poses[:1], 110)

    def test_native_intrinsics_not_world_pose(self):
        p = parameters(); p['cameraViewTransform'] = np.full((4,4),np.nan)
        validate_intrinsics(p)
        for key, value in [('cameraFocalLength', 12.), ('metersPerSceneUnit', .01), ('cameraModel','fisheye')]:
            p = parameters(); p[key] = value
            with self.assertRaises(ValueError): validate_intrinsics(p)


class MultiCameraArchiveTests(unittest.TestCase):
    def test_owned_pixels_original_clock_and_bounded_queue(self):
        r = render(); accept(r)
        self.assertEqual(r.rtx_cache[0]['stamp_ns'], 200_000_000)
        self.assertFalse(r.rtx_cache[0]['time_evidence']['body_now_substituted'])
        accept(r); self.assertEqual(len(r.rtx_cache), 1)
        with self.assertRaises(ValueError): accept(r, .1)
        for t in (.3,.4,.5): accept(r,t)
        with self.assertRaises(RuntimeError): accept(r,.6)
        r.reset_start(1_000_000_000)
        self.assertEqual(len(r.rtx_cache), 0)
        accept(r, .7); self.assertEqual(len(r.rtx_cache),0)

    def test_archive_rate_does_not_retimestamp_or_overwrite_generation(self):
        with TemporaryDirectory() as root:
            r = render(); primary = SimpleNamespace()
            c = MultiCameraCapture([r], Path(root)/'multicamera', primary)
            receipts = SimpleNamespace(result={})
            for t in (.2,.3,1.2):
                accept(r,t); c.write_pending(receipts,t)
            records = [json.loads(s) for s in (Path(root)/'multicamera/index.jsonl').read_text().splitlines()]
            self.assertEqual([r['stamp_ns'] for r in records], [200_000_000,1_200_000_000])
            self.assertNotIn('cameraViewTransform', json.dumps(records))
            self.assertFalse(records[0]['extra_depth_slam_or_planner_input'])

    def test_capture_summary_distinguishes_native_and_archived_frames(self):
        with TemporaryDirectory() as root:
            view = render()
            primary = SimpleNamespace(rtx_error=None, rtx_callback_count=3,
                                      native_frame_drop_count=0)
            capture = MultiCameraCapture([view], Path(root)/'multicamera', primary)
            for source in (.2,.3,1.2):
                accept(view, source)
                capture.write_pending(SimpleNamespace(result={}), source)
            summary = capture.close()['camera_0']
            self.assertEqual(summary['accepted_native_frames'], 3)
            self.assertEqual(summary['archived_frames'], 2)
            self.assertEqual(summary['pending_frames'], 0)
            self.assertEqual(summary['callbacks'], 3)
            self.assertIsNone(primary.bound_frame_observer)

    def test_snapshot_shared_map_and_missing_or_wrong_generation_rejected(self):
        with TemporaryDirectory() as root:
            run = Path(root)/'capture'; run.mkdir()
            views = [render(i) for i in range(6)]
            c = MultiCameraCapture(views, run/'multicamera', SimpleNamespace())
            for v in views: accept(v)
            c.write_pending(SimpleNamespace(result={}),.2)
            (run/'interactive').mkdir()
            source = run/'interactive/planner_measured_map.npz'
            np.savez_compressed(source, points_xyz_m=np.array([[3.3,0,.13],[0,3.3,.13]]),
                point_source_stamp_ns=np.array([200_000_000,300_000_000]),
                lio_pose_stamp_ns=np.array([200_000_000]),
                lio_pose_xyz_xyzw=np.array([[0,0,0,0,0,0,1.]]),
                provenance_json=json.dumps(dict(run_id='capture',map_frame='dynanav_lio_map',child_frame='go2_imu',pairing='exact_integer_nanoseconds',transform='none_already_world')))
            report = snapshot.build_snapshot(run, Path(root)/'output')
            self.assertEqual(report['map_projection']['shared_causal_input_points'],1)
            self.assertEqual(report['map_projection']['future_point_exclusions'],1)
            self.assertEqual(report['map_projection']['cameras'][0]['counts']['depth_supported_points'],1)
            self.assertTrue((Path(root)/'output/six_views.png').is_file())
            records = report['source_frames']
            with self.assertRaises(ValueError): snapshot.select_frames(records[:-1], [v.calibration['camera_id'] for v in views])
            records[-1]['stamp_ns'] += 100_000_000
            with self.assertRaises(ValueError): snapshot.select_frames(records, [v.calibration['camera_id'] for v in views])



class CameraCountTests(unittest.TestCase):
    def test_five_views_use_72_degree_spacing_and_preserve_primary(self):
        views=[camera_calibration(i,[.02,0,-.005],camera_count=5) for i in range(5)]
        np.testing.assert_allclose([v['yaw_base_rad'] for v in views],np.arange(5)*2*np.pi/5)
        original=camera_calibration(0,[.02,0,-.005])
        for field in ['intrinsics_fx_fy_cx_cy','R_camera_from_imu','translation_camera_from_imu_m','translation_base_m']:
            np.testing.assert_array_equal(views[0][field],original[field])
        for angle in np.linspace(-np.pi,np.pi,720):
            self.assertTrue(any(abs(np.arctan2(np.sin(angle-v['yaw_base_rad']),np.cos(angle-v['yaw_base_rad'])))<np.deg2rad(v['horizontal_fov_deg']/2) for v in views))
        # The native four-level patch guard is a distinct, narrower footprint.
        fx=views[0]['intrinsics_fx_fy_cx_cy'][0]
        tracking_fov=np.rad2deg(2*np.arctan((256-80)/fx))
        self.assertLess(tracking_fov,72.)

    def test_four_120_pinhole_calibration_and_native_model_match(self):
        views=[camera_calibration(i,[.02,0,-.005],camera_count=4,horizontal_fov_deg=120.) for i in range(4)]
        np.testing.assert_allclose([v['yaw_base_rad'] for v in views],np.arange(4)*np.pi/2)
        for c in views:
            self.assertAlmostEqual(c['horizontal_fov_deg'],120.)
            self.assertAlmostEqual(c['intrinsics_fx_fy_cx_cy'][0],256/np.sqrt(3))
            self.assertAlmostEqual(c['authored_focal_length'],21/np.sqrt(3))
            p=parameters();p['cameraFocalLength']=c['authored_focal_length']
            validate_intrinsics(p,c)
            with self.assertRaises(ValueError):validate_intrinsics(parameters(),c)
            with self.assertRaises(ValueError):validate_intrinsics(p)
        # Same original default and far-field cropped support, not scene visibility.
        self.assertEqual(focal_length_for_hfov(),24.)
        self.assertGreater(np.rad2deg(2*np.arctan(176/views[0]['intrinsics_fx_fy_cx_cy'][0])),90.)
        for bad in (0.,180.,float('nan'),float('inf'),-10.):
            with self.assertRaises(ValueError):focal_length_for_hfov(bad)

    def test_wide_front_receipt_uses_experimental_focal(self):
        from embodied_navigation.simulation.dynanav.capture.sensors import sensor_installation_receipt
        rig=SimpleNamespace(base_path='/base',imu_mount=np.zeros(3),Rcl=np.eye(3),mount=np.zeros(3),theta=0.,focal_length=21/np.sqrt(3))
        receipt=sensor_installation_receipt(rig,'no truth input',1.)
        np.testing.assert_allclose(receipt['camera_intrinsics'],[256/np.sqrt(3),256/np.sqrt(3),256,256])
        del rig.focal_length
        self.assertEqual(sensor_installation_receipt(rig,'no truth input',1.)['camera_intrinsics'][0],512*24/42)

    def test_invalid_count_and_camera_index_are_rejected(self):
        for count in [2,7,True,5.0]:
            with self.subTest(count=count),self.assertRaises(ValueError):
                camera_calibration(0,[0,0,0],camera_count=count)
        with self.assertRaises(ValueError):camera_calibration(5,[0,0,0],camera_count=5)

    def test_five_view_archive_and_grid_do_not_claim_a_sixth_camera(self):
        with TemporaryDirectory() as root:
            run=Path(root)/'capture';run.mkdir()
            views=[render(i) for i in range(5)]
            for i,view in enumerate(views):view.calibration=camera_calibration(i,[0,0,0],camera_count=5)
            capture=MultiCameraCapture(views,run/'multicamera',SimpleNamespace())
            for view in views:accept(view)
            capture.write_pending(SimpleNamespace(result={}),.2)
            output=Path(root)/'view'
            result=snapshot.build_snapshot(run,output,images_only=True)
            self.assertEqual(result['camera_count'],5)
            self.assertEqual(len(result['source_frames']),5)
            self.assertTrue((output/'view_grid.png').is_file())
            self.assertFalse((output/'six_views.png').exists())

if __name__ == '__main__': unittest.main()
