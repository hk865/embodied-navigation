"""CPU-only virtual scan tests: no simulator, truth map, or execution calls."""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from embodied_navigation.planning.exploration.route_information import (
    CALIBRATION, MeasuredSnapshot, RouteInformationScorer, load_measured_snapshot,
    newly_measured_voxel_delta, sample_route,
)


class RouteInformationTests(unittest.TestCase):
    def snapshot(self, points=(), generation='run'):
        return MeasuredSnapshot(points, generation, 'map')

    def scorer(self, points=(), **kwargs):
        options = dict(voxel_m=.25, max_range_m=3., azimuth_samples=8,
                       lidar_offset_in_route_frame=(0,0,0), max_route_samples=16)
        options.update(kwargs)
        return RouteInformationScorer(self.snapshot(points), **options)

    def test_actual_sensor_fixture(self):
        self.assertEqual(len(CALIBRATION['elevation_deg']), 32)
        self.assertEqual(len(CALIBRATION['azimuth_offset_deg']), 32)
        self.assertAlmostEqual(max(CALIBRATION['elevation_deg']),22.1,places=4)
        self.assertEqual(CALIBRATION['far_range_m'],120.)
        self.assertEqual(CALIBRATION['asset_sha256'],
            'ec5b732ef18521efe33ccdfc4e1242e9c3ce066e9482a91d29b3cea91fc7000b')

    def test_wall_occludes_beyond_wall(self):
        wall = [[1.125,y,z] for y in np.arange(-4,4,.25)+.125
                for z in np.arange(-4,4,.25)+.125]
        clear = self.scorer()._visible_unknown(np.zeros(3), 0.)
        blocked = self.scorer(wall)._visible_unknown(np.zeros(3), 0.)
        self.assertTrue(any(k[0]>=5 and abs(k[1])<=1 for k in clear))
        self.assertFalse(any(k[0]>=5 and abs(k[1])<=1 for k in blocked))
        self.assertLess(len(blocked),len(clear))
        self.assertFalse(blocked & self.scorer(wall).occupied)

    def test_range_and_vertical_fov(self):
        scorer=self.scorer()
        keys=scorer._visible_unknown(np.zeros(3),0.)
        self.assertNotIn((0,0,8),keys)  # directly above is not an OS1 beam
        self.assertNotIn((16,0,0),keys) # beyond computation horizon
        centers=(np.array(list(keys))+.5)*.25
        self.assertTrue(np.all(np.linalg.norm(centers,axis=1)<=3.+np.sqrt(3)*.25))
        self.assertLess(abs(centers[:,2]).max(),1.4)
        fine=self.scorer(voxel_m=.1)._visible_unknown(np.zeros(3),0.)
        self.assertNotIn((0,0,0),fine) # this finer cell is wholly inside near-range

    def test_baseline_and_overlap_deduplicate(self):
        scorer=self.scorer(baseline_poses_xyz=[[0,0,0]])
        score=scorer.score_route([[0,0,0],[0,0,0]])
        self.assertEqual(score['potential_novel_voxel_count'],0)
        a=self.scorer().score_route([[0,0,0],[2,0,0]])
        b=self.scorer().score_route([[0,0,0],[1,0,0],[2,0,0],[2,0,0]])
        self.assertEqual(a,b)
        one=self.scorer().score_route([[0,0,0]])
        self.assertLess(a['prospective_unknown_voxel_count'],3*one['prospective_unknown_voxel_count'])

    def test_route_benefit_not_endpoint_only(self):
        scorer=self.scorer(baseline_poses_xyz=[[0,0,0]])
        endpoint=scorer.score_route([[0,0,0]])
        excursion=scorer.score_route([[0,0,0],[0,4,0],[0,0,0]])
        self.assertGreater(excursion['potential_novel_voxel_count'],endpoint['potential_novel_voxel_count'])
        self.assertEqual(excursion['route_length_m'],8.)
        self.assertEqual(excursion,scorer.score_route([[0,0,0],[0,4,0],[0,0,0]]))

    def test_sampling_covers_end_to_end_when_budget_coarsens(self):
        points,_,meta=sample_route([[0,0,0],[100,0,0]],.1,4)
        np.testing.assert_allclose(points[[0,-1]],[[0,0,0],[100,0,0]])
        self.assertTrue(meta['budget_coarsened'])
        self.assertEqual(len(points),4)

    def test_inferred_support_metadata_never_enters_occupancy(self):
        a=self.snapshot([[1.,0.,0.]])
        b=MeasuredSnapshot([[1.,0.,0.]],'run','map',
            {'inferred_support_cells':[[0,0,0],[1,2,3]]})
        one=RouteInformationScorer(a,max_range_m=2.,azimuth_samples=4)
        two=RouteInformationScorer(b,max_range_m=2.,azimuth_samples=4)
        self.assertEqual(one.occupied,two.occupied)
        self.assertEqual(one.baseline,two.baseline)
        out=two.score_route([[0,0,0]])
        self.assertFalse(out['assumptions']['inferred_support_used'])
        self.assertFalse(out['assumptions']['observed_free_available'])
        self.assertFalse(out['assumptions']['visual_localization_quality_available'])

    def test_measured_endpoint_delta(self):
        before=self.snapshot([[0,0,0],[.01,.01,.01]])
        after=self.snapshot([[0,0,0],[.6,0,0],[.7,0,0]])
        self.assertEqual(newly_measured_voxel_delta(before,after)['new_measured_endpoint_voxel_count'],1)
        with self.assertRaises(ValueError):
            newly_measured_voxel_delta(before,self.snapshot([[0,0,0]],'other'))

    def test_loader_retains_hash_and_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'measured.npz'
            np.savez(p,points_xyz_m=np.array([[0,0,0]]),provenance_json=json.dumps(
                {'run_id':'run','map_frame':'map','child_frame':'go2_imu','ground_truth_used':False}))
            snapshot=load_measured_snapshot(p,'run')
            self.assertEqual(snapshot.map_frame,'map')
            self.assertEqual(len(snapshot.source_metadata['source_sha256']),64)
            self.assertFalse(snapshot.points_xyz.flags.writeable)
            with self.assertRaises(ValueError):
                load_measured_snapshot(p,'other')

    def test_explicit_baseline_yaw_and_mount(self):
        scorer=self.scorer(baseline_poses_xyz=[[0,0,0]], baseline_yaws_rad=[np.pi/2],
                           lidar_offset_in_route_frame=(1,0,0))
        expected=scorer._visible_unknown(np.array([0.,1.,0.]),np.pi/2)
        self.assertEqual(scorer.baseline,expected)
        self.assertEqual(scorer.baseline_sampling["yaw_source"],"explicit")

    def test_active_route_baseline_not_starved_by_long_history(self):
        history = np.zeros((1000,3))
        route = [[0,0,0],[0,6,0],[6,6,0]]
        scorer = self.scorer(baseline_poses_xyz=history, baseline_yaws_rad=np.zeros(1000),
                             extra_baseline_routes=[route])
        result = scorer.score_route(route)
        self.assertEqual(result['potential_novel_voxel_count'],0)
        self.assertEqual(len(result['sampling']['baseline']['extra_routes']),1)
        self.assertGreater(result['sampling']['baseline']['extra_routes'][0]['sample_count'],2)
        endpoint_only = self.scorer(baseline_poses_xyz=np.vstack([history,route[-1]]))
        self.assertGreater(endpoint_only.score_route(route)['potential_novel_voxel_count'],0)

    def test_mount_transform_rotates_with_route_yaw(self):
        scorer=self.scorer(lidar_offset_in_route_frame=(1.,0.,0.))
        origins=[]
        scorer._visible_unknown=lambda origin,yaw: origins.append(origin.copy()) or set()
        scorer.score_route([[0,0,0],[0,1,0]])
        np.testing.assert_allclose(origins[0],[0,1,0],atol=1e-10)


if __name__=='__main__':
    unittest.main()
