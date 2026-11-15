"""Candidate advice must never submit control or claim measured free space."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from embodied_navigation.planning.exploration.waypoint_candidates import (
    frontier_candidates, prepare_candidates, remaining_active_route, run_binding)


class FakePlanner:
    instances=0
    def __init__(self,*args,**kwargs):
        type(self).instances += 1
    def plan(self,start,goal):
        found=goal[0]>=0
        return dict(found=found,body_path_xyz=[start,goal] if found else [],
                    reason=None if found else 'native_no_path',provenance={})


class FakeScorer:
    baseline=None
    extra_routes=None
    def __init__(self,snapshot,**kwargs):
        type(self).baseline=kwargs['baseline_poses_xyz']
        type(self).extra_routes=kwargs['extra_baseline_routes']
    def score_route(self,route):
        return dict(utility_voxels_per_m=1.,potential_novel_voxel_count=10)


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.capture=self.root/'capture'
        self.mailbox=self.capture/'interactive';self.mailbox.mkdir(parents=True)
        self.provenance=dict(run_id='capture',map_frame='map',child_frame='imu',source_interval_ns=[10,20])
        np.savez(self.mailbox/'planner_measured_map.npz',points_xyz_m=np.array([[0,0,0],[1,0,0]]),
                 trajectory_body_xyz_m=np.array([[0,0,.3]]),provenance_json=json.dumps(self.provenance))
        self.observation=dict(run_id='capture',pose=dict(source='livo_post_vio',map_frame='map',sensor_frame='imu',
            position_xyz_m=[0,0,.3],fresh=True),waypoints=dict(active_seq=4,active_goal_xyz_m=[2,0,.3]))
        (self.mailbox/'observation.json').write_text(json.dumps(self.observation))
        self.candidates=[dict(id=name,goal_xyz_m=[x,0,.3],role='long_goal',reason='test')
                         for name,x in [('b',2),('a',1),('failed',-1)]]
        FakePlanner.instances=0
    def tearDown(self):
        self.temp.cleanup()
    def prepare(self,**kwargs):
        return prepare_candidates(self.root,self.candidates,planner_factory=FakePlanner,scorer_factory=FakeScorer,**kwargs)
    def test_read_only_reuse_ranking_and_failure(self):
        before={p.name:p.read_bytes() for p in self.mailbox.iterdir()}
        result=self.prepare()
        self.assertEqual(FakePlanner.instances,1)
        self.assertEqual(result['ranked_candidate_ids'],['a','b'])
        self.assertEqual(result['candidates'][2]['route_reason'],'native_no_path')
        self.assertIsNone(result['candidates'][2]['score'])
        self.assertEqual(before,{p.name:p.read_bytes() for p in self.mailbox.iterdir()})
        self.assertEqual(result['visual_localization']['status'],'unavailable')
        self.assertIsNone(result['visual_localization']['score'])
        self.assertTrue(result['diagnostics']['modeled_baseline_not_measured_free'])
        self.assertEqual(result['source']['map_sha256'],hashlib.sha256(before['planner_measured_map.npz']).hexdigest())
    def test_frontier_cluster_requires_explicit_z(self):
        source=dict(resolution_m=.5,level_z_m=99,frontier=dict(cell_indices=[[0,0],[1,0],[8,8]],count=8,actionable=False))
        result=frontier_candidates(source)
        self.assertEqual(len(result['suggestions']),2)
        self.assertTrue(result['source_truncated'])
        self.assertNotIn('goal_xyz_m',result['suggestions'][0])
        result=frontier_candidates(source,.3)
        self.assertTrue(all(row['goal_xyz_m'][2]==.3 for row in result['suggestions']))
        self.assertFalse(result['actionable_used_as_gate'])
    def artifact(self):
        _,binding=run_binding(self.root)
        return dict(run_id='capture',seq=4,frame_id='map',child_frame_id='imu',**binding,
                    goal_xyz_m=[2.,0.,.3],body_path_xyz_m=[[0,0,.3],[1,0,.3],[2,0,.3]])
    def test_prepare_next_subtracts_modeled_remaining_leg(self):
        (self.mailbox/'planner_global_route.json').write_text(json.dumps(self.artifact()))
        result=self.prepare(prepare_next=True)
        self.assertEqual(result['start_xyz_m'],[2,0,.3])
        self.assertEqual(result['preparation']['active_seq'],4)
        self.assertFalse(result['preparation']['installs_cached_route'])
        self.assertTrue(any(np.allclose(p,[2,0,.3]) for p in FakeScorer.extra_routes[0]))
        self.assertTrue(all(np.allclose(p,[0,0,.3]) for p in FakeScorer.baseline))
    def test_active_sequence_and_generation_binding(self):
        artifact=self.artifact();_,binding=run_binding(self.root)
        for key,value in [('seq',3),('canonical_run_path_sha256','other'),('child_frame_id','other')]:
            changed=dict(artifact,**{key:value})
            with self.assertRaisesRegex(ValueError,'binding_mismatch'):
                remaining_active_route(self.observation,changed,binding,[0,0,.3])
    def test_duplicate_ids_rejected(self):
        self.candidates.append(self.candidates[0])
        with self.assertRaisesRegex(ValueError,'unique'):
            self.prepare()
    def test_offline_overrides_and_cross_run_refusal(self):
        frozen=self.root/'live_review';frozen.mkdir()
        saved_map=frozen/'measured_snapshot.npz'
        saved_map.write_bytes((self.mailbox/'planner_measured_map.npz').read_bytes())
        saved_observation=frozen/'observation.json'
        saved_observation.write_text((self.mailbox/'observation.json').read_text())
        result=self.prepare(offline=True,snapshot_path=saved_map,observation_path=saved_observation)
        self.assertTrue(result['offline_replay'])
        self.assertFalse(result['source_times_rewritten'])
        with self.assertRaisesRegex(ValueError,'overrides require'):
            self.prepare(snapshot_path=saved_map)
        with self.assertRaisesRegex(ValueError,'outside_current_run'):
            self.prepare(offline=True,snapshot_path=self.root.parent/'other.npz')
    def test_frontier_source_and_truth_refusal(self):
        source=dict(resolution_m=.5,ground_truth_used=True,frontier=dict(cell_indices=[[0,0]]))
        path=self.root/'frontier.json';path.write_text(json.dumps(source))
        with self.assertRaisesRegex(ValueError,'truth_frontier'):
            self.prepare(frontier_path=path,frontier_z=.3)
        with self.assertRaisesRegex(ValueError,'frontier_outside'):
            self.prepare(frontier_path=self.root.parent/'other.json')
    def test_real_scorer_with_fake_route_planner(self):
        result=prepare_candidates(self.root,self.candidates,planner_factory=FakePlanner,
            scorer_options=dict(azimuth_samples=2,max_route_samples=2,max_range_m=2.,voxel_m=.5))
        self.assertEqual(len(result['ranked_candidate_ids']),2)
        self.assertIsNotNone(result['candidates'][0]['score'])
        self.assertEqual(result['source']['canonical_run_path_sha256'],
                         result['candidates'][0]['score']['provenance']['generation'])

    def test_cli_refuses_action_mailbox_output(self):
        path=Path(__file__).resolve().parents[2]/'scripts/prepare_dynanav_waypoint_candidates.py'
        spec=importlib.util.spec_from_file_location('candidate_cli',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with self.assertRaises(SystemExit) as error:
            module.main(['--run-dir',str(self.root),'--lidar-offset','0','0','.25',
                         '--output',str(self.mailbox/'request.json')])
        self.assertEqual(error.exception.code,2)
        self.assertFalse((self.mailbox/'request.json').exists())

if __name__=='__main__':
    unittest.main()
