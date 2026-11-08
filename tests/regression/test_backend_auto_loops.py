import unittest,time,tempfile,json
from pathlib import Path
import numpy as np
from embodied_navigation.localization_mapping.backend.worker import BackendWorker
from embodied_navigation.localization_mapping.backend.history import Keyframe
from test_backend_registration import scene

class AutomaticLoopFlows(unittest.TestCase):
    def run_history(self,moving):
        world=scene();temporary=tempfile.TemporaryDirectory();result_path=Path(temporary.name)/'correction.json';worker=BackendWorker(capacity=80,result_path=result_path)
        try:
            for i in range(65):
                true=np.eye(4)
                if moving and i<=24:
                    angle=i*2*np.pi/24;true[:3,3]=[2.5*(np.cos(angle)-1),2.5*np.sin(angle),0]
                raw=true.copy()
                if moving:raw[0,3]+=.4*min(i,32)/32
                cloud=world-true[:3,3]
                k=Keyframe.owned(42,i+1,(i+1)*1000000000,raw,np.eye(4),cloud)
                self.assertTrue(worker.try_submit(k))
            until=time.monotonic()+30
            while worker.stats['processed']<65 and time.monotonic()<until:time.sleep(.01)
            self.assertEqual(worker.stats['errors'],0,worker.stats)
            self.assertEqual(worker.stats['processed'],65,worker.stats)
            self.assertTrue(worker.close())
            if moving:
                self.assertGreaterEqual(worker.stats['verified_loops'],2,worker.stats)
                self.assertEqual(worker.stats['optimized_graphs'],1,worker.stats)
                self.assertLess(np.linalg.norm(worker.history.poses[-1][:3,3]),.10)
                proposals=[json.loads(line)for line in result_path.with_name('backend_results.jsonl').read_text().splitlines()]
                self.assertEqual(sum(r['accepted']for r in proposals),1)
                self.assertEqual(next(r for r in proposals if r['accepted'])['correction_kind'],'verified_loop_graph')
            else:
                self.assertEqual(worker.stats['loop_candidates'],0)
                self.assertEqual(worker.stats['verified_loops'],0)
                self.assertEqual(worker.stats['optimized_graphs'],0)
            print('auto_loop_flow',moving,worker.stats)
        finally:self.assertTrue(worker.close());temporary.cleanup()
    def test_stationary_samples_do_not_become_loops(self):self.run_history(False)
    def test_excursion_second_geometry_then_graph_and_single_proposal(self):self.run_history(True)

if __name__=='__main__':unittest.main()
