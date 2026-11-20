"""Actual worker cancellation remains authoritative over delayed native replies."""
import json
import tempfile
import unittest

from scan_worker_acceptance_fixtures import WorkerRig, String


class DelayedYawCancelAcceptance(unittest.TestCase):
    def test_old_reply_after_cancel_and_new_action_cannot_revive_old_child(self):
        with tempfile.TemporaryDirectory() as directory:
            rig=WorkerRig(directory)
            rig.adapter._handle_request(dict(version=1,run_id='generation',seq=2,
                action='yaw',pose_source='livo_post_vio',delta_yaw_rad=.3,
                created_wall_s=10.,ttl_s=30.),now_wall=10.)
            rig.tick_adapter(10.01)
            def scenario(active):
                active.prepare()
                request=json.loads(active.node.publisher('rotation_query').messages[-1].data)
                child=active.processes[-1]
                reply=dict(request,status='clear',occupancy=0,reason='delayed_consumption',
                    native_identity=dict(pid=child.pid,startup_nonce=request['startup_nonce']))
                path=active.mailbox/'planner_yaw_space.json'
                before=path.read_bytes()
                active.adapter._handle_request(dict(version=1,run_id='generation',seq=3,action='cancel'))
                self.assertEqual(active.tick_adapter(10.6)[0],[0.,0.,0.])
                active.node.receive('rotation_result',String(json.dumps(reply)))
                self.assertEqual(path.read_bytes(),before)
                active.poll()
                self.assertTrue(child.terminated)
                active.adapter._handle_request(dict(version=1,run_id='generation',seq=4,
                    action='yaw',pose_source='livo_post_vio',delta_yaw_rad=-.3,
                    created_wall_s=10.61,ttl_s=30.),now_wall=10.61)
                self.assertEqual(active.tick_adapter(10.61)[0],[0.,0.,0.])
                active.poll()
                new=path.read_bytes()
                for status in ('clear','blocked','failed'):
                    active.node.receive('rotation_result',String(json.dumps(dict(reply,status=status))))
                    self.assertEqual(path.read_bytes(),new)
                self.assertIsNone(active.adapter.terminal_reason)
            rig.run_scenario(scenario)
            self.assertTrue(all(child.terminated for child in rig.processes))


if __name__=='__main__':unittest.main()
