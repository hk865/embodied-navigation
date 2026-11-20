"""Black-box native ROS delayed-consumption acceptance; no planner emulation."""
import argparse
import json
from pathlib import Path
import tempfile
import unittest

import rclpy
from ros_consumption_fixture import RosConsumptionRig

OPTIONS=None


class NativeDelayedConsumptionAcceptance(unittest.TestCase):
    def test_delayed_inputs_reply_without_resend_and_sustain_original_source_dwell(self):
        with tempfile.TemporaryDirectory(prefix='delayed-yaw-') as directory:
            rig=RosConsumptionRig(directory,OPTIONS.prefix,OPTIONS.repository)
            try:
                rig.wait_connected()
                rig.control(100.)
                rig.query(100.)
                for tick in range(1,21):
                    rig.spin_for(.01)
                    self.assertEqual(rig.control(100.+tick*.01)[0],[0.,0.,0.])
                    self.assertFalse(any(row.get('status')=='clear' for row in rig.replies))
                # A waiting request must be completed by actual consumption,
                # without needing another ROS query to trigger a new attempt.
                rig.publish_inputs(100.)
                rig.spin_for(.05)
                # Topics are independent: repeat the same input after sensor
                # pose arrival so an early first cloud cannot be dropped.
                rig.publish_inputs(100.)
                rig.spin_for(.15)
                self.assertTrue(any(row.get('status')=='clear' for row in rig.replies),
                                'No final response after the requested inputs were consumed')
                pending=[]; completed=False; longest=0.; segment=None
                for tick in range(21,321):
                    sim=100.+tick*.01
                    if tick%5==1:
                        request=rig.query(sim)
                        pending.append((tick+8,sim))
                    for due,stamp in list(pending):
                        if due<=tick:
                            rig.publish_inputs(stamp)
                            pending.remove((due,stamp))
                    rig.spin_for(.01)
                    command,feedback=rig.control(sim)
                    self.assertEqual(command[:2],[0.,0.])
                    if feedback['reason']=='yaw_adjustment':
                        segment=sim if segment is None else segment
                        longest=max(longest,sim-segment)
                    elif feedback['reason']=='yaw_complete':
                        longest=max(longest,sim-segment if segment is not None else 0.)
                        completed=True;break
                    else: segment=None
                self.assertTrue(completed,'No original 0.5 s source-dwell completion under continuous delayed input')
                self.assertGreaterEqual(longest,.5-1e-9)
                final=[row for row in rig.replies if row.get('status') in ('clear','blocked')]
                self.assertGreaterEqual(len(final),3)
                for row in final:
                    request=rig.sent[row['sample_id']]
                    self.assertGreaterEqual(row['body_stamp_ns'],request['body_stamp_ns'])
                    self.assertEqual(row['body_stamp_ns'],row['source']['body_stamp_ns'])
                    self.assertGreaterEqual(row['source']['cloud_stamp_ns'],request['minimum_cloud_stamp_ns'])
                    self.assertEqual(row['native_identity'],dict(pid=rig.process.pid,startup_nonce=rig.nonce))
                    self.assertEqual(row['source']['body_frame_id'],'acceptance_map')
                # The asynchronous transport must still expose a truly occupied
                # consumed map, including a block arriving behind newer requests.
                start=tick+1; blocking=None
                for step in range(start,start+200):
                    sim=100.+step*.01
                    if step%5==1:
                        rig.query(sim);pending.append((step+8,sim))
                    for due,stamp in list(pending):
                        if due<=step:
                            rig.publish_inputs(stamp,blocked=True);pending.remove((due,stamp))
                    rig.spin_for(.01)
                    blocking=next((row for row in reversed(rig.replies) if row.get('status')=='blocked'),None)
                    if blocking:break
                self.assertIsNotNone(blocking,'Consumed obstacle was hidden by asynchronous waiting')
                self.assertEqual(blocking['occupancy'],1)
                request=rig.sent[blocking['sample_id']]
                self.assertGreaterEqual(blocking['source']['cloud_stamp_ns'],request['minimum_cloud_stamp_ns'])
                result=dict(longest_clear_window_s=longest,completion=True,blocking=blocking,
                            sent=len(rig.sent),responses=len(rig.replies))
                Path(OPTIONS.evidence,'result.json').write_text(json.dumps(result,indent=2))
            finally:
                rig.close()
                # Preserve this new run's real messages/logs even on failure.
                import shutil
                shutil.copytree(directory,Path(OPTIONS.evidence)/Path(directory).name)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--prefix',required=True)
    parser.add_argument('--repository',required=True);parser.add_argument('--evidence',required=True)
    OPTIONS,remaining=parser.parse_known_args()
    Path(OPTIONS.evidence).mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try: unittest.main(argv=['native-delayed-consumption',*remaining])
    finally:rclpy.shutdown()
