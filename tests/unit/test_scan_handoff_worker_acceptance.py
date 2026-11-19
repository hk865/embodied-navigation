"""Black-box native handoff outcomes through the actual worker's ROS callbacks."""
import json
import tempfile
import unittest

import numpy as np

from scan_worker_acceptance_fixtures import WorkerRig


class ScanHandoffWorkerAcceptance(unittest.TestCase):
    def exercise(self, scenario):
        with tempfile.TemporaryDirectory() as directory:
            rig = WorkerRig(directory)
            rig.run_scenario(scenario)
            self.assertTrue(rig.node.destroyed)
            self.assertTrue(all(process.terminated for process in rig.processes))

    def test_subscription_counts_and_elapsed_sleep_are_not_child_consumption(self):
        def scenario(rig):
            rig.prepare()
            for _ in range(8):
                rig.poll(.2)
            self.assertEqual(rig.path_messages(), [])
            self.assertNotEqual(rig.output().get('status'), 'tracking')
            self.assertEqual(len(rig.processes), 1)
        self.exercise(scenario)

    def test_only_scoped_ack_for_actually_sent_source_can_send_path_once(self):
        def scenario(rig):
            rig.prepare()
            self.assertEqual(rig.path_messages(), [])
            for fields in (
                dict(run_id='other'), dict(planner_epoch=2),
                dict(planner_pid=rig.processes[-1].pid + 1),
                dict(source_frame_id='other'), dict(source_stamp_ns=999_000_000_000),
                dict(have_odom=False),
            ):
                rig.ack(**fields)
                rig.poll(.1)
                self.assertEqual(rig.path_messages(), [], fields)
            rig.ack()
            rig.poll(.1)
            self.assertEqual(len(rig.path_messages()), 1)
            rig.ack(ack_sequence=101)
            rig.poll(.2)
            self.assertEqual(len(rig.path_messages()), 1)
            # The PCT start snapshot drifted by 0.29 mm. Its first element is
            # a structural start, not another intended waypoint.
            path = rig.path_messages()[0]
            self.assertEqual(len(path.poses), 2)
            points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses]
            np.testing.assert_allclose(points, [[1., 0., 0.], [4., 0., 0.]], atol=1e-12, rtol=0)
        self.exercise(scenario)

    def test_motion_is_not_committed_without_matching_route_definition(self):
        def scenario(rig):
            rig.prepare(); rig.ack(); rig.poll(.1)
            rig.motion(1)
            self.assertNotEqual(rig.output().get('status'), 'tracking')
            for i, fields in enumerate((dict(run_id='other'), dict(reference_epoch=9),
                    dict(planner_process_id=rig.processes[-1].pid+1),
                    dict(reference_stamp=[9, 0])), 2):
                rig.definition(**fields)
                rig.motion(i)
                self.assertNotEqual(rig.output().get('status'), 'tracking', fields)
            rig.definition()
            rig.motion(9)
            self.assertEqual(rig.output()['status'], 'tracking')
            self.assertEqual(rig.output()['seq'], 1)
        self.exercise(scenario)

    def test_current_input_rejection_latches_failure_and_closes_only_exact_goal(self):
        def scenario(rig):
            rig.prepare(); rig.ack(); rig.poll(.1)
            rig.event()
            failed = rig.output()
            self.assertEqual(failed['status'], 'failed')
            self.assertEqual(failed['seq'], 1)
            self.assertIn('acceptance_rejection', failed['reason'])
            rig.definition(); rig.motion(1); rig.poll(.2)
            self.assertEqual(rig.output(), failed)
            self.assertEqual(len(rig.path_messages()), 1)
            command, result = rig.tick_adapter()
            self.assertEqual(command, [0., 0., 0.])
            self.assertEqual(result['status'], 'idle')
            self.assertIsNone(rig.adapter.failure)
            self.assertIsNone(rig.adapter.terminal_reason)
            receipts = [json.loads(line) for line in rig.adapter.receipts_path.read_text().splitlines()]
            closed = [row for row in receipts if row['event']=='waypoint_failed']
            self.assertEqual([row['seq'] for row in closed], [1])
            self.assertTrue(closed[0]['run_continues'])
        self.exercise(scenario)

    def test_wrong_scope_or_old_input_status_cannot_fail_current_goal(self):
        def scenario(rig):
            rig.prepare(); rig.ack(); rig.poll(.1)
            for fields in (dict(run_id='other'), dict(reference_epoch=2),
                    dict(planner_process_id=rig.processes[-1].pid+1),
                    dict(frame_id='other'), dict(reference_stamp=[9, 0])):
                rig.event(**fields)
                self.assertNotEqual(rig.output().get('status'), 'failed', fields)
            rig.definition(); rig.motion()
            self.assertEqual(rig.output()['status'], 'tracking')
        self.exercise(scenario)

    def test_child_silence_has_response_deadline_without_resend_or_respawn(self):
        def scenario(rig):
            rig.prepare()
            rig.poll(9.)
            self.assertNotEqual(rig.output().get('status'), 'failed')
            rig.poll(1.1)
            failed = rig.output()
            self.assertEqual(failed['status'], 'failed')
            self.assertEqual(failed['seq'], 1)
            rig.ack(); rig.poll(20.)
            self.assertEqual(rig.output(), failed)
            self.assertEqual(rig.path_messages(), [])
            self.assertEqual(len(rig.processes), 1)
        self.exercise(scenario)

    def test_reference_silence_fails_without_republishing_input(self):
        def scenario(rig):
            rig.prepare(); rig.ack(); rig.poll(.1)
            self.assertEqual(len(rig.path_messages()), 1)
            rig.poll(9.)
            self.assertNotEqual(rig.output().get('status'), 'failed')
            rig.poll(1.1)
            failed = rig.output()
            self.assertEqual(failed['status'], 'failed')
            rig.definition(); rig.motion(); rig.poll(20.)
            self.assertEqual(rig.output(), failed)
            self.assertEqual(len(rig.path_messages()), 1)
        self.exercise(scenario)

    def test_target_abandoned_is_failure_while_safety_stop_can_recover(self):
        def scenario(rig):
            rig.prepare(); rig.ack(); rig.poll(.1); rig.definition(); rig.motion(1)
            self.assertEqual(rig.output()['status'], 'tracking')
            rig.motion(2, kind='emergency_stop')
            self.assertEqual(rig.output()['status'], 'hold')
            rig.motion(3)
            self.assertEqual(rig.output()['status'], 'tracking')
            rig.event('target_abandoned')
            failed = rig.output()
            self.assertEqual(failed['status'], 'failed')
            rig.motion(4)
            self.assertEqual(rig.output(), failed)
        self.exercise(scenario)

    def test_cancel_closes_owned_child_and_old_events_cannot_overwrite_output(self):
        def scenario(rig):
            rig.prepare(); rig.ack(); rig.poll(.1); rig.definition(); rig.motion(1)
            rig.adapter._handle_request(dict(version=1, run_id='generation', seq=2,
                action='cancel', pose_source='livo_post_vio'))
            self.assertEqual(rig.tick_adapter()[0], [0., 0., 0.])
            rig.poll(.1)
            self.assertTrue(rig.processes[-1].terminated)
            before = rig.output()
            rig.event(); rig.motion(2); rig.poll(.1)
            self.assertEqual(rig.output(), before)
        self.exercise(scenario)

    def test_body_pose_uses_measured_vector_without_mutating_zero_twist_postvio(self):
        def scenario(rig):
            rig.prepare()
            velocity = np.array([.2, -.1, .05])
            sent = rig.node.publisher('body_pose')
            # A genuine source gap resets the source-time window.
            before = len(sent.messages)
            original = rig.observe(11.5, (0., 0., .4))
            self.assertEqual(len(sent.messages), before)
            for i in range(1, 6):
                p = np.array([0., 0., .4]) + velocity * (.1*i)
                original = rig.observe(11.5 + .1*i, tuple(p))
            measured = sent.messages[-1]
            values = measured.twist.twist.linear
            np.testing.assert_allclose([values.x, values.y, values.z], velocity, atol=1e-12, rtol=0)
            self.assertEqual([original.twist.twist.linear.x, original.twist.twist.linear.y,
                              original.twist.twist.linear.z], [0., 0., 0.])
            self.assertIsNot(measured, original)
        self.exercise(scenario)


if __name__ == '__main__':
    unittest.main()
