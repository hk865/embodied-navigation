"""Independent CPU contract for explicitly selected reverse native waypoints."""
import os
import subprocess
import sys
import json
import math
from pathlib import Path
import tempfile
import unittest

from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker
from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker


class ReverseRouteTest(unittest.TestCase):
    def route(self, goal=None, **kwargs):
        return RouteTracker([goal or [-2., 0., 0.]], [0., 0.],
                            speed=.15, motion_end=200., reverse=True, **kwargs)

    def test_reverse_moves_backward_without_half_turn(self):
        route = self.route()
        cmd = route.tick(10., [0., 0., 0.], 0., 0., 0.)
        self.assertLess(cmd[0], 0.)
        self.assertGreaterEqual(cmd[0], -.15)
        self.assertAlmostEqual(cmd[1], 0.)
        self.assertAlmostEqual(cmd[2], 0.)

    def test_reverse_rotated_frame_and_cross_track_sign(self):
        route = self.route([0., -2., 0.])
        cmd = route.tick(10., [.1, 0., 0.], math.pi / 2, 0., 0.)
        self.assertLess(cmd[0], 0.)
        self.assertGreater(cmd[1], 0.)  # body left is world -x
        self.assertAlmostEqual(cmd[2], 0.)

    def test_reverse_stall_is_not_disabled_by_negative_velocity(self):
        route = self.route()
        for i in range(82):
            route.tick(10. + .1 * i, [0., 0., 0.], 0., 0., 0.)
        self.assertEqual(route.failure, 'route_low_progress_stall')
        self.assertEqual(route.tick(19., [0., 0., 0.], 0., 0., 0.), [0., 0., 0.])

    def test_reverse_arrival_preserves_heading(self):
        route = self.route([-.25, 0., 0.])
        route.tick(10., [0., 0., 0.], 0., 0., 0.)
        for i in range(1, 15):
            cmd = route.tick(10. + .1 * i, [-.1, 0., 0.], 0., 0., 0.)
            self.assertAlmostEqual(cmd[2], 0.)
        self.assertTrue(route.arrived)
        self.assertEqual(cmd, [0., 0., 0.])

    def test_default_forward_behavior_unchanged(self):
        route = RouteTracker([[-2., 0., 0.]], [0., 0.], speed=.15)
        cmd = route.tick(10., [0., 0., 0.], 0., 0., 0.)
        self.assertEqual(cmd[0], 0.)
        self.assertGreater(abs(cmd[2]), 0.)


class ReverseAdapterTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.mailbox = Path(self.tmp.name) / 'run' / 'interactive'
        self.profile = dict(slam_pose_source='livo_post_vio', speed_mps=.15,
                            motion_end_s=200., interactive_mailbox=str(self.mailbox))
        self.adapter = NativeWaypointTracker(self.profile)

    def request(self, seq=1, **changes):
        payload = dict(version=1, run_id='run', seq=seq, action='goto',
                       pose_source='livo_post_vio', goal_xyz_m=[-2., 0., 0.], reverse=True)
        payload.update(changes)
        self.adapter._handle_request(payload)

    def tick(self, stamp=10., position=None, **changes):
        args = dict(current_sim_s=stamp, odom_stamp_s=stamp,
                    position=[0., 0., 0.] if position is None else position,
                    quat_wxyz=[1., 0., 0., 0.], estimated_velocity=None,
                    measured_yaw_rate=0., ready=True, now_wall=stamp,
                    arrival_wall=stamp, frame_id='map', child_frame_id='imu',
                    pose_source='livo_post_vio')
        args.update(changes)
        return self.adapter.tick(**args)

    def receipts(self):
        return [json.loads(line) for line in self.adapter.receipts_path.read_text().splitlines()]

    def test_reverse_is_passed_to_follower_and_stale_holds_zero(self):
        self.request()
        self.assertLess(self.tick()[0][0], 0.)
        cmd, receipt = self.tick(10., current_sim_s=10.4, now_wall=10.4)
        self.assertEqual(cmd, [0., 0., 0.])
        self.assertEqual(receipt['reason'], 'stale_sim_odometry')
        self.assertIsNotNone(self.adapter.active_waypoint)

    def test_reverse_cancel_and_stop_remain_native(self):
        self.request()
        self.assertLess(self.tick()[0][0], 0.)
        self.request(2, action='cancel')
        self.assertEqual(self.tick(10.1)[0], [0., 0., 0.])
        self.assertEqual(self.adapter.phase, 'idle')
        self.request(3, action='stop')
        self.assertTrue(self.adapter.arrived)
        self.assertEqual(self.adapter.terminal_reason, 'interactive_stop')

    def test_nonboolean_reverse_request_rejected(self):
        self.request(reverse='false')
        self.assertEqual(self.tick()[0], [0., 0., 0.])
        self.assertTrue(any('reject' in row['event'] for row in self.receipts()))
        self.assertFalse(any(row['event'] == 'waypoint_accepted' for row in self.receipts()))

    def test_scan_cannot_silently_accept_unsupported_reverse(self):
        self.adapter = PctScanWaypointTracker(self.profile)
        self.request()
        self.tick()
        rows = self.receipts()
        self.assertTrue(any('reject' in row['event'] for row in rows))
        self.assertFalse(any(row['event'] == 'waypoint_accepted' for row in rows))
        self.assertFalse((self.mailbox / 'planner_request.json').exists())

    def test_cli_reverse_reaches_request_payload(self):
        path = Path(__file__).resolve().parents[2] / 'scripts' / 'explore_waypoint_client.py'
        argv = [
            'goto', '--run-dir', str(self.mailbox.parent), '--x', '-1',
            '--y', '0', '--z', '0', '--pose-source', 'livo_post_vio', '--reverse']
        env = dict(os.environ, PYTHONPATH=str(path.parents[1] / 'src'))
        result = subprocess.run([sys.executable, '-B', str(path), *argv],
                                env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        payload = json.loads((self.mailbox / 'request.json').read_text())
        self.assertIs(payload['reverse'], True)
        self.assertEqual(payload['goal_xyz_m'], [-1., 0., 0.])


if __name__ == '__main__':
    unittest.main()
