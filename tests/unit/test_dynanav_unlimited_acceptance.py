"""Independent acceptance for interactive sessions without a total time budget."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker
from embodied_navigation.simulation.dynanav import experiment
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker
from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker
from duration_acceptance_fixtures import MailboxRig


class UnlimitedSessionAcceptance(unittest.TestCase):
    def rig(self, tracker_class, deadline=None):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        return MailboxRig(tmp.name, tracker_class, deadline)

    def test_official_cli_produces_null_horizons_only_for_interactive(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / 'scene.usd'
            scene.touch()
            robot = root / 'runtime_assets/robot/go2_description/go2_description.usda'
            robot.parent.mkdir(parents=True)
            robot.touch()
            output = root / 'capture'
            stream = io.StringIO()
            with patch.object(experiment, 'data_root', return_value=root), patch(
                'embodied_navigation.robot.unitree.robogauge.runtime.verify_model'
            ), patch(
                'embodied_navigation.robot.unitree.robogauge.runtime.default_model_path',
                return_value=root / 'model.pt'
            ), contextlib.redirect_stdout(stream):
                code = experiment.main([
                    '--output', str(output), '--scene', str(scene), '--check',
                    '--interactive', '--interactive-pose-source', 'livo_post_vio',
                    '--point-cloud-source', 'rtx_lidar', '--until-stopped'])
            self.assertEqual(code, 0)
            profile = json.loads(stream.getvalue())['profile']
            self.assertIsNone(profile['duration_s'])
            self.assertIsNone(profile['motion_end_s'])
            self.assertEqual(profile['slam_max_sim_age_s'], .25)
            self.assertEqual(profile['slam_max_wall_age_s'], .5)
            self.assertFalse(output.exists())
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            experiment.main(['--output', '/unused', '--until-stopped'])
        self.assertEqual(error.exception.code, 2)

    def test_original_finite_profile_limits_are_unchanged(self):
        self.assertEqual(experiment.make_profile()['duration_s'], 90.)
        self.assertEqual(experiment.make_profile()['motion_end_s'], 85.)
        self.assertEqual(experiment.make_profile(duration_s=1800, interactive=True)['motion_end_s'], 1795.)
        for duration in (0, 7.9, 1800.1, float('inf'), float('nan')):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                experiment.make_profile(duration_s=duration, interactive=True)

    def test_route_without_horizon_continues_past_original_budget(self):
        route = RouteTracker([[4., 0., 0.]], [0., 0.], motion_end=None)
        command = route.tick(2000., [0., 0., 0.], 0., .1, 0.)
        self.assertGreater(command[0], 0.)
        self.assertIsNone(route.failure)
        finite = RouteTracker([[4., 0., 0.]], [0., 0.], motion_end=20.)
        self.assertEqual(finite.tick(20., [0., 0., 0.], 0., .1, 0.), [0., 0., 0.])
        self.assertEqual(finite.failure, 'route_motion_budget')

    def test_both_adapters_continue_after_1800_seconds(self):
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            with self.subTest(adapter=cls.__name__):
                rig = self.rig(cls)
                command, row = rig.start()
                self.assertGreater(command[0], 0.)
                self.assertIsNone(row['failure'])
                self.assertIsNone(rig.adapter.terminal_reason)
                self.assertEqual(rig.adapter.parameters['max_sim_age_s'], .25)
                self.assertEqual(rig.adapter.parameters['max_wall_age_s'], .5)

    def test_unlimited_mode_keeps_stale_and_nonfinite_measurement_zeroing(self):
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            cases = (
                dict(odom_stamp_s=2000.),
                dict(arrival_wall=1999.),
                dict(measured_yaw_rate=float('nan')),
                dict(position=[float('nan'), 0., 0.]),
                dict(pose_source='native_truth'),
            )
            for changes in cases:
                with self.subTest(adapter=cls.__name__, changes=changes):
                    rig = self.rig(cls)
                    self.assertGreater(rig.start()[0][0], 0.)
                    command, row = rig.tick(2000.4, **changes)
                    self.assertEqual(command, [0., 0., 0.])
                    self.assertEqual(row['status'], 'hold')
                    self.assertIsNotNone(rig.adapter.active_waypoint)

    def test_unlimited_mode_keeps_stall_failure_local_to_goal(self):
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            with self.subTest(adapter=cls.__name__):
                rig = self.rig(cls)
                rig.start()
                for i in range(2, 84):
                    command, _ = rig.tick(2000. + i * .1)
                failures = [row for row in rig.receipts() if row['event'] == 'waypoint_failed']
                self.assertEqual(len(failures), 1)
                self.assertEqual(failures[0]['reason'], 'route_low_progress_stall')
                self.assertTrue(failures[0]['run_continues'])
                self.assertEqual(command, [0., 0., 0.])
                self.assertIsNone(rig.adapter.failure)
                self.assertIsNone(rig.adapter.terminal_reason)
                self.assertIsNone(rig.adapter.active_waypoint)

    def test_unlimited_mode_cancel_and_stop_still_close_owned_sequence(self):
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            with self.subTest(adapter=cls.__name__):
                rig = self.rig(cls)
                rig.start()
                rig.request(2, 'cancel')
                self.assertEqual(rig.tick(2000.2)[0], [0., 0., 0.])
                self.assertIsNone(rig.adapter.active_waypoint)
                self.assertIsNone(rig.adapter.terminal_reason)
                rig.request(3, 'stop')
                self.assertEqual(rig.tick(2000.3)[0], [0., 0., 0.])
                self.assertTrue(rig.adapter.arrived)
                self.assertEqual(rig.adapter.terminal_reason, 'interactive_stop')
                self.assertEqual([(r['event'], r['seq']) for r in rig.receipts()[-2:]],
                                 [('cancel_accepted', 2), ('stop_accepted', 3)])

    def test_finite_mailbox_deadline_still_ends_run(self):
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            with self.subTest(adapter=cls.__name__):
                rig = self.rig(cls, deadline=2001.)
                rig.start()
                self.assertEqual(rig.tick(2001.)[0], [0., 0., 0.])
                self.assertEqual(rig.adapter.failure, 'route_motion_budget')
                self.assertEqual(rig.receipts()[-1]['event'], 'terminal')
                rig.request(2)
                self.assertEqual(rig.receipts()[-1]['reason'], 'run_terminal')

    def test_long_clock_float32_rounding_does_not_create_false_future_hold(self):
        sim = 16384.05
        source = float(np.float32(sim))
        self.assertGreater(source - sim, 2e-6)
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            with self.subTest(adapter=cls.__name__):
                rig = self.rig(cls)
                _, row = rig.tick(sim, odom_stamp_s=source)
                self.assertEqual(row['status'], 'idle')
                self.assertEqual(row['reason'], 'awaiting_waypoint')

    def test_clock_precision_does_not_admit_real_future_or_stale_sources(self):
        for cls in (NativeWaypointTracker, PctScanWaypointTracker):
            for sim, delta, reason in (
                (20., .001, 'future_odometry'),
                (16384.05, .02, 'future_odometry'),
                (16384.05, -.251, 'stale_sim_odometry'),
            ):
                with self.subTest(adapter=cls.__name__, sim=sim, delta=delta):
                    rig = self.rig(cls)
                    command, row = rig.tick(sim, odom_stamp_s=sim + delta)
                    self.assertEqual(command, [0., 0., 0.])
                    self.assertEqual(row['reason'], reason)


if __name__ == '__main__':
    unittest.main()
