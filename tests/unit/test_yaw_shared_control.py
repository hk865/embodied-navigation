"""Declared yaw_action API: old control/deadband/source dwell semantics."""
import math
import unittest

from embodied_navigation.navigation.yaw_action import yaw_action_step, yaw_config


def action(**parameters):
    return dict(target_yaw_rad=None, parameters=parameters)


def context(sim, *, stamp=None, distinct=True, gyro=0., gyro_stamp=None, valid=True):
    stamp = sim if stamp is None else stamp
    return dict(distinct=distinct, sim=sim, stamp=stamp, raw_age=sim-stamp, wall_age=0.,
        imu_yaw_rate=gyro, imu_source_stamp_s=sim if gyro_stamp is None else gyro_stamp,
        imu_arrival_wall_s=sim, imu_frame="go2_imu", imu_valid=valid)


class SharedYawControlAcceptance(unittest.TestCase):
    def setUp(self):
        self.config = dict(yaw_config({}), max_sim_age_s=.25, max_wall_age_s=.5)

    def step(self, request, window, yaw, sample):
        return yaw_action_step(request, self.config, sample, window,
            yaw=yaw, now_wall=sample["sim"], future_tolerance_s=2e-6)

    def test_pi_wrap_relative_and_absolute_choose_same_short_pure_yaw(self):
        for start, delta, target, expected in (
            (math.pi-.05, .1, -math.pi+.05, .12),
            (-math.pi+.05, -.1, math.pi-.05, -.12),
        ):
            for parameters in (dict(delta_yaw_rad=delta), dict(target_yaw_rad=target)):
                with self.subTest(start=start, parameters=parameters):
                    request = action(**parameters)
                    command, window, done = self.step(request, None, start, context(10.))
                    self.assertEqual(command[:2], [0., 0.])
                    self.assertAlmostEqual(command[2], expected, places=12)
                    self.assertAlmostEqual(request["target_yaw_rad"], target, places=12)
                    self.assertIsNone(done)

    def test_gain_cap_and_control_deadband_are_separate_from_completion(self):
        for error, expected in ((.01, 0.), (.019, 0.), (.021, .0252), (.03, .036),
                                (.1, .12), (1., .15), (-1., -.15)):
            with self.subTest(error=error):
                request = action(delta_yaw_rad=error)
                command, _, done = self.step(request, None, 0., context(10.))
                self.assertEqual(command[:2], [0., 0.])
                self.assertAlmostEqual(command[2], expected, places=12)
                self.assertIsNone(done)

    def test_relative_target_freezes_at_first_pose_instead_of_drifting(self):
        request = action(delta_yaw_rad=.07)
        command, window, _ = self.step(request, None, .3, context(10.))
        self.assertAlmostEqual(request["target_yaw_rad"], .37, places=12)
        self.assertAlmostEqual(command[2], .084, places=12)
        command, _, _ = self.step(request, window, .33, context(10.01))
        self.assertAlmostEqual(request["target_yaw_rad"], .37, places=12)
        self.assertAlmostEqual(command[2], .048, places=12)

    def test_source_and_gyro_stationary_window_completes_at_half_second(self):
        request, window, completed = action(delta_yaw_rad=.03), None, None
        for index in range(51):
            stamp = 10. + index * .01
            command, window, completed = self.step(request, window, 0., context(stamp, gyro=.029))
            self.assertEqual(command[:2], [0., 0.])
            if index < 50:
                self.assertIsNone(completed)
                self.assertAlmostEqual(command[2], .036, places=12)
        self.assertEqual(command, [0., 0., 0.])
        self.assertIsNotNone(completed)
        self.assertAlmostEqual(completed["target_yaw_rad"], .03, places=12)
        self.assertAlmostEqual(completed["yaw_rad"], 0., places=12)
        self.assertAlmostEqual(completed["yaw_error_rad"], .03, places=12)
        self.assertAlmostEqual(completed["measured_yaw_rate_rps"], .029, places=12)
        self.assertEqual(completed["imu_frame"], "go2_imu")
        self.assertAlmostEqual(completed["pose_source_stamp_s"], 10.5, places=12)
        self.assertAlmostEqual(completed["imu_source_stamp_s"], 10.5, places=12)
        self.assertAlmostEqual(completed["completion_sim_s"], 10.5, places=12)
        self.assertEqual(completed["released_wz"], 0.)

    def test_duplicate_pose_or_unadvanced_gyro_cannot_complete_by_elapsed_wall_time(self):
        for frozen in ("pose", "gyro"):
            with self.subTest(frozen=frozen):
                request, window = action(delta_yaw_rad=.01), None
                for index in range(61):
                    stamp = 10. + index * .01
                    sample = context(stamp,
                        stamp=10. if frozen == "pose" else stamp,
                        distinct=index == 0 if frozen == "pose" else True,
                        gyro_stamp=10. if frozen == "gyro" else stamp)
                    _, window, completed = self.step(request, window, 0., sample)
                    self.assertIsNone(completed)

    def test_high_rate_invalid_gyro_or_evaluation_gap_resets_original_window(self):
        for disruption in ("high_rate", "invalid_gyro", "gap"):
            with self.subTest(disruption=disruption):
                request, window = action(delta_yaw_rad=.01), None
                for index in range(31):
                    _, window, done = self.step(request, window, 0., context(10.+index*.01))
                    self.assertIsNone(done)
                if disruption == "gap":
                    restart = 10.35
                    _, window, done = self.step(request, window, 0., context(restart))
                else:
                    _, window, done = self.step(request, window, 0., context(10.31,
                        gyro=.031 if disruption == "high_rate" else 0., valid=disruption != "invalid_gyro"))
                    self.assertIsNone(window)
                    restart = 10.32
                    _, window, done = self.step(request, window, 0., context(restart))
                self.assertIsNone(done)
                for index in range(1, 50):
                    _, window, done = self.step(request, window, 0., context(restart+index*.01))
                    self.assertIsNone(done)
                command, _, done = self.step(request, window, 0., context(restart+.5))
                self.assertEqual(command, [0., 0., 0.])
                self.assertIsNotNone(done)


if __name__ == "__main__":
    unittest.main()
