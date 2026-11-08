import unittest
from embodied_navigation.launch.exit_status import failed_exits


class LaunchExitStatusTests(unittest.TestCase):
    def test_zero_parent_does_not_hide_failed_component(self):
        self.assertEqual(failed_exits({'launch': 0}, [{'role': 'sensor_gate', 'returncode': 1}]),
                         [{'role': 'sensor_gate', 'returncode': 1}])

    def test_normal_complete_shutdown_passes(self):
        self.assertEqual(failed_exits({'launch': 0}, [{'role': 'actor', 'returncode': 0}]), [])

    def test_signal_exit_remains_a_failure(self):
        self.assertEqual(failed_exits({'launch': 0}, [{'role': 'actor', 'returncode': -2}]),
                         [{'role': 'actor', 'returncode': -2}])

    def test_parent_failure_or_unfinished_process_is_not_success(self):
        self.assertEqual(failed_exits({'launch': 1, 'other': None}, []),
                         [{'role': 'launch', 'returncode': 1}, {'role': 'other', 'returncode': None}])
