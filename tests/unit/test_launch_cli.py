import contextlib
import io
import json
import unittest
from unittest import mock

from embodied_navigation.launch import main as launch


class LaunchCliTests(unittest.TestCase):
    def run_check(self, arguments, ready=True, resource_error=None):
        dependencies = {'ready': ready, 'dependencies': {}, 'simulation_started': False}
        output = io.StringIO()
        with mock.patch.object(launch, 'read', return_value={}), \
                mock.patch('embodied_navigation.launch.preflight.check', return_value=dependencies), \
                mock.patch.object(launch, 'preflight_resources', side_effect=resource_error) as resources, \
                mock.patch.object(launch.Path, 'open', side_effect=AssertionError('unexpected file write')), \
                mock.patch.object(launch.subprocess, 'Popen', side_effect=AssertionError('unexpected process')), \
                contextlib.redirect_stdout(output):
            code = launch.main(arguments)
        return code, json.loads(output.getvalue()), resources

    def test_invalid_duration_rejected_before_configuration(self):
        for value in ('nan', 'inf', '-inf', '0', '-1'):
            with self.subTest(duration=value), \
                    mock.patch.object(launch, 'read', return_value={}) as read, \
                    mock.patch('embodied_navigation.launch.preflight.check', return_value={'ready': True}), \
                    contextlib.redirect_stderr(io.StringIO()), \
                    contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    launch.main(['--check', '--duration=' + value])
                self.assertEqual(raised.exception.code, 2)
                read.assert_not_called()

    def test_finite_positive_duration_is_accepted_in_check_mode(self):
        code, result, _ = self.run_check(['--check', '--duration=0.25'])
        self.assertEqual(code, 0)
        self.assertTrue(result['ready'])

    def test_check_reports_file_readiness_without_resource_probe(self):
        for ready, expected in ((True, 0), (False, 2)):
            with self.subTest(ready=ready):
                code, result, resources = self.run_check(['--check'], ready=ready)
                self.assertEqual(code, expected)
                self.assertEqual(result['ready'], ready)
                self.assertFalse(result['simulation_started'])
                resources.assert_not_called()

    def test_explicit_resource_check_runs_once_in_check_mode(self):
        for ready, expected in ((True, 0), (False, 2)):
            with self.subTest(ready=ready):
                code, result, resources = self.run_check(['--check', '--resource-check'], ready=ready)
                self.assertEqual(code, expected)
                self.assertEqual(result['ready'], ready)
                resources.assert_called_once_with()

    def test_resource_failure_is_not_hidden_by_check_mode(self):
        with self.assertRaisesRegex(RuntimeError, 'Available memory below 4GiB'):
            self.run_check(['--check', '--resource-check'],
                           resource_error=RuntimeError('Available memory below 4GiB'))

    def test_normal_missing_dependencies_fail_before_resource_probe(self):
        with mock.patch.object(launch, 'read', return_value={}), \
                mock.patch('embodied_navigation.launch.preflight.check', return_value={'ready': False}), \
                mock.patch.object(launch, 'preflight_resources') as resources, \
                mock.patch.object(launch.Path, 'open', side_effect=AssertionError('unexpected file write')), \
                mock.patch.object(launch.subprocess, 'Popen', side_effect=AssertionError('unexpected process')):
            with self.assertRaisesRegex(RuntimeError, 'Missing runtime dependencies'):
                launch.main(['--resource-check'])
            resources.assert_not_called()


if __name__ == '__main__':
    unittest.main()
