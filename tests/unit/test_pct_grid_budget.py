"""Independent budget contract: actual native PCT and actual worker/CLI dispatch."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner, MeasuredPctError
from embodied_navigation.simulation.dynanav import pct_scan_worker as worker
from pct_budget_acceptance_fixtures import (
    BudgetWorkerRig, build_native, launch_without_external_processes, sparse_multilayer_points)


class NativeGridBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory(prefix='pct-budget-native-')
        cls.binary = build_native(Path(cls.work.name))
        cls.points = sparse_multilayer_points()

    @classmethod
    def tearDownClass(cls):
        cls.work.cleanup()

    def test_original_default_rejects_real_multilayer_bounds(self):
        self.assertEqual(len(self.points), 601)
        with self.assertRaisesRegex(MeasuredPctError, 'multilayer.*allocation budget'):
            MeasuredPctPlanner(self.points, native_binary=self.binary)

    def test_eight_million_plans_same_measured_cells_with_no_filtering(self):
        planner = MeasuredPctPlanner(self.points, native_binary=self.binary, max_grid_cells=8_000_000)
        self.assertEqual(planner.cost.shape, (4, 808, 938))
        self.assertEqual(planner.cost.size, 3_031_616)
        np.testing.assert_array_equal(planner.points, self.points)
        self.assertEqual(planner.provenance['measured_points'], 601)
        self.assertEqual(planner.provenance['measured_support_cells'], 600)
        start, goal = [.225, .225, .32], [4.125, .225, .32]
        result = planner.plan(start, goal)
        self.assertTrue(result['found'], result)
        self.assertEqual(result['body_path_xyz'][0], start)
        self.assertEqual(result['body_path_xyz'][-1], goal)
        self.assertFalse(result['provenance']['ground_truth_used'])
        self.assertFalse(result['provenance']['unknown_support_traversable'])
        self.assertEqual(result['provenance']['resolution_m'], .15)
        self.assertEqual(result['provenance']['start_surface_model'], {'enabled': False, 'applied': False})
        smaller = MeasuredPctPlanner(self.points, native_binary=self.binary, max_grid_cells=4_000_000)
        self.assertEqual(smaller.plan(start, goal)['body_path_xyz'], result['body_path_xyz'])
        # The distant measured return never acquires inferred support.
        unsupported = planner.plan(start, [self.points[-1, 0], self.points[-1, 1], .32])
        self.assertFalse(unsupported['found'])
        self.assertEqual(unsupported['reason'], 'goal_support_unobserved')

    def test_exact_total_budget_accepted_and_one_less_rejected(self):
        self.assertEqual(MeasuredPctPlanner(self.points, max_grid_cells=3_031_616).cost.size, 3_031_616)
        with self.assertRaisesRegex(MeasuredPctError, 'multilayer.*allocation budget'):
            MeasuredPctPlanner(self.points, max_grid_cells=3_031_615)

    def test_configured_eight_million_still_rejects_both_allocation_limits(self):
        # XY alone remains under 8M, but four measured layers exceed it.
        with self.assertRaisesRegex(MeasuredPctError, 'multilayer.*allocation budget'):
            MeasuredPctPlanner(sparse_multilayer_points(1500, 1500), max_grid_cells=8_000_000)
        # Reject the raw XY bounds before allocating its array.
        with self.assertRaisesRegex(MeasuredPctError, '^measured map.*allocation budget'):
            MeasuredPctPlanner([[.075, .075, 0.], [450.075, 450.075, 0.]], max_grid_cells=8_000_000)


class WorkerBudgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory(prefix='pct-budget-worker-native-')
        cls.binary = build_native(Path(cls.work.name))

    @classmethod
    def tearDownClass(cls):
        cls.work.cleanup()

    def check_worker(self, cli, expected, legacy=False):
        with tempfile.TemporaryDirectory(prefix='pct-budget-worker-') as directory:
            rig = BudgetWorkerRig(directory)
            rig.execute(self.binary, cli, legacy=legacy)
            self.assertEqual(len(rig.constructed), 1)
            self.assertEqual(rig.kwargs[0]['max_grid_cells'], expected)
            self.assertEqual(rig.frozen_at_construction[0]['pct_max_grid_cells'], expected)
            final = json.loads((rig.mailbox / 'planner_worker_parameters.json').read_text())
            self.assertEqual(final, rig.frozen_at_construction[0])
            self.assertEqual(final['reference_response_timeout_s'], 10.)
            self.assertEqual(final['speed'], .5)
            planner = rig.constructed[0]
            self.assertEqual(planner.cost.shape, (1, 3, 30))
            self.assertEqual(planner.provenance['measured_points'], 90)
            route = json.loads((rig.mailbox / 'planner_global_route.json').read_text())
            self.assertEqual(route['goal_xyz_m'], [4., 0., .4])
            self.assertEqual(route['body_path_xyz_m'][-1], [4., 0., .4])
            self.assertEqual(len(rig.processes), 1)
            params = rig.processes[0].params
            for key in ('manager.max_acc', 'optimization.max_acc', 'optimization.acc_tolerance'):
                self.assertEqual(params[key], 2.)

    def test_worker_cli_default_frozen_and_reaches_actual_constructor(self):
        self.check_worker([], 2_000_000)

    def test_worker_cli_eight_million_frozen_and_reaches_actual_constructor(self):
        self.check_worker(['--pct-max-grid-cells', '8000000'], 8_000_000)

    def test_original_run_namespace_without_new_field_remains_compatible(self):
        self.check_worker([], 2_000_000, legacy=True)

    def test_cli_rejects_zero_negative_and_noninteger_before_run(self):
        for value in ('0', '-1', '2.5'):
            with self.subTest(value=value), patch.object(worker, 'run') as run:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    worker.main(['--run-dir', '/not-created', '--pct-max-grid-cells', value])
                self.assertEqual(error.exception.code, 2)
                run.assert_not_called()


class LauncherBudgetTests(unittest.TestCase):
    def test_official_launcher_dispatches_default_and_explicit_budget(self):
        for extra, expected in (([], 2_000_000), (['--pct-max-grid-cells', '8000000'], 8_000_000)):
            with self.subTest(extra=extra), tempfile.TemporaryDirectory(prefix='pct-budget-launcher-') as directory:
                done, commands = launch_without_external_processes(directory, extra)
                self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
                invocations = [argv for argv in commands if argv[0] == 'scripts/run_dynanav_pct_scan.py']
                self.assertEqual(len(invocations), 2, commands)
                self.assertIn('--build-only', invocations[0])
                self.assertNotIn('--build-only', invocations[1])
                if extra:
                    for argv in invocations:
                        self.assertEqual(argv[argv.index('--pct-max-grid-cells') + 1], '8000000')
                # Replay the launcher argv through the actual worker parser.
                with patch.object(worker, 'run') as run:
                    worker.main(invocations[1][1:])
                    self.assertEqual(run.call_args.kwargs['pct_max_grid_cells'], expected)

    def test_launcher_rejects_invalid_budget_before_starting_any_component(self):
        for value in ('0', '-1', '2.5'):
            with self.subTest(value=value), tempfile.TemporaryDirectory(prefix='pct-budget-invalid-') as directory:
                done, commands = launch_without_external_processes(
                    directory, ['--pct-max-grid-cells', value])
                self.assertEqual(done.returncode, 2, done.stdout + done.stderr)
                self.assertEqual(commands, [])

    def test_launcher_freezes_default_and_explicit_backend_config(self):
        with tempfile.TemporaryDirectory(prefix='backend-cfg-default-') as directory:
            done, commands = launch_without_external_processes(directory, [])
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
            backend_cmds = [
                argv for argv in commands
                if len(argv) >= 2 and argv[0] == '-m' and argv[1] == 'embodied_navigation.localization_mapping.backend.node'
            ]
            self.assertEqual(len(backend_cmds), 1, commands)
            self.assertEqual(backend_cmds[0][backend_cmds[0].index('--odom-frame') + 1], 'dynanav_lio_map')
            cfg_idx = backend_cmds[0].index('--config')
            frozen_cfg = Path(backend_cmds[0][cfg_idx + 1])
            self.assertEqual(frozen_cfg.name, 'slam_backend_config.json')
            self.assertTrue(frozen_cfg.is_file(), frozen_cfg)
            data = json.loads(frozen_cfg.read_text(encoding='utf-8'))
            self.assertTrue(data['zupt_enabled'])
            self.assertAlmostEqual(data['icp_gravity_sigma_rad'], 0.02)

        with tempfile.TemporaryDirectory(prefix='backend-cfg-custom-') as directory:
            custom = Path(directory) / 'custom_backend.json'
            custom.write_text(json.dumps({'zupt_enabled': False, 'min_loop_travel_m': 9.0}), encoding='utf-8')
            done, commands = launch_without_external_processes(directory, ['--backend-config', str(custom)])
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
            backend_cmds = [
                argv for argv in commands
                if len(argv) >= 2 and argv[0] == '-m' and argv[1] == 'embodied_navigation.localization_mapping.backend.node'
            ]
            self.assertEqual(len(backend_cmds), 1, commands)
            frozen_cfg = Path(backend_cmds[0][backend_cmds[0].index('--config') + 1])
            data = json.loads(frozen_cfg.read_text(encoding='utf-8'))
            self.assertFalse(data['zupt_enabled'])
            self.assertAlmostEqual(data['min_loop_travel_m'], 9.0)


if __name__ == '__main__':
    unittest.main()

