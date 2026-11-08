"""Building production entry closure without ROS, model loading or simulation."""
import ast
from contextlib import redirect_stdout, redirect_stderr
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
from embodied_navigation.launch import building

ROOT = Path(__file__).resolve().parents[2]


class BuildingEntryClosureTests(unittest.TestCase):
    def test_prepare_rejects_truth_or_capture_before_io(self):
        with tempfile.TemporaryDirectory() as folder:
            absent = Path(folder)/'not-created'
            with patch.object(building, 'ROOT', absent):
                for mode,capture in [('truth',False),(None,False),('slam',True)]:
                    with self.subTest(mode=mode,capture=capture), self.assertRaises(ValueError):
                        building.prepare('stairs_short','rejected',mode,capture)
            self.assertFalse(absent.exists())

    def test_prepare_default_uses_slam_and_preserves_selected_parameters(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            files = ['assets/building/routes/stairs_short.json','assets/building/world.sdf.in',
                     'assets/building/sensor_bridge.yaml','assets/building/sensor_contract.json',
                     'assets/building/terrain_targets.json','configs/scenarios/building/student_profile.json',
                     'configs/scenarios/building/scan.json','configs/localization_mapping/fastlivo.yaml',
                     'configs/localization_mapping/camera.yaml']
            # Use tiny explicit fixtures, so Git-only CPU tests need no external assets.
            for relative in files:
                target=root/relative;target.parent.mkdir(parents=True,exist_ok=True)
                if relative.startswith('configs/'):
                    shutil.copy2(ROOT/relative,target)
            (root/files[0]).write_text(json.dumps({'start_world_ground':[0,0,0],
                'world_body_points':[[0,0,.32],[1,0,.32]],'length_m':1.}))
            (root/files[1]).write_text('<sdf version="1.9"><world name="fixture"><model name="go2"><pose>0 0 0 0 0 0</pose></model></world></sdf>')
            for relative in files[2:5]:(root/relative).write_text('{}')
            with patch.object(building,'ROOT',root):
                run,profile=building.prepare('stairs_short','prepared')
            context=json.loads((run/'runtime_context.json').read_text())
            setup=json.loads((run/'experiment_setup.json').read_text())
            self.assertEqual(profile['localization_mode'],'fast_livo2_async')
            self.assertFalse(context['navigation_ground_truth_used'])
            self.assertEqual(setup['navigation_mode'],'slam')
            self.assertFalse(setup['navigation_ground_truth_used'])
            self.assertFalse(setup['backend_capture_only'])
            self.assertEqual(setup['stair_speed_cap_mps'],.25)
            self.assertEqual(setup['terminal_position_PD']['Kp_xy'],4.)
            self.assertTrue((run/'building_fastlivo.json').is_file())
            self.assertIn('building_navigation_slam',(run/'world.sdf').read_text())

    def test_cli_truth_rejected_without_preparation(self):
        with patch.object(building,'prepare') as prepare, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                building.main(['--mode','truth','--prepare-only'])
            self.assertEqual(caught.exception.code,2)
            prepare.assert_not_called()

    def test_cli_default_check_reports_slam_only(self):
        output=io.StringIO()
        with redirect_stdout(output):building.main(['--check'])
        value=json.loads(output.getvalue())
        self.assertEqual(value['navigation_mode'],'slam')
        self.assertFalse(value['navigation_ground_truth_used'])

    def test_worker_rejects_non_slam_before_model_or_files(self):
        path=ROOT/'src/embodied_navigation/simulation/truth_worker.py'
        main=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
        index=next(i for i,n in enumerate(main.body) if isinstance(n,ast.If) and ast.unparse(n.test)=='not slam')
        guard=ast.fix_missing_locations(ast.Module(body=[main.body[index]],type_ignores=[]))
        with self.assertRaises(ValueError):exec(compile(guard,str(path),'exec'),{'slam':False})
        exec(compile(guard,str(path),'exec'),{'slam':True})
        model_index=next(i for i,n in enumerate(main.body) if 'StudentRuntime(' in ast.unparse(n))
        self.assertLess(index,model_index)

    def test_resource_module_import_has_no_process_side_effects(self):
        import importlib.util
        path=ROOT/'src/embodied_navigation/evaluation/process_resources.py'
        spec=importlib.util.spec_from_file_location('resource_import_test',path)
        module=importlib.util.module_from_spec(spec)
        with patch('signal.signal') as signals, patch.object(Path,'open') as opened:
            spec.loader.exec_module(module)
            signals.assert_not_called()
            opened.assert_not_called()
        self.assertTrue(callable(module.main))

    def test_process_module_and_scan_config_resolve_inside_clone(self):
        source=(ROOT/'src/embodied_navigation/launch/building.py').read_text()
        self.assertIn("'-m','embodied_navigation.evaluation.process_resources'",source)
        self.assertIn("ROOT/'third_party/scan/plan_manage/config/planner.yaml'",source)
        self.assertTrue((ROOT/'third_party/scan/plan_manage/config/planner.yaml').is_file())
        self.assertTrue((ROOT/'src/embodied_navigation/evaluation/process_resources.py').is_file())
        self.assertNotIn('tools/validation/runtime_resources.py',source)
        self.assertNotIn('src/embodied_navigation/planning/scan/',source)


if __name__=='__main__':unittest.main()
