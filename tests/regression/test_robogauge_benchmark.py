"""Preparation/launch-plan fixtures: no child process or simulator is started."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from embodied_navigation.project import project_root
from embodied_navigation.simulation.gazebo.robogauge.world import prepare, actuator_path
from embodied_navigation.navigation.robogauge.benchmark import process_commands, preflight


class RoboGaugeBenchmark(unittest.TestCase):
    def test_complete_preparation_and_local_source_plan(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ,PROJECT_DATA_ROOT=tmp):
            run,profile=prepare('single_floor_turn','fixture')
            self.assertTrue(run.is_relative_to(Path(tmp)))
            self.assertEqual(profile['localization_mode'],'fast_livo2_async')
            context=json.loads((run/'runtime_context.json').read_text())
            self.assertFalse(context['navigation_ground_truth_used'])
            self.assertEqual(context['actor_kind'],'robogauge_moe_cts')
            setup=json.loads((run/'experiment_setup.json').read_text())
            self.assertFalse(setup['existing_native_COM_velocity_PID'])
            self.assertFalse(setup['no_SLAM_or_global_matcher'])
            self.assertTrue(setup['configured_spawn_prior'])
            world=ET.parse(run/'world.sdf')
            motor=world.find(".//plugin[@name='teacher_sim::TeacherActuator']")
            self.assertEqual(motor.attrib['filename'],str(actuator_path()))
            self.assertTrue(all(node.text.startswith('file://'+tmp) for node in world.iter('uri')))
            plan=process_commands(run,socket_path='/tmp/test_not_opened.sock')
            self.assertEqual(len(plan),10)
            for role,command in plan:
                if '-m' in command:
                    name=command[command.index('-m')+1]
                    self.assertTrue((project_root()/'src'/Path(*name.split('.'))).with_suffix('.py').is_file(),(role,name))
                self.assertNotIn('StudentRuntime',' '.join(command))
            self.assertEqual(plan[-1][0],'gazebo')
            check=preflight();self.assertFalse(check['ready']);self.assertIsNotNone(check['model_error'])
            with self.assertRaises(FileExistsError):prepare('single_floor_turn','fixture')

    def test_unsupported_modes_and_path_traversal_rejected(self):
        for case,label,mode in [('bad','fixture','slam'),('single_floor_turn','../escape','slam'),('single_floor_turn','fixture','truth')]:
            with self.assertRaises(ValueError):prepare(case,label,mode)


if __name__=='__main__':unittest.main()
