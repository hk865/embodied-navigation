import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'src/embodied_navigation/navigation/building_navigation.py'


class ProductionSlamModeTests(unittest.TestCase):
    def mode_gate(self):
        tree = ast.parse(SOURCE.read_text())
        for node in tree.body:
            if (isinstance(node, ast.If)
                    and 'navigation_mode' in ast.unparse(node.test)):
                return ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
        self.fail('Production mode gate missing')

    def test_truth_or_implicit_mode_is_rejected_before_ros_start(self):
        for settings in ({}, {'navigation_mode': 'truth'}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                exec(compile(self.mode_gate(), str(SOURCE), 'exec'), {'settings': settings})

    def test_explicit_slam_mode_passes_the_gate(self):
        exec(compile(self.mode_gate(), str(SOURCE), 'exec'), {'settings': {'navigation_mode': 'slam'}})

    def test_no_truth_publisher_on_canonical_slam_pose_topic(self):
        for node in ast.walk(ast.parse(SOURCE.read_text())):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'create_publisher' and len(node.args) > 1
                    and isinstance(node.args[1], ast.Constant)):
                self.assertNotEqual(node.args[1].value, '/demo/slam/body_odom')


if __name__ == '__main__':
    unittest.main()
