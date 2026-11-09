"""Exercise the actual capture callbacks and entry contract without Isaac/ROS.

Only repository AST blocks are executed (the ROS callback) or the entry module is
imported in a throwaway subprocess. No estimator, Isaac app or ROS node starts.
"""

import ast
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
DYNANAV = ROOT / "src/embodied_navigation/simulation/dynanav"
CAPTURE_ENTRY = DYNANAV / "capture_go2_rgbd.py"
CAPTURE_PACKAGE = DYNANAV / "capture"
# odom_callback lives in the ROS interface module after the responsibility split.
ROS_INTERFACE = CAPTURE_PACKAGE / "ros_interface.py"
CAPTURE_SOURCES = sorted(CAPTURE_PACKAGE.glob("*.py")) + [CAPTURE_ENTRY]


def capture_trees():
    return [ast.parse(path.read_text()) for path in CAPTURE_SOURCES]


class UnavailableTwist:
    @property
    def twist(self):
        raise AssertionError("post-VIO pose-only consumer accessed unavailable twist")


def message(twist):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=2, nanosec=500000000), frame_id="map"
        ),
        child_frame_id="imu",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
            )
        ),
        twist=twist,
    )


class CaptureOdometrySource(unittest.TestCase):
    def callback(self, source):
        tree = ast.parse(ROS_INTERFACE.read_text())
        function = next(
            n
            for n in ast.walk(tree)
            if isinstance(n, ast.FunctionDef) and n.name == "odom_callback"
        )
        module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
        namespace = {
            "experiment": source,
            "latest_odom": [None],
            "learned_mapper": None,
            "learned_scorer": None,
            "time": SimpleNamespace(perf_counter=lambda: 10.0),
        }
        exec(  # noqa: S102 - execute only the repository callback AST, without SDK side effects.
            compile(module, str(ROS_INTERFACE), "exec"), namespace
        )
        return namespace

    def test_post_vio_never_reads_placeholder_twist(self):
        env = self.callback({"slam_pose_source": "livo_post_vio"})
        env["odom_callback"](message(UnavailableTwist()))
        row = env["latest_odom"][0]
        self.assertIsNone(row["estimated_velocity"])
        self.assertEqual(row["position"], [1.0, 2.0, 3.0])
        self.assertEqual(row["pose_source"], "livo_post_vio")
        self.assertEqual(row["time"], 2.5)

    def test_legacy_lio_retains_estimated_twist(self):
        env = self.callback({})
        env["odom_callback"](
            message(
                SimpleNamespace(
                    twist=SimpleNamespace(linear=SimpleNamespace(x=4.0, y=5.0, z=6.0))
                )
            )
        )
        self.assertEqual(env["latest_odom"][0]["estimated_velocity"], [4.0, 5.0, 6.0])
        self.assertEqual(env["latest_odom"][0]["pose_source"], "lio_estimate")

    def test_capture_does_not_start_crashing_background_stack_walker(self):
        calls = [
            n for tree in capture_trees() for n in ast.walk(tree) if isinstance(n, ast.Call)
        ]
        self.assertFalse(
            any(
                isinstance(n.func, ast.Attribute)
                and n.func.attr == "dump_traceback_later"
                for n in calls
            )
        )

    def test_import_has_no_side_effects_and_exposes_explicit_entrypoints(self):
        with tempfile.TemporaryDirectory() as temporary:
            workdir = Path(temporary)
            probe = (
                "import embodied_navigation.simulation.dynanav.capture_go2_rgbd as m;"
                "assert callable(m.run);"
                "assert callable(m.main);"
                "assert callable(m.parse_config)"
            )
            environment = {
                "PYTHONPATH": str(ROOT / "src"),
                "PATH": "/usr/bin:/bin",
            }
            result = subprocess.run(
                [sys.executable, "-B", "-c", probe],
                cwd=workdir,
                env=environment,
                capture_output=True,
                text=True,
                check=False,
                timeout=120,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertNotIn("RuntimeError", result.stderr)
            self.assertEqual(result.stdout, "")
            self.assertEqual(sorted(workdir.iterdir()), [])

    def test_entry_keeps_runpy_main_and_legacy_argv_contract(self):
        tree = ast.parse(CAPTURE_ENTRY.read_text())
        functions = {
            node.name: node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
        }
        self.assertIn("run", functions)
        self.assertIn("main", functions)
        main_guard = next(
            node
            for node in tree.body
            if isinstance(node, ast.If)
            and ast.unparse(node.test) == "__name__ == '__main__'"
        )
        self.assertTrue(
            any(
                isinstance(node, ast.Expr)
                and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name)
                and node.value.func.id == "main"
                for node in main_guard.body
            )
        )
        # argv[1] / argv[2] stay the output-dir / scene contract; no module-level
        # sys.argv read remains (the probe used to parse it at import time).
        parse_config = functions["parse_config"]
        argv_indices = {
            node.slice.value
            for node in ast.walk(parse_config)
            if isinstance(node, ast.Subscript)
            and isinstance(node.value, ast.Name)
            and node.value.id == "argv"
            and isinstance(node.slice, ast.Constant)
        }
        self.assertEqual(argv_indices, {1, 2})
        top_level = [
            statement
            for statement in tree.body
            if not isinstance(statement, (ast.FunctionDef, ast.ClassDef))
        ]
        self.assertFalse(
            any(
                isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id == "sys"
                and node.attr == "argv"
                for statement in top_level
                for node in ast.walk(statement)
            )
        )
        self.assertIn(
            "CaptureConfig",
            {node.name for node in tree.body if isinstance(node, ast.ClassDef)},
        )


if __name__ == "__main__":
    unittest.main()
