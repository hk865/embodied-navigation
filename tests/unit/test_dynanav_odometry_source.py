"""Exercise the actual capture callback without importing Isaac or starting it."""

import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

CAPTURE = (
    Path(__file__).resolve().parents[2]
    / "src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py"
)


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
        tree = ast.parse(CAPTURE.read_text())
        function = next(
            n
            for n in ast.walk(tree)
            if isinstance(n, ast.FunctionDef) and n.name == "odom_callback"
        )
        module = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
        namespace = {
            "experiment": source,
            "latest_odom": [None],
            "time": SimpleNamespace(perf_counter=lambda: 10.0),
        }
        exec(compile(module, str(CAPTURE), "exec"), namespace)  # noqa: S102 - execute only the repository callback AST, without SDK side effects.
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
            n
            for n in ast.walk(ast.parse(CAPTURE.read_text()))
            if isinstance(n, ast.Call)
        ]
        self.assertFalse(
            any(
                isinstance(n.func, ast.Attribute)
                and n.func.attr == "dump_traceback_later"
                for n in calls
            )
        )


if __name__ == "__main__":
    unittest.main()
