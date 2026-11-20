"""Real C++ ``snapshot_bridge`` spool flow decoded by the D-path backend reader.

Moved from ``test_backend_flow.test_cpp_spool_bounded_overload_and_stale_rejection``.
The C++ fixture (``tests/backend_snapshot_flow.cpp``) asserts its own bounded
overload, pairing and stale-rejection behaviour; this test compiles and runs
it, then decodes the produced keyframe with ``keyframes.read_keyframe``.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.keyframes import read_keyframe, transform_points


class SnapshotBridgeFlow(unittest.TestCase):
    def test_cpp_spool_bounded_overload_and_stale_rejection(self):
        if shutil.which("g++") is None:
            self.skipTest("g++ required for real C++/Python spool flow")
        repo = Path(__file__).resolve().parents[2]
        core = repo / "third_party/fast_livo2/fast_livo2_core"
        with tempfile.TemporaryDirectory() as build:
            executable = Path(build) / "snapshot_flow"
            subprocess.run(
                [
                    "g++",
                    "-std=c++17",
                    "-O2",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-pthread",
                    "-I" + str(core / "include"),
                    str(core / "src/backend/snapshot_bridge.cpp"),
                    str(core / "tests/backend_snapshot_flow.cpp"),
                    "-o",
                    str(executable),
                ],
                check=True,
            )
            subprocess.run([str(executable), str(Path(build) / "fixture")], check=True)
            k = read_keyframe(Path(build) / "fixture/paired/kf_73_1")
        self.assertEqual(k.key, (73, 1))
        self.assertEqual(len(k.points_lidar), 5)
        world = transform_points(k.T_odom_body @ k.T_body_lidar, k.points_lidar)
        np.testing.assert_allclose(world[0], [4, 1.2, 2], atol=1e-7)


if __name__ == "__main__":
    unittest.main()
