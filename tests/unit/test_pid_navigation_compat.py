"""Compatibility checks for the ``pid_navigation`` facade after the split.

The pre-split module exposed 27 evidence/probe helpers plus ``_drain_executor``,
``_main`` and ``main`` at its import path, and it could be started as a script
(absolute imports) as well as through ``python -m``. These tests exercise both
import compatibility and both process entries with a stub for the one third-party
message package that is not installed here; they do not start ROS or create a
node (``--help`` exits before rclpy context creation).
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
ENTRY = SRC / "embodied_navigation/navigation/pid_navigation.py"

HELPERS = [
    "_plain_json_evidence_graph",
    "_enqueue_navigation_imu_record",
    "_copy_path_receipt_metadata",
    "_final_path_receipt_snapshot",
    "_pid_receipt_json_bytes",
    "_append_pid_row_evidence",
    "_TeacherSubstageTiming",
    "_r47_begin",
    "_r47_guard_dimensions",
    "_r47_publication_anchor",
    "_r47_mark",
    "_r47_bind",
    "_r47_end",
    "_r56_observe_admission_body",
    "_r47_flush",
    "_pre_turn_external_hold_seed_wait",
    "_r67_capture_known_frame_false",
    "_r67_attach_known_frame_failure",
    "_r67_observe_known_generation_ready",
    "observe_business_gate",
    "finite_retention_reference",
    "diagnostic_profile_view",
    "admission_callback_rejected",
    "_admission_metadata_fragment",
    "_receipt_json_with_metadata_fragment",
    "_r47_initialize",
    "ResetBookkeeping",
]

PROBE = '''
import inspect
import json
import embodied_navigation.navigation.pid_navigation as navigation
missing = [name for name in {helpers!r} if not hasattr(navigation, name)]
entry_names = [
    name for name in ("_drain_executor", "_main", "main", "PIDNavigation")
    if not hasattr(navigation, name)
]
print(json.dumps({{
    "missing_helpers": missing,
    "missing_entry": entry_names,
    "main_signature": str(inspect.signature(navigation.main)),
    "private_main_signature": str(inspect.signature(navigation._main)),
    "drain_module": navigation._drain_executor.__module__,
}}))
'''


def _ros_site():
    candidates = [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/iron/lib/python3.10/site-packages",
    ]
    candidates.extend(
        entry
        for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep)
        if entry
    )
    for candidate in candidates:
        if (Path(candidate) / "rclpy").is_dir():
            return candidate
    return None


def _write_message_stub(directory):
    package = Path(directory) / "scan_planner_msgs"
    package.mkdir()
    (package / "__init__.py").write_text(
        '"""Offline stub for the uninstalled SCAN message package."""\n'
    )
    for module, base in (("msg", "_Message"), ("srv", "_Service")):
        (package / f"{module}.py").write_text(
            f'class {base}:\n'
            "    def __init__(self, *args, **kwargs):\n"
            "        pass\n\n"
            "def __getattr__(name):\n"
            f"    return type(name, ({base},), {{}})\n"
        )
    return package


class PidNavigationCompatibilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ros_site = _ros_site()
        if cls.ros_site is None:
            # Some installs put rclpy on the interpreter's default path.
            probe = subprocess.run(
                [sys.executable, "-B", "-c", "import rclpy"],
                capture_output=True,
                text=True,
                check=False,
            )
            if probe.returncode != 0:
                raise unittest.SkipTest(
                    "rclpy is not importable; facade import needs ROS 2 Python packages"
                )
        cls.temporary = tempfile.TemporaryDirectory()
        cls.stub_root = cls.temporary.name
        _write_message_stub(cls.stub_root)
        cls.environment = dict(os.environ)
        cls.environment["PYTHONPATH"] = os.pathsep.join(
            path
            for path in (cls.stub_root, str(SRC), cls.ros_site)
            if path
        )
        probe = subprocess.run(
            [sys.executable, "-B", "-c", "import rclpy"],
            env=cls.environment,
            capture_output=True,
            text=True,
            check=False,
        )
        if probe.returncode != 0:
            cls.temporary.cleanup()
            raise unittest.SkipTest("rclpy is not importable in this environment")

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_command(self, arguments):
        return subprocess.run(
            [sys.executable, "-B", *arguments],
            cwd=self.stub_root,
            env=self.environment,
            capture_output=True,
            text=True,
            check=False,
        )

    def test_import_facade_reexports_original_helpers_and_entry_signatures(self):
        result = self.run_command(["-c", PROBE.format(helpers=HELPERS)])
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads(result.stdout.strip().splitlines()[-1])
        self.assertEqual(payload["missing_helpers"], [])
        self.assertEqual(payload["missing_entry"], [])
        self.assertEqual(payload["private_main_signature"], "(shutdown_signals)")
        self.assertEqual(payload["main_signature"], "()")
        self.assertEqual(payload["drain_module"], "embodied_navigation.navigation.pid_entrypoint")

    def test_direct_script_and_module_entry_accept_help(self):
        direct = self.run_command([str(ENTRY), "--help"])
        self.assertEqual(direct.returncode, 0, direct.stderr)
        self.assertIn("usage:", direct.stdout)
        self.assertIn("--run", direct.stdout)
        self.assertNotIn("attempted relative import", direct.stderr)

        module = self.run_command(
            ["-m", "embodied_navigation.navigation.pid_navigation", "--help"]
        )
        self.assertEqual(module.returncode, 0, module.stderr)
        self.assertIn("usage:", module.stdout)
        self.assertIn("--run", module.stdout)


if __name__ == "__main__":
    unittest.main()
