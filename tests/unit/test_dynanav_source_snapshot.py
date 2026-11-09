"""Provenance snapshots must include the code that actually executes queries."""

import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = ROOT / "src/embodied_navigation/simulation/dynanav/experiment.py"
RECONSTRUCT = ROOT / "scripts/reconstruct_dynanav_learned_person_map.py"
QUERY_IMPL = "src/embodied_navigation/localization_mapping/semantic_mapping/query.py"
QUERY_CLI = "scripts/query_scene_map.py"
DYNANAV = ROOT / "src/embodied_navigation/simulation/dynanav"
CAPTURE_ENTRY = "src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py"


def experiment_sources() -> list:
    tree = ast.parse(EXPERIMENT.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "sources":
                    return [
                        element.value
                        for element in node.value.elts
                        if isinstance(element, ast.Constant)
                        and isinstance(element.value, str)
                    ]
    raise AssertionError("sources list not found in experiment.py")


class SourceSnapshotTest(unittest.TestCase):
    def test_experiment_snapshot_has_package_query_and_legacy_cli(self):
        sources = experiment_sources()
        self.assertIn(QUERY_IMPL, sources)
        self.assertIn(QUERY_CLI, sources)

    def test_experiment_snapshot_covers_split_capture_implementation(self):
        sources = experiment_sources()
        self.assertIn(CAPTURE_ENTRY, sources)
        package = DYNANAV / "capture"
        implementation = sorted(
            f"src/embodied_navigation/simulation/dynanav/capture/{path.name}"
            for path in package.glob("*.py")
        )
        self.assertTrue(implementation)
        missing = [relative for relative in implementation if relative not in sources]
        self.assertEqual(missing, [], "source snapshot misses capture modules")

    def test_reconstruction_provenance_has_package_query_and_legacy_cli(self):
        text = RECONSTRUCT.read_text()
        self.assertIn("semantic_mapping/query.py", text)
        self.assertIn("scripts/query_scene_map.py", text)


if __name__ == "__main__":
    unittest.main()
