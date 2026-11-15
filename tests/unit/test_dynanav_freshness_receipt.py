"""Execute the real freshness receipt assignments without starting Isaac/ROS."""

import ast
import json
from pathlib import Path
import unittest

import numpy as np


CONTROL = (
    Path(__file__).resolve().parents[2]
    / "src/embodied_navigation/simulation/dynanav/capture/control.py"
)


class FreshnessReceiptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree = ast.parse(CONTROL.read_text())
        # Execute the production expressions, not a copy of the freshness rule.
        assignments = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "fresh"
                or isinstance(target, ast.Subscript)
                and isinstance(target.value, ast.Name)
                and target.value.id == "result"
                and isinstance(target.slice, ast.Constant)
                and target.slice.value == "last_freshness_check"
                for target in node.targets
            )
        ]
        if len(assignments) != 2:
            raise AssertionError("Expected freshness decision and receipt assignments")
        cls.receipt_code = compile(
            ast.Module(body=sorted(assignments, key=lambda node: node.lineno),
                       type_ignores=[]),
            str(CONTROL), "exec",
        )

    def test_numpy_freshness_is_builtin_bool_and_json_serializable(self):
        for sim_age, wall_age, expected in (
            (0.125, 0.125, True),
            (0.5, 0.125, False),
            (0.125, 0.75, False),
            (0.25, 0.125, False),
            (0.125, 0.5, False),
        ):
            with self.subTest(sim_age=sim_age, wall_age=wall_age):
                state = np.array([8.5], dtype=np.float64)
                namespace = {
                    "state": state,
                    "latest_odom": [{"time": float(state[0] - sim_age)}],
                    "odom_sim_age": state[0] - float(state[0] - sim_age),
                    "odom_wall_age": np.float64(wall_age),
                    "result": {},
                }
                exec(self.receipt_code, namespace)
                receipt = namespace["result"]["last_freshness_check"]
                self.assertIs(namespace["fresh"], expected)
                self.assertIs(receipt["fresh"], expected)
                self.assertIs(json.loads(json.dumps(namespace["result"]))
                              ["last_freshness_check"]["fresh"], expected)


if __name__ == "__main__":
    unittest.main()
