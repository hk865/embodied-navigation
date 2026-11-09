"""Exercise capture wiring without importing Isaac, ROS, torch or model weights.

Only the repository's selected AST blocks run. The launcher test substitutes a
fake runner and dependency inspection; no estimator or ROS node is started.
"""

import ast
import json
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from embodied_navigation.simulation.dynanav.dynamic_hold import DynamicHold
from embodied_navigation.simulation.dynanav.learned_mapping import LearnedMapping

ROOT = Path(__file__).resolve().parents[2]
DYNANAV = ROOT / "src/embodied_navigation/simulation/dynanav"
CAPTURE = DYNANAV / "capture_go2_rgbd.py"
CAPTURE_PACKAGE = DYNANAV / "capture"
CAPTURE_FILES = sorted(CAPTURE_PACKAGE.glob("*.py")) + [CAPTURE]


def capture_nodes():
    """Yield ``(path, node)`` for every AST node in the split capture package."""
    for path in CAPTURE_FILES:
        for node in ast.walk(ast.parse(path.read_text())):
            yield path, node


def execute(nodes, namespace):
    module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    exec(compile(module, str(CAPTURE), "exec"), namespace)  # noqa: S102 - execute only selected repository AST blocks, without importing the SDK entry point.


def calls_method(node, owner, method):
    return any(
        isinstance(item, ast.Call)
        and isinstance(item.func, ast.Attribute)
        and isinstance(item.func.value, ast.Name)
        and item.func.value.id == owner
        and item.func.attr == method
        for item in ast.walk(node)
    )


def learned_frame_block():
    return next(
        node
        for _, node in capture_nodes()
        if isinstance(node, ast.If)
        and ast.unparse(node.test) == "learned_mapper is not None"
        and calls_method(node, "learned_mapper", "process")
    )


class CaptureLearnedWiringTests(unittest.TestCase):
    def frame_environment(self, *, fresh, guard=None, scorer=None):
        states = np.array([2, 0, 0], dtype=np.uint8)
        keep = np.array([False, True, False])
        quarantine = np.array([False, False, True])
        mapper = Mock()
        mapper.process.return_value = (
            states,
            keep,
            {"fresh_inference": fresh, "quarantine_pose_valid": True},
            quarantine,
        )
        return {
            "np": np,
            "learned_mapper": mapper,
            "packet": SimpleNamespace(
                points=np.arange(9).reshape(3, 3),
                pixel_flat_indices=np.arange(3),
            ),
            "xyz": np.arange(9).reshape(3, 3),
            "it": 1.4,
            "actual_time": 1.4,
            "rgb": np.zeros((2, 2, 3), np.uint8),
            "zz": np.ones((2, 2)),
            "latest_odom": [{"source_ns": 1_300_000_000}],
            "result": {},
            "learned_scorer": scorer,
            "dynamic_guard": guard,
            "journal": Mock(),
        }

    def test_real_mapping_projects_actual_worker_masks_into_frontend_keep(self):
        client, trigger = Mock(), Mock()
        masks = np.zeros((1, 2, 2), bool)
        masks[0, 0, 1] = True
        client.predict.return_value = SimpleNamespace(
            masks=masks,
            scores=np.array([0.9]),
            temporal_support=np.array([0]),
            metadata={"inference_index": 1, "fresh_inference": True},
        )
        trigger.decide.return_value = {"trigger": True, "reason": "test"}
        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                {"map_keyframe_interval_ns": 1_000_000_000},
                client=client,
                trigger=trigger,
            )
            packet = SimpleNamespace(
                points=np.array([[1.0, 0, 0], [1.0, 0.1, 0], [1.0, 0.2, 0]]),
                pixel_flat_indices=np.array([0, 1, 2]),
            )
            rgb = np.zeros((2, 2, 3), np.uint8)
            states, keep, receipt, _ = mapping.process(
                packet=packet, source_ns=1_000, rgb=rgb, depth=np.ones((2, 2))
            )
            self.assertIs(client.predict.call_args.args[0], rgb)
            self.assertEqual(client.predict.call_args.kwargs["source_ns"], 1_000)
            np.testing.assert_array_equal(states, [0, 2, 0])
            np.testing.assert_array_equal(keep, [True, False, True])
            self.assertTrue(receipt["fresh_inference"])
            self.assertFalse(receipt["background_certified_static"])
            self.assertEqual(mapping.matched, 0)
            self.assertIn(1_000, mapping.pending)

    def test_wrong_source_or_frame_cannot_project_active_quarantine(self):
        for invalid in (
            {"pose_source": "sim_oracle"},
            {"frame_id": "wrong_map"},
            {"child_frame_id": "wrong_body"},
        ):
            with (
                self.subTest(invalid=invalid),
                tempfile.TemporaryDirectory() as temporary,
            ):
                client, trigger, tracker = Mock(), Mock(), Mock()
                client.predict.return_value = SimpleNamespace(
                    masks=np.zeros((0, 2, 2), bool),
                    scores=np.zeros(0),
                    temporal_support=None,
                    metadata={"inference_index": 1, "fresh_inference": True},
                )
                trigger.decide.return_value = {"trigger": True, "reason": "test"}
                tracker.current_quarantine.return_value = (
                    {"bounds_map": [[0, -1, -1], [2, 1, 1]]},
                )
                mapping = LearnedMapping(
                    Path(temporary) / "map",
                    {"map_keyframe_interval_ns": 1_000},
                    client=client,
                    trigger=trigger,
                    tracker=tracker,
                )
                odom = {
                    "source_ns": 1_000,
                    "time": 0.000001,
                    "position": [100, 0, 0],
                    "orientation_wxyz": [1, 0, 0, 0],
                    "pose_source": "livo_post_vio",
                    "frame_id": "dynanav_lio_map",
                    "child_frame_id": "go2_imu",
                    **invalid,
                }
                mapping.offer_pose(odom)
                states, keep, receipt, _ = mapping.process(
                    packet=SimpleNamespace(
                        points=np.array([[1.0, 0, 0]]),
                        pixel_flat_indices=np.array([0]),
                    ),
                    source_ns=2_000,
                    rgb=np.zeros((2, 2, 3), np.uint8),
                    depth=np.ones((2, 2)),
                    latest_odom=odom,
                )
                self.assertFalse(mapping.pose_cache)
                self.assertFalse(receipt["quarantine_pose_valid"])
                self.assertFalse(keep.any())
                self.assertEqual(
                    receipt["frontend_withheld_reason"],
                    "active_person_quarantine_without_fresh_estimated_pose",
                )
                np.testing.assert_array_equal(states, 0)
                self.assertEqual(mapping.matched, 0)

    def test_callback_preserves_integer_pose_stamp_and_ignores_twist(self):
        class PoisonedTwist:
            @property
            def twist(self):
                raise AssertionError("unavailable twist was consumed")

        function = next(
            node
            for _, node in capture_nodes()
            if isinstance(node, ast.FunctionDef) and node.name == "odom_callback"
        )
        learner, scorer = Mock(), Mock()
        completed = {"source_ns": 7}
        learner.take_completed.return_value = [completed]
        env = {
            "experiment": {"slam_pose_source": "livo_post_vio"},
            "latest_odom": [None],
            "learned_mapper": learner,
            "learned_scorer": scorer,
            "time": time,
        }
        execute([function], env)
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=2_147_483_647, nanosec=999_999_997),
                frame_id="dynanav_lio_map",
            ),
            child_frame_id="go2_imu",
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                    orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
                )
            ),
            twist=PoisonedTwist(),
        )
        env["odom_callback"](message)
        row = env["latest_odom"][0]
        self.assertEqual(row["source_ns"], 2_147_483_647_999_999_997)
        self.assertIsInstance(row["source_ns"], int)
        self.assertIsNone(row["estimated_velocity"])
        learner.offer_pose.assert_called_once_with(row)
        scorer.observe_completion.assert_called_once_with(completed)

    def test_learned_input_does_not_require_native_or_renderer_pose(self):
        env = self.frame_environment(fresh=False)
        # Rb, sensor_position and instance_ann are deliberately absent. They are
        # necessary only when the independent evaluation sink is enabled.
        execute([learned_frame_block()], env)
        keywords = env["learned_mapper"].process.call_args.kwargs
        self.assertEqual(
            set(keywords), {"packet", "source_ns", "rgb", "depth", "latest_odom"}
        )
        self.assertEqual(keywords["source_ns"], 1_400_000_000)
        self.assertIs(keywords["packet"], env["packet"])
        self.assertIs(keywords["latest_odom"], env["latest_odom"][0])
        np.testing.assert_array_equal(env["frontend_keep"], [False, True, False])

    def test_oracle_sink_outputs_cannot_replace_model_states_or_frontend_keep(self):
        outputs = []
        for oracle_value in (0, 2):
            guard, scorer = Mock(), Mock()
            guard.update.return_value = (True, {})
            env = self.frame_environment(fresh=True, guard=guard, scorer=scorer)
            env.update(
                instance_ann=SimpleNamespace(get_data=lambda: {"oracle": True}),
                person_oracle_labels=lambda segmentation, pixels, value=oracle_value: (
                    np.full(len(pixels), value)
                ),
            )
            # No Rb/native sensor_position exists, even with scoring enabled.
            execute([learned_frame_block()], env)
            outputs.append((env["states"].copy(), env["frontend_keep"].copy()))
            scorer.observe.assert_called_once()
            np.testing.assert_array_equal(
                guard.update.call_args.args[0], env["xyz"][[0, 2]]
            )
            self.assertNotIn(
                "transform", env["learned_mapper"].process.call_args.kwargs
            )
        for left, right in zip(outputs[0], outputs[1], strict=True):
            np.testing.assert_array_equal(left, right)

    def test_hold_updates_only_for_fresh_successful_inference(self):
        for fresh in (False, True):
            with self.subTest(fresh=fresh):
                guard = Mock()
                guard.update.return_value = (True, {})
                env = self.frame_environment(fresh=fresh, guard=guard)
                execute([learned_frame_block()], env)
                self.assertEqual(guard.update.call_count, int(fresh))
                if fresh:
                    self.assertEqual(
                        guard.update.call_args.kwargs["mask_source_ns"],
                        1_400_000_000,
                    )

    def test_unscheduled_frame_does_not_refresh_clearance_evidence(self):
        guard = DynamicHold(imu_origin_in_base_m=[0, 0, 0])
        for stamp in (1_000_000_000, 1_200_000_000):
            guard.update(
                np.empty((0, 3)),
                source_ns=stamp,
                mask_source_ns=stamp,
                now_ns=stamp,
                mask_valid=True,
                sampled_point_count=3,
            )
        env = self.frame_environment(fresh=False, guard=guard)
        execute([learned_frame_block()], env)
        control = next(
            node
            for _, node in capture_nodes()
            if isinstance(node, ast.If)
            and calls_method(node, "dynamic_guard", "decision")
            and ast.unparse(node.test) == "dynamic_guard is not None"
        )
        env["state"] = [1.6]
        execute([control], env)
        self.assertFalse(env["held"])
        self.assertEqual(env["hold_receipt"]["source_ns"], 1_200_000_000)
        self.assertEqual(env["hold_receipt"]["clear_frames"], 2)
        env["state"] = [1.700000001]
        execute([control], env)
        self.assertTrue(env["held"])
        self.assertEqual(env["hold_receipt"]["reason"], "stale_observation")

    def test_published_candidate_buffer_uses_frontend_keep(self):
        assignment = next(
            node
            for _, node in capture_nodes()
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "filtered_xyzi"
                for target in node.targets
            )
        )
        env = {
            "xyzi": np.arange(12).reshape(3, 4),
            "frontend_keep": np.array([False, True, False]),
        }
        execute([assignment], env)
        np.testing.assert_array_equal(env["filtered_xyzi"], env["xyzi"][[1]])


class LearnedCloudLauncherWiringTests(unittest.TestCase):
    def test_explicit_candidate_input_is_remapped_and_receipted(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            data = base / "data" / "embodied-navigation"
            prefix = data / "build_cache" / "fake"
            lib = prefix / "install/ros/lib"
            (lib / "fast_livo2_ros").mkdir(parents=True)
            (lib / "libfast_livo2_core.so").write_bytes(b"common.allow_lio_only")
            runner = lib / "fast_livo2_ros/fastlivo_mapping"
            runner.write_text(
                "#!/usr/bin/python3\nimport json,os,pathlib,sys\n"
                "(pathlib.Path(os.environ['DEMO_RUN_DIR'])/'argv.json').write_text(json.dumps(sys.argv[1:]))\n"
            )
            runner.chmod(0o755)
            tools = base / "tools"
            tools.mkdir()
            (tools / "ldd").write_text("#!/bin/sh\nexit 0\n")
            (tools / "ldd").chmod(0o755)
            output = data / "experiments" / "candidate-wiring"
            run = subprocess.run(
                [
                    "bash",
                    str(ROOT / "scripts/run_dynanav_lio.sh"),
                    "--binary-prefix",
                    str(prefix),
                    "--output-dir",
                    str(output),
                    "--observer-namespace",
                    "/dynanav/learned_test",
                    "--lidar-topic",
                    "/dynanav/lidar_perception_filtered",
                ],
                env={
                    **os.environ,
                    "PROJECT_DATA_ROOT": str(base / "data"),
                    "PATH": str(tools) + os.pathsep + os.environ["PATH"],
                    "ROS_DOMAIN_ID": "191",
                },
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(run.returncode, 0, run.stderr)
            argv = json.loads((output / "argv.json").read_text())
            receipt = json.loads((output / "launch.json").read_text())
            self.assertIn(
                "/dynanav/lidar_static:=/dynanav/lidar_perception_filtered", argv
            )
            self.assertEqual(
                receipt["lidar_input_topic"], "/dynanav/lidar_perception_filtered"
            )
            self.assertEqual(
                receipt["sensor_inputs"],
                ["/dynanav/lidar_perception_filtered", "/dynanav/imu"],
            )
            self.assertIn("not certified static", receipt["lidar_input_scope"])
            self.assertFalse(receipt["ground_truth_input"])
            self.assertEqual(receipt["command"][1:], argv)


if __name__ == "__main__":
    unittest.main()
