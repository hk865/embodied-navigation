"""CPU-only file-version consumption tests; no control/ROS/GPU is started."""

from contextlib import contextmanager
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from embodied_navigation.simulation.dynanav import exploration_waypoints as waypoints


class PerceptionFileConsumptionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.tracker = waypoints.InteractiveWaypointTracker({
            "control_mode": "interactive_waypoint_exploration",
            "slam_pose_source": "lio_estimate",
            "navigation_pose_source": "measured_lio",
            "interactive_mailbox": self.temporary.name,
            "speed_mps": 0.5,
            "motion_end_s": 300.0,
        })
        self.path = self.tracker.perception_path
        self.now = 1000.0
        self.sim = 10.0
        self.payload = {
            "version": 1, "source": "live_rgbd_obstacle_cloud",
            "frame_id": "go2_base", "stamp_s": self.sim,
            "arrival_wall_s": self.now, "clearance_m": 2.0,
            "corridor_points": 120, "cloud_points": 20000,
            "observation_state": "measured", "depth_valid_ratio": 1.0,
            "directions": {"front": {"observation_state": "measured",
                                      "clearance_m": 2.0}},
        }

    def publish(self, **fields):
        payload = copy.deepcopy(self.payload)
        payload.update(fields)
        waypoints.atomic_write_json(self.path, payload)
        return payload

    def read(self, advance=0.0):
        return self.tracker._read_perception_file(self.now + advance)

    def gate(self, advance=0.0, sim_advance=0.0):
        return self.tracker._perception_gate(self.now + advance, self.sim + sim_advance)

    def prime_api(self):
        self.tracker.update_perception(
            now_wall=self.now, stamp_s=self.sim, clearance_m=2.0,
            source="static_depth", frame_id="go2_base", points=4096,
            directions=self.payload["directions"],
        )

    def assert_unavailable(self):
        self.assertEqual("perception_unavailable", self.gate(0.02)[0])
        self.assertEqual("perception_directions_unavailable", self.tracker._direction_reason(
            {"body_forward_mps": 0.1}, self.now + 0.02))
        self.assertIsNone(self.tracker._perception_file_cache)

    def test_atomic_new_version_is_consumed_inside_old_50ms_window(self):
        self.publish(arrival_wall_s=self.now - 0.49)
        self.assertIsNone(self.gate())
        self.publish(stamp_s=self.sim + 0.02, arrival_wall_s=self.now + 0.01)
        self.assertIsNone(self.gate(0.02, 0.02))
        self.assertEqual(self.sim + 0.02, self.read(0.02)["stamp_s"])
        self.assertEqual(self.now + 0.01, self.read(0.02)["arrival_wall_s"])

    def test_unchanged_file_parsed_once_and_original_clocks_still_expire(self):
        self.publish()
        original = copy.deepcopy(self.payload)
        with mock.patch.object(waypoints.json, "loads", wraps=json.loads) as loads:
            self.assertIsNone(self.gate())
            first = copy.deepcopy(self.read())
            for tick in range(1, 51):
                self.assertEqual(first, self.read(tick * 0.02))
            gate = self.gate(0.51)
            self.assertEqual("perception_stale", gate[0])
            self.assertAlmostEqual(0.51, gate[1]["perception_wall_age_s"])
            self.assertEqual(1, loads.call_count)
        self.assertEqual(original, json.loads(self.path.read_bytes()))
        self.assertEqual(self.now, self.read()["arrival_wall_s"])
        self.assertEqual(self.sim, self.read()["stamp_s"])

    def test_same_mtime_and_size_new_inode_is_not_cached(self):
        self.publish()
        self.read()
        old = self.path.stat()
        self.publish(clearance_m=0.2)
        os.utime(self.path, ns=(old.st_atime_ns, old.st_mtime_ns))
        new = self.path.stat()
        self.assertNotEqual(old.st_ino, new.st_ino)
        self.assertEqual(old.st_size, new.st_size)
        self.assertEqual(old.st_mtime_ns, new.st_mtime_ns)
        self.assertEqual("obstacle_too_close", self.gate(0.02)[0])

    def test_same_inode_and_restored_mtime_still_detects_ctime_change(self):
        self.publish()
        self.read()
        old = self.path.stat()
        data = self.path.read_text().replace('"clearance_m":2.0', '"clearance_m":0.2')
        self.path.write_text(data)
        os.utime(self.path, ns=(old.st_atime_ns, old.st_mtime_ns))
        new = self.path.stat()
        self.assertEqual(old.st_ino, new.st_ino)
        self.assertEqual(old.st_size, new.st_size)
        self.assertEqual(old.st_mtime_ns, new.st_mtime_ns)
        self.assertNotEqual(old.st_ctime_ns, new.st_ctime_ns)
        self.assertEqual("obstacle_too_close", self.gate(0.02)[0])

    def test_new_unknown_immediately_overrides_cached_and_api_clearance(self):
        self.prime_api()
        self.publish()
        self.assertIsNone(self.gate())
        self.publish(observation_state="insufficient_observation", clearance_m=None,
                     directions={"front": {"observation_state": "insufficient_observation",
                                            "clearance_m": None}})
        self.assertEqual("perception_clearance_unknown", self.gate(0.02)[0])
        self.assertEqual("perception_direction_uncovered:front", self.tracker._direction_reason(
            {"body_forward_mps": 0.1}, self.now + 0.02))

    def test_invalid_json_clears_old_samples_and_is_not_reparsed(self):
        self.prime_api()
        self.publish()
        self.read()
        self.path.write_bytes(b'{"stamp_s":')
        with mock.patch.object(waypoints.json, "loads", wraps=json.loads) as loads:
            self.assert_unavailable()
            self.assert_unavailable()
            self.assertEqual(1, loads.call_count)
        self.publish(clearance_m=0.2)
        self.assertEqual("obstacle_too_close", self.gate(0.03)[0])

    def test_invalid_utf8_deep_json_and_schema_fail_closed(self):
        self.prime_api()
        self.publish()
        self.read()
        for raw in (b'\xff', b'[' * 2000 + b']' * 2000, b'9' * 5000, b'[]',
                    json.dumps(dict(self.payload, arrival_wall_s=float("nan"))).encode(),
                    json.dumps(dict(self.payload, corridor_points=float("inf"))).encode()):
            with self.subTest(raw=raw[:25]):
                self.path.write_bytes(raw)
                self.assert_unavailable()

    def test_deletion_does_not_revive_api_or_file_cache(self):
        self.prime_api()
        self.publish()
        self.read()
        self.path.unlink()
        self.assert_unavailable()
        self.assert_unavailable()
        self.publish()
        self.assertIsNone(self.gate(0.03))

    def test_api_is_preserved_only_when_no_file_was_observed(self):
        self.prime_api()
        self.assertIsNone(self.gate())
        self.assertIsNone(self.tracker._direction_reason({"body_forward_mps": 0.1}, self.now))
        self.path.write_bytes(b'broken')
        self.assert_unavailable()
        self.path.unlink()
        self.assert_unavailable()

    def test_unreadable_even_unchanged_file_holds_then_retries(self):
        self.prime_api()
        self.publish()
        self.read()
        with mock.patch.object(waypoints.os, "open", side_effect=PermissionError("unreadable")):
            self.assert_unavailable()
        self.assertIsNone(self.tracker._perception_file_version)
        self.assertIsNone(self.gate(0.03))

    def test_directory_and_fifo_are_rejected_without_reading(self):
        self.prime_api()
        self.path.mkdir()
        self.assert_unavailable()
        self.path.rmdir()
        os.mkfifo(self.path)
        self.assert_unavailable()

    def test_exact_2mib_limit_accepted_but_larger_file_not_parsed(self):
        self.prime_api()
        raw = json.dumps(self.payload).encode()
        self.path.write_bytes(raw + b' ' * (waypoints.MAX_PERCEPTION_FILE_BYTES - len(raw)))
        self.assertIsNone(self.gate())
        with self.path.open("ab") as stream:
            stream.write(b' ')
        with mock.patch.object(waypoints.json, "loads", wraps=json.loads) as loads:
            self.assert_unavailable()
            self.assertEqual(0, loads.call_count)
        self.assertIsNone(self.tracker._perception_file_version)

    def test_growth_during_read_is_bounded_and_not_cached(self):
        self.publish()
        real_fdopen = os.fdopen
        read_sizes = []

        @contextmanager
        def growing_stream(*args):
            with real_fdopen(*args) as stream:
                proxy = mock.Mock(wraps=stream)
                def read(size):
                    read_sizes.append(size)
                    with self.path.open("ab") as writer:
                        writer.write(b' ' * (waypoints.MAX_PERCEPTION_FILE_BYTES * 2))
                    return stream.read(size)
                proxy.read.side_effect = read
                yield proxy

        with mock.patch.object(waypoints.os, "fdopen", side_effect=growing_stream):
            self.assertIsNone(self.read())
        self.assertEqual([waypoints.MAX_PERCEPTION_FILE_BYTES + 1], read_sizes)
        self.assertIsNone(self.tracker._perception_file_version)

    def test_replace_after_open_cannot_pair_old_bytes_with_new_fingerprint(self):
        self.publish()
        real_fstat = os.fstat
        calls = []

        def replace_after_open(fd):
            info = real_fstat(fd)
            calls.append(fd)
            if len(calls) == 1:
                self.publish(clearance_m=0.2)
            return info

        with mock.patch.object(waypoints.os, "fstat", side_effect=replace_after_open):
            self.assertIsNone(self.read())
        self.assertEqual(2, len(calls))
        self.assertIsNone(self.tracker._perception_file_version)
        self.assertEqual("obstacle_too_close", self.gate(0.02)[0])

    def test_replace_after_read_fails_closed_without_retry_and_next_call_recovers(self):
        self.prime_api()
        self.publish()
        real_fstat = os.fstat
        calls = []

        def replace_after_read(fd):
            info = real_fstat(fd)
            calls.append(fd)
            if len(calls) == 2:
                self.publish(observation_state="insufficient_observation", clearance_m=None)
            return info

        with mock.patch.object(waypoints.os, "fstat", side_effect=replace_after_read):
            self.assertEqual("perception_unavailable", self.gate()[0])
        self.assertEqual(2, len(calls))
        self.assertIsNone(self.tracker._perception_file_version)
        self.assertEqual("perception_clearance_unknown", self.gate(0.02)[0])

    def test_replace_during_cache_lookup_fails_closed(self):
        self.publish()
        self.read()
        real_stat = Path.stat

        def replace_at_stat(path, *args, **kwargs):
            if path == self.path:
                self.publish(clearance_m=0.2)
            return real_stat(path, *args, **kwargs)

        with mock.patch.object(Path, "stat", new=replace_at_stat):
            self.assertIsNone(self.read(0.02))
        self.assertIsNone(self.tracker._perception_file_version)
        self.assertEqual("obstacle_too_close", self.gate(0.04)[0])

    def test_same_inode_change_during_read_is_rejected(self):
        self.publish()
        real_fstat = os.fstat
        calls = []

        def change_after_read(fd):
            calls.append(fd)
            if len(calls) == 2:
                self.path.write_text(json.dumps(dict(self.payload, clearance_m=0.2)))
            return real_fstat(fd)

        with mock.patch.object(waypoints.os, "fstat", side_effect=change_after_read):
            self.assertIsNone(self.read())
        self.assertIsNone(self.tracker._perception_file_version)
        self.assertEqual("obstacle_too_close", self.gate(0.02)[0])

    def test_read_has_no_controller_or_clock_or_request_side_effect(self):
        self.publish()
        config = copy.deepcopy(self.tracker.config)
        with mock.patch.object(self.tracker.controller, "tick") as tick:
            for i in range(5):
                self.read(i * 0.02)
            tick.assert_not_called()
        self.assertEqual(config, self.tracker.config)
        self.assertEqual(0, self.tracker._tick_counter)
        self.assertIsNone(self.tracker._action)
        self.assertEqual(self.now, self.read()["arrival_wall_s"])
        self.assertEqual(self.sim, self.read()["stamp_s"])


class PerceptionTickVersionBindingTests(unittest.TestCase):
    """Whole normal ticks cannot combine A's admission with B's directions."""

    MODES = ("maneuver_fresh", "maneuver_cached", "goto_fresh", "goto_cached")
    FAULTS = (
        "unknown_clear_directions", "invalid_after_compute_clear_directions",
        "invalid_fusion_contract_clear_directions", "wrong_frame", "forbidden_source",
        "future_arrival", "actual_invalidator", "unknown_directions", "bad_json",
        "deleted", "valid_identical_clocks",
    )

    def make_case(self, mode, transport):
        # Reuse the existing real-tick fixture, including link/pose/TTL admission.
        from test_dynanav_interactive_waypoints import MailboxCase
        case = MailboxCase()
        case.setUp()
        self.addCleanup(case.tearDown)
        tracker = case.tracker
        payload = {
            "version": 1, "source": "live_rgbd_obstacle_cloud", "frame_id": "go2_base",
            "stamp_s": case.sim, "arrival_wall_s": case.now, "clearance_m": 2.0,
            "corridor_points": 120, "cloud_points": 20000,
            "observation_state": "measured", "depth_valid_ratio": 1.0,
            "directions": {name: {
                "observation_state": "measured", "clearance_m": 2.0, "points": 100,
                "floor_observation_state": "supported", "floor_support_complete": True,
                "radial_bin_floor_points": [10, 10, 10, 10],
                "near_field_floor_unobserved_m": [0.0, 0.35],
            } for name in ("front", "rear", "left", "right")},
        }

        def publish(value):
            if transport == "file":
                waypoints.atomic_write_json(tracker.perception_path, value)
            else:
                return tracker.update_perception(
                    now_wall=value["arrival_wall_s"], stamp_s=value["stamp_s"],
                    clearance_m=value["clearance_m"], source=value["source"],
                    frame_id=value["frame_id"], points=value["corridor_points"],
                    observation_state=value["observation_state"],
                    depth_valid_ratio=value["depth_valid_ratio"], directions=value["directions"],
                )

        publish(payload)
        if mode.startswith("maneuver"):
            case.write_request(seq=1, action="maneuver", body_forward_mps=0.1, duration_s=2.0)
        else:
            case.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        case.step(perception=None)
        if mode == "goto_cached":
            case.step(perception=None, position=[0.0, 0.1, 0.0])
            self.assertLess(tracker._last_command[1], 0.0)
        payload.update(stamp_s=case.sim, arrival_wall_s=case.now)
        return case, tracker, payload, publish

    def check_transition(self, mode, transport, fault):
        from embodied_navigation.simulation.dynanav.depth_safety_fusion import (
            FUSION_SOURCE, invalidate_fused_perception,
        )
        case, tracker, payload, publish = self.make_case(mode, transport)
        required = "front" if mode.startswith("maneuver") else "right"
        before = copy.deepcopy(payload)
        before["directions"][required].update(
            observation_state="insufficient_observation", clearance_m=None)
        after = copy.deepcopy(payload)
        if fault == "unknown_clear_directions":
            after.update(observation_state="insufficient_observation", clearance_m=None)
        elif fault == "invalid_after_compute_clear_directions":
            after.update(observation_state="source_invalid_after_compute", clearance_m=None,
                         fail_closed=True)
        elif fault == "invalid_fusion_contract_clear_directions":
            after.update(source=FUSION_SOURCE, safety_fusion={})
        elif fault == "wrong_frame":
            after["frame_id"] = "wrong_base"
        elif fault == "forbidden_source":
            after["source"] = "native_truth"
        elif fault == "future_arrival":
            after["arrival_wall_s"] += 10.0
        elif fault == "actual_invalidator":
            after = invalidate_fused_perception(after, "test")
        elif fault == "unknown_directions":
            after.update(observation_state="insufficient_observation", clearance_m=None)
            after["directions"][required].update(
                observation_state="insufficient_observation", clearance_m=None)
        publish(before)
        parameters = ({"body_forward_mps": 0.1} if mode.startswith("maneuver")
                      else {"body_lateral_mps": -0.05})
        self.assertIsNone(tracker._perception_gate(case.now, case.sim))
        self.assertIsNotNone(tracker._direction_reason(parameters, case.now))
        real_gate = tracker._perception_gate
        action = copy.deepcopy(tracker._action)
        config = copy.deepcopy(tracker.config)
        gate_calls = []

        def gate_then_replace(now, sim):
            result = real_gate(now, sim)
            self.assertIsNone(result)
            gate_calls.append((now, sim))
            if fault == "bad_json":
                tracker.perception_path.write_bytes(b"broken")
            elif fault == "deleted":
                tracker.perception_path.unlink()
            else:
                publish(after)
            return result

        stamp = case.sim if mode.endswith("cached") else None
        with mock.patch.object(tracker, "_perception_gate", side_effect=gate_then_replace), \
             mock.patch.object(tracker.controller, "tick", wraps=tracker.controller.tick) as tick:
            command, receipt = case.step(
                perception=None, stamp=stamp,
                position=[0.0, 0.1, 0.0] if mode.startswith("goto") else None,
            )
            self.assertEqual(1 if mode == "goto_fresh" else 0, tick.call_count)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual(1, len(gate_calls))
        self.assertEqual(config, tracker.config)
        if mode.startswith("maneuver"):
            self.assertEqual(action, tracker._action)  # Includes the original TTL.
        else:
            self.assertEqual("goto_lateral_uncovered", receipt["reason"])
            self.assertEqual("perception_changed_during_tick",
                             receipt["lateral_direction_reason"])
        if fault == "valid_identical_clocks":
            # B is genuinely new, despite identical source clocks. The hold lasts
            # only until B passes every original gate in the next normal tick.
            command, _ = case.step(
                perception=None,
                position=[0.0, 0.1, 0.0] if mode.startswith("goto") else None,
            )
            self.assertTrue(any(command))
        else:
            command, _ = case.step(perception=None)
            self.assertEqual([0.0, 0.0, 0.0], command)
        if transport == "api":
            self.assertFalse(tracker.perception_path.exists())

    def test_fresh_maneuver_file_versions(self):
        for fault in self.FAULTS:
            with self.subTest(fault=fault):
                self.check_transition("maneuver_fresh", "file", fault)

    def test_cached_maneuver_file_versions(self):
        for fault in self.FAULTS:
            with self.subTest(fault=fault):
                self.check_transition("maneuver_cached", "file", fault)

    def test_fresh_goto_file_versions(self):
        for fault in self.FAULTS:
            with self.subTest(fault=fault):
                self.check_transition("goto_fresh", "file", fault)

    def test_cached_goto_file_versions(self):
        for fault in self.FAULTS:
            with self.subTest(fault=fault):
                self.check_transition("goto_cached", "file", fault)

    def test_in_process_only_new_sample_versions(self):
        for mode in self.MODES:
            for fault in self.FAULTS:
                if fault in ("bad_json", "deleted"):
                    continue
                with self.subTest(mode=mode, fault=fault):
                    self.check_transition(mode, "api", fault)

    def test_unchanged_file_or_api_sample_keeps_original_commands(self):
        for mode in self.MODES:
            for transport in ("file", "api"):
                with self.subTest(mode=mode, transport=transport):
                    case, tracker, _, _ = self.make_case(mode, transport)
                    command, _ = case.step(
                        perception=None, stamp=case.sim if mode.endswith("cached") else None,
                        position=[0.0, 0.1, 0.0] if mode.startswith("goto") else None,
                    )
                    self.assertTrue(any(command))

    def test_request_precheck_does_not_inherit_previous_tick_admission(self):
        for transport in ("file", "api"):
            with self.subTest(transport=transport):
                case, tracker, payload, publish = self.make_case("goto_fresh", transport)
                publish(payload)
                case.write_request(seq=2, action="maneuver", body_forward_mps=0.1, duration_s=2.0)
                command, _ = case.step(perception=None, advance=0.12)
                self.assertEqual([0.1, 0.0, 0.0], command)
                self.assertEqual("maneuver", tracker._action["kind"])


    def check_api_alias(self, alias):
        for mode in self.MODES:
            with self.subTest(mode=mode, alias=alias):
                case, tracker, payload, publish = self.make_case(mode, "api")
                payload["source"] = "rtx_lidar_cloud_registered_full"
                returned = publish(payload)
                original = copy.deepcopy(tracker._perception)
                real_gate = tracker._perception_gate

                def gate_then_mutate_external_alias(now, sim):
                    result = real_gate(now, sim)
                    self.assertIsNone(result)
                    if alias == "returned_dict":
                        returned["directions"]["front"]["floor_support_complete"] = False
                    else:
                        payload["directions"]["front"]["radial_bin_floor_points"][0] = 0
                    self.assertEqual(original, tracker._perception)
                    return result

                with mock.patch.object(tracker, "_perception_gate",
                                       side_effect=gate_then_mutate_external_alias):
                    command, _ = case.step(
                        perception=None, stamp=case.sim if mode.endswith("cached") else None,
                        position=[0.0, 0.1, 0.0] if mode.startswith("goto") else None,
                    )
                self.assertTrue(any(command))
                self.assertEqual(original, tracker._perception)
                self.assertIsNone(real_gate(case.now, case.sim))
                command, _ = case.step(
                    perception=None,
                    position=[0.0, 0.1, 0.0] if mode.startswith("goto") else None,
                )
                self.assertTrue(any(command))

    def test_api_returned_nested_dict_cannot_mutate_admitted_sample(self):
        self.check_api_alias("returned_dict")

    def test_api_input_nested_list_cannot_mutate_admitted_sample(self):
        self.check_api_alias("input_list")


if __name__ == "__main__":
    unittest.main()
