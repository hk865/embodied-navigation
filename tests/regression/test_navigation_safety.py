"""CPU-only safety contracts: clear guard, parking pose/zero gates, geometry checks."""

from pathlib import Path
import copy
import math
import sys
import unittest

from embodied_navigation.control.safety.clear_guard_window import ClearGuardWindow
from embodied_navigation.control.parking.parking_pose10hz import PoseGrid
from embodied_navigation.control.parking.parking_zero_contract import (
    make_event,
    validate_event,
    authorize_frame,
    before_actor_observation,
)
from embodied_navigation.control.parking.terminal_heading_contract import (
    attach_reference_heading,
    validate_reference_heading,
)
from embodied_navigation.common.config import read, ROOT, loads
from embodied_navigation.planning.admission.static_geometry import (
    certify_physical_curve,
    certify_start_connector,
)


class NavigationSafety(unittest.TestCase):
    def test_real_clear_samples_and_gap(self):
        w = ClearGuardWindow()
        for n in range(9):
            self.assertFalse(w.observe(n * 140_000_000, False, True))
        self.assertTrue(w.observe(1_260_000_000, False, True))
        self.assertFalse(w.observe(1_700_000_000, False, True))
        self.assertEqual(len(w.clocks), 1)

    def test_cached_clear_does_not_count(self):
        w = ClearGuardWindow()
        for n in range(20):
            self.assertFalse(w.observe(n * 100_000_000, False, False))
        self.assertEqual(len(w.clocks), 0)

    def row(self, t, x=0.0, speed=99.0):
        return dict(
            stamp_ns=round(t * 1e9),
            position=[x, 0.0, 0.0],
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            body_velocity=[speed, 0.0, 0.0],
            source_topic="/demo/slam/body_odom",
            navigation_ground_truth_used=False,
            pose_producer="fast_livo2_async_backend",
        )

    def test_existing_pose_rule_does_not_add_velocity_veto(self):
        g = PoseGrid(0, self.row(0))
        for n in range(1, 51):
            g.observe(self.row(n * 0.1))
        self.assertTrue(g.evidence()["passed"])

    def test_parking_excursion_cannot_hide_behind_returned_endpoint(self):
        g = PoseGrid(0, self.row(0))
        for n in range(1, 51):
            g.observe(self.row(n * 0.1, 0.06 if n == 20 else 0.0))
        self.assertFalse(g.evidence()["passed"])

    def test_missing_parking_source_cannot_be_filled(self):
        g = PoseGrid(0, self.row(0))
        for n in range(4, 51):
            g.observe(self.row(n * 0.1))
        self.assertFalse(g.evidence()["data_available"])

    def event(self):
        event = make_event(
            "run",
            "run:request",
            "diagnostic_goal",
            "diagnostic_path",
            1,
            900_000_000,
            1_000_000_000,
            1,
        )
        lineage = dict(
            request_id="run:request",
            goal_sha256="unrelated-diagnostic",
            path_sha256=None,
            navigation_ground_truth_used=False,
            command_after_slew=[0.0, 0.0, 0.0],
            ordinary_zero_event_id=event["event_id"],
            ordinary_zero_event_sequence=1,
            source_pose_stamp_ns=1_000_000_000,
            paired_imu_stamp_ns=990_000_000,
            control_stamp_ns=1_000_000_000,
            cascade_sequence=1,
            ordinary_parking_stage="handoff_zero",
        )
        return event, lineage

    def test_zero_control_needs_no_geometry_or_file_hash(self):
        event, lineage = self.event()
        validate_event(
            event, lineage, "run", [0.0, 0.0, 0.0], 1_020_000_000, source_max_age_ns=300_000_000
        )
        frame = dict(frame_serial=1, input_clock_ns=1_020_000_000, producer_sequence=1)
        authorization = authorize_frame(
            event, lineage, "run", [0.0, 0.0, 0.0], frame, source_max_age_ns=300_000_000
        )
        cmd = [0.01, 0.0, 0.0]
        before_actor_observation(cmd, [0.0, 0.0, 0.0], authorization, 1_020_000_000, frame=frame)
        self.assertEqual(cmd, [0.0, 0.0, 0.0])

    def test_stale_zero_source_still_rejected(self):
        e, l = self.event()
        with self.assertRaises(ValueError):
            validate_event(
                e, l, "run", [0.0, 0.0, 0.0], 1_400_000_000, source_max_age_ns=300_000_000
            )

    def test_resumed_command_cannot_receive_zero_takeover(self):
        e, l = self.event()
        f = dict(frame_serial=1, input_clock_ns=1_020_000_000, producer_sequence=1)
        a = authorize_frame(e, l, "run", [0.0, 0.0, 0.0], f, source_max_age_ns=300_000_000)
        with self.assertRaises(ValueError):
            before_actor_observation([0.0, 0.0, 0.0], [0.1, 0.0, 0.0], a, 1_020_000_000, frame=f)

    @unittest.skipUnless(
        (ROOT / "assets/geometry/metadata.json").is_file(),
        "measured private map supplied separately",
    )
    def test_actual_physical_curve_and_outside_map(self):
        for y, expected in [(0.0, True), (1000.0, False)]:
            p = dict(
                frame_id="camera_init",
                order=3,
                pos_pts=[[x, y, 0.02] for x in [0.0, 0.033, 0.067, 0.1]],
                knots=[0, 0, 0, 0, 1, 1, 1, 1],
            )
            self.assertEqual(certify_physical_curve(p)["allowed"], expected)

    @unittest.skipUnless(
        (ROOT / "assets/geometry/route.json").is_file(),
        "runtime geometry route supplied separately",
    )
    def test_heading_uses_current_geometry(self):
        p = read()
        doc = loads((ROOT / "assets/geometry/route.json").read_text())
        doc = attach_reference_heading(p, doc)
        validate_reference_heading(p, doc)
        doc["task_terminal_heading_rad"] += 0.1
        with self.assertRaises(ValueError):
            validate_reference_heading(p, doc)


if __name__ == "__main__":
    unittest.main()
