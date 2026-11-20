"""Actual worker callbacks must preserve owned-child and sample ordering."""
import copy
import json
import tempfile
import unittest

from embodied_navigation.simulation.dynanav import pct_scan_worker as worker
from scan_worker_acceptance_fixtures import WorkerRig, String


class ForbiddenPctRoute:
    def __deepcopy__(self, memo):
        raise AssertionError("Yaw must not request a PCT route")


class YawWorkerBridgeAcceptance(unittest.TestCase):
    def test_native_reply_identity_and_monotonic_sample_matcher(self):
        request = dict(version=1, run_id="run", seq=7, query_id=9,
                       target_yaw_rad=.3, sample_id=4)
        valid = dict(request, status="clear", occupancy=0,
                     native_identity=dict(pid=123, startup_nonce="owned"))
        self.assertTrue(worker.rotation_response_matches(valid, request, 123, "owned", 3))
        for change in (dict(run_id="other"), dict(seq=6), dict(query_id=8),
                       dict(target_yaw_rad=-.3), dict(sample_id=3),
                       dict(native_identity=dict(pid=124, startup_nonce="owned")),
                       dict(native_identity=dict(pid=123, startup_nonce="old_child"))):
            with self.subTest(change=change):
                self.assertFalse(worker.rotation_response_matches(dict(valid, **change),
                                 request, 123, "owned", 3))
        for status in ("pending", "blocked", "failed"):
            self.assertTrue(worker.rotation_response_matches(dict(valid, status=status),
                            request, 123, "owned", 3))

    def test_yaw_uses_native_query_not_path_and_stale_clear_cannot_override_pending(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = WorkerRig(directory)
            rig.route = ForbiddenPctRoute()
            rig.adapter._handle_request(dict(version=1, run_id="generation", seq=2,
                action="yaw", pose_source="livo_post_vio", delta_yaw_rad=.3,
                created_wall_s=10., ttl_s=30.), now_wall=10.)
            rig.tick_adapter(10.01)

            def scenario(active):
                active.prepare()
                self.assertEqual(active.path_messages(), [])
                published = active.node.publisher("rotation_query").messages
                self.assertTrue(published)
                def latest_request():
                    return json.loads(active.node.publisher("rotation_query").messages[-1].data)
                def response(request, status="clear"):
                    result = dict(request, status=status, occupancy=0 if status=="clear" else -1,
                        reason="acceptance_"+status, yaw_rad=0., position_xyz_m=[0., 0., .4],
                        source=dict(body_stamp_ns=request["body_stamp_ns"],
                            body_frame_id="camera_init", body_child_frame_id="body",
                            cloud_stamp_ns=request["body_stamp_ns"], cloud_frame_id="camera_init"),
                        native_identity=dict(pid=active.processes[-1].pid,
                                             startup_nonce=request["startup_nonce"]))
                    return result
                def deliver(payload):
                    active.node.receive("rotation_result", String(json.dumps(payload)))
                def output():
                    return json.loads((active.mailbox / "planner_yaw_space.json").read_text())
                current = latest_request()
                good = response(current)
                before = output()
                for identity in (dict(pid=active.processes[-1].pid+1, startup_nonce=current["startup_nonce"]),
                                 dict(pid=active.processes[-1].pid, startup_nonce="other-child")):
                    deliver(dict(good, native_identity=identity))
                    self.assertEqual(output(), before)
                deliver(good)
                self.assertEqual(output()["status"], "clear")
                active.poll(.05)
                newer = latest_request()
                self.assertGreater(newer["sample_id"], current["sample_id"])
                deliver(response(newer, "pending"))
                pending = output()
                self.assertEqual(pending["status"], "pending")
                deliver(good)
                self.assertEqual(output(), pending)
                active.poll(.05)
                latest = latest_request()
                deliver(response(latest, "blocked"))
                blocked = output()
                self.assertEqual(blocked["status"], "blocked")
                deliver(good)
                self.assertEqual(output(), blocked)
                self.assertEqual(active.path_messages(), [])

            rig.run_scenario(scenario)
            self.assertTrue(rig.node.destroyed)
            self.assertTrue(all(child.terminated for child in rig.processes))

    def test_fresh_odometry_emits_rotation_query_without_waiting_for_poll_timer(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = WorkerRig(directory)
            rig.route = ForbiddenPctRoute()
            rig.adapter._handle_request(dict(version=1, run_id="generation", seq=2,
                action="yaw", pose_source="livo_post_vio", delta_yaw_rad=.3,
                created_wall_s=10., ttl_s=30.), now_wall=10.)
            rig.tick_adapter(10.01)

            def scenario(active):
                active.prepare()
                queries = active.node.publisher("rotation_query").messages
                before_count = len(queries)
                before_payload = json.loads(queries[-1].data)
                # Fresh VIO arrival must publish a new rotation_query immediately
                # without waiting for the 50 ms poll timer.
                active.observe(10.7, (.0004, 0., .4))
                self.assertGreater(len(queries), before_count)
                after_payload = json.loads(queries[-1].data)
                self.assertGreater(after_payload["sample_id"], before_payload["sample_id"])
                self.assertEqual(after_payload["body_stamp_ns"], 10_700_000_000)

            rig.run_scenario(scenario)

    def test_yaw_minimum_cloud_stamp_strict_before_first_clear_and_fails_closed_after(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = WorkerRig(directory)
            rig.route = ForbiddenPctRoute()
            rig.adapter._handle_request(dict(version=1, run_id="generation", seq=2,
                action="yaw", pose_source="livo_post_vio", delta_yaw_rad=.3,
                created_wall_s=10., ttl_s=30.), now_wall=10.)
            rig.tick_adapter(10.01)

            def scenario(active):
                active.prepare()
                queries = active.node.publisher("rotation_query").messages
                first = json.loads(queries[-1].data)
                # Before first clear reply, minimum_cloud_stamp_ns must equal the current cloud stamp.
                self.assertEqual(first["minimum_cloud_stamp_ns"], 10_600_000_000)
                clear_reply = dict(
                    first, status="clear", occupancy=0, reason="clear",
                    yaw_rad=0., position_xyz_m=[0., 0., .4],
                    source=dict(body_stamp_ns=first["body_stamp_ns"],
                        body_frame_id="camera_init", body_child_frame_id="body",
                        cloud_stamp_ns=first["minimum_cloud_stamp_ns"], cloud_frame_id="camera_init"),
                    native_identity=dict(pid=active.processes[-1].pid,
                                         startup_nonce=first["startup_nonce"]),
                )
                active.node.receive("rotation_result", String(json.dumps(clear_reply)))
                # After first clear, a 0.1 s newer pose+cloud allows 0.2 s pipeline overlap
                # (clamped at initial cloud stamp 10.6 s), and a 0.3 s newer pose advances
                # minimum_cloud_stamp_ns to body_stamp_ns - 0.2 s so a frozen LiDAR fails closed.
                active.observe(10.7, (.0004, 0., .4))
                q1 = json.loads(queries[-1].data)
                self.assertEqual(q1["body_stamp_ns"], 10_700_000_000)
                self.assertEqual(q1["minimum_cloud_stamp_ns"], 10_600_000_000)
                active.observe(10.9, (.0005, 0., .4))
                q2 = json.loads(queries[-1].data)
                self.assertEqual(q2["body_stamp_ns"], 10_900_000_000)
                self.assertEqual(q2["minimum_cloud_stamp_ns"], 10_700_000_000)

            rig.run_scenario(scenario)


if __name__ == "__main__":
    unittest.main()
