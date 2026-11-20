"""A blocked rotation closes the action before later in-flight samples arrive."""
import json
import tempfile
import unittest

from scan_worker_acceptance_fixtures import WorkerRig, String


class YawBlockedLatchAcceptance(unittest.TestCase):
    def test_newer_inflight_clear_or_pending_cannot_replace_blocked(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = WorkerRig(directory)
            rig.adapter._handle_request(dict(version=1, run_id="generation", seq=2,
                action="yaw", pose_source="livo_post_vio", delta_yaw_rad=.3,
                created_wall_s=10., ttl_s=30.), now_wall=10.)
            rig.tick_adapter(10.01)

            def scenario(active):
                active.prepare()
                publisher = active.node.publisher("rotation_query")
                first = json.loads(publisher.messages[-1].data)
                active.poll(.05)
                second = json.loads(publisher.messages[-1].data)
                self.assertGreater(second["sample_id"], first["sample_id"])
                def reply(request, status):
                    return dict(request, status=status, reason="acceptance_"+status,
                        occupancy=1 if status=="blocked" else 0,
                        native_identity=dict(pid=active.processes[-1].pid,
                                             startup_nonce=request["startup_nonce"]))
                active.node.receive("rotation_result", String(json.dumps(reply(first, "blocked"))))
                path = active.mailbox / "planner_yaw_space.json"
                blocked = json.loads(path.read_text())
                self.assertEqual(blocked["status"], "blocked")
                # Both samples were actually sent before the blocking reply.
                # The later native result may arrive after it on the same child.
                for status in ("clear", "pending"):
                    active.node.receive("rotation_result", String(json.dumps(reply(second, status))))
                    self.assertEqual(json.loads(path.read_text()), blocked)
                active.poll(.05)
                self.assertEqual(json.loads(path.read_text()), blocked)

            rig.run_scenario(scenario)
            self.assertTrue(rig.node.destroyed)
            self.assertTrue(all(child.terminated for child in rig.processes))


if __name__ == "__main__":
    unittest.main()
