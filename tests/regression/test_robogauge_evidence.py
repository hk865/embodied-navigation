import json
from pathlib import Path
import tempfile
import unittest
from embodied_navigation.navigation.robogauge.analyze import audit


class RoboGaugeEvidence(unittest.TestCase):
    def test_empty_logs_never_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = audit(tmp)
        self.assertFalse(result["execution_evidence_passed"])
        self.assertFalse(result["navigation_and_execution_passed"])

    def test_receipts_need_matching_physics_and_reject_malformed(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            telemetry = dict(
                world_sim_time=1.0, q_target=[0.2] * 12, position=[0, 0, 0.32], fault=None
            )
            exchange = dict(kind="policy_exchange", t=1.0, terminate=0, response_mode=0)
            step = dict(
                kind="physics_step",
                t=1.0,
                qtarget=[0.2] * 12,
                pd_raw=[30] * 12,
                qd=[0] * 12,
                tau=[23.4] * 12,
            )
            (run / "telemetry.jsonl").write_text(json.dumps(telemetry) + "\n")
            (run / "actuator_physics.jsonl").write_text(json.dumps(exchange) + "\n")
            self.assertFalse(audit(run)["execution_evidence_passed"])
            (run / "actuator_physics.jsonl").write_text(
                json.dumps(exchange) + "\n" + json.dumps(step) + "\n"
            )
            result = audit(run)
            self.assertTrue(result["execution_evidence_passed"])
            self.assertFalse(result["navigation_and_execution_passed"])
            (run / "truth_navigation_status.json").write_text('{"state":"succeeded"}')
            (run / "worker_result.json").write_text('{"fault":null}')
            self.assertTrue(audit(run)["navigation_and_execution_passed"])
            with (run / "actuator_physics.jsonl").open("a") as f:
                f.write("broken record\n")
            self.assertFalse(audit(run)["execution_evidence_passed"])

    def test_mismatched_target_is_not_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "telemetry.jsonl").write_text(
                json.dumps(
                    dict(world_sim_time=1.0, q_target=[0.1] * 12, position=[0, 0, 0.32], fault=None)
                )
                + "\n"
            )
            rows = [
                dict(kind="policy_exchange", t=1.0, terminate=0, response_mode=0),
                dict(
                    kind="physics_step",
                    t=1.0,
                    qtarget=[0.2] * 12,
                    pd_raw=[0] * 12,
                    qd=[0] * 12,
                    tau=[0] * 12,
                ),
            ]
            (run / "actuator_physics.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows)
            )
            result = audit(run)
            self.assertFalse(result["execution_evidence_passed"])
            self.assertAlmostEqual(result["maximum_same_stamp_qtarget_delta"], 0.1)


if __name__ == "__main__":
    unittest.main()
