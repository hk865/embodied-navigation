"""Throttle starts after the real write finishes, including atomic replacement."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

from embodied_navigation.simulation.dynanav.capture import receipts as module


class ReceiptCompletionThrottleTests(unittest.TestCase):
    def test_long_save_completion_starts_the_interval_and_force_still_saves(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            receipts = module.Receipts(output, {}, False, True)
            clock = [10.0]
            original_dumps, original_replace = json.dumps, Path.replace
            saves = []

            def serialize(*args, **kwargs):
                encoded = original_dumps(*args, **kwargs)
                clock[0] += 0.4
                return encoded

            def replace(path, target):
                returned = original_replace(path, target)
                clock[0] += 0.6
                saves.append(clock[0])
                return returned

            with patch.object(module.time, "perf_counter", side_effect=lambda: clock[0]), \
                    patch.object(module.json, "dumps", side_effect=serialize), \
                    patch.object(Path, "replace", autospec=True, side_effect=replace):
                receipts.result["revision"] = 1
                receipts.save()
                completed = clock[0]
                self.assertEqual(saves, [completed])
                receipts.result["revision"] = 2
                for delay in (0.005, 0.05, 0.249):
                    clock[0] = completed + delay
                    receipts.save()
                    self.assertEqual(saves, [completed])
                    self.assertEqual(json.loads((output / "result.json").read_text())["revision"], 1)
                clock[0] = completed + 0.251
                receipts.save()
                self.assertEqual(len(saves), 2)
                self.assertEqual(json.loads((output / "result.json").read_text())["revision"], 2)
                receipts.result["revision"] = 3
                receipts.save(force=True)
                self.assertEqual(len(saves), 3)
                self.assertEqual(json.loads((output / "result.json").read_text())["revision"], 3)
                self.assertFalse((output / "result.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
