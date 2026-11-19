"""CPU-only native mailbox pause contract tests; no Isaac or ROS startup."""
import json
from pathlib import Path
import tempfile
import unittest
from embodied_navigation.simulation.dynanav.capture.pause import FilePause


class Tracker:
    terminal_reason = None
    invalidated = 0
    polls = 0
    def invalidate_measurement(self):
        self.invalidated += 1
    def poll_request(self):
        self.polls += 1


class PauseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'pause_request.json'
        self.events, self.statuses = [], []
        self.state = dict(simtime=1.2, steps=240, published={'clock':120}, video_frames=12)
        self.snapshot_calls = 0
        self.tracker = Tracker()
        self.gate = FilePause(self.temp.name, journal=self.events.append,
            snapshot=self.snapshot, status_write=lambda path, value: self.statuses.append(value))

    def snapshot(self):
        self.snapshot_calls += 1
        return self.state.copy()

    def request(self, paused):
        self.path.write_text(json.dumps(dict(paused=paused, reason='decision')))

    def wait(self, idle, stop_requested=lambda: False):
        return self.gate.wait(tracker=self.tracker, cancelled=lambda: None,
                              stop_requested=stop_requested, idle=idle)

    def test_absent_file_is_noop(self):
        self.assertEqual(self.wait(lambda _: self.fail())['status'], 'running')
        self.assertEqual(self.snapshot_calls, 0)
        self.assertEqual(self.tracker.invalidated, 0)

    def test_pause_no_steps_then_resume_only_enter_exit_snapshot(self):
        self.request(True)
        calls = []
        def idle(seconds):
            calls.append(seconds)
            if len(calls) == 3:
                self.request(False)
        result = self.wait(idle)
        self.assertEqual(result['status'], 'resumed')
        self.assertTrue(result['counters_unchanged'])
        self.assertEqual(self.snapshot_calls, 2)
        self.assertEqual(self.tracker.invalidated, 1)
        self.assertEqual(self.tracker.polls, 4)
        self.assertEqual([s['paused'] for s in self.statuses], [True, False])

    def test_mailbox_stop_received_without_physics(self):
        self.request(True)
        self.tracker.poll_request = lambda: setattr(self.tracker, 'terminal_reason', 'interactive_stop')
        self.assertEqual(self.wait(lambda _: self.fail())['status'], 'stop_requested')
        self.assertEqual(self.snapshot_calls, 2)

    def test_cancel_can_be_received_and_pause_continues_until_resume(self):
        self.request(True)
        def poll():
            self.tracker.polls += 1  # Original cancel clears active goal, no terminal.
        self.tracker.poll_request = poll
        result = self.wait(lambda _: self.request(False))
        self.assertEqual(result['status'], 'resumed')
        self.assertEqual(self.tracker.polls, 2)

    def test_existing_stop_exits_pause(self):
        self.request(True)
        self.assertEqual(self.wait(lambda _: self.fail(), lambda: True)['status'], 'running')
        self.assertEqual(self.snapshot_calls, 0)

    def test_terminal_parking_does_not_reenter_persistent_pause(self):
        self.request(True)
        self.tracker.terminal_reason = 'interactive_stop'
        for _ in range(5):
            result = self.wait(lambda _: self.fail('parking must never idle in pause'))
            self.assertEqual(result, dict(status='running', paused_wall_s=0.0))
        self.assertEqual(self.snapshot_calls, 0)
        self.assertEqual(self.tracker.invalidated, 0)
        self.assertEqual(self.tracker.polls, 0)
        self.assertEqual(self.events, [])
        self.assertEqual(self.statuses, [])

    def test_malformed_input_requests_original_parking(self):
        self.path.write_text('{partial')
        self.assertEqual(self.wait(lambda _: self.fail())['status'], 'stop_requested')
        self.assertEqual(self.events[0]['event'], 'pause_request_invalid')

    def test_counter_difference_is_diagnostic_not_gate(self):
        self.request(True)
        def idle(seconds):
            self.state['steps'] += 1
            self.request(False)
        result = self.wait(idle)
        self.assertFalse(result['counters_unchanged'])
        self.assertEqual(result['status'], 'resumed')


if __name__ == '__main__':
    unittest.main()
