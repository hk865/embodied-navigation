"""Native-route mailbox pause; stop/cancel use the existing tracker protocol."""
import json
from pathlib import Path
import time


class FilePause:
    def __init__(self, mailbox, *, journal, snapshot, status_write,
                 clock=time.perf_counter):
        self.request_path = Path(mailbox) / 'pause_request.json'
        self.status_path = Path(mailbox) / 'pause_status.json'
        self.journal = journal
        self.snapshot = snapshot
        self.status_write = status_write
        self.clock = clock

    def request(self):
        try:
            row = json.loads(self.request_path.read_text())
            if type(row.get('paused')) is not bool:
                raise ValueError('paused must be true or false')
            return row['paused'], row.get('reason'), None
        except FileNotFoundError:
            return False, None, None
        except (OSError, ValueError, AttributeError) as error:
            return False, None, f'{type(error).__name__}: {error}'

    def wait(self, *, tracker, cancelled, stop_requested, idle):
        """No physics/render/publish here; only read requests and spin ROS.

        Enter/exit counters are diagnostic, never a new runtime admission gate.
        """
        if stop_requested() or tracker.terminal_reason is not None:
            return dict(status='running', paused_wall_s=0.0)
        paused, reason, error = self.request()
        if error:
            self.journal(dict(event='pause_request_invalid', error=error))
            self.status_write(self.status_path, dict(paused=False, state='invalid_request', error=error))
            return dict(status='stop_requested', paused_wall_s=0.0)
        if not paused:
            return dict(status='running', paused_wall_s=0.0)
        begin = self.clock()
        before = self.snapshot()
        tracker.invalidate_measurement()
        entered = dict(event='pause_enter', paused=True, state='paused',
                       started_wall_s=begin, reason=reason, before=before)
        self.journal(entered)
        self.status_write(self.status_path, entered)
        status = 'resumed'
        while True:
            cancelled()
            tracker.poll_request()
            if stop_requested() or tracker.terminal_reason is not None:
                status = 'stop_requested'
                break
            paused, _, error = self.request()
            if error:
                self.journal(dict(event='pause_request_invalid', error=error))
                status = 'stop_requested'
                break
            if not paused:
                break
            idle(0.02)
        after = self.snapshot()
        evidence = dict(event='pause_exit', paused=False, state=status, status=status,
                        paused_wall_s=self.clock()-begin, before=before, after=after,
                        counters_unchanged=before == after, error=error)
        self.journal(evidence)
        self.status_write(self.status_path, evidence)
        return evidence
