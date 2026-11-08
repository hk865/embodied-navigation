"""Opt-in ready notification; the existing control group still owns commits.

The mailbox is the work queue. Guard conditions may coalesce notifications;
there is no second pending flag, unbounded drain, clock read or hot-path I/O.
"""
import os


def selected():
    value = os.environ.get('R92_READY_COMMIT', '0')
    if value not in ('0', '1'):
        raise ValueError('R92_READY_COMMIT must be 0 or 1')
    return value == '1'


class ReadyCommit:
    def __init__(self, node, mailbox, commit):
        self.mailbox, self.commit = mailbox, commit
        self.notifications = self.callbacks = self.trigger_errors = 0
        self.closed = False
        self.guard = node.create_guard_condition(self._execute,
            callback_group=node.default_callback_group)
        self.same_control_group = self.guard.callback_group is node.default_callback_group
        if not self.same_control_group:
            raise RuntimeError('Ready commit must use the existing control callback group')
        mailbox.ready_notify = self.notify

    def notify(self):
        # Called only after a successful append and outside the mailbox lock.
        if self.closed:
            return
        self.notifications += 1
        try:
            self.guard.trigger()
        except Exception as error:
            self.trigger_errors += 1
            # Intake must never mutate business state. The unchanged timer
            # remains responsible for fail-closed publication on this fault.
            self.mailbox.fail('ready_notify:' + type(error).__name__)

    def _execute(self):
        if self.closed:
            return
        self.callbacks += 1
        self.commit()  # Exactly one original fixed batch, never a control tick.

    def close(self):
        # Existing owner shutdown has already joined executor callbacks.
        self.closed = True
        self.mailbox.ready_notify = None
        # The original Node.destroy_node owns guard destruction.

    def snapshot(self):
        return dict(schema='r92_ready_commit/v1', enabled=True,
            same_control_group=self.same_control_group,
            notifications=self.notifications, callbacks=self.callbacks,
            trigger_errors=self.trigger_errors, closed=self.closed,
            fixed_batch=True, original_control_timer_unchanged=True,
            source_stamp_or_receipts_refreshed=False)
