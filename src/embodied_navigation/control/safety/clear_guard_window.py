"""Actual fresh geometry observations required before releasing obstacle hold."""

from collections import deque


class ClearGuardWindow:
    def __init__(self):
        self.clocks = deque()

    def observe(self, clock_ns, blocked, evaluated):
        """Feed one real guard evaluation; True only after a full clear window.

        A blocked sample clears the window; an unevaluated sample returns False
        without changing it. A gap over 300 ms between evaluated samples restarts it.
        Release requires >= 10 fresh samples spanning >= 1 s and no blocked
        sample in the trailing 1.35 s window.
        """
        if blocked:
            self.clocks.clear()
            return False
        if not evaluated:
            return False
        if type(clock_ns) is not int or clock_ns < 0:
            raise ValueError("Actual guard clock required")
        if self.clocks and clock_ns <= self.clocks[-1]:
            raise ValueError("Guard clock did not advance")
        if self.clocks and clock_ns - self.clocks[-1] > 300_000_000:
            self.clocks.clear()
        self.clocks.append(clock_ns)
        while self.clocks and self.clocks[0] < clock_ns - 1_350_000_000:
            self.clocks.popleft()
        return len(self.clocks) >= 10 and self.clocks[-1] - self.clocks[0] >= 1_000_000_000
