"""Actual fresh geometry observations required before releasing obstacle hold."""
from collections import deque

class ClearGuardWindow:
    def __init__(self):self.clocks=deque()
    def observe(self,clock_ns,blocked,evaluated):
        if blocked:self.clocks.clear();return False
        if not evaluated:return False
        if type(clock_ns)is not int or clock_ns<0:raise ValueError('Actual guard clock required')
        if self.clocks and clock_ns<=self.clocks[-1]:raise ValueError('Guard clock did not advance')
        if self.clocks and clock_ns-self.clocks[-1]>300_000_000:self.clocks.clear()
        self.clocks.append(clock_ns)
        while self.clocks and self.clocks[0]<clock_ns-1_350_000_000:self.clocks.popleft()
        return len(self.clocks)>=10 and self.clocks[-1]-self.clocks[0]>=1_000_000_000
