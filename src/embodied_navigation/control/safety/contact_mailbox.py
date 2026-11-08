"""Bounded nonblocking contacts-only IPC; no position, orientation or velocity."""
import errno
import math
import socket
import struct
import time

PACKET = struct.Struct('<Q5dQ')


class ContactPublisher:
    def __init__(self, endpoint):
        self.endpoint = str(endpoint)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.dropped = 0

    def publish(self, stamp_ns, contacts, fault=False):
        packet = PACKET.pack(stamp_ns, *contacts, int(bool(fault)))
        try:
            self.socket.sendto(packet, self.endpoint)
            return True
        except OSError as error:
            if error.errno not in (errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN, errno.ENOBUFS):
                raise
            self.dropped += 1
            return False

    def close(self):
        self.socket.close()


class ContactMailbox:
    def __init__(self, endpoint):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.socket.setblocking(False)
        self.socket.bind(str(endpoint))
        self.stamp_ns = -1
        self.receipt_wall_ns = 0
        self.contacts = (0.,) * 5
        self.fault = True

    def drain(self):
        for _ in range(64):
            try:
                packet = self.socket.recv(PACKET.size + 1)
            except BlockingIOError:
                break
            if len(packet) != PACKET.size:
                continue
            stamp, *values = PACKET.unpack(packet)
            contacts, fault = values[:5], values[5]
            if stamp <= self.stamp_ns or not all(math.isfinite(v) and v >= 0 for v in contacts):
                continue
            self.stamp_ns, self.contacts, self.fault = stamp, tuple(contacts), bool(fault)
            self.receipt_wall_ns = time.monotonic_ns()

    def fresh(self, clock_ns):
        return (not self.fault and -50_000_000 <= clock_ns-self.stamp_ns < 300_000_000
                and 0 <= time.monotonic_ns()-self.receipt_wall_ns < 300_000_000)

    def close(self):
        self.socket.close()
