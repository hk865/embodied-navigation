"""Bounded nonblocking contacts-only IPC; no position, orientation or velocity."""

import errno
import math
import socket
import struct
import time

# Wire layout, fixed little-endian: stamp_ns (u64), five contact values (f64),
# fault flag (u64). The five values are scalar contact counts/flags, not poses.
PACKET = struct.Struct("<Q5dQ")


class ContactPublisher:
    """Best-effort datagram sender; a full or absent mailbox drops a sample."""

    def __init__(self, endpoint):
        self.endpoint = str(endpoint)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.dropped = 0

    def publish(self, stamp_ns, contacts, fault=False):
        """Send one packet; return False when a recoverable send error drops it.

        Only the expected nonblocking mailbox conditions (no listener, refused,
        would-block, buffer full) are counted; any other OS error is re-raised so
        real transport breakage stays visible.
        """
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
    """Latest-sample receiver; ``drain`` keeps only strictly newer valid packets."""

    def __init__(self, endpoint):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.socket.setblocking(False)
        self.socket.bind(str(endpoint))
        self.stamp_ns = -1
        self.receipt_wall_ns = 0
        self.contacts = (0.0,) * 5
        self.fault = True

    def drain(self):
        """Consume at most 64 queued packets and retain the newest valid one.

        Backward stamps, wrong-sized datagrams and non-finite or negative contact
        values are ignored, so a corrupt sample cannot age out a good one.
        """
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
        """Require no fault, sim age in [-50, 300) ms and wall receipt age in [0, 300) ms."""
        return (
            not self.fault
            and -50_000_000 <= clock_ns - self.stamp_ns < 300_000_000
            and 0 <= time.monotonic_ns() - self.receipt_wall_ns < 300_000_000
        )

    def close(self):
        self.socket.close()
