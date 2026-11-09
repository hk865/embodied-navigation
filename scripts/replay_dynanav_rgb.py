#!/usr/bin/env python3
"""Replay original RGB NPZ frames on received clock; never create sensor time.

Reads only rgb/source_ns/frame_id/encoding/frame_index arrays. Other arrays,
including any truth data, are never loaded. Images are streamed and reverified
before publication. --check does not import ROS or publish anything.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import signal
import sys
import time
import zipfile
from bisect import bisect_right
from dataclasses import dataclass
from datetime import UTC, datetime
from io import BytesIO
from pathlib import Path

import numpy as np

FIELDS = ("rgb", "source_ns", "frame_id", "encoding", "frame_index")
NS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class RgbFrame:
    path: str
    stamp_ns: int
    frame_index: int
    frame_id: str
    encoding: str
    shape: tuple[int, int, int]
    source_sha256: str
    rgb_bytes_sha256: str


@dataclass(frozen=True)
class RgbArchive:
    path: str
    manifest_sha256: str
    frames: tuple[RgbFrame, ...]


def read_frame(path):
    """Read an actual source file once and load only the sensor whitelist."""
    path = Path(path)
    blob = path.read_bytes()
    file_hash = hashlib.sha256(blob).hexdigest()
    with zipfile.ZipFile(BytesIO(blob)) as archive:
        members = {entry.filename: entry for entry in archive.infolist()}
        if len(members) != len(archive.infolist()):
            raise ValueError(f"duplicate NPZ member: {path}")
        for name in FIELDS:
            member = members.get(name + ".npy")
            if member is None or member.compress_type != zipfile.ZIP_STORED:
                raise ValueError(
                    f"missing or non-original uncompressed NPZ field {name}: {path}"
                )
    with np.load(BytesIO(blob), allow_pickle=False) as arrays:
        for name in FIELDS:
            if name not in arrays.files:
                raise ValueError(f"missing RGB source field {name}: {path}")
        stamp, index = arrays["source_ns"], arrays["frame_index"]
        if (
            stamp.shape != ()
            or stamp.dtype != np.dtype("int64")
            or not 0 <= int(stamp) < 2**31 * NS_PER_SECOND
        ):
            raise ValueError(
                f"source_ns must be a nonnegative ROS-range scalar int64: {path}"
            )
        if index.shape != () or index.dtype != np.dtype("int64") or int(index) < 0:
            raise ValueError(f"frame_index must be a nonnegative scalar int64: {path}")
        stamp, index = int(stamp), int(index)
        if path.name != f"{index:06d}.npz":
            raise ValueError(f"filename does not match original frame_index: {path}")
        frame_id, encoding = arrays["frame_id"], arrays["encoding"]
        if (
            frame_id.shape != ()
            or frame_id.dtype.kind != "U"
            or not frame_id.item()
            or "\0" in frame_id.item()
        ):
            raise ValueError(f"invalid original frame_id: {path}")
        if (
            encoding.shape != ()
            or encoding.dtype.kind != "U"
            or encoding.item() != "rgb8"
        ):
            raise ValueError(f"original encoding must be rgb8: {path}")
        pixels = arrays["rgb"]
        if (
            pixels.dtype != np.uint8
            or pixels.ndim != 3
            or pixels.shape[2] != 3
            or min(pixels.shape[:2]) < 1
            or not pixels.flags.c_contiguous
        ):
            raise ValueError(f"RGB must be contiguous nonempty HxWx3 uint8: {path}")
        pixel_bytes = pixels.tobytes(order="C")
        frame = RgbFrame(
            str(path.resolve()),
            stamp,
            index,
            frame_id.item(),
            encoding.item(),
            tuple(pixels.shape),
            file_hash,
            hashlib.sha256(pixel_bytes).hexdigest(),
        )
    return frame, pixel_bytes


def load_archive(directory, cancel_check=lambda: None):
    directory = Path(directory).resolve(strict=True)
    if not directory.is_dir():
        raise ValueError("archive-dir must be an original rgb_raw directory")
    files = sorted(directory.glob("*.npz"))
    if not files:
        raise ValueError("archive contains no original RGB NPZ frames")
    frames, indices, stamps = [], set(), set()
    for path in files:
        cancel_check()
        frame, _ = read_frame(path)
        if frame.frame_index in indices or frame.stamp_ns in stamps:
            raise ValueError("duplicate original frame_index or source_ns")
        indices.add(frame.frame_index)
        stamps.add(frame.stamp_ns)
        frames.append(frame)
    frames.sort(key=lambda frame: frame.frame_index)
    if [frame.frame_index for frame in frames] != list(range(len(frames))):
        raise ValueError(
            "archive indices must form a complete original zero-based consecutive prefix"
        )
    for before, after in itertools.pairwise(frames):
        if after.stamp_ns <= before.stamp_ns:
            raise ValueError("backward original RGB source timestamp")
        if (after.shape, after.frame_id, after.encoding) != (
            before.shape,
            before.frame_id,
            before.encoding,
        ):
            raise ValueError("source RGB shape/frame/encoding changed within archive")
    manifest = [
        {
            "filename": Path(frame.path).name,
            "source_sha256": frame.source_sha256,
            "source_ns": frame.stamp_ns,
            "frame_index": frame.frame_index,
            "rgb_bytes_sha256": frame.rgb_bytes_sha256,
        }
        for frame in frames
    ]
    digest = hashlib.sha256(
        json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return RgbArchive(str(directory), digest, tuple(frames))


def select_frames(archive, filter_path=None):
    if filter_path is None:
        return archive.frames, {
            "mode": "all_original_archived_frames",
            "filter_path": None,
            "filter_sha256": None,
        }
    path = Path(filter_path).resolve(strict=True)
    blob = path.read_bytes()
    value = json.loads(blob)
    stamps = value.get("source_stamps_ns") if isinstance(value, dict) else value
    if (
        not isinstance(stamps, list)
        or not stamps
        or any(type(stamp) is not int or stamp < 0 for stamp in stamps)
    ):
        raise ValueError(
            "frozen source filter must contain a nonempty list of exact integer source_stamps_ns"
        )
    if len(stamps) != len(set(stamps)):
        raise ValueError("duplicate frozen source filter timestamp")
    wanted = set(stamps)
    available = {frame.stamp_ns for frame in archive.frames}
    if not wanted <= available:
        raise ValueError("frozen filter timestamp absent from original archive")
    selected = tuple(frame for frame in archive.frames if frame.stamp_ns in wanted)
    return selected, {
        "mode": "frozen_original_received_stamp_subset",
        "filter_path": str(path),
        "filter_sha256": hashlib.sha256(blob).hexdigest(),
        "filter_count": len(stamps),
        "filter_order_handling": "original archive acquisition order retained",
    }


class ClockBuffer:
    """Same due-sample contract as replay_dynanav_imu: no drop or rewind."""

    def __init__(self, frames, max_images_per_clock=64, max_clock_gap_ns=None):
        if type(max_images_per_clock) is not int or max_images_per_clock < 1:
            raise ValueError("max-images-per-clock must be positive")
        if max_clock_gap_ns is not None and (
            type(max_clock_gap_ns) is not int or max_clock_gap_ns < 1
        ):
            raise ValueError("max-clock-gap-ns must be a positive integer")
        self.frames = tuple(frames)
        if not self.frames or any(
            b.stamp_ns <= a.stamp_ns for a, b in zip(self.frames, self.frames[1:])
        ):
            raise ValueError(
                "scheduled frames must have unique increasing original timestamps"
            )
        self.stamps = tuple(frame.stamp_ns for frame in self.frames)
        self.max_images_per_clock = max_images_per_clock
        self.max_clock_gap_ns = max_clock_gap_ns
        self.published_count = 0
        self.clock_count = self.repeated_clocks = self.largest_burst = 0
        self.first_clock_ns = self.last_clock_ns = None
        self.minimum_positive_clock_gap_ns = self.maximum_positive_clock_gap_ns = None

    @property
    def complete(self):
        return self.published_count == len(self.frames)

    def observe_clock(self, stamp_ns):
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError("clock must be a nonnegative integer ns stamp")
        if self.last_clock_ns is not None:
            gap = stamp_ns - self.last_clock_ns
            if gap < 0:
                raise ValueError("clock reset/backward jump; no source rewind allowed")
            if gap == 0:
                self.repeated_clocks += 1
            else:
                self.minimum_positive_clock_gap_ns = min(
                    gap, self.minimum_positive_clock_gap_ns or gap
                )
                self.maximum_positive_clock_gap_ns = max(
                    gap, self.maximum_positive_clock_gap_ns or gap
                )
                if self.max_clock_gap_ns is not None and gap > self.max_clock_gap_ns:
                    raise ValueError(
                        f"observed clock gap {gap} exceeds bound {self.max_clock_gap_ns}"
                    )
        if self.first_clock_ns is None:
            self.first_clock_ns = stamp_ns
        self.last_clock_ns = stamp_ns
        self.clock_count += 1

    def publish_due(self, publish):
        if self.last_clock_ns is None:
            return 0
        end = bisect_right(self.stamps, self.last_clock_ns)
        count = end - self.published_count
        if count > self.max_images_per_clock:
            raise ValueError(
                f"RGB due burst {count} exceeds bound {self.max_images_per_clock}; no images dropped"
            )
        self.largest_burst = max(self.largest_burst, count)
        while self.published_count < end:
            publish(self.frames[self.published_count])
            self.published_count += 1
        return count

    def advance(self, stamp_ns, publish):
        self.observe_clock(stamp_ns)
        return self.publish_due(publish)


def verified_pixels(frame):
    current, pixels = read_frame(frame.path)
    if current != frame:
        raise ValueError(f"original RGB source changed after validation: {frame.path}")
    return pixels


def fill_image(frame, message):
    pixels = verified_pixels(frame)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        frame.stamp_ns, NS_PER_SECOND
    )
    message.header.frame_id = frame.frame_id
    message.height, message.width = frame.shape[:2]
    message.encoding = frame.encoding
    message.is_bigendian = 0
    message.step = frame.shape[1] * 3
    message.data = pixels
    return message


def receipt_document(
    archive, selected, selection, buffer, args, *, status, error=None, ack=None
):
    published = buffer.published_count
    return {
        "schema": "dynanav_original_rgb_replay_receipt/v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": status,
        "error": error,
        "source_validation_completed": True,
        "archive_dir": archive.path,
        "archive_manifest_sha256": archive.manifest_sha256,
        "source_fields_used": list(FIELDS),
        "NO_GT_fields_used": True,
        "source_archive_count": len(archive.frames),
        "scheduled_count": len(selected),
        "published_count": published,
        "selection": selection,
        "all_scheduled_frames_published": buffer.complete,
        "all_archive_frames_selected": len(selected) == len(archive.frames),
        "all_archive_frames_published": buffer.complete
        and len(selected) == len(archive.frames),
        "completion_confirmed": status == "completed"
        and buffer.complete
        and ack is True,
        "completion_scope": "publisher-side selected original-frame schedule plus reliable acknowledgment only; not route completion or proof of native observer receipt",
        "archive_scope": "validated consecutive stored source prefix; does not establish completion of acquisition or route",
        "first_scheduled_stamp_ns": selected[0].stamp_ns,
        "last_scheduled_stamp_ns": selected[-1].stamp_ns,
        "first_published_stamp_ns": selected[0].stamp_ns if published else None,
        "last_published_stamp_ns": selected[published - 1].stamp_ns
        if published
        else None,
        "original_frame_id": selected[0].frame_id,
        "encoding": selected[0].encoding,
        "shape": list(selected[0].shape),
        "image_topic": args.image_topic,
        "clock_topic": args.clock_topic,
        "generated_clock": False,
        "timestamp_conversion": "none; exact original source_ns integer",
        "resampling": False,
        "interpolation": False,
        "extrapolation": False,
        "pixel_changes": False,
        "source_file_reverified_before_each_publish": True,
        "clock_count": buffer.clock_count,
        "first_received_clock_ns": buffer.first_clock_ns,
        "last_received_clock_ns": buffer.last_clock_ns,
        "repeated_clock_count": buffer.repeated_clocks,
        "minimum_observed_positive_clock_gap_ns": buffer.minimum_positive_clock_gap_ns,
        "maximum_observed_positive_clock_gap_ns": buffer.maximum_positive_clock_gap_ns,
        "clock_gap_scope": "observed received-clock gaps only; no inferred clock/sample loss count",
        "last_clock_reached_final_scheduled_stamp": buffer.last_clock_ns is not None
        and buffer.last_clock_ns >= selected[-1].stamp_ns,
        "max_images_per_clock": buffer.max_images_per_clock,
        "largest_published_burst": buffer.largest_burst,
        "max_clock_gap_ns": buffer.max_clock_gap_ns,
        "reliable_acknowledgment_succeeded": ack,
        "ack_scope": "reliable publisher acknowledgment does not prove receipt by BEST_EFFORT native subscribers",
        "native_observer_receipt_verified": False,
        "source_frames": [
            {
                "path": f.path,
                "frame_index": f.frame_index,
                "source_ns": f.stamp_ns,
                "source_sha256": f.source_sha256,
                "rgb_bytes_sha256": f.rgb_bytes_sha256,
                "scheduled": f in selected,
            }
            for f in archive.frames
        ],
    }


def write_receipt(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf8") as handle:
        json.dump(document, handle, indent=2, allow_nan=False)
        handle.write("\n")


class Cancelled(Exception):
    pass


class Cancellation:
    def __init__(self):
        self.reason = None

    def signal(self, signum, _frame):
        self.reason = signal.Signals(signum).name

    def check(self):
        if self.reason:
            raise Cancelled(self.reason)


def run_ros(archive, selected, selection, buffer, args, cancellation):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    class Replay(Node):
        def __init__(self):
            super().__init__("dynanav_original_rgb_replay")
            self.failure = None
            self.last_clock_arrival = time.monotonic()
            self.publisher = self.create_publisher(
                Image,
                args.image_topic,
                QoSProfile(
                    depth=4096,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self.subscription = self.create_subscription(
                Clock,
                args.clock_topic,
                self.clock,
                QoSProfile(
                    depth=10000,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self.get_logger().info(
                f"Ready: {len(selected)}/{len(archive.frames)} original RGB frames; "
                f"selection={selection['mode']}; waiting for recorded clock/subscribers"
            )

        def clock(self, message):
            try:
                stamp = message.clock
                if stamp.sec < 0 or not 0 <= stamp.nanosec < NS_PER_SECOND:
                    raise ValueError("invalid received clock")
                buffer.observe_clock(stamp.sec * NS_PER_SECOND + stamp.nanosec)
                self.last_clock_arrival = time.monotonic()
                self.flush()
            except Exception as error:  # noqa: BLE001 - native ROS/serialization errors must produce a failed replay receipt.
                self.failure = str(error)

        def flush(self):
            if (
                self.failure
                or cancellation.reason
                or self.publisher.get_subscription_count() < args.min_image_subscribers
            ):
                return
            try:
                buffer.publish_due(
                    lambda frame: self.publisher.publish(fill_image(frame, Image()))
                )
            except Exception as error:  # noqa: BLE001 - native ROS/serialization errors must produce a failed replay receipt.
                self.failure = str(error)

    node, status, error, ack = None, "incomplete", None, None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = Replay()
        while (
            rclpy.ok()
            and not buffer.complete
            and not node.failure
            and not cancellation.reason
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
            node.flush()
            if (
                args.idle_timeout_s is not None
                and time.monotonic() - node.last_clock_arrival > args.idle_timeout_s
            ):
                error = "received-clock idle timeout before complete source schedule"
                break
        if cancellation.reason:
            status, error = "cancelled", cancellation.reason
        elif node.failure:
            status, error = "failed", node.failure
        elif buffer.complete:
            # Short bounded waits let SIGTERM/SIGINT cancel during acknowledgment.
            deadline = time.monotonic() + args.ack_timeout_s
            ack = False
            while not cancellation.reason and time.monotonic() < deadline:
                if node.publisher.wait_for_all_acked(
                    Duration(seconds=min(0.1, max(0, deadline - time.monotonic())))
                ):
                    ack = True
                    break
            if cancellation.reason:
                status, error = "cancelled", cancellation.reason
            elif ack:
                status = "completed"
            else:
                status, error = "failed", "reliable acknowledgment timed out"
        elif error is None:
            error = "ROS stopped before final original-frame schedule completed"
    except (KeyboardInterrupt, ExternalShutdownException) as failure:
        status, error = "cancelled", str(failure) or "ROS external shutdown"
    except Exception as failure:  # noqa: BLE001 - record unexpected middleware failure without claiming completion.
        status, error = "failed", str(failure)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        write_receipt(
            args.receipt,
            receipt_document(
                archive,
                selected,
                selection,
                buffer,
                args,
                status=status,
                error=error,
                ack=ack,
            ),
        )
    if error:
        print(f"RGB replay {status}: {error}", file=sys.stderr)
    return 0 if status == "completed" else (130 if status == "cancelled" else 2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive-dir", type=Path, required=True)
    parser.add_argument(
        "--receipt", type=Path, required=True, help="new exclusive JSON receipt path"
    )
    parser.add_argument(
        "--source-received-stamps",
        type=Path,
        help="frozen JSON integer list or {source_stamps_ns:[...]}; preserve original acquisition order",
    )
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--image-topic", default="/dynanav/camera")
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate/hash only; do not import or start ROS",
    )
    parser.add_argument("--max-images-per-clock", type=int, default=64)
    parser.add_argument("--max-clock-gap-ns", type=int)
    parser.add_argument("--min-image-subscribers", type=int, default=1)
    parser.add_argument("--ack-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--idle-timeout-s",
        type=float,
        help="optional received-clock inactivity bound; no resampling",
    )
    args = parser.parse_args(argv)
    cancellation = Cancellation()
    old_handlers = {}
    archive = selected = selection = buffer = None
    try:
        if args.receipt.exists() or args.receipt.is_symlink():
            raise ValueError(f"receipt already exists: {args.receipt}")
        for name in ("clock_topic", "image_topic"):
            value = getattr(args, name)
            if not value.startswith("/") or value == "/":
                raise ValueError(f"{name} must be an absolute non-root ROS topic")
        if args.min_image_subscribers < 0:
            raise ValueError("min-image-subscribers must be nonnegative")
        for name in ("ack_timeout_s", "idle_timeout_s"):
            value = getattr(args, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive")
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, cancellation.signal)
        archive = load_archive(args.archive_dir, cancellation.check)
        selected, selection = select_frames(archive, args.source_received_stamps)
        buffer = ClockBuffer(selected, args.max_images_per_clock, args.max_clock_gap_ns)
        cancellation.check()
        if args.check:
            document = receipt_document(
                archive,
                selected,
                selection,
                buffer,
                args,
                status="validated_check_only",
            )
            write_receipt(args.receipt, document)
            print(
                json.dumps(
                    {
                        key: document[key]
                        for key in (
                            "status",
                            "archive_manifest_sha256",
                            "source_archive_count",
                            "scheduled_count",
                            "published_count",
                            "first_scheduled_stamp_ns",
                            "last_scheduled_stamp_ns",
                            "NO_GT_fields_used",
                        )
                    },
                    sort_keys=True,
                )
            )
            return 0
        return run_ros(archive, selected, selection, buffer, args, cancellation)
    except (
        OSError,
        ValueError,
        RuntimeError,
        ImportError,
        zipfile.BadZipFile,
        Cancelled,
    ) as failure:
        status = "cancelled" if isinstance(failure, Cancelled) else "failed"
        document = (
            receipt_document(
                archive,
                selected,
                selection,
                buffer,
                args,
                status=status,
                error=str(failure),
            )
            if buffer is not None
            else {
                "schema": "dynanav_original_rgb_replay_receipt/v1",
                "status": status,
                "error": str(failure),
                "source_validation_completed": False,
                "archive_dir": str(args.archive_dir),
                "published_count": 0,
                "completion_confirmed": False,
                "NO_GT_fields_used": True,
                "generated_clock": False,
            }
        )
        if not args.receipt.exists() and not args.receipt.is_symlink():
            try:
                write_receipt(args.receipt, document)
            except OSError as receipt_failure:
                print(f"RGB receipt error: {receipt_failure}", file=sys.stderr)
        print(f"RGB replay {status}: {failure}", file=sys.stderr)
        return 130 if status == "cancelled" else 2
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
