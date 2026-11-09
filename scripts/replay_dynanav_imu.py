#!/usr/bin/env python3
"""Replay actual SDK IMU rows on received /clock, without truth or synthesized data.

Only time, accel_body, and gyro_body are selected from the JSONL. Importing this
module and --check require no ROS installation. ROS timestamps use the same
round(time * 1e9) conversion as the original capture publisher; float32 SDK time
roundoff is retained, not snapped to a nominal 200 Hz grid.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import sys
from bisect import bisect_right
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

FRAME_ID = "go2_imu"
IMU_TOPIC = "/dynanav/imu"
CLOCK_TOPIC = "/clock"
SELECTED_FIELDS = ("time", "accel_body", "gyro_body")
NS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class ImuSample:
    stamp_ns: int
    acceleration: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class ImuSource:
    path: str
    sha256: str
    size_bytes: int
    samples: tuple[ImuSample, ...]


def finite_number(value, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def sample_from_row(row, *, line_number: int = 1) -> ImuSample:
    """Whitelist three sensor fields; never inspect any other field's value."""
    prefix = f"line {line_number}"
    if not isinstance(row, dict):
        raise ValueError(f"{prefix}: expected a JSON object")  # noqa: TRY004 - Preserve the existing ValueError input-validation contract.
    for name in SELECTED_FIELDS:
        if name not in row:
            raise ValueError(f"{prefix}: missing required field {name}")
    seconds = finite_number(row["time"], f"{prefix}: time")
    if not 0 <= seconds < 2**31:
        raise ValueError(f"{prefix}: time is outside nonnegative ROS int32 seconds")
    stamp_ns = round(seconds * NS_PER_SECOND)
    if stamp_ns >= 2**31 * NS_PER_SECOND:
        raise ValueError(f"{prefix}: rounded time overflows ROS int32 seconds")

    def vector(name):
        values = row[name]
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError(f"{prefix}: {name} must have exactly three numbers")
        return tuple(
            finite_number(value, f"{prefix}: {name}[{index}]")
            for index, value in enumerate(values)
        )

    return ImuSample(stamp_ns, vector("accel_body"), vector("gyro_body"))


def load_source(path: str | Path) -> ImuSource:
    """Validate a finite, strictly ordered source while hashing its exact bytes."""
    path = Path(path).resolve(strict=True)
    if not path.is_file():
        raise ValueError("source must be a regular JSONL file")
    samples = []
    digest = hashlib.sha256()
    size_bytes = 0
    with path.open("rb") as handle:
        for line_number, line in enumerate(handle, 1):
            digest.update(line)
            size_bytes += len(line)
            if not line.strip():
                raise ValueError(
                    f"line {line_number}: blank row is missing sensor data"
                )
            try:
                row = json.loads(line)
            except (ValueError, UnicodeDecodeError) as error:
                raise ValueError(
                    f"line {line_number}: invalid JSON: {error}"
                ) from error
            sample = sample_from_row(row, line_number=line_number)
            if samples and sample.stamp_ns <= samples[-1].stamp_ns:
                raise ValueError(
                    f"line {line_number}: duplicate or backward source timestamp"
                )
            samples.append(sample)
    if not samples:
        raise ValueError("source contains no IMU samples")
    return ImuSource(str(path), digest.hexdigest(), size_bytes, tuple(samples))


class ClockSourceBuffer:
    """Publish every due physical sample once; reject reset or excessive backlog.

    A burst that exceeds the configured bound fails before any of that burst is
    emitted. No row is dropped, resampled, interpolated, or assigned a new time.
    Repeated /clock stamps are accepted without repeating an IMU sample.
    """

    def __init__(self, source: ImuSource, max_messages_per_clock: int = 4096):
        if type(max_messages_per_clock) is not int or max_messages_per_clock < 1:
            raise ValueError("max messages per clock must be a positive integer")
        self.source = source
        self.max_messages_per_clock = max_messages_per_clock
        self.stamps = tuple(sample.stamp_ns for sample in source.samples)
        self.published_count = 0
        self.last_clock_ns = None
        self.clock_messages_received = 0
        self.largest_burst = 0

    @property
    def complete(self) -> bool:
        return self.published_count == len(self.source.samples)

    def observe_clock(self, clock_ns: int) -> None:
        if type(clock_ns) is not int or clock_ns < 0:
            raise ValueError(
                "clock stamp must be a nonnegative integer nanosecond count"
            )
        if self.last_clock_ns is not None and clock_ns < self.last_clock_ns:
            raise ValueError(
                f"clock reset/backward jump: {clock_ns} < {self.last_clock_ns}"
            )
        self.last_clock_ns = clock_ns
        self.clock_messages_received += 1

    def publish_due(self, publish: Callable[[ImuSample], None]) -> int:
        if self.last_clock_ns is None:
            return 0
        end = bisect_right(self.stamps, self.last_clock_ns)
        count = end - self.published_count
        if count > self.max_messages_per_clock:
            raise ValueError(
                f"clock backlog {count} exceeds burst bound "
                f"{self.max_messages_per_clock}; no samples dropped"
            )
        self.largest_burst = max(self.largest_burst, count)
        while self.published_count < end:
            publish(self.source.samples[self.published_count])
            self.published_count += 1
        return count

    def advance(self, clock_ns: int, publish: Callable[[ImuSample], None]) -> int:
        self.observe_clock(clock_ns)
        return self.publish_due(publish)


def fill_imu_message(sample: ImuSample, message):
    """Populate a sensor_msgs/Imu-shaped object without any truth/orientation input."""
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        sample.stamp_ns, NS_PER_SECOND
    )
    message.header.frame_id = FRAME_ID
    for name, value in zip(("x", "y", "z"), sample.acceleration):
        setattr(message.linear_acceleration, name, value)
    for name, value in zip(("x", "y", "z"), sample.angular_velocity):
        setattr(message.angular_velocity, name, value)
    for name in ("x", "y", "z", "w"):
        setattr(message.orientation, name, 0.0)
    # sensor_msgs/Imu specifies -1 when orientation is not supplied.
    message.orientation_covariance = [-1.0] + [0.0] * 8
    message.angular_velocity_covariance = [0.0] * 9
    message.linear_acceleration_covariance = [0.0] * 9
    return message


def receipt_document(
    source: ImuSource,
    buffer: ClockSourceBuffer,
    *,
    status: str,
    error: str | None = None,
    delivery_acknowledged: bool | None = None,
):
    samples = source.samples
    gaps = [b.stamp_ns - a.stamp_ns for a, b in itertools.pairwise(samples)]
    published = buffer.published_count
    return {
        "schema": "dynanav_sdk_imu_replay_receipt_v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": status,
        "error": error,
        "source_path": source.path,
        "source_sha256": source.sha256,
        "source_size_bytes": source.size_bytes,
        "source_fields_used": list(SELECTED_FIELDS),
        "NO_GT_fields_used": True,
        "orientation_available": False,
        "orientation_covariance_0": -1.0,
        "frame_id": FRAME_ID,
        "output_topic": IMU_TOPIC,
        "clock_topic": CLOCK_TOPIC,
        "generated_clock": False,
        "timestamp_conversion": "round(recorded SDK time * 1000000000); no grid snapping",
        "acceleration_and_gyro": "actual recorded SDK values unchanged",
        "resampling": False,
        "interpolation": False,
        "extrapolation": False,
        "source_total_count": len(samples),
        "published_count": published,
        "all_source_samples_published": buffer.complete,
        "first_stamp_ns": samples[0].stamp_ns,
        "last_stamp_ns": samples[-1].stamp_ns,
        "first_published_stamp_ns": samples[0].stamp_ns if published else None,
        "last_published_stamp_ns": samples[published - 1].stamp_ns
        if published
        else None,
        "minimum_source_gap_ns": min(gaps) if gaps else None,
        "maximum_source_gap_ns": max(gaps) if gaps else None,
        "mean_source_rate_hz": (
            (len(samples) - 1)
            * NS_PER_SECOND
            / (samples[-1].stamp_ns - samples[0].stamp_ns)
        )
        if gaps
        else None,
        "clock_messages_received": buffer.clock_messages_received,
        "last_received_clock_ns": buffer.last_clock_ns,
        "max_messages_per_clock": buffer.max_messages_per_clock,
        "largest_published_burst": buffer.largest_burst,
        "reliable_delivery_acknowledged": delivery_acknowledged,
    }


def write_receipt(path: str | Path, document) -> None:
    """Create new evidence exclusively; existing artifacts are never replaced."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, allow_nan=False)
        handle.write("\n")


def run_ros(source: ImuSource, buffer: ClockSourceBuffer, args) -> int:
    # Imports are deliberately lazy: --check and unit tests cannot start ROS.
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu
    except ImportError as error:
        raise RuntimeError(
            "ROS unavailable; source /opt/ros/jazzy/setup.bash first"
        ) from error

    class ReplayNode(Node):
        def __init__(self):
            super().__init__("dynanav_sdk_imu_replay")
            self.failure = None
            self.done = False
            self.publisher = self.create_publisher(
                Imu,
                IMU_TOPIC,
                QoSProfile(
                    depth=max(4096, buffer.max_messages_per_clock),
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self.clock_subscription = self.create_subscription(
                Clock,
                CLOCK_TOPIC,
                self.on_clock,
                QoSProfile(
                    depth=10000,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self.retry_timer = self.create_timer(0.02, self.flush)
            self.get_logger().info(
                f"Ready: {len(source.samples)} actual SDK IMU samples; waiting for "
                f"/clock and {args.min_imu_subscribers} IMU subscriber(s)"
            )

        def fail(self, error):
            self.failure = str(error)
            self.get_logger().error(self.failure)
            self.done = True

        def on_clock(self, message):
            if self.done:
                return
            try:
                stamp = message.clock
                if stamp.sec < 0 or not 0 <= stamp.nanosec < NS_PER_SECOND:
                    raise ValueError("invalid received /clock stamp")
                buffer.observe_clock(stamp.sec * NS_PER_SECOND + stamp.nanosec)
                self.flush()
            except Exception as error:  # noqa: BLE001 - Record callback/publish failures from middleware-specific exception types.
                self.fail(error)

        def flush(self):
            if (
                self.done
                or self.publisher.get_subscription_count() < args.min_imu_subscribers
            ):
                return
            try:
                buffer.publish_due(
                    lambda sample: self.publisher.publish(
                        fill_imu_message(sample, Imu())
                    )
                )
                self.done = buffer.complete
            except Exception as error:  # noqa: BLE001 - Record callback/publish failures from middleware-specific exception types.
                self.fail(error)

    rclpy.init(args=[])
    node = None
    status, failure, acknowledged = "interrupted", None, None
    try:
        node = ReplayNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.failure:
            status, failure = "failed", node.failure
        elif buffer.complete:
            acknowledged = node.publisher.wait_for_all_acked(
                Duration(seconds=args.ack_timeout_s)
            )
            status = "completed" if acknowledged else "failed"
            if not acknowledged:
                failure = "timed out waiting for reliable IMU delivery acknowledgments"
    except (KeyboardInterrupt, ExternalShutdownException):
        failure = "stopped before complete replay/delivery acknowledgment"
    except Exception as error:  # noqa: BLE001 - Persist ROS startup/spin/ack failures before cleanup; middleware exception types vary.
        status, failure = "failed", str(error)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        write_receipt(
            args.receipt,
            receipt_document(
                source,
                buffer,
                status=status,
                error=failure,
                delivery_acknowledged=acknowledged,
            ),
        )
    if failure:
        print(f"IMU replay {status}: {failure}", file=sys.stderr)
    return 0 if status == "completed" else 2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", required=True, type=Path, help="actual SDK imu_stream.jsonl"
    )
    parser.add_argument(
        "--receipt",
        required=True,
        type=Path,
        help="new JSON evidence path; never overwritten",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate/hash only, without importing or starting ROS",
    )
    parser.add_argument(
        "--max-messages-per-clock",
        type=int,
        default=4096,
        help="reject an excessive due burst before publishing any of it (default: 4096)",
    )
    parser.add_argument(
        "--min-imu-subscribers",
        type=int,
        default=1,
        help="buffer received clock until this many IMU subscribers are discovered (default: 1)",
    )
    parser.add_argument(
        "--ack-timeout-s",
        type=float,
        default=10.0,
        help="maximum reliable delivery acknowledgment wait after final sample (default: 10)",
    )
    args = parser.parse_args(argv)
    try:
        if args.receipt.exists() or args.receipt.is_symlink():
            raise ValueError(f"receipt path already exists: {args.receipt}")
        if args.min_imu_subscribers < 0:
            raise ValueError("minimum IMU subscriber count must be nonnegative")
        if not math.isfinite(args.ack_timeout_s) or args.ack_timeout_s <= 0:
            raise ValueError("acknowledgment timeout must be finite and positive")
        source = load_source(args.source)
        buffer = ClockSourceBuffer(source, args.max_messages_per_clock)
        if args.check:
            document = receipt_document(source, buffer, status="validated_check_only")
            write_receipt(args.receipt, document)
            print(
                json.dumps(
                    {
                        key: document[key]
                        for key in (
                            "status",
                            "source_sha256",
                            "source_total_count",
                            "first_stamp_ns",
                            "last_stamp_ns",
                            "frame_id",
                            "NO_GT_fields_used",
                        )
                    },
                    sort_keys=True,
                )
            )
            return 0
        return run_ros(source, buffer, args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"IMU replay error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
