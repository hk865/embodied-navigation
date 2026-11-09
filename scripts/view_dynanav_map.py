#!/usr/bin/env python3
"""View an existing estimated XYZ map on one isolated, static review topic.

No controller, TF, simulator, map reconstruction, or GT pose is used. --check
validates the map and reports its real metadata/bounds without importing ROS.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

TOPIC = "/dynanav/review/static_map"
METADATA_FIELDS = (
    "provenance",
    "pose_source",
    "mask_source",
    "voxel_size",
    "no_GT_pose_used",
    "semantic_oracle",
    "complete_full_scene",
)


@dataclass(frozen=True)
class MapData:
    path: str
    source_sha256: str
    points: np.ndarray
    source_dtype: str
    map_frame: str
    frame_source: str
    metadata: dict
    minimum: tuple
    maximum: tuple


def valid_frame(value):
    if (
        not isinstance(value, str)
        or not value
        or value.strip() != value
        or any(character.isspace() for character in value)
        or "\0" in value
    ):
        raise ValueError(
            "map frame must be a nonempty frame ID without whitespace or NUL"
        )
    return value


def load_map(path, fallback_frame=None):
    path = Path(path).expanduser().resolve(strict=True)
    with np.load(path, allow_pickle=False) as arrays:
        if "points" not in arrays.files:
            raise ValueError("map NPZ has no points array")
        original = arrays["points"]
        if (
            original.ndim != 2
            or original.shape[1] != 3
            or original.shape[0] < 1
            or original.dtype.kind not in "fiu"
            or not np.isfinite(original).all()
        ):
            raise ValueError("points must be a nonempty finite numeric Nx3 array")
        if "map_frame" in arrays.files:
            frame = arrays["map_frame"]
            if frame.shape != () or frame.dtype.kind != "U":
                raise ValueError("map_frame metadata must be a scalar string")
            map_frame = valid_frame(frame.item())
            if fallback_frame is not None:
                raise ValueError(
                    "--frame is only permitted when map_frame metadata is absent"
                )
            frame_source = "original NPZ map_frame metadata"
        else:
            if fallback_frame is None:
                raise ValueError("map_frame metadata missing; provide explicit --frame")
            map_frame = valid_frame(fallback_frame)
            frame_source = "explicit CLI fallback because NPZ map_frame is absent"
        metadata = {}
        for name in METADATA_FIELDS:
            if name not in arrays.files:
                continue
            value = arrays[name]
            if value.shape != () or value.dtype.kind not in "Ufiub":
                raise ValueError(f"invalid scalar map metadata: {name}")
            scalar = value.item()
            if isinstance(scalar, (int, float)) and not math.isfinite(scalar):
                raise ValueError(f"nonfinite map metadata: {name}")
            metadata[name] = scalar
        minimum, maximum = (
            tuple(original.min(axis=0).tolist()),
            tuple(original.max(axis=0).tolist()),
        )
        with np.errstate(over="ignore", invalid="ignore"):
            points = np.ascontiguousarray(original, dtype="<f4")
        if not np.isfinite(points).all():
            raise ValueError(
                "map points cannot be represented as finite PointCloud2 float32 XYZ"
            )
        source_dtype = str(original.dtype)
    points.flags.writeable = False
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return MapData(
        str(path),
        digest.hexdigest(),
        points,
        source_dtype,
        map_frame,
        frame_source,
        metadata,
        minimum,
        maximum,
    )


def summary(data):
    center = [(a + b) / 2 for a, b in zip(data.minimum, data.maximum)]
    return {
        "source_path": data.path,
        "source_sha256": data.source_sha256,
        "point_count": len(data.points),
        "source_dtype": data.source_dtype,
        "published_xyz_dtype": "little-endian float32; only numeric encoding conversion",
        "map_frame": data.map_frame,
        "frame_source": data.frame_source,
        "metadata": data.metadata,
        "bounds_min_xyz_m": data.minimum,
        "bounds_max_xyz_m": data.maximum,
        "bounds_center_xyz_m": center,
        "topic": TOPIC,
        "qos": "reliable transient_local keep_last depth1",
        "header_stamp": "0: static aggregate; no original acquisition time asserted",
        "points_filtered_or_downsampled": False,
        "GT_pose_read": False,
        "TF_or_control_published": False,
    }


def cloud_message(data, cloud_type, point_field_type):
    """Build only XYZ from actual map points; message classes are dependency injected."""
    message = cloud_type()
    message.header.frame_id = data.map_frame
    message.header.stamp.sec = 0
    message.header.stamp.nanosec = 0
    message.height, message.width = 1, len(data.points)
    message.fields = [
        point_field_type(
            name=name, offset=index * 4, datatype=point_field_type.FLOAT32, count=1
        )
        for index, name in enumerate(("x", "y", "z"))
    ]
    message.is_bigendian = False
    message.point_step, message.row_step = 12, 12 * len(data.points)
    message.is_dense = True
    message.data = data.points.tobytes(order="C")
    return message


def static_publisher(node, cloud_type, qos):
    # Deliberately no arbitrary-topic argument or navigation/TF publisher.
    return node.create_publisher(cloud_type, TOPIC, qos)


def run_ros(data, duration):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField

    rclpy.init(args=[])
    node = None
    try:
        node = Node("dynanav_static_map_review")
        publisher = static_publisher(
            node,
            PointCloud2,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        publisher.publish(cloud_message(data, PointCloud2, PointField))
        node.get_logger().info(
            f"Review map: {len(data.points)} points in {data.map_frame} on {TOPIC}; publisher remains alive for late RViz subscribers"
        )
        deadline = time.monotonic() + duration if duration is not None else None
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(
                node,
                timeout_sec=0.1
                if deadline is None
                else min(0.1, max(0, deadline - time.monotonic())),
            )
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument(
        "--frame",
        help="explicit fallback only when source map_frame metadata is absent",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="optional finite positive publisher lifetime in wall seconds",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate/print only; do not import or start ROS",
    )
    args = parser.parse_args(argv)
    try:
        if args.duration is not None and (
            not math.isfinite(args.duration) or args.duration <= 0
        ):
            raise ValueError("duration must be finite and positive")
        data = load_map(args.map, args.frame)
        print(
            json.dumps(
                {
                    **summary(data),
                    "ROS_node_started": False,
                    "mode": "validated_check_only"
                    if args.check
                    else "validated_before_ROS_start",
                },
                indent=2,
                ensure_ascii=False,
                allow_nan=False,
            ),
            flush=True,
        )
        if args.check:
            return 0
        return run_ros(data, args.duration)
    except (OSError, ValueError, KeyError, RuntimeError, ImportError) as error:
        print(f"Map review error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
