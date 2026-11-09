"""Lossless renderer RGB evidence, independent of ROS delivery and video overlays."""

from __future__ import annotations

import operator
from pathlib import Path

import numpy as np


class RawRgbArchive:
    """Write one uncompressed, non-overwriting NPZ per acquired camera frame."""

    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(exist_ok=False)
        self.count = 0
        self.first_source_ns = None
        self.last_source_ns = None
        self.shape = None

    def write(self, rgb, *, source_ns, frame_index, frame_id="go2_camera_optical"):
        """Persist the RGB annotator's bytes before any display annotation."""
        pixels = np.asarray(rgb)
        if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
            raise ValueError("Raw RGB must be an H x W x 3 uint8 array")
        if not pixels.shape[0] or not pixels.shape[1]:
            raise ValueError("Raw RGB dimensions must be nonempty")
        source_ns, frame_index = operator.index(source_ns), operator.index(frame_index)
        if source_ns < 0 or frame_index != self.count:
            raise ValueError(
                "Nonnegative source time and consecutive frame indices are required"
            )
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("A nonempty sensor frame ID is required")
        path = self.directory / f"{frame_index:06d}.npz"
        # Passing an exclusively created stream prevents np.savez from silently
        # replacing an existing evidence file. np.savez uses ZIP_STORED, not deflate.
        with path.open("xb") as stream:
            np.savez(
                stream,
                rgb=np.ascontiguousarray(pixels),
                source_ns=np.int64(source_ns),
                frame_index=np.int64(frame_index),
                frame_id=np.asarray(frame_id),
                encoding=np.asarray("rgb8"),
            )
        self.count += 1
        self.first_source_ns = (
            source_ns if self.first_source_ns is None else self.first_source_ns
        )
        self.last_source_ns = source_ns
        self.shape = list(pixels.shape)
        return path

    def receipt(
        self,
        *,
        acquired_frames,
        completed=False,
        published_frames=None,
        video_frames=None,
    ):
        """Report successful writes, with explicit normal-completion count checks."""
        complete_counts = (
            self.count == acquired_frames
            and self.count == published_frames
            and self.count == video_frames
        )
        return {
            "enabled": True,
            "path": str(self.directory),
            "count": self.count,
            "acquired_frames": acquired_frames,
            "published_rgb_frames": published_frames,
            "video_frames": video_frames,
            "capture_completed": completed,
            "all_acquired_frames_archived": self.count == acquired_frames,
            "all_frames_condition": (
                "successful NPZ writes equal acquired RGB frames; normal completion also requires "
                "equal published RGB and video frame counts"
            ),
            "all_frames_condition_met": complete_counts
            if completed
            else self.count == acquired_frames,
            "source": "rgb_ann.get_data RGB channels, before video overlay; not decoded from video",
            "source_timestamp": "round(SDK IMU reading time * 1e9), identical to RGB ROS header",
            "format": "uncompressed np.savez (ZIP_STORED)",
            "encoding": "rgb8",
            "dtype": "uint8",
            "shape": self.shape,
            "frame_index": "zero-based camera sample and video frame ordinal",
            "first_source_ns": self.first_source_ns,
            "last_source_ns": self.last_source_ns,
        }
