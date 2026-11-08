"""Trajectory-free PCD maps and overlapping spatial blocks.

Map construction and persistence run off the frontend owner thread. An external
map has no fabricated keyframes, odometry chain, or loop constraints. RGB is
retained as an optional attribute; this first geometric matcher uses XYZ only.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .geometry import voxel_sample


def read_pcd(path) -> tuple[np.ndarray, np.ndarray | None]:
    """Read standard little-endian ASCII/binary PCD XYZ, optionally RGB(A).

    Compressed PCD is deliberately rejected, not misread as raw binary. PCD's
    packed RGB float field is reinterpreted bitwise, not numerically converted.
    """
    with Path(path).open("rb") as stream:
        header = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError("PCD header has no DATA field")
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise ValueError("invalid PCD header") from error
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break
        fields = header.get("FIELDS", header.get("FIELD", []))
        if not all(x in fields for x in ("x", "y", "z")):
            raise ValueError("PCD requires x, y and z fields")
        try:
            sizes = list(map(int, header["SIZE"]))
            types = header["TYPE"]
            counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
            n = (int(header["POINTS"][0]) if "POINTS" in header else
                 int(header["WIDTH"][0]) * int(header.get("HEIGHT", ["1"])[0]))
        except (KeyError, ValueError, IndexError) as error:
            raise ValueError("invalid PCD dimensions or field definitions") from error
        if not (len(fields) == len(sizes) == len(types) == len(counts)) or n < 0 or any(c < 1 for c in counts):
            raise ValueError("inconsistent PCD field definitions")
        scalar_types = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4"}
        dtype_fields = []
        for field, typ, size, count in zip(fields, types, sizes, counts):
            if (typ, size) not in scalar_types:
                raise ValueError("unsupported PCD scalar type")
            dtype_fields.append((field, scalar_types[typ, size], (count,)) if count > 1 else (field, scalar_types[typ, size]))
        mode = header["DATA"][0].lower()
        if mode == "binary":
            dtype = np.dtype(dtype_fields)
            payload = stream.read()
            if len(payload) != n * dtype.itemsize:
                raise ValueError("PCD binary payload size differs from POINTS")
            records = np.frombuffer(payload, dtype=dtype, count=n)
            values = {field: records[field] for field in fields}
        elif mode == "ascii":
            try:
                matrix = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            except ValueError as error:
                raise ValueError("invalid PCD ASCII payload") from error
            if matrix.shape != (n, sum(counts)):
                raise ValueError("PCD ASCII payload dimensions differ from header")
            offsets = np.r_[0, np.cumsum(counts)]
            values = {field: matrix[:, offsets[i]] if counts[i] == 1 else matrix[:, offsets[i]:offsets[i + 1]]
                      for i, field in enumerate(fields)}
        else:
            raise ValueError("PCD DATA mode unsupported: " + mode)
        xyz = np.column_stack([values[k] for k in ("x", "y", "z")]).astype(np.float64)
        if xyz.shape != (n, 3) or not np.isfinite(xyz).all():
            raise ValueError("PCD XYZ must be finite scalar fields")
        colors = None
        packed_name = "rgb" if "rgb" in fields else "rgba" if "rgba" in fields else None
        if all(k in fields for k in ("r", "g", "b")):
            colors = np.column_stack([values[k] for k in ("r", "g", "b")]).astype(np.uint8)
        elif packed_name is not None:
            index = fields.index(packed_name)
            raw_color = np.asarray(values[packed_name])
            if types[index] == "F":
                if sizes[index] != 4:
                    raise ValueError("packed RGB float must have size 4")
                packed = raw_color.astype("<f4").view("<u4")
            else:
                packed = raw_color.astype(np.uint32)
            colors = np.column_stack([(packed >> shift) & 255 for shift in (16, 8, 0)]).astype(np.uint8)
        return xyz, colors


@dataclass(frozen=True)
class SpatialBlock:
    index: int
    minimum: np.ndarray
    maximum: np.ndarray
    points: np.ndarray


class ExternalMap:
    def __init__(self, points, block_size=20.0, overlap=6.0, voxel_size=0.12,
                 max_points_per_block=6000, colors=None, min_block_points=40):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
            raise ValueError("external map requires nonempty finite Nx3 XYZ")
        if (not 0 <= overlap < block_size or voxel_size < 0 or max_points_per_block < 40
                or min_block_points < 40 or min_block_points > max_points_per_block):
            raise ValueError("invalid spatial block configuration")
        self.block_size, self.overlap, self.voxel_size = float(block_size), float(overlap), float(voxel_size)
        self.max_points_per_block, self.min_block_points = int(max_points_per_block), int(min_block_points)
        self.points = points.copy()
        self.points.setflags(write=False)
        self.colors = None if colors is None else np.asarray(colors, dtype=np.uint8).copy()
        if self.colors is not None:
            if self.colors.shape != points.shape:
                raise ValueError("RGB must have the same Nx3 shape as XYZ")
            self.colors.setflags(write=False)
        sampled = voxel_sample(points, self.voxel_size)
        self.index = cKDTree(sampled)
        self.sampled_points = sampled
        self.sampled_points.setflags(write=False)
        stride = self.block_size - self.overlap
        minimum, maximum = sampled.min(axis=0), sampled.max(axis=0)
        dimensions = np.maximum(1, np.ceil((maximum - minimum - self.overlap) / stride).astype(int))
        if np.prod(dimensions, dtype=np.int64) > 100000:
            raise ValueError("map extent exceeds spatial-block build budget; increase block_size")
        self.blocks = []
        for ix in np.ndindex(tuple(dimensions)):
            lo = minimum + np.asarray(ix) * stride
            hi = lo + self.block_size
            ids = self.index.query_ball_point((lo + hi) / 2, np.sqrt(3) * self.block_size / 2 + 1e-9)
            p = sampled[np.asarray(ids, dtype=int)]
            p = p[np.all((p >= lo - 1e-9) & (p <= hi + 1e-9), axis=1)]
            if len(p) < self.min_block_points:
                continue
            # Stable spatial order before the bounded reduction.
            p = p[np.lexsort((p[:, 2], p[:, 1], p[:, 0]))]
            if len(p) > self.max_points_per_block:
                p = p[np.linspace(0, len(p) - 1, self.max_points_per_block, dtype=np.int64)]
            p.setflags(write=False)
            self.blocks.append(SpatialBlock(len(self.blocks), lo, hi, p))
        if not self.blocks:
            raise ValueError("map contains no sufficiently populated spatial block")

    @classmethod
    def from_pcd(cls, path, **kwargs):
        points, colors = read_pcd(path)
        return cls(points, colors=colors, **kwargs)

    def save(self, path):
        directory = Path(path)
        directory.mkdir(parents=True, exist_ok=True)
        # Offline asset writes, not a frontend publication protocol.
        np.save(directory / "points.npy", self.points, allow_pickle=False)
        if self.colors is not None:
            np.save(directory / "colors.npy", self.colors, allow_pickle=False)
        metadata = dict(schema="external_xyz_map/v1", frame="map", trajectory=None,
                        block_size=self.block_size, overlap=self.overlap, voxel_size=self.voxel_size,
                        max_points_per_block=self.max_points_per_block, min_block_points=self.min_block_points,
                        colors=self.colors is not None)
        (directory / "map.json").write_text(json.dumps(metadata, allow_nan=False) + "\n")

    @classmethod
    def load(cls, path):
        directory = Path(path)
        metadata = json.loads((directory / "map.json").read_text())
        if metadata.get("schema") != "external_xyz_map/v1" or metadata.get("trajectory") is not None:
            raise ValueError("not a trajectory-free external map asset")
        points = np.load(directory / "points.npy", allow_pickle=False)
        colors = np.load(directory / "colors.npy", allow_pickle=False) if metadata.get("colors") else None
        return cls(points, colors=colors, **{k: metadata[k] for k in
                   ("block_size", "overlap", "voxel_size", "max_points_per_block", "min_block_points")})
