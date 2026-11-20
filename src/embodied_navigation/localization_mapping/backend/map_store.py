"""Keyframe map: lidar clouds plus their ``T_world_body`` poses.

``world`` is the map frame (in mapping mode the first keyframe's odom frame).
Clouds are stored in the lidar frame, voxel-downsampled once on ``add``, and
moved into the world frame on demand with ``T_world_body @ T_body_lidar``, so
pose-graph updates never rewrite point data.

On-disk layout of ``save``/``load``: ``map.json`` (schema
``embodied_navigation_map/v1``) lists keyframes in insertion order with their
key, ``stamp_ns``, ``T_odom_body``, ``T_world_body``, ``T_body_lidar`` and npz
file name; each ``kf_<epoch>_<sequence>.npz`` holds ``points_lidar`` and
``descriptor``.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from .keyframes import Keyframe, transform_points, voxel_downsample

SCHEMA = "embodied_navigation_map/v1"


class MapStore:
    def __init__(self, voxel_m: float = 0.2):
        self.voxel_m = voxel_m
        self.keyframes: dict = {}
        self.poses: dict = {}  # key -> T_world_body
        self.descriptors: dict = {}

    def add(self, keyframe: Keyframe, T_world_body: np.ndarray, descriptor: np.ndarray) -> None:
        points = voxel_downsample(keyframe.points_lidar, self.voxel_m)
        self.keyframes[keyframe.key] = replace(keyframe, points_lidar=points)
        self.poses[keyframe.key] = np.array(T_world_body, dtype=np.float64)
        self.descriptors[keyframe.key] = descriptor

    def update_poses(self, poses: dict) -> None:
        """Replace ``T_world_body`` of the given keys (e.g. after optimisation)."""
        for key, pose in poses.items():
            self.poses[key] = np.array(pose, dtype=np.float64)

    def points_world(self, key) -> np.ndarray:
        keyframe = self.keyframes[key]
        return transform_points(self.poses[key] @ keyframe.T_body_lidar, keyframe.points_lidar)

    def submap(self, center_xyz: np.ndarray, radius_m: float, *, exclude_keys=()) -> np.ndarray:
        """World points of keyframes whose body origin lies within ``radius_m``."""
        excluded = set(exclude_keys)
        keys = [
            key
            for key, pose in self.poses.items()
            if key not in excluded and np.linalg.norm(pose[:3, 3] - center_xyz) <= radius_m
        ]
        return self._merge(keys, self.voxel_m)

    def global_map(self, voxel_m: float = 0.2) -> np.ndarray:
        return self._merge(list(self.poses), voxel_m)

    def _merge(self, keys, voxel_m) -> np.ndarray:
        if not keys:
            return np.empty((0, 3))
        return voxel_downsample(np.vstack([self.points_world(key) for key in keys]), voxel_m)

    def save(self, directory: Path) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        rows = []
        for key, keyframe in self.keyframes.items():
            name = f"kf_{key[0]}_{key[1]}.npz"
            np.savez(directory / name, points_lidar=keyframe.points_lidar, descriptor=self.descriptors[key])
            rows.append(
                {
                    "key": list(key),
                    "stamp_ns": keyframe.stamp_ns,
                    "T_odom_body": keyframe.T_odom_body.tolist(),
                    "T_world_body": self.poses[key].tolist(),
                    "T_body_lidar": keyframe.T_body_lidar.tolist(),
                    "file": name,
                }
            )
        document = {"schema": SCHEMA, "voxel_m": self.voxel_m, "keyframes": rows}
        (directory / "map.json").write_text(json.dumps(document))

    @classmethod
    def load(cls, directory: Path) -> "MapStore":
        directory = Path(directory)
        document = json.loads((directory / "map.json").read_text())
        if document.get("schema") != SCHEMA:
            raise ValueError(f"{directory}: schema is not {SCHEMA}")
        store = cls(document["voxel_m"])
        for row in document["keyframes"]:
            key = tuple(row["key"])
            with np.load(directory / Path(row["file"]).name) as arrays:
                points, descriptor = arrays["points_lidar"], arrays["descriptor"]
            keyframe = Keyframe(
                key, row["stamp_ns"], np.array(row["T_odom_body"]), np.array(row["T_body_lidar"]), points
            )
            store.keyframes[key] = keyframe
            store.poses[key] = np.array(row["T_world_body"])
            store.descriptors[key] = descriptor
        return store
