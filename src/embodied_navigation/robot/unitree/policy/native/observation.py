"""Frozen Teacher observations with explicit privileged Gazebo/SDF provenance.

The Isaac Lab scan's +20 m config offset raises ray STARTS only. The
observation reference remains the base frame position, so a flat scan is
``base_z - ground_z - 0.5``, never a constant 1 or 5. See contract.json.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

CONTRACT = json.loads(Path(__file__).with_name("contract.json").read_text())
DEFAULT_Q = np.asarray(CONTRACT["default_joint_positions"], dtype=np.float32)
JOINT_NAMES = tuple(CONTRACT["joint_names"])


def quaternion_rotation(quat_wxyz):
    """World-from-body rotation; transport uses Gazebo wxyz, Isaac uses xyzw."""
    q = np.asarray(quat_wxyz, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("Invalid base quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _pose(element):
    pose = element.find("pose")
    if pose is not None and pose.attrib.get("relative_to", ""):
        raise ValueError("SDF relative_to poses need explicit frame resolution")
    values = np.fromstring(pose.text or "", sep=" ") if pose is not None else np.zeros(6)
    if values.shape != (6,):
        raise ValueError("SDF pose must contain six xyz/rpy values")
    xyz, (r, p, y) = values[:3], values[3:]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rot = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    return xyz, rot


def _compose(parent, child):
    pos, rot = parent
    cp, cr = child
    return pos + rot @ cp, rot @ cr


@dataclass(frozen=True)
class _Box:
    name: str
    center: np.ndarray
    rotation: np.ndarray
    half_size: np.ndarray

    def intersect(self, starts):
        origins = (starts - self.center) @ self.rotation
        direction = np.asarray([0.0, 0.0, -1.0]) @ self.rotation
        t_enter = np.full(len(starts), -np.inf)
        t_exit = np.full(len(starts), np.inf)
        valid = np.ones(len(starts), dtype=bool)
        for axis in range(3):
            if abs(direction[axis]) < 1e-12:
                # Expanding XY bounds even 1e-9 m can select an upper deck
                # 2.4 m above a ray just outside its edge. Keep exact bounds.
                valid &= abs(origins[:, axis]) <= self.half_size[axis]
            else:
                a = (-self.half_size[axis] - origins[:, axis]) / direction[axis]
                b = (self.half_size[axis] - origins[:, axis]) / direction[axis]
                t_enter = np.maximum(t_enter, np.minimum(a, b))
                t_exit = np.minimum(t_exit, np.maximum(a, b))
        valid &= t_exit >= np.maximum(t_enter, 0.0)
        distance = np.where(t_enter >= 0.0, t_enter, t_exit)
        return np.where(valid & (distance >= 0.0), distance, np.inf)


@dataclass(frozen=True)
class _Plane:
    name: str
    center: np.ndarray
    rotation: np.ndarray
    half_size: np.ndarray
    normal: np.ndarray

    def intersect(self, starts):
        origins = (starts - self.center) @ self.rotation
        direction = np.asarray([0.0, 0.0, -1.0]) @ self.rotation
        divisor = np.dot(direction, self.normal)
        if abs(divisor) < 1e-12:
            return np.full(len(starts), np.inf)
        distance = -(origins @ self.normal) / divisor
        hits = origins + distance[:, None] * direction
        # Current demo planes are horizontal in their local frames.
        if not np.allclose(self.normal, [0.0, 0.0, 1.0]):
            raise ValueError("Finite plane with non-z local normal is unsupported")
        inside = (np.abs(hits[:, :2]) <= self.half_size).all(axis=1)
        return np.where(inside & (distance >= 0.0), distance, np.inf)


class TerrainHeightMap:
    """Exact vertical ray intersections for this demo's static box/plane collisions.

    SDF geometry is privileged ground truth used ONLY as a locomotion input.
    Navigation must use SLAM pose/maps. No floor, missing ray, or overhead
    surface is silently replaced with a flat ground assumption.
    """

    def __init__(self, shapes, source, source_sha256, excluded_models):
        self.shapes = tuple(shapes)
        self.source = str(source)
        self.source_sha256 = source_sha256
        self.excluded_models = tuple(excluded_models)
        if not self.shapes:
            raise ValueError("No supported static terrain geometry in SDF")

    @classmethod
    def from_sdf(
        cls, path, include_models=None, excluded_models=("go2", "teacher_go2", "moving_obstacle")
    ):
        path = Path(path).resolve()
        content = path.read_bytes()
        root = ET.fromstring(content)
        selected = set(include_models) if include_models is not None else None
        excluded = set(excluded_models)
        shapes = []

        def visit(model, parent_pose, parent_static=False, prefix=""):
            name = model.attrib.get("name", "")
            full_name = f"{prefix}/{name}" if prefix else name
            if name in excluded or full_name in excluded:
                return
            model_pose = _compose(parent_pose, _pose(model))
            static_text = model.findtext("static")
            static = (
                parent_static
                if static_text is None
                else static_text.strip().lower() in ("true", "1")
            )
            include = static and (selected is None or name in selected or full_name in selected)
            if include:
                for link in model.findall("link"):
                    link_pose = _compose(model_pose, _pose(link))
                    for collision in link.findall("collision"):
                        pos, rot = _compose(link_pose, _pose(collision))
                        geometry = collision.find("geometry")
                        collision_name = (
                            f"{full_name}/{link.attrib.get('name')}/{collision.attrib.get('name')}"
                        )
                        box = geometry.find("box") if geometry is not None else None
                        plane = geometry.find("plane") if geometry is not None else None
                        if box is not None:
                            size = np.fromstring(box.findtext("size", ""), sep=" ")
                            if size.shape != (3,) or (size <= 0).any():
                                raise ValueError(f"Invalid box {collision_name}")
                            shapes.append(_Box(collision_name, pos, rot, size / 2.0))
                        elif plane is not None:
                            size = np.fromstring(plane.findtext("size", ""), sep=" ")
                            normal = np.fromstring(plane.findtext("normal", "0 0 1"), sep=" ")
                            if (
                                size.shape != (2,)
                                or normal.shape != (3,)
                                or np.linalg.norm(normal) < 1e-8
                            ):
                                raise ValueError(f"Invalid plane {collision_name}")
                            shapes.append(
                                _Plane(
                                    collision_name,
                                    pos,
                                    rot,
                                    size / 2.0,
                                    normal / np.linalg.norm(normal),
                                )
                            )
                        elif geometry is not None and geometry.find("mesh") is not None:
                            if full_name != "Building":
                                raise ValueError("Unsupported experiment mesh " + full_name)
                            from embodied_navigation.simulation.mesh_terrain import MeshSurface

                            shapes.append(
                                MeshSurface(collision_name, geometry.findtext("mesh/uri"))
                            )
                        else:
                            raise ValueError(
                                f"Unsupported static collision shape: {collision_name}"
                            )
            for nested in model.findall("model"):
                visit(nested, model_pose, static, full_name)

        containers = root.findall("world") or [root]
        for container in containers:
            for model in container.findall("model"):
                visit(model, (np.zeros(3), np.eye(3)))
        return cls(shapes, path, None, excluded_models)

    load = from_sdf

    def raycast(self, starts):
        starts = np.asarray(starts, dtype=np.float64)
        if starts.ndim != 2 or starts.shape[1] != 3 or not np.isfinite(starts).all():
            raise ValueError("Ray starts must be finite Nx3")
        distances = np.asarray([shape.intersect(starts) for shape in self.shapes])
        winner = np.argmin(distances, axis=0)
        best = distances[winner, np.arange(len(starts))]
        valid = np.isfinite(best) & (best <= 1e6)
        hits_z = np.where(valid, starts[:, 2] - best, np.inf)
        hit_names = [self.shapes[i].name if ok else None for i, ok in zip(winner, valid)]
        return hits_z, hit_names

    def height(self, xy, ray_start_z=20.0):
        xy = np.asarray(xy, dtype=np.float64)
        if xy.ndim != 2 or xy.shape[1] != 2:
            raise ValueError("Height query must be Nx2")
        return self.raycast(np.column_stack((xy, np.broadcast_to(ray_start_z, len(xy)))))[0]


# Match torch.arange and torch.meshgrid(..., indexing="xy") in frozen Isaac Lab.
_gx, _gy = np.meshgrid(np.linspace(-0.8, 0.8, 17), np.linspace(-0.5, 0.5, 11), indexing="xy")
SCAN_GRID_XY = np.column_stack((_gx.ravel(), _gy.ravel()))


def build_observation(
    state, command, last_action, terrain, noisy=False, rng=None, base_com_offset=None
):
    """Return (float32[247], diagnostic dict) from the explicit request64 schema.

    state[8:11] must be body-frame COM velocity. For a source that instead
    provides link-origin velocity, supply its ACTUAL base_com_offset (body
    metres); the correction is ``omega cross offset``. Do not use a training
    offset for a Gazebo body whose inertial properties differ.
    """
    state = np.asarray(state, dtype=np.float64)
    command = np.asarray(command, dtype=np.float64)
    last_action = np.asarray(last_action, dtype=np.float64)
    if state.shape != (64,) or command.shape != (3,) or last_action.shape != (12,):
        raise ValueError("Expected state64, command3, last_action12")
    if (
        not np.isfinite(state).all()
        or not np.isfinite(command).all()
        or not np.isfinite(last_action).all()
    ):
        raise ValueError("Nonfinite source state/command/action")
    rot = quaternion_rotation(state[4:8])
    yaw = np.arctan2(rot[1, 0], rot[0, 0])
    cy, sy = np.cos(yaw), np.sin(yaw)
    yaw_rot = np.asarray([[cy, -sy], [sy, cy]])
    xy = SCAN_GRID_XY @ yaw_rot.T + state[1:3]
    starts = np.column_stack((xy, np.full(187, state[3] + 20.0)))
    hit_z, hit_names = terrain.raycast(starts)
    raw_scan = state[3] - hit_z - 0.5
    lin_vel = state[8:11].copy()
    if base_com_offset is not None:
        offset = np.asarray(base_com_offset, dtype=np.float64)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("base_com_offset must be finite body-frame xyz")
        lin_vel += np.cross(state[11:14], offset)
    raw_terms = (
        lin_vel,
        state[11:14],
        rot.T @ np.asarray([0.0, 0.0, -1.0]),
        command,
        state[14:26] - DEFAULT_Q,
        state[26:38],
        state[38:50],
        last_action,
        raw_scan,
    )
    if noisy and rng is None:
        raise ValueError("Matched noisy evaluation requires an explicit seeded RNG")
    terms = []
    for raw, spec in zip(raw_terms, CONTRACT["observations"]["concatenation_order"]):
        value = raw.copy()
        noise = spec["training_uniform_noise_before_clip"]
        if noisy and noise is not None:
            value += rng.uniform(noise[0], noise[1], size=value.shape)
        value = np.clip(value, *spec["clip"]) * spec["scale_after_clip"]
        terms.append(value)
    observation = np.concatenate(terms).astype(np.float32)
    if observation.shape != (247,) or not np.isfinite(observation).all():
        raise ValueError("Teacher observation is not finite 247D")
    valid = np.isfinite(hit_z)
    finite_raw = raw_scan[valid]
    extra = {
        "observation_source": "gazebo_privileged_state_and_static_sdf_vertical_rays",
        "terrain_source": terrain.source,
        "terrain_sha256": terrain.source_sha256,
        "terrain_collision_count": len(terrain.shapes),
        "excluded_models": list(terrain.excluded_models),
        "linear_velocity_reference": "base_com"
        if base_com_offset is None
        else "base_origin_corrected_to_com",
        "noise_enabled": bool(noisy),
        "height_scan_raw": [float(v) if np.isfinite(v) else None for v in raw_scan],
        "height_scan_hit_z": [float(v) if np.isfinite(v) else None for v in hit_z],
        "height_scan_valid_mask": valid.tolist(),
        "height_scan_missing_raw_semantics": "Missing hit is +inf and raw height is -inf; JSON null marks nonfinite raw evidence, policy clip produces -1",
        "height_scan_hit_models": hit_names,
        "height_scan_valid_count": int(valid.sum()),
        "height_scan_invalid_count": int((~valid).sum()),
        "height_scan_clip_fraction": float((abs(raw_scan) > 1.0).mean()),
        "height_scan_overhead_count": int((valid & (hit_z > state[3])).sum()),
        "height_scan_raw_min": float(finite_raw.min()) if len(finite_raw) else None,
        "height_scan_raw_max": float(finite_raw.max()) if len(finite_raw) else None,
        "height_scan_raw_range": float(np.ptp(finite_raw)) if len(finite_raw) else None,
        "height_scan_yaw_rad": float(yaw),
    }
    return observation, extra


from_state = build_observation
