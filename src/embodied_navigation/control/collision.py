"""Observed support and braking sweep checks from the existing shuttle."""

import numpy as np


def certified_support(points_xy, origin_xy, resolution_m, free_grid):
    """True only if every XY sample falls on a known-free grid cell.

    ``origin_xy`` is the grid's lower-corner world position and ``resolution_m``
    the cell size in metres; samples outside the grid are unsupported, never
    clamped to the nearest border cell.
    """
    indices = np.floor((np.asarray(points_xy) - origin_xy) / resolution_m).astype(int)
    return bool(
        np.all(indices >= 0)
        and np.all(indices < np.array(free_grid.shape))
        and free_grid[indices[:, 0], indices[:, 1]].all()
    )


def braking_sweep(position_xy, rotation_xy, command, measured_velocity_xy):
    """Planar braking sweep from the current body position and velocity.

    ``rotation_xy`` maps the body-frame XY command (m/s) into world axes.
    The larger commanded/measured speed sets the braking distance using 0.25 s
    reaction time, 0.25 m/s^2 deceleration and a 0.15 m margin. Fifteen points
    follow the commanded world velocity over a horizon of at least 0.8 s;
    this is not a minimum path length. A zero XY command leaves points coincident.
    """
    world = np.asarray(rotation_xy) @ np.asarray(command)[:2]
    planned_speed = max(float(np.linalg.norm(world)), float(np.linalg.norm(measured_velocity_xy)))
    braking_distance = planned_speed * 0.25 + planned_speed**2 / (2 * 0.25) + 0.15
    horizon = max(0.8, braking_distance / max(np.linalg.norm(world), 0.01))
    sweep = np.asarray(position_xy) + np.linspace(0, horizon, 15)[:, None] * world
    return sweep, braking_distance


def depth_collision(sweep, obstacle_xyz, forward_axis_xy, turning=False):
    """Closest planar obstacle distance along a precomputed sweep.

    Turning checks each sweep point itself; otherwise the two checked points
    are ``point +/- 0.18 * forward_axis_xy``, without a separate centre sample.
    The caller supplies the body forward axis. Danger radii are 0.30 m for
    straight motion and 0.48 m while turning. Returns
    ``(danger, closest_distance_or_None)`` without mutating inputs.
    """
    obstacle = np.asarray(obstacle_xyz)
    closest = None
    danger = False
    if len(obstacle):
        for point in sweep:
            offsets = [0.0] if turning else [-0.18, 0.18]
            for offset in offsets:
                distance = float(
                    np.linalg.norm(
                        obstacle[:, :2] - (point + offset * forward_axis_xy), axis=1
                    ).min()
                )
                closest = distance if closest is None else min(closest, distance)
                danger |= distance < (0.48 if turning else 0.30)
    return bool(danger), closest
