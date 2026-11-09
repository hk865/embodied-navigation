"""Tracked office routes for explicit native-pose diagnostics.

The geometry provenance describes candidate surfaces, never successful traversal.
Loading a route neither starts Isaac nor sets the caller's speed or duration.
"""

from __future__ import annotations

from embodied_navigation.project import project_root
from embodied_navigation.simulation.dynanav.session_collision import (
    CONFIG_PATH,
    load_config,
)

ROUTE_NAMES = ("first-flight", "basement-to-ground")


def get_route(name):
    """Return fresh waypoints, collision-config path and geometry provenance.

    ``first-flight`` preserves the existing diagnostic target. The full route
    retains its explicitly chosen base-height targets rather than substituting
    the archived candidate's different surface-plus-clearance base heights.
    Runtime inputs come only from the tracked configuration, not the archive.
    """
    if name == "first-flight":
        config_path = CONFIG_PATH
        config = load_config(config_path)
        waypoints = [[-22.0, -7.5, -1.1441]]
    elif name == "basement-to-ground":
        config_path = (
            project_root()
            / "configs/simulation/dynanav_basement_to_ground_collision.json"
        )
        config = load_config(config_path)
        waypoints = config["route"]["waypoints_xyz"]
    else:
        raise ValueError(
            f"Unknown diagnostic route {name!r}; choose one of {ROUTE_NAMES}"
        )
    return {
        "name": name,
        "waypoints_xyz": waypoints,
        "config_path": str(config_path),
        "provenance": config["provenance"],
    }
