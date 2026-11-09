"""Canonical project paths; historical directories never enter runtime lookup."""

from embodied_navigation.project import project_root as _project_root
from pathlib import Path

REPOSITORY = _project_root()
TEACHER_ROOT = REPOSITORY
DEMO_ROOT = REPOSITORY
NAVIGATION_ROOT = REPOSITORY / "src"
RUN_STORAGE_ROOT = REPOSITORY / "runs"
GEOMETRY_STAGE = REPOSITORY / "assets/geometry"
GEOMETRY_HELPER = (
    REPOSITORY / "src/embodied_navigation/planning/admission/validate_route_support.py"
)
GEOMETRY_CONFIG = REPOSITORY / "configs/planning/static_support_limits.json"
GEOMETRY_METADATA = GEOMETRY_STAGE / "metadata.json"
