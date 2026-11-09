"""Repository/data root discovery without personal or sibling-project paths."""

import os
from pathlib import Path


def project_root():
    override = os.environ.get("EMBODIED_NAV_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    raise RuntimeError("Set EMBODIED_NAV_ROOT to the cloned repository")


def data_root():
    """This project's data directory beneath the global 6.project_data root."""
    value = os.environ.get("PROJECT_DATA_ROOT")
    if not value:
        raise RuntimeError("Set PROJECT_DATA_ROOT; large assets are not in Git")
    return Path(value).expanduser().resolve() / "embodied-navigation"
