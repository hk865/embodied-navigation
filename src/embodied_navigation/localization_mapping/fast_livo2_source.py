"""Stage vendored FAST-LIVO2 with build-time patches in a new build directory."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from embodied_navigation.project import project_root

# Patches 0001..0010 are already recorded in docs/dependencies/vendored-source-manifest.json
# on the vendored tree. Subsequent logic patches stay in patches/ and apply at build time.
FAST_LIVO2_BUILD_PATCHES = (
    "0011-imu-bias-covariance-params.patch",
    "0012-lidar-merge-capacity.patch",
)


def _file_hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def stage_fast_livo2_source(
    vendor_root: Path | str,
    build_root: Path | str,
    *,
    patches: tuple[str, ...] = FAST_LIVO2_BUILD_PATCHES,
    allow_inside_repo: bool = False,
) -> Path:
    """Return a copied, patched FAST-LIVO2 tree; never modifies the vendored tree."""
    vendor_root = Path(vendor_root).resolve()
    build_root = Path(build_root).resolve()
    if build_root.is_relative_to(vendor_root):
        raise ValueError("FAST-LIVO2 staging directory must be outside the vendor tree")
    if not allow_inside_repo and build_root.is_relative_to(project_root()):
        raise ValueError("FAST-LIVO2 staging directory must be outside the checkout")
    build_root.mkdir(parents=True)
    workspace = build_root / "source"
    staged = workspace / "third_party" / "fast_livo2"
    shutil.copytree(vendor_root, staged)
    source_files = _file_hashes(staged)
    patch_paths = [staged / "patches" / name for name in patches]
    missing = [str(p.name) for p in patch_paths if not p.is_file()]
    if missing:
        raise FileNotFoundError(f"Missing FAST-LIVO2 build patches: {missing}")
    manifest = {
        "vendor_root": str(vendor_root),
        "staged_root": str(staged),
        "source_files_sha256": source_files,
        "patch_order": list(patches),
        "patches": [],
        "status": "applying",
    }
    receipt = build_root / "source_stage.json"
    receipt.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    with (build_root / "patches.log").open("x", encoding="utf-8") as log:
        for patch in patch_paths:
            relative = patch.relative_to(workspace).as_posix()
            # Use GNU patch anchored at workspace so staging inside a git
            # checkout cannot be redirected to the outer repository root.
            command = ["patch", "--forward", "--batch", "-p1", "-d", str(workspace), "-i", str(patch)]
            applied = subprocess.run(command, cwd=workspace, stdout=log, stderr=log)
            manifest["patches"].append({
                "path": relative,
                "sha256": source_files[patch.relative_to(staged).as_posix()],
                "command": command,
                "returncode": applied.returncode,
            })
            manifest["status"] = "applying" if applied.returncode == 0 else "failed"
            receipt.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
            applied.check_returncode()
    staged_files = _file_hashes(staged)
    if patches and staged_files == source_files:
        raise RuntimeError("FAST-LIVO2 build patches produced no staged file changes")
    manifest.update(status="complete", staged_files_sha256=staged_files)
    receipt.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return staged
