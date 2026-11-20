"""Stage vendored SCAN and its patches in a new external build directory."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from embodied_navigation.project import project_root


def _file_hashes(root):
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def stage_scan_source(vendor_root, build_root):
    """Return the copied, patched SCAN tree; never reuse an existing build root.

    Patches in ``vendor_root/patches/*.patch`` apply in filename order and use
    repository-relative paths (``a/third_party/scan/...``). Their original bytes
    remain in the copied tree. Git's context-whitespace handling accepts both LF
    and CRLF checkouts without rewriting unchanged vendored lines.

    ``source_stage.json`` records input/output file hashes, ordered patch hashes,
    and application commands. A failed apply leaves its new directory and log
    available for diagnosis; it never updates the vendor tree or an old build.
    """
    vendor_root = Path(vendor_root).resolve()
    build_root = Path(build_root).resolve()
    if build_root.is_relative_to(project_root()) or build_root.is_relative_to(vendor_root):
        raise ValueError("SCAN staging directory must be outside the checkout and vendor tree")
    build_root.mkdir(parents=True)
    workspace = build_root / "source"
    staged = workspace / "third_party" / "scan"
    shutil.copytree(vendor_root, staged)
    source_files = _file_hashes(staged)
    patches = sorted((staged / "patches").glob("*.patch"))
    manifest = {
        "vendor_root": str(vendor_root),
        "staged_root": str(staged),
        "source_files_sha256": source_files,
        "patch_order": "filename",
        "patches": [],
        "status": "applying",
    }
    receipt = build_root / "source_stage.json"
    receipt.write_text(json.dumps(manifest, indent=2) + "\n")
    with (build_root / "patches.log").open("x") as log:
        for patch in patches:
            relative = patch.relative_to(workspace).as_posix()
            command = ["git", "apply", "--ignore-space-change", "--whitespace=nowarn", relative]
            applied = subprocess.run(command, cwd=workspace, stdout=log, stderr=log)
            manifest["patches"].append({
                "path": relative,
                "sha256": source_files[patch.relative_to(staged).as_posix()],
                "command": command,
                "returncode": applied.returncode,
            })
            manifest["status"] = "applying" if applied.returncode == 0 else "failed"
            receipt.write_text(json.dumps(manifest, indent=2) + "\n")
            applied.check_returncode()
    manifest.update(status="complete", staged_files_sha256=_file_hashes(staged))
    receipt.write_text(json.dumps(manifest, indent=2) + "\n")
    return staged
