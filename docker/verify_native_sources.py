#!/usr/bin/env python3
"""Read source hashes/XML/CMake, optionally build and smoke-test only PCT on CPU.

No network, ROS nodes, model loading or GPU operations. Temporary compiler
outputs stay below docs/dependencies and are removed on completion.
"""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import re
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {"fast_livo2_core", "fast_livo2_ros", "vikit_common", "vikit_ros",
            "livox_ros_driver2", "plan_env", "path_searching", "bspline_opt",
            "traj_utils", "scan_planner", "scan_planner_msgs"}


def command(args):
    completed = subprocess.run(list(map(str, args)), text=True, capture_output=True)
    if completed.returncode:
        raise RuntimeError(" ".join(map(str, args)) + "\n" + completed.stdout + completed.stderr)
    return completed.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compile-pct", action="store_true")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "docs/dependencies/vendored-source-manifest.json").read_text())
    for record in manifest["files"]:
        path = ROOT / record["destination"]
        if not path.is_file() or path.is_symlink():
            raise ValueError("Missing or symlinked source: " + str(path))
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != record["vendored_sha256"]:
            raise ValueError("Vendored source changed: " + str(path))
    packages = {}
    for path in (ROOT / "third_party").rglob("package.xml"):
        tree = ET.parse(path).getroot()
        name = tree.findtext("name")
        if name in packages:
            raise ValueError("Duplicate ROS package " + name)
        if not tree.findtext("license"):
            raise ValueError("Missing package license declaration " + str(path))
        packages[name] = str(path.parent.relative_to(ROOT))
    if set(packages) != EXPECTED:
        raise ValueError("Unexpected ROS package set: " + str(set(packages) ^ EXPECTED))
    python_count = 0
    for base in (ROOT / "docker", ROOT / "third_party"):
        for path in base.rglob("*.py"):
            ast.parse(path.read_text(), filename=str(path))
            python_count += 1
    missing = []
    cmakes = sorted((ROOT / "third_party").rglob("CMakeLists.txt"))
    for path in cmakes:
        text = path.read_text()
        for relative in re.findall(r"(?<![A-Za-z0-9_./])((?:src|test|msg|scripts)/[A-Za-z0-9_./-]+\.(?:cpp|cc|c|msg|py))", text):
            if not (path.parent / relative).is_file():
                missing.append(str(path.parent / relative))
        for relative in re.findall(r"\$\{CMAKE_CURRENT_SOURCE_DIR\}(/\.\./[^\s\)\"]+)", text):
            if not Path(str(path.parent) + relative).exists():
                missing.append(str(path.parent) + relative)
    if missing:
        raise ValueError("Missing CMake source references: " + repr(missing))
    with tempfile.TemporaryDirectory(prefix=".verification_tmp_", dir=ROOT / "docs/dependencies") as temporary:
        work = Path(temporary)
        # A function body is parsed, not executed. No find_package or builds here.
        for index, path in enumerate(cmakes):
            script = work / ("syntax_" + str(index) + ".cmake")
            script.write_text("function(source_syntax_only)\n" + path.read_text() + "\nendfunction()\n")
            command(["cmake", "-P", script])
        patch = ROOT / "third_party/fast_livo2/patches/0001-consistent-negative-voxel-floor.patch"
        command(["patch", "--dry-run", "--reverse", "--batch", "-p1", "-d",
                 ROOT / "third_party/fast_livo2", "-i", patch])
        report = dict(manifest_files_verified=len(manifest["files"]), ros_packages=packages,
                      cmake_files_parsed=len(cmakes), python_files_parsed=python_count,
                      symlinks=[str(p.relative_to(ROOT)) for p in (ROOT / "third_party").rglob("*") if p.is_symlink()],
                      vio_patch_reverse_dry_run="passed", pct_compile="not requested",
                      ros_compile="not run", docker_build="not run", simulation="not run")
        if report["symlinks"]:
            raise ValueError("Symlinks found in vendored sources")
        if args.compile_pct:
            command(["cmake", "-S", ROOT / "third_party/pct", "-B", work / "pct", "-DCMAKE_BUILD_TYPE=Release"])
            command(["cmake", "--build", work / "pct", "--parallel", "2"])
            data = work / "map.bin"
            values = [0.0] * (8 * 8 * 3)
            data.write_bytes(struct.pack("<iiid", 1, 8, 8, 0.1) + struct.pack("<" + "d" * len(values), *values))
            query = work / "query.txt"
            query.write_text("0 1 1 0 6 6\n")
            result = work / "path.json"
            command([work / "pct/pct_astar", data, query, result])
            path = json.loads(result.read_text())
            if not path["found"] or not path["cells_layer_row_col"]:
                raise ValueError("PCT did not find the bounded free-grid test path")
            report["pct_compile"] = "passed"
            report["pct_free_grid_smoke"] = dict(found=path["found"], points=len(path["cells_layer_row_col"]))
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
