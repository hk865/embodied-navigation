#!/usr/bin/env python3
"""Build project-local ROS navigation dependencies from physical vendored source.

Default is a read-only plan. --execute downloads pinned public source archives
and builds CPU code; it never starts ROS nodes, a simulator or a robot.
"""

from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from embodied_navigation.localization_mapping.fast_livo2_source import (
    FAST_LIVO2_BUILD_PATCHES,
    stage_fast_livo2_source,
)

PACKAGES = [
    "livox_ros_driver2",
    "vikit_common",
    "vikit_ros",
    "dynanav_sensor_msgs",
    "fast_livo2_core",
    "fast_livo2_ros",
    "plan_env",
    "path_searching",
    "bspline_opt",
    "traj_utils",
    "scan_planner_msgs",
    "scan_planner",
]
BASES = [ROOT / "third_party" / d for d in ("fast_livo2", "scan", "vikit", "livox_ros_driver2")]


def run(command, **kwargs):
    """Echo and run one build command with ``check=True`` (failure is fatal)."""
    print("+", " ".join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, **kwargs)


def digest(path):
    """Return the SHA256 hex digest of one local source archive."""
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def download(pin, target):
    """Download one pinned public archive and verify it before caching.

    A pre-existing cache file must already match the pin; a fresh download is
    written to ``*.part`` and renamed only after the checksum matches.
    """
    if target.exists():
        if digest(target) != pin["sha256"]:
            raise ValueError("Cached source checksum mismatch: " + str(target))
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".part")
    with urllib.request.urlopen(pin["url"], timeout=90) as source, temporary.open("wb") as output:
        while data := source.read(1024 * 1024):
            output.write(data)
    if digest(temporary) != pin["sha256"]:
        raise ValueError("Downloaded source checksum mismatch: " + str(temporary))
    temporary.replace(target)


def extract(archive, destination, expected_root):
    """Reject traversal/device members before extracting one pinned archive."""
    with tarfile.open(archive, "r:gz") as source:
        for member in source.getmembers():
            target = destination / member.name
            if (
                not target.resolve().is_relative_to(destination.resolve())
                or member.name.split("/")[0] != expected_root
                or member.isdev()
            ):
                raise ValueError("Unexpected source archive member: " + member.name)
        source.extractall(destination, filter="data")


def cmake(source, build, prefix, jobs, extra=(), env=None):
    """Configure with a Release default, then build/install; no test runner is invoked."""
    run(
        [
            "cmake",
            "-S",
            source,
            "-B",
            build,
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
            *extra,
        ],
        env=env,
    )
    run(["cmake", "--build", build, "--parallel", jobs], env=env)
    run(["cmake", "--install", build], env=env)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--source-cache",
        type=Path,
        help="Use only the verified livox_sdk2.tar.gz archive; no downloads or fallback",
    )
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error("Use 1..4 CPU build jobs")
    work, install = ROOT / "build/native", ROOT / "install"
    pins = json.loads((ROOT / "docker/source-pins.json").read_text())
    plan = dict(
        source_roots=list(map(str, BASES)),
        packages=PACKAGES,
        source_pins=pins,
        build_root=str(work),
        install_root=str(install),
        jobs=args.jobs,
        source_cache=str(args.source_cache) if args.source_cache else None,
        vio_floor_patch="Already applied in vendored fast_livo2_core/src/vio.cpp",
        fast_livo2_build_patches=[
            f"third_party/fast_livo2/patches/{name}" for name in FAST_LIVO2_BUILD_PATCHES
        ],
        starts_nodes=False,
        starts_simulation=False,
        requires_gpu=False,
    )
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return
    if work.exists() or any((install / p).exists() for p in ("livox", "ros", "pct")):
        raise RuntimeError(
            "Use a fresh build/native and install/{livox,ros,pct}; no reused CMake objects"
        )
    if args.source_cache:
        for name, pin in pins.items():
            cached = args.source_cache / (name + ".tar.gz")
            if not cached.is_file() or digest(cached) != pin["sha256"]:
                raise ValueError("Missing or checksum-invalid source cache: " + str(cached))
    sources = work / "sources"
    sources.mkdir(parents=True)
    for name, pin in pins.items():
        target = sources / (name + ".tar.gz")
        if args.source_cache:
            shutil.copyfile(args.source_cache / target.name, target)
            if digest(target) != pin["sha256"]:
                raise ValueError("Copied source checksum mismatch: " + str(target))
        else:
            download(pin, target)
    sdk = sources / pins["livox_sdk2"]["archive_root"]
    extract(sources / "livox_sdk2.tar.gz", sources, sdk.name)
    # Keep the original GCC13 compatibility fix explicit and reproducible.
    header = sdk / "sdk_core/comm/define.h"
    original = header.read_text()
    if "#include <cstdint>" not in original:
        marker = "#define LIVOX_DEFINE_H_"
        if original.count(marker) != 1:
            raise ValueError("Unexpected pinned Livox comm header")
        header.write_text(original.replace(marker, marker + "\n#include <cstdint>", 1))
    cmake(
        sdk,
        work / "livox",
        install / "livox",
        args.jobs,
        ["-DBUILD_SHARED_LIBS=ON", "-DCMAKE_CXX_FLAGS=-include cstdint"],
    )
    env = dict(os.environ)
    env["CMAKE_PREFIX_PATH"] = ":".join(
        [str(install / "livox"), env.get("CMAKE_PREFIX_PATH", "")]
    )
    env["LD_LIBRARY_PATH"] = ":".join(
        [str(install / "livox/lib"), env.get("LD_LIBRARY_PATH", "")]
    )
    env["MAKEFLAGS"] = "-j" + str(args.jobs)
    staged_fast_livo2 = stage_fast_livo2_source(
        ROOT / "third_party/fast_livo2",
        work / "sources/fast_livo2_stage",
        allow_inside_repo=True,
    )
    base_paths = [
        staged_fast_livo2 if path.name == "fast_livo2" else path
        for path in BASES
    ]
    run(
        [
            "colcon",
            "--log-base",
            work / "logs",
            "build",
            "--merge-install",
            "--executor",
            "sequential",
            "--base-paths",
            *base_paths,
            "--packages-select",
            *PACKAGES,
            "--build-base",
            work / "ros",
            "--install-base",
            install / "ros",
            "--cmake-args",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DBUILD_TESTING=OFF",
            "-DROS_EDITION=ROS2",
            "-DDISTRO_ROS=jazzy",
            "-DV16_VIO_PATCH_THREADS=4",
        ],
        env=env,
        cwd=ROOT,
    )
    cmake(ROOT / "third_party/pct", work / "pct", install / "pct", args.jobs)
    (install / "native-build-sources.json").write_text(json.dumps(plan, indent=2) + "\n")
    packages = subprocess.check_output(
        ["dpkg-query", "-W", "-f=${Package}\t${Version}\n"], text=True
    )
    (install / "native-system-packages.tsv").write_text(packages)
    print("Native dependencies built. This does not validate navigation or simulation.")


if __name__ == "__main__":
    main()
