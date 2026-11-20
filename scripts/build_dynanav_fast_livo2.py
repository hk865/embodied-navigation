#!/usr/bin/env python3
"""Stage and build patched FAST-LIVO2 (dynanav_sensor_msgs, core, ros, optional tests)."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from embodied_navigation.localization_mapping.fast_livo2_source import (
    stage_fast_livo2_source,
)

CORE_TEST_TARGETS = (
    "test_dynanav_lio",
    "test_dynanav_post_vio",
    "test_dynanav_ouster",
    "test_dynanav_sync_observation",
    "test_dynanav_same_pair",
    "test_dynanav_complete_scan",
)
CORE_CTEST_REGEX = "^dynanav_(lio|post_vio|ouster|sync_observation|same_pair|complete_scan)_native$"


def _run(cmd: list[str], env: dict[str, str]) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, env=env)


def _write_capture_message_env(build_root: Path) -> None:
    env_script = build_root / "capture_message_env.sh"
    env_script.write_text(
        "# Source only in the owned runtime environment. No processes launched.\n"
        "source /opt/ros/jazzy/setup.bash\n"
        'source "${ISAAC_SIM_ROOT:-$HOME/isaacsim}/setup_python_env.sh"\n'
        f"PREFIX={shlex.quote(str(build_root))}\n"
        'export PYTHONPATH="$PREFIX/install/ros/lib/python3.12/site-packages:${PYTHONPATH:-}"\n'
        'export LD_LIBRARY_PATH="$PREFIX/install/ros/lib:${LD_LIBRARY_PATH:-}"\n'
        'export AMENT_PREFIX_PATH="$PREFIX/install/ros:${AMENT_PREFIX_PATH:-}"\n'
        "export DYNANAV_RTX_COMPLETE_SCAN=1\n",
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-root", type=Path, required=True, help="New external build directory")
    parser.add_argument("--dependency-prefix", type=Path, required=True, help="ROS install prefix with livox/vikit")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--with-tests", action="store_true", help="Also build and run core dynanav_lio native tests")
    args = parser.parse_args(argv)
    if not 1 <= args.jobs <= 8:
        parser.error("--jobs must be within 1..8")

    build_root = args.build_root.resolve()
    dep_prefix = args.dependency_prefix.resolve()
    install_ros = build_root / "install" / "ros"
    staged = stage_fast_livo2_source(ROOT / "third_party" / "fast_livo2", build_root)

    env = dict(os.environ)
    ld_parts = [str(install_ros / "lib"), str(dep_prefix / "lib")]
    if env.get("LD_LIBRARY_PATH"):
        ld_parts.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = ":".join(ld_parts)
    prefix_path = f"{install_ros};{dep_prefix};/opt/ros/jazzy"

    packages = [
        ("messages", staged / "dynanav_sensor_msgs", True, ()),
        ("core", staged / "fast_livo2_core", True, ()),
        ("ros", staged / "fast_livo2_ros", True, ()),
    ]
    if args.with_tests:
        packages.append(("test", ROOT / "tests" / "native" / "dynanav_lio", False, CORE_TEST_TARGETS))

    for name, src_dir, do_install, targets in packages:
        bld_dir = build_root / "build" / name
        _run(
            [
                "cmake", "-S", str(src_dir), "-B", str(bld_dir),
                "-DCMAKE_BUILD_TYPE=Release",
                f"-DCMAKE_INSTALL_PREFIX={install_ros}",
                f"-DCMAKE_PREFIX_PATH={prefix_path}",
                "-DPython3_EXECUTABLE=/usr/bin/python3",
            ],
            env,
        )
        build_cmd = ["cmake", "--build", str(bld_dir), "--parallel", str(args.jobs)]
        if targets:
            build_cmd.extend(["--target", *targets])
        _run(build_cmd, env)
        if do_install:
            _run(["cmake", "--install", str(bld_dir)], env)

    _write_capture_message_env(build_root)
    if args.with_tests:
        _run(
            [
                "ctest", "--test-dir", str(build_root / "build" / "test"),
                "-R", CORE_CTEST_REGEX, "--no-tests=error", "--output-on-failure",
            ],
            env,
        )

    print(f"Built patched FAST-LIVO2 into {install_ros}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
