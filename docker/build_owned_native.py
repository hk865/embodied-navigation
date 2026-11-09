#!/usr/bin/env python3
"""Build the three own native modules; source/headers come only from this clone."""

from pathlib import Path
import argparse, os, subprocess

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument("--execute", action="store_true")
p.add_argument("--jobs", type=int, default=2)
a = p.parse_args()
modules = [
    ("localization_mapping/backend/native", "backend"),
    ("simulation/actuators", "actuators"),
    ("common/communication", "communication"),
    ("simulation/gazebo/robogauge/motor_src", "robogauge"),
]
for relative, name in modules:
    source = ROOT / "src/embodied_navigation" / relative
    build = ROOT / "build/owned" / name
    prefix = ROOT / "install" / name
    commands = [
        [
            "cmake",
            "-S",
            str(source),
            "-B",
            str(build),
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
        ],
        ["cmake", "--build", str(build), "--parallel", str(a.jobs)],
        ["cmake", "--install", str(build)],
    ]
    for command in commands:
        print("+", " ".join(command), flush=True)
        if a.execute:
            subprocess.run(command, check=True)
