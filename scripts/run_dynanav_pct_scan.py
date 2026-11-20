#!/usr/bin/env python3
"""ROS-system-Python entry for the measured-generation PCT + SCAN sidecar."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from embodied_navigation.simulation.dynanav.pct_scan_worker import main

if __name__ == '__main__':
    main()
