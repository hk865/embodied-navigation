#!/usr/bin/env python3
"""Start/check the assembled navigation system; see --help before running."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'src'))
from embodied_navigation.launch.main import main
if __name__=='__main__':raise SystemExit(main())
