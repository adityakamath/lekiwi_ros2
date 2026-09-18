#!/usr/bin/env python3
"""Source-checkout entry point; no sourced ROS environment required."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_mujoco.benchmark_mujoco import main  # noqa: E402

if __name__ == '__main__':
    main()
