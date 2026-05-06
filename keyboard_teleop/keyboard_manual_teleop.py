#!/usr/bin/env python3
"""
Launcher script for keyboard teleop.

Run:
  python3 keyboard_teleop/keyboard_manual_teleop.py [--use-tty]
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys


def main() -> None:
    # Make sibling modules importable (node.py, csv_logger.py).
    this_dir = pathlib.Path(__file__).resolve().parent
    sys.path.insert(0, str(this_dir))

    ap = argparse.ArgumentParser(description="Keyboard teleop → manual_control_setpoint (ROS 2)")
    ap.add_argument(
        "--use-tty",
        action="store_true",
        help="Input via terminal (cbreak) instead of pynput; use on Jetson/SSH or if X11 RECORD is missing (record_create_context).",
    )
    args, _ = ap.parse_known_args()
    if args.use_tty:
        os.environ["TELEOP_USE_TTY"] = "1"

    from node import run

    run()


if __name__ == "__main__":
    main()

