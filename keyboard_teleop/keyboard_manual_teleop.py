#!/usr/bin/env python3
"""
Launcher script for keyboard teleop.

Run:
  python keyboard_manual_teleop.py
  python keyboard_manual_teleop.py --use-tty
  python keyboard_manual_teleop.py --use-x11-pynput   # requires X11 XRECORD (often missing on Jetson)
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
        help="Force TTY/cbreak input (interactive terminal or ssh -t).",
    )
    ap.add_argument(
        "--use-x11-pynput",
        action="store_true",
        help="Use pynput global keyboard (needs X11 XRECORD). Default is TTY to avoid Jetson record_create_context errors.",
    )
    args, _ = ap.parse_known_args()
    if args.use_x11_pynput:
        os.environ["TELEOP_USE_PYNPUT"] = "1"
        os.environ.pop("TELEOP_USE_TTY", None)
    elif args.use_tty:
        os.environ["TELEOP_USE_TTY"] = "1"
        os.environ.pop("TELEOP_USE_PYNPUT", None)

    from node import run

    run()


if __name__ == "__main__":
    main()

