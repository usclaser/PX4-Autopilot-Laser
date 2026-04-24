#!/usr/bin/env python3
"""
PX4 keyboard teleop: switch to Manual mode and drive manual_control_setpoint (arrow keys).

Requires ROS 2 (rclpy), px4_msgs, and pynput:
  pip install pynput
  # px4_msgs from a colcon workspace that matches your PX4 version

Before running, source your ROS 2 overlay that provides px4_msgs, e.g.:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source install/setup.bash   # if you built px4_msgs locally

Run:
  python3 px4_keyboard_manual_teleop.py
  # Jetson / SSH / missing X11 RECORD (record_create_context) — TTY input, no pynput:
  TELEOP_USE_TTY=1 python3 px4_keyboard_manual_teleop.py
  python3 px4_keyboard_manual_teleop.py --use-tty

True open-loop (no commanded wrench unless you move sticks / keys):
  • FC must be in PX4 Manual (nav_state MANUAL), not Position/Hold/Mission/Offboard-position.
    If QGC still station-keeps, you are not in Manual — verify nav_state and control_position.
  • In Manual + spacecraft, centered inputs → zero thrust/torque from the manual/direct path.
  • Disarm (D) when not testing so outputs stay in the disarmed state; do not arm until you intend to fire.
  • Avoid a second manual source (QGC virtual stick, RC) fighting this node.

On Linux/Ubuntu terminals, arrow keys send escape sequences; the script disables TTY echo so you
do not see stray characters like ^[A. If anything still prints, run: stty sane

pynput requires the X11 RECORD extension (pynput → python-xlib → record_create_context). That often
fails on Jetson / headless / Wayland / some SSH + X setups. If you see ``AttributeError:
record_create_context`` or similar, use TTY input instead: ``TELEOP_USE_TTY=1`` or ``--use-tty``.
TTY mode uses the same key map (WASD, arrows if your terminal sends VT sequences, M/A/D, etc.) and
does not use X11.

Controls (spacecraft Manual/direct mapping in SpacecraftRateControl):
  Arrow Up/Down     body X thrust (forward / back)
  Arrow Left/Right    body Y thrust (left / right)
  Z / X               yaw torque (left / right)
  M                   send Manual mode (VEHICLE_CMD_DO_SET_MODE)
  A / D               arm / disarm (default: same as `commander arm -f` / normal disarm — see parameters)
  Space               zero all sticks
  Q or Ctrl+C         quit

Arming uses VEHICLE_CMD_COMPONENT_ARM_DISARM with param2=21196 when force_arm is true (PX4
magic value used by `commander arm -f`). from_external is set false in that case so preflight
checks are skipped like the NSH shell command — you cannot run `commander` itself from the host.

Safety: only use with props unpowered or vehicle restrained. You are responsible for arming rules.
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import select
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from pynput import keyboard as pynput_keyboard
except ImportError:
    pynput_keyboard = None  # TTY mode only; optional

from px4_msgs.msg import VehicleCommand, ManualControlSetpoint


# PX4: px4_custom_mode.h — MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, PX4_CUSTOM_MAIN_MODE_MANUAL = 1
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_MANUAL = 1.0

# Commander.cpp: `commander arm -f` / `disarm -f` use param2 == 21196 (force, skip arming checks)
PX4_FORCE_ARM_DISARM_MAGIC = 21196.0


def _tty_echo_off() -> tuple[int, list] | None:
    """Stop the terminal from echoing arrow-key escape sequences (e.g. ^[A on Linux). pynput still receives keys."""
    if os.name != "posix" or not sys.stdin.isatty():
        return None
    import termios

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    # lflags: disable echo (and control-char echo if available)
    new[3] &= ~termios.ECHO
    if hasattr(termios, "ECHOCTL"):
        new[3] &= ~termios.ECHOCTL
    termios.tcsetattr(fd, termios.TCSADRAIN, new)
    return (fd, old)


def _tty_restore(saved: tuple[int, list] | None) -> None:
    if saved is None:
        return
    import termios

    fd, old = saved
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except OSError:
        pass


# TTY: stick inputs use short activity windows (repeat on hold); see _tty_key_loop.
_TTY_ACTIVITY_S = 0.12


def _env_flag(name: str) -> bool:
    v = os.environ.get(name, "").strip().lower()
    return v in ("1", "true", "yes", "y")


class Px4KeyboardManualTeleop(Node):
    def __init__(self) -> None:
        super().__init__("px4_keyboard_manual_teleop")
        self._shutdown_requested = False
        # pynput (X11) vs /dev/tty cbreak (Jetson, SSH, no X11 RECORD / Wayland)
        if (not pynput_keyboard) or _env_flag("TELEOP_USE_TTY"):
            self._use_tty = True
        elif _env_flag("TELEOP_USE_PYNPUT"):
            self._use_tty = False
        else:
            self._use_tty = False  # default: use pynput if installed

        self.declare_parameter("target_system", 1)
        self.declare_parameter("stick_gain", 0.85)  # max deflection [-1, 1]
        self.declare_parameter("cmd_topic_vehicle_command", "/fmu/in/vehicle_command")
        self.declare_parameter("cmd_topic_manual_control", "/fmu/in/manual_control_input")
        self.declare_parameter("publish_rate_hz", 50.0)
        # Match `commander arm -f` (VEHICLE_CMD_COMPONENT_ARM_DISARM + magic param2, from_external=false)
        self.declare_parameter("force_arm", True)
        self.declare_parameter("force_disarm", False)
        # Manual stick throttle when not using throttle for translation [-1, 1]; -1 = full down (no +Z thrust cmd).
        # Avoids NaN so arming checks / consumers see a defined idle input.
        self.declare_parameter("idle_throttle", -1.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_cmd = self.create_publisher(
            VehicleCommand,
            self.get_parameter("cmd_topic_vehicle_command").get_parameter_value().string_value,
            qos,
        )
        self._pub_manual = self.create_publisher(
            ManualControlSetpoint,
            self.get_parameter("cmd_topic_manual_control").get_parameter_value().string_value,
            qos,
        )

        self._target_system = (
            self.get_parameter("target_system").get_parameter_value().integer_value
        )
        self._stick_gain = self.get_parameter("stick_gain").get_parameter_value().double_value
        self._force_arm = self.get_parameter("force_arm").get_parameter_value().bool_value
        self._force_disarm = self.get_parameter("force_disarm").get_parameter_value().bool_value
        self._idle_throttle = self.get_parameter("idle_throttle").get_parameter_value().double_value
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        period = 1.0 / max(rate, 1.0)

        self._lock = threading.Lock()
        self._pressed: set = set()  # pynput: Key/KeyCode
        # TTY: monotonic() timestamp of last activity per logical axis
        self._last_activity: dict[str, float] = {}
        self._event_q: "queue.SimpleQueue[str]" = queue.SimpleQueue()
        self._listener = None
        self._tty_thread: threading.Thread | None = None

        self._timer = self.create_timer(period, self._on_timer)

        if not self._use_tty:
            self._listener = pynput_keyboard.Listener(
                on_press=self._on_pynput_press, on_release=self._on_pynput_release
            )
            self._listener.start()
            msg = (
                "Started (pynput/X11). M=Manual, A=arm (force=%s), D=disarm (force=%s). Arrows+Z/X. Q=quit."
                % (self._force_arm, self._force_disarm)
            )
        else:
            if pynput_keyboard and _env_flag("TELEOP_USE_TTY"):
                self.get_logger().info("Using TTY keyboard (TELEOP_USE_TTY) — set TELEOP_USE_PYNPUT=1 to prefer X11/pynput.")
            elif not pynput_keyboard:
                self.get_logger().info("pynput not installed — using TTY keyboard.")
            self._tty_thread = threading.Thread(target=self._tty_key_loop, name="px4_tty_keys", daemon=True)
            self._tty_thread.start()
            msg = (
                "Started (TTY). M=Manual, A=arm (force=%s), D=disarm (force=%s). "
                "W/S=I/K pitch, J/L roll, arrows, Z/X yaw, Q=quit"
                % (self._force_arm, self._force_disarm)
            )
        self.get_logger().info(msg)

    def destroy_node(self) -> bool:
        self._shutdown_requested = True
        try:
            if self._listener is not None:
                self._listener.stop()
        except Exception:
            pass
        if self._tty_thread is not None and self._tty_thread.is_alive():
            self._tty_thread.join(timeout=1.0)
        return super().destroy_node()

    def _bump_activity(self, name: str) -> None:
        with self._lock:
            self._last_activity[name] = time.monotonic()

    def _tty_key_loop(self) -> None:
        import termios
        import tty

        if not sys.stdin.isatty():
            self.get_logger().error("TTY mode requires an interactive terminal (stdin is not a TTY).")
            return
        fd = sys.stdin.fileno()
        old = None
        try:
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except (termios.error, OSError) as e:
            self.get_logger().error("TTY setcbreak failed: %s" % e)
            return
        buf = bytearray()
        try:
            while not self._shutdown_requested:
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r or self._shutdown_requested:
                    continue
                chunk = os.read(fd, 64)
                if not chunk:
                    break
                buf.extend(chunk)
                while len(buf) > 0 and (not self._shutdown_requested):
                    if buf[0] == 0x1B:
                        if len(buf) < 2:
                            break
                        if buf[1:3] == b"[" and len(buf) < 3:
                            break
                        if buf[1:3] == b"[" and len(buf) >= 3:
                            ch = buf[2]
                            del buf[:3]
                            if ch == 65:  # A
                                self._bump_activity("pitch+")
                            elif ch == 66:  # B
                                self._bump_activity("pitch-")
                            elif ch == 68:  # D
                                self._bump_activity("roll-")
                            elif ch == 67:  # C
                                self._bump_activity("roll+")
                        else:
                            # ESC + non-bracket (e.g. alt combos): drop one byte
                            del buf[0]
                    else:
                        c = bytes([buf[0]]).decode("utf-8", errors="replace")
                        del buf[0:1]
                        c = c.lower()
                        if c in ("q",):
                            self._shutdown_requested = True
                            break
                        if c == "m":
                            self._event_q.put("manual")
                        elif c == "a":
                            self._event_q.put("arm")
                        elif c == "d":
                            self._event_q.put("disarm")
                        elif c == " ":
                            with self._lock:
                                self._last_activity.clear()
                        elif c in ("w", "i"):
                            self._bump_activity("pitch+")
                        elif c in ("s", "k"):
                            self._bump_activity("pitch-")
                        elif c == "j":
                            self._bump_activity("roll-")
                        elif c == "l":
                            self._bump_activity("roll+")
                        elif c == "z":
                            self._bump_activity("yaw-")
                        elif c == "x":
                            self._bump_activity("yaw+")
        finally:
            if old is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
                except OSError:
                    pass

    def _on_pynput_press(
        self, key: pynput_keyboard.Key | pynput_keyboard.KeyCode | None
    ) -> None:
        if pynput_keyboard is None or key is None:
            return
        try:
            if key == pynput_keyboard.Key.esc:
                self._shutdown_requested = True
                raise pynput_keyboard.Listener.StopException()
            if getattr(key, "char", None) in ("q", "Q"):
                self._shutdown_requested = True
                raise pynput_keyboard.Listener.StopException()
        except pynput_keyboard.Listener.StopException:
            raise
        except Exception:
            pass

        if key == pynput_keyboard.Key.space:
            with self._lock:
                self._pressed.clear()
            return

        if hasattr(key, "char") and key.char is not None:
            c = key.char.lower()
            if c == "m":
                self._send_manual_mode()
            elif c == "a":
                self._send_arm(True)
            elif c == "d":
                self._send_arm(False)

        with self._lock:
            self._pressed.add(key)

    def _on_pynput_release(
        self, key: pynput_keyboard.Key | pynput_keyboard.KeyCode | None
    ) -> None:
        if key is None:
            return
        with self._lock:
            self._pressed.discard(key)

    def _sticks_from_keys(self) -> tuple[float, float, float]:
        """Returns (pitch, roll, yaw) in [-1, 1] for ManualControlSetpoint."""
        g = float(self._stick_gain)
        pitch = 0.0
        roll = 0.0
        yaw = 0.0

        if self._use_tty:
            now = time.monotonic()

            def active(name: str) -> bool:
                with self._lock:
                    t0 = self._last_activity.get(name, 0.0)
                return (now - t0) < _TTY_ACTIVITY_S

            if active("pitch+"):
                pitch += g
            if active("pitch-"):
                pitch -= g
            if active("roll-"):
                roll -= g
            if active("roll+"):
                roll += g
            if active("yaw-"):
                yaw -= g
            if active("yaw+"):
                yaw += g
        else:
            if pynput_keyboard is None:
                return 0.0, 0.0, 0.0
            with self._lock:
                keys = set(self._pressed)

            if pynput_keyboard.Key.up in keys:
                pitch += g
            if pynput_keyboard.Key.down in keys:
                pitch -= g
            if pynput_keyboard.Key.left in keys:
                roll -= g
            if pynput_keyboard.Key.right in keys:
                roll += g

            for k in keys:
                if hasattr(k, "char") and k.char is not None:
                    if k.char.lower() == "z":
                        yaw -= g
                    elif k.char.lower() == "x":
                        yaw += g

        def clamp(x: float) -> float:
            return max(-1.0, min(1.0, x))

        return clamp(pitch), clamp(roll), clamp(yaw)

    def _stamp_us(self) -> int:
        return int(time.time() * 1e6) & 0xFFFFFFFFFFFFFFFF

    def _send_manual_mode(self) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._stamp_us()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2 = PX4_CUSTOM_MAIN_MODE_MANUAL
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = int(self._target_system)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self._pub_cmd.publish(msg)
        self.get_logger().info("Sent VEHICLE_CMD_DO_SET_MODE -> Manual")

    def _send_arm(self, arm: bool) -> None:
        """Same mechanism as `commander arm [-f]` / `disarm [-f]` (not subprocess — PX4 NSH only)."""
        force = self._force_arm if arm else self._force_disarm
        msg = VehicleCommand()
        msg.timestamp = self._stamp_us()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 if arm else 0.0
        msg.param2 = PX4_FORCE_ARM_DISARM_MAGIC if force else 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = int(self._target_system)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        # Commander.cpp: arm(..., cmd.from_external || !forced) — for forced arm, from_external must
        # be false to skip preflight (matches internal send_vehicle_command used by NSH `commander`).
        msg.from_external = not force
        self._pub_cmd.publish(msg)
        tag = "ARM" if arm else "DISARM"
        if force:
            tag += " (force, param2=%d)" % int(PX4_FORCE_ARM_DISARM_MAGIC)
        self.get_logger().info("Sent %s" % tag)

    def _on_timer(self) -> None:
        if getattr(self, "_shutdown_requested", False):
            rclpy.shutdown()
            return

        while True:
            try:
                ev = self._event_q.get_nowait()
            except queue.Empty:
                break
            if ev == "manual":
                self._send_manual_mode()
            elif ev == "arm":
                self._send_arm(True)
            elif ev == "disarm":
                self._send_arm(False)

        pitch, roll, yaw = self._sticks_from_keys()

        out = ManualControlSetpoint()
        out.timestamp = self._stamp_us()
        out.timestamp_sample = out.timestamp
        out.valid = True
        out.data_source = ManualControlSetpoint.SOURCE_UNKNOWN
        out.pitch = float(pitch)
        out.roll = float(roll)
        out.yaw = float(yaw)
        out.throttle = float(self._idle_throttle)
        out.flaps = float("nan")
        for name in ("aux1", "aux2", "aux3", "aux4", "aux5", "aux6"):
            setattr(out, name, float("nan"))
        out.sticks_moving = bool(
            math.hypot(pitch, roll) > 0.05 or abs(yaw) > 0.05
        )
        out.buttons = 0

        self._pub_manual.publish(out)


def main() -> None:
    ap = argparse.ArgumentParser(description="PX4 keyboard → manual_control_setpoint (ROS 2)")
    ap.add_argument(
        "--use-tty",
        action="store_true",
        help="Input via terminal (cbreak) instead of pynput; use on Jetson/SSH or if X11 RECORD is missing (record_create_context).",
    )
    args, _ = ap.parse_known_args()
    if args.use_tty:
        os.environ["TELEOP_USE_TTY"] = "1"
    # Echo off helps pynput + terminal; TTY mode uses cbreak in the key thread
    tty_saved: tuple[int, list] | None = _tty_echo_off() if not _env_flag("TELEOP_USE_TTY") else None
    rclpy.init()
    node = None
    try:
        node = Px4KeyboardManualTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        _tty_restore(tty_saved)


if __name__ == "__main__":
    main()
