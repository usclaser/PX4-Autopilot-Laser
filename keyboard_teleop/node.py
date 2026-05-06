#!/usr/bin/env python3
"""
Keyboard teleop (ROS 2): publish ManualControlSetpoint + arm/mode commands.

Publishes:
  - /fmu/in/manual_control_input  (px4_msgs/ManualControlSetpoint)
  - /fmu/in/vehicle_command       (px4_msgs/VehicleCommand) for Arm/Disarm/Manual mode
"""

from __future__ import annotations

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

from px4_msgs.msg import ManualControlSetpoint, SensorCombined, VehicleCommand, VehicleOdometry

from csv_logger import CsvLogger, CsvLoggerConfig


# PX4: px4_custom_mode.h — MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, PX4_CUSTOM_MAIN_MODE_MANUAL = 1
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_MANUAL = 1.0

# Commander.cpp: `commander arm -f` / `disarm -f` use param2 == 21196 (force, skip arming checks)
PX4_FORCE_ARM_DISARM_MAGIC = 21196.0

# TTY: stick inputs use short activity windows (repeat on hold); see _tty_key_loop.
_TTY_ACTIVITY_S = 0.12


def _env_flag(name: str) -> bool:
    v = os.environ.get(name, "").strip().lower()
    return v in ("1", "true", "yes", "y")


def _tty_echo_off() -> tuple[int, list] | None:
    """Stop the terminal from echoing arrow-key escape sequences."""
    if os.name != "posix" or not sys.stdin.isatty():
        return None
    import termios

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
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


class KeyboardManualTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_manual_teleop")
        self._shutdown_requested = False

        if (not pynput_keyboard) or _env_flag("TELEOP_USE_TTY"):
            self._use_tty = True
        elif _env_flag("TELEOP_USE_PYNPUT"):
            self._use_tty = False
        else:
            self._use_tty = False

        self.declare_parameter("target_system", 1)
        self.declare_parameter("stick_gain", 0.85)  # max deflection [-1, 1]
        self.declare_parameter("cmd_topic_vehicle_command", "/fmu/in/vehicle_command")
        self.declare_parameter("cmd_topic_manual_control", "/fmu/in/manual_control_input")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("force_arm", True)
        self.declare_parameter("force_disarm", False)
        self.declare_parameter("idle_throttle", 0.0)
        self.declare_parameter("manual_data_source", 2)
        self.declare_parameter("log_input_events", True)
        self.declare_parameter("log_manual_sticks_s", 0.0)
        # CSV logging (one row per tick)
        self.declare_parameter("log_csv", True)
        self.declare_parameter("log_csv_path", "")

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

        # Subscriptions for logging/alignment (latest-sample latch)
        self._sub_sensor_combined = self.create_subscription(
            SensorCombined, "/fmu/out/sensor_combined", self._on_sensor_combined, qos
        )
        self._sub_vehicle_odometry = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._on_vehicle_odometry, qos
        )
        self._last_sensor_combined: SensorCombined | None = None
        self._last_vehicle_odometry: VehicleOdometry | None = None

        self._target_system = (
            self.get_parameter("target_system").get_parameter_value().integer_value
        )
        self._stick_gain = self.get_parameter("stick_gain").get_parameter_value().double_value
        self._force_arm = self.get_parameter("force_arm").get_parameter_value().bool_value
        self._force_disarm = self.get_parameter("force_disarm").get_parameter_value().bool_value
        self._idle_throttle = self.get_parameter("idle_throttle").get_parameter_value().double_value
        _ds = self.get_parameter("manual_data_source").get_parameter_value().integer_value
        self._manual_data_source = int(_ds) if 0 <= int(_ds) < 8 else 2
        self._log_on_zero = self.get_parameter("log_input_events").get_parameter_value().bool_value
        self._log_manual_debug_iv = (
            self.get_parameter("log_manual_sticks_s").get_parameter_value().double_value
        )
        self._last_debug_log_monotonic = 0.0

        log_csv = self.get_parameter("log_csv").get_parameter_value().bool_value
        log_csv_path = self.get_parameter("log_csv_path").get_parameter_value().string_value
        self._csv = CsvLogger(CsvLoggerConfig(enabled=bool(log_csv), path=log_csv_path))
        if log_csv:
            try:
                path = self._csv.open()
                if path is not None:
                    self.get_logger().info(f"CSV logging enabled: {path}")
            except Exception as e:
                self.get_logger().error(f"Failed to init CSV logger: {e}")

        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        period = 1.0 / max(rate, 1.0)

        self._lock = threading.Lock()
        self._pressed: set = set()  # pynput: Key/KeyCode
        self._last_activity: dict[str, float] = {}  # TTY: monotonic() timestamp of last activity per axis
        self._event_q: "queue.SimpleQueue[str]" = queue.SimpleQueue()
        self._listener = None
        self._tty_thread: threading.Thread | None = None

        self._timer = self.create_timer(period, self._on_timer)

        msg = ""

        want_pynput = (not self._use_tty) and (pynput_keyboard is not None)
        if want_pynput:
            listener = pynput_keyboard.Listener(
                on_press=self._on_pynput_press, on_release=self._on_pynput_release
            )
            listener.start()
            # RECORD extension required; crashes in background thread on many Jetsons / Wayland / minimal X
            time.sleep(0.35)
            backend_thread = getattr(listener, "_thread", None)
            if backend_thread is not None and backend_thread.is_alive():
                self._listener = listener
                msg = (
                    "Started (pynput/X11). M=Manual, A=arm (force=%s), D=disarm (force=%s). "
                    "Arrows+Z/X. Q=quit."
                    % (self._force_arm, self._force_disarm)
                )
            else:
                try:
                    listener.stop()
                except Exception:
                    pass
                self._listener = None
                self._use_tty = True
                self.get_logger().warning(
                    "pynput/X11 RECORD is not available on this session (Jetson/minimal-X/Wayland). "
                    "Falling back to TTY keyboard. Use: python keyboard_manual_teleop.py --use-tty "
                    "or TELEOP_USE_TTY=1 from an interactive terminal."
                )

        if self._use_tty:
            if pynput_keyboard and _env_flag("TELEOP_USE_TTY"):
                self.get_logger().info(
                    "Using TTY keyboard (TELEOP_USE_TTY) — set TELEOP_USE_PYNPUT=1 to prefer X11/pynput."
                )
            elif not pynput_keyboard:
                self.get_logger().info("pynput not installed — using TTY keyboard.")
            self._tty_thread = threading.Thread(
                target=self._tty_key_loop, name="keyboard_tty_keys", daemon=True
            )
            self._tty_thread.start()
            msg = (
                "Started (TTY). M=Manual, A=arm (force=%s), D=disarm (force=%s). "
                "W/S=I/K pitch, J/L roll, arrows, Z/X yaw, Q=quit"
                % (self._force_arm, self._force_disarm)
            )

        self.get_logger().info(msg)
        self.get_logger().info(
            "ManualControl data_source=%d (MAVLink_0=2). If arming still moves solenoids from "
            "another stick, set FC param COM_RC_IN_MODE=1 (Mavlink only) or power RC after this node "
            "(COM_RC_IN_MODE=3 uses the first source until reboot, often RC)."
            % (self._manual_data_source,)
        )

    def destroy_node(self) -> bool:
        self._shutdown_requested = True
        try:
            if self._listener is not None:
                self._listener.stop()
        except Exception:
            pass
        if self._tty_thread is not None and self._tty_thread.is_alive():
            self._tty_thread.join(timeout=1.0)
        self._csv.close()
        return super().destroy_node()

    def _on_sensor_combined(self, msg: SensorCombined) -> None:
        self._last_sensor_combined = msg

    def _on_vehicle_odometry(self, msg: VehicleOdometry) -> None:
        self._last_vehicle_odometry = msg

    def _bump_activity(self, name: str) -> None:
        with self._lock:
            self._last_activity[name] = time.monotonic()

    def _tty_key_loop(self) -> None:
        import termios
        import tty

        if not sys.stdin.isatty():
            self.get_logger().error(
                "TTY mode requires an interactive terminal (stdin is not a TTY)."
            )
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
                        if buf[1] == 0x5B:  # [
                            if len(buf) < 3:
                                break
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
                            self._event_q.put("zero_sticks")
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
        self, key: "pynput_keyboard.Key | pynput_keyboard.KeyCode | None"
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
            self._event_q.put("zero_sticks")
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
        self, key: "pynput_keyboard.Key | pynput_keyboard.KeyCode | None"
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
        msg.from_external = not force
        self._pub_cmd.publish(msg)
        tag = "ARM" if arm else "DISARM"
        if force:
            tag += " (force, param2=%d)" % int(PX4_FORCE_ARM_DISARM_MAGIC)
        self.get_logger().info("Sent %s" % tag)

    def _maybe_log_row(self, manual: ManualControlSetpoint) -> None:
        if not self._csv.enabled:
            return

        imu = self._last_sensor_combined
        odom = self._last_vehicle_odometry

        def _f(x: float) -> float:
            return float(x)

        row = {
            "manual_timestamp_us": int(manual.timestamp),
            "manual_pitch": _f(manual.pitch),
            "manual_roll": _f(manual.roll),
            "manual_yaw": _f(manual.yaw),
            "manual_throttle": _f(manual.throttle),
            "manual_data_source": int(manual.data_source),
            "manual_sticks_moving": int(bool(manual.sticks_moving)),
            # proxies (normalized)
            "cmd_force_x": float(manual.pitch),
            "cmd_force_y": float(manual.roll),
            "cmd_force_z": 0.0,
            "cmd_torque_x": 0.0,
            "cmd_torque_y": 0.0,
            "cmd_torque_z": float(manual.yaw),
            # IMU
            "imu_timestamp_us": int(imu.timestamp) if imu is not None else 0,
            "imu_gyro_rad_s_x": _f(imu.gyro_rad[0]) if imu is not None else float("nan"),
            "imu_gyro_rad_s_y": _f(imu.gyro_rad[1]) if imu is not None else float("nan"),
            "imu_gyro_rad_s_z": _f(imu.gyro_rad[2]) if imu is not None else float("nan"),
            "imu_accel_m_s2_x": _f(imu.accelerometer_m_s2[0]) if imu is not None else float("nan"),
            "imu_accel_m_s2_y": _f(imu.accelerometer_m_s2[1]) if imu is not None else float("nan"),
            "imu_accel_m_s2_z": _f(imu.accelerometer_m_s2[2]) if imu is not None else float("nan"),
            # Odom
            "odom_timestamp_us": int(odom.timestamp) if odom is not None else 0,
            "odom_position_x": _f(odom.position[0]) if odom is not None else float("nan"),
            "odom_position_y": _f(odom.position[1]) if odom is not None else float("nan"),
            "odom_position_z": _f(odom.position[2]) if odom is not None else float("nan"),
            "odom_velocity_x": _f(odom.velocity[0]) if odom is not None else float("nan"),
            "odom_velocity_y": _f(odom.velocity[1]) if odom is not None else float("nan"),
            "odom_velocity_z": _f(odom.velocity[2]) if odom is not None else float("nan"),
            "odom_angular_velocity_x": _f(odom.angular_velocity[0]) if odom is not None else float("nan"),
            "odom_angular_velocity_y": _f(odom.angular_velocity[1]) if odom is not None else float("nan"),
            "odom_angular_velocity_z": _f(odom.angular_velocity[2]) if odom is not None else float("nan"),
        }

        self._csv.write_row(row)

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
            elif ev == "zero_sticks":
                if self._log_on_zero:
                    self.get_logger().info(
                        "Zeroed manual sticks (next publishes p/r/y=0 at timer rate)"
                    )

        pitch, roll, yaw = self._sticks_from_keys()

        out = ManualControlSetpoint()
        out.timestamp = self._stamp_us()
        out.timestamp_sample = out.timestamp
        out.valid = True
        out.data_source = self._manual_data_source
        out.pitch = float(pitch)
        out.roll = float(roll)
        out.yaw = float(yaw)
        out.throttle = float(self._idle_throttle)
        out.flaps = float("nan")
        for name in ("aux1", "aux2", "aux3", "aux4", "aux5", "aux6"):
            setattr(out, name, float("nan"))
        out.sticks_moving = bool(math.hypot(pitch, roll) > 0.05 or abs(yaw) > 0.05)
        out.buttons = 0

        self._pub_manual.publish(out)
        self._maybe_log_row(out)

        iv = float(self._log_manual_debug_iv)
        if iv > 0.0:
            nowm = time.monotonic()
            if (nowm - self._last_debug_log_monotonic) >= iv:
                self._last_debug_log_monotonic = nowm
                self.get_logger().info(
                    "Published ManualControl: pitch=%.3f roll=%.3f yaw=%.3f thr=%.3f (open-loop; "
                    "see /fmu/out/manual_control_setpoint on FC to compare after selector)"
                    % (pitch, roll, yaw, float(self._idle_throttle))
                )


def run() -> None:
    tty_saved: tuple[int, list] | None = (
        _tty_echo_off() if not _env_flag("TELEOP_USE_TTY") else None
    )
    rclpy.init()
    node: KeyboardManualTeleop | None = None
    try:
        node = KeyboardManualTeleop()
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

