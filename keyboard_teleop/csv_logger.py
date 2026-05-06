from __future__ import annotations

import csv
import pathlib
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class CsvLoggerConfig:
    enabled: bool = True
    path: str = ""


class CsvLogger:
    def __init__(self, config: CsvLoggerConfig) -> None:
        self._enabled = bool(config.enabled)
        self._path = str(config.path or "")
        self._fp: Any | None = None
        self._writer: csv.DictWriter | None = None
        self._seq = 0

    @property
    def enabled(self) -> bool:
        return self._enabled and self._writer is not None

    def open(self) -> pathlib.Path | None:
        if not self._enabled:
            return None

        if self._path.strip():
            path = pathlib.Path(self._path).expanduser()
        else:
            ts = time.strftime("%Y%m%d_%H%M%S")
            outputs_dir = pathlib.Path(__file__).resolve().parent / "outputs"
            path = outputs_dir / f"keyboard_teleop_log_{ts}.csv"

        path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = open(path, "w", newline="")

        fieldnames = [
            # log row
            "seq",
            "host_time_s",
            "host_mono_s",
            # what we publish (manual input)
            "manual_timestamp_us",
            "manual_pitch",
            "manual_roll",
            "manual_yaw",
            "manual_throttle",
            "manual_data_source",
            "manual_sticks_moving",
            # derived "twist" proxy (what spacecraft Manual/direct turns this into)
            "cmd_force_x",
            "cmd_force_y",
            "cmd_force_z",
            "cmd_torque_x",
            "cmd_torque_y",
            "cmd_torque_z",
            # IMU (SensorCombined); imu_have_msg=0 means IMU floats are NaN (no DDS sample latched yet)
            "imu_have_msg",
            "imu_timestamp_us",
            "imu_gyro_rad_s_x",
            "imu_gyro_rad_s_y",
            "imu_gyro_rad_s_z",
            "imu_accel_m_s2_x",
            "imu_accel_m_s2_y",
            "imu_accel_m_s2_z",
            # Odometry (VehicleOdometry)
            "odom_have_msg",
            "odom_timestamp_us",
            "odom_position_x",
            "odom_position_y",
            "odom_position_z",
            "odom_velocity_x",
            "odom_velocity_y",
            "odom_velocity_z",
            "odom_angular_velocity_x",
            "odom_angular_velocity_y",
            "odom_angular_velocity_z",
        ]

        self._writer = csv.DictWriter(self._fp, fieldnames=fieldnames)
        self._writer.writeheader()
        self._fp.flush()
        return path

    def close(self) -> None:
        try:
            if self._fp is not None:
                try:
                    self._fp.flush()
                except Exception:
                    pass
                try:
                    self._fp.close()
                except Exception:
                    pass
        finally:
            self._fp = None
            self._writer = None

    def write_row(self, row: dict[str, Any]) -> None:
        if not self.enabled:
            return

        row = dict(row)
        row["seq"] = self._seq
        row.setdefault("host_time_s", time.time())
        row.setdefault("host_mono_s", time.monotonic())

        self._writer.writerow(row)  # type: ignore[union-attr]
        self._seq += 1

        if self._seq % 10 == 0 and self._fp is not None:
            self._fp.flush()

