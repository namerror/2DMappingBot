import time
from dataclasses import dataclass

import serial

from robot_config import ControlConfig, SerialConfig


COMMAND_CODES = {
    "forward": "F",
    "backward": "B",
    "left": "L",
    "right": "R",
}


@dataclass
class RobotTelemetry:
    distance_cm: float = 100.0
    last_raw_line: str = ""


class RobotConnectionError(Exception):
    pass


class RobotController:
    def __init__(self, serial_config: SerialConfig, control_config: ControlConfig) -> None:
        self.serial_config = serial_config
        self.control_config = control_config
        self.telemetry = RobotTelemetry()
        self._last_sent_command = ""
        self._last_send_time = 0.0
        if not hasattr(serial, "Serial"):
            raise RobotConnectionError(
                "The installed 'serial' module is not pyserial. "
                "Run 'pip uninstall serial' and keep 'pyserial' installed."
            )

        try:
            self._serial = serial.Serial(
                serial_config.port,
                serial_config.baud_rate,
                timeout=serial_config.timeout_seconds,
            )
        except serial.SerialException as exc:
            raise RobotConnectionError(str(exc)) from exc

    @property
    def port(self) -> str:
        return self.serial_config.port

    def close(self) -> None:
        if self._serial.is_open:
            self.stop()
            self._serial.close()

    def set_speed(self, pwm_percent: int) -> None:
        pwm = max(0, min(100, int(pwm_percent)))
        self._write(f"V{pwm}")

    def brake(self) -> None:
        self._write(f"K{self.control_config.brake_duration_ms}")

    def stop(self) -> None:
        self._write("X")
        self._last_sent_command = ""

    def hold_drive(self, action: str) -> None:
        command_code = COMMAND_CODES.get(action)
        if command_code is None:
            self.stop()
            return

        command = f"{command_code}{self.control_config.command_duration_ms}"
        now = time.monotonic()
        interval_seconds = self.control_config.command_interval_ms / 1000.0
        if command != self._last_sent_command or now - self._last_send_time >= interval_seconds:
            self._write(command)
            self._last_sent_command = command
            self._last_send_time = now

    def read_distance(self) -> float:
        while self._serial.in_waiting > 0:
            line = self._serial.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            self.telemetry.last_raw_line = line
            try:
                self.telemetry.distance_cm = float(line)
            except ValueError:
                continue

        return self.telemetry.distance_cm

    def _write(self, command: str) -> None:
        self._serial.write(f"{command}\n".encode("utf-8"))
