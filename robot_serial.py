import csv
import time
from dataclasses import dataclass, field

import serial

from robot_config import ControlConfig, SerialConfig


COMMAND_CODES = {
    "forward": "F",
    "backward": "B",
    "left": "L",
    "right": "R",
}


@dataclass
class ImuTelemetry:
    timestamp_ms: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    yaw: float


@dataclass
class CalibrationStatus:
    timestamp_ms: int
    state: str
    reason: str | None = None
    gz_bias: float | None = None
    samples: int | None = None
    accel_axis_range: float | None = None
    accel_mag_range: float | None = None
    raw_values: list[str] = field(default_factory=list)


@dataclass
class RobotTelemetry:
    distance_cm: float = 100.0
    last_raw_line: str = ""
    last_imu: ImuTelemetry | None = None
    last_calibration: CalibrationStatus | None = None


class RobotConnectionError(Exception):
    pass


class RobotCalibrationError(Exception):
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

    def calibrate_imu(
        self,
        timeout_seconds: float = 10.0,
        settle_seconds: float = 1.0,
    ) -> CalibrationStatus:
        self.telemetry.last_calibration = None
        self.stop()
        settle_deadline = time.monotonic() + settle_seconds
        while time.monotonic() < settle_deadline:
            self.poll_telemetry()
            time.sleep(0.05)
        self._write("C")

        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self.poll_telemetry()
            status = self.telemetry.last_calibration
            if status is None:
                time.sleep(0.02)
                continue
            if status.state == "ok":
                self._last_sent_command = ""
                return status
            if status.state == "failed":
                reason = status.reason or "calibration failed"
                raise RobotCalibrationError(reason)
            time.sleep(0.02)

        raise RobotCalibrationError("timed out waiting for calibration result")

    def read_distance(self) -> float:
        self.poll_telemetry()
        return self.telemetry.distance_cm

    def poll_telemetry(self) -> None:
        while self._serial.in_waiting > 0:
            line = self._serial.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            self._handle_serial_line(line)

    def _handle_serial_line(self, line: str) -> None:
        self.telemetry.last_raw_line = line
        try:
            fields = next(csv.reader([line]))
        except csv.Error:
            return

        fields = [field.strip() for field in fields]
        if len(fields) < 2:
            return

        try:
            timestamp_ms = int(fields[0])
        except ValueError:
            return

        data_type = fields[1].lower()
        values = fields[2:]
        if data_type == "distance":
            self._handle_distance_record(timestamp_ms, values)
        elif data_type == "imu":
            self._handle_imu_record(timestamp_ms, values)
        elif data_type == "status":
            self._handle_status_record(timestamp_ms, values)

    def _handle_distance_record(self, timestamp_ms: int, values: list[str]) -> None:
        del timestamp_ms
        if len(values) != 1:
            return

        try:
            self.telemetry.distance_cm = float(values[0])
        except ValueError:
            return

    def _handle_imu_record(self, timestamp_ms: int, values: list[str]) -> None:
        if len(values) != 7:
            return

        try:
            ax, ay, az, gx, gy, gz, yaw = (float(value) for value in values)
        except ValueError:
            return

        self.telemetry.last_imu = ImuTelemetry(
            timestamp_ms=timestamp_ms,
            ax=ax,
            ay=ay,
            az=az,
            gx=gx,
            gy=gy,
            gz=gz,
            yaw=yaw,
        )

    def _handle_status_record(self, timestamp_ms: int, values: list[str]) -> None:
        if len(values) < 2 or values[0].lower() != "calibration":
            return

        state = values[1].lower()
        extras = values[2:]
        reason = None
        if state == "failed" and extras:
            reason = extras[0].lower()
            extras = extras[1:]

        details = self._parse_key_value_fields(extras)
        self.telemetry.last_calibration = CalibrationStatus(
            timestamp_ms=timestamp_ms,
            state=state,
            reason=reason,
            gz_bias=self._optional_float(details.get("gz_bias")),
            samples=self._optional_int(details.get("samples")),
            accel_axis_range=self._optional_float(details.get("accel_axis_range")),
            accel_mag_range=self._optional_float(details.get("accel_mag_range")),
            raw_values=values,
        )

    def _parse_key_value_fields(self, values: list[str]) -> dict[str, str]:
        details: dict[str, str] = {}
        for index in range(0, len(values) - 1, 2):
            details[values[index].lower()] = values[index + 1]
        return details

    def _optional_float(self, value: str | None) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    def _optional_int(self, value: str | None) -> int | None:
        if value is None:
            return None
        try:
            return int(value)
        except ValueError:
            return None

    def _write(self, command: str) -> None:
        self._serial.write(f"{command}\n".encode("utf-8"))
