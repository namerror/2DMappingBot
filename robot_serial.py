import csv
import time
from dataclasses import dataclass, field

import serial

from robot_config import ControlConfig, SerialConfig
from session_logger import SessionLogger


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
    robot_seq: int | None = None


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
    robot_seq: int | None = None


@dataclass
class RobotTelemetry:
    distance_cm: float = 100.0
    last_raw_line: str = ""
    last_imu: ImuTelemetry | None = None
    imu_samples: list[ImuTelemetry] = field(default_factory=list)
    last_imu_status: str | None = None
    last_calibration: CalibrationStatus | None = None


class RobotConnectionError(Exception):
    pass


class RobotCalibrationError(Exception):
    pass


SERIAL_RECORD_TYPES = {"distance", "imu", "status"}


class RobotController:
    def __init__(
        self,
        serial_config: SerialConfig,
        control_config: ControlConfig,
        logger: SessionLogger | None = None,
    ) -> None:
        self.serial_config = serial_config
        self.control_config = control_config
        self.logger = logger
        self.telemetry = RobotTelemetry()
        self._last_sent_command = ""
        self._last_send_time = 0.0
        self._serial_rx_buffer = ""
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

    def set_imu_required(self, required: bool, settle_seconds: float = 1.0) -> None:
        settle_deadline = time.monotonic() + settle_seconds
        while time.monotonic() < settle_deadline:
            self.poll_telemetry()
            time.sleep(0.05)
        self._write("I1" if required else "I0")

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

    def drain_imu_samples(self) -> list[ImuTelemetry]:
        samples = self.telemetry.imu_samples
        self.telemetry.imu_samples = []
        return samples

    def poll_telemetry(self) -> None:
        while self._serial.in_waiting > 0:
            chunk = self._serial.read(self._serial.in_waiting).decode("utf-8", errors="ignore")
            if not chunk:
                continue
            self._serial_rx_buffer += chunk
            while "\n" in self._serial_rx_buffer:
                line, self._serial_rx_buffer = self._serial_rx_buffer.split("\n", 1)
                line = line.rstrip("\r").strip()
                if line:
                    self._handle_serial_line(line)

    def _handle_serial_line(self, line: str) -> None:
        self.telemetry.last_raw_line = line
        self._log_event("serial", "rx_line", raw_line=line)
        try:
            fields = next(csv.reader([line]))
        except csv.Error:
            self._log_parse_failure("csv_error", line)
            return

        fields = [field.strip() for field in fields]
        header = self._parse_record_header(fields, line)
        if header is None:
            return

        timestamp_ms, robot_seq, data_type, values = header
        if data_type == "distance":
            payload, reason = self._handle_distance_record(values)
        elif data_type == "imu":
            payload, reason = self._handle_imu_record(timestamp_ms, robot_seq, values)
        elif data_type == "status":
            payload, reason = self._handle_status_record(timestamp_ms, robot_seq, values)
        else:
            payload, reason = None, "unknown_type"

        if payload is None:
            self._log_parse_failure(reason or "invalid_record", line, fields, timestamp_ms, robot_seq)
            return

        self._log_event(
            "serial",
            f"parsed_{data_type}",
            payload,
            robot_timestamp_ms=timestamp_ms,
            robot_seq=robot_seq,
            raw_line=line,
        )

    def _parse_record_header(
        self,
        fields: list[str],
        raw_line: str,
    ) -> tuple[int, int | None, str, list[str]] | None:
        if len(fields) < 2:
            self._log_parse_failure("too_few_fields", raw_line, fields)
            return None

        try:
            timestamp_ms = int(fields[0])
        except ValueError:
            self._log_parse_failure("invalid_timestamp", raw_line, fields)
            return None

        old_data_type = fields[1].lower()
        if old_data_type in SERIAL_RECORD_TYPES:
            return timestamp_ms, None, old_data_type, fields[2:]

        if len(fields) >= 3 and fields[2].lower() in SERIAL_RECORD_TYPES:
            try:
                robot_seq = int(fields[1])
            except ValueError:
                self._log_parse_failure("invalid_robot_seq", raw_line, fields, timestamp_ms)
                return None
            return timestamp_ms, robot_seq, fields[2].lower(), fields[3:]

        self._log_parse_failure("unknown_type", raw_line, fields, timestamp_ms)
        return None

    def _handle_distance_record(self, values: list[str]) -> tuple[dict[str, float] | None, str | None]:
        if len(values) != 1:
            return None, "distance_field_count"

        try:
            distance_cm = float(values[0])
        except ValueError:
            return None, "invalid_distance"

        self.telemetry.distance_cm = distance_cm
        return {"distance_cm": distance_cm}, None

    def _handle_imu_record(
        self,
        timestamp_ms: int,
        robot_seq: int | None,
        values: list[str],
    ) -> tuple[dict[str, float] | None, str | None]:
        if len(values) != 7:
            return None, "imu_field_count"

        try:
            ax, ay, az, gx, gy, gz, yaw = (float(value) for value in values)
        except ValueError:
            return None, "invalid_imu"

        imu = ImuTelemetry(
            timestamp_ms=timestamp_ms,
            ax=ax,
            ay=ay,
            az=az,
            gx=gx,
            gy=gy,
            gz=gz,
            yaw=yaw,
            robot_seq=robot_seq,
        )
        self.telemetry.last_imu = imu
        self.telemetry.imu_samples.append(imu)
        return {
            "ax": ax,
            "ay": ay,
            "az": az,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "yaw": yaw,
        }, None

    def _handle_status_record(
        self,
        timestamp_ms: int,
        robot_seq: int | None,
        values: list[str],
    ) -> tuple[dict[str, object] | None, str | None]:
        if len(values) < 2:
            return None, "status_field_count"

        status_type = values[0].lower()
        if status_type == "imu":
            self.telemetry.last_imu_status = values[1].lower()
            return {
                "status_type": status_type,
                "state": self.telemetry.last_imu_status,
                "values": values,
            }, None
        if status_type != "calibration":
            return None, "unknown_status_type"

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
            robot_seq=robot_seq,
        )
        return {
            "status_type": status_type,
            "state": state,
            "reason": reason,
            "details": details,
            "values": values,
        }, None

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
        self._log_event("host", "tx_command", {"command": command}, raw_line=command)

    def _log_parse_failure(
        self,
        reason: str,
        raw_line: str,
        fields: list[str] | None = None,
        robot_timestamp_ms: int | None = None,
        robot_seq: int | None = None,
    ) -> None:
        self._log_event(
            "serial",
            "parse_failure",
            {
                "reason": reason,
                "fields": fields or [],
            },
            robot_timestamp_ms=robot_timestamp_ms,
            robot_seq=robot_seq,
            raw_line=raw_line,
        )

    def _log_event(
        self,
        source: str,
        event_type: str,
        payload: dict[str, object] | None = None,
        robot_timestamp_ms: int | None = None,
        robot_seq: int | None = None,
        raw_line: str = "",
    ) -> None:
        if self.logger is None:
            return
        self.logger.log_event(
            source,
            event_type,
            payload,
            robot_timestamp_ms=robot_timestamp_ms,
            robot_seq=robot_seq,
            raw_line=raw_line,
        )
