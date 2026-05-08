import csv
import json
import tempfile
import unittest
from pathlib import Path

from pose_estimator import PoseEstimator
from robot_config import ControlConfig, ImuEstimateConfig, PoseConfig, SerialConfig
from robot_serial import RobotController, RobotTelemetry
from session_logger import SessionLogger


class RecordingLogger:
    def __init__(self) -> None:
        self.events: list[dict[str, object]] = []

    def log_event(
        self,
        source: str,
        event_type: str,
        payload: dict[str, object] | None = None,
        robot_timestamp_ms: int | None = None,
        robot_seq: int | None = None,
        raw_line: str = "",
    ) -> None:
        self.events.append(
            {
                "source": source,
                "event_type": event_type,
                "payload": payload or {},
                "robot_timestamp_ms": robot_timestamp_ms,
                "robot_seq": robot_seq,
                "raw_line": raw_line,
            }
        )

    def of_type(self, event_type: str) -> list[dict[str, object]]:
        return [event for event in self.events if event["event_type"] == event_type]


class FakeSerial:
    def __init__(self) -> None:
        self.buffer = b""
        self.writes: list[bytes] = []
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        return len(self.buffer)

    def feed(self, data: bytes) -> None:
        self.buffer += data

    def read(self, size: int) -> bytes:
        data = self.buffer[:size]
        self.buffer = self.buffer[size:]
        return data

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def close(self) -> None:
        self.is_open = False


def make_controller(logger: RecordingLogger | None = None) -> RobotController:
    controller = RobotController.__new__(RobotController)
    controller.serial_config = SerialConfig()
    controller.control_config = ControlConfig()
    controller.logger = logger
    controller.telemetry = RobotTelemetry()
    controller._last_sent_command = ""
    controller._last_send_time = 0.0
    controller._serial_rx_buffer = ""
    controller._serial = FakeSerial()
    return controller


class SerialParsingTests(unittest.TestCase):
    def test_old_and_new_serial_formats_are_parsed(self) -> None:
        logger = RecordingLogger()
        controller = make_controller(logger)

        controller._handle_serial_line("100,distance,12.50")
        controller._handle_serial_line("120,7,distance,15.75")
        controller._handle_serial_line("140,8,imu,0.1,0.2,1.0,1.1,1.2,1.3,42.0")
        controller._handle_serial_line("160,9,status,calibration,ok,gz_bias,0.1234,samples,250")

        self.assertAlmostEqual(controller.telemetry.distance_cm, 15.75)
        self.assertIsNotNone(controller.telemetry.last_imu)
        self.assertEqual(controller.telemetry.last_imu.robot_seq, 8)
        self.assertIsNotNone(controller.telemetry.last_calibration)
        self.assertEqual(controller.telemetry.last_calibration.robot_seq, 9)

        parsed_distances = logger.of_type("parsed_distance")
        self.assertEqual(len(parsed_distances), 2)
        self.assertIsNone(parsed_distances[0]["robot_seq"])
        self.assertEqual(parsed_distances[1]["robot_seq"], 7)
        self.assertEqual(logger.of_type("parsed_status")[0]["robot_seq"], 9)

    def test_buffered_serial_keeps_split_lines_and_logs_malformed_lines(self) -> None:
        logger = RecordingLogger()
        controller = make_controller(logger)

        controller._serial.feed(b"10,1,distance,")
        controller.poll_telemetry()
        self.assertEqual(controller.telemetry.distance_cm, 100.0)
        self.assertEqual(logger.events, [])

        controller._serial.feed(b"20.00\n11,2,distance,30.00\nbad line\n")
        controller.poll_telemetry()

        self.assertAlmostEqual(controller.telemetry.distance_cm, 30.0)
        self.assertEqual(len(logger.of_type("rx_line")), 3)
        self.assertEqual(len(logger.of_type("parsed_distance")), 2)
        self.assertEqual(len(logger.of_type("parse_failure")), 1)
        self.assertEqual(logger.of_type("parse_failure")[0]["raw_line"], "bad line")

    def test_distance_records_are_logged_individually_when_state_keeps_latest(self) -> None:
        logger = RecordingLogger()
        controller = make_controller(logger)

        controller._handle_serial_line("200,3,distance,21.00")
        controller._handle_serial_line("220,4,distance,22.00")

        self.assertAlmostEqual(controller.telemetry.distance_cm, 22.0)
        parsed_distances = logger.of_type("parsed_distance")
        self.assertEqual(len(parsed_distances), 2)
        self.assertEqual(parsed_distances[0]["robot_seq"], 3)
        self.assertEqual(parsed_distances[1]["robot_seq"], 4)

    def test_imu_samples_drain_for_pose_estimator_replay(self) -> None:
        controller = make_controller()
        controller._handle_serial_line("100,1,imu,0.0,0.0,1.0,0.0,0.0,0.0,0.0")
        controller._handle_serial_line("120,2,imu,0.1,0.0,1.0,0.0,0.0,0.0,0.0")

        samples = controller.drain_imu_samples()
        estimator = PoseEstimator(
            PoseConfig(
                mode="imu_estimate",
                imu_estimate=ImuEstimateConfig(
                    accel_bias_samples=0,
                    accel_deadband_g=0.0,
                    stationary_accel_threshold_g=0.0,
                    stationary_gyro_threshold_dps=0.0,
                    velocity_decay_per_second=0.0,
                    max_dt_seconds=1.0,
                ),
            ),
            0.0,
            0.0,
            1.0,
        )
        for sample in samples:
            estimator.update_imu(sample)

        self.assertEqual([sample.robot_seq for sample in samples], [1, 2])
        self.assertEqual(controller.drain_imu_samples(), [])
        self.assertEqual(estimator._last_imu_timestamp_ms, 120)


class SessionLoggerTests(unittest.TestCase):
    def test_csv_logger_round_trips_raw_lines_with_commas(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            logger = SessionLogger(
                enabled=True,
                directory=temp_dir,
                flush_each_record=True,
                filename="session.csv",
            )
            logger.log_event(
                "serial",
                "rx_line",
                {"note": "raw comma line"},
                robot_timestamp_ms=100,
                robot_seq=5,
                raw_line="100,5,distance,12.30",
            )
            logger.close()

            path = Path(temp_dir) / "session.csv"
            with path.open("r", encoding="utf-8", newline="") as csv_file:
                rows = list(csv.DictReader(csv_file))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["raw_line"], "100,5,distance,12.30")
        self.assertEqual(rows[0]["robot_seq"], "5")
        self.assertEqual(json.loads(rows[0]["payload_json"])["note"], "raw comma line")


if __name__ == "__main__":
    unittest.main()
