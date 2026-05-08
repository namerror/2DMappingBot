import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch

from pose_estimator import PoseEstimator
from robot_config import (
    CommandEstimateConfig,
    ControlConfig,
    ImuEstimateConfig,
    PoseConfig,
    SerialConfig,
    WheelRampConfig,
    load_config,
)
from robot_serial import RobotController


@dataclass
class SampleImuTelemetry:
    timestamp_ms: int
    ax: float = 0.0
    ay: float = 0.0
    az: float = 1.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    yaw: float = 0.0


class ConfigTests(unittest.TestCase):
    def load_yaml(self, yaml_text: str):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "control_config.yaml"
            path.write_text(yaml_text, encoding="utf-8")
            return load_config(path)

    def test_pose_mode_is_authoritative(self) -> None:
        config = self.load_yaml(
            """
pose:
  mode: command_estimate
  imu_estimate:
    enabled: true
"""
        )

        self.assertEqual(config.pose.mode, "command_estimate")

    def test_signed_axes_are_loaded(self) -> None:
        config = self.load_yaml(
            """
pose:
  mode: imu_estimate
  imu_estimate:
    forward_axis: -y
    left_axis: x
    up_axis: -z
"""
        )

        self.assertEqual(config.pose.imu_estimate.forward_axis, "-y")
        self.assertEqual(config.pose.imu_estimate.left_axis, "x")
        self.assertEqual(config.pose.imu_estimate.up_axis, "-z")

    def test_invalid_axis_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.load_yaml(
                """
pose:
  mode: imu_estimate
  imu_estimate:
    forward_axis: sideways
    left_axis: y
    up_axis: z
"""
            )

    def test_wheel_ramp_defaults_to_disabled(self) -> None:
        config = self.load_yaml("")

        self.assertFalse(config.control.wheel_ramp.enabled)
        self.assertEqual(config.control.wheel_ramp.rate_percent_per_second, 100.0)

    def test_wheel_ramp_is_loaded(self) -> None:
        config = self.load_yaml(
            """
control:
  wheel_ramp:
    enabled: true
    rate_percent_per_second: 75.5
"""
        )

        self.assertTrue(config.control.wheel_ramp.enabled)
        self.assertEqual(config.control.wheel_ramp.rate_percent_per_second, 75.5)

    def test_enabled_wheel_ramp_requires_positive_rate(self) -> None:
        with self.assertRaises(ValueError):
            self.load_yaml(
                """
control:
  wheel_ramp:
    enabled: true
    rate_percent_per_second: 0
"""
            )


class FakeSerial:
    def __init__(self) -> None:
        self.is_open = True
        self.in_waiting = 0
        self.writes: list[str] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data.decode("utf-8").strip())
        return len(data)

    def readline(self) -> bytes:
        return b""

    def close(self) -> None:
        self.is_open = False


class RobotControllerSerialTests(unittest.TestCase):
    def make_controller(self, control_config: ControlConfig, fake_serial: FakeSerial):
        with patch("robot_serial.serial.Serial", return_value=fake_serial):
            return RobotController(SerialConfig(), control_config)

    def test_configure_wheel_ramp_sends_enabled_command(self) -> None:
        fake_serial = FakeSerial()
        controller = self.make_controller(
            ControlConfig(
                wheel_ramp=WheelRampConfig(
                    enabled=True,
                    rate_percent_per_second=100.0,
                )
            ),
            fake_serial,
        )

        controller.configure_wheel_ramp()

        self.assertEqual(fake_serial.writes, ["W1,100.0"])

    def test_configure_wheel_ramp_sends_disabled_command(self) -> None:
        fake_serial = FakeSerial()
        controller = self.make_controller(ControlConfig(), fake_serial)

        controller.configure_wheel_ramp()

        self.assertEqual(fake_serial.writes, ["W0,0"])


class PoseEstimatorTests(unittest.TestCase):
    def command_pose_config(self) -> PoseConfig:
        return PoseConfig(
            mode="command_estimate",
            command_estimate=CommandEstimateConfig(
                wheel_diameter_cm=10.0,
                track_width_cm=10.0,
                motor_rpm_at_100_pwm=60.0,
                pwm_to_speed_scale=1.0,
            ),
        )

    def imu_pose_config(self) -> PoseConfig:
        return PoseConfig(
            mode="imu_estimate",
            imu_estimate=ImuEstimateConfig(
                accel_bias_samples=0,
                accel_deadband_g=0.0,
                stationary_accel_threshold_g=0.0,
                stationary_gyro_threshold_dps=0.0,
                velocity_decay_per_second=0.0,
                max_dt_seconds=1.0,
            ),
        )

    def test_command_mode_updates_from_commands(self) -> None:
        estimator = PoseEstimator(self.command_pose_config(), 0.0, 0.0, 1.0)

        estimator.update("forward", 100, 1.0)

        self.assertGreater(estimator.pose.x, 0.0)
        self.assertAlmostEqual(estimator.pose.y, 0.0)

    def test_imu_mode_ignores_command_motion(self) -> None:
        estimator = PoseEstimator(self.imu_pose_config(), 0.0, 0.0, 1.0)

        estimator.update("forward", 100, 1.0)

        self.assertAlmostEqual(estimator.pose.x, 0.0)
        self.assertAlmostEqual(estimator.pose.y, 0.0)

    def test_imu_forward_acceleration_changes_position(self) -> None:
        estimator = PoseEstimator(self.imu_pose_config(), 0.0, 0.0, 1.0)

        estimator.update_imu(SampleImuTelemetry(timestamp_ms=0))
        estimator.update_imu(SampleImuTelemetry(timestamp_ms=1000, ax=1.0))

        self.assertGreater(estimator.pose.x, 0.0)
        self.assertAlmostEqual(estimator.pose.y, 0.0)

    def test_imu_gyro_left_turn_decreases_angle(self) -> None:
        estimator = PoseEstimator(self.imu_pose_config(), 0.0, 0.0, 1.0)

        estimator.update_imu(SampleImuTelemetry(timestamp_ms=0))
        estimator.update_imu(SampleImuTelemetry(timestamp_ms=1000, gz=90.0))

        self.assertAlmostEqual(estimator.pose.angle_deg, 270.0)

    def test_duplicate_imu_timestamps_are_ignored(self) -> None:
        estimator = PoseEstimator(self.imu_pose_config(), 0.0, 0.0, 1.0)

        estimator.update_imu(SampleImuTelemetry(timestamp_ms=0))
        estimator.update_imu(SampleImuTelemetry(timestamp_ms=1000, ax=1.0))
        x_after_first_sample = estimator.pose.x
        estimator.update_imu(SampleImuTelemetry(timestamp_ms=1000, ax=10.0))

        self.assertAlmostEqual(estimator.pose.x, x_after_first_sample)


if __name__ == "__main__":
    unittest.main()
