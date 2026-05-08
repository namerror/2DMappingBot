import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

from pose_estimator import PoseEstimator
from robot_config import CommandEstimateConfig, ImuEstimateConfig, PoseConfig, load_config


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
