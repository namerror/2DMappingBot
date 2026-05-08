import math
from dataclasses import dataclass
from typing import Protocol

from robot_config import PoseConfig


G_TO_CM_PER_SECOND2 = 980.665


@dataclass
class RobotPose:
    x: float
    y: float
    angle_deg: float


class ImuSample(Protocol):
    timestamp_ms: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


class PoseEstimator:
    def __init__(
        self,
        pose_config: PoseConfig,
        start_x: float,
        start_y: float,
        distance_scale_px_per_cm: float,
    ) -> None:
        self.pose_config = pose_config
        self.distance_scale_px_per_cm = distance_scale_px_per_cm
        self.pose = RobotPose(start_x, start_y, 0.0)
        self._velocity_x_cm_per_second = 0.0
        self._velocity_y_cm_per_second = 0.0
        self._last_imu_timestamp_ms: int | None = None
        self._accel_bias_sample_count = 0
        self._accel_bias_forward_sum_g = 0.0
        self._accel_bias_left_sum_g = 0.0
        self._accel_bias_forward_g = 0.0
        self._accel_bias_left_g = 0.0
        self._accel_bias_ready = pose_config.imu_estimate.accel_bias_samples <= 0
        self.status = "Command estimate"
        if pose_config.mode == "imu_estimate":
            self._set_imu_status()

    def reset(self, x: float, y: float, angle_deg: float = 0.0) -> None:
        self.pose = RobotPose(x, y, angle_deg)
        self._velocity_x_cm_per_second = 0.0
        self._velocity_y_cm_per_second = 0.0
        self._last_imu_timestamp_ms = None
        if self.pose_config.mode == "imu_estimate":
            self._set_imu_status()

    def update(self, active_action: str | None, pwm_percent: int, dt_seconds: float) -> None:
        if self.pose_config.mode == "imu_estimate":
            return
        if self.pose_config.mode != "command_estimate" or active_action is None:
            return

        config = self.pose_config.command_estimate
        wheel_circumference_cm = math.pi * max(config.wheel_diameter_cm, 0.1)
        pwm_fraction = max(0.0, min(1.0, pwm_percent / 100.0))
        wheel_rpm = config.motor_rpm_at_100_pwm * pwm_fraction * config.pwm_to_speed_scale
        linear_cm_per_second = wheel_circumference_cm * wheel_rpm / 60.0

        if active_action == "forward":
            self._move(linear_cm_per_second, dt_seconds)
        elif active_action == "backward":
            self._move(-linear_cm_per_second, dt_seconds)
        elif active_action in ("left", "right"):
            track_width_cm = max(config.track_width_cm, 0.1)
            angular_rad_per_second = (2.0 * linear_cm_per_second) / track_width_cm
            direction = -1.0 if active_action == "left" else 1.0
            self.pose.angle_deg += math.degrees(angular_rad_per_second) * direction * dt_seconds
            self.pose.angle_deg %= 360.0

    def _move(self, speed_cm_per_second: float, dt_seconds: float) -> None:
        distance_px = speed_cm_per_second * dt_seconds * self.distance_scale_px_per_cm
        self.pose.x += distance_px * math.cos(math.radians(self.pose.angle_deg))
        self.pose.y += distance_px * math.sin(math.radians(self.pose.angle_deg))

    def update_imu(self, imu: ImuSample | None) -> None:
        if self.pose_config.mode != "imu_estimate":
            return

        if imu is None:
            self._set_imu_status()
            return

        timestamp_ms = int(imu.timestamp_ms)
        if self._last_imu_timestamp_ms is None:
            self._last_imu_timestamp_ms = timestamp_ms
            self._collect_accel_bias(imu)
            self._set_imu_status()
            return

        if timestamp_ms <= self._last_imu_timestamp_ms:
            return

        raw_dt_seconds = (timestamp_ms - self._last_imu_timestamp_ms) / 1000.0
        self._last_imu_timestamp_ms = timestamp_ms
        config = self.pose_config.imu_estimate
        dt_seconds = min(raw_dt_seconds, max(config.max_dt_seconds, 0.001))

        up_gyro_dps = self._axis_value(imu, config.up_axis, "g")
        self.pose.angle_deg = (self.pose.angle_deg - up_gyro_dps * dt_seconds) % 360.0

        if not self._accel_bias_ready:
            self._collect_accel_bias(imu)
            self._set_imu_status()
            return

        forward_accel_g = self._axis_value(imu, config.forward_axis, "a")
        left_accel_g = self._axis_value(imu, config.left_axis, "a")
        forward_accel_g -= self._accel_bias_forward_g
        left_accel_g -= self._accel_bias_left_g
        forward_accel_g = self._apply_deadband(forward_accel_g, config.accel_deadband_g)
        left_accel_g = self._apply_deadband(left_accel_g, config.accel_deadband_g)

        planar_accel_g = math.hypot(forward_accel_g, left_accel_g)
        if (
            planar_accel_g <= config.stationary_accel_threshold_g
            and abs(up_gyro_dps) <= config.stationary_gyro_threshold_dps
        ):
            self._velocity_x_cm_per_second = 0.0
            self._velocity_y_cm_per_second = 0.0
            self._set_imu_status()
            return

        forward_accel_cm = forward_accel_g * G_TO_CM_PER_SECOND2
        left_accel_cm = left_accel_g * G_TO_CM_PER_SECOND2
        angle_rad = math.radians(self.pose.angle_deg)
        world_accel_x_cm = (
            forward_accel_cm * math.cos(angle_rad) + left_accel_cm * math.sin(angle_rad)
        )
        world_accel_y_cm = (
            forward_accel_cm * math.sin(angle_rad) - left_accel_cm * math.cos(angle_rad)
        )

        delta_x_cm = (
            self._velocity_x_cm_per_second * dt_seconds
            + 0.5 * world_accel_x_cm * dt_seconds * dt_seconds
        )
        delta_y_cm = (
            self._velocity_y_cm_per_second * dt_seconds
            + 0.5 * world_accel_y_cm * dt_seconds * dt_seconds
        )
        self.pose.x += delta_x_cm * self.distance_scale_px_per_cm
        self.pose.y += delta_y_cm * self.distance_scale_px_per_cm
        self._velocity_x_cm_per_second += world_accel_x_cm * dt_seconds
        self._velocity_y_cm_per_second += world_accel_y_cm * dt_seconds

        if config.velocity_decay_per_second > 0.0:
            decay_factor = math.exp(-config.velocity_decay_per_second * dt_seconds)
            self._velocity_x_cm_per_second *= decay_factor
            self._velocity_y_cm_per_second *= decay_factor

        self._set_imu_status()

    def _collect_accel_bias(self, imu: ImuSample) -> None:
        config = self.pose_config.imu_estimate
        if self._accel_bias_ready:
            return

        self._accel_bias_forward_sum_g += self._axis_value(imu, config.forward_axis, "a")
        self._accel_bias_left_sum_g += self._axis_value(imu, config.left_axis, "a")
        self._accel_bias_sample_count += 1
        if self._accel_bias_sample_count >= config.accel_bias_samples:
            self._accel_bias_forward_g = (
                self._accel_bias_forward_sum_g / self._accel_bias_sample_count
            )
            self._accel_bias_left_g = self._accel_bias_left_sum_g / self._accel_bias_sample_count
            self._accel_bias_ready = True

    def _set_imu_status(self) -> None:
        config = self.pose_config.imu_estimate
        if not self._accel_bias_ready:
            target = max(config.accel_bias_samples, 1)
            self.status = f"IMU estimate: bias {self._accel_bias_sample_count}/{target}"
        elif self._last_imu_timestamp_ms is None:
            self.status = "IMU estimate: waiting for IMU"
        else:
            self.status = "IMU estimate"

    def _axis_value(self, imu: ImuSample, signed_axis: str, prefix: str) -> float:
        sign = -1.0 if signed_axis.startswith("-") else 1.0
        axis = signed_axis[-1]
        return sign * float(getattr(imu, f"{prefix}{axis}"))

    def _apply_deadband(self, value: float, threshold: float) -> float:
        if abs(value) < threshold:
            return 0.0
        return value
