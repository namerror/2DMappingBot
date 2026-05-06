import math
from dataclasses import dataclass

from robot_config import PoseConfig


@dataclass
class RobotPose:
    x: float
    y: float
    angle_deg: float


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
        self.status = "Command estimate"
        if pose_config.mode == "imu_estimate":
            self.status = "IMU estimate placeholder - not implemented"

    def reset(self, x: float, y: float, angle_deg: float = 0.0) -> None:
        self.pose = RobotPose(x, y, angle_deg)

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
