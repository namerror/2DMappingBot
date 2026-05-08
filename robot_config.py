from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


CONFIG_PATH = Path(__file__).with_name("control_config.yaml")
POSE_MODES = {"command_estimate", "imu_estimate"}
SIGNED_AXES = {"x", "y", "z", "-x", "-y", "-z"}


@dataclass(frozen=True)
class SerialConfig:
    port: str = "COM7"
    baud_rate: int = 115200
    timeout_seconds: float = 0.05


@dataclass(frozen=True)
class ControlConfig:
    command_interval_ms: int = 75
    command_duration_ms: int = 150
    default_pwm_percent: int = 50
    brake_duration_ms: int = 150


@dataclass(frozen=True)
class SensorConfig:
    angle_offset_deg: float = 0.0


@dataclass(frozen=True)
class CommandEstimateConfig:
    wheel_diameter_cm: float = 6.5
    track_width_cm: float = 12.0
    motor_rpm_at_100_pwm: float = 180.0
    pwm_to_speed_scale: float = 1.0


@dataclass(frozen=True)
class ImuEstimateConfig:
    forward_axis: str = "x"
    left_axis: str = "y"
    up_axis: str = "z"
    accel_bias_samples: int = 50
    accel_deadband_g: float = 0.03
    stationary_accel_threshold_g: float = 0.04
    stationary_gyro_threshold_dps: float = 2.0
    velocity_decay_per_second: float = 0.15
    max_dt_seconds: float = 0.10


@dataclass(frozen=True)
class PoseConfig:
    mode: str = "command_estimate"
    command_estimate: CommandEstimateConfig = CommandEstimateConfig()
    imu_estimate: ImuEstimateConfig = ImuEstimateConfig()


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig = SerialConfig()
    control: ControlConfig = ControlConfig()
    sensor: SensorConfig = SensorConfig()
    pose: PoseConfig = PoseConfig()


def _section(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key, {})
    return value if isinstance(value, dict) else {}


def _pose_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    if mode not in POSE_MODES:
        expected = ", ".join(sorted(POSE_MODES))
        raise ValueError(f"Invalid pose.mode {mode!r}; expected one of: {expected}")
    return mode


def _signed_axis(section: dict[str, Any], key: str, default: str) -> str:
    axis = str(section.get(key, default)).strip().lower()
    if axis not in SIGNED_AXES:
        expected = ", ".join(sorted(SIGNED_AXES))
        raise ValueError(f"Invalid pose.imu_estimate.{key} {axis!r}; expected one of: {expected}")
    return axis


def _validate_distinct_axes(forward_axis: str, left_axis: str, up_axis: str) -> None:
    base_axes = [axis[-1] for axis in (forward_axis, left_axis, up_axis)]
    if len(set(base_axes)) != len(base_axes):
        raise ValueError(
            "pose.imu_estimate forward_axis, left_axis, and up_axis must use distinct axes"
        )


def load_config(path: Path = CONFIG_PATH) -> AppConfig:
    if not path.exists():
        return AppConfig()

    with path.open("r", encoding="utf-8") as config_file:
        raw = yaml.safe_load(config_file) or {}

    if not isinstance(raw, dict):
        return AppConfig()

    serial = _section(raw, "serial")
    control = _section(raw, "control")
    sensor = _section(raw, "sensor")
    pose = _section(raw, "pose")
    command_estimate = _section(pose, "command_estimate")
    imu_estimate = _section(pose, "imu_estimate")
    forward_axis = _signed_axis(imu_estimate, "forward_axis", ImuEstimateConfig.forward_axis)
    left_axis = _signed_axis(imu_estimate, "left_axis", ImuEstimateConfig.left_axis)
    up_axis = _signed_axis(imu_estimate, "up_axis", ImuEstimateConfig.up_axis)
    _validate_distinct_axes(forward_axis, left_axis, up_axis)

    return AppConfig(
        serial=SerialConfig(
            port=str(serial.get("port", SerialConfig.port)),
            baud_rate=int(serial.get("baud_rate", SerialConfig.baud_rate)),
            timeout_seconds=float(serial.get("timeout_seconds", SerialConfig.timeout_seconds)),
        ),
        control=ControlConfig(
            command_interval_ms=int(
                control.get("command_interval_ms", ControlConfig.command_interval_ms)
            ),
            command_duration_ms=int(
                control.get("command_duration_ms", ControlConfig.command_duration_ms)
            ),
            default_pwm_percent=int(
                control.get("default_pwm_percent", ControlConfig.default_pwm_percent)
            ),
            brake_duration_ms=int(control.get("brake_duration_ms", ControlConfig.brake_duration_ms)),
        ),
        sensor=SensorConfig(
            angle_offset_deg=float(sensor.get("angle_offset_deg", SensorConfig.angle_offset_deg)),
        ),
        pose=PoseConfig(
            mode=_pose_mode(pose.get("mode", PoseConfig.mode)),
            command_estimate=CommandEstimateConfig(
                wheel_diameter_cm=float(
                    command_estimate.get(
                        "wheel_diameter_cm", CommandEstimateConfig.wheel_diameter_cm
                    )
                ),
                track_width_cm=float(
                    command_estimate.get("track_width_cm", CommandEstimateConfig.track_width_cm)
                ),
                motor_rpm_at_100_pwm=float(
                    command_estimate.get(
                        "motor_rpm_at_100_pwm",
                        CommandEstimateConfig.motor_rpm_at_100_pwm,
                    )
                ),
                pwm_to_speed_scale=float(
                    command_estimate.get(
                        "pwm_to_speed_scale", CommandEstimateConfig.pwm_to_speed_scale
                    )
                ),
            ),
            imu_estimate=ImuEstimateConfig(
                forward_axis=forward_axis,
                left_axis=left_axis,
                up_axis=up_axis,
                accel_bias_samples=max(
                    0,
                    int(
                        imu_estimate.get(
                            "accel_bias_samples", ImuEstimateConfig.accel_bias_samples
                        )
                    ),
                ),
                accel_deadband_g=max(
                    0.0,
                    float(
                        imu_estimate.get("accel_deadband_g", ImuEstimateConfig.accel_deadband_g)
                    ),
                ),
                stationary_accel_threshold_g=max(
                    0.0,
                    float(
                        imu_estimate.get(
                            "stationary_accel_threshold_g",
                            ImuEstimateConfig.stationary_accel_threshold_g,
                        )
                    ),
                ),
                stationary_gyro_threshold_dps=max(
                    0.0,
                    float(
                        imu_estimate.get(
                            "stationary_gyro_threshold_dps",
                            ImuEstimateConfig.stationary_gyro_threshold_dps,
                        )
                    ),
                ),
                velocity_decay_per_second=max(
                    0.0,
                    float(
                        imu_estimate.get(
                            "velocity_decay_per_second",
                            ImuEstimateConfig.velocity_decay_per_second,
                        )
                    ),
                ),
                max_dt_seconds=max(
                    0.001,
                    float(imu_estimate.get("max_dt_seconds", ImuEstimateConfig.max_dt_seconds)),
                ),
            ),
        ),
    )
