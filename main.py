import math
import sys
from dataclasses import dataclass

import pygame

from pose_estimator import PoseEstimator
from robot_config import CONFIG_PATH, load_config
from robot_serial import RobotCalibrationError, RobotConnectionError, RobotController


WINDOW_WIDTH = 1800
WINDOW_HEIGHT = 800
PANEL_WIDTH = 280
MAP_WIDTH = WINDOW_WIDTH - PANEL_WIDTH
DISTANCE_SCALE = 5

ROBOT_COLOR = (0, 220, 120)
WALL_COLOR = (245, 80, 80)
BACKGROUND_COLOR = (8, 10, 12)
GRID_COLOR = (24, 29, 34)
PANEL_COLOR = (26, 30, 34)
PANEL_BORDER_COLOR = (70, 78, 86)
TEXT_COLOR = (220, 226, 232)
MUTED_TEXT_COLOR = (150, 160, 170)
ACCENT_COLOR = (62, 142, 255)
ACTIVE_COLOR = (70, 160, 105)
WARNING_COLOR = (250, 185, 80)
SCANNED_WALL_COLOR = (230, 145, 55)
SENSOR_BEAM_COLOR = (60, 95, 180)

ROBOT_RADIUS = 8
WALL_LENGTH = 80
WALL_THICKNESS = 3
AUTO_SCAN_MOVE_PX = 30
SCAN_COOLDOWN_FRAMES = 10
MAX_WALL_DISTANCE_CM = 200


@dataclass
class Button:
    name: str
    label: str
    rect: pygame.Rect
    action: str | None = None

    def contains(self, pos: tuple[int, int]) -> bool:
        return self.rect.collidepoint(pos)


def get_sensor_endpoint(robot_x: float, robot_y: float, angle: float, distance_cm: float) -> tuple[float, float]:
    distance_px = distance_cm * DISTANCE_SCALE
    end_x = robot_x + distance_px * math.cos(math.radians(angle))
    end_y = robot_y + distance_px * math.sin(math.radians(angle))
    return end_x, end_y


def get_sensor_angle(pose_angle: float, sensor_angle_offset_deg: float) -> float:
    return (pose_angle - sensor_angle_offset_deg) % 360.0


def get_wall_angle(sensor_angle: float) -> float:
    return sensor_angle + 90


def draw_text(
    surface: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    x: int,
    y: int,
    color: tuple[int, int, int] = TEXT_COLOR,
) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def draw_button(
    surface: pygame.Surface,
    font: pygame.font.Font,
    button: Button,
    active: bool = False,
    disabled: bool = False,
) -> None:
    fill = ACTIVE_COLOR if active else (43, 49, 55)
    border = ACCENT_COLOR if active else PANEL_BORDER_COLOR
    if disabled:
        fill = (34, 38, 42)
        border = (52, 57, 62)

    pygame.draw.rect(surface, fill, button.rect, border_radius=6)
    pygame.draw.rect(surface, border, button.rect, width=1, border_radius=6)
    text_color = MUTED_TEXT_COLOR if disabled else TEXT_COLOR
    rendered = font.render(button.label, True, text_color)
    surface.blit(rendered, rendered.get_rect(center=button.rect.center))


def draw_angled_wall(
    surface: pygame.Surface,
    x: float,
    y: float,
    wall_angle: float,
    length: int,
    thickness: int,
) -> None:
    half_length = length // 2
    end1_x = x + half_length * math.cos(math.radians(wall_angle))
    end1_y = y + half_length * math.sin(math.radians(wall_angle))
    end2_x = x - half_length * math.cos(math.radians(wall_angle))
    end2_y = y - half_length * math.sin(math.radians(wall_angle))
    pygame.draw.line(surface, WALL_COLOR, (end1_x, end1_y), (end2_x, end2_y), thickness)


def add_wall_to_map(
    detected_walls: list[dict[str, float]],
    world_x: float,
    world_y: float,
    sensor_angle: float,
) -> bool:
    wall_angle = get_wall_angle(sensor_angle)
    for wall in detected_walls:
        dist = math.sqrt((wall["x"] - world_x) ** 2 + (wall["y"] - world_y) ** 2)
        angle_diff = abs(wall["wall_angle"] - wall_angle) % 180
        if dist < 20 and angle_diff < 15:
            return False

    detected_walls.append(
        {
            "x": world_x,
            "y": world_y,
            "wall_angle": wall_angle,
            "sensor_angle": sensor_angle,
        }
    )
    return True


def draw_robot(surface: pygame.Surface, x: float, y: float, angle: float) -> None:
    pygame.draw.circle(surface, ROBOT_COLOR, (int(x), int(y)), ROBOT_RADIUS)
    pygame.draw.circle(surface, (0, 95, 65), (int(x), int(y)), ROBOT_RADIUS - 2)

    end_x = x + ROBOT_RADIUS * 2 * math.cos(math.radians(angle))
    end_y = y + ROBOT_RADIUS * 2 * math.sin(math.radians(angle))
    pygame.draw.line(surface, (255, 255, 255), (x, y), (end_x, end_y), 2)

    for sweep_angle in [-30, -15, 0, 15, 30]:
        arc_x = x + (ROBOT_RADIUS + 8) * math.cos(math.radians(angle + sweep_angle))
        arc_y = y + (ROBOT_RADIUS + 8) * math.sin(math.radians(angle + sweep_angle))
        pygame.draw.circle(surface, (80, 88, 96), (int(arc_x), int(arc_y)), 2)


def draw_sensor_beam(
    surface: pygame.Surface,
    robot_x: float,
    robot_y: float,
    angle: float,
    distance_cm: float,
) -> tuple[float | None, float | None]:
    if 2 < distance_cm < 400:
        end_x, end_y = get_sensor_endpoint(robot_x, robot_y, angle, distance_cm)
        pygame.draw.line(surface, SENSOR_BEAM_COLOR, (robot_x, robot_y), (end_x, end_y), 2)
        pygame.draw.circle(surface, SCANNED_WALL_COLOR, (int(end_x), int(end_y)), 6)
        return end_x, end_y
    return None, None


def draw_map(
    surface: pygame.Surface,
    pose_x: float,
    pose_y: float,
    pose_angle: float,
    sensor_angle: float,
    current_distance: float,
    detected_walls: list[dict[str, float]],
) -> tuple[float | None, float | None]:
    pygame.draw.rect(surface, BACKGROUND_COLOR, (0, 0, MAP_WIDTH, WINDOW_HEIGHT))

    for x in range(0, MAP_WIDTH, 50):
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, WINDOW_HEIGHT), 1)
    for y in range(0, WINDOW_HEIGHT, 50):
        pygame.draw.line(surface, GRID_COLOR, (0, y), (MAP_WIDTH, y), 1)

    origin = (MAP_WIDTH // 2, WINDOW_HEIGHT // 2)
    pygame.draw.circle(surface, (70, 76, 82), origin, 4)
    pygame.draw.line(surface, (70, 76, 82), (origin[0] - 10, origin[1]), (origin[0] + 10, origin[1]), 1)
    pygame.draw.line(surface, (70, 76, 82), (origin[0], origin[1] - 10), (origin[0], origin[1] + 10), 1)

    for wall in detected_walls:
        draw_angled_wall(surface, wall["x"], wall["y"], wall["wall_angle"], WALL_LENGTH, WALL_THICKNESS)

    hit = draw_sensor_beam(surface, pose_x, pose_y, sensor_angle, current_distance)
    draw_robot(surface, pose_x, pose_y, pose_angle)
    return hit


def build_buttons() -> list[Button]:
    panel_x = MAP_WIDTH + 18
    y = 230
    size = 56
    gap = 10
    center_x = panel_x + size + gap

    return [
        Button("forward", "^", pygame.Rect(center_x, y, size, size), "forward"),
        Button("left", "<", pygame.Rect(panel_x, y + size + gap, size, size), "left"),
        Button("brake", "BRK", pygame.Rect(center_x, y + size + gap, size, size), "brake"),
        Button("right", ">", pygame.Rect(center_x + size + gap, y + size + gap, size, size), "right"),
        Button("backward", "v", pygame.Rect(center_x, y + 2 * (size + gap), size, size), "backward"),
        Button("scan", "SCAN", pygame.Rect(panel_x, y + 3 * (size + gap) + 12, 82, 38), None),
        Button("clear", "CLEAR", pygame.Rect(panel_x + 92, y + 3 * (size + gap) + 12, 82, 38), None),
        Button("reset", "RESET", pygame.Rect(panel_x, y + 3 * (size + gap) + 60, 82, 38), None),
        Button("auto", "AUTO", pygame.Rect(panel_x + 92, y + 3 * (size + gap) + 60, 82, 38), None),
    ]


def selected_mouse_button(buttons: list[Button], mouse_down: bool) -> Button | None:
    if not mouse_down:
        return None
    mouse_pos = pygame.mouse.get_pos()
    for button in buttons:
        if button.contains(mouse_pos):
            return button
    return None


def keyboard_drive_action(keys: pygame.key.ScancodeWrapper) -> str | None:
    pressed = {
        "forward": keys[pygame.K_UP],
        "backward": keys[pygame.K_DOWN],
        "left": keys[pygame.K_LEFT],
        "right": keys[pygame.K_RIGHT],
    }
    if pressed["forward"] and not pressed["backward"]:
        return "forward"
    if pressed["backward"] and not pressed["forward"]:
        return "backward"
    if pressed["left"] and not pressed["right"]:
        return "left"
    if pressed["right"] and not pressed["left"]:
        return "right"
    return None


def draw_panel(
    surface: pygame.Surface,
    fonts: dict[str, pygame.font.Font],
    buttons: list[Button],
    active_action: str | None,
    current_distance: float,
    pose_x: float,
    pose_y: float,
    pose_angle: float,
    sensor_angle: float,
    wall_count: int,
    auto_scan: bool,
    pose_status: str,
    imu_required: bool,
    pwm_percent: int,
    port: str,
    hit: tuple[float | None, float | None],
) -> None:
    panel_rect = pygame.Rect(MAP_WIDTH, 0, PANEL_WIDTH, WINDOW_HEIGHT)
    pygame.draw.rect(surface, PANEL_COLOR, panel_rect)
    pygame.draw.line(surface, PANEL_BORDER_COLOR, (MAP_WIDTH, 0), (MAP_WIDTH, WINDOW_HEIGHT), 1)

    x = MAP_WIDTH + 18
    y = 18
    draw_text(surface, fonts["title"], "Robot Control", x, y)
    y += 34
    draw_text(surface, fonts["small"], f"Port: {port}", x, y, MUTED_TEXT_COLOR)
    y += 22
    draw_text(surface, fonts["small"], f"Distance: {current_distance:.1f} cm", x, y)
    y += 22
    draw_text(surface, fonts["small"], f"Pose: ({int(pose_x)}, {int(pose_y)})", x, y)
    y += 22
    draw_text(surface, fonts["small"], f"Angle: {int(pose_angle) % 360} deg", x, y)
    y += 22
    draw_text(surface, fonts["small"], f"Walls: {wall_count}", x, y)
    y += 22
    draw_text(surface, fonts["small"], f"Auto-scan: {'ON' if auto_scan else 'OFF'}", x, y)
    y += 22
    draw_text(surface, fonts["small"], f"PWM: {pwm_percent}%", x, y)
    y += 22
    imu_text = "IMU: required" if imu_required else "IMU: off"
    imu_color = MUTED_TEXT_COLOR if imu_required else WARNING_COLOR
    draw_text(surface, fonts["small"], imu_text, x, y, imu_color)
    y += 22
    status_color = WARNING_COLOR if "not implemented" in pose_status else MUTED_TEXT_COLOR
    draw_text(surface, fonts["small"], pose_status, x, y, status_color)

    for button in buttons:
        active = button.action == active_action
        if button.name == "auto":
            active = auto_scan
        draw_button(surface, fonts["button"], button, active=active)

    help_y = 590
    help_lines = [
        "Keys",
        "Arrows: hold to drive",
        "+/-: adjust PWM",
        "Space: brake",
        "S: scan wall",
        "C: clear map",
        "R: reset pose",
        "A: auto-scan",
        "Esc: quit",
    ]
    for i, line in enumerate(help_lines):
        color = TEXT_COLOR if i == 0 else MUTED_TEXT_COLOR
        draw_text(surface, fonts["small"], line, x, help_y + i * 20, color)

    if hit[0] is not None and hit[1] is not None and 2 < current_distance < MAX_WALL_DISTANCE_CM:
        draw_text(surface, fonts["small"], f"Hit: ({int(hit[0])}, {int(hit[1])})", x, 760, WARNING_COLOR)
        draw_text(surface, fonts["small"], f"Wall angle: {int(get_wall_angle(sensor_angle)) % 180} deg", x, 780, WARNING_COLOR)


def clamp_pose_to_map(pose_estimator: PoseEstimator) -> None:
    pose = pose_estimator.pose
    pose.x = max(ROBOT_RADIUS, min(MAP_WIDTH - ROBOT_RADIUS, pose.x))
    pose.y = max(ROBOT_RADIUS, min(WINDOW_HEIGHT - ROBOT_RADIUS, pose.y))


def scan_wall(
    detected_walls: list[dict[str, float]],
    current_distance: float,
    pose_x: float,
    pose_y: float,
    sensor_angle: float,
) -> None:
    if 2 < current_distance < MAX_WALL_DISTANCE_CM:
        wall_x, wall_y = get_sensor_endpoint(pose_x, pose_y, sensor_angle, current_distance)
        if add_wall_to_map(detected_walls, wall_x, wall_y, sensor_angle):
            print(f"Wall added at ({int(wall_x)}, {int(wall_y)})")
        else:
            print("Wall already exists nearby.")
    else:
        print(f"No valid wall detected. Distance: {current_distance:.1f} cm")


def print_startup(
    config_path: str,
    port: str,
    pose_status: str,
    sensor_angle_offset_deg: float,
    imu_required: bool,
) -> None:
    print("2D Mapping Bot control panel")
    print(f"Config: {config_path}")
    print(f"Serial port: {port}")
    print(f"Pose mode: {pose_status}")
    print(f"IMU required: {'yes' if imu_required else 'no (distance-only)'}")
    print(f"Sensor angle offset: {sensor_angle_offset_deg:.1f} deg (+left, -right)")
    print("")
    print("Controls:")
    print("  Arrow keys or panel buttons: hold to drive")
    print("  Space: brake")
    print("  +/-: adjust PWM")
    print("  S: scan wall")
    print("  C: clear map")
    print("  R: reset pose")
    print("  A: toggle auto-scan")
    print("  Esc: quit")
    print("")


def print_calibration_summary(controller: RobotController) -> None:
    print("Calibrating IMU: keep the robot still for 5 seconds...")
    status = controller.calibrate_imu()
    if status.gz_bias is None:
        print("IMU calibration complete.")
        return

    samples = f", samples={status.samples}" if status.samples is not None else ""
    print(f"IMU calibration complete: gz_bias={status.gz_bias:.4f} deg/s{samples}")


def main() -> int:
    config = load_config()
    imu_required = config.pose.imu_estimate.enabled
    start_x = MAP_WIDTH // 2
    start_y = WINDOW_HEIGHT // 2
    pose_estimator = PoseEstimator(config.pose, start_x, start_y, DISTANCE_SCALE)
    pwm_percent = max(0, min(100, config.control.default_pwm_percent))

    try:
        controller = RobotController(config.serial, config.control)
    except RobotConnectionError as exc:
        print(f"ERROR: Could not open serial port {config.serial.port}.")
        print("Make sure the robot is connected and Serial Monitor is closed.")
        print(f"Details: {exc}")
        return 1

    controller.set_imu_required(imu_required)
    if imu_required:
        try:
            print_calibration_summary(controller)
        except RobotCalibrationError as exc:
            print("ERROR: IMU calibration failed.")
            print("Leave the robot still, then restart the control app to try again.")
            print(f"Details: {exc}")
            controller.close()
            return 1
    else:
        print("IMU calibration skipped: imu_estimate.enabled=false; using distance-only mode.")

    pose_estimator.reset(start_x, start_y, 0.0)
    controller.set_speed(pwm_percent)
    print_startup(
        str(CONFIG_PATH),
        config.serial.port,
        pose_estimator.status,
        config.sensor.angle_offset_deg,
        imu_required,
    )

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("2D Mapping Bot Control Panel")
    clock = pygame.time.Clock()
    fonts = {
        "title": pygame.font.Font(None, 34),
        "small": pygame.font.Font(None, 20),
        "button": pygame.font.Font(None, 24),
    }
    buttons = build_buttons()

    detected_walls: list[dict[str, float]] = []
    current_distance = 100.0
    auto_scan_mode = True
    auto_scan_last_pos = (pose_estimator.pose.x, pose_estimator.pose.y)
    scan_cooldown = 0
    mouse_down = False
    last_drive_action: str | None = None
    running = True

    try:
        while running:
            dt_seconds = clock.tick(60) / 1000.0
            keys = pygame.key.get_pressed()
            brake_requested = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mouse_down = True
                    clicked = selected_mouse_button(buttons, True)
                    if clicked and clicked.name == "brake":
                        controller.brake()
                        brake_requested = True
                    elif clicked and clicked.name == "scan":
                        sensor_angle = get_sensor_angle(
                            pose_estimator.pose.angle_deg,
                            config.sensor.angle_offset_deg,
                        )
                        scan_wall(
                            detected_walls,
                            current_distance,
                            pose_estimator.pose.x,
                            pose_estimator.pose.y,
                            sensor_angle,
                        )
                    elif clicked and clicked.name == "clear":
                        detected_walls.clear()
                        print("Map cleared.")
                    elif clicked and clicked.name == "reset":
                        pose_estimator.reset(start_x, start_y)
                        auto_scan_last_pos = (pose_estimator.pose.x, pose_estimator.pose.y)
                        print("Pose reset.")
                    elif clicked and clicked.name == "auto":
                        auto_scan_mode = not auto_scan_mode
                        auto_scan_last_pos = (pose_estimator.pose.x, pose_estimator.pose.y)
                        print(f"Auto-scan: {'ON' if auto_scan_mode else 'OFF'}")
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    mouse_down = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        controller.brake()
                        brake_requested = True
                    elif event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                        pwm_percent = min(100, pwm_percent + 5)
                        controller.set_speed(pwm_percent)
                        print(f"PWM: {pwm_percent}%")
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        pwm_percent = max(0, pwm_percent - 5)
                        controller.set_speed(pwm_percent)
                        print(f"PWM: {pwm_percent}%")
                    elif event.key == pygame.K_c:
                        detected_walls.clear()
                        print("Map cleared.")
                    elif event.key == pygame.K_r:
                        pose_estimator.reset(start_x, start_y)
                        auto_scan_last_pos = (pose_estimator.pose.x, pose_estimator.pose.y)
                        print("Pose reset.")
                    elif event.key == pygame.K_s:
                        sensor_angle = get_sensor_angle(
                            pose_estimator.pose.angle_deg,
                            config.sensor.angle_offset_deg,
                        )
                        scan_wall(
                            detected_walls,
                            current_distance,
                            pose_estimator.pose.x,
                            pose_estimator.pose.y,
                            sensor_angle,
                        )
                    elif event.key == pygame.K_a:
                        auto_scan_mode = not auto_scan_mode
                        auto_scan_last_pos = (pose_estimator.pose.x, pose_estimator.pose.y)
                        print(f"Auto-scan: {'ON' if auto_scan_mode else 'OFF'}")

            mouse_button = selected_mouse_button(buttons, mouse_down)
            mouse_action = mouse_button.action if mouse_button and mouse_button.action != "brake" else None
            drive_action = None if brake_requested else mouse_action or keyboard_drive_action(keys)

            if drive_action:
                controller.hold_drive(drive_action)
            elif brake_requested:
                last_drive_action = None
            elif last_drive_action:
                controller.stop()
            last_drive_action = drive_action

            pose_estimator.update(drive_action, pwm_percent, dt_seconds)
            clamp_pose_to_map(pose_estimator)
            current_distance = controller.read_distance()

            pose = pose_estimator.pose
            sensor_angle = get_sensor_angle(pose.angle_deg, config.sensor.angle_offset_deg)
            if auto_scan_mode and scan_cooldown <= 0 and 2 < current_distance < MAX_WALL_DISTANCE_CM:
                moved_dist = math.sqrt(
                    (pose.x - auto_scan_last_pos[0]) ** 2 + (pose.y - auto_scan_last_pos[1]) ** 2
                )
                if moved_dist > AUTO_SCAN_MOVE_PX:
                    wall_x, wall_y = get_sensor_endpoint(pose.x, pose.y, sensor_angle, current_distance)
                    if add_wall_to_map(detected_walls, wall_x, wall_y, sensor_angle):
                        print(f"Auto-scan: wall added at ({int(wall_x)}, {int(wall_y)})")
                    auto_scan_last_pos = (pose.x, pose.y)
                    scan_cooldown = SCAN_COOLDOWN_FRAMES
            if scan_cooldown > 0:
                scan_cooldown -= 1

            hit = draw_map(
                screen,
                pose.x,
                pose.y,
                pose.angle_deg,
                sensor_angle,
                current_distance,
                detected_walls,
            )
            draw_panel(
                screen,
                fonts,
                buttons,
                drive_action,
                current_distance,
                pose.x,
                pose.y,
                pose.angle_deg,
                sensor_angle,
                len(detected_walls),
                auto_scan_mode,
                pose_estimator.status,
                imu_required,
                pwm_percent,
                controller.port,
                hit,
            )
            pygame.display.flip()
    finally:
        controller.stop()
        controller.close()
        pygame.quit()

    return 0


if __name__ == "__main__":
    sys.exit(main())
