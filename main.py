import pygame
import serial
import sys
import math

# --- CONFIGURATION ---
SERIAL_PORT = 'COM7'      # 🔁 CHANGE THIS to your port
BAUD_RATE = 115200

WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 800
DISTANCE_SCALE = 5        # Pixels per cm

# Colors
ROBOT_COLOR = (0, 255, 0)           # Green
WALL_COLOR = (255, 0, 0)            # Red
BACKGROUND_COLOR = (0, 0, 0)        # Black
SCANNED_WALL_COLOR = (200, 100, 0)  # Orange
SENSOR_BEAM_COLOR = (50, 50, 150)   # Blueish

# Sizes
ROBOT_RADIUS = 8
WALL_LENGTH = 80
WALL_THICKNESS = 3

# Robot properties
ROBOT_X = WINDOW_WIDTH // 2
ROBOT_Y = WINDOW_HEIGHT // 2
ROBOT_ANGLE = 0
ROBOT_SPEED = 3

# Map storage - each wall stores position, angle, and orientation
detected_walls = []  # Each wall: {'x': x, 'y': y, 'wall_angle': angle}

# --- Initialize Pygame ---
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("2D SLAM Robot - Walls Face the Robot")
clock = pygame.time.Clock()
font = pygame.font.Font(None, 24)
small_font = pygame.font.Font(None, 18)

# --- Connect to Maker board ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✅ Connected to Maker board on {SERIAL_PORT}")
except serial.SerialException:
    print(f"❌ ERROR: Could not open {SERIAL_PORT}")
    print("   Make sure Serial Monitor is closed and port is correct")
    sys.exit()

# --- Helper Functions ---
def get_sensor_endpoint(robot_x, robot_y, angle, distance_cm):
    """Calculate where the sensor beam hits the wall"""
    distance_px = distance_cm * DISTANCE_SCALE
    end_x = robot_x + distance_px * math.cos(math.radians(angle))
    end_y = robot_y + distance_px * math.sin(math.radians(angle))
    return end_x, end_y

def get_wall_angle(robot_angle):
    """
    Returns the angle of the wall (perpendicular to robot's viewing direction)
    If robot faces 0° (right), wall is vertical (90° or -90°)
    If robot faces 90° (down), wall is horizontal (0° or 180°)
    """
    # Wall is perpendicular to robot's facing direction
    wall_angle = robot_angle + 90  # Add 90 degrees for perpendicular
    return wall_angle

def draw_angled_wall(x, y, wall_angle, length, thickness):
    """
    Draw a wall line at any angle
    wall_angle: angle in degrees (0 = horizontal to the right)
    """
    # Calculate wall endpoints
    half_length = length // 2
    end1_x = x + half_length * math.cos(math.radians(wall_angle))
    end1_y = y + half_length * math.sin(math.radians(wall_angle))
    end2_x = x - half_length * math.cos(math.radians(wall_angle))
    end2_y = y - half_length * math.sin(math.radians(wall_angle))
    
    pygame.draw.line(screen, WALL_COLOR, (end1_x, end1_y), (end2_x, end2_y), thickness)

def add_wall_to_map(world_x, world_y, robot_angle):
    """Add a detected wall with proper perpendicular orientation"""
    wall_angle = get_wall_angle(robot_angle)
    
    # Check if similar wall already exists within 20 pixels
    for wall in detected_walls:
        dist = math.sqrt((wall['x'] - world_x)**2 + (wall['y'] - world_y)**2)
        angle_diff = abs(wall['wall_angle'] - wall_angle) % 180
        if dist < 20 and angle_diff < 15:
            return False
    
    detected_walls.append({
        'x': world_x,
        'y': world_y,
        'wall_angle': wall_angle,
        'robot_angle': robot_angle  # Store original viewing angle for reference
    })
    return True

def draw_robot(x, y, angle):
    """Draw robot with direction indicator and field of view"""
    # Draw robot body
    pygame.draw.circle(screen, ROBOT_COLOR, (int(x), int(y)), ROBOT_RADIUS)
    pygame.draw.circle(screen, (0, 100, 0), (int(x), int(y)), ROBOT_RADIUS - 2)
    
    # Draw direction line (where robot is facing)
    end_x = x + ROBOT_RADIUS * 2 * math.cos(math.radians(angle))
    end_y = y + ROBOT_RADIUS * 2 * math.sin(math.radians(angle))
    pygame.draw.line(screen, (255, 255, 255), (x, y), (end_x, end_y), 2)
    
    # Draw sensor arc (shows 60° field of view)
    for sweep_angle in [-30, -15, 0, 15, 30]:
        arc_x = x + (ROBOT_RADIUS + 8) * math.cos(math.radians(angle + sweep_angle))
        arc_y = y + (ROBOT_RADIUS + 8) * math.sin(math.radians(angle + sweep_angle))
        pygame.draw.circle(screen, (80, 80, 80), (int(arc_x), int(arc_y)), 2)

def draw_sensor_beam(robot_x, robot_y, angle, distance_cm):
    """Draw the current sensor reading"""
    if distance_cm < 400 and distance_cm > 2:
        end_x, end_y = get_sensor_endpoint(robot_x, robot_y, angle, distance_cm)
        
        # Draw beam line
        pygame.draw.line(screen, SENSOR_BEAM_COLOR, (robot_x, robot_y), (end_x, end_y), 2)
        
        # Draw hit point
        pygame.draw.circle(screen, SCANNED_WALL_COLOR, (int(end_x), int(end_y)), 6)
        
        return end_x, end_y
    return None, None

def draw_all_walls():
    """Draw all mapped walls with correct orientations"""
    for wall in detected_walls:
        draw_angled_wall(wall['x'], wall['y'], wall['wall_angle'], WALL_LENGTH, WALL_THICKNESS)

# --- Main SLAM Loop ---
current_distance = 100
auto_scan_mode = True
auto_scan_last_pos = (ROBOT_X, ROBOT_Y)  # FIXED: Separate variable for position tracking
scan_cooldown = 0

print("🎮 PERPENDICULAR WALL SLAM - Walls face the robot!")
print("")
print("   HOW IT WORKS:")
print("   - When you scan, a wall is created PERPENDICULAR to your view")
print("   - Face a wall, press S, and a red line appears FACING you")
print("   - Turn 90°, and new walls will be horizontal/vertical accordingly")
print("")
print("   CONTROLS:")
print("   ← → : Turn robot")
print("   ↑ ↓ : Move forward/backward")
print("   S   : Scan (add wall perpendicular to robot)")
print("   C   : Clear all walls")
print("   R   : Reset robot position")
print("   A   : Toggle auto-scan (current: ON)")
print("   ESC : Quit")
print("")

while True:
    # --- Handle keyboard input ---
    keys = pygame.key.get_pressed()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            ser.close()
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                ser.close()
                pygame.quit()
                sys.exit()
            elif event.key == pygame.K_c:
                detected_walls.clear()
                print("🗺️ Map cleared!")
            elif event.key == pygame.K_r:
                ROBOT_X = WINDOW_WIDTH // 2
                ROBOT_Y = WINDOW_HEIGHT // 2
                ROBOT_ANGLE = 0
                auto_scan_last_pos = (ROBOT_X, ROBOT_Y)  # Reset auto-scan position
                print("📍 Robot reset to center")
            elif event.key == pygame.K_s:
                # Manual scan - add perpendicular wall
                if 2 < current_distance < 200:
                    wall_x, wall_y = get_sensor_endpoint(ROBOT_X, ROBOT_Y, ROBOT_ANGLE, current_distance)
                    perpendicular_angle = ROBOT_ANGLE + 90
                    if add_wall_to_map(wall_x, wall_y, ROBOT_ANGLE):
                        print(f"➕ Wall added at ({int(wall_x)}, {int(wall_y)})")
                        print(f"   Robot facing: {ROBOT_ANGLE}° | Wall angle: {perpendicular_angle % 180}°")
                    else:
                        print("⚠️ Wall already exists nearby!")
                else:
                    print(f"⚠️ No valid wall detected (distance: {current_distance:.1f}cm)")
            elif event.key == pygame.K_a:
                auto_scan_mode = not auto_scan_mode
                auto_scan_last_pos = (ROBOT_X, ROBOT_Y)  # Reset position when toggling
                print(f"🔄 Auto-scan: {'ON' if auto_scan_mode else 'OFF'}")
    
    # --- Robot Movement ---
    if keys[pygame.K_LEFT]:
        ROBOT_ANGLE -= 5
    if keys[pygame.K_RIGHT]:
        ROBOT_ANGLE += 5
    if keys[pygame.K_UP]:
        ROBOT_X += ROBOT_SPEED * math.cos(math.radians(ROBOT_ANGLE))
        ROBOT_Y += ROBOT_SPEED * math.sin(math.radians(ROBOT_ANGLE))
    if keys[pygame.K_DOWN]:
        ROBOT_X -= ROBOT_SPEED * math.cos(math.radians(ROBOT_ANGLE))
        ROBOT_Y -= ROBOT_SPEED * math.sin(math.radians(ROBOT_ANGLE))
    
    # Keep robot on screen
    ROBOT_X = max(ROBOT_RADIUS, min(WINDOW_WIDTH - ROBOT_RADIUS, ROBOT_X))
    ROBOT_Y = max(ROBOT_RADIUS, min(WINDOW_HEIGHT - ROBOT_RADIUS, ROBOT_Y))
    
    # --- Read distance sensor ---
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                current_distance = float(line)
        except ValueError:
            pass
    
    # --- Auto-scan (FIXED: no attribute error now) ---
    if auto_scan_mode and scan_cooldown <= 0:
        if 2 < current_distance < 200:
            # Check if robot has moved since last auto-scan
            moved_dist = math.sqrt((ROBOT_X - auto_scan_last_pos[0])**2 + 
                                   (ROBOT_Y - auto_scan_last_pos[1])**2)
            if moved_dist > 30:  # Scan every 30 pixels of movement
                wall_x, wall_y = get_sensor_endpoint(ROBOT_X, ROBOT_Y, ROBOT_ANGLE, current_distance)
                if add_wall_to_map(wall_x, wall_y, ROBOT_ANGLE):
                    print(f"📍 Auto-scan: Wall added at ({int(wall_x)}, {int(wall_y)})")
                auto_scan_last_pos = (ROBOT_X, ROBOT_Y)
                scan_cooldown = 10
    if scan_cooldown > 0:
        scan_cooldown -= 1
    
    # --- Draw everything ---
    screen.fill(BACKGROUND_COLOR)
    
    # Draw grid
    for x in range(0, WINDOW_WIDTH, 50):
        pygame.draw.line(screen, (20, 20, 20), (x, 0), (x, WINDOW_HEIGHT), 1)
    for y in range(0, WINDOW_HEIGHT, 50):
        pygame.draw.line(screen, (20, 20, 20), (0, y), (WINDOW_WIDTH, y), 1)
    
    # Draw origin marker
    pygame.draw.circle(screen, (50, 50, 50), (WINDOW_WIDTH//2, WINDOW_HEIGHT//2), 4)
    pygame.draw.line(screen, (50, 50, 50), (WINDOW_WIDTH//2 - 10, WINDOW_HEIGHT//2), 
                    (WINDOW_WIDTH//2 + 10, WINDOW_HEIGHT//2), 1)
    pygame.draw.line(screen, (50, 50, 50), (WINDOW_WIDTH//2, WINDOW_HEIGHT//2 - 10), 
                    (WINDOW_WIDTH//2, WINDOW_HEIGHT//2 + 10), 1)
    
    # Draw all mapped walls
    draw_all_walls()
    
    # Draw current sensor beam and hit point
    hit_x, hit_y = draw_sensor_beam(ROBOT_X, ROBOT_Y, ROBOT_ANGLE, current_distance)
    
    # Draw robot
    draw_robot(ROBOT_X, ROBOT_Y, ROBOT_ANGLE)
    
    # --- HUD Display ---
    info_lines = [
        f"Distance: {current_distance:.1f} cm",
        f"Robot: ({int(ROBOT_X)}, {int(ROBOT_Y)}) @ {int(ROBOT_ANGLE)}°",
        f"Walls: {len(detected_walls)}",
        f"Auto-scan: {'ON' if auto_scan_mode else 'OFF'}",
        "",
        "Current wall angle:",
        f"  Perpendicular to {int(ROBOT_ANGLE)}° = {int(get_wall_angle(ROBOT_ANGLE)) % 180}°",
        "",
        "CONTROLS:",
        "← →  : Turn",
        "↑ ↓  : Move",
        "S    : Scan wall (perpendicular)",
        "C    : Clear map",
        "A    : Toggle auto-scan",
        "R    : Reset robot"
    ]
    
    for i, line in enumerate(info_lines):
        if line.startswith("CONTROLS:"):
            color = (150, 150, 150)
        elif line.startswith("Current wall"):
            color = (200, 200, 100)
        else:
            color = (200, 200, 200)
        text = small_font.render(line, True, color)
        screen.blit(text, (10, 10 + i * 18))
    
    # Show sensor hit info
    if hit_x and hit_y and 2 < current_distance < 200:
        hit_text = small_font.render(f"Sensor hit: ({int(hit_x)}, {int(hit_y)})", True, (255, 200, 100))
        screen.blit(hit_text, (WINDOW_WIDTH - 220, 10))
        
        # Show wall orientation preview
        preview_angle = get_wall_angle(ROBOT_ANGLE)
        preview_text = small_font.render(f"Wall angle: {int(preview_angle % 180)}°", True, (255, 200, 100))
        screen.blit(preview_text, (WINDOW_WIDTH - 220, 30))
    
    # Update display
    pygame.display.flip()
    clock.tick(60)