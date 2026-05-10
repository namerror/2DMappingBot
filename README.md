# 2D SLAM Robot Prototype

## Introduction
<img align="right" src="/assets/robot_front.jpg"/>

This project is a prototype of a low-cost 2D SLAM (Simultaneous Localization and Mapping) robot designed to navigate and map a maze-like environment. The robot is built using affordable components, including an ultrasonic distance sensor for mapping nearby walls and an IMU (Inertial Measurement Unit) for estimating its position and orientation. The collected sensor data is streamed to a laptop, where a live visualization displays a rough 2D map of the maze in real time. This project serves as a proof of concept for using simple robotics and sensing technologies to achieve real-time environmental mapping without relying on expensive sensors or complex algorithms.

<img src="/assets/robot_top.jpg"/> <img src="/assets/robot_in_action.png"/>

![Demo](/assets/demo.gif)

## Setup Instructions

### Python Environment

1. To use a virtual environment:
```bash
python -m venv .venv
source .venv/bin/activate
```

2. Set up the required python dependencies:
```bash
pip install -r requirements.txt
```

### PlatformIO Setup

If you want to work with PlatformIO, you can follow these steps:

1. Install PlatformIO IDE in Visual Studio Code.
2. Open the project folder in Visual Studio Code.
3. Open the `platformio.ini` file and ensure that the correct board and environment settings are configured for your hardware.
4. Build and upload the code to your microcontroller using the Platform

### Other Dependencies
If you set up the project without using PlatformIO, make sure to install the following libraries:
- [jrowberg/I2Cdevlib-MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) for the MPU6050 IMU sensor.
- [esp8266-oled-ssd1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) for the OLED display (if used).

## Project Materials

Below is a list of the materials used in this project:
- MPU6050 IMU
- Arduino + ESP32 (we used a maker board with both)
- 4 AA Batteries + Battery holder for motor power
- 2 Wheels
- Breadboard
- Capacitors
- TB6612FNG Motor driver
- 2 DC motors
- Robot Chassis
- HC-SR04 Ultrasonic Distance Sensor
- Solo Wheel

## Running the Project
1. Connect the hardware components according to the schematics provided in the `KiCAD/`
2. Upload the code to the microcontroller using PlatformIO or your preferred method.
3. Configure the control settings as needed (see [Control Configuration](#control-configuration) below).
4. Run the Python script to start controlling and the SLAM process.

    ```bash
    python main.py
    ```

## Project Components

The project consists of the following components:
- [Main Program Files](#main-program-files)
- [Firmware](#firmware)
- [Configuration Files](#configuration-files)
- [Test File](#test-file)
- [Hardware Design Files](#hardware-design-files)
- [Reference Documents](#reference-documents)

The main application is controlled by a small group of files that each handle one part of the system. A new user can think of the project as three layers:

- The firmware on the robot reads sensors and drives the motors.
- The Python desktop app sends commands, reads live data, and draws the map.
- The configuration files tell both sides what hardware and behavior to expect.

### Main Program Files

#### `main.py`
This is the main desktop application and the file most users will run first with `python main.py`.

It opens the control window, connects to the robot over serial, loads settings from `control_config.yaml`, and starts the live mapping view. It also handles keyboard and on-screen button input, requests IMU calibration when needed, reads distance data from the robot, and adds detected walls to the map.

If you want to understand the overall software flow of the project, this is the best file to start with.

#### `robot_serial.py`
This file is the communication layer between Python and the robot.

It opens the serial port, sends short motor-control commands such as forward, backward, left, right, brake, and speed changes, then parses the lines of telemetry sent back by the microcontroller. It also handles IMU calibration status and stores the latest distance and IMU readings in a structured way so the rest of the app can use them safely.

In simple terms, `main.py` decides what should happen, and `robot_serial.py` is the messenger that talks to the robot.

#### `pose_estimator.py`
This file estimates where the robot is on the 2D map.

It supports two pose modes:

- `command_estimate`: estimates movement from the commands being sent to the motors.
- `imu_estimate`: estimates movement from IMU acceleration and gyro data.

This is important because the robot does not use wheel encoders. Instead, the software makes its best guess about position using either expected movement or live IMU readings. The estimated pose is then used to place scanned wall points in the map window.

#### `robot_config.py`
This file loads and validates the settings from `control_config.yaml`.

It defines the configuration structure used by the Python app, provides default values, and checks for invalid options such as unsupported pose modes or duplicate IMU axes. Keeping the parsing logic here keeps `main.py` cleaner and makes configuration errors easier to understand.

### Firmware

#### `Robot.ino`
This is the microcontroller program that runs on the robot itself.

It controls the motors through the TB6612FNG driver, reads distance from the HC-SR04 ultrasonic sensor, reads motion data from the MPU6050 IMU, and sends sensor updates back to the laptop over serial. It also performs IMU calibration, applies optional motor speed ramping, updates the OLED display, and listens for commands from the Python app.

From a beginner's point of view, this file is the robot's "body-level" logic, while `main.py` is the "operator console" running on the computer.

### Configuration Files

#### `control_config.yaml`
This is the main user-editable settings file for the project.

If the robot connects to the wrong serial port, turns the wrong way, uses the wrong sensor direction, or feels too fast or too slow, this is usually the first file to adjust.

The sections mean:

- `serial`: chooses the serial port, baud rate, and timeout used by the Python app to talk to the robot.
- `control`: sets command timing, default PWM motor speed, brake timing, and optional wheel ramp behavior.
- `sensor`: defines the angle offset of the ultrasonic sensor relative to the robot's forward direction.
- `pose`: chooses whether the app estimates pose from commands or from the IMU.
- `pose.command_estimate`: contains approximate physical values such as wheel diameter, track width, and expected motor RPM for command-based movement estimation.
- `pose.imu_estimate`: contains IMU axis directions and filters used to make IMU-based pose estimation more stable.

This file is especially important because the code is built to be reusable across slightly different robot builds. Instead of hard-coding all hardware assumptions in Python, those assumptions live here.

#### `platformio.ini`
This is the PlatformIO project configuration for the microcontroller firmware.

It tells PlatformIO which board to build for, which framework to use, what serial monitor speed to expect, and which Arduino libraries must be installed. In this repository it is set up for an `esp32dev` target, uses the Arduino framework, and pulls in the OLED and MPU6050 dependencies used by `Robot.ino`.

If the firmware does not build or uploads for the wrong board, this is the file to inspect first.

#### `requirements.txt`
This file lists the Python packages needed by the desktop application.

### Test File

#### `test_pose_estimation.py`
This file contains automated tests for the Python-side logic.

It checks configuration loading, validates wheel-ramp settings, verifies that serial control commands are sent correctly, and tests the pose estimator in both command-based and IMU-based modes. For a new contributor, this file is useful because it shows the expected behavior of the configuration and motion-estimation code in a compact form.

### Hardware Design Files

#### `KiCAD/mapping_bot_schematics/mapping_bot_schematics.kicad_sch`
This is the main KiCad schematic file. It shows how the electrical parts are connected, including the controller board, motor driver, sensors, and power wiring.

#### `KiCAD/mapping_bot_schematics/mapping_bot_schematics.kicad_pcb`
This is the KiCad PCB layout file. It is used if you want to move from a breadboard-style prototype to a designed circuit board layout.

#### `KiCAD/mapping_bot_schematics/mapping_bot_schematics.kicad_pro`
This is the main KiCad project file. It stores project-level settings and ties the schematic and PCB files together inside KiCad.

#### `KiCAD/mapping_bot_schematics/mapping_bot_schematics.kicad_prl`
This is a KiCad project local settings file. It usually stores local editor and view preferences for the project.

#### `KiCAD/mapping_bot_schematics/Mappingbot.bak`
This is a backup file created during schematic work. It can be useful as a recovery copy if changes were made accidentally.

### Reference Documents

#### `References/MPU6050.pdf`
Reference document for the MPU6050 IMU used by the robot. This helps when checking sensor behavior, pinout, or measurement details.

#### `References/SparkFun_Motor_Driver-TB6612FNG_v11c.pdf`
Reference document for the SparkFun TB6612FNG motor driver board version used as a guide in the project.

#### `References/TB6612FNG.pdf`
Reference document for the TB6612FNG chip itself. This is useful when you want lower-level electrical details than the SparkFun board guide provides.
