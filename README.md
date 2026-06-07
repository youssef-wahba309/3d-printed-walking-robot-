# 3d-printed-walking-robot-
# Development and Testing of a 3D Printed Walking Robot

This repository contains the Arduino firmware used for the graduation research project:

**Development and Testing of a 3D Printed Walking Robot**

The project implements a 19-DOF humanoid bipedal walking robot using:
- ESP32 microcontroller
- Two PCA9685 PWM servo drivers
- 19 MG996R servo motors
- MPU6050 IMU
- Kalman filter for pitch estimation
- Predefined joint-angle walking sequences
- IMU-based fall detection and neutral recovery

## Control Approach

The robot uses predefined joint-angle walking poses. Therefore, the walking gait is mainly open-loop. The MPU6050 IMU and Kalman filter are used for safety monitoring and fall detection, not for continuous gait correction.

## Repository Contents

- `main_code/humanoid_robot_main.ino`  
  Main Arduino code used to control the robot.

- `code_snippets/servo_motion_function.ino`  
  Smooth servo interpolation function.

- `code_snippets/kalman_filter_function.ino`  
  IMU reading and Kalman filter update function.

- `code_snippets/fall_detection_logic.ino`  
  Fall detection and return-to-neutral logic.

- `documentation/code_description.md`  
  Explanation of the firmware structure.

## Notes

This code was developed as part of a BUE Robotics Engineering individual research project. The mechanical structure was 3D printed using PLA and controlled using ESP32-based embedded software.
