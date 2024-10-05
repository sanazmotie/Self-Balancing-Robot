# Self-Balancing Robot with Arduino

## Project Overview

This project was developed as part of a **Mechatronic Course** to design and implement a two-wheeled, self-balancing robot. The robot uses an MPU6050 sensor for motion detection and balance control, along with a PID controller to adjust the movement of the motors. The primary objective is to maintain the robot's balance by adjusting motor speeds based on the robot's tilt angle.

## Features

- **MPU6050 6-Axis Motion Sensor**: Measures the robot's yaw, pitch, and roll angles to detect its tilt.
- **PID Control System**: Ensures stability by dynamically adjusting motor speeds based on sensor feedback.
- **Motor Controller (L298N)**: Controls two DC motors to correct the robot's position and maintain balance.
- **I2C Communication**: Communicates with the MPU6050 sensor via I2C protocol for real-time data acquisition.

## Components

- **Arduino Uno**: The microcontroller used for controlling the system.
- **MPU6050 Sensor**: A 6-axis accelerometer and gyroscope used to sense the robot's orientation.
- **L298N Motor Driver**: Controls the speed and direction of two DC motors.
- **DC Motors**: Motors that drive the wheels of the robot to adjust its position.

## Libraries Used

- `PID_v1.h`: A PID controller library for balancing the robot.
- `LMotorController.h`: A custom motor controller library for controlling two DC motors.
- `I2Cdev.h` & `MPU6050_6Axis_MotionApps20.h`: Libraries for interfacing with the MPU6050 sensor and handling the motion data processing.

## System Setup

The robot is designed to remain balanced by adjusting the motor speed based on real-time feedback from the MPU6050 sensor. The sensor data is passed through the PID controller, which calculates the necessary adjustments to keep the robot upright.

### PID Tuning Parameters

- **Kp (Proportional Gain)**: 60  
- **Ki (Integral Gain)**: 270  
- **Kd (Derivative Gain)**: 2.2  

These values were tuned experimentally to ensure the robot remains stable.

## How It Works

1. **Initialization**: The MPU6050 sensor is initialized, and the PID controller parameters are set up.
2. **Sensor Feedback**: The MPU6050 continuously provides yaw, pitch, and roll data. The pitch data (`ypr[1]`) is used to determine the tilt of the robot.
3. **PID Control**: The pitch value is processed through the PID controller to compute the necessary motor adjustments.
4. **Motor Control**: Based on the PID output, the motors are driven forward or backward to maintain balance. The `LMotorController` library ensures smooth motor operation by managing PWM signals.

## Circuit Diagram

- **Arduino Uno**: Connects to the MPU6050 via I2C (SDA to A4, SCL to A5).
- **MPU6050**: Sends motion data to the Arduino.
- **Motor Driver (L298N)**: Connects the Arduino to two DC motors (one for each wheel). The motors are powered by an external power supply.

## Code

The main logic of the project is implemented in the `loop()` function, which checks for interrupts from the MPU6050 sensor, reads the orientation data, and passes it to the PID controller. The motor speeds are adjusted based on the controller's output to keep the robot balanced.

```cpp
void loop() {
    // ... 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
    // ...
}
