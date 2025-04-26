# Maze solving and Object placement with PID Control

## Overview
This project involves designing and implementing a maze solving and object placement robot that uses five TCRT1000 infrared sensors for detecting a white line on a dark surface. The robot is controlled using a PID (Proportional-Integral-Derivative) algorithm to maintain smooth and accurate line tracking.

## Major Components Used
- **Microcontroller:** ESP-32
- **Sensors:** 5x TCRT1000 IR sensors
- **Motors:** 2x DC Motors with Motor Driver (TB6612FNG)
- **Power Supply:** 3x Li-ion Battery Pack (12V)
- **Chassis:** 3-wheel robot chassis (2 motors + 1 castor wheel for stability)

## Working Principle
1. The TCRT1000 sensors detect the reflection from the white line and provide analog values to the microcontroller.
2. The readings from all five sensors are processed to determine the position of the line relative to the robot.
3. A PID control algorithm computes the required corrections in motor speed to keep the robot centered on the line by adjusting their PWM.
4. The motor driver receives the control signals and adjusts the speed of the motors accordingly.
5. For turns, the direction is determined by recording the previous 15 LSA values.
## PID Control Algorithm
The PID control system helps in minimizing oscillations and improving tracking accuracy. The formula used is:

```
Error = Setpoint - Sensor_Position
PID_output = (Kp * Error) + (Ki * Integral) + (Kd * Derivative)
```

Where:
- **Kp** (Proportional Gain): Controls the response based on the current error.
- **Ki** (Integral Gain): Adjusts for past errors to eliminate steady-state errors.
- **Kd** (Derivative Gain): Predicts future errors and dampens oscillations.


## Adjusting the PID Parameters
- Start with **Kp** to get a basic response.
- Increase **Kd** to reduce oscillations.
- Tune **Ki** to minimize steady-state error.

## Applications
- Autonomous navigation in warehouses
- Educational robotics projects
- Industrial automation

## Future Enhancements
- Implementing adaptive PID tuning
- Integrating machine learning for path optimization

## Video Demonstration of SAC Finals
- [https://drive.google.com/file/d/1jiYRGQWqHMdFfAtZDlvRXE6fJCm3Wj6X/view?usp=drive_link ](https://drive.google.com/file/d/1jwIqWakAozG5SfbpKLwxp1zVBzRboQ-u/view?usp=drive_link)
