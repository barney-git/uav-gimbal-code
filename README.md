# UAV Gimbal Stabilisation

Arduino firmware for a dual-axis camera gimbal, developed as part of my University Of Southampton fixed-wing UAV group project.

Decodes and unwraps IMU readings, applies an integral-only control system and outputs control signals to two servo motors (pitch and roll) to stabilise a gimballed FPV camera during flight.

## Hardware
- Arduino Nano
- LSM6DSOX 6-DOF IMU (accelerometer + gyroscope)
- 2x SG90 9g Micro Servos (Tower Pro)
