# UAV Gimbal Stabilisation

Arduino firmware for a dual-axis camera gimbal, developed as part of my University Of Southampton fixed-wing UAV group project.

Decodes and unwraps IMU readings, applies an integral-only control system and outputs control signals to two servo motors (pitch and roll) to stabilise a gimballed FPV camera during flight.

## Hardware
- Arduino Nano
- LSM6DSOX 6-DOF IMU (accelerometer + gyroscope)
- 2x SG90 9g Micro Servos (Tower Pro)

## Files
- "integral-only-controller.ino" - Final, tuned gimbal firmware using an integral-only controller
- "Deprecated/pid-controller.ino" - Early, PID controller approach, discarded due to steady-state-error
- "Tools/default-servo-pos.ino" - Utility to return both servos to their 0° position
