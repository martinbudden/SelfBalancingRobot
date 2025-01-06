# Code Overview

This document provides an overview of the Self Balancing Robot Framework code.

## Overall structure

The software contains the following top level objects:

1. AHRS (Attitude and Heading Reference System)
2. MotorPairController (MPC)
3. Receiver
4. Backchannel
5. Screen
6. Buttons

The objects are statically created in the `setup()` function of `main.cpp`

## Task structure

There are three main tasks:

1. The main loop task (MAIN_LOOP_TASK)
2. The AHRS task (AHRS_TASK)
3. The MPC task (MPC_TASK)

The MAIN_LOOP_TASK is automatically created by the Arduino ESP32 framework. The AHRS_TASK and MPC_TASK are statically created in the `setup()` function
of `main.cpp`. Additionally there are an number of
[ESP32 FreeRTOS background tasks](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html#background-tasks)
created by the Arduino ESP32 framework

The main loop task handles:

1. Input from the receiver(joystick), buttons, and backchannel(PID tuning).
2. Output to the screen and backchannel(telemetry).
3. Failsafe when contact is lost with the transmitter.

The AHRS task reads the values from the IMU and converts them to roll, pitch, and yaw values.

The MPC task uses 3 PIDs (a pitch PID, a speed PID, and a yaw rate PID) to set the power output to the motor pair, based on:

1. setpoints set by inputs from the receiver (which are themselves derived from the joystick positions)
2. the orientation of the self balancing robot (derived from the AHRS)
3. if available, the speed of the self balancing robot (derived from encoders on the motors)
