# TODOs

List of potential TODOs for Self Balancing Robot. Not all of these will be implemented.

1. Implement and test variant using IMU FIFO with increased update rate.
2. Implement and test single core version.
3. Implement version for BBC micro:bit.
4. Implement version for Raspberry Pi Pico.
5. Investigate using microsecond timer for calculating `deltaT`.
6. Implement variant that uses a semaphore that is set in IMU data ready interrupt.
7. Get ODrive versions going.
8. Get TWAI versions going.
9. Write driver for LSM6DSOX IMU.
10. Write driver for BMI270 IMU.
11. Implement AtomS3R variant, uses BMI270. AtomS3R has IMU using pins G0 and G45, whereas AtomMotionBase attaches to pins G38 and G39, so no I2C mutex is required.
    It also has the interrupt pin connected.
12. Add setGain function to Madgwick filter.
13. Ramp up beta on Madgwick filter.
14. Rename SensorFusionFilter.* SensorFusion.*

## Library release order

1. IMU_TYPES
2. PIDF
3. Filters
4. SensorFusion (depends on IMU_TYPES)
5. AtomJoyStickReceiver
6. IMU (depends on IMU_TYPES)
7. StabilizedVehicle (depends on IMU_TYPES, PIDF, Filters, SensorFusion, IMU)

To remain in this repository:

1. MotorPairs
2. SBR_M5Stack
3. SelfBalancingRobot

