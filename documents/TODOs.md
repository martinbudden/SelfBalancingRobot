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
