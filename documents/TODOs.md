# TODOs

List of potential TODOs for Self Balancing Robot. Not all of these will be implemented.

1. Additional test code for PID and make `update` without `measurementDelta` inline wrapper of `update` with `measurementDelta`.
2. Implement and test variant using IMU FIFO with increased update rate.
3. Implement and test single core version.
4. Implement version for BBC micro:bit.
5. Implement version for Raspberry Pi Pico.
6. Investigate using microsecond timer for calculating `deltaT`.
7. Implement variant that uses semaphore set in IMU data ready interrupt.
8. Get ODrive versions going.
9. Get TWAI versions going.
10. Writer driver for LSM6DSOX IMU.
