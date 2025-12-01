#include "Main.h"

#include <AHRS.h>
#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#elif defined(FRAMEWORK_ARDUINO)
#if !defined(FRAMEWORK_ARDUINO_ESP32)
#include <Arduino.h>
#endif
#endif

#include <NonVolatileStorage.h>


void Main::calibrateIMUandSave(NonVolatileStorage& nonVolatileStorage, IMU_Base& imu, IMU_Base::calibration_type_e calibrationType)
{
#if defined(M5_STACK) || defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.setTextSize(2);
    }
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.fillScreen(TFT_BLACK);

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Starting gyro calibration\r\n");
    M5.Lcd.printf("Please keep the robot\r\n");
    M5.Lcd.printf("still for 10 seconds\r\n\r\n");
#endif

#if defined(FRAMEWORK_ARDUINO)
    delay(4000); // delay 4 seconds to allow robot to stabilize after user lets go
#endif

#if defined(USE_IMU_M5_UNIFIED)

    (void)nonVolatileStorage;
    (void)imu;
    (void)calibrationType;
    // Strength of the calibration operation;
    // 0: disables calibration.
    // 1 is weakest and 255 is strongest.
    enum { CALIBRATION_STRENGTH = 128 };
    M5.Imu.setCalibration(0, CALIBRATION_STRENGTH, 0); // just calibrate the gyro
    for (auto ii = 0; ii < 10; ++ii) {
        M5.Imu.update();
        delay(1000); // 1000ms=1s
        M5.Imu.setCalibration(0, CALIBRATION_STRENGTH, 0); // just calibrate the gyro
    }
    M5.Imu.setCalibration(0, 0, 0);
    M5.Imu.saveOffsetToNVS();
    M5.Lcd.printf("Finished calibration\r\n");
    //M5.Power.powerOff();
    M5.Power.timerSleep(5); // sleep for 5 seconds and reboot

#else

    enum { CALIBRATION_COUNT = 5000 };
    imu.calibrate(calibrationType, CALIBRATION_COUNT);
    const xyz_t gyroOffset = imu.getGyroOffset();
    const xyz_t accOffset = imu.getAccOffset();
    nonVolatileStorage.storeGyroOffset(gyroOffset);
    nonVolatileStorage.storeGyroCalibrationState(NonVolatileStorage::CALIBRATED);
    if (calibrationType == IMU_Base::CALIBRATE_ACC_AND_GYRO) {
        nonVolatileStorage.storeAccOffset(accOffset);
        nonVolatileStorage.storeAccCalibrationState(NonVolatileStorage::CALIBRATED);
    }
#if defined(M5_STACK)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(gyroOffset.x), static_cast<double>(gyroOffset.y), static_cast<double>(gyroOffset.z));
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(accOffset.x), static_cast<double>(accOffset.y), static_cast<double>(accOffset.z));
        M5.Lcd.printf("Finished calibration\r\n");
    }
#if defined(FRAMEWORK_ARDUINO)
    delay(5000); // delay 5 seconds to allow user to read screen
#endif
    M5.Power.reset();
#endif

#endif
}
