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


void Main::runIMU_Calibration(NonVolatileStorage& nonVolatileStorage, AHRS& ahrs, calibration_type_e calibrationType)
{
#if defined(USE_IMU_M5_UNIFIED)
    (void)ahrs;
    (void)nonVolatileStorage;
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
#endif
    int64_t gyroX = 0;
    int64_t gyroY = 0;
    int64_t gyroZ = 0;
    int64_t accX = 0;
    int64_t accY = 0;
    int64_t accZ = 0;

    int32_t x {};
    int32_t y {};
    int32_t z {};

    const int count = 5000;
    for (auto ii = 0; ii < count; ++ii) {
#if defined(FRAMEWORK_ARDUINO)
        delay(1);
#endif
        ahrs.readGyroRaw(x, y, z);
        gyroX += x;
        gyroY += y;
        gyroZ += z;
        ahrs.readAccRaw(x, y, z);
        accX += x;
        accY += y;
        accZ += z;
    }

    const xyz_t gyroOffset = xyz_t {
        .x = static_cast<float>(gyroX) / count,
        .y = static_cast<float>(gyroY) / count,
        .z = static_cast<float>(gyroZ) / count
    } * ahrs.getIMU().getAccResolution();

    xyz_t accOffset = {
        .x = static_cast<float>(accX) / count,
        .y = static_cast<float>(accY) / count,
        .z = static_cast<float>(accZ) / count
    };

    const float oneG = 1.0F / ahrs.getIMU().getAccResolution();
    const float halfG = oneG / 2.0F;
    if (accOffset.x > halfG) {
        accOffset.x -= oneG;
    } else if (accOffset.x < - halfG) {
        accOffset.x += oneG;
    } else if (accOffset.y > halfG) {
        accOffset.y -= oneG;
    } else if (accOffset.y < - halfG) {
        accOffset.y += oneG;
    } else if (accOffset.z > halfG) {
        accOffset.z -= oneG;
    } else if (accOffset.z < - halfG) {
        accOffset.z += oneG;
    }
    accOffset *= ahrs.getIMU().getAccResolution();

#if defined(M5_STACK) || defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(gyroOffset.x), static_cast<double>(gyroOffset.y), static_cast<double>(gyroOffset.z));
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%f y:%f z:%f\r\n\r\n", static_cast<double>(accOffset.x), static_cast<double>(accOffset.y), static_cast<double>(accOffset.z));
    }
#endif

    nonVolatileStorage.storeGyroOffset(gyroOffset);
    nonVolatileStorage.storeGyroCalibrationState(NonVolatileStorage::CALIBRATED);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        nonVolatileStorage.storeAccOffset(accOffset);
        nonVolatileStorage.storeAccCalibrationState(NonVolatileStorage::CALIBRATED);
    }
}

void Main::calibrateIMU(NonVolatileStorage& nonVolatileStorage, AHRS& ahrs, calibration_type_e calibrationType)
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

    runIMU_Calibration(nonVolatileStorage, ahrs, calibrationType);

#if defined(M5_STACK) || defined(M5_UNIFIED)
    M5.Lcd.printf("Finished calibration\r\n");
#endif

#if defined(FRAMEWORK_ARDUINO)
    delay(4000); // delay 4 seconds to allow user to read screen
#endif

#if defined(M5_STACK)
    M5.Power.reset();
#elif defined(M5_UNIFIED)
    //M5.Power.powerOff();
    M5.Power.timerSleep(0); // sleep for zero seconds and reboot
#endif
}
