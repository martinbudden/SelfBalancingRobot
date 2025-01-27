#include "Calibration.h"

#include <AHRS.h>
#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include <SV_Preferences.h>


#if !defined(M5_UNIFIED) || defined(USE_IMU_MPU6886)
static void calibrate(AHRS& ahrs, SV_Preferences& preferences, calibrate_t calibrationType)
{
    int64_t gyroX = 0;
    int64_t gyroY = 0;
    int64_t gyroZ = 0;
    int64_t accX = 0;
    int64_t accY = 0;
    int64_t accZ = 0;

    int32_t x {};
    int32_t y {};
    int32_t z {};

    const int count = 50;
    for (auto ii = 0; ii < count; ++ii) {
        delay(2);
        ahrs.readGyroRaw(x, y, z);
        gyroX += x;
        gyroY += y;
        gyroZ += z;
        ahrs.readAccRaw(x, y, z);
        accX += x;
        accY += y;
        accZ += z;
    }

    const auto gyroOffset_x = static_cast<int32_t>(gyroX / count);
    const auto gyroOffset_y = static_cast<int32_t>(gyroY / count);
    const auto gyroOffset_z = static_cast<int32_t>(gyroZ / count);

    auto accOffset_x = static_cast<int32_t>(accX / count);
    auto accOffset_y = static_cast<int32_t>(accY / count);
    auto accOffset_z = static_cast<int32_t>(accZ / count);

    const int32_t oneG = ahrs.getAccOneG_Raw();
    const int32_t halfG = oneG / 2;
    if (accOffset_x > halfG) {
        accOffset_x -= oneG;
    } else if (accOffset_x < - halfG) {
        accOffset_x += oneG;
    } else if (accOffset_y > halfG) {
        accOffset_y -= oneG;
    } else if (accOffset_y < - halfG) {
        accOffset_y += oneG;
    } else if (accOffset_z > halfG) {
        accOffset_z -= oneG;
    } else if (accOffset_z < - halfG) {
        accOffset_z += oneG;
    }

    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", gyroOffset_x, gyroOffset_y, gyroOffset_z);
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", accOffset_x, accOffset_y, accOffset_z);
    }
    preferences.putGyroOffset(gyroOffset_x, gyroOffset_y, gyroOffset_z);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        preferences.putAccOffset(accOffset_x, accOffset_y, accOffset_z);
    }
}

#elif defined(M5_UNIFIED)
static void calibrate()
{
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
}
#endif

void calibrateGyro([[maybe_unused]] AHRS& ahrs, [[maybe_unused]] SV_Preferences& preferences, [[maybe_unused]] calibrate_t calibrationType)
{
    if (M5.Lcd.width() > 300) {
        M5.Lcd.setTextSize(2);
    }
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.fillScreen(TFT_BLACK);

    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Starting  gyro calibration\r\n");
    M5.Lcd.printf("Please keep the robot\r\n");
    M5.Lcd.printf("still for 10 seconds\r\n\r\n");
    delay(3000);

#if defined(M5_STACK) || defined(USE_IMU_MPU6886)
    calibrate(ahrs, preferences, calibrationType);
#elif defined(M5_UNIFIED)
    calibrate();
#endif

    M5.Lcd.printf("Finished calibration\r\n");
    delay(4000);

#if defined(M5_STACK)
    M5.Power.reset();
#elif defined(M5_UNIFIED)
    M5.Power.powerOff();
#endif
}
