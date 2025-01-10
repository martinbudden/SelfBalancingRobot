#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif
#include "AHRS_Base.h"
#include "Calibration.h"
#include "SBR_Preferences.h"


#if !defined(M5_UNIFIED) || defined(USE_IMU_MPU6886_DIRECT)
static void calibrate(AHRS_Base& ahrs, SBR_Preferences& preferences, calibrate_t calibrationType)
{
    int64_t gyroX = 0;
    int64_t gyroY = 0;
    int64_t gyroZ = 0;
    int64_t accX = 0;
    int64_t accY = 0;
    int64_t accZ = 0;

    const int count = 1000;
    for (auto ii = 0; ii < count; ++ii) {
        delay(2);
        const xyz_int16_t gyro = ahrs.readGyroRaw();
        gyroX += gyro.x;
        gyroY += gyro.y;
        gyroZ += gyro.z;
        const xyz_int16_t acc = ahrs.readAccRaw();
        accX += acc.x;
        accY += acc.y;
        accZ += acc.z;
    }

    const xyz_int16_t gyroOffset {
        .x = static_cast<int16_t>(gyroX / count),
        .y = static_cast<int16_t>(gyroY / count),
        .z = static_cast<int16_t>(gyroZ / count),
    };

    xyz_int16_t accOffset {
        .x = static_cast<int16_t>(accX / count),
        .y = static_cast<int16_t>(accY / count),
        .z = static_cast<int16_t>(accZ / count),
    };
    constexpr int16_t oneG = 4096;
    constexpr int16_t halfG = 2048;
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

    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", gyroOffset.x, gyroOffset.y, gyroOffset.z);
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", accOffset.x, accOffset.y, accOffset.z);
    }
    preferences.putGyroOffset(gyroOffset);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        preferences.putAccOffset(accOffset);
    }
}

#elif defined(M5_UNIFIED)
static void calibrate()
{
    // Strength of the calibration operation;
    // 0: disables calibration.
    // 1 is weakest and 255 is strongest.
    enum { CALIBRATION_STRENGTH = 64 };
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

void calibrateGyro([[maybe_unused]] AHRS_Base& ahrs, [[maybe_unused]] SBR_Preferences& preferences, [[maybe_unused]] calibrate_t calibrationType)
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
    delay(2000);

#if defined(M5_STACK) || defined(USE_IMU_MPU6886_DIRECT)
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
