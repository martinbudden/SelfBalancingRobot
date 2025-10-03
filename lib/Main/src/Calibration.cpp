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

#include <SV_Preferences.h>


void Main::runIMU_Calibration(SV_Preferences& preferences, AHRS& ahrs)
{
#if defined(USE_IMU_M5_UNIFIED)
    (void)ahrs;
    (void)preferences;
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

#if defined(M5_STACK) || defined(M5_UNIFIED)
    if (M5.Lcd.width() > 300) {
        M5.Lcd.printf("gyro offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", static_cast<int>(gyroOffset_x), static_cast<int>(gyroOffset_y), static_cast<int>(gyroOffset_z));
        M5.Lcd.printf("acc offsets\r\n");
        M5.Lcd.printf("x:%5d y:%5d z:%5d\r\n\r\n", static_cast<int>(accOffset_x), static_cast<int>(accOffset_y), static_cast<int>(accOffset_z));
    }
#endif

    preferences.putGyroOffset(gyroOffset_x, gyroOffset_y, gyroOffset_z);
    preferences.putAccOffset(accOffset_x, accOffset_y, accOffset_z);
}

void Main::calibrateIMU(SV_Preferences& preferences, AHRS& ahrs)
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

    runIMU_Calibration(preferences, ahrs);

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
