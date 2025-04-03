#include "Calibration.h"

#include <AHRS.h>
#include <SV_Preferences.h>


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

    const int count = 5000;
    for (auto ii = 0; ii < count; ++ii) {
        delay(1);
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

    preferences.putGyroOffset(gyroOffset_x, gyroOffset_y, gyroOffset_z);
    if (calibrationType == CALIBRATE_ACC_AND_GYRO) {
        preferences.putAccOffset(accOffset_x, accOffset_y, accOffset_z);
    }
}

void calibrateGyro(AHRS& ahrs, SV_Preferences& preferences, calibrate_t calibrationType)
{
    delay(4000); // delay 4 seconds to allow robot to stabilize after user lets go

    calibrate(ahrs, preferences, calibrationType);
}
