#include "AHRS_Test.h"


Quaternion SensorFusionFilterTest::update(const xyz_t& gyroRadians, const xyz_t& accelerometer, float deltaT)
{
    return Quaternion {};
}

Quaternion SensorFusionFilterTest::update(const xyz_t& gyroRadians, const xyz_t& accelerometer, xyz_t& magnetometer, float deltaT)
{
    return Quaternion {};
}

void SensorFusionFilterTest::setFreeParameters(float parameter0, float parameter1)
{
}

void AHRS_Test::setGyroOffset([[maybe_unused]] const xyz_int16_t& gyroOffset)
{
}

xyz_int16_t  AHRS_Test::readGyroRaw() const
{
    return xyz_int16_t {};
}

void AHRS_Test::setAccOffset([[maybe_unused]] const xyz_int16_t& gyroOffset)
{
}

xyz_int16_t  AHRS_Test::readAccRaw() const
{
    return xyz_int16_t {};
}

AHRS_Base::data_t AHRS_Test::getAhrsDataUsingLock() const
{
    return AHRS_Base::data_t{};
}

Quaternion AHRS_Test::getOrientationUsingLock() const
{
    return Quaternion {};
}
