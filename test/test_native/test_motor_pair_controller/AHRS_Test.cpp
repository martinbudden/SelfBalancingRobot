#include "AHRS_Test.h"


Quaternion SensorFusionFilterTest::update(const xyz_t& gyroRadians, const xyz_t& accelerometer, float deltaT) { return Quaternion {}; }
Quaternion SensorFusionFilterTest::update(const xyz_t& gyroRadians, const xyz_t& accelerometer, xyz_t& magnetometer, float deltaT) { return Quaternion {}; }
void SensorFusionFilterTest::setFreeParameters(float parameter0, float parameter1) {}

void IMU_Test::setGyroOffset(const xyz_int16_t& gyroOffset) {}
void IMU_Test::setAccOffset(const xyz_int16_t& accOffset) {}
xyz_int16_t IMU_Test::readGyroRaw() const { return xyz_int16_t{}; }
xyz_int16_t IMU_Test::readAccRaw() const { return xyz_int16_t{}; }
bool IMU_Test::readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const { return false; }
int IMU_Test::readFIFO_ToBuffer() { return 0; }
void IMU_Test::readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index) {}

void IMU_Filters_Test::filter(xyz_t& gyroRadians, xyz_t& acc, float deltaT) {}
