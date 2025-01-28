#include "AHRS_Test.h"


Quaternion SensorFusionFilterTest::update([[maybe_unused]] const xyz_t& gyroRPS, [[maybe_unused]] const xyz_t& accelerometer, [[maybe_unused]] float deltaT) { return Quaternion {}; }
Quaternion SensorFusionFilterTest::update([[maybe_unused]] const  xyz_t& gyroRPS, [[maybe_unused]] const  xyz_t& accelerometer, [[maybe_unused]] const xyz_t& magnetometer, [[maybe_unused]] float deltaT) { return Quaternion {}; }
void SensorFusionFilterTest::setFreeParameters(float parameter0, float parameter1) {}

void IMU_Test::setGyroOffset([[maybe_unused]] const  xyz_int32_t& gyroOffset) {}
void IMU_Test::setAccOffset([[maybe_unused]] const  xyz_int32_t& accOffset) {}
IMU_Base::xyz_int32_t IMU_Test::readGyroRaw() const { return xyz_int32_t{}; }
IMU_Base::xyz_int32_t IMU_Test::readAccRaw() const { return xyz_int32_t{}; }

xyz_t IMU_Test::readGyroRPS() const { return xyz_t{}; }
xyz_t IMU_Test::readGyroDPS() const { return xyz_t{}; }
xyz_t IMU_Test::readAcc() const { return xyz_t{}; }
IMU_Base::gyroRPS_Acc_t IMU_Test::readGyroRPS_Acc() const { return gyroRPS_Acc_t{}; }

size_t IMU_Test::readFIFO_ToBuffer() { return 0; }
IMU_Base::gyroRPS_Acc_t  IMU_Test::readFIFO_Item([[maybe_unused]] size_t index) { return gyroRPS_Acc_t{}; }

void IMU_Filters_Test::filter([[maybe_unused]] xyz_t& gyroRPS, [[maybe_unused]] xyz_t& acc, [[maybe_unused]] float deltaT) {}


