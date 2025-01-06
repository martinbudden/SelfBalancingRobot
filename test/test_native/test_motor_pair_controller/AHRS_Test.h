#pragma once

#include "AHRS_Base.h"

class SensorFusionFilterTest : public SensorFusionFilterBase {
public:
    virtual Quaternion update(const xyz_t& gyroRadians, const xyz_t& accelerometer, float deltaT) override;
    virtual Quaternion update(const xyz_t& gyroRadians, const xyz_t& accelerometer, xyz_t& magnetometer, float deltaT) override;
    virtual void setFreeParameters(float parameter0, float parameter1) override;
};

class AHRS_Test : public AHRS_Base {
public:
    AHRS_Test() : AHRS_Base(_sensorFusionFilter) {}
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual void setAccOffset(const xyz_int16_t& gyroOffset) override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual AHRS_Base::data_t getAhrsDataUsingLock() const override;
    virtual Quaternion getOrientationUsingLock() const override;
};
