#pragma once

#include "IMU_Base.h"
#include "SensorFusionFilter.h"
#include "IMU_FiltersBase.h"


class SensorFusionFilterTest : public SensorFusionFilterBase {
public:
    virtual Quaternion update(const xyz_t& gyroRPS, const xyz_t& accelerometer, float deltaT) override;
    virtual Quaternion update(const xyz_t& gyroRPS, const xyz_t& accelerometer, xyz_t& magnetometer, float deltaT) override;
    virtual void setFreeParameters(float parameter0, float parameter1) override;
};

class IMU_Test : public IMU_Base {
public:
    IMU_Test() : IMU_Base(nullptr) {}
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;

    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;

    virtual bool readGyroRPS_Acc(xyz_t& gyroRPS, xyz_t& acc) const override;

    virtual int readFIFO_ToBuffer() override;
    virtual void readFIFO_Item(xyz_t& gyroRPS, xyz_t& acc, size_t index) override;
};

class IMU_Filters_Test : public IMU_FiltersBase {
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
};