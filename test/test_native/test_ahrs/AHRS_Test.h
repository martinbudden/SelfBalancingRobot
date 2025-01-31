#pragma once

#include "IMU_Base.h"
#include "IMU_FiltersBase.h"
#include "SensorFusionFilter.h"


class SensorFusionFilterTest : public SensorFusionFilterBase {
public:
    virtual Quaternion update(const xyz_t& gyroRPS, const xyz_t& accelerometer, float deltaT) override;
    virtual Quaternion update(const xyz_t& gyroRPS, const xyz_t& accelerometer, const xyz_t& magnetometer, float deltaT) override;
    virtual void setFreeParameters(float parameter0, float parameter1) override;
};

class IMU_Test : public IMU_Base {
public:
    IMU_Test() : IMU_Base(XPOS_YPOS_ZPOS, nullptr) {}
public:
    virtual void setGyroOffset(const xyz_int32_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int32_t& accOffset) override;

    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual gyroRPS_Acc_t readGyroRPS_Acc() override;

    virtual size_t readFIFO_ToBuffer() override;
    virtual gyroRPS_Acc_t  readFIFO_Item(size_t index) override;
};

class IMU_Filters_Test : public IMU_FiltersBase {
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
};