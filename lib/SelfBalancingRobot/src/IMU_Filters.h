#pragma once

#include <Filters.h>
#include <IMU_FiltersBase.h>
#include <xyz_type.h>

/*!
Simple set of filters with a LPF on each of the gyro and accelerometer axes.
*/
class IMU_Filters : public IMU_FiltersBase {
public:
    struct filters_t {
        uint16_t gyro_lpf1_hz;
    };
public:
    IMU_Filters(float frequencyCutoff, float deltaT);
    IMU_Filters();
public:
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
    virtual void setFilters() override;
    void setFilters(const filters_t& filters, float deltaT);
protected:
    float _deltaT;
    filters_t _filters;
    PowerTransferFilter1 _gyroX_PT1_LPF1 {};
    PowerTransferFilter1 _gyroY_PT1_LPF1 {};
    PowerTransferFilter1 _gyroZ_PT1_LPF1 {};
    PowerTransferFilter1 _accX_PT1_LPF1 {};
    PowerTransferFilter1 _accY_PT1_LPF1 {};
    PowerTransferFilter1 _accZ_PT1_LPF1 {};
};