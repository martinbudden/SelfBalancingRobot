#include "IMU_Filters.h"


IMU_Filters::IMU_Filters(float frequencyCutoff, float deltaT) :
    _deltaT(deltaT),
    _filters { .gyro_lpf1_hz = static_cast<uint16_t>(frequencyCutoff) },
    _gyroX_PT1_LPF1(frequencyCutoff, deltaT),
    _gyroY_PT1_LPF1(frequencyCutoff, deltaT),
    _gyroZ_PT1_LPF1(frequencyCutoff, deltaT),
    _accX_PT1_LPF1(frequencyCutoff, deltaT),
    _accY_PT1_LPF1(frequencyCutoff, deltaT),
    _accZ_PT1_LPF1(frequencyCutoff, deltaT)
{
}

IMU_Filters::IMU_Filters() :
    _deltaT(1.0F),
    _filters { .gyro_lpf1_hz = 0 }
{
    _gyroX_PT1_LPF1.setToPassthrough();
    _gyroY_PT1_LPF1.setToPassthrough();
    _gyroZ_PT1_LPF1.setToPassthrough();

    _accX_PT1_LPF1.setToPassthrough();
    _accY_PT1_LPF1.setToPassthrough();
    _accZ_PT1_LPF1.setToPassthrough();
}

void IMU_Filters::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference; false positive
{
    (void)deltaT;
    gyroRPS.x = _gyroX_PT1_LPF1.update(gyroRPS.x);
    gyroRPS.y = _gyroY_PT1_LPF1.update(gyroRPS.y);
    gyroRPS.z = _gyroZ_PT1_LPF1.update(gyroRPS.z);
    acc.x = _accX_PT1_LPF1.update(acc.x);
    acc.y = _accY_PT1_LPF1.update(acc.y);
    acc.z = _accZ_PT1_LPF1.update(acc.z);
};

void IMU_Filters::setFilters()
{
    // Do nothing
}

void IMU_Filters::setFilters(const filters_t& filters, float deltaT)
{
    _filters = filters;

    if (filters.gyro_lpf1_hz == 0) {
        _gyroX_PT1_LPF1.setToPassthrough();
        _gyroY_PT1_LPF1.setToPassthrough();
        _gyroZ_PT1_LPF1.setToPassthrough();
    } else {
        _gyroX_PT1_LPF1.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
        _gyroY_PT1_LPF1.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
        _gyroZ_PT1_LPF1.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
    }
}