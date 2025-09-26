#include "IMU_Filters.h"


IMU_Filters::IMU_Filters(float frequencyCutoff, float deltaT) :
    _deltaT(deltaT),
    _filters { .gyro_lpf1_hz = static_cast<uint16_t>(frequencyCutoff) },
    _gyroLPF(frequencyCutoff, deltaT),
    _accLPF(frequencyCutoff, deltaT)
{
}

IMU_Filters::IMU_Filters() :
    _deltaT(1.0F),
    _filters { .gyro_lpf1_hz = 0 }
{
    _gyroLPF.setToPassthrough();
    _accLPF.setToPassthrough();
}

void IMU_Filters::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference; false positive
{
    (void)deltaT;
    gyroRPS = _gyroLPF.filter(gyroRPS);
    acc = _accLPF.filter(acc);
}

void IMU_Filters::setFilters()
{
    // Do nothing
}

void IMU_Filters::setFilters(const filters_t& filters, float deltaT)
{
    _filters = filters;
    if (filters.gyro_lpf1_hz == 0) { // NOLINT(bugprone-branch-clone)
        _gyroLPF.setToPassthrough();
    } else {
        _gyroLPF.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
    }
}