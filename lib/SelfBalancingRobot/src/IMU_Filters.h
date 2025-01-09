#pragma once

#include <Filters.h>
#include <xyz_type.h>


class IMU_Filters {
public:
    IMU_Filters(float frequencyCutoff, float deltaT) :
        _deltaT(deltaT),
        _filterGyroX(frequencyCutoff, deltaT),
        _filterGyroY(frequencyCutoff, deltaT),
        _filterGyroZ(frequencyCutoff, deltaT),
        _filterAccX(frequencyCutoff, deltaT),
        _filterAccY(frequencyCutoff, deltaT),
        _filterAccZ(frequencyCutoff, deltaT)
    {}
public:
public:
    void filter(xyz_t& gyroRadians, xyz_t& acc, float deltaT);
private:
    float _deltaT;
    IIR_filter _filterGyroX;
    IIR_filter _filterGyroY;
    IIR_filter _filterGyroZ;
    IIR_filter _filterAccX;
    IIR_filter _filterAccY;
    IIR_filter _filterAccZ;
};