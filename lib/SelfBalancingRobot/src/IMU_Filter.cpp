#include "IMU_Filter.h"


void IMU_Filter::filter(xyz_t& acc, xyz_t& gyroRadians, float deltaT)
{
    _filterAccX.update(acc.x, deltaT);
    _filterAccY.update(acc.y, deltaT);
    _filterAccZ.update(acc.z, deltaT);

    _filterGyroX.update(gyroRadians.x, deltaT);
    _filterGyroY.update(gyroRadians.y, deltaT);
    _filterGyroZ.update(gyroRadians.z, deltaT);
};