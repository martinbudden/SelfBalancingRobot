#include "IMU_Filters.h"


void IMU_Filters::filter(xyz_t& gyroRadians, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference false positive
{
    _filterGyroX.update(gyroRadians.x, deltaT);
    _filterGyroY.update(gyroRadians.y, deltaT);
    _filterGyroZ.update(gyroRadians.z, deltaT);

    _filterAccX.update(acc.x, deltaT);
    _filterAccY.update(acc.y, deltaT);
    _filterAccZ.update(acc.z, deltaT);
};