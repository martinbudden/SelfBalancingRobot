#include "IMU_Filters.h"


void IMU_Filters::filter(xyz_t& gyroRadians, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference false positive
{
    gyroRadians.x = _filterGyroX.update(gyroRadians.x, deltaT);
    gyroRadians.y = _filterGyroX.update(gyroRadians.y, deltaT);
    gyroRadians.z = _filterGyroX.update(gyroRadians.z, deltaT);
    acc.x = _filterAccZ.update(acc.x, deltaT);
    acc.y = _filterAccZ.update(acc.y, deltaT);
    acc.z = _filterAccZ.update(acc.z, deltaT);
};