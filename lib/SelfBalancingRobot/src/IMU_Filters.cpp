#include "IMU_Filters.h"


void IMU_Filters::filter(xyz_t& gyroRadians, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference false positive
{
    gyroRadians.x = _filterGyroX.update(gyroRadians.x, deltaT);
    gyroRadians.y = _filterGyroY.update(gyroRadians.y, deltaT);
    gyroRadians.z = _filterGyroZ.update(gyroRadians.z, deltaT);
    acc.x = _filterAccX.update(acc.x, deltaT);
    acc.y = _filterAccY.update(acc.y, deltaT);
    acc.z = _filterAccZ.update(acc.z, deltaT);
};