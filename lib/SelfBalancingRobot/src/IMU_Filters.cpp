#include "IMU_Filters.h"


void IMU_Filters::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference false positive
{
    gyroRPS.x = _filterGyroX.update(gyroRPS.x, deltaT);
    gyroRPS.y = _filterGyroY.update(gyroRPS.y, deltaT);
    gyroRPS.z = _filterGyroZ.update(gyroRPS.z, deltaT);
    acc.x = _filterAccX.update(acc.x, deltaT);
    acc.y = _filterAccY.update(acc.y, deltaT);
    acc.z = _filterAccZ.update(acc.z, deltaT);
};
