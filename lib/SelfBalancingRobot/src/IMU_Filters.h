#pragma once

#include <Filters.h>
#include <xyz_type.h>


class IMU_Filters {
public:
    void filter(xyz_t& gyroRadians, xyz_t& acc, float deltaT);
private:
    FilterNull _filterGyroX;
    FilterNull _filterGyroY;
    FilterNull _filterGyroZ;
    FilterNull _filterAccX;
    FilterNull _filterAccY;
    FilterNull _filterAccZ;
};