#pragma once

#include <Filters.h>
#include <xyz_type.h>


class IMU_Filter {
public:
    void filter(xyz_t& acc, xyz_t& gyroRadians, float deltaT);
private:
    FilterNull _filterGyroX;
    FilterNull _filterGyroY;
    FilterNull _filterGyroZ;
    FilterNull _filterAccX;
    FilterNull _filterAccY;
    FilterNull _filterAccZ;
};