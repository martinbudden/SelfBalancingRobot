#if defined(M5_UNIFIED)
#include <M5Unified.h>

#include <IMU_M5Unified.h>

IMU_M5_UNIFIED::IMU_M5_UNIFIED(void* i2cMutex) :
    IMU_Base(i2cMutex)
{
    i2cSemaphoreTake();
#if defined(IMU_Y_AXIS_POINTS_LEFT)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_Y_AXIS_POINTS_RIGHT)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_Y_AXIS_POINTS_DOWN)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
#endif
    i2cSemaphoreGive();
}

void IMU_M5_UNIFIED::setAccOffset(const xyz_int16_t& accOffset)
{
}

void IMU_M5_UNIFIED::setGyroOffset(const xyz_int16_t& gyroOffset)
{
}

xyz_int16_t IMU_M5_UNIFIED::readAccRaw() const
{
    xyz_int16_t acc {};
    assert(false && ("M5Unified variants should not call readAccRaw")); // NOLINT(readability-simplify-boolean-expr)
    return acc;
}

xyz_int16_t IMU_M5_UNIFIED::readGyroRaw() const
{
    xyz_int16_t gyro {};
    assert(false && ("M5Unified variants should not call readGyroRaw")); // NOLINT(readability-simplify-boolean-expr)
    return gyro;
}

bool IMU_M5_UNIFIED::readGyroRadiansAcc(xyz_t& gyroRadians, xyz_t& acc) const
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake();
    const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive();
    if (imu_update == 0) {
        return false;
    }

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    // convert gyro values to radians for Madgwick filter
    constexpr float degreesToRadians {M_PI / 180.0};
    gyroRadians = {
        .x = data.gyro.x * degreesToRadians,
        .y = data.gyro.y * degreesToRadians,
        .z = data.gyro.z * degreesToRadians
    };
    acc = {
        .x = data.accel.x,
        .y = data.accel.y,
        .z = data.accel.z
    };
// NOLINTEND(cppcoreguidelines-pro-type-union-access)
    return true;
}

int IMU_M5_UNIFIED::readFIFO_ToBuffer()
{
    return 0;
}

void IMU_M5_UNIFIED::readFIFO_Item(xyz_t& gyroRadians, xyz_t& acc, size_t index)
{
    (void)gyroRadians;
    (void)acc;
    (void)index;
}
#endif