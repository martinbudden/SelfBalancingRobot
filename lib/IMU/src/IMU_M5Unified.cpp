#if defined(M5_UNIFIED)
#include <M5Unified.h>

#include <IMU_M5Unified.h>

IMU_M5_UNIFIED::IMU_M5_UNIFIED(void* i2cMutex) :
    IMU_Base(i2cMutex)
{
    i2cSemaphoreTake();
#if defined(IMU_X_AXIS_FRONT_Y_AXIS_LEFT)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_X_AXIS_BACK_Y_AXIS_RIGHT)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
#elif defined(IMU_X_AXIS_RIGHT_Y_AXIS_DOWN)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
#elif defined(IMU_X_AXIS_RIGHT_Y_AXIS_FRONT)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
#else
    static_assert(false && "IMU orientation not implemented for M5Unified.");
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

xyz_t IMU_M5_UNIFIED::readAcc() const
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake();
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive();

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t acc = {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.accel.x,
        .y = data.accel.y,
        .z = data.accel.z
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };

    return acc;
}

xyz_int16_t IMU_M5_UNIFIED::readGyroRaw() const
{
    xyz_int16_t gyro {};
    assert(false && ("M5Unified variants should not call readGyroRaw")); // NOLINT(readability-simplify-boolean-expr)
    return gyro;
}

xyz_t IMU_M5_UNIFIED::readGyroRPS() const
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake();
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive();

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t gyroRPS {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.gyro.x * degreesToRadians,
        .y = data.gyro.y * degreesToRadians,
        .z = data.gyro.z * degreesToRadians
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyroRPS;
}

xyz_t IMU_M5_UNIFIED::readGyroDPS() const
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake();
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive();

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t gyroDPS {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.gyro.x,
        .y = data.gyro.y,
        .z = data.gyro.z
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyroDPS;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_UNIFIED::readGyroRPS_Acc() const
{
    // This is very slow on the M5 Atom.
    i2cSemaphoreTake();
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    i2cSemaphoreGive();

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    gyroRPS_Acc_t gyroAcc {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .gyroRPS = {
            .x = data.gyro.x * degreesToRadians,
            .y = data.gyro.y * degreesToRadians,
            .z = data.gyro.z * degreesToRadians
        },
        .acc = {
            .x = data.accel.x,
            .y = data.accel.y,
            .z = data.accel.z
        }
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyroAcc;
}

int IMU_M5_UNIFIED::readFIFO_ToBuffer()
{
    return 0;
}

IMU_Base::gyroRPS_Acc_t IMU_M5_UNIFIED::readFIFO_Item(size_t index)
{
    (void)index;

    gyroRPS_Acc_t gyroAcc {};
    return gyroAcc;
}
#endif