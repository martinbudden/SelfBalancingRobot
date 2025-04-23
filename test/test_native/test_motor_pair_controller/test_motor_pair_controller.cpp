#include "MotorPairController.h"
#include "SBR_TelemetryData.h"

#include <AHRS.h>
#include <IMU_FiltersBase.h>
#include <ReceiverNull.h>
#include <SensorFusion.h>

#include <unity.h>


class IMU_Test : public IMU_Base {
public:
    explicit IMU_Test(axis_order_t axisOrder) : IMU_Base(axisOrder) {}
    void init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity) override;
    xyz_int32_t readGyroRaw() override;
    xyz_int32_t readAccRaw() override;
};
void IMU_Test::init(uint32_t outputDataRateHz, gyro_sensitivity_t gyroSensitivity, acc_sensitivity_t accSensitivity)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;
}
IMU_Base::xyz_int32_t IMU_Test::readGyroRaw() { return xyz_int32_t {}; }
IMU_Base::xyz_int32_t IMU_Test::readAccRaw() { return xyz_int32_t {}; }


class IMU_Filters_Test : public IMU_FiltersBase {
public:
    virtual ~IMU_Filters_Test() = default;
    IMU_Filters_Test() = default;

    // IMU_Filters_Test is not copyable or moveable
    IMU_Filters_Test(const IMU_Filters_Test&) = delete;
    IMU_Filters_Test& operator=(const IMU_Filters_Test&) = delete;
    IMU_Filters_Test(IMU_Filters_Test&&) = delete;
    IMU_Filters_Test& operator=(IMU_Filters_Test&&) = delete;

    void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
};
void IMU_Filters_Test::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) { (void)gyroRPS; (void)acc; (void)deltaT; }


void setUp() {
}

void tearDown() {
}

void test_motor_pair_controller()
{
    static MadgwickFilter sensorFusionFilter; // NOLINT(misc-const-correctness) false positive
    static IMU_Test imu(IMU_Base::XPOS_YPOS_ZPOS); // NOLINT(misc-const-correctness) false positive
    static IMU_Filters_Test imuFilters; // NOLINT(misc-const-correctness) false positive
    static AHRS ahrs(sensorFusionFilter, imu, imuFilters);
    static ReceiverNull receiver; // NOLINT(misc-const-correctness) false positive

    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    MotorPairController mpc(ahrs, receiver);
    TEST_ASSERT_FALSE(mpc.motorsIsOn());

    mpc.motorsSwitchOn();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    ahrs.setSensorFusionFilterInitializing(false);
    mpc.motorsSwitchOn();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());

    mpc.motorsToggleOnOff();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.motorsToggleOnOff();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());

    mpc.motorsSwitchOff();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.motorsSwitchOn();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());

    static const std::string pidNameRoll = mpc.getPID_Name(MotorPairController::ROLL_ANGLE_DEGREES);
    TEST_ASSERT_TRUE(pidNameRoll.compare("ROLL_ANGLE") == 0);

    static const std::string pidNamePitch = mpc.getPID_Name(MotorPairController::PITCH_ANGLE_DEGREES);
    TEST_ASSERT_TRUE(pidNamePitch.compare("PITCH_ANGLE") == 0);

    static const std::string pidNameYaw = mpc.getPID_Name(MotorPairController::YAW_RATE_DPS);
    TEST_ASSERT_TRUE(pidNameYaw.compare("YAW_RATE") == 0);

    static const std::string pidNameSpeed = mpc.getPID_Name(MotorPairController::SPEED_DPS);
    TEST_ASSERT_TRUE(pidNameSpeed.compare("SPEED") == 0);

    static const std::string pidNamePosition = mpc.getPID_Name(MotorPairController::POSITION_DEGREES);
    TEST_ASSERT_TRUE(pidNamePosition.compare("POSITION") == 0);
}

void test_motor_pair_controller_pid_indexes()
{
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::ROLL_ANGLE_DEGREES) == static_cast<int>(TD_SBR_PIDS::ROLL_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::PITCH_ANGLE_DEGREES) == static_cast<int>(TD_SBR_PIDS::PITCH_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::YAW_RATE_DPS) == static_cast<int>(TD_SBR_PIDS::YAW_RATE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::SPEED_DPS) == static_cast<int>(TD_SBR_PIDS::SPEED));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::POSITION_DEGREES) == static_cast<int>(TD_SBR_PIDS::POSITION));
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);
    RUN_TEST(test_motor_pair_controller_pid_indexes);

    UNITY_END();
}
