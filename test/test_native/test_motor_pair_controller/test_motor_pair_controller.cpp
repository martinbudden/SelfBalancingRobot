#include "MotorPairController.h"

#include <AHRS.h>
#include <IMU_FiltersNull.h>
#include <IMU_Null.h>
#include <RadioController.h>
#include <ReceiverNull.h>
#include <SV_TelemetryData.h>
#include <SensorFusion.h>

#include <unity.h>

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif


void setUp()
{
}

void tearDown()
{
}

void test_motor_pair_controller()
{
    static MadgwickFilter sensorFusionFilter; // NOLINT(misc-const-correctness) false positive
    static IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS); // NOLINT(misc-const-correctness) false positive
    static IMU_FiltersNull imuFilters; // NOLINT(misc-const-correctness) false positive
    static AHRS ahrs(AHRS_TASK_INTERVAL_MICROSECONDS, sensorFusionFilter, imu, imuFilters);
    static ReceiverNull receiver; // NOLINT(misc-const-correctness) false positive
    static RadioController radioController(receiver);

    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    enum { TASk_INTERVAL_MICROSECONDS = 5000 };
    MotorPairController mpc(TASk_INTERVAL_MICROSECONDS, ahrs, radioController);
    TEST_ASSERT_FALSE(mpc.motorsIsOn());

    mpc.motorsSwitchOn();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    ahrs.setSensorFusionInitializing(false);
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

    static const std::string pidNameSpeedSerial = mpc.getPID_Name(MotorPairController::SPEED_SERIAL_DPS);
    TEST_ASSERT_TRUE(pidNameSpeedSerial.compare("SPEED_SERIAL") == 0);

    static const std::string pidNameSpeedParallel = mpc.getPID_Name(MotorPairController::SPEED_PARALLEL_DPS);
    TEST_ASSERT_TRUE(pidNameSpeedParallel.compare("SPEED_PARALLEL") == 0);

    static const std::string pidNamePosition = mpc.getPID_Name(MotorPairController::POSITION_DEGREES);
    TEST_ASSERT_TRUE(pidNamePosition.compare("POSITION") == 0);

    const MotorPairController::pidf_array_t& scaleFactors = mpc.getScaleFactors();
    TEST_ASSERT_EQUAL_FLOAT(0.0002F, scaleFactors[MotorPairController::PITCH_ANGLE_DEGREES].kp);

    TEST_ASSERT_EQUAL(TD_PIDS::SELF_BALANCING_ROBOT, VehicleControllerBase::SELF_BALANCING_ROBOT);
    TEST_ASSERT_EQUAL(TD_PIDS::AIRCRAFT, VehicleControllerBase::AIRCRAFT);

    TEST_ASSERT_TRUE(static_cast<int>(TD_PIDS::MAX_PID_COUNT) >= static_cast<int>(MotorPairController::PID_COUNT));
}

void test_motor_pair_controller_pid_indexes()
{
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::ROLL_ANGLE_DEGREES) == static_cast<int>(TD_SBR_PIDS::ROLL_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::PITCH_ANGLE_DEGREES) == static_cast<int>(TD_SBR_PIDS::PITCH_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::YAW_RATE_DPS) == static_cast<int>(TD_SBR_PIDS::YAW_RATE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::SPEED_SERIAL_DPS) == static_cast<int>(TD_SBR_PIDS::SPEED_SERIAL));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::SPEED_PARALLEL_DPS) == static_cast<int>(TD_SBR_PIDS::SPEED_PARALLEL));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::POSITION_DEGREES) == static_cast<int>(TD_SBR_PIDS::POSITION));
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);
    RUN_TEST(test_motor_pair_controller_pid_indexes);

    UNITY_END();
}
