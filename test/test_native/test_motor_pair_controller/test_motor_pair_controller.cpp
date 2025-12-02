#include "IMU_FiltersNull.h"

#include <AHRS.h>
#include <AHRS_MessageQueue.h>
#include <IMU_Null.h>
#include <MotorPairController.h>
#include <NonVolatileStorage.h>
#include <SV_TelemetryData.h>
#include <SensorFusion.h>

#include <unity.h>

enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };


void setUp()
{
}

void tearDown()
{
}

void test_motor_pair_controller()
{
    enum { TASK_DENOMINATOR = 2 };
    static AHRS_MessageQueue ahrsMessageQueue;
    MotorPairBase& motors = MotorPairController::allocateMotors(); // NOLINT(misc-const-correctness)
    MotorPairController mpc(AHRS_TASK_INTERVAL_MICROSECONDS, TASK_DENOMINATOR, motors, ahrsMessageQueue, nullptr);
    TEST_ASSERT_FALSE(mpc.motorsIsOn());

    mpc.motorsSwitchOn();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.setSensorFusionFilterIsInitializing(false);
    mpc.motorsSwitchOn();
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

    TEST_ASSERT_EQUAL(TD_PID::SELF_BALANCING_ROBOT, VehicleControllerBase::SELF_BALANCING_ROBOT);
    TEST_ASSERT_EQUAL(TD_PID::AIRCRAFT, VehicleControllerBase::AIRCRAFT);

    TEST_ASSERT_TRUE(static_cast<int>(TD_PID::MAX_PID_COUNT) >= static_cast<int>(MotorPairController::PID_COUNT));
}

void test_motor_pair_controller_pid_indexes()
{
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::ROLL_ANGLE_DEGREES) == static_cast<int>(TD_SBR_PID::ROLL_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::PITCH_ANGLE_DEGREES) == static_cast<int>(TD_SBR_PID::PITCH_ANGLE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::YAW_RATE_DPS) == static_cast<int>(TD_SBR_PID::YAW_RATE));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::SPEED_SERIAL_DPS) == static_cast<int>(TD_SBR_PID::SPEED_SERIAL));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::SPEED_PARALLEL_DPS) == static_cast<int>(TD_SBR_PID::SPEED_PARALLEL));
    TEST_ASSERT_TRUE(static_cast<int>(MotorPairController::POSITION_DEGREES) == static_cast<int>(TD_SBR_PID::POSITION));
}

void test_motor_non_volatile_storage()
{
    std::array<char, 8> chars;

    NonVolatileStorage::toHexChars(&chars[0], 0x1234);
    TEST_ASSERT_EQUAL('0', chars[0]);
    TEST_ASSERT_EQUAL('x', chars[1]);
    TEST_ASSERT_EQUAL('1', chars[2]);
    TEST_ASSERT_EQUAL('2', chars[3]);
    TEST_ASSERT_EQUAL('3', chars[4]);
    TEST_ASSERT_EQUAL('4', chars[5]);
    TEST_ASSERT_EQUAL(0, chars[6]);

    NonVolatileStorage::toHexChars(&chars[0], 0x010C);
    TEST_ASSERT_EQUAL('0', chars[0]);
    TEST_ASSERT_EQUAL('x', chars[1]);
    TEST_ASSERT_EQUAL('0', chars[2]);
    TEST_ASSERT_EQUAL('1', chars[3]);
    TEST_ASSERT_EQUAL('0', chars[4]);
    TEST_ASSERT_EQUAL('C', chars[5]);
    TEST_ASSERT_EQUAL(0, chars[6]);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);
    RUN_TEST(test_motor_pair_controller_pid_indexes);
    RUN_TEST(test_motor_non_volatile_storage);

    UNITY_END();
}
