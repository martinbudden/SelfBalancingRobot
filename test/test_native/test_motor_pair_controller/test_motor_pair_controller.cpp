#include "AHRS_Test.h"
#include "MotorPairController.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_motor_pair_controller() {
    AHRS_Test ahrs;
    MotorPairController mpc(ahrs, nullptr);
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.motorsSwitchOn();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());
    mpc.motorsToggleOnOff();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());

    TEST_ASSERT_TRUE(mpc.getPitchRateIsFiltered());
}

void test_map_yaw_stick() {
    TEST_ASSERT_EQUAL_FLOAT(0.0F,  MotorPairController::mapYawStick(0.0F));
    TEST_ASSERT_EQUAL_FLOAT(0.45F, MotorPairController::mapYawStick(0.5F));
    TEST_ASSERT_EQUAL_FLOAT(1.0F,  MotorPairController::mapYawStick(1.0F));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);
    RUN_TEST(test_map_yaw_stick);

    UNITY_END();
}
