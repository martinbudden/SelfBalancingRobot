#include "Quaternion.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_quaternion()
{
    const float degrees23inRadians = 23.0F * Quaternion::degreesToRadians;
    const float degrees67inRadians = 67.0F * Quaternion::degreesToRadians;
    const float degrees77inRadians = 77.0F * Quaternion::degreesToRadians;

    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees23inRadians, degrees67inRadians, degrees77inRadians);
    TEST_ASSERT_EQUAL_FLOAT(23.0F, q0.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0F, q0.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(77.0F, q0.calculateYawDegrees());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(degrees67inRadians, -degrees23inRadians, degrees77inRadians);
    TEST_ASSERT_EQUAL_FLOAT(67.0F, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-23.0F, q1.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(77.0F, q1.calculateYawDegrees());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_quaternion);

    UNITY_END();
}
