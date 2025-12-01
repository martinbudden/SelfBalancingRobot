#include "MotorsBalaC.h"
#include "Motors4EncoderMotor.h"

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

void test_motor_construction()
{
    enum {STEPS_PER_REVOLUTION = 123};

    const Motors4EncoderMotor motors(0, 0, STEPS_PER_REVOLUTION);
    TEST_ASSERT_EQUAL_FLOAT(STEPS_PER_REVOLUTION, motors.getStepsPerRevolution());
    TEST_ASSERT_EQUAL_INT32(0, motors.getLeftEncoder());
    TEST_ASSERT_EQUAL_INT32(0, motors.getRightEncoder());
}

void test_BalaC_motors_clipping()
{
    constexpr float deadbandPower = 0.0F;
    const MotorsBalaC motors(deadbandPower);

    TEST_ASSERT_EQUAL_FLOAT(0.0F, motors.getStepsPerRevolution());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, motors.getDeadbandPower());
    TEST_ASSERT_EQUAL_INT32(0, motors.getLeftEncoder());
    TEST_ASSERT_EQUAL_INT32(0, motors.getRightEncoder());

    float power = motors.scalePower(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, power);
    power = motors.scalePower(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.5, power);
    power = motors.scalePower(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, power);
    power = motors.scalePower(2.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, power);
    power = motors.scalePower(-0.5F);
    TEST_ASSERT_EQUAL_FLOAT(-0.5F, power);
    power = motors.scalePower(-1.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, power);
    power = motors.scalePower(-2.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, power);
}

void test_BalaC_motors_deadband()
{
    constexpr float deadbandPower = 0.2F;
    const MotorsBalaC motors(deadbandPower);

    TEST_ASSERT_EQUAL_FLOAT(deadbandPower, motors.getDeadbandPower());

    float power = motors.scalePower(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, power);

    power = motors.scalePower(0.01F);
    TEST_ASSERT_EQUAL_FLOAT(0.208F, power);
    power = motors.scalePower(0.1F);
    TEST_ASSERT_EQUAL_FLOAT(0.28F, power);
    power = motors.scalePower(0.2F);
    TEST_ASSERT_EQUAL_FLOAT(0.36F, power);
    power = motors.scalePower(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, power);
    power = motors.scalePower(2.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, power);

    power = motors.scalePower(-0.01F);
    TEST_ASSERT_EQUAL_FLOAT(-0.208F, power);
    power = motors.scalePower(-0.1F);
    TEST_ASSERT_EQUAL_FLOAT(-0.28F, power);
    power = motors.scalePower(-0.2F);
    TEST_ASSERT_EQUAL_FLOAT(-0.36F, power);
    power = motors.scalePower(-1.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, power);
    power = motors.scalePower(-2.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, power);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_motor_construction);
    RUN_TEST(test_BalaC_motors_clipping);
    RUN_TEST(test_BalaC_motors_deadband);

    UNITY_END();
}
