#include "MotorsBalaC.h"
#include "Motors4EncoderMotor.h"

#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_motor_construction() {
    enum {STEPS_PER_REVOLUTION = 123};

    Motors4EncoderMotor motors(0, 0, STEPS_PER_REVOLUTION);
    TEST_ASSERT_EQUAL_FLOAT(STEPS_PER_REVOLUTION, motors.getStepsPerRevolution());
    TEST_ASSERT_EQUAL_INT32(0, motors.getLeftEncoder());
    TEST_ASSERT_EQUAL_INT32(0, motors.getRightEncoder());
}

void test_BalaC_motors_clipping() {
    MotorsBalaC motors(0, 0);

    TEST_ASSERT_EQUAL_FLOAT(0, motors.getStepsPerRevolution());
    TEST_ASSERT_EQUAL_INT32(0, motors.getLeftEncoder());
    TEST_ASSERT_EQUAL_INT32(0, motors.getRightEncoder());

    float power = motors.scalePower(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0, power);
    power = motors.scalePower(0.5F);
    TEST_ASSERT_EQUAL_FLOAT(0.5, power);
    power = motors.scalePower(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0, power);
    power = motors.scalePower(2.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0, power);
    power = motors.scalePower(-0.5F);
    TEST_ASSERT_EQUAL_FLOAT(-0.5, power);
    power = motors.scalePower(-1.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, power);
    power = motors.scalePower(-2.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, power);
}

void test_BalaC_motors_deadband() {
    MotorsBalaC motors(0, 0);
    motors.setDeadbandPower(0.2);

    float power = motors.scalePower(0.0F);
    TEST_ASSERT_EQUAL_FLOAT(0.0, power);

    power = motors.scalePower(0.01F);
    TEST_ASSERT_EQUAL_FLOAT(0.208, power);
    power = motors.scalePower(0.1F);
    TEST_ASSERT_EQUAL_FLOAT(0.28, power);
    power = motors.scalePower(0.2F);
    TEST_ASSERT_EQUAL_FLOAT(0.36, power);
    power = motors.scalePower(1.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0, power);
    power = motors.scalePower(2.0F);
    TEST_ASSERT_EQUAL_FLOAT(1.0, power);

    power = motors.scalePower(-0.01F);
    TEST_ASSERT_EQUAL_FLOAT(-0.208, power);
    power = motors.scalePower(-0.1F);
    TEST_ASSERT_EQUAL_FLOAT(-0.28, power);
    power = motors.scalePower(-0.2F);
    TEST_ASSERT_EQUAL_FLOAT(-0.36, power);
    power = motors.scalePower(-1.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, power);
    power = motors.scalePower(-2.0F);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, power);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_motor_construction);
    RUN_TEST(test_BalaC_motors_clipping);
    RUN_TEST(test_BalaC_motors_deadband);

    UNITY_END();
}
