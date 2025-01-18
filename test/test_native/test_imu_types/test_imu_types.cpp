#include "Quaternion.h"
#include "xyz_type.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_xyz_t()
{
    const xyz_t a{2, 3, 5};
    const xyz_t b{7, 11, 17};
    TEST_ASSERT_FALSE(a == b);
    TEST_ASSERT_TRUE(a != b);

    const xyz_t a2{4, 6, 10};
    TEST_ASSERT_TRUE(a2 == a*2);

    const xyz_t b2{14, 22, 34};
    TEST_ASSERT_TRUE(b2 == b*2);

    const xyz_t a_plus_b =  a + b;

    TEST_ASSERT_EQUAL(9, a_plus_b.x);
    TEST_ASSERT_EQUAL(14, a_plus_b.y);
    TEST_ASSERT_EQUAL(22, a_plus_b.z);

    const xyz_t a_minus_b =  a - b;

    TEST_ASSERT_EQUAL(-5, a_minus_b.x);
    TEST_ASSERT_EQUAL(-8, a_minus_b.y);
    TEST_ASSERT_EQUAL(-12, a_minus_b.z);

    TEST_ASSERT_TRUE(a2 == a_plus_b + a_minus_b);
    TEST_ASSERT_TRUE(b2 == a_plus_b - a_minus_b);

    const float a_dot_b =  14 + 33 + 85;
    TEST_ASSERT_TRUE(a_dot_b == a.dot_product(b));
    TEST_ASSERT_TRUE(a.magnitude_squared() == a.dot_product(a));
    TEST_ASSERT_TRUE(b.magnitude_squared() == b.dot_product(b));

    const xyz_t a_cross_b = a.cross_product(b);
    TEST_ASSERT_EQUAL(3*17 - 5*11, a_cross_b.x);
    TEST_ASSERT_EQUAL(-2*17 + 5*7, a_cross_b.y);
    TEST_ASSERT_EQUAL(2*11 - 3*7, a_cross_b.z);

    TEST_ASSERT_TRUE(a_cross_b != b.cross_product(a));
}

void test_quaternion()
{
    Quaternion a{2, 3, 5, 7};
    Quaternion b{11, 13, 17, 23};
    TEST_ASSERT_FALSE(a == b);
    TEST_ASSERT_TRUE(a != b);

    const Quaternion a2{4, 6, 10, 14};
    TEST_ASSERT_TRUE(a2 == a*2);

    const Quaternion b2{22, 26, 34, 46};
    TEST_ASSERT_TRUE(b2 == b*2);

    const Quaternion a_plus_b =  a + b;

    TEST_ASSERT_EQUAL(13, a_plus_b.getW());
    TEST_ASSERT_EQUAL(16, a_plus_b.getX());
    TEST_ASSERT_EQUAL(22, a_plus_b.getY());
    TEST_ASSERT_EQUAL(30, a_plus_b.getZ());

    const Quaternion a_minus_b =  a - b;

    TEST_ASSERT_EQUAL(-9, a_minus_b.getW());
    TEST_ASSERT_EQUAL(-10, a_minus_b.getX());
    TEST_ASSERT_EQUAL(-12, a_minus_b.getY());
    TEST_ASSERT_EQUAL(-16, a_minus_b.getZ());

    TEST_ASSERT_TRUE(a2 == a_plus_b + a_minus_b);
    TEST_ASSERT_TRUE(b2 == a_plus_b - a_minus_b);

    Quaternion c = a;
    c *= b;
    TEST_ASSERT_TRUE(c == a*b);

    TEST_ASSERT_TRUE(a.magnitude_squared()*a.magnitude_squared() == (a*a).magnitude_squared());
    TEST_ASSERT_TRUE(a.magnitude_squared()*a.magnitude_squared() == (a*a.conjugate()).magnitude_squared());

}

void test_quaternion_angles()
{
    const float degrees19inRadians = 19.0F * Quaternion::degreesToRadians;
    const float degrees43inRadians = 43.0F * Quaternion::degreesToRadians;
    const float degrees67inRadians = 67.0F * Quaternion::degreesToRadians;

    const Quaternion q0 = Quaternion::fromEulerAnglesRadians(degrees19inRadians, degrees43inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(19.0, q0.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q0.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q0.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q0.magnitude_squared());

    const Quaternion q1 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, -degrees19inRadians, degrees67inRadians);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q1.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q1.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(67.0, q1.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q1.magnitude_squared());

    const Quaternion q2 = Quaternion::fromEulerAnglesRadians(degrees43inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(43.0, q2.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2.magnitude_squared());

    const Quaternion q3 = Quaternion::fromEulerAnglesRadians(-degrees19inRadians, 0, 0);
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q3.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q3.magnitude_squared());

    Quaternion q2q3 = q2 * q3;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q2q3.magnitude_squared());
    TEST_ASSERT_EQUAL_FLOAT(24.0, q2q3.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q2q3.calculateYawDegrees());

    const Quaternion q4 = Quaternion::fromEulerAnglesRadians(0, -degrees67inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-67.0, q4.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4.magnitude_squared());

    const Quaternion q5 = Quaternion::fromEulerAnglesRadians(0, degrees43inRadians, 0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q5.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q5.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q5.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q5.magnitude_squared());

    Quaternion q4q5 = q4 * q5;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q4q5.magnitude_squared());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4q5.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-24.0, q4q5.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q4q5.calculateYawDegrees());

    const Quaternion q6 = Quaternion::fromEulerAnglesRadians(0, 0, -degrees19inRadians);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-19.0, q6.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q6.magnitude_squared());

    const Quaternion q7 = Quaternion::fromEulerAnglesRadians(0, 0, degrees43inRadians);
    TEST_ASSERT_EQUAL_FLOAT(0.0, q7.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q7.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(43.0, q7.calculateYawDegrees());
    TEST_ASSERT_EQUAL_FLOAT(1.0, q7.magnitude_squared());

    Quaternion q6q7 = q6 * q7;
    TEST_ASSERT_EQUAL_FLOAT(1.0, q6q7.magnitude_squared());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6q7.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0.0, q6q7.calculatePitchDegrees());
    TEST_ASSERT_EQUAL_FLOAT(24.0, q6q7.calculateYawDegrees());

}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_xyz_t);
    RUN_TEST(test_quaternion);
    RUN_TEST(test_quaternion_angles);

    UNITY_END();
}
