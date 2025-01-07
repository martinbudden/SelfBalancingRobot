#include <cmath>
#include "SensorFusionFilter.h"
#include <unity.h>


void setUp() {
}

void tearDown() {
}

void test_quaternion_g() {
    Quaternion q(1, 2, 3, 4);
    QuaternionG g(q);

    TEST_ASSERT_EQUAL_FLOAT(1, g.getW());
    TEST_ASSERT_EQUAL_FLOAT(2, g.getX());
    TEST_ASSERT_EQUAL_FLOAT(3, g.getY());
    TEST_ASSERT_EQUAL_FLOAT(4, g.getZ());
}

void test_conversions() {
    const float degrees45inRadians = 45.0F * Quaternion::degreesToRadians;

    //const float roll = atan2(acc.y, acc.z);
    //const float pitch = atan2(-acc.x, sqrt(acc.y*acc.y + acc.z*acc.z));


    TEST_ASSERT_EQUAL_FLOAT(0.0, SensorFusionFilterBase::rollRadiansFromAccNormalized(xyz_t { .x = 0.0, .y = 0.0, .z = 1.0 }));
    TEST_ASSERT_EQUAL_FLOAT(0.0, SensorFusionFilterBase::pitchRadiansFromAccNormalized(xyz_t { .x = 0.0, .y = 0.0, .z = 1.0 }));

    const xyz_t accX1_Y0_Z1normalized = { .x = 1.0F/sqrtf(2.0F), .y = 0.0, .z = 1.0F/sqrtf(2.0F) };
    TEST_ASSERT_EQUAL_FLOAT(0.0, SensorFusionFilterBase::rollRadiansFromAccNormalized(accX1_Y0_Z1normalized));
    TEST_ASSERT_EQUAL_FLOAT(-degrees45inRadians, SensorFusionFilterBase::pitchRadiansFromAccNormalized(accX1_Y0_Z1normalized));

    const xyz_t accX0_Y1_Z1normalized = { .x = 0.0F, .y = 1.0F/sqrtf(2.0F), .z = 1.0F/sqrtf(2.0F) };
    TEST_ASSERT_EQUAL_FLOAT(degrees45inRadians, SensorFusionFilterBase::rollRadiansFromAccNormalized(accX0_Y1_Z1normalized));
    TEST_ASSERT_EQUAL_FLOAT(0.0, SensorFusionFilterBase::pitchRadiansFromAccNormalized(accX0_Y1_Z1normalized));
}

void test_complementary_filter() {
    static ComplementaryFilter complementaryFilter;

    const xyz_t gyro0 = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    float deltaT = 0.1f;

    complementaryFilter.reset();
    complementaryFilter.setAlpha(0.0F); // set alpha to zero so just the accelerometer component is returned
    const xyz_t accZ1 = { .x = 0.0, .y = 0.0, .z = 1.0 };

    Quaternion q = complementaryFilter.update(gyro0, xyz_t { .x = 0.0, .y = 0.0, .z = 1.0 }, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0, q.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0, q.calculatePitchDegrees());


    complementaryFilter.reset();
    q = complementaryFilter.update(gyro0, xyz_t { .x = 1.0F/sqrtf(2.0F), .y = 0.0, .z = 1.0F/sqrtf(2.0F) }, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(0, q.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(-45, q.calculatePitchDegrees());

    complementaryFilter.reset();
    q = complementaryFilter.update(gyro0, xyz_t { .x = 0.0, .y = 1.0, .z = 1.0 }, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(45, q.calculateRollDegrees());
    TEST_ASSERT_EQUAL_FLOAT(0, q.calculatePitchDegrees());

}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_quaternion_g);
    RUN_TEST(test_conversions);
    RUN_TEST(test_complementary_filter);

    UNITY_END();
}
