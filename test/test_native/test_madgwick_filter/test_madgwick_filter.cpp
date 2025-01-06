#include <cmath>
#include "arduino_madgwick.h"
#include "SensorFusionFilter.h"
#include <unity.h>


void setUp() {
}

void tearDown() {
}

void test_arduino_madgwick() {
    static Madgwick arduinoMadgwick;
    static MadgwickFilter madgwickFilter;

    float gx = 0.33f;
    float gy = 0.55f;
    float gz = -0.19f;
    xyz_t acc = { .x =0.66f, .y = 0.77f, .z = 0.88f };

    const float q0 = 3.0f;
    const float q1 = 5.0f;
    const float q2 = 7.0f;
    const float q3 = 9.0f;

    xyz_t gyro = { .x = gx*0.0174533f, .y = gy*0.0174533f, .z = gz*0.0174533f };
    float beta { 1.0 };
    float deltaT = 0.1f;

    arduinoMadgwick.begin(1.0F / deltaT, beta);
    madgwickFilter.setBeta(beta);

    arduinoMadgwick._setAndNormalizeQ(q0, q1, q2, q3);
    arduinoMadgwick.updateIMU(gx, gy, gz, acc.x, acc.y, acc.z);
    TEST_ASSERT_EQUAL(0, arduinoMadgwick.anglesComputed);
    float qE0 = arduinoMadgwick.q0;
    float qE1 = arduinoMadgwick.q1;
    float qE2 = arduinoMadgwick.q2;
    float qE3 = arduinoMadgwick.q3;

    madgwickFilter._setAndNormalizeQ(q0, q1, q2, q3);
    Quaternion q = madgwickFilter.update(gyro, acc, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(qE0, q.getW());
    TEST_ASSERT_EQUAL_FLOAT(qE1, q.getX());
    TEST_ASSERT_EQUAL_FLOAT(qE2, q.getY());
    TEST_ASSERT_EQUAL_FLOAT(qE3, q.getZ());


    beta = 0.1f;
    deltaT = 0.01;
    arduinoMadgwick.begin(1.0F / deltaT, beta);
    madgwickFilter.setBeta(beta);
    acc = { .x = -0.66f, .y = 1.77f, .z = 3.88f };

    arduinoMadgwick._setAndNormalizeQ(q0, q1, q2, q3);
    arduinoMadgwick.updateIMU(gx, gy, gz, acc.x, acc.y, acc.z);
    qE0 = arduinoMadgwick.q0;
    qE1 = arduinoMadgwick.q1;
    qE2 = arduinoMadgwick.q2;
    qE3 = arduinoMadgwick.q3;

    madgwickFilter._setAndNormalizeQ(q0, q1, q2, q3);
    q = madgwickFilter.update(gyro, acc, deltaT);
    TEST_ASSERT_EQUAL_FLOAT(qE0, q.getW());
    TEST_ASSERT_EQUAL_FLOAT(qE1, q.getX());
    TEST_ASSERT_EQUAL_FLOAT(qE2, q.getY());
    TEST_ASSERT_EQUAL_FLOAT(qE3, q.getZ());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_arduino_madgwick);

    UNITY_END();
}
