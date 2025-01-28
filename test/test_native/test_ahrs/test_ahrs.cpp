#include "AHRS.h"
#include "AHRS_Test.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_ahrs() {
    SensorFusionFilterTest sensorFusionFilter; // NOLINT(misc-const-correctness)
    IMU_Test imu; // NOLINT(misc-const-correctness) false positive
    IMU_Filters_Test imuFilters; // NOLINT(misc-const-correctness) false positive
    AHRS ahrs(sensorFusionFilter, imu, imuFilters);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing()); // initializing should be set on construction
    ahrs.setSensorFusionFilterInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionFilterInitializing(false);
    TEST_ASSERT_FALSE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionFilterInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);

    UNITY_END();
}
