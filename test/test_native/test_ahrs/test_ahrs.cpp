#include "AHRS_Test.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_ahrs() {
    AHRS_Test ahrs;
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing()); // initializing should be set on construction
    ahrs.setSensorFusionFilterInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionFilterInitializing(false);
    TEST_ASSERT_FALSE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionFilterInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);

    UNITY_END();
}
