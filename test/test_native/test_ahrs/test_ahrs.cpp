#include "AHRS_Test.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_ahrs() {
    AHRS_Test ahrs;
    TEST_ASSERT_FALSE(ahrs.filterIsInitializing());
    ahrs.setFilterInitializing(true);
    TEST_ASSERT_TRUE(ahrs.filterIsInitializing());
    ahrs.setFilterInitializing(false);
    TEST_ASSERT_FALSE(ahrs.filterIsInitializing());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);

    UNITY_END();
}
