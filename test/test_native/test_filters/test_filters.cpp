#include "Filters.h"
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_null_filter() {
    FilterNull filter;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, filter.update(1.0F));
    TEST_ASSERT_EQUAL_FLOAT(1.0F, filter.update(1.0F));
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, filter.update(-1.0F));

    filter.reset();
    TEST_ASSERT_EQUAL_FLOAT(4.0F, filter.update(4.0F));
}

void test_moving_average_filter(){
    FilterMovingAverage<3> filter;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, filter.update(1.0F));
    TEST_ASSERT_EQUAL_FLOAT(1.5F, filter.update(2.0F));
    TEST_ASSERT_EQUAL_FLOAT(2.0F, filter.update(3.0F));
    TEST_ASSERT_EQUAL_FLOAT(3.0F, filter.update(4.0F));
    TEST_ASSERT_EQUAL_FLOAT(4.0F, filter.update(5.0F));
    TEST_ASSERT_EQUAL_FLOAT(5.0F, filter.update(6.0F));
    TEST_ASSERT_EQUAL_FLOAT(7.0F, filter.update(10.0F));

    filter.reset();
    TEST_ASSERT_EQUAL_FLOAT(4.0F, filter.update(4.0F));
    TEST_ASSERT_EQUAL_FLOAT(12.0F, filter.update(20.0F));
    TEST_ASSERT_EQUAL_FLOAT(5.0F, filter.update(-9.0F));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_null_filter);
    RUN_TEST(test_moving_average_filter);

    UNITY_END();
}
