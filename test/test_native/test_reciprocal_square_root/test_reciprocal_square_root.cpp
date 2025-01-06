#include <cmath>
#include <unity.h>

void setUp() {
}

void tearDown() {
}

inline static float reciprocalSqrt(float x)
{
    const float halfx = 0.5F * x;
    union {
        float y;
        long i;
    } u = {x};

    u.i = 0x5f3759dF - (u.i >> 1); // Initial estimate for Newton–Raphson method
    u.y *= 1.5F - (halfx * u.y * u.y); // First iteration
    u.y *= 1.5F - (halfx * u.y * u.y); // Second iteration

    return u.y;
}

// [Pizer’s optimisation](https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/)
float reciprocalSqrtPizer(float x){
    union {
        float y;
        long i;
    } u = {x};

    u.i = 0x5f1f1412 - (u.i >> 1); // Initial estimate for Newton–Raphson method
    u.y *= 1.69000231F - 0.714158168F * x * u.y * u.y; // First iteration
    u.y *= 1.5F - (0.5F * x * u.y * u.y); // Second iteration

    return u.y;
}

void test_square_root_reciprocal()
{

    TEST_ASSERT_FLOAT_WITHIN(0.0009F,    1.41421356F, reciprocalSqrt(0.5F));
    TEST_ASSERT_EQUAL_FLOAT(1.41421356F, reciprocalSqrtPizer(0.5F));

    TEST_ASSERT_FLOAT_WITHIN(0.0009F,    0.5F, reciprocalSqrt(4));
    TEST_ASSERT_FLOAT_WITHIN(0.0000025F, 0.5F, reciprocalSqrt(4));
    TEST_ASSERT_EQUAL_FLOAT(0.5F, reciprocalSqrtPizer(4));

    TEST_ASSERT_FLOAT_WITHIN(0.0003F,    0.125F, reciprocalSqrt(64));
    TEST_ASSERT_EQUAL_FLOAT(0.125F, reciprocalSqrtPizer(64));

    TEST_ASSERT_FLOAT_WITHIN(0.0002F,    0.1F, reciprocalSqrt(100));
    TEST_ASSERT_EQUAL_FLOAT(0.1F, reciprocalSqrtPizer(100));

    TEST_ASSERT_FLOAT_WITHIN(0.00002F,   0.01F, reciprocalSqrt(10000));
    TEST_ASSERT_EQUAL_FLOAT(0.01F, reciprocalSqrtPizer(10000));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_square_root_reciprocal);

    UNITY_END();
}
