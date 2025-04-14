#include <IMU_Base.h>
#include <unity.h>

class IMU_Test : public IMU_Base {
public:
    virtual ~IMU_Test() = default;
    IMU_Test(const IMU_Test&) = delete;
    IMU_Test& operator=(const IMU_Test&) = delete;
    IMU_Test(IMU_Test&&) = delete;
    IMU_Test& operator=(IMU_Test&&) = delete;
    IMU_Test() = default;
    explicit IMU_Test(axis_order_t axisOrder) : IMU_Base(axisOrder) {}
    virtual xyz_int32_t readGyroRaw() override; // NOLINT(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual xyz_int32_t readAccRaw() override; // NOLINT(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
};

IMU_Base::xyz_int32_t IMU_Test::readGyroRaw()
{
    return xyz_int32_t {};
}

IMU_Base::xyz_int32_t IMU_Test::readAccRaw()
{
    return xyz_int32_t {};
}


void setUp() {
}

void tearDown() {
}

void test_map_axes() {
    const xyz_t input { .x =3, .y = 5, .z = 7 };
    xyz_t output {};

    static const IMU_Test XPOS_YPOS_ZPOS(IMU_Base::XPOS_YPOS_ZPOS);
    output = XPOS_YPOS_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.x, output.x);
    TEST_ASSERT_EQUAL(input.y, output.y);
    TEST_ASSERT_EQUAL(input.z, output.z);

    static const IMU_Test YPOS_XNEG_ZPOS(IMU_Base::YPOS_XNEG_ZPOS);
    output = YPOS_XNEG_ZPOS.mapAxes(input);
    TEST_ASSERT_EQUAL(input.y, output.x);
    TEST_ASSERT_EQUAL(-input.x, output.y);
    TEST_ASSERT_EQUAL(input.z, output.z);

}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_map_axes);

    UNITY_END();
}
