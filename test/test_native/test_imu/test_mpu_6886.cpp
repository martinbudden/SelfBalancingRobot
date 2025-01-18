#include <IMU_MPU6886.h>
#include <unity.h>

void setUp() {
}

void tearDown() {
}

void test_mpu_6886() {
    xyz_int16_t input { .x =70, .y = -70, .z = 400 };
    IMU_MPU6886::mems_sensor_data_t output = IMU_MPU6886::gyroOffsetFromXYZ(input);

    TEST_ASSERT_EQUAL_UINT8(0xFF, output.x_h);
    TEST_ASSERT_EQUAL_UINT8(0xBA, output.x_l);

    TEST_ASSERT_EQUAL_UINT8(0x00, output.y_h);
    TEST_ASSERT_EQUAL_UINT8(0x46, output.y_l);

    TEST_ASSERT_EQUAL_UINT8(0xFE, output.z_h);
    TEST_ASSERT_EQUAL_UINT8(0x70, output.z_l);

}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_mpu_6886);

    UNITY_END();
}
