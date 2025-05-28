#include "AHRS.h"
#include "IMU_FiltersBase.h"
#include <IMU_Null.h>
#include <SensorFusion.h>
#include <unity.h>

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif


class IMU_Filters_Test : public IMU_FiltersBase {
public:
    virtual ~IMU_Filters_Test() = default;
    IMU_Filters_Test() = default;

    // IMU_Filters_Test is not copyable or moveable
    IMU_Filters_Test(const IMU_Filters_Test&) = delete;
    IMU_Filters_Test& operator=(const IMU_Filters_Test&) = delete;
    IMU_Filters_Test(IMU_Filters_Test&&) = delete;
    IMU_Filters_Test& operator=(IMU_Filters_Test&&) = delete;

    void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
};
void IMU_Filters_Test::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) { (void)gyroRPS; (void)acc; (void)deltaT; }


void setUp()
{
}

void tearDown()
{
}

void test_ahrs()
{
    MadgwickFilter sensorFusionFilter; // NOLINT(misc-const-correctness)
    IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS); // NOLINT(misc-const-correctness) false positive
    IMU_Filters_Test imuFilters; // NOLINT(misc-const-correctness) false positive
    AHRS ahrs(AHRS_TASK_INTERVAL_MICROSECONDS, sensorFusionFilter, imu, imuFilters);

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
