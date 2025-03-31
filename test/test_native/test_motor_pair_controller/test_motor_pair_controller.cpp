#include "AHRS.h"
#include "AHRS_Test.h"
#include "MotorPairController.h"
#include "ReceiverBase.h"

#include <unity.h>

class ReceiverTest : public ReceiverBase {
public:
    virtual ~ReceiverTest() = default;
    ReceiverTest(const ReceiverTest&) = delete;
    ReceiverTest& operator=(const ReceiverTest&) = delete;
    ReceiverTest(ReceiverTest&&) = delete;
    ReceiverTest& operator=(ReceiverTest&&) = delete;
    ReceiverTest() = default;
// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual bool update(uint32_t tickCountDelta) override;
    virtual void getStickValues(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual EUI_48_t getMyEUI() const override;
    virtual EUI_48_t getPrimaryPeerEUI() const override;
    virtual void broadcastMyEUI() const override;
    virtual uint32_t getAuxiliaryChannel(size_t index) const override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
};

bool ReceiverTest::update([[maybe_unused]] uint32_t tickCountDelta) { return true; }
void ReceiverTest::getStickValues([[maybe_unused]] float& throttleStick, [[maybe_unused]] float& rollStick, [[maybe_unused]] float& pitchStick, [[maybe_unused]] float& yawStick) const {};
ReceiverBase::EUI_48_t ReceiverTest::getMyEUI() const { return EUI_48_t{}; }
ReceiverBase::EUI_48_t ReceiverTest::getPrimaryPeerEUI() const { return EUI_48_t{}; }
void ReceiverTest::broadcastMyEUI() const { }
uint32_t ReceiverTest::getAuxiliaryChannel(size_t index) const { (void)index; return 0; }


void setUp() {
}

void tearDown() {
}

void test_motor_pair_controller() {
    static SensorFusionFilterTest sensorFusionFilter; // NOLINT(misc-const-correctness) false positive
    static IMU_Test imu; // NOLINT(misc-const-correctness) false positive
    static IMU_Filters_Test imuFilters; // NOLINT(misc-const-correctness) false positive
    static AHRS ahrs(sensorFusionFilter, imu, imuFilters);
    static ReceiverTest receiver; // NOLINT(misc-const-correctness) false positive

    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    MotorPairController mpc(ahrs, receiver);
    TEST_ASSERT_FALSE(mpc.motorsIsOn());

    mpc.motorsSwitchOn();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    ahrs.setSensorFusionFilterInitializing(false);
    mpc.motorsSwitchOn();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());

    mpc.motorsToggleOnOff();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.motorsToggleOnOff();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());

    mpc.motorsSwitchOff();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.motorsSwitchOn();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());

    static const std::string pidNameRoll = mpc.getPID_Name(MotorPairController::ROLL_ANGLE_DEGREES);
    TEST_ASSERT_TRUE(pidNameRoll.compare("ROLL_ANGLE") == 0);

    static const std::string pidNamePitch = mpc.getPID_Name(MotorPairController::PITCH_ANGLE_DEGREES);
    TEST_ASSERT_TRUE(pidNamePitch.compare("PITCH_ANGLE") == 0);

    static const std::string pidNameYaw = mpc.getPID_Name(MotorPairController::YAW_RATE_DPS);
    TEST_ASSERT_TRUE(pidNameYaw.compare("YAW_RATE") == 0);

    static const std::string pidNameSpeed = mpc.getPID_Name(MotorPairController::SPEED_DPS);
    TEST_ASSERT_TRUE(pidNameSpeed.compare("SPEED") == 0);

    static const std::string pidNamePosition = mpc.getPID_Name(MotorPairController::POSITION_DEGREES);
    TEST_ASSERT_TRUE(pidNamePosition.compare("POSITION") == 0);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);

    UNITY_END();
}
