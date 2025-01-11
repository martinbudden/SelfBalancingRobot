#include "AHRS_Test.h"
#include "MotorPairController.h"
#include "ReceiverBase.h"

#include <unity.h>

class ReceiverTest : public ReceiverBase {
public:
    ReceiverTest() {};
    virtual bool update(uint32_t tickCountDelta) override;
    virtual void mapControls(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
};

bool ReceiverTest::update(uint32_t tickCountDelta) { return true; }
void ReceiverTest::mapControls(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const {};

void setUp() {
}

void tearDown() {
}

void test_motor_pair_controller() {
    AHRS_Test ahrs;
    ReceiverTest receiver;
    MotorPairController mpc(ahrs, receiver);
    TEST_ASSERT_FALSE(mpc.motorsIsOn());
    mpc.motorsSwitchOn();
    TEST_ASSERT_TRUE(mpc.motorsIsOn());
    mpc.motorsToggleOnOff();
    TEST_ASSERT_FALSE(mpc.motorsIsOn());

    TEST_ASSERT_TRUE(mpc.getPitchRateIsFiltered());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);

    UNITY_END();
}
