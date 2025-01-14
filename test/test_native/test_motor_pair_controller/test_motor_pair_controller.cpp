#include "AHRS_Test.h"
#include "MotorPairController.h"
#include "ReceiverBase.h"

#include <unity.h>

class ReceiverTest : public ReceiverBase {
public:
    ReceiverTest() {};
    virtual bool update(uint32_t tickCountDelta) override;
    virtual void mapControls(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const override;
    virtual EUI_48_t getMyEUI() const override;
    virtual EUI_48_t getPrimaryPeerEUI() const override;
};

bool ReceiverTest::update(uint32_t tickCountDelta) { return true; }
void ReceiverTest::mapControls(float& throttleStick, float& rollStick, float& pitchStick, float& yawStick) const {};
ReceiverBase::EUI_48_t ReceiverTest::getMyEUI() const { return EUI_48_t{}; }
ReceiverBase::EUI_48_t ReceiverTest::getPrimaryPeerEUI() const { return EUI_48_t{}; }

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
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_motor_pair_controller);

    UNITY_END();
}
