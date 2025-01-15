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

void test_receiver_switches() {
    ReceiverTest receiver;

    uint8_t switchIndex = 0;
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 1);
    TEST_ASSERT_EQUAL(1, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 3);
    TEST_ASSERT_EQUAL(3, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));


    switchIndex = 1;
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 1);
    TEST_ASSERT_EQUAL(1, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 3);
    TEST_ASSERT_EQUAL(3, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));

    switchIndex = 2;
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 1);
    TEST_ASSERT_EQUAL(1, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 3);
    TEST_ASSERT_EQUAL(3, receiver.getSwitch(switchIndex));
    receiver.setSwitch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.getSwitch(switchIndex));

}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_receiver_switches);

    UNITY_END();
}
