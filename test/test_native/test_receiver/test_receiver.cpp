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
void ReceiverTest::broadcastMyEUI() const {}
uint32_t ReceiverTest::getAuxiliaryChannel(size_t index) const { (void)index; return 0; }

void setUp() {
}

void tearDown() {
}

void test_receiver_switches() {
    ReceiverTest receiver; // NOLINT(misc-const-correctness) false positive

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

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_switches);

    UNITY_END();
}
