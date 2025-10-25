#pragma once

#include <RadioControllerBase.h>

#include <array>
#include <cstddef>
#include <cstdint>

class MotorPairController;

class RadioController : public RadioControllerBase {
public:
    explicit RadioController(ReceiverBase& receiver, MotorPairController& motorPairController);
public:
    enum failsafe_phase_e {
        FAILSAFE_IDLE = 0,
        FAILSAFE_RX_LOSS_DETECTED,
        FAILSAFE_LANDING,
        FAILSAFE_LANDED,
        FAILSAFE_RX_LOSS_MONITORING,
        FAILSAFE_RX_LOSS_RECOVERED,
        FAILSAFE_GPS_RESCUE
    };
    struct failsafe_t {
        uint8_t delay;
        uint8_t landing_time;
        uint8_t switch_mode;
        uint8_t procedure;
        uint16_t throttle;
        uint16_t throttle_low_delay;
    };
public:
    virtual void updateControls(const controls_t& controls) override;

    virtual void checkFailsafe(uint32_t tickCount) override;
    const failsafe_t& getFailsafe() const { return _failsafe; }
    failsafe_phase_e getFailsafePhase() const { return _failsafePhase; }


    static float mapStick(float stick, float alpha);
private:
    MotorPairController& _motorPairController;
    int32_t _onOffSwitchPressed {false}; // on/off switch debouncing
    // failsafe handling
    failsafe_phase_e _failsafePhase {FAILSAFE_IDLE};
    failsafe_t _failsafe {};
    int32_t _receiverInUse {false};
    uint32_t _failsafeTickCount {0}; //<! failsafe counter, so the vehicle doesn't fly away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _failsafeTickCountThreshold {1500};
    uint32_t _failsafeTickCountSwitchOffThreshold {5000};
    const float _rollMaxAngleDegrees {45.0};
    const float _pitchMaxAngleDegrees {20.0};
};