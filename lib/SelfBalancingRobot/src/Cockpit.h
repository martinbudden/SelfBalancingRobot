#pragma once

#include <CockpitBase.h>

#include <array>
#include <cstddef>
#include <cstdint>

class MotorPairController;

class Cockpit : public CockpitBase {
public:
    Cockpit(ReceiverBase& receiver, MotorPairController& motorPairController);
public:
    enum failsafe_phase_e {
        FAILSAFE_IDLE = 0,
        FAILSAFE_RX_LOSS_DETECTED,
        FAILSAFE_RX_LOSS_MONITORING,
        FAILSAFE_RX_LOSS_RECOVERED,
    };
    struct failsafe_config_t {
        uint16_t throttle_pwm;
        uint16_t throttle_low_delay_deciseconds;
        uint16_t recovery_delay_deciseconds;
        uint16_t delay_deciseconds;
        uint16_t landing_time_seconds;
        uint8_t procedure;
        uint8_t switch_mode;
    };
public:
    virtual void updateControls(const controls_t& controls) override;

    virtual void checkFailsafe(uint32_t tickCount) override;
    const failsafe_config_t& getFailsafeConfig() const { return _failsafeConfig; }
    failsafe_phase_e getFailsafePhase() const { return _failsafePhase; }


    static float mapStick(float stick, float alpha);
private:
    MotorPairController& _motorPairController;
    int32_t _onOffSwitchPressed {false}; // on/off switch debouncing
    // failsafe handling
    failsafe_phase_e _failsafePhase {FAILSAFE_IDLE};
    failsafe_config_t _failsafeConfig {};
    int32_t _receiverInUse {false};
    uint32_t _failsafeTickCount {0}; //<! failsafe counter, so the vehicle doesn't fly away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _failsafeTickCountThreshold {1500};
    uint32_t _failsafeTickCountSwitchOffThreshold {5000};
    const float _rollMaxAngleDegrees {45.0F};
    const float _pitchMaxAngleDegrees {20.0F};
};