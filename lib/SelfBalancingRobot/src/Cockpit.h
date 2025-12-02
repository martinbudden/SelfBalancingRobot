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
        FAILSAFE_SWITCH_OFF = 0,
        FAILSAFE_IDLE,
        FAILSAFE_RX_LOSS_DETECTED,
    };
    struct failsafe_t {
        failsafe_phase_e phase;
        uint32_t tickCount; //<! failsafe counter, so the vehicle doesn't run away if it looses contact with the transmitter (for example by going out of range)
        uint32_t tickCountLossDetectedThreshold;
        uint32_t tickCountSwitchOffThreshold;
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
    failsafe_phase_e getFailsafePhase() const { return _failsafe.phase; }

    static float mapStick(float stick, float alpha);
private:
    MotorPairController& _motorPairController;
    int32_t _onOffSwitchPressed {false}; // on/off switch debouncing
    // failsafe handling
    failsafe_config_t _failsafeConfig {};
    failsafe_t _failsafe {
        .phase = FAILSAFE_SWITCH_OFF, //initialized FAILSAFE_SWITCH_OFF, so the motors won't turn off if the transmitter hasn't been turned on yet.
        .tickCount = 0,
        .tickCountLossDetectedThreshold = 1500,
        .tickCountSwitchOffThreshold = 5000
    };
    const float _rollMaxAngleDegrees {45.0F};
    const float _pitchMaxAngleDegrees {20.0F};
};