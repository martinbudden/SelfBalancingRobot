#include "MotorPairController.h"
#include "RadioController.h"
#include <cmath>

/*!
Map a control stick to a parabolic curve to give more control for small values of yaw.

Higher values of alpha increase the effect
    alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
*/
float RadioController::mapStick(float stick, float alpha)
{
    stick *= 1.0F - alpha*(1.0F - (stick < 0.0F ? - stick : stick));
    return stick;
}

/*!
Called from within ReceiverTask::loop()
*/
void RadioController::updateControls(const controls_t& controls)
{
    // failsafe handling
    _receiverInUse = true;
    _failsafePhase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it

    if (_receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // motorOnOff false and _onOffPressed true means the  on/off button is being released, so toggle the motor state
            _motorPairController->motorsToggleOnOff();
            _onOffSwitchPressed = false;
        }
    }

    // alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
    static constexpr float alpha { 0.2F };
    MotorPairController::controls_t mpcControls = {
        .tickCount = controls.tickCount,
        .throttleStick = controls.throttleStick,
        .rollStickDegrees = controls.rollStick * _rollMaxAngleDegrees,
        .pitchStickDegrees = controls.pitchStick * _pitchMaxAngleDegrees,
        .yawStickDPS = mapStick(controls.yawStick, alpha) // map the YAW stick values to give better control at low stick values
    };

    _motorPairController->updateSetpoints(mpcControls);
}

uint32_t RadioController::getFailsafePhase() const
{
    return _failsafePhase;
}

void RadioController::setFailsafe(const failsafe_t& failsafe)
{
    _failsafe = failsafe;
}

void RadioController::checkFailsafe(uint32_t tickCount)
{
    if ((tickCount - _failsafeTickCount > _failsafeTickCountThreshold) && _receiverInUse) {
        // _receiverInUse is initialized to false, so the motors won't turn off it the transmitter hasn't been turned on yet.
        // We've had 1500 ticks (1.5 seconds) without a packet, so we seem to have lost contact with the transmitter,
        // so enter failsafe mode.
        _failsafePhase = FAILSAFE_RX_LOSS_DETECTED;
        if ((tickCount - _failsafeTickCount > _failsafeTickCountSwitchOffThreshold)) {
            _motorPairController->motorsSwitchOff();
            _receiverInUse = false; // set to false to allow us to switch the motors on again if we regain a signal
        }
    }
}
