#include "MotorPairController.h"
#include "Cockpit.h"
#include <ReceiverBase.h>


Cockpit::Cockpit(ReceiverBase& receiver, MotorPairController& motorPairController) :
    CockpitBase(receiver),
    _motorPairController(motorPairController)
{
}

/*!
Map a control stick to a parabolic curve to give more control for small values of yaw.

Higher values of alpha increase the effect
    alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
*/
float Cockpit::mapStick(float stick, float alpha)
{
    stick *= 1.0F - alpha*(1.0F - (stick < 0.0F ? - stick : stick));
    return stick;
}

/*!
Called from within ReceiverTask::loop()
*/
void Cockpit::updateControls(const controls_t& controls)
{
    // failsafe handling
    _failsafe.phase = FAILSAFE_IDLE; // we've received a packet, so exit failsafe if we were in it
    _failsafe.tickCount = controls.tickCount;

    if (_receiver.getSwitch(ReceiverBase::MOTOR_ON_OFF_SWITCH)) {
        _onOffSwitchPressed = true;
    } else {
        if (_onOffSwitchPressed) {
            // motorOnOff false and _onOffPressed true means the  on/off button is being released, so toggle the motor state
            if (_motorPairController.motorsIsOn()) {
                _motorPairController.motorsSwitchOff();
            } else {
                _motorPairController.motorsSwitchOn();
            }
            _onOffSwitchPressed = false;
        }
    }
    const MotorPairController::control_mode_e controlMode = MotorPairController::CONTROL_MODE_SERIAL_PIDS;
    // alpha=0 gives a linear response, alpha=1 gives a parabolic (x^2) curve
    static constexpr float alpha { 0.2F };
    const MotorPairController::controls_t mpcControls = {
        .tickCount = controls.tickCount,
        .throttleStick = controls.throttleStick,
        .rollStickDegrees = controls.rollStick * _rollMaxAngleDegrees,
        .pitchStickDegrees = controls.pitchStick * _pitchMaxAngleDegrees,
        .yawStickDPS = mapStick(controls.yawStick, alpha), // map the YAW stick values to give better control at low stick values
        .controlMode = controlMode
    };
    _motorPairController.updateSetpoints(mpcControls);
}

void Cockpit::checkFailsafe(uint32_t tickCount)
{
    // _failsafe.phase is initialized FAILSAFE_SWITCH_OFF, so the motors won't turn off if the transmitter hasn't been turned on yet.
    if ((tickCount - _failsafe.tickCount > _failsafe.tickCountLossDetectedThreshold) && (_failsafe.phase != FAILSAFE_SWITCH_OFF)) {
        // We've had 1500 ticks (1.5 seconds) without a packet, so we seem to have lost contact with the transmitter,
        if ((tickCount - _failsafe.tickCount < _failsafe.tickCountSwitchOffThreshold)) {
            // first phase of failsafe: set all controls to zero so the vehicle stops and holds its current position
            _failsafe.phase = FAILSAFE_RX_LOSS_DETECTED;
            const MotorPairController::controls_t controls = {
                .tickCount = tickCount,
                .throttleStick = 0.0F,
                .rollStickDegrees = 0.0F,
                .pitchStickDegrees = 0.0F,
                .yawStickDPS = 0.0F,
                .controlMode = MotorPairController::CONTROL_MODE_SERIAL_PIDS
             };
            _motorPairController.updateSetpoints(controls);
        } else {
            // second phase of failsafe, switch the motors off
            _failsafe.phase = FAILSAFE_SWITCH_OFF; // this will allow us to switch the motors on again if we regain a signal
            _motorPairController.motorsSwitchOff();
        }
    }
}
