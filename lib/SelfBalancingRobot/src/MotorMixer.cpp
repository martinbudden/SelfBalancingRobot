#include "MotorMixer.h"

#include <MotorMixerBase.h>
#include <MotorPairBase.h>
#include <cmath>
#if defined(AHRS_RECORD_TIMES_CHECKS)
#include <esp32-hal.h>
#endif

#if !defined(UNIT_TEST_BUILD)
//#define SERIAL_OUTPUT
#if defined(SERIAL_OUTPUT)
#include <HardwareSerial.h>
#endif
#endif


void MotorMixer::outputToMotors(const output_t& outputs, float deltaT, uint32_t tickCount)
{
    (void) deltaT;

    // Disable the motors if the pitchAngle exceeds the switchOffAngle.
    // Don't switch on again for at least 2 seconds after robot falls over (ie don't switch on if it falls over and bounces back up again).
    constexpr uint32_t robotDebounceIntervalMs = 2000;
    _motorsIsDisabled = (fabs(_pitchAngleDegreesRaw) >= _motorSwitchOffAngleDegrees) || ((tickCount - _motorSwitchOffTickCount) < robotDebounceIntervalMs);

    if (_motorsIsOn && !_motorsIsDisabled) {
        _motorSwitchOffTickCount = 0; // reset the bounce prevention tickcount

        _powerLeft  = outputs.pitch + outputs.speed - outputs.yaw;
        _powerRight = outputs.pitch + outputs.speed + outputs.yaw;

        // filter the power input into the motors so they run more smoothly.
        const float powerLeftFiltered = _powerLeftFilter.update(_powerLeft);
        const float powerRightFiltered = _powerRightFilter.update(_powerRight);
#if defined(AHRS_RECORD_TIMES_CHECKS)
        const uint32_t timeMicroSeconds0 = micros();
#endif
        _motors.setPower(powerLeftFiltered, powerRightFiltered);
#if defined(AHRS_RECORD_TIMES_CHECKS)
        _outputPowerTimeMicroSeconds = micros() - timeMicroSeconds0;
#endif
    } else {
        if (_motorSwitchOffTickCount == 0) { // the motors haven't already been switched off
            // Record the current tickCount so we can stop the motors turning back on if the robot bounces when it falls over.
            _motorSwitchOffTickCount = tickCount;
        }
        // Motors switched off, so set everything to zero, ready for motors to be switched on again.
        _motors.setPower(0.0F, 0.0F);
        _powerLeft  = 0.0F;
        _powerRight = 0.0F;
        _powerLeftFilter.reset();
        _powerRightFilter.reset();
    }
#if defined(SERIAL_OUTPUT)
    static int loopCount {0};
    ++loopCount;
    if (loopCount == 1) {
        loopCount = 0;

    //Serial.printf(">pitchPidErrorP:%8.2f, pidErrorI:%8.2f, pidErrorD:%8.2f, update:%8.2f\r\n",
    //    _pitchPID.getError().P, _pitchPID.getError().I, _pitchPID.getError().D, _outputs[PITCH_ANGLE_DEGREES]);

    //Serial.printf(">pitchSetpoint:%7.2f, pitchAngleDegrees:%6.2f, pitchOutput:%8.4f, speedSetpoint:%7.2f, speedOutput:%7.3f, speedError:%7.3f\r\n",
    //   _pitchPID.getSetpoint(), _pitchAngleDegreesRaw, _outputs[PITCH_ANGLE_DEGREES], _PIDS[SPEED_DPS].getSetpoint(), _outputs[SPEED_DPS], _PIDS[SPEED_DPS].getError().P/_PIDS[SPEED_DPS].getP());

    //Serial.printf(">pitchSetpoint:%7.2f, pitchAngleDegrees:%6.2f, pitchOutput:%8.4f, speedSetpoint:%7.2f, speedOutput:%7.3f\r\n",
    //   _pitchPID.getSetpoint(), _pitchAngleDegreesRaw, _outputs[PITCH_ANGLE_DEGREES], _PIDS[SPEED_DPS].getSetpoint(), _outputs[SPEED_DPS]);

    //Serial.printf(">pitchAngleDegrees:%6.2f, pitchOutput:%6.3f, speedDPS:%5.0F, speedOutput:%8.5f\r\n",
    //    _pitchAngleDegreesRaw, _outputs[PITCH_ANGLE_DEGREES], _speedDPS, _outputs[SPEED_DPS]);

    Serial.printf(">speed:%8.2f, setpoint:%8.2f, pidErrorP:%8.2f, update:%8.2f, eL:%d, eR:%d\r\n",
        _speedDPS, _PIDS[SPEED_DPS].getSetpoint(), _PIDS[SPEED_DPS].getError().P, _outputs[SPEED_DPS], _encoderLeftDelta, _encoderRightDelta);
    }
#endif
}

