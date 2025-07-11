#pragma once

#include "BlackboxCallbacksBase.h"

class AHRS;
class MotorPairController;
class RadioController;
class ReceiverBase;


class BlackboxCallbacksSelfBalancingRobot : public BlackboxCallbacksBase {
public:
    BlackboxCallbacksSelfBalancingRobot(AHRS& ahrs, MotorPairController& motorPairController, RadioController& radioController, ReceiverBase& receiver) :
        _ahrs(ahrs),
        _motorPairController(motorPairController),
        _radioController(radioController),
        _receiver(receiver)
        {}
public:
    virtual void loadSlowStateFromFlightController(blackboxSlowState_t& slowState) override;
    //!! Fill the current state of the blackbox using values read from the flight controller
    virtual void loadMainStateFromFlightController(blackboxMainState_t& mainState) override;
    virtual void loadMainStateFromFlightController(blackboxMainState_t& mainState, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroSeconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    AHRS& _ahrs;
    MotorPairController& _motorPairController;
    RadioController& _radioController;
    ReceiverBase& _receiver;
};
