#pragma once

#include "BlackboxCallbacksBase.h"

class AHRS;
class MotorPairController;
class ReceiverBase;


class BlackboxCallbacksSelfBalancingRobot : public BlackboxCallbacksBase {
public:
    BlackboxCallbacksSelfBalancingRobot(AHRS& ahrs, MotorPairController& motorPairController, ReceiverBase& receiver) :
        _ahrs(ahrs),
        _motorPairController(motorPairController),
        _receiver(receiver)
        {}
public:
    virtual void loadSlowStateFromFlightController(blackboxSlowState_t& slowState) override;
    //!! Fill the current state of the blackbox using values read from the flight controller
    virtual void loadMainStateFromFlightController(blackboxMainState_t& blackboxCurrent) override;
    virtual void loadMainStateFromFlightController(blackboxMainState_t& blackboxCurrent, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroSeconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    AHRS& _ahrs;
    MotorPairController& _motorPairController;
    ReceiverBase& _receiver;
};
