#pragma once

#include "BlackboxCallbacksBase.h"
#include <RadioController.h>


class AHRS;
class AHRS_MessageQueue;
class MotorPairController;

class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks(AHRS_MessageQueue& messageQueue, AHRS& ahrs, MotorPairController& motorPairController, RadioController& radioController) :
        _messageQueue(messageQueue),
        _ahrs(ahrs),
        _motorPairController(motorPairController),
        _radioController(radioController),
        _receiver(radioController.getReceiver())
        {}
public:
    virtual void loadSlowState(blackboxSlowState_t& slowState) override;
    virtual void loadMainState(blackboxMainState_t& mainState, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroseconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    AHRS_MessageQueue& _messageQueue;
    AHRS& _ahrs;
    MotorPairController& _motorPairController;
    RadioController& _radioController;
    ReceiverBase& _receiver;
};
