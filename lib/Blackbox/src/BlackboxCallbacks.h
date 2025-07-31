#pragma once

#include "BlackboxCallbacksBase.h"

class AHRS;
class BlackboxMessageQueue;
class MotorPairController;
class RadioController;
class ReceiverBase;


class BlackboxCallbacks : public BlackboxCallbacksBase {
public:
    BlackboxCallbacks(BlackboxMessageQueue& messageQueue, AHRS& ahrs, MotorPairController& motorPairController, RadioController& radioController, ReceiverBase& receiver) :
        _messageQueue(messageQueue),
        _ahrs(ahrs),
        _motorPairController(motorPairController),
        _radioController(radioController),
        _receiver(receiver)
        {}
public:
    virtual void loadSlowState(blackboxSlowState_t& slowState) override;
    virtual void loadMainState(blackboxMainState_t& mainState, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroSeconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
private:
    BlackboxMessageQueue& _messageQueue;
    AHRS& _ahrs;
    MotorPairController& _motorPairController;
    RadioController& _radioController;
    ReceiverBase& _receiver;
};
