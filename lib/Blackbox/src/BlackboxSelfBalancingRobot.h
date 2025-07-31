#pragma once

#include <Blackbox.h>
#include <MotorPairController.h>

/*!
Class to write out the Blackbox header, written in writeSystemInformation()
*/
class BlackboxSelfBalancingRobot : public Blackbox {
public:
    BlackboxSelfBalancingRobot(BlackboxCallbacksBase& callbacks, BlackboxMessageQueueBase& messageQueue, BlackboxSerialDevice& serialDevice, MotorPairController& motorPairController) :
        Blackbox(motorPairController.getTaskIntervalMicroSeconds(), callbacks, messageQueue, serialDevice),
        _motorPairController(motorPairController)
        {}
public:
    virtual Blackbox::write_e writeSystemInformation() override;
private:
    MotorPairController& _motorPairController;
};
