#pragma once

#include <Blackbox.h>
#include <MotorPairController.h>

/*!
Class to write out the Blackbox header, written in writeSystemInformation()
*/
class BlackboxSelfBalancingRobot : public Blackbox {
public:
    BlackboxSelfBalancingRobot(BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice, MotorPairController& motorPairController) :
        Blackbox(motorPairController.getTaskIntervalMicroSeconds(), callbacks, serialDevice),
        _motorPairController(motorPairController)
        {}
public:
    virtual Blackbox::write_e writeSystemInformation() override;
private:
    MotorPairController& _motorPairController;
};
