#pragma once

#include <Blackbox.h>
#include <MotorPairController.h>

/*!
Class to write out the Blackbox header, written in blackboxWriteSysinfo()
*/
class BlackboxSelfBalancingRobot : public Blackbox {
public:
    BlackboxSelfBalancingRobot(BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice, MotorPairController& motorPairController) :
        Blackbox(motorPairController.getTaskIntervalMicroSeconds(), callbacks, serialDevice),
        _motorPairController(motorPairController)
        {}
public:
    virtual bool blackboxWriteSysinfo() override;
private:
    MotorPairController& _motorPairController;
};
