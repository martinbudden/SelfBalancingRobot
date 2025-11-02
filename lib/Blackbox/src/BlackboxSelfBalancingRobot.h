#pragma once

#include <Blackbox.h>
#include <MotorPairController.h>


/*!
Class to write out the Blackbox header, written in writeSystemInformation()
*/
class BlackboxSelfBalancingRobot : public Blackbox {
public:
    BlackboxSelfBalancingRobot(uint32_t pidLooptimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice, const MotorPairController& motorPairController) :
        Blackbox(pidLooptimeUs, callbacks, serialDevice),
        _motorPairController(motorPairController)
        {}
public:
    virtual Blackbox::write_e writeSystemInformation() override;
private:
    const MotorPairController& _motorPairController;
};
