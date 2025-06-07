#pragma once

class MotorPairController;
class ReceiverBase;
class ScreenBase;


class ButtonsBase {
public:
    virtual ~ButtonsBase() = default;
    ButtonsBase(MotorPairController& motorPairController, const ReceiverBase& receiver, ScreenBase* screen) :
        _motorPairController(motorPairController), _receiver(receiver), _screen(screen) {}
    virtual void update() = 0;
protected:
    MotorPairController& _motorPairController;
    const ReceiverBase& _receiver;
    ScreenBase* _screen;
};
