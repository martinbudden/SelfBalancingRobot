#pragma once

class MotorPairController;
class ReceiverBase;
class ScreenBase;


class ButtonsBase {
public:
    ButtonsBase(MotorPairController& motorController, const ReceiverBase& receiver, ScreenBase* screen) :
        _motorController(motorController), _receiver(receiver), _screen(screen) {}
    virtual void update() = 0;
protected:
    MotorPairController& _motorController;
    const ReceiverBase& _receiver;
    ScreenBase* _screen;
};
