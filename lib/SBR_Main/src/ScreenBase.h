#pragma once

class AHRS;
class MotorPairController;
class ReceiverBase;


class ScreenBase {
public:
    virtual ~ScreenBase() = default;
    ScreenBase(const AHRS& ahrs, const MotorPairController& motorPairController, const ReceiverBase& receiver) :
        _ahrs(ahrs), _motorPairController(motorPairController), _receiver(receiver) {}
public:
    virtual void nextScreenMode() = 0;
    virtual void update(bool packetReceived) = 0;
    inline void update() { update(false); }
    inline int getScreenSizeX() const { return _screenSizeX; }
    inline int getScreenSizeY() const { return _screenSizeY; }
protected:
    const AHRS& _ahrs;
    const MotorPairController& _motorPairController;
    const ReceiverBase& _receiver;
    int _screenSizeX {};
    int _screenSizeY {};
};
