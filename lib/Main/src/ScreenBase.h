#pragma once

#include <ReceiverBase.h>

class AHRS;
class MotorPairController;


class ScreenBase  : public ReceiverWatcher {
public:
    virtual ~ScreenBase() = default;
    ScreenBase(const AHRS& ahrs, const MotorPairController& motorPairController, const ReceiverBase& receiver) :
        _ahrs(ahrs), _motorPairController(motorPairController), _receiver(receiver) {}
public:
    virtual void nextScreenMode() = 0;
    virtual void update() = 0;
    void updateScreenAndTemplate() { _templateIsUpdated = false; update(); }
    inline int getScreenSizeX() const { return _screenSizeX; }
    inline int getScreenSizeY() const { return _screenSizeY; }

    virtual void newReceiverPacketAvailable() override { _newReceiverPacketAvailable = true; }
protected:
    inline void clearNewReceiverPacketAvailable() { _newReceiverPacketAvailable = false; }
    inline bool isNewReceiverPacketAvailable() const { return _newReceiverPacketAvailable; }
protected:
    const AHRS& _ahrs;
    const MotorPairController& _motorPairController;
    const ReceiverBase& _receiver;
    int _screenSizeX {};
    int _screenSizeY {};
    int _newReceiverPacketAvailable {};
    int _templateIsUpdated {false};
    int _remoteEUI_updated {false};
};
