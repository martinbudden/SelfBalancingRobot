#pragma once

#include <TaskBase.h>
#include <cstdint>

class AHRS_Base;
class Backchannel;
class MotorPairController;
class Receiver;
class SBR_Preferences;
class Screen;
class Buttons;

class MainTask : public TaskBase {
public:
    void setup();
    void loop();
private:
    void checkGyroCalibration();
    void loadPreferences();
private:
    AHRS_Base* _ahrs {nullptr};
    MotorPairController* _motorPairController {nullptr};
    Receiver* _receiver {nullptr};
    SBR_Preferences* _preferences {nullptr};
    Backchannel* _backchannel {nullptr};
    uint32_t _failSafeTickCount {UINT32_MAX}; //<! failsafe counter, so the robot doesn't run away if it looses contact with the transmitter (for example by going out of range)
    uint32_t _screenTickCount {0};
    Screen* _screen {nullptr};
    int32_t _screenTemplateIsUpdated {false};
    uint32_t _buttonsTickCount {0};
    Buttons* _buttons {nullptr};
};
