#pragma once

#include <TaskBase.h>

class AHRS;
class Backchannel;
class MotorPairController;
class Receiver;
class SV_Preferences;
class Screen;
class Buttons;

class MainTask : public TaskBase {
public:
    void setup();
    void loop();
private:
    void setupAHRS(void* i2cMutex);
    void checkGyroCalibration();
    void resetPreferences();
    void loadPreferences();
    void setupTasks();
private:
    AHRS* _ahrs {nullptr};
    MotorPairController* _motorPairController {nullptr};
    Receiver* _receiver {nullptr};
    SV_Preferences* _preferences {nullptr};
    Backchannel* _backchannel {nullptr};
    uint32_t _failSafeTickCount {}; //<! failsafe counter, so the robot doesn't run away if it looses contact with the transmitter (for example by going out of range)
    int32_t _receiverInUse {false};

    uint32_t _screenTickCount {0};
    Screen* _screen {nullptr};
    int32_t _screenTemplateIsUpdated {false};

    uint32_t _buttonsTickCount {0};
    Buttons* _buttons {nullptr};
};
