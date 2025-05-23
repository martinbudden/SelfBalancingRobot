#pragma once

#include <TaskBase.h>

class AHRS;
class Backchannel;
class MotorPairController;
class ReceiverBase;
class SV_Preferences;
class ScreenBase;
class ButtonsBase;


class MainTask : public TaskBase {
public:
    void setup();
    void loop();
private:
    AHRS& setupAHRS(void* i2cMutex);
    static void checkGyroCalibration(SV_Preferences& preferences, AHRS& ahrs);
    static void resetPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    static void loadPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    static void setupTasks(AHRS& ahrs, MotorPairController& motorPairController);
private:
    ReceiverBase* _receiver {nullptr};
    Backchannel* _backchannel {nullptr};

    uint32_t _screenTickCount {0};
    ScreenBase* _screen {nullptr};

    uint32_t _buttonsTickCount {0};
    ButtonsBase* _buttons {nullptr};
};
