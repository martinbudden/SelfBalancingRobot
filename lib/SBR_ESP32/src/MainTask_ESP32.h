#pragma once

#include <TaskBase.h>
#include <cstdint>

class AHRS;
class Backchannel;
class MotorPairController;
class Receiver;
class SV_Preferences;

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
};
