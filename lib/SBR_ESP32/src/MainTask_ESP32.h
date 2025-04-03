#pragma once

#include <TaskBase.h>

class AHRS;
class Backchannel;
class MotorPairController;
class ReceiverBase;
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
    ReceiverBase* _receiver {nullptr};
    SV_Preferences* _preferences {nullptr};
    Backchannel* _backchannel {nullptr};
};
