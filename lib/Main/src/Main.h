#pragma once

#include <TaskBase.h>

class AHRS;
class AHRS_Task;
class Backchannel;
class MotorPairController;
class MotorPairControllerTask;
class ReceiverBase;
class ReceiverTask;
class ReceiverWatcher;
class SV_Preferences;
class ScreenBase;
class ButtonsBase;

#if !defined(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS)
enum { MAIN_LOOP_TASK_INTERVAL_MICROSECONDS = 10000 };
#endif

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif

#if !defined(MPC_TASK_INTERVAL_MICROSECONDS)
#if defined(USE_IMU_M5_UNIFIED) || defined(USE_IMU_M5_STACK)
    enum { MPC_TASK_INTERVAL_MICROSECONDS = 10000 }; // M5Stack IMU code blocks I2C bus for extended periods, so MPC_TASK must be set to run slower.
#else
    enum { MPC_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif
#endif

#if !defined(RECEIVER_TASK_INTERVAL_MICROSECONDS)
enum { RECEIVER_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif


class MainTask : public TaskBase {
public:
    explicit MainTask(uint32_t taskIntervalMicroSeconds) : TaskBase(taskIntervalMicroSeconds) {}
    void loop();
};

class Main {
public:
    Main() = default;
public:
    void setup();
    void loop();
private:
    AHRS& setupAHRS(void* i2cMutex);
    static void checkGyroCalibration(SV_Preferences& preferences, AHRS& ahrs);
    static void resetPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    static void loadPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    struct tasks_t {
        MainTask* mainTask;
        AHRS_Task* ahrsTask;
        MotorPairControllerTask* mpcTask;
        ReceiverTask* receiverTask;
    };
    void setupTasks(tasks_t& tasks, AHRS& ahrs, MotorPairController& motorPairController, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher);
    tasks_t _tasks {};
private:
    AHRS* _ahrs {nullptr};
    MotorPairController* _motorPairController {};
    ReceiverBase* _receiver {nullptr};
    Backchannel* _backchannel {nullptr};

    uint32_t _screenTickCount {0};
    ScreenBase* _screen {nullptr};

    uint32_t _buttonsTickCount {0};
    ButtonsBase* _buttons {nullptr};
};
