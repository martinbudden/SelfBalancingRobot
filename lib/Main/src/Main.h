#pragma once

#include <SV_Tasks.h>
#include <TaskBase.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#endif

class AHRS;
class AHRS_Task;
class BackchannelBase;
class BackchannelTask;
class MotorPairController;
class ReceiverTask;
class SV_Preferences;
class VehicleControllerBase;
class VehicleControllerTask;

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

#if !defined(BACKCHANNEL_TASK_INTERVAL_MICROSECONDS)
enum { BACKCHANNEL_TASK_INTERVAL_MICROSECONDS = 10000 };
#endif

enum {
    AHRS_TASK_PRIORITY = 6,
    MPC_TASK_PRIORITY = 5,
    RECEIVER_TASK_PRIORITY = MPC_TASK_PRIORITY,
    BACKCHANNEL_TASK_PRIORITY = 3,
    MSP_TASK_PRIORITY = 2
};

#if !defined(PRO_CPU_NUM)
#define PRO_CPU_NUM (0)
#endif
#if !defined(APP_CPU_NUM)
// the processor has only one core
#define APP_CPU_NUM PRO_CPU_NUM
#endif

enum {
    AHRS_TASK_CORE = APP_CPU_NUM, // AHRS should be the only task running on the second core
    MPC_TASK_CORE = PRO_CPU_NUM,
    RECEIVER_TASK_CORE = PRO_CPU_NUM,
    BACKCHANNEL_TASK_CORE = PRO_CPU_NUM,
    MSP_TASK_CORE = PRO_CPU_NUM,
};


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
    void checkStackUsage();
    AHRS& setupAHRS(void* i2cMutex);
    static void checkGyroCalibration(SV_Preferences& preferences, AHRS& ahrs);
    static void resetPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    static void loadPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    struct tasks_t {
        MainTask* mainTask;

        AHRS_Task* ahrsTask;
        SV_Tasks::task_info_t ahrsTaskInfo;

        VehicleControllerTask* vehicleControllerTask;
        SV_Tasks::task_info_t vehicleControllerTaskInfo;

        ReceiverTask* receiverTask;
        SV_Tasks::task_info_t receiverTaskInfo;

        BackchannelTask* backchannelTask;
        SV_Tasks::task_info_t backchannelTaskInfo;
    };
private:
    tasks_t _tasks {};

    uint32_t _screenTickCount {0};
    ScreenBase* _screen {nullptr};

    uint32_t _buttonsTickCount {0};
    ButtonsBase* _buttons {nullptr};
};
