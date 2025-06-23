#pragma once

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

/*!
The ESP32S3 is dual core containing a Protocol CPU (known as CPU 0 or PRO_CPU) and an Application CPU (known as CPU 1 or APP_CPU).

The core affinities, priorities, and tick intervals for the 3 application tasks (AHRS_TASK, VEHICLE_CONTROLLER_TASK, and MAIN_LOOP_TASK):
1. The AHRS_TASK must have a higher priority than the MAIN_LOOP_TASK.
2. The VEHICLE_CONTROLLER_TASK must have a higher priority than the MAIN_LOOP_TASK
3. For single-processors the AHRS_TASK and the VEHICLE_CONTROLLER_TASK must have the same priority.
4. For dual-core processors
    1. The AHRS_TASK runs on the Application CPU (CPU 1).
    2. The VEHICLE_CONTROLLER_TASK runs on the Protocol CPU (CPU 0).
5. The MAIN_LOOP_TASK runs on the Application CPU (CPU 1) with priority 1 (this is set by the ESP32 Arduino framework).

The AHRS_TASK and the VEHICLE_CONTROLLER_TASK are deliberately chosen to run on different cores on ESP32 dual-core processors. This is so
that a context switch between the AHRS_TASK and the VEHICLE_CONTROLLER_TASK does not require saving the FPU(Floating Point Unit) registers
(see https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/api-guides/freertos-smp.html#floating-point-usage).

The Atom JoyStick transmits a packet every 10ms.
Updating the screen takes approximately 50 ticks.
*/


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
    static void checkIMU_Calibration(SV_Preferences& preferences, AHRS& ahrs);
    static void runIMU_Calibration(SV_Preferences& preferences, AHRS& ahrs);
    static void calibrateIMU(SV_Preferences& preferences, AHRS& ahrs);
    static void resetPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    static void loadPreferences(SV_Preferences& preferences, MotorPairController& motorPairController);
    static void reportMainTask();

    struct tasks_t {
        MainTask* mainTask;

        AHRS_Task* ahrsTask;
        task_info_t ahrsTaskInfo;

        VehicleControllerTask* vehicleControllerTask;
        task_info_t vehicleControllerTaskInfo;

        ReceiverTask* receiverTask;
        task_info_t receiverTaskInfo;

        BackchannelTask* backchannelTask;
        task_info_t backchannelTaskInfo;
    };
private:
    tasks_t _tasks {};

    uint32_t _screenTickCount {0};
    ScreenBase* _screen {nullptr};

    uint32_t _buttonsTickCount {0};
    ButtonsBase* _buttons {nullptr};
};
