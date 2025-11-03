#pragma once

#include "Targets.h"

#include <TaskBase.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#endif
#endif

class AHRS;
class AHRS_MessageQueue;
class AHRS_Task;
class BackchannelBase;
class BackchannelTask;
class Blackbox;
class BlackboxTask;
class IMU_Base;
class IMU_FiltersBase;
class MotorPairController;
class NonVolatileStorage;
class RadioController;
class ReceiverBase;
class ReceiverTask;
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

#define AHRS_TASK_IS_TIMER_DRIVEN

#if !defined(DASHBOARD_TASK_INTERVAL_MICROSECONDS)
enum { DASHBOARD_TASK_INTERVAL_MICROSECONDS = 10000 };
#endif

#if !defined(GYRO_SAMPLE_RATE_HZ)
enum { GYRO_SAMPLE_RATE_HZ = 200 };
#endif

#if !defined(OUTPUT_TO_MOTORS_DENOMINATOR)
enum { OUTPUT_TO_MOTORS_DENOMINATOR = 1 };
#endif

#if !defined(BACKCHANNEL_TASK_INTERVAL_MICROSECONDS)
enum { BACKCHANNEL_TASK_INTERVAL_MICROSECONDS = 10000 };
#endif

#if !defined(BLACKBOX_TASK_INTERVAL_MICROSECONDS)
enum { BLACKBOX_TASK_INTERVAL_MICROSECONDS = 2000 };
#endif


enum {
    AHRS_TASK_PRIORITY = 6,
    MPC_TASK_PRIORITY = 5,
    RECEIVER_TASK_PRIORITY = MPC_TASK_PRIORITY,
    BACKCHANNEL_TASK_PRIORITY = 3,
    MSP_TASK_PRIORITY = 2,
    BLACKBOX_TASK_PRIORITY = 3,
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
    BLACKBOX_TASK_CORE = PRO_CPU_NUM,
};


class DashboardTask : public TaskBase {
public:
    explicit DashboardTask(uint32_t taskIntervalMicroseconds) : TaskBase(taskIntervalMicroseconds) {}
    void loop();
};

class Main {
public:
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7}; // Note: defining PI=8 will cause conflict with Arduino's #define of PI (3.14..)
    enum {P0=0, P1=1, P2=2, P3=3, P4=4, P5=5, P6=6, P7=7};
    enum calibration_type_e { CALIBRATE_ACC_AND_GYRO, CALIBRATE_GYRO_ONLY };
public:
    Main() = default;
public:
    void setup();
    void loop();
private:
    void checkStackUsage();
    static IMU_Base& createIMU(void* i2cMutex);
    static AHRS& createAHRS(VehicleControllerBase& vehicleController, IMU_Base& imuSensor);
    static ReceiverBase& createReceiver();
    static BackchannelBase& createBackchannel(MotorPairController& motorPairController, AHRS& ahrs, ReceiverBase& receiver, const TaskBase* dashboardTask, NonVolatileStorage& nvs);
    static Blackbox& createBlackBox(AHRS& ahrs, MotorPairController& motorPairController, AHRS_MessageQueue& ahrsMessageQueue, RadioController& radioController);
    static void checkIMU_Calibration(NonVolatileStorage& nonVolatileStorage, AHRS& ahrs);
    static void runIMU_Calibration(NonVolatileStorage& nonVolatileStorage, AHRS& ahrs, calibration_type_e calibrationType);
    static void calibrateIMU(NonVolatileStorage& nonVolatileStorage, AHRS& ahrs, calibration_type_e calibrationType);
    static void clearSettings(NonVolatileStorage& nonVolatileStorage, MotorPairController& motorPairController, AHRS& ahrs);
    static void loadSettings(NonVolatileStorage& nonVolatileStorage, MotorPairController& motorPairController);
    static void reportDashboardTask();

    struct tasks_t {
        DashboardTask* dashboardTask;

        AHRS_Task* ahrsTask;
        TaskBase::task_info_t ahrsTaskInfo;

        VehicleControllerTask* vehicleControllerTask;
        TaskBase::task_info_t vehicleControllerTaskInfo;

        ReceiverTask* receiverTask;
        TaskBase::task_info_t receiverTaskInfo;

        BackchannelTask* backchannelTask;
        TaskBase::task_info_t backchannelTaskInfo;

        BlackboxTask* blackboxTask;
        TaskBase::task_info_t blackboxTaskInfo;
    };
private:
    tasks_t _tasks {};

    uint32_t _screenTickCount {0};
    ScreenBase* _screen {nullptr};

    uint32_t _buttonsTickCount {0};
    ButtonsBase* _buttons {nullptr};
};
