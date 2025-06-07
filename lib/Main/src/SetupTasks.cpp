#include "Main.h"

#include "TelemetryScaleFactors.h"

#include <AHRS_Task.h>

#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#include <MotorPairControllerTask.h>
#include <ReceiverTask.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif


/*!
The ESP32S3 is dual core containing a Protocol CPU (known as CPU 0 or PRO_CPU) and an Application CPU (known as CPU 1 or APP_CPU).

The core affinities, priorities, and tick intervals and priorities for the 3 application tasks (AHRS_TASK, MPC_TASK, and MAIN_LOOP_TASK).
1. The AHRS_TASK must have a higher priority than the MAIN_LOOP_TASK.
2. The MPC_TASK must have a higher priority than the MAIN_LOOP_TASK
3. For single-processors the AHRS_TASK and the MPC_TASK must have the same priority.
4. For dual-core processors
    1. The AHRS_TASK runs on the Application CPU (CPU 1).
    2. The MPC_TASK runs on the Protocol CPU (CPU 0).
5. The MAIN_LOOP_TASK runs on the Application CPU (CPU 1) with priority 1 (this is set by the ESP32 Arduino framework).

The AHRS_TASK and the MPC_TASK are deliberately chosen to run on different cores on ESP32 dual-core processors. This is so
that a context switch between the AHRS_TASK and the MPC_TASK does not require saving the FPU(Floating Point Unit) registers
(see https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/api-guides/freertos-smp.html#floating-point-usage).

The Atom JoyStick transmits a packet every 10ms, so the MAIN_LOOP_TASK must run at least every 10m to ensure no dropped packets.
Updating the screen takes approximately 50 ticks, so packets will be dropped if the screen is not in QRCODE mode.
*/


void Main::setupTasks(tasks_t& tasks, AHRS& ahrs, MotorPairController& motorPairController, ReceiverBase& receiver, ReceiverWatcher* receiverWatcher)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static MainTask mainTask(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS);
    tasks.mainTask = &mainTask;
    static AHRS_Task ahrsTask(AHRS_TASK_INTERVAL_MICROSECONDS, ahrs);
    tasks.ahrsTask = &ahrsTask;
    static MotorPairControllerTask mpcTask(MPC_TASK_INTERVAL_MICROSECONDS, motorPairController);
    tasks.mpcTask = &mpcTask;
    static ReceiverTask receiverTask(RECEIVER_TASK_INTERVAL_MICROSECONDS, receiver, receiverWatcher);
    tasks.receiverTask = &receiverTask;

#if defined(USE_FREERTOS)
    enum { AHRS_TASK_PRIORITY = 5, MPC_TASK_PRIORITY = 4, RECEIVER_TASK_PRIORITY = 3, MSP_TASK_PRIORITY = 2 };

enum { MPC_TASK_CORE = PRO_CPU_NUM };
enum { RECEIVER_TASK_CORE = PRO_CPU_NUM };
#if defined(APP_CPU_NUM) // The processor has two cores
    enum { AHRS_TASK_CORE = APP_CPU_NUM }; // AHRS should be the only task running on the second core
#else // single core processor
    enum { AHRS_TASK_CORE = PRO_CPU_NUM };
#endif

#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
#endif

#if defined(USE_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    sprintf(&buf[0], "\r\n\r\n**** Main loop task, name:'%s' priority:%d, tickRate:%dHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
    Serial.print(&buf[0]);
#endif

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t ahrsTaskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &ahrsTask,
    };
    enum { AHRS_TASK_STACK_DEPTH = 4096 };
    static StaticTask_t ahrsTaskBuffer;
    static std::array <StackType_t, AHRS_TASK_STACK_DEPTH> ahrsStack;
    const TaskHandle_t ahrsTaskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        "AHRS_Task",
        AHRS_TASK_STACK_DEPTH,
        &ahrsTaskParameters,
        AHRS_TASK_PRIORITY,
        &ahrsStack[0],
        &ahrsTaskBuffer,
        AHRS_TASK_CORE
    );
    assert(ahrsTaskHandle != nullptr && "Unable to create AHRS task.");
#if !defined(FRAMEWORK_ESPIDF)
    sprintf(&buf[0], "**** AHRS_Task,      core:%d, priority:%d, task interval:%dms\r\n", AHRS_TASK_CORE, AHRS_TASK_PRIORITY, AHRS_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif

    static TaskBase::parameters_t mpcTaskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &mpcTask,
    };
    enum { MPC_TASK_STACK_DEPTH = 4096 };
    static StaticTask_t mpcTaskBuffer;
    static std::array<StackType_t, MPC_TASK_STACK_DEPTH> mpcStack;
    const TaskHandle_t mpcTaskHandle = xTaskCreateStaticPinnedToCore(MotorPairControllerTask::Task,
        "MPC_Task",
        MPC_TASK_STACK_DEPTH,
        &mpcTaskParameters,
        MPC_TASK_PRIORITY,
        &mpcStack[0],
        &mpcTaskBuffer,
        MPC_TASK_CORE
    );
    assert(mpcTaskHandle != nullptr && "Unable to create MotorPairController task.");
#if !defined(FRAMEWORK_ESPIDF)
    sprintf(&buf[0], "**** MPC_Task,       core:%d, priority:%d, task interval:%dms\r\n", MPC_TASK_CORE, MPC_TASK_PRIORITY, MPC_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t receiverTaskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &receiverTask,
    };
    enum { RECEIVER_TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, RECEIVER_TASK_STACK_DEPTH> receiverStack;
    static StaticTask_t receiverTaskBuffer;
    const TaskHandle_t receiverTaskHandle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::Task,
        "Receiver_Task",
        RECEIVER_TASK_STACK_DEPTH,
        &receiverTaskParameters,
        RECEIVER_TASK_PRIORITY,
        &receiverStack[0],
        &receiverTaskBuffer,
        RECEIVER_TASK_CORE
    );
    assert(receiverTaskHandle != nullptr && "Unable to create ReceiverTask task.");
    sprintf(&buf[0], "**** RECEIVER_Task,  core:%d, priority:%d, task interval:%dms\r\n\r\n", RECEIVER_TASK_CORE, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);

#endif // USE_FREERTOS
}
