#include "Main.h"

#include <AHRS_Task.h>
#include <BackchannelTask.h>

#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#include <ReceiverTask.h>
#include <VehicleControllerTask.h>

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


enum {
    AHRS_TASK_PRIORITY = 6,
    MPC_TASK_PRIORITY = 5,
    RECEIVER_TASK_PRIORITY = MPC_TASK_PRIORITY,
    BACKCHANNEL_TASK_PRIORITY = 3,
    MSP_TASK_PRIORITY = 2
};

#if defined(USE_FREERTOS)
enum {
#if defined(APP_CPU_NUM) // The processor has two cores
    AHRS_TASK_CORE = APP_CPU_NUM, // AHRS should be the only task running on the second core
#else // single core processor
    AHRS_TASK_CORE = PRO_CPU_NUM,
#endif
    MPC_TASK_CORE = PRO_CPU_NUM,
    RECEIVER_TASK_CORE = PRO_CPU_NUM,
    BACKCHANNEL_TASK_CORE = PRO_CPU_NUM,
    MSP_TASK_CORE = PRO_CPU_NUM,
};
#endif // USE_FREERTOS

MainTask* Main::setupMainTask()
{
    static MainTask task(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS);
#if defined(USE_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    std::array<char, 128> buf;
    sprintf(&buf[0], "\r\n\r\n**** Main loop task, name:'%s' priority:%d, tickRate:%dHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
    Serial.print(&buf[0]);
#endif
    return &task;
}

AHRS_Task* Main::setupTask(AHRS& ahrs)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AHRS_Task task(AHRS_TASK_INTERVAL_MICROSECONDS, ahrs);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static StaticTask_t taskBuffer;
    static std::array <StackType_t, TASK_STACK_DEPTH> stack;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        "AHRS_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        AHRS_TASK_PRIORITY,
        &stack[0],
        &taskBuffer,
        AHRS_TASK_CORE
    );
    assert(taskHandle != nullptr && "Unable to create AHRS task.");
#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** AHRS_Task,      core:%d, priority:%d, task interval:%dms\r\n", AHRS_TASK_CORE, AHRS_TASK_PRIORITY, AHRS_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif
#endif // USE_FREERTOS
    return &task;
}

VehicleControllerTask* Main::setupTask(VehicleControllerBase& vehicleController)
{
    static VehicleControllerTask task(MPC_TASK_INTERVAL_MICROSECONDS, vehicleController);
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };

#if defined(USE_FREERTOS)
    enum { TASK_STACK_DEPTH = 4096 };
    static StaticTask_t taskBuffer;
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(VehicleControllerTask::Task,
        "MPC_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        MPC_TASK_PRIORITY,
        &stack[0],
        &taskBuffer,
        MPC_TASK_CORE
    );
    assert(taskHandle != nullptr && "Unable to create MotorPairController task.");
#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** MPC_Task,       core:%d, priority:%d, task interval:%dms\r\n", MPC_TASK_CORE, MPC_TASK_PRIORITY, MPC_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif
#endif // USE_FREERTOS
    return &task;
}

ReceiverTask* Main::setupTask(ReceiverBase& receiver, ReceiverWatcher* receiverWatcher)
{
    static ReceiverTask task(RECEIVER_TASK_INTERVAL_MICROSECONDS, receiver, receiverWatcher);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        ReceiverTask::Task,
        "Receiver_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        RECEIVER_TASK_PRIORITY,
        &stack[0],
        &taskBuffer,
        RECEIVER_TASK_CORE
    );
    assert(taskHandle != nullptr && "Unable to create ReceiverTask task.");
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** RECEIVER_Task,  core:%d, priority:%d, task interval:%dms\r\n\r\n", RECEIVER_TASK_CORE, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_INTERVAL_MICROSECONDS / 1000);
    Serial.print(&buf[0]);
#endif // USE_FREERTOS
    return &task;
}

BackchannelTask* Main::setupTask(BackchannelBase& backchannel)
{
    static BackchannelTask task(backchannel);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &task,
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::Task,
        "Backchannel_Task",
        TASK_STACK_DEPTH,
        &taskParameters,
        BACKCHANNEL_TASK_PRIORITY,
        &stack[0],
        &taskBuffer,
        BACKCHANNEL_TASK_CORE
    );
    assert(taskHandle != nullptr && "Unable to create ReceiverTask task.");
    std::array<char, 128> buf;
    Serial.printf(&buf[0], "**** BACKCHANNEL_Task,core:%d, priority:%d, task is interrupt driven\r\n", BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_PRIORITY);

    Serial.print(&buf[0]);
#endif // USE_FREERTOS
    return &task;
}
