#if defined(M5_STACK)
#include <M5Stack.h>
#elif defined(M5_UNIFIED)
#include <M5Unified.h>
#endif

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <WiFi.h>
#endif

#include "ButtonsM5.h"
#include "Main.h"
#include "ScreenM5.h"

#include <AHRS.h>
#include <AHRS_Task.h>

#include <BackchannelSBR.h>
#include <BackchannelTask.h>
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <BackchannelTransceiverESPNOW.h>
#endif

#if defined(USE_BLACKBOX)
#include <Blackbox.h>
#include <BlackboxCallbacks.h>
#include <BlackboxMessageQueueAHRS.h>
#include <BlackboxSelfBalancingRobot.h>
#include <BlackboxSerialDeviceSDCard.h>
#include <BlackboxTask.h>
#endif

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#include <MotorPairController.h>
#include <RadioController.h>

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
#include <ReceiverAtomJoyStick.h>
#endif
#include <ReceiverNull.h>
#include <ReceiverTask.h>
#include <SV_Preferences.h>
#include <TimeMicroSeconds.h>
#include <VehicleControllerTask.h>

#if defined(FRAMEWORK_ESPIDF)
#include <esp_timer.h>
#endif

/*!
Setup for the main loop, motor control task, and AHRS(Attitude and Heading Reference System) task.
*/
void Main::setup()
{
    // Initialize the M5Stack object
#if defined(M5_UNIFIED)
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();
#if defined(MOTORS_BALA_C)
    // with additional battery, we need to increase charge current
    M5.Power.setChargeCurrent(360);
#endif
#elif defined(M5_STACK)
    //M5Stack::begin(bool LCDEnable, bool SDEnable, bool SerialEnable, bool I2CEnable)
    M5.begin(true, false, false, false);
    M5.IMU.Init();
    M5.Power.begin();
#endif

#if defined(FRAMEWORK_RPI_PICO)
#elif defined(FRAMEWORK_ESPIDF)
    esp_timer_init();
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    Serial.begin(115200);
#endif

    // Create a mutex to ensure there is no conflict between objects using the I2C bus, namely the motors and the AHRS.
    // The mutex is created statically, ie without dynamic memory allocation.
    // If the motors and the AHRS are on separate busses (for example the motors were on a CAN bus, or the AHRS was on an SPI bus),
    // then the mutex is not required and may be set to nullptr.
#if defined(I2C_MUTEX_REQUIRED)
    static StaticSemaphore_t i2cMutexBuffer;
    SemaphoreHandle_t i2cMutex = xSemaphoreCreateMutexStatic(&i2cMutexBuffer);
#else
    void* i2cMutex = nullptr;
#endif

    AHRS& ahrs = createAHRS(i2cMutex); // NOLINT(misc-const-correctness) false positive

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
    // Set WiFi to station mode
    WiFi.mode(WIFI_STA);
    // Disconnect from Access Point if it was previously connected
    WiFi.disconnect();
    // get my MAC address
    uint8_t myMacAddress[ESP_NOW_ETH_ALEN];
    WiFi.macAddress(&myMacAddress[0]);

    // Statically allocate and setup the receiver.
#if !defined(RECEIVER_CHANNEL)
    constexpr uint8_t RECEIVER_CHANNEL {3};
#endif
    static ReceiverAtomJoyStick receiver(&myMacAddress[0], RECEIVER_CHANNEL);
    const esp_err_t espErr = receiver.init();
    //delay(400); // delay to allow serial port to initialize before first print
    Serial.print("\r\n\r\n**** ESP-NOW Ready:");
    Serial.println(espErr);
    Serial.println();
    assert(espErr == ESP_OK && "Unable to setup receiver.");
#else
    static ReceiverNull receiver;
#endif // LIBRARY_RECEIVER_USE_ESPNOW

    static RadioController radioController(receiver);

    // Statically allocate the motorPairController.
    MotorPairBase& motorPairBase = MotorPairController::allocateMotors();
    static MotorPairController motorPairController(MPC_TASK_DENOMINATOR, ahrs, motorPairBase, radioController, i2cMutex);
    ahrs.setVehicleController(&motorPairController);
    radioController.setMotorPairController(&motorPairController);

#if defined(USE_BLACKBOX)
    static BlackboxMessageQueue         blackboxMessageQueue;
    static BlackboxCallbacks            blackboxCallbacks(blackboxMessageQueue, ahrs, motorPairController, radioController, receiver); // NOLINT(misc-const-correctness) false positive
    static BlackboxSerialDeviceSDCard   blackboxSerialDevice(BlackboxSerialDeviceSDCard::SDCARD_SPI_PINS); // NOLINT(misc-const-correctness) false positive
    static BlackboxSelfBalancingRobot   blackbox(blackboxCallbacks, blackboxMessageQueue, blackboxSerialDevice, motorPairController);
    static BlackboxMessageQueueAHRS     blackboxMessageQueueAHRS(blackboxMessageQueue);
    ahrs.setMessageQueue(&blackboxMessageQueueAHRS);
    motorPairController.setBlackbox(blackbox);
    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        //.device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL, // logging starts on arming, file is saved when disarmed
        //.mode = Blackbox::MODE_ALWAYS_ON
    });
#endif
    static SV_Preferences preferences;

#if defined(M5_STACK) || defined(M5_UNIFIED)
    // Holding BtnA down while switching on enters calibration mode.
    if (M5.BtnA.isPressed()) {
        calibrateIMU(preferences, ahrs);
    }
    checkIMU_Calibration(preferences, ahrs);

    // Holding BtnC down while switching on resets the preferences.
    if (M5.BtnC.isPressed()) {
        resetPreferences(preferences, motorPairController);
    }
    loadPreferences(preferences, motorPairController);

    // Statically allocate the screen.
    static ScreenM5 screen(ahrs, motorPairController, receiver);
    ReceiverWatcher* const receiverWatcher =  &screen;
    _screen = &screen;
    _screen->updateTemplate(); // Update the template as soon as we can, to minimize the time the screen is blank

    // Statically allocate the buttons.
    static ButtonsM5 buttons(motorPairController, receiver, _screen);
    _buttons = &buttons;
#if defined(M5_ATOM)
    // The Atom has no BtnB, so it always broadcasts address for binding on startup.
    receiver.broadcastMyEUI();
#else
    // Holding BtnB down while switching on initiates binding.
    if (M5.BtnB.wasPressed()) {
        receiver.broadcastMyEUI();
    }
#endif

#else

    checkIMU_Calibration(preferences, ahrs);
    loadPreferences(preferences, motorPairController);
    // no buttons defined, so always broadcast address for binding on startup
    receiver.broadcastMyEUI();
    ReceiverWatcher* const receiverWatcher =  nullptr; // no screen available

#endif // M5_STACK || M5_UNIFIED

    // Create the tasks
    static MainTask mainTask(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS); // NOLINT(misc-const-correctness) false positive
    _tasks.mainTask = &mainTask;
    reportMainTask();
    _tasks.ahrsTask = AHRS_Task::createTask(_tasks.ahrsTaskInfo, ahrs, AHRS_TASK_PRIORITY, AHRS_TASK_CORE, AHRS_TASK_INTERVAL_MICROSECONDS);
    _tasks.vehicleControllerTask = VehicleControllerTask::createTask(_tasks.vehicleControllerTaskInfo, motorPairController, MPC_TASK_PRIORITY, MPC_TASK_CORE);
    _tasks.receiverTask = ReceiverTask::createTask(_tasks.receiverTaskInfo, receiver, radioController, receiverWatcher, RECEIVER_TASK_PRIORITY, RECEIVER_TASK_CORE);
#if defined(USE_BLACKBOX)
    _tasks.blackboxTask = BlackboxTask::createTask(blackbox, BLACKBOX_TASK_PRIORITY, BLACKBOX_TASK_CORE, BLACKBOX_TASK_INTERVAL_MICROSECONDS);
#endif

#if defined(BACKCHANNEL_MAC_ADDRESS) && defined(LIBRARY_RECEIVER_USE_ESPNOW)
    // Statically allocate the backchannel.
    constexpr uint8_t backchannelMacAddress[ESP_NOW_ETH_ALEN] BACKCHANNEL_MAC_ADDRESS;
    static BackchannelTransceiverESPNOW backchannelTransceiverESPNOW(receiver.getESPNOW_Transceiver(), &backchannelMacAddress[0]);
    static BackchannelSBR backchannel(
        backchannelTransceiverESPNOW,
        &backchannelMacAddress[0],
        &myMacAddress[0],
        motorPairController,
        ahrs,
        receiver,
        &mainTask,
        preferences
    );
    _tasks.backchannelTask = BackchannelTask::createTask(_tasks.backchannelTaskInfo, backchannel, BACKCHANNEL_TASK_PRIORITY, BACKCHANNEL_TASK_CORE, BACKCHANNEL_TASK_INTERVAL_MICROSECONDS);
#endif
}

void Main::reportMainTask()
{
#if defined(USE_ARDUINO_ESP32)
    // The main task is set up by the framework, so just print its details.
    // It has name "loopTask" and priority 1.
    const TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
    const UBaseType_t taskPriority = uxTaskPriorityGet(taskHandle);
    const char* taskName = pcTaskGetName(taskHandle);
    Serial.printf("\r\n\r\n**** Main loop task, name:'%s' priority:%u, tickRate:%uHz\r\n", taskName, taskPriority, configTICK_RATE_HZ);
#endif
}

#if defined(FRAMEWORK_USE_FREERTOS)
[[noreturn]] void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    assert(false && "stack overflow");
    Serial.printf("\r\n\r\n*********\r\n");
    Serial.printf("********Task '%s' stack overflow ********\r\n", pcTaskName);
    Serial.printf("*********\r\n\r\n");
}
#endif


void Main::checkIMU_Calibration(SV_Preferences& preferences, AHRS& ahrs)
{
    // Set the gyro offsets from non-volatile storage.
#if defined(USE_IMU_M5_UNIFIED)
    // M5_UNIFIED directly uses NVS (non-volatile storage) to store the gyro offsets.
    if (!M5.Imu.loadOffsetFromNVS()) {
        calibrateIMU(preferences, ahrs);
    }
#else
    // For M5_STACK and USE_IMU_MPU6886, the gyro offsets are stored in preferences.
    IMU_Base::xyz_int32_t offset {};
    if (preferences.getGyroOffset(offset.x, offset.y, offset.z)) {
        ahrs.setGyroOffset(offset);
#if defined(FRAMEWORK_ARDUINO)
        std::array<char, 128> buf;
        sprintf(&buf[0], "**** AHRS gyroOffsets loaded from preferences: gx:%5d, gy:%5d, gz:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
        Serial.print(&buf[0]);
#endif
        if (preferences.getAccOffset(offset.x, offset.y, offset.z)) {
            ahrs.setAccOffset(offset);
#if defined(FRAMEWORK_ARDUINO)
            sprintf(&buf[0], "**** AHRS accOffsets loaded from preferences:  ax:%5d, ay:%5d, az:%5d\r\n", static_cast<int>(offset.x), static_cast<int>(offset.y), static_cast<int>(offset.z));
            Serial.print(&buf[0]);
#endif
        }
    } else {
        calibrateIMU(preferences, ahrs);
    }
#endif
}

/*!
Resets the PID preferences and the Balance Angle to SV_Preferences::NOT_SET (which represents unset).
*/
void Main::resetPreferences(SV_Preferences& preferences, MotorPairController& motorPairController)
{
    //_preferences->clear();
    //_preferences->removeGyroOffset();
    //_preferences->removeAccOffset();
    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        const std::string pidName = motorPairController.getPID_Name(static_cast<MotorPairController::pid_index_e>(ii));
        constexpr PIDF::PIDF_t pidNOT_SET { SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET, SV_Preferences::NOT_SET };
        preferences.putPID(pidName, pidNOT_SET);
    }
    preferences.putFloat(motorPairController.getBalanceAngleName(), SV_Preferences::NOT_SET);
#if defined(FRAMEWORK_ARDUINO)
    Serial.print("**** preferences reset");
#endif
}

/*!
Loads the PID settings for the MotorPairController. Must be called *after* the MPC is created.
*/
void Main::loadPreferences(SV_Preferences& preferences, MotorPairController& motorPairController)
{
    // Set all the preferences to zero if they have not been set
    if (!preferences.isSetPID()) {
        resetPreferences(preferences, motorPairController);
    }
    const float pitchBalanceAngleDegrees = preferences.getFloat(motorPairController.getBalanceAngleName());
    if (pitchBalanceAngleDegrees != SV_Preferences::NOT_SET) {
        motorPairController.setPitchBalanceAngleDegrees(pitchBalanceAngleDegrees);
#if defined(FRAMEWORK_ARDUINO)
        Serial.print("**** pitch balance angle loaded from preferences:");
        Serial.print(pitchBalanceAngleDegrees);
#endif
    }

    // Load the PID constants from preferences, and if they are non-zero then use them to set the motorPairController PIDs.
    for (int ii = MotorPairController::PID_BEGIN; ii < MotorPairController::PID_COUNT; ++ii) {
        std::string pidName = motorPairController.getPID_Name(static_cast<MotorPairController::pid_index_e>(ii));
        const PIDF::PIDF_t pid = preferences.getPID(pidName);
        if (pid.kp != SV_Preferences::NOT_SET) {
            motorPairController.setPID_Constants(static_cast<MotorPairController::pid_index_e>(ii), pid);
#if defined(FRAMEWORK_ARDUINO)
            std::array<char, 128> buf;
            sprintf(&buf[0], "**** %s PID loaded from preferences: P:%f, I:%f, D:%f, F:%f\r\n", pidName.c_str(), static_cast<double>(pid.kp), static_cast<double>(pid.ki), static_cast<double>(pid.kd), static_cast<double>(pid.kf));
            Serial.print(&buf[0]);
#endif
        }
    }
}

void MainTask::loop()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;
#endif // USE_FREERTOS
}

/*!
The main loop handles:
1. Input from the buttons
2. Output to the screen

The IMU(Inertial Measurement Unit) is read in the AHRS(Attitude and Heading Reference System) task.
The motors are controlled in the MotorPairController task.
The receiver (joystick) values are obtained in the Receiver task.
*/
void Main::loop() // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_USE_FREERTOS)

    vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_TASK_INTERVAL_MICROSECONDS / 1000));
    [[maybe_unused]] const TickType_t tickCount = xTaskGetTickCount();
    //checkStackUsage();

#else
    [[maybe_unused]] const uint32_t tickCount = timeUs() / 1000;
    // simple round-robbin scheduling
    _tasks.mainTask->loop();
    _tasks.ahrsTask->loop();
    _tasks.vehicleControllerTask->loop();
    _tasks.receiverTask->loop();

#endif

#if defined(USE_SCREEN)
    // screen and button update tick counts are coprime, so screen and buttons are not normally updated in same loop
    // update the screen every 101 ticks (0.1 seconds)
    if (_screenTickCount - tickCount > 101) {
        _screenTickCount = tickCount;
        _screen->update();
    }
#endif
#if defined(USE_BUTTONS)
    // update the buttons every 149 ticks (0.15 seconds)
    if (_buttonsTickCount - tickCount > 149) {
        _buttonsTickCount = tickCount;
        _buttons->update();
    }
#endif
}

void Main::checkStackUsage()
{
#if defined(INCLUDE_uxTaskGetStackHighWaterMark)
    static uint32_t ahrsStackUsedMax = 0;
    const UBaseType_t ahrsStackUsed =  _tasks.ahrsTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.ahrsTaskInfo.taskHandle);
    if (ahrsStackUsed > ahrsStackUsedMax) {
        ahrsStackUsedMax = ahrsStackUsed;
        Serial.printf("AHRS,                stack used:%d\r\n", ahrsStackUsed);
    }

    static uint32_t mpcStackUsedMax = 0;
    const UBaseType_t mpcStackUsed =  _tasks.vehicleControllerTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.vehicleControllerTaskInfo.taskHandle);
    if (mpcStackUsed > mpcStackUsedMax) {
        mpcStackUsedMax = mpcStackUsed;
        Serial.printf("MotorPairController, stack used:%d\r\n", mpcStackUsed);
    }

    static uint32_t receiverStackUsedMax = 0;
    const UBaseType_t receiverStackUsed =  _tasks.receiverTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.receiverTaskInfo.taskHandle);
    if (receiverStackUsed > receiverStackUsedMax) {
        receiverStackUsedMax = receiverStackUsed;
        Serial.printf("Receiver             stack used:%d\r\n", receiverStackUsed);
    }
    static uint32_t backchannelStackUsedMax = 0;
    const UBaseType_t backchannelStackUsed =  _tasks.backchannelTaskInfo.stackDepthBytes -  uxTaskGetStackHighWaterMark(_tasks.backchannelTaskInfo.taskHandle);
    if (backchannelStackUsed > backchannelStackUsedMax) {
        backchannelStackUsedMax = backchannelStackUsed;
        Serial.printf("backchannel,     stack used:%d\r\n", backchannelStackUsed);
    }
#endif
}
