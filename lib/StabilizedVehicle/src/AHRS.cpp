#include "AHRS.h"
#include "IMU_FiltersBase.h"
#include "VehicleControllerBase.h"

#include <SensorFusion.h>
#include <cmath>

#if defined(FRAMEWORK_RPI_PICO)
#include <pico/time.h>
static uint32_t timeUs() { return time_us_32(); }
#elif defined(FRAMEWORK_ESPIDF)
#include <esp_timer.h>
static uint32_t timeUs() { return static_cast<uint32_t>(esp_timer_get_time()); }
#elif defined(FRAMEWORK_TEST)
static uint32_t timeUs() { return 0; }
#else // defaults to FRAMEWORK_ARDUINO
#if defined(USE_ARDUINO_ESP32)
#include <esp32-hal.h>
#endif
#include <Arduino.h>
static uint32_t timeUs() { return micros(); }
#endif // FRAMEWORK


// Either the USE_AHRS_DATA_MUTEX or USE_AHRS_DATA_CRITICAL_SECTION build flag can be set (but not both).
// The critical section variant seems to give better performance.
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_AHRS_DATA_CRITICAL_SECTION)
static_assert(false);
#endif

#if defined(USE_FREERTOS)
inline void YIELD_TASK() { taskYIELD(); }
#else
inline void YIELD_TASK() {}
#endif


/*!
Main AHRS task function. Reads the IMU and uses the sensor fusion filter to update the orientation quaternion.
Returns false if there was no new data to be read from the IMU.

NOTE: calls to YIELD_TASK have no effect on multi-core implementations, but are useful for single-core variants.
*/
bool AHRS::readIMUandUpdateOrientation(float deltaT)
{
    const uint32_t time0 = timeUs();

#if defined(IMU_DOES_SENSOR_FUSION)
    _accGyroRPS.gyroRPS = _IMU.readGyroRPS();
    const uint32_t time1 = timeUs();
    _timeChecksMicroSeconds[0] = time1 - time0;
    _timeChecksMicroSeconds[1] = 0; // filter time set to zero, since filtering is as part of IMU sensor fusion
    const Quaternion orientation = _IMU.readOrientation();
    const uint32_t time3 = timeUs();
    _timeChecksMicroSeconds[2] = time3 - time1;
#else
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    // the data was read in the IMU interrupt service routine, so we can just get the data, rather than read it
    _accGyroRPS = _IMU.getAccGyroRPS();
#else
    _accGyroRPS = _IMU.readAccGyroRPS();
#endif
    const uint32_t time1 = timeUs();
    _timeChecksMicroSeconds[0] = time1 - time0;

    _imuFilters.filter(_accGyroRPS.gyroRPS, _accGyroRPS.acc, deltaT); // 15us, 207us

    const uint32_t time2 = timeUs();
    _timeChecksMicroSeconds[1] = time2 - time1;

    const Quaternion orientation = _sensorFusionFilter.update(_accGyroRPS.gyroRPS, _accGyroRPS.acc, deltaT); // 15us, 140us

    const uint32_t time3 = timeUs();
    _timeChecksMicroSeconds[2] = time3 - time2;

    if (sensorFusionFilterIsInitializing()) {
        checkFusionFilterConvergence(_accGyroRPS.acc, orientation);
    }
#endif

    if (_vehicleController != nullptr) {
        // If _vehicleController is not nullptr, then things have been configured so that updateOutputsUsingPIDs
        // is called by the AHRS (ie here) rather than the motor controller.
        _vehicleController->updateOutputsUsingPIDs(_accGyroRPS.gyroRPS, _accGyroRPS.acc, orientation, deltaT); //25us, 900us
    }

    const uint32_t time4 = timeUs();
    _timeChecksMicroSeconds[3] = time4 - time3;

    // If _vehicleController != nullptr then the locked data is only used for instrumentation (screen display and telemetry),
    // so it might be possible not to use the lock in this case.
    LOCK_AHRS_DATA();
    _ahrsDataUpdatedSinceLastRead = true;
    _orientationUpdatedSinceLastRead = true;
    _orientation = orientation;
    _accGyroRPS_Locked = _accGyroRPS;
    UNLOCK_AHRS_DATA();

    return true;
}

/*!
loop() function for when not using FREERTOS
*/
void AHRS::loop()
{
    const uint32_t timeMicroSeconds = timeUs();
    _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
    _timeMicroSecondsPrevious = timeMicroSeconds;

    if (_timeMicroSecondsDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
        const float deltaT = static_cast<float>(_timeMicroSecondsDelta) * 0.000001F;
        readIMUandUpdateOrientation(deltaT);
    }
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AHRS::Task([[maybe_unused]] const TaskParameters* taskParameters)
{
    _taskIntervalMicroSeconds = taskParameters->taskIntervalMicroSeconds;
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
#if !defined(AHRS_IS_INTERRUPT_DRIVEN)
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(taskParameters->taskIntervalMicroSeconds / 1000);
    assert(taskIntervalTicks > 0 && "AHRS taskIntervalTicks is zero.");
    Serial.print("AHRS us:");
    Serial.println(taskIntervalTicks);
#endif
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
        _IMU.WAIT_IMU_DATA_READY(); // wait until there is IMU data.

        const uint32_t timeMicroSeconds = timeUs();
        const uint32_t timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;
        if (timeMicroSecondsDelta > 0) {
            readIMUandUpdateOrientation(static_cast<float>(timeMicroSecondsDelta)* 0.000001F);
        }
#else
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            const float deltaT = pdTICKS_TO_MS(_tickCountDelta) * 0.001F;
            readIMUandUpdateOrientation(deltaT);
        }
#endif // AHRS_IS_INTERRUPT_DRIVEN
    }
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void AHRS::Task(void* arg)
{
    const AHRS::TaskParameters* taskParameters = static_cast<AHRS::TaskParameters*>(arg);

    taskParameters->ahrs->Task(taskParameters);
}

/*!
Read the raw gyro values. Used in calibration.
*/
void AHRS::readGyroRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    const IMU_Base::xyz_int32_t gyro = _IMU.readGyroRaw();
    x = gyro.x;
    y = gyro.y;
    z = gyro.z;
}

/*!
Read the raw accelerometer values. Used in calibration.
*/
void AHRS::readAccRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    const IMU_Base::xyz_int32_t acc = _IMU.readAccRaw();
    x = acc.x;
    y = acc.y;
    z = acc.z;
}

int32_t AHRS::getAccOneG_Raw() const
{
    return _IMU.getAccOneG_Raw();
}


IMU_Base::xyz_int32_t AHRS::getGyroOffset() const
{
    return _IMU.getGyroOffset();
}

/*!
Set the gyro offset. Used in calibration.
*/
void AHRS::setGyroOffset(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setGyroOffset(offset);
}

IMU_Base::xyz_int32_t AHRS::getAccOffset() const
{
    return _IMU.getAccOffset();
}

/*!
Set the accelerometer offset. Used in calibration.
*/
void AHRS::setAccOffset(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setAccOffset(offset);
}

IMU_Base::xyz_int32_t AHRS::mapOffset(const IMU_Base::xyz_int32_t& offset, IMU_Base::axis_order_e axisOrder)
{
    xyz_t offsetF = { static_cast<float>(offset.x), static_cast<float>(offset.y), static_cast<float>(offset.z) };
    offsetF = IMU_Base::mapAxes(offsetF, axisOrder);
    const IMU_Base::xyz_int32_t offsetMapped = { static_cast<int32_t>(offsetF.x), static_cast<int32_t>(offsetF.y), static_cast<int32_t>(offsetF.z) };
    return offsetMapped;
}

IMU_Base::xyz_int32_t AHRS::getGyroOffsetMapped() const
{
    return mapOffset(_IMU.getGyroOffset(), _IMU.getAxisOrder());
}

void AHRS::setGyroOffsetMapped(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setGyroOffset(mapOffset(offset, IMU_Base::axisOrderInverse(_IMU.getAxisOrder())));
}

IMU_Base::xyz_int32_t AHRS::getAccOffsetMapped() const
{
    return mapOffset(_IMU.getAccOffset(), _IMU.getAxisOrder());
}

void AHRS::setAccOffsetMapped(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setAccOffset(mapOffset(offset, IMU_Base::axisOrderInverse(_IMU.getAxisOrder())));
}

void AHRS::setFilters(const filters_t& filters)
{
    _filters = filters;
}

Quaternion AHRS::getOrientationUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _orientationUpdatedSinceLastRead;
    _orientationUpdatedSinceLastRead = false;
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

/*!
Returns orientation without clearing the _orientationUpdatedSinceLastRead flag.
*/
Quaternion AHRS::getOrientationForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS::data_t AHRS::getAhrsDataUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _ahrsDataUpdatedSinceLastRead;
    _ahrsDataUpdatedSinceLastRead = false;
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _accGyroRPS_Locked.gyroRPS,
        .acc = _accGyroRPS_Locked.acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

/*!
Returns AHRS data without clearing the _ahrsDataUpdatedSinceLastRead flag.
*/
AHRS::data_t AHRS::getAhrsDataForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _accGyroRPS.gyroRPS,
        .acc = _accGyroRPS.acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

void AHRS::checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation)
{
#if defined(USE_MADGWICK_FILTER)
    constexpr float twoDegreesInRadians = 2.0F * Quaternion::degreesToRadians;

    // NOTE COORDINATE TRANSFORM: Madgwick filter uses Euler angles where roll is defined as rotation around the x-axis and pitch is rotation around the y-axis.
    // For the Self Balancing Robot, pitch is rotation around the x-axis and roll is rotation around the y-axis,
    // so SBR.roll = Madgwick.pitch and SPR.pitch = Madgwick.roll
    const float madgwickRollAngleRadians = orientation.calculateRollRadians();
    const float accPitchAngleRadians = std::atan2(acc.y, acc.z);
    //const float accRollAngleRadians = std::atan2(-acc.x, sqrtf(acc.y*acc.y + acc.z*acc.z));

    //Serial.printf("acc:P%5.1f mag:P%5.1f         diff:%5.1f\r\n", accPitchAngleRadians/Quaternion::degreesToRadians, madgwickRollAngleRadians/Quaternion::degreesToRadians, fabsf(accPitchAngleRadians - madgwickRollAngleRadians)/Quaternion::degreesToRadians);
    if (fabsf(accPitchAngleRadians - madgwickRollAngleRadians) < twoDegreesInRadians && accPitchAngleRadians != madgwickRollAngleRadians) {
        // the angles have converged to within 2 degrees, so we can reduce the gain.
        setSensorFusionFilterInitializing(false);
        _sensorFusionFilter.setFreeParameters(0.615F, 0.0F); // corresponds to gyro measurement error of 15*2.7 degrees/second, as discussed by Madgwick
    }
#else
    (void)acc;
    (void)orientation;
    setSensorFusionFilterInitializing(false);
#endif
}

/*!
Constructor: sets the sensor fusion filter, IMU, and IMU filters
*/
AHRS::AHRS(uint32_t taskIntervalMicroSeconds, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters) :
    TaskBase(taskIntervalMicroSeconds),
    _sensorFusionFilter(sensorFusionFilter),
    _IMU(imuSensor),
    _imuFilters(imuFilters)
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_FREERTOS)
    , _ahrsDataMutex(xSemaphoreCreateRecursiveMutexStatic(&_ahrsDataMutexBuffer)) // statically allocate the imuDataMutex
#endif

{
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    _IMU.setInterruptDriven();
#endif

    setSensorFusionFilterInitializing(true);

#if defined(USE_FREERTOS)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#if defined(USE_AHRS_DATA_MUTEX)
    // ensure _ahrsDataMutexBuffer declared before _ahrsDataMutex
    static_assert(offsetof(AHRS, _ahrsDataMutex) > offsetof(AHRS, _ahrsDataMutexBuffer));
#endif
#pragma GCC diagnostic pop

#elif defined(FRAMEWORK_RPI_PICO)

#if defined(USE_AHRS_DATA_MUTEX)
    mutex_init(&_ahrsDataMutex);
#elif defined(USE_AHRS_DATA_CRITICAL_SECTION)
    critical_section_init(&_ahrsDataCriticalSection);
#endif

#endif
}
