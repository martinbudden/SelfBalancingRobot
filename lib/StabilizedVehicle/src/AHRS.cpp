#include "AHRS.h"
#include "IMU_FiltersBase.h"
#include "VehicleControllerBase.h"

#include <SensorFusion.h>
#include <cmath>

#if defined(USE_FREERTOS) && defined(FRAMEWORK_ARDUINO)
#include <esp32-hal.h>
static uint32_t timeUs() { return micros(); }
#elif defined(USE_FREERTOS) && defined(FRAMEWORK_ESPIDF)
#include <esp_timer.h>
static uint32_t timeUs() { return static_cast<uint32_t>(esp_timer_get_time()); }
#else
static uint32_t timeUs() { return 0; }
#endif


// Either the USE_AHRS_DATA_MUTEX or USE_AHRS_DATA_CRITICAL_SECTION build flag can be set (but not both).
// The critical section variant seems to give better performance.
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_AHRS_DATA_CRITICAL_SECTION)
static_assert(false);
#endif


#if defined(AHRS_IS_INTERRUPT_DRIVEN)
AHRS* AHRS::ahrs {nullptr};
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
    const IMU_Base::gyroRPS_Acc_t gyroAcc = {
        .gyroRPS = _IMU.readGyroRPS(),
        .acc = {}
    };
    const uint32_t time1 = timeUs();
    _timeChecksMicroSeconds[0] = time1 - time0;
    _timeChecksMicroSeconds[1] = 0; // filter time set to zero, since filtering is as part of IMU sensor fusion
    const Quaternion orientation = _IMU.readOrientation();
    const uint32_t time3 = timeUs();
    _timeChecksMicroSeconds[2] = time3 - time1;
#else
    IMU_Base::gyroRPS_Acc_t gyroAcc = _IMU.readGyroRPS_Acc(); // NOLINT(misc-const-correctness) false positive
    const uint32_t time1 = timeUs();
    _timeChecksMicroSeconds[0] = time1 - time0;

    _imuFilters.filter(gyroAcc.gyroRPS, gyroAcc.acc, deltaT); // 15us, 207us

    const uint32_t time2 = timeUs();
    _timeChecksMicroSeconds[1] = time2 - time1;

    const Quaternion orientation = _sensorFusionFilter.update(gyroAcc.gyroRPS, gyroAcc.acc, deltaT); // 15us, 140us

    const uint32_t time3 = timeUs();
    _timeChecksMicroSeconds[2] = time3 - time2;

    if (sensorFusionFilterIsInitializing()) {
        checkFusionFilterConvergence(gyroAcc.acc, orientation);
    }
#endif

    // If _vehicleController is not nullptr, then things have been configured so that updateOutputsUsingPIDs
    // is called by the AHRS rather than the motor controller.
    if (_vehicleController != nullptr) {
        _vehicleController->updateOutputsUsingPIDs(gyroAcc.gyroRPS, gyroAcc.acc, orientation, deltaT); //25us, 900us
    }
    const uint32_t time4 = timeUs();
    _timeChecksMicroSeconds[3] = time4 - time3;

    LOCK_AHRS_DATA();
    _ahrsDataUpdatedSinceLastRead = true;
    _orientationUpdatedSinceLastRead = true;
    _orientation = orientation;
    _gyroRPS = gyroAcc.gyroRPS;
    _acc = gyroAcc.acc;
    UNLOCK_AHRS_DATA();

    return true;
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AHRS::Task([[maybe_unused]] const TaskParameters* taskParameters)
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    _taskIntervalTicks = pdMS_TO_TICKS(taskParameters->taskIntervalMilliSeconds);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
        LOCK_IMU_DATA_READY(); // wait until the ISR unlocks data ready
        _imuDataReadyCount = 0;

        const uint32_t timeMicroSeconds = timeUs();
        const float deltaT = static_cast<float>(timeMicroSeconds - _timeMicroSecondsPrevious) * 0.000001F;
        _timeMicroSecondsPrevious = timeMicroSeconds;
        if (deltaT > 0.0F) {
            readIMUandUpdateOrientation(deltaT);
        }
#else
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, _taskIntervalTicks);
        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than _taskIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of the while loop executing twice on the same tick interval
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

IMU_Base::xyz_int32_t AHRS::mapOffset(const IMU_Base::xyz_int32_t& offset, IMU_Base::axis_order_t axisOrder)
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
        .gyroRPS = _gyroRPS,
        .acc = _acc
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
        .gyroRPS = _gyroRPS,
        .acc = _acc
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

#if defined(AHRS_IS_INTERRUPT_DRIVEN)
/*!
IMU data ready interrupt service routine (ISR)
*/
IRAM_ATTR void AHRS::imuDataReadyISR()
{
    ++ahrs->_imuDataReadyCount;
    ahrs->UNLOCK_IMU_DATA_READY();
}
#endif

/*!
Constructor: sets the sensor fusion filter, IMU, and IMU filters
*/
AHRS::AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters) :
    _sensorFusionFilter(sensorFusionFilter),
    _IMU(imuSensor),
    _imuFilters(imuFilters)
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    , _imuDataReadyMutex(xSemaphoreCreateRecursiveMutexStatic(&_imuDataReadyMutexBuffer)) // statically allocate the imuDataMutex
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    , _ahrsDataMutex(xSemaphoreCreateRecursiveMutexStatic(&_ahrsDataMutexBuffer)) // statically allocate the imuDataMutex
#endif

{
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    ahrs = this;
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), imuDataReadyISR, LOW);
#endif
    setSensorFusionFilterInitializing(true);

#if defined(USE_FREERTOS)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    // ensure _imuDataReadyMutexBuffer declared before _imuDataReadyMutex
    static_assert(offsetof(AHRS, _imuDataReadyMutex) > offsetof(AHRS, _imuDataReadyMutexBuffer));
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    // ensure _ahrsDataMutexBuffer declared before _ahrsDataMutex
    static_assert(offsetof(AHRS, _ahrsDataMutex) > offsetof(AHRS, _ahrsDataMutexBuffer));
#endif
#pragma GCC diagnostic pop

#elif defined(USE_PICO_BARE_METAL)

#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    mutex_init(&_imuDataReadyMutex);
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    mutex_init(&_ahrsDataMutex);
#elif defined(USE_AHRS_DATA_CRITICAL_SECTION)
    critical_section_init(_ahrsDataCriticalSection);
#endif

#endif
}
