#if defined(USE_AHRS)

#include "AHRS.h"
#include "IMU_Base.h"
#include "IMU_Filters.h"
#include "MotorPairController.h"
#include "SensorFusionFilter.h"
#include <cmath>
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
#include <driver/gpio.h>
#include <esp32-hal-gpio.h>
#endif
#include <esp32-hal.h>

// Either the USE_AHRS_DATA_MUTEX or USE_AHRS_DATA_CRITICAL_SECTION build flag can be set (but not both).
// The critical section variant seems to give better performance.
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_AHRS_DATA_CRITICAL_SECTION)
static_assert(false);
#endif


AHRS* AHRS::ahrs {nullptr};


/*!
Main AHRS task function. Reads the IMU and uses the sensor fusion filter to update the orientation quaternion.
Returns false if there was no new data to be read from the IMU.

NOTE: calls to YIELD_TASK have no effect on multi-core implementations, but are useful for single-core variants.
*/
bool AHRS::readIMUandUpdateOrientation(float deltaT)
{
    xyz_t gyroRadians; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    xyz_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)

#if defined(USE_IMU_FIFO)
    // It uses the IMU FIFO. This ensures all IMU readings are processed, but it has the disadvantage that
    // reading the FIFO blocks the I2C bus, which in turn blocks the MPC_TASK.
    // I'm starting to come to the conclusion that, for M5Stack devices, better overall performance is obtained by not using the FIFO.

    _fifoCount = _IMU.readFIFO_ToBuffer();
    if (_fifoCount == 0) {
        YIELD_TASK();
        return false;
    }

    xyz_t gyroRadiansSum {0.0, 0.0, 0.0};
    xyz_t accSum {0.0, 0.0, 0.0};
    const float fifoCountReciprocal = 1.0F / static_cast<float>(_fifoCount);
    const float fifoDeltaT = deltaT * fifoCountReciprocal;
    // constexpr float dT {1.0 / 500.0}; // use fixed deltaT corresponding to the update rate of the FIFO
    for (auto ii = 0; ii < _fifoCount; ++ii) {
        _IMU.readFIFO_Item(gyroRadians, acc, ii);
        _imuFilters->filter(gyroRadians, acc, fifoDeltaT);
        gyroRadiansSum += gyroRadians;
        accSum += acc;
    }
    gyroRadians = gyroRadiansSum * fifoCountReciprocal;
    acc = accSum * fifoCountReciprocal;
    const Quaternion orientation = _sensorFusionFilter.update(gyroRadians, acc, deltaT);
#else
#if defined(AHRS_RECORD_UPDATE_TIMES)
    const uint32_t timeMicroSeconds0 = micros();
#endif
    const bool dataRead = _IMU.readGyroRadiansAcc(gyroRadians, acc);
    if (dataRead == false) {
        YIELD_TASK();
        return false;
    }
#if defined(AHRS_RECORD_UPDATE_TIMES)
    const uint32_t timeMicroSeconds1 = micros();
    _updateTimeIMU_ReadMicroSeconds = timeMicroSeconds1 - timeMicroSeconds0;
#endif
    _imuFilters->filter(gyroRadians, acc, deltaT);
#if defined(AHRS_RECORD_UPDATE_TIMES)
    const uint32_t timeMicroSeconds2 = micros();
    _updateTimeFiltersMicroSeconds = timeMicroSeconds2 - timeMicroSeconds1;
#endif
    const Quaternion orientation = _sensorFusionFilter.update(gyroRadians, acc, deltaT);
#if defined(AHRS_RECORD_UPDATE_TIMES)
    const uint32_t timeMicroSeconds3 = micros();
    _updateTimeSensorFusionMicroSeconds = timeMicroSeconds3 - timeMicroSeconds2;
#endif
#endif // USE_IMU_FIFO
    if (_motorController != nullptr) {
        _motorController->updatePIDs(orientation, deltaT);
#if defined(AHRS_RECORD_UPDATE_TIMES)
        const uint32_t timeMicroSeconds4 = micros();
        _updateTimePID_MicroSeconds = timeMicroSeconds4 - timeMicroSeconds3;
#endif
    }
    if (sensorFusionFilterIsInitializing()) {
        checkMadgwickConvergence(acc, orientation);
    }

    LOCK_AHRS_DATA();
    _ahrsDataUpdatedSinceLastRead = true;
    _orientationUpdatedSinceLastRead = true;
    _orientation = orientation;
    _gyroRadians = gyroRadians;
    _acc = acc;
    UNLOCK_AHRS_DATA();
    return true;
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
void AHRS::Task(const TaskParameters* taskParameters)
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    _tickIntervalTicks = pdMS_TO_TICKS(taskParameters->tickIntervalMilliSeconds);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
        LOCK_IMU_DATA_READY();
        _imuDataReadyCount = 0;

        _timeMicroSecondsPrevious = _timeMicroSeconds;
        _timeMicroSeconds = micros();
        const float deltaT = static_cast<float>(_timeMicroSeconds - _timeMicroSecondsPrevious) * 0.000001F;
        if (deltaT > 0.0F) {
            (void)readIMUandUpdateOrientation(deltaT);
        }
#else
        // delay until the end of the next tickIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, _tickIntervalTicks);
        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than _tickIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSecond = micros();
        _timeMicroSecondDelta = timeMicroSecond - _timeMicroSecondPrevious;
        _timeMicroSecondPrevious = timeMicroSecond;

        if (_tickCountDelta > 0) { // guard against the case of the while loop executing twice on the same tick interval
            const float deltaT = pdTICKS_TO_MS(_tickCountDelta) * 0.001F;
            (void)readIMUandUpdateOrientation(deltaT);
        }
#endif
    }
#endif
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
void AHRS::Task(void* arg)
{
    const AHRS::TaskParameters* taskParameters = static_cast<AHRS::TaskParameters*>(arg);

    AHRS* ahrs = taskParameters->ahrs;
    ahrs->Task(taskParameters);
}

/*!
Read the raw gyro values. Used in calibration.
*/
xyz_int16_t AHRS::readGyroRaw() const
{
    return _IMU.readGyroRaw();
}

/*!
Read the raw accelerometer values. Used in calibration.
*/
xyz_int16_t AHRS::readAccRaw() const
{
    return _IMU.readAccRaw();
}

/*!
Set the gyro offset. Used in calibration.
*/
void AHRS::setGyroOffset(const xyz_int16_t& gyroOffset)
{
    _IMU.setGyroOffset(gyroOffset);
}

/*!
Set the accelerometer offset. Used in calibration.
*/
void AHRS::setAccOffset(const xyz_int16_t& accOffset)
{
    _IMU.setAccOffset(accOffset);
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
Returns orientation without clearing _orientationUpdatedSinceLastRead.
*/
Quaternion AHRS::getOrientationForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS_Base::data_t AHRS::getAhrsDataUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _ahrsDataUpdatedSinceLastRead;
    _ahrsDataUpdatedSinceLastRead = false;
    const AHRS_Base::data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRadians = _gyroRadians,
        .acc = _acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS_Base::data_t AHRS::getAhrsDataForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const AHRS_Base::data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRadians = _gyroRadians,
        .acc = _acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

void AHRS::checkMadgwickConvergence(const xyz_t& acc, const Quaternion& orientation)
{
    constexpr float degreesToRadians {M_PI / 180.0};
    constexpr float twoDegreesInRadians = 2.0F * degreesToRadians;

    const float madgwickPitchAngleRadians = orientation.calculatePitchRadians();
    const float accPitchAngleRadians = std::atan2(acc.y, acc.z);
    if (fabs(accPitchAngleRadians - madgwickPitchAngleRadians) < twoDegreesInRadians && accPitchAngleRadians != madgwickPitchAngleRadians) {
        // the angles have converged to within 2 degrees, so set we can reduce the gain.
        setSensorFusionFilterInitializing(false);
        _sensorFusionFilter.setFreeParameters(0.1F, 0.0F);
    }
}

#if defined(AHRS_IS_INTERRUPT_DRIVEN)
IRAM_ATTR void AHRS::imuDataReadyInterruptServiceRoutine()
{
    //AHRS* ahrs = reinterpret_cast<AHRS*>(arg);
    ++ahrs->_imuDataReadyCount;
    ahrs->UNLOCK_IMU_DATA_READY();
}
#endif

/*!
Constructor: set the sensor fusion filter and IMU to be used by the AHRS.
*/
AHRS::AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, uint32_t tickIntervalMicroSeconds) :
    AHRS_Base(sensorFusionFilter),
    _IMU(imuSensor)
#if defined(USE_IMU_DATA_READY_MUTEX)
    ,_imuDataReadyMutex(xSemaphoreCreateRecursiveMutexStatic(&_imuDataReadyMutexBuffer)) // statically allocate the imuDataMutex
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    , _ahrsDataMutex(xSemaphoreCreateRecursiveMutexStatic(&_ahrsDataMutexBuffer)) // statically allocate the imuDataMutex
#endif
{
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    ahrs = this;
    attachInterrupt(IMU_INTERRUPT_PIN, imuDataReadyInterruptServiceRoutine, FALLING);
#endif

    setSensorFusionFilterInitializing(true);
    // statically allocate the IMU_Filters
    constexpr float cutoffFrequency = 100.0F;
    static IMU_Filters imuFilters(cutoffFrequency, static_cast<float>(tickIntervalMicroSeconds) * 1.0e-6F);
    _imuFilters = &imuFilters;


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#if defined(USE_IMU_DATA_READY_MUTEX)
     // ensure _imuDataReadyMutexBuffer declared before _imuDataReadyMutex
    static_assert(offsetof(AHRS, _imuDataReadyMutex) > offsetof(AHRS, _imuDataReadyMutexBuffer));
#endif
#if defined(USE_AHRS_DATA_MUTEX)
     // ensure _ahrsDataMutexBuffer declared before _ahrsDataMutex
    static_assert(offsetof(AHRS, _ahrsDataMutex) > offsetof(AHRS, _ahrsDataMutexBuffer));
#endif
#pragma GCC diagnostic pop
}

#endif // USE_AHRS
