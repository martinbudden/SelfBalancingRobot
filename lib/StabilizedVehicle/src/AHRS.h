#pragma once

#include "TaskBase.h"
#include <Quaternion.h>
#include <xyz_int16_type.h>
#include <xyz_type.h>

#include <cassert>
#include <cfloat>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
#if defined(AHRS_RECORD_TIMES_CHECKS)
#include <esp32-hal.h>
#endif

class IMU_Base;
class IMU_FiltersBase;
class MotorControllerBase;
class SensorFusionFilterBase;


class AHRS : public TaskBase {
public:
    struct data_t {
        uint32_t tickCountDelta;
        xyz_t gyroRPS;
        xyz_t acc;
    };
    static constexpr int TIME_CHECKS_COUNT = 4;
public:
    AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
public:
    void setMotorController(MotorControllerBase* motorController) { _motorController = motorController; }
    bool configuredToUpdateOutputs() const { return (_motorController==nullptr) ? false : true; }
private:
    // class is not copyable or moveable
    AHRS(const AHRS&) = delete;
    AHRS& operator=(const AHRS&) = delete;
    AHRS(AHRS&&) = delete;
    AHRS& operator=(AHRS&&) = delete;
public:
    void setGyroOffset(const xyz_int16_t& gyroOffset);
    void setAccOffset(const xyz_int16_t& accOffset);
    xyz_int16_t readGyroRaw() const;
    xyz_int16_t readAccRaw() const;

    data_t getAhrsDataUsingLock(bool& updatedSinceLastRead) const;
    data_t getAhrsDataForInstrumentationUsingLock() const;
    Quaternion getOrientationUsingLock(bool& updatedSinceLastRead) const;
    Quaternion getOrientationForInstrumentationUsingLock() const;

    void checkMadgwickConvergence(const xyz_t& acc, const Quaternion& orientation);
    inline bool sensorFusionFilterIsInitializing() const { return _sensorFusionFilterInitializing; }
    inline void setSensorFusionFilterInitializing(bool sensorFusionFilterInitializing) { _sensorFusionFilterInitializing = sensorFusionFilterInitializing; }

    inline uint32_t getFifoCount() const { return _fifoCount; } // for instrumentation
    inline uint32_t getTimeChecksMicroSeconds(size_t index) const { assert(index < TIME_CHECKS_COUNT); return _timeChecksMicroSeconds[index + 1] - _timeChecksMicroSeconds[index]; } //!< Instrumentation time checks
public:
    struct TaskParameters {
        AHRS* ahrs;
        uint32_t tickIntervalMilliSeconds;
    };
    static void Task(void* arg);
    bool readIMUandUpdateOrientation(float deltaT);
private:
    void Task(const TaskParameters* taskParameters);
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    static IRAM_ATTR void imuDataReadyInterruptServiceRoutine();
#endif
private:
    SensorFusionFilterBase& _sensorFusionFilter;
    IMU_Base& _IMU;
    IMU_FiltersBase& _imuFilters;
    MotorControllerBase* _motorController {nullptr};

    xyz_t _acc {0.0, 0.0, 0.0};
    xyz_t _gyroRPS {0.0, 0.0, 0.0};
    mutable int32_t _ahrsDataUpdatedSinceLastRead {false};

    uint32_t _sensorFusionFilterInitializing {true};
    Quaternion _orientation {};
    mutable int32_t _orientationUpdatedSinceLastRead {false};

    // interrupt service routine data
    static AHRS* ahrs; //!< alias of `this` to be used in imuDataReceivedInterruptServiceRoutine
    uint32_t _imuDataReadyCount {0}; //<! data ready count, used in interrupt service routine

    // instrumentation member data
    uint32_t _fifoCount {0};
    uint32_t _timeCheck0 {0};
    uint32_t _timeChecksMicroSeconds[TIME_CHECKS_COUNT + 1] {};
#if defined(AHRS_RECORD_TIMES_CHECKS)
    inline void TIME_CHECK(size_t index) { _timeChecksMicroSeconds[index] = micros(); }
    inline void TIME_CHECK(size_t index, uint32_t timeMicroSeconds) { _timeChecksMicroSeconds[index] = timeMicroSeconds; }
#else
    inline void TIME_CHECK(size_t index) { (void)index;}
    inline void TIME_CHECK(size_t index, uint32_t timeMicroSeconds) { (void)index; (void)timeMicroSeconds; }
#endif

    // data synchronization primitives
#if defined(USE_IMU_DATA_READY_MUTEX)
    StaticSemaphore_t _imuDataReadyMutexBuffer {}; // _imuDataReadyMutexBuffer must be declared before _imuDataReadyMutex
    SemaphoreHandle_t _imuDataReadyMutex {};
    inline void LOCK_IMU_DATA_READY() const { xSemaphoreTake(_imuDataReadyMutex, portMAX_DELAY); }
    inline void UNLOCK_IMU_DATA_READY() const { xSemaphoreGive(_imuDataReadyMutex); }
#else
    inline void LOCK_IMU_DATA_READY() const {}
    inline void UNLOCK_IMU_DATA_READY() const {}
#endif
#if defined(USE_AHRS_DATA_MUTEX)
    StaticSemaphore_t _ahrsDataMutexBuffer {}; // _ahrsDataMutexBuffer must be declared before _ahrsDataMutex
    mutable SemaphoreHandle_t _ahrsDataMutex {};
    inline void LOCK_AHRS_DATA() const { xSemaphoreTake(_ahrsDataMutex, portMAX_DELAY); }
    inline void UNLOCK_AHRS_DATA() const { xSemaphoreGive(_ahrsDataMutex); }
#elif defined(USE_AHRS_DATA_CRITICAL_SECTION)
    mutable portMUX_TYPE _ahrsDataSpinlock = portMUX_INITIALIZER_UNLOCKED;
    inline void LOCK_AHRS_DATA() const { taskENTER_CRITICAL(&_ahrsDataSpinlock); }
    inline void UNLOCK_AHRS_DATA() const { taskEXIT_CRITICAL(&_ahrsDataSpinlock); }
#else
    inline void LOCK_AHRS_DATA() const {}
    inline void UNLOCK_AHRS_DATA() const {}
#endif
};
