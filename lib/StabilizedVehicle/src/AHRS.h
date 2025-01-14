#pragma once

#include "AHRS_Base.h"
#include "IMU_Base.h"
#include <cfloat>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

class IMU_FiltersBase;


class AHRS : public AHRS_Base {
public:
    AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
private:
    // class is not copyable or moveable
    AHRS(const AHRS&) = delete;
    AHRS& operator=(const AHRS&) = delete;
    AHRS(AHRS&&) = delete;
    AHRS& operator=(AHRS&&) = delete;
public:
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual AHRS_Base::data_t getAhrsDataUsingLock(bool& updatedSinceLastRead) const override;
    virtual AHRS_Base::data_t getAhrsDataForInstrumentationUsingLock() const override;
    virtual Quaternion getOrientationUsingLock(bool& updatedSinceLastRead) const override;
    virtual Quaternion getOrientationForInstrumentationUsingLock() const override;
    void checkMadgwickConvergence(const xyz_t& acc, const Quaternion& orientation);
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
    static AHRS* ahrs; // alias of `this` to be used in imuDataReceivedInterruptServiceRoutine 
    uint32_t _imuDataReadyCount {0};
    uint32_t _timeMicroSeconds {0};
    uint32_t _timeMicroSecondsPrevious {0};

    xyz_t _acc {0.0, 0.0, 0.0};
    xyz_t _gyroRadians {0.0, 0.0, 0.0};

    IMU_Base& _IMU;
    IMU_FiltersBase& _imuFilters;

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
#if defined(USE_FREERTOS)
    inline void YIELD_TASK() const { taskYIELD(); }
#else
    inline void YIELD_TASK() const {}
#endif
};
