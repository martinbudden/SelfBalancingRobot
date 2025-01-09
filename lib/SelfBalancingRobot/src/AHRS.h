#pragma once

#include "AHRS_Base.h"
#include "IMU_Base.h"
#include <cfloat>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

class IMU_Filter;


class AHRS final : public AHRS_Base {
public:
    AHRS(SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor);
private:
    // class is not copyable or moveable
    AHRS(const AHRS&) = delete;
    AHRS& operator=(const AHRS&) = delete;
    AHRS(AHRS&&) = delete;
    AHRS& operator=(AHRS&&) = delete;
public:
    inline IMU_Filter& getIMU_Filter() { return *_imuFilter; }
    virtual void setGyroOffset(const xyz_int16_t& gyroOffset) override;
    virtual void setAccOffset(const xyz_int16_t& accOffset) override;
    virtual xyz_int16_t readGyroRaw() const override;
    virtual xyz_int16_t readAccRaw() const override;
    virtual AHRS_Base::data_t getAhrsDataUsingLock() const override;
    virtual Quaternion getOrientationUsingLock() const override;
    void checkMadgwickConvergence(const xyz_t& acc, const Quaternion& orientation);
public:
    struct TaskParameters {
        AHRS* ahrs;
        uint32_t tickIntervalMilliSeconds;
    };
    static void Task(void* arg);
    void loop(float deltaT);
private:
    void Task(const TaskParameters* taskParameters);
private:
    xyz_t _acc {0.0, 0.0, 0.0};
    xyz_t _gyroRadians {0.0, 0.0, 0.0};

    IMU_Base& _IMU;
    IMU_Filter* _imuFilter;

#if defined(USE_AHRS_DATA_MUTEX)
    StaticSemaphore_t _imuDataMutexBuffer {};
    mutable SemaphoreHandle_t _imuDataMutex {}; // _imuDataMutexBuffer must be declared before _imuDataMutex
    inline void LOCK() const { xSemaphoreTake(_imuDataMutex, portMAX_DELAY); }
    inline void UNLOCK() const { xSemaphoreGive(_imuDataMutex); }
#elif defined(USE_AHRS_DATA_CRITICAL_SECTION)
    mutable portMUX_TYPE _imuDataSpinlock = portMUX_INITIALIZER_UNLOCKED;
    inline void LOCK() const { taskENTER_CRITICAL(&_imuDataSpinlock); }
    inline void UNLOCK() const {taskEXIT_CRITICAL(&_imuDataSpinlock); }
#else
    inline void LOCK() const {}
    inline void UNLOCK() const {}
#endif
};
