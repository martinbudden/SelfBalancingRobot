#pragma once

#include "Defaults.h"


#if defined(USE_FLASH_KLV)

#include <FlashKLV.h>

#else

// ESP32 and/or ARDUINO_ARCH_ESP32 are defined by Arduino in platform.txt
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#if !defined(USE_ARDUINO_ESP32_PREFERENCES)
#define USE_ARDUINO_ESP32_PREFERENCES
#endif
#endif

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
#include <Preferences.h>
#endif

#endif

#include <cstdint>
#include <string>


/*!
*/
class NonVolatileStorage {
public:
    enum {
        OK = 0, OK_IS_DEFAULT,
        ERROR_FLASH_FULL = -1, ERROR_NOT_FOUND = -2, ERROR_NOT_WRITTEN = -3, ERROR_INVALID_PROFILE
    };

    enum { DEFAULT_PID_PROFILE = 0, PID_PROFILE_COUNT = 1 };
    enum { DEFAULT_RATE_PROFILE = 0, RATE_PROFILE_COUNT = 1 };

    enum { READ_WRITE=false, READ_ONLY=true };
    enum { MAC_ADDRESS_LEN = 6 };

    enum calibration_state_e { NOT_CALIBRATED = 0, CALIBRATED = 1 };
public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    NonVolatileStorage();
    explicit NonVolatileStorage(uint32_t flashMemorySize);
    void init();
    static void toHexChars(char* charPtr, uint16_t value);

    int32_t clear();
    int32_t remove(uint16_t key);

    calibration_state_e loadAccCalibrationState() const;
    int32_t storeAccCalibrationState(calibration_state_e calibrationState);

    xyz_t loadAccOffset() const;
    int32_t storeAccOffset(const xyz_t& offset);

    calibration_state_e loadGyroCalibrationState() const;
    int32_t storeGyroCalibrationState(calibration_state_e calibrationState);

    xyz_t loadGyroOffset() const;
    int32_t storeGyroOffset(const xyz_t& offset);

    void loadMacAddress(uint8_t* macAddress) const;
    int32_t storeMacAddress(const uint8_t* macAddress);

    float loadBalanceAngle() const;
    int32_t storeBalanceAngle(float balanceAngle);

    VehicleControllerBase::PIDF_uint16_t loadPID(uint8_t pidIndex) const;
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex);
    void resetPID(uint8_t pidIndex);

    bool loadItem(uint16_t key, void* item, size_t length) const;
    bool loadItem(uint16_t key, uint8_t pidProfileIndex, void* item, size_t length) const;
    int32_t storeItem(uint16_t key, const void* item, size_t length, const void* defaults);
    int32_t storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults);

    Cockpit::failsafe_config_t loadFailsafeConfig();
    int32_t storeFailsafeConfig(const Cockpit::failsafe_config_t& config);

private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences {};
#endif
};
