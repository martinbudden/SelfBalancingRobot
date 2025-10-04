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

    struct xyz_int32_t {
        int32_t x;
        int32_t y;
        int32_t z;
    };
public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    NonVolatileStorage();
    explicit NonVolatileStorage(uint32_t flashMemorySize);
    void init();
    static void toHexChars(char* charPtr, uint16_t value);

    uint8_t getCurrentPidProfileIndex() const { return _currentPidProfileIndex; }
    void setCurrentPidProfileIndex(uint8_t currentPidProfileIndex) { _currentPidProfileIndex = currentPidProfileIndex; }

    int32_t clear();
    int32_t remove(uint16_t key);

    bool loadAccOffset(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t storeAccOffset(int32_t x, int32_t y, int32_t z);

    bool loadGyroOffset(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t storeGyroOffset(int32_t x, int32_t y, int32_t z);

    void loadMacAddress(uint8_t* macAddress) const;
    int32_t storeMacAddress(const uint8_t* macAddress);

    uint8_t loadPidProfileIndex() const;
    int32_t storePidProfileIndex(uint8_t pidProfileIndex);

    float loadBalanceAngle() const;
    int32_t storeBalanceAngle(float balanceAngle);

    VehicleControllerBase::PIDF_uint16_t loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const;
    VehicleControllerBase::PIDF_uint16_t loadPID(uint8_t pidIndex) const { return loadPID(pidIndex, _currentPidProfileIndex); }
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex);
    int32_t storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex) { return storePID(pid, pidIndex, _currentPidProfileIndex); }
    void resetPID(uint8_t pidIndex, uint8_t pidProfileIndex);
    void resetPID(uint8_t pidIndex) { resetPID(pidIndex, _currentPidProfileIndex); }

    bool loadItem(uint16_t key, void* item, size_t length) const;
    bool loadItem(uint16_t key, uint8_t pidProfileIndex, void* item, size_t length) const;
    int32_t storeItem(uint16_t key, const void* item, size_t length, const void* defaults);
    int32_t storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults);

    RadioController::failsafe_t loadRadioControllerFailsafe();
    int32_t storeRadioControllerFailsafe(const RadioController::failsafe_t& failsafe);

private:
#if defined(USE_FLASH_KLV)
    FlashKLV _flashKLV;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    mutable Preferences _preferences {};
#endif
    uint8_t _currentPidProfileIndex {0};
};
