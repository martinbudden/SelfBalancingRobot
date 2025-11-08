#include"NonVolatileStorage.h"
#include <AHRS.h>
#include <cstring>


/*
Flash KLV keys:

Keys 0x01-0x3F (1-63): the key and the length are stored as 8-bit values and so there is an
overhead of 2 bytes per record stored.

Keys 0x0100-0x1FFF (256-8191): the key is stored as a 16-bit value and the length is stored as a 8-bit value and so there is an
overhead of 3 bytes per record stored.

Keys 0x2000-0x3FFD (8192-16361): the key and the length are stored as 16-bit values and so there is an
overhead of 4 bytes per record stored.

Other keys are invalid and may not be used. In particular keys of 0, 64-255, and > 16361 are invalid.

This gives a total of 16,169 usable keys.
*/

//static constexpr uint16_t PID_ProfileIndexKey = 0x0001;
//static constexpr uint16_t RateProfileIndexKey = 0x0002;
static constexpr uint16_t AccCalibrationStateKey = 0x0003;
static constexpr uint16_t GyroCalibrationStateKey = 0x0004;
static constexpr uint16_t BalanceAngleKey = 0x0005;

static constexpr uint16_t AccOffsetKey = 0x0007;
static constexpr uint16_t GyroOffsetKey = 0x0008;
static constexpr uint16_t MacAddressKey = 0x0008;

static constexpr uint16_t MotorPairPitchBalanceAngleConfigKey = 0x0009;
static constexpr uint16_t MotorPairControllerFiltersConfigKey = 0x000A;

static constexpr uint16_t FailsafeKey = 0x000B;

static const std::array<uint16_t, MotorPairController::PID_COUNT> PID_Keys = {
    0x0100, 0x0101, 0x0102, 0x0103, 0x0104, 0x0105
};

#if defined(USE_ARDUINO_ESP32_PREFERENCES)
static const char* nonVolatileStorageNamespace {"SBRB"}; // Self Balancing RoBot
#endif


/*!
NOTE: NonVolatileStorage load functions return items by value. c++ uses Return Value Optimization (RVO)
so this is not inefficient.
*/
NonVolatileStorage::NonVolatileStorage(uint32_t flashMemorySize)
#if defined(USE_FLASH_KLV)
    : _flashKLV(flashMemorySize)
#endif
{
    (void)flashMemorySize;
}

NonVolatileStorage::NonVolatileStorage() :
    NonVolatileStorage(4096)
{
}

void NonVolatileStorage::init()
{
#if defined(USE_FLASH_KLV)
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        if (!_preferences.getBool("init")) {
            _preferences.putBool("init", true);
        }
        _preferences.end();
    }
#endif
}

int32_t NonVolatileStorage::clear()
{
#if defined(USE_FLASH_KLV)
    return OK;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        _preferences.clear();
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    return OK;
#endif
}

void NonVolatileStorage::toHexChars(char* charPtr, uint16_t value)
{
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
    *charPtr++ = '0';
    *charPtr++ = 'x';

    auto digit = static_cast<uint8_t>(value >> 12U);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    digit = static_cast<uint8_t>((value >> 8U) & 0x000FU);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    digit = static_cast<uint8_t>((value >> 4U) & 0x000FU);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    digit = static_cast<uint8_t>(value & 0x000FU);
    *charPtr++ = static_cast<char>((digit <= 9) ? digit +'0' : digit + 'A' - 10);
    *charPtr = 0;
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
}

int32_t NonVolatileStorage::remove(uint16_t key)
{
#if defined(USE_FLASH_KLV)
    return _flashKLV.remove(key);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], key);
        _preferences.remove(&keyS[0]);
        _preferences.end();
    }
    return OK;
#else
    (void)key;
    return OK;
#endif
}


bool NonVolatileStorage::loadItem(uint16_t key, void* item, size_t length) const
{
#if defined(USE_FLASH_KLV)
    if (FlashKLV::OK == _flashKLV.read(item, length, key)) {
        return true;
    }
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], key);
        if (_preferences.isKey(&keyS[0])) {
            _preferences.getBytes(&keyS[0], item, length);
            _preferences.end();
            return true;
        }
        _preferences.end();
    }
#else
    (void)key;
    (void)item;
    (void)length;
#endif
    return false;
}

bool NonVolatileStorage::loadItem(uint16_t key, uint8_t pidProfileIndex, void* item, size_t length) const
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return false;
    }
    return loadItem(key + pidProfileIndex, item, length);
}

int32_t NonVolatileStorage::storeItem(uint16_t key, const void* item, size_t length, const void* defaults)
{
#if defined(USE_FLASH_KLV)
    if (!memcmp(defaults, item, length)) {
        // value is the same as default, so no need to store it
        _flashKLV.remove(key);
        return OK_IS_DEFAULT;
    }
    return _flashKLV.write(key, length, item);
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], key);
        if (!memcmp(defaults, item, length)) {
            // value is the same as default, so no need to store it
            _preferences.remove(&keyS[0]);
            _preferences.end();
            return OK_IS_DEFAULT;
        }
        _preferences.putBytes(&keyS[0], item, length);
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)key;
    (void)item;
    (void)length;
    (void)defaults;
    return ERROR_NOT_WRITTEN;
#endif
}

int32_t NonVolatileStorage::storeItem(uint16_t key, uint8_t pidProfileIndex, const void* item, size_t length, const void* defaults)
{
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    return storeItem(key + pidProfileIndex, item, length, defaults);
}


NonVolatileStorage::calibration_state_e NonVolatileStorage::loadAccCalibrationState() const
{
    calibration_state_e calibrationState {};
    if (loadItem(AccCalibrationStateKey, &calibrationState, sizeof(calibrationState))) { // cppcheck-suppress knownConditionTrueFalse
        return calibrationState;
    }
    return NOT_CALIBRATED;
}

int32_t NonVolatileStorage::storeAccCalibrationState(calibration_state_e calibrationState)
{
    const calibration_state_e defaultCalibrationState = NOT_CALIBRATED;
    return storeItem(AccCalibrationStateKey, &calibrationState, sizeof(calibrationState), &defaultCalibrationState);
}

xyz_t NonVolatileStorage::loadAccOffset() const
{
    {xyz_t offset {};
    if (loadItem(AccOffsetKey, &offset, sizeof(offset))) { // cppcheck-suppress knownConditionTrueFalse
        return offset;
    }}
    return xyz_t { .x = 0.0F, .y = 0.0F, .z = 0.0F };
}

int32_t NonVolatileStorage::storeAccOffset(const xyz_t& offset)
{
    const xyz_t defaultOffset = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    return storeItem(AccOffsetKey, &offset, sizeof(offset), &defaultOffset);
}

NonVolatileStorage::calibration_state_e NonVolatileStorage::loadGyroCalibrationState() const
{
    calibration_state_e calibrationState {};
    if (loadItem(GyroCalibrationStateKey, &calibrationState, sizeof(calibrationState))) { // cppcheck-suppress knownConditionTrueFalse
        return calibrationState;
    }
    return NOT_CALIBRATED;
}

int32_t NonVolatileStorage::storeGyroCalibrationState(calibration_state_e calibrationState)
{
    const calibration_state_e defaultCalibrationState = NOT_CALIBRATED;
    return storeItem(GyroCalibrationStateKey, &calibrationState, sizeof(calibrationState), &defaultCalibrationState);
}

xyz_t NonVolatileStorage::loadGyroOffset() const
{
    {xyz_t offset {};
    if (loadItem(GyroOffsetKey, &offset, sizeof(offset))) { // cppcheck-suppress knownConditionTrueFalse
        return offset;
    }}
    return xyz_t { .x = 0.0F, .y = 0.0F, .z = 0.0F };
}

int32_t NonVolatileStorage::storeGyroOffset(const xyz_t& offset)
{
    const xyz_t defaultOffset = { .x = 0.0F, .y = 0.0F, .z = 0.0F };
    return storeItem(GyroOffsetKey, &offset, sizeof(offset), &defaultOffset);
}

float NonVolatileStorage::loadBalanceAngle() const
{
    float balanceAngle {}; // NOLINT(misc-const-correctness)
    if (loadItem(BalanceAngleKey, &balanceAngle, sizeof(balanceAngle))) { // cppcheck-suppress knownConditionTrueFalse
        return balanceAngle;
    }
    return DEFAULTS::balanceAngle;
}

int32_t NonVolatileStorage::storeBalanceAngle(float balanceAngle)
{
    const float defaultBalanceAngle = DEFAULTS::balanceAngle;
    return storeItem(BalanceAngleKey, &balanceAngle, sizeof(balanceAngle), &defaultBalanceAngle);
}

Cockpit::failsafe_t NonVolatileStorage::loadFailsafe() // NOLINT(readability-make-member-function-const)
{
    Cockpit::failsafe_t failsafe {}; // NOLINT(misc-const-correctness)
    if (loadItem(FailsafeKey, &failsafe, sizeof(failsafe))) { // cppcheck-suppress knownConditionTrueFalse
    }
    return DEFAULTS::failsafe;
}

int32_t NonVolatileStorage::storeFailsafe(const Cockpit::failsafe_t& failsafe)
{
    return storeItem(FailsafeKey, &failsafe, sizeof(failsafe), &DEFAULTS::failsafe);
}

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::loadPID(uint8_t pidIndex) const
{
    assert(pidIndex <= MotorPairController::PID_COUNT);
    {VehicleControllerBase::PIDF_uint16_t pid {};
    if (loadItem(PID_Keys[pidIndex], &pid, sizeof(pid))) { // cppcheck-suppress knownConditionTrueFalse
        return pid;
    }}
    return DEFAULTS::motorPairControllerPIDs[pidIndex];
}

int32_t NonVolatileStorage::storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex)
{
    assert(pidIndex <= MotorPairController::PID_COUNT);
    return storeItem(PID_Keys[pidIndex], &pid, sizeof(pid), &DEFAULTS::motorPairControllerPIDs[pidIndex]);
}

void NonVolatileStorage::resetPID(uint8_t pidIndex)
{
    assert(pidIndex <= MotorPairController::PID_COUNT);
    remove(PID_Keys[pidIndex]);
}

void NonVolatileStorage::loadMacAddress(uint8_t* macAddress) const // NOLINT(readability-non-const-parameter)
{
#if defined(USE_FLASH_KLV)
    (void)macAddress;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_ONLY)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], MacAddressKey);
        _preferences.getBytes(&keyS[0], macAddress, MAC_ADDRESS_LEN);
        _preferences.end();
    }
#else
    (void)macAddress;
#endif
}

int32_t NonVolatileStorage::storeMacAddress(const uint8_t* macAddress)
{
#if defined(USE_FLASH_KLV)
    (void)macAddress;
    return OK;
#elif defined(USE_ARDUINO_ESP32_PREFERENCES)
    if (_preferences.begin(nonVolatileStorageNamespace, READ_WRITE)) {
        std::array<char, 8> keyS;
        toHexChars(&keyS[0], MacAddressKey);
        _preferences.putBytes(&keyS[0], macAddress, MAC_ADDRESS_LEN);
        _preferences.end();
        return OK;
    }
    return ERROR_NOT_WRITTEN;
#else
    (void)macAddress;
    return OK;
#endif
}
