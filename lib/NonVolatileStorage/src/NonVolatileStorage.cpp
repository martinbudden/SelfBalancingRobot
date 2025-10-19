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

static constexpr uint16_t PID_ProfileIndexKey = 0x0001;
//static constexpr uint16_t RateProfileIndexKey = 0x0002;
static constexpr uint16_t BalanceAngleKey = 0x0003;
static const std::array<uint16_t, MotorPairController::PID_COUNT> PID_Keys = {
    // note these must go up in jumps of 4, since one key is used for each profile
    0x0100, 0x0104, 0x0108, 0x010C, 0x0110, 0x0114
};
static constexpr uint16_t AccOffsetKey = 0x0200;
static constexpr uint16_t GyroOffsetKey = 0x0201;
static constexpr uint16_t MacAddressKey = 0x0202;

static constexpr uint16_t MotorPairPitchBalanceAngleConfigKey = 0x0300;
// Part of PID profile
static constexpr uint16_t MotorPairControllerFiltersConfigKey = 0x0400;

static constexpr uint16_t RadioControllerFailsafeKey = 0x0506;


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


uint8_t NonVolatileStorage::loadPidProfileIndex() const
{
    uint8_t profileIndex {}; // NOLINT(misc-const-correctness)
    if (loadItem(PID_ProfileIndexKey, &profileIndex, sizeof(profileIndex))) { // cppcheck-suppress knownConditionTrueFalse
        return profileIndex;
    }
    return DEFAULT_PID_PROFILE;
}

int32_t NonVolatileStorage::storePidProfileIndex(uint8_t profileIndex)
{
    if (profileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint8_t defaultProfileIndex = DEFAULT_PID_PROFILE;
    return storeItem(PID_ProfileIndexKey, &profileIndex, sizeof(profileIndex), &defaultProfileIndex);
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

#if !defined(FRAMEWORK_TEST)
RadioController::failsafe_t NonVolatileStorage::loadRadioControllerFailsafe() // NOLINT(readability-make-member-function-const)
{
    RadioController::failsafe_t failsafe {}; // NOLINT(misc-const-correctness)
    if (loadItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe))) { // cppcheck-suppress knownConditionTrueFalse
    }
    return DEFAULTS::radioControllerFailsafe;
}

int32_t NonVolatileStorage::storeRadioControllerFailsafe(const RadioController::failsafe_t& failsafe)
{
    return storeItem(RadioControllerFailsafeKey, &failsafe, sizeof(failsafe), &DEFAULTS::radioControllerFailsafe);
}

VehicleControllerBase::PIDF_uint16_t NonVolatileStorage::loadPID(uint8_t pidIndex, uint8_t pidProfileIndex) const
{
    assert(pidIndex <= MotorPairController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return DEFAULTS::motorPairControllerPIDs[pidIndex];
    }
    const uint16_t key = PID_Keys[pidIndex] + pidProfileIndex;
    VehicleControllerBase::PIDF_uint16_t pid {};
    if (loadItem(key, &pid, sizeof(pid))) { // cppcheck-suppress knownConditionTrueFalse
        return pid;
    }
    return DEFAULTS::motorPairControllerPIDs[pidIndex];
}

int32_t NonVolatileStorage::storePID(const VehicleControllerBase::PIDF_uint16_t& pid, uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= MotorPairController::PID_COUNT);
    if (pidProfileIndex >= PID_PROFILE_COUNT) {
        return ERROR_INVALID_PROFILE;
    }
    const uint16_t key = PID_Keys[pidIndex] + pidProfileIndex;
    return storeItem(key, &key, sizeof(pid), &DEFAULTS::motorPairControllerPIDs[pidIndex]);
}
#endif

void NonVolatileStorage::resetPID(uint8_t pidIndex, uint8_t pidProfileIndex)
{
    assert(pidIndex <= MotorPairController::PID_COUNT);
    assert(pidProfileIndex < PID_PROFILE_COUNT);
    remove(PID_Keys[pidIndex]);
}


bool NonVolatileStorage::loadAccOffset(int32_t& x, int32_t& y, int32_t& z) const
{
    xyz_int32_t xyz {}; // NOLINT(misc-const-correctness)
    if (loadItem(AccOffsetKey, &xyz, sizeof(xyz))) { // cppcheck-suppress knownConditionTrueFalse
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
        return true;
    }
    return false;
}

int32_t NonVolatileStorage::storeAccOffset(int32_t x, int32_t y, int32_t z)
{
    const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
    const xyz_int32_t xyzDefault = { .x = 0, .y = 0, .z = 0 };
    return storeItem(AccOffsetKey, &xyz, sizeof(xyz), &xyzDefault);
}

bool NonVolatileStorage::loadGyroOffset(int32_t& x, int32_t& y, int32_t& z) const
{
    xyz_int32_t xyz {}; // NOLINT(misc-const-correctness)
    if (loadItem(GyroOffsetKey, &xyz, sizeof(xyz))) { // cppcheck-suppress knownConditionTrueFalse
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
        return true;
    }
    return false;
}

int32_t NonVolatileStorage::storeGyroOffset(int32_t x, int32_t y, int32_t z)
{
    const xyz_int32_t xyz = { .x = x, .y = y, .z = z };
    const xyz_int32_t xyzDefault = { .x = 0, .y = 0, .z = 0 };
    return storeItem(GyroOffsetKey, &xyz, sizeof(xyz), &xyzDefault);
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
