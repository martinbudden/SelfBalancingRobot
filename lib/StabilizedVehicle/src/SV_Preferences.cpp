#if defined(USE_ESP32_PREFERENCES)

#include "SV_Preferences.h"
#include <cfloat>

namespace { // use anonymous namespace to make items local to this translation unit
const char* preferencesNamespace {"SV"};
} // end namespace


void SV_Preferences::clear()
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.clear();

    _preferences.end();
}

bool SV_Preferences::isSetPID() const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("PIDS_SET");

    _preferences.end();
    return ret;
}

PIDF::PIDF_t SV_Preferences::getPID(const std::string& name) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const PIDF::PIDF_t pid {
        .kp = _preferences.getFloat((name + "_P").c_str(), 0.0F),
        .ki = _preferences.getFloat((name + "_I").c_str(), 0.0F),
        .kd = _preferences.getFloat((name + "_D").c_str(), 0.0F),
        .kf = _preferences.getFloat((name + "_F").c_str(), 0.0F)
    };

    _preferences.end();
    return pid;
}

void SV_Preferences::putPID(const std::string& name, const PIDF::PIDF_t& pid)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("PIDS_SET", true);
    _preferences.putFloat((name + "_P").c_str(), pid.kp);
    _preferences.putFloat((name + "_I").c_str(), pid.ki);
    _preferences.putFloat((name + "_D").c_str(), pid.kd);
    _preferences.putFloat((name + "_F").c_str(), pid.kf);

    _preferences.end();
}

float SV_Preferences::getFloat(const std::string& name) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const float pitchBalanceAngleDegrees = _preferences.getFloat(name.c_str(), FLT_MAX);

    _preferences.end();
    return pitchBalanceAngleDegrees;
}

void SV_Preferences::putFloat(const std::string& name, float value)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putFloat(name.c_str(), value);

    _preferences.end();
}

bool SV_Preferences::getAccOffset(xyz_int16_t* accOffset) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("acc", false);
    if (ret) {
        accOffset->x = _preferences.getShort("acc_x", 0);
        accOffset->y = _preferences.getShort("acc_y", 0);
        accOffset->z = _preferences.getShort("acc_z", 0);
    }

    _preferences.end();
    return ret;
}

void SV_Preferences::putAccOffset(const xyz_int16_t& accOffset)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("acc", true);
    _preferences.putShort("acc_x", accOffset.x);
    _preferences.putShort("acc_y", accOffset.y);
    _preferences.putShort("acc_z", accOffset.z);

    _preferences.end();
}

bool SV_Preferences::getGyroOffset(xyz_int16_t* gyroOffset) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("gyro", false);
    if (ret) {
        gyroOffset->x = _preferences.getShort("gyro_x", 0);
        gyroOffset->y = _preferences.getShort("gyro_y", 0);
        gyroOffset->z = _preferences.getShort("gyro_z", 0);
    }

    _preferences.end();
    return ret;
}

void SV_Preferences::putGyroOffset(const xyz_int16_t& gyroOffset)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("gyro", true);
    _preferences.putShort("gyro_x", gyroOffset.x);
    _preferences.putShort("gyro_y", gyroOffset.y);
    _preferences.putShort("gyro_z", gyroOffset.z);

    _preferences.end();
}

void SV_Preferences::getMacAddress(uint8_t* macAddress, const std::string& name) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    _preferences.getBytes(name.c_str(), macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
}

void SV_Preferences::putMacAddress(const std::string& name, const uint8_t* macAddress)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBytes(name.c_str(), macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
}

#endif // USE_ESP32_PREFERENCES
