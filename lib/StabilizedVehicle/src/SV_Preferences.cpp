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
        .kp = _preferences.getFloat((name + "_P").c_str(), FLT_MAX),
        .ki = _preferences.getFloat((name + "_I").c_str(), FLT_MAX),
        .kd = _preferences.getFloat((name + "_D").c_str(), FLT_MAX),
        .kf = _preferences.getFloat((name + "_F").c_str(), FLT_MAX)
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

void SV_Preferences::removeAccOffset()
{
    _preferences.begin(preferencesNamespace, READ_WRITE);
    _preferences.remove("acc");
    _preferences.remove("acc_x");
    _preferences.remove("acc_y");
    _preferences.remove("acc_z");
}

bool SV_Preferences::getAccOffset(int32_t& x, int32_t& y, int32_t& z) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("acc", false);
    if (ret) {
        x = _preferences.getInt("acc_x", 0);
        y = _preferences.getInt("acc_y", 0);
        z = _preferences.getInt("acc_z", 0);
    }

    _preferences.end();
    return ret;
}

void SV_Preferences::putAccOffset(int32_t x, int32_t y, int32_t z)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("acc", true);
    _preferences.putInt("acc_x", x);
    _preferences.putInt("acc_y", y);
    _preferences.putInt("acc_z", z);

    _preferences.end();
}

void SV_Preferences::removeGyroOffset()
{
    _preferences.begin(preferencesNamespace, READ_WRITE);
    _preferences.remove("gyro");
    _preferences.remove("gyro_x");
    _preferences.remove("gyro_y");
    _preferences.remove("gyro_z");
}

bool SV_Preferences::getGyroOffset(int32_t& x, int32_t& y, int32_t& z) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const bool ret = _preferences.getBool("gyro", false);
    if (ret) {
        x = _preferences.getInt("gyro_x", 0);
        y = _preferences.getInt("gyro_y", 0);
        z = _preferences.getInt("gyro_z", 0);
    }

    _preferences.end();
    return ret;
}

void SV_Preferences::putGyroOffset(int32_t x, int32_t y, int32_t z)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("gyro", true);
    _preferences.putInt("gyro_x", x);
    _preferences.putInt("gyro_y", y);
    _preferences.putInt("gyro_z", z);

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
