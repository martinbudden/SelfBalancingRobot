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

    const bool ret = _preferences.getBool("PIDSET");

    _preferences.end();
    return ret;
}

void SV_Preferences::setPID_PreferencesToZero()
{
    _preferences.begin(preferencesNamespace, READ_ONLY);
    _preferences.putBool("PIDSET", true);
    _preferences.end();

    static constexpr PIDF::PIDF_t pidZero { 0.0, 0.0, 0.0, 0.0 };
    putPitchPID(pidZero);
    putSpeedPID(pidZero);
    putYawRatePID(pidZero);
    putPitchBalanceAngleDegrees(FLT_MAX);

}

PIDF::PIDF_t SV_Preferences::getRollPID() const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const PIDF::PIDF_t pid {
        .kp = _preferences.getFloat("ROLL_P", 0.0F),
        .ki = _preferences.getFloat("ROLL_I", 0.0F),
        .kd = _preferences.getFloat("ROLL_D", 0.0F),
        .kf = _preferences.getFloat("ROLL_F", 0.0F)
    };

    _preferences.end();
    return pid;
}

void SV_Preferences::putRollPID(const PIDF::PIDF_t& pid)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("PIDSET", true);
    _preferences.putFloat("ROLL_P", pid.kp);
    _preferences.putFloat("ROLL_I", pid.ki);
    _preferences.putFloat("ROLL_D", pid.kd);
    _preferences.putFloat("ROLL_F", pid.kf);

    _preferences.end();
}

PIDF::PIDF_t SV_Preferences::getPitchPID() const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const PIDF::PIDF_t pid {
        .kp = _preferences.getFloat("PITCH_P", 0.0F),
        .ki = _preferences.getFloat("PITCH_I", 0.0F),
        .kd = _preferences.getFloat("PITCH_D", 0.0F),
        .kf = _preferences.getFloat("PITCH_F", 0.0F)
    };

    _preferences.end();
    return pid;
}

void SV_Preferences::putPitchPID(const PIDF::PIDF_t& pid)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("PIDSET", true);
    _preferences.putFloat("PITCH_P", pid.kp);
    _preferences.putFloat("PITCH_I", pid.ki);
    _preferences.putFloat("PITCH_D", pid.kd);
    _preferences.putFloat("PITCH_F", pid.kf);

    _preferences.end();
}

PIDF::PIDF_t SV_Preferences::getYawRatePID() const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const PIDF::PIDF_t pid {
        .kp = _preferences.getFloat("YAW_RATE_P", 0.0F),
        .ki = _preferences.getFloat("YAW_RATE_I", 0.0F),
        .kd = _preferences.getFloat("YAW_RATE_D", 0.0F),
        .kf = _preferences.getFloat("YAW_RATE_F", 0.0F)
    };

    _preferences.end();
    return pid;
}

void SV_Preferences::putYawRatePID(const PIDF::PIDF_t& pid)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("PIDSET", true);
    _preferences.putFloat("YAW_RATE_P", pid.kp);
    _preferences.putFloat("YAW_RATE_I", pid.ki);
    _preferences.putFloat("YAW_RATE_D", pid.kd);
    _preferences.putFloat("YAW_RATE_F", pid.kf);

    _preferences.end();
}

PIDF::PIDF_t SV_Preferences::getSpeedPID() const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const PIDF::PIDF_t pid {
        .kp = _preferences.getFloat("SPEED_P", 0.0F),
        .ki = _preferences.getFloat("SPEED_I", 0.0F),
        .kd = _preferences.getFloat("SPEED_D", 0.0F),
        .kf = _preferences.getFloat("SPEED_F", 0.0F)
    };

    _preferences.end();
    return pid;
}

void SV_Preferences::putSpeedPID(const PIDF::PIDF_t& pid)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("PIDSET", true);
    _preferences.putFloat("SPEED_P", pid.kp);
    _preferences.putFloat("SPEED_I", pid.ki);
    _preferences.putFloat("SPEED_D", pid.kd);
    _preferences.putFloat("SPEED_F", pid.kf);

    _preferences.end();
}

float SV_Preferences::getPitchBalanceAngleDegrees() const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    const float pitchBalanceAngleDegrees = _preferences.getFloat("balAngle", FLT_MAX);

    _preferences.end();
    return pitchBalanceAngleDegrees;
}

void SV_Preferences::putPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBool("PIDSET", true);
    _preferences.putFloat("balAngle", pitchBalanceAngleDegrees);

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

void SV_Preferences::getTransmitMacAddress(uint8_t* macAddress) const
{
    _preferences.begin(preferencesNamespace, READ_ONLY);

    _preferences.getBytes("tMacAddr", macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
}

void SV_Preferences::putTransmitMacAddress(const uint8_t* macAddress)
{
    _preferences.begin(preferencesNamespace, READ_WRITE);

    _preferences.putBytes("tMacAddr", macAddress, MAC_ADDRESS_LEN);

    _preferences.end();
}

#endif // USE_ESP32_PREFERENCES
