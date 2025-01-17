#pragma once

#include <PIDF.h>
#include <Preferences.h>
#include <xyz_int16_type.h>

#include <string>

class SV_Preferences {
public:
    enum { READ_WRITE=false, READ_ONLY=true };
    enum { MAC_ADDRESS_LEN = 6 };
public:
    // NOTE: "get" functions are declared const, since they are logically const, although not physically const

    void clear();
    bool isSetPID() const;

    PIDF::PIDF_t getPID(const std::string& name) const;
    void putPID(const std::string& name, const PIDF::PIDF_t& pid);

    float getPitchBalanceAngleDegrees() const;
    void putPitchBalanceAngleDegrees(float pitchBalanceAngleDegrees);

    bool getAccOffset(xyz_int16_t* accOffset) const;
    void putAccOffset(const xyz_int16_t& accOffset);

    bool getGyroOffset(xyz_int16_t* gyroOffset) const;
    void putGyroOffset(const xyz_int16_t& gyroOffset);

    void getTransmitMacAddress(uint8_t* macAddress) const;
    void putTransmitMacAddress(const uint8_t* macAddress);
private:
    mutable Preferences _preferences;
};
