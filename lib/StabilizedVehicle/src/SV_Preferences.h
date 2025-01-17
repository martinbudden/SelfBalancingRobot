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

    float getFloat(const std::string& name) const;
    void putFloat(const std::string& name, float value);

    bool getAccOffset(xyz_int16_t* accOffset) const;
    void putAccOffset(const xyz_int16_t& accOffset);

    bool getGyroOffset(xyz_int16_t* gyroOffset) const;
    void putGyroOffset(const xyz_int16_t& gyroOffset);

    void getMacAddress(uint8_t* macAddress, const std::string& name) const;
    void putMacAddress(const std::string& name, const uint8_t* macAddress);
private:
    mutable Preferences _preferences;
};
