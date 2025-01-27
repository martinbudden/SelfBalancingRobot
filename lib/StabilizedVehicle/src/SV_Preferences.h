#pragma once

#include <PIDF.h>
#include <Preferences.h>
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

    void removeAccOffset();
    bool getAccOffset(int32_t& x, int32_t& y, int32_t& z) const;
    void putAccOffset(int32_t x, int32_t y, int32_t z);

    void removeGyroOffset();
    bool getGyroOffset(int32_t& x, int32_t& y, int32_t& z) const;
    void putGyroOffset(int32_t x, int32_t y, int32_t z);

    void getMacAddress(uint8_t* macAddress, const std::string& name) const;
    void putMacAddress(const std::string& name, const uint8_t* macAddress);
private:
    mutable Preferences _preferences;
};
