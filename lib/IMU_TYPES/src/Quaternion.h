#pragma once

#include <cmath>

class Quaternion {
public:
    Quaternion() : w(1.0F), x(0.0F), y(0.0F), z(0.0F) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
public:
    static Quaternion fromEulerAnglesRadians(float rollRadians, float pitchRadians, float yawRadians);
    static Quaternion fromEulerAnglesRadians(float rollRadians, float pitchRadians);
public:
    static constexpr float radiansToDegrees { 180.0 / M_PI };
    static constexpr float degreesToRadians { M_PI / 180.0};
public:
    inline float getW() const { return w; }
    inline float getX() const { return x; }
    inline float getY() const { return y; }
    inline float getZ() const { return z; }
    inline void getWXYZ(float& w_, float& x_, float&y_, float& z_) const { w_ = w; x_ = x; y_ = y; z_ = z; } 
public:
    inline float magnitude_squared() const { return w*w + x*x + y*y +z*z; } //<! The square of the norm
    inline Quaternion conjugate() const { return Quaternion(w, -x, -y, -z); }

    // Unary operations
    inline Quaternion operator+() const { return *this; } //<!Unary plus
    inline Quaternion operator-() const { return Quaternion(-w, -x, -y, -z); } //<! Unary minus

    inline Quaternion operator+=(const Quaternion& q) {
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }
    inline Quaternion operator-=(const Quaternion& q) {
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
        return *this;
    }
    inline Quaternion operator*=(float k) { w*=k; x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a constant
    inline Quaternion operator*=(const Quaternion& q) {
        const float wt = w*q.w - x*q.x - y*q.y - z*q.z;
        const float xt = w*q.x + x*q.w + y*q.z - z*q.y;
        const float yt = w*q.y - x*q.z + y*q.w + z*q.x;
        z              = w*q.z + x*q.y - y*q.x + z*q.w;
        w = wt;
        x = xt;
        y = yt;
        return *this;
    }

    // Binary operations
    inline Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }
    inline Quaternion operator-(const Quaternion& q) const {
        return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }
    inline Quaternion operator*(float k) const {//<! Multiplication by a constant
        return Quaternion(w*k, x*k, y*k, z*k);
    }
    inline Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
public:
    static inline float asinClipped(float angleRadians) {
        if (angleRadians <= -1.0F) { return {-M_PI/2.0}; }
        if (angleRadians >=  1.0f) { return {M_PI/2.0}; }
        return asinf(angleRadians);
    }
    inline float calculateRollRadians() const  { return atan2f(w*x + y*z, 0.5F - x*x - y*y); }
    inline float calculatePitchRadians() const { return asinClipped(2.0F*(w*y - x*z)); }
    inline float calculateYawRadians() const   { return atan2f(w*z + x*y, 0.5F - y*y - z*z); } // alternatively atan2f(2*(w*z + x*y), w*w + x*x - y*y - z*z)

    inline float calculateRollDegrees() const { return radiansToDegrees * calculateRollRadians(); }
    inline float calculatePitchDegrees() const { return radiansToDegrees * calculatePitchRadians(); }
    inline float calculateYawDegrees() const { return radiansToDegrees * calculateYawRadians(); }
protected:
    float w;
    float x;
    float y;
    float z;
};
