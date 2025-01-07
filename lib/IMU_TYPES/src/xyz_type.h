#pragma once

struct xyz_t {
public:
    inline float magnitude_squared() const { return x*x + y*y + z*z; } //<! The square of the magnitude
    // Unary operations
    inline xyz_t operator+() const { return *this; } //<! Unary plus
    inline xyz_t operator-() const { return xyz_t {-x, -y, -z }; } //<! Unary minus

    inline xyz_t operator+=(const xyz_t& q) { x += q.x; y += q.y; z += q.z; return *this; }
    inline xyz_t operator-=(const xyz_t& q) { x -= q.x; y -= q.y; z -= q.z; return *this; }
    inline xyz_t operator*=(float k) { x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a constant

    // Binary operations
    inline xyz_t operator+(const xyz_t& q) const { return xyz_t{x + q.x, y + q.y, z + q.z}; }
    inline xyz_t operator-(const xyz_t& q) const { return xyz_t{x - q.x, y - q.y, z - q.z}; }
    inline xyz_t operator*(float k) const { return xyz_t{x*k, y*k, z*k}; } //<! Multiplication by a constant
    inline float dot_product(const xyz_t& q) const { return  x*q.x + y*q.y + z*q.z; } //!< Dot product
    inline xyz_t cross_product(const xyz_t& q) const { return xyz_t{y*q.z - z*q.y, z*q.x - x*q.z, x*q.y - y*q.x}; } //!< Cross product
public:
    float x;
    float y;
    float z;
};
