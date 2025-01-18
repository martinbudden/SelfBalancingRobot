#pragma once

struct xyz_t {
public:
    inline float magnitude_squared() const { return x*x + y*y + z*z; } //<! The square of the magnitude
    // Unary operations
    inline xyz_t operator+() const { return *this; } //<! Unary plus
    inline xyz_t operator-() const { return xyz_t {-x, -y, -z }; } //<! Unary negation

    inline xyz_t operator+=(const xyz_t& v) { x += v.x; y += v.y; z += v.z; return *this; }
    inline xyz_t operator-=(const xyz_t& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    inline xyz_t operator*=(float k) { x*=k; y*=k; z*=k; return *this; } //<! Multiplication by a scalar

    // Binary operations
    inline bool operator==(const xyz_t& v) const { return x == v.x &&  y == v.y && z == v.z; }
    inline bool operator!=(const xyz_t& v) const { return !(*this == v); }
    inline xyz_t operator+(const xyz_t& v) const { return xyz_t{x + v.x, y + v.y, z + v.z}; }
    inline xyz_t operator-(const xyz_t& v) const { return xyz_t{x - v.x, y - v.y, z - v.z}; }
    inline xyz_t operator*(float k) const { return xyz_t{x*k, y*k, z*k}; } //<! Multiplication by a scalar
    inline float dot_product(const xyz_t& v) const { return  x*v.x + y*v.y + z*v.z; } //!< Vector dot product
    inline xyz_t cross_product(const xyz_t& v) const { return xyz_t{y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x}; } //!< Vector cross product
public:
    float x;
    float y;
    float z;
};
