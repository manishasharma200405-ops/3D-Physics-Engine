#pragma once
// Core math types: Vec3, Mat3, Quat, AABB.  Header-only, float precision.
#include <algorithm>
#include <cmath>

namespace eng {

constexpr float kPi = 3.14159265358979f;

struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float& operator[](int i) {
        switch (i) { case 0: return x; case 1: return y; default: return z; }
    }
    float operator[](int i) const {
        switch (i) { case 0: return x; case 1: return y; default: return z; }
    }

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { float inv = 1.f / s; return {x * inv, y * inv, z * inv}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(lengthSq()); }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Vec3 normalize(const Vec3& v) {
    float l = v.length();
    return l > 1e-12f ? v / l : Vec3(0.f, 0.f, 0.f);
}
inline Vec3 vmin(const Vec3& a, const Vec3& b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(const Vec3& a, const Vec3& b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }

// Row-major 3x3 matrix.
struct Mat3 {
    float m[3][3];

    Mat3() {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) m[i][j] = (i == j) ? 1.f : 0.f;
    }

    static Mat3 zero() {
        Mat3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = 0.f;
        return r;
    }
    static Mat3 diag(const Vec3& d) {
        Mat3 r = zero();
        r.m[0][0] = d.x; r.m[1][1] = d.y; r.m[2][2] = d.z;
        return r;
    }

    Vec3 col(int j) const { return {m[0][j], m[1][j], m[2][j]}; }

    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r = zero();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++) r.m[i][j] += m[i][k] * o.m[k][j];
        return r;
    }
    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) r.m[i][j] = m[j][i];
        return r;
    }
};

// Unit quaternion (w, x, y, z) for orientations.
struct Quat {
    float w = 1.f, x = 0.f, y = 0.f, z = 0.f;

    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat fromAxisAngle(const Vec3& axis, float angle) {
        Vec3 a = normalize(axis);
        float s = std::sin(angle * 0.5f);
        return {std::cos(angle * 0.5f), a.x * s, a.y * s, a.z * s};
    }

    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    Quat normalized() const {
        float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12f) return Quat();
        float inv = 1.f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    Mat3 toMat3() const {
        Mat3 r;
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;
        r.m[0][0] = 1.f - 2.f * (yy + zz); r.m[0][1] = 2.f * (xy - wz);       r.m[0][2] = 2.f * (xz + wy);
        r.m[1][0] = 2.f * (xy + wz);       r.m[1][1] = 1.f - 2.f * (xx + zz); r.m[1][2] = 2.f * (yz - wx);
        r.m[2][0] = 2.f * (xz - wy);       r.m[2][1] = 2.f * (yz + wx);       r.m[2][2] = 1.f - 2.f * (xx + yy);
        return r;
    }

    Vec3 rotate(const Vec3& v) const { return toMat3() * v; }
};

// Axis-aligned bounding box.
struct AABB {
    Vec3 min, max;

    AABB() = default;
    AABB(const Vec3& mn, const Vec3& mx) : min(mn), max(mx) {}

    static AABB merge(const AABB& a, const AABB& b) { return {vmin(a.min, b.min), vmax(a.max, b.max)}; }

    float surfaceArea() const {
        Vec3 d = max - min;
        return 2.f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
    bool overlaps(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x &&
               min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
    bool contains(const AABB& o) const {
        return min.x <= o.min.x && min.y <= o.min.y && min.z <= o.min.z &&
               max.x >= o.max.x && max.y >= o.max.y && max.z >= o.max.z;
    }
    AABB fattened(float m) const { return {min - Vec3(m, m, m), max + Vec3(m, m, m)}; }
};

}  // namespace eng
