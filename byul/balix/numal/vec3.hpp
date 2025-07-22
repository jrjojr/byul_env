#ifndef VEC3_HPP
#define VEC3_HPP

#include "internal/vec3.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

class Vec3 {
public:
    vec3_t v;

    // ---------------------------------------------------------
    // 생성자 & 변환자
    // ---------------------------------------------------------
    Vec3() : v{0.0f, 0.0f, 0.0f} {}
    Vec3(float x, float y, float z) : v{x, y, z} {}
    explicit Vec3(const vec3_t& src) : v{src.x, src.y, src.z} {}

    operator vec3_t() const { return v; }

    // ---------------------------------------------------------
    // 비교 연산자
    // ---------------------------------------------------------
    bool operator==(const Vec3& rhs) const noexcept {
        return vec3_equal(&v, &rhs.v);
    }
    bool operator!=(const Vec3& rhs) const noexcept {
        return !(*this == rhs);
    }

    // ---------------------------------------------------------
    // 산술 연산자 (새 벡터 반환)
    // ---------------------------------------------------------
    Vec3 operator+(const Vec3& rhs) const noexcept {
        vec3_t result;
        vec3_add(&result, &v, &rhs.v);
        return Vec3(result);
    }
    Vec3 operator-(const Vec3& rhs) const noexcept {
        vec3_t result;
        vec3_sub(&result, &v, &rhs.v);
        return Vec3(result);
    }
    Vec3 operator-() const noexcept {
        vec3_t r;
        vec3_negate(&r, &v);
        return Vec3(r);
    }
    Vec3 operator*(const Vec3& rhs) const noexcept { // 요소곱
        vec3_t result;
        vec3_mul(&result, &v, &rhs.v);
        return Vec3(result);
    }
    Vec3 operator/(const Vec3& rhs) const {
        vec3_t result;
        vec3_div(&result, &v, &rhs.v);
        return Vec3(result);
    }
    Vec3 operator*(float scalar) const noexcept {
        vec3_t result;
        vec3_scale(&result, &v, scalar);
        return Vec3(result);
    }
    Vec3 operator/(float scalar) const {
        if (scalar == 0.0f)
            throw std::runtime_error("Vec3: divide by zero");
        return *this * (1.0f / scalar);
    }

    // ---------------------------------------------------------
    // In-place 연산자 (자기 자신 갱신)
    // ---------------------------------------------------------
    Vec3& operator+=(const Vec3& rhs) noexcept {
        v.x += rhs.v.x; v.y += rhs.v.y; v.z += rhs.v.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& rhs) noexcept {
        v.x -= rhs.v.x; v.y -= rhs.v.y; v.z -= rhs.v.z;
        return *this;
    }
    Vec3& operator*=(float scalar) noexcept {
        v.x *= scalar; v.y *= scalar; v.z *= scalar;
        return *this;
    }
    Vec3& operator/=(float scalar) {
        if (scalar == 0.0f)
            throw std::runtime_error("Vec3: divide by zero");
        float inv = 1.0f / scalar;
        v.x *= inv; v.y *= inv; v.z *= inv;
        return *this;
    }

    // ---------------------------------------------------------
    // 벡터 연산
    // ---------------------------------------------------------
    float dot(const Vec3& rhs) const noexcept {
        return vec3_dot(&v, &rhs.v);
    }
    Vec3 cross(const Vec3& rhs) const noexcept {
        vec3_t result;
        vec3_cross(&result, &v, &rhs.v);
        return Vec3(result);
    }
    float length() const noexcept {
        return vec3_length(&v);
    }
    float length_sq() const noexcept {
        return vec3_length_sq(&v);
    }
    float distance_to(const Vec3& other) const noexcept {
        return vec3_distance(&v, &other.v);
    }

    // ---------------------------------------------------------
    // 정규화
    // ---------------------------------------------------------
    Vec3 normalized() const {
        vec3_t r = v;
        vec3_normalize(&r);
        return Vec3(r);
    }
    void normalize() noexcept {
        vec3_normalize(&v);
    }

    // ---------------------------------------------------------
    // 정적 유틸 함수
    // ---------------------------------------------------------
    static Vec3 Zero() noexcept { return Vec3(0, 0, 0); }

    static Vec3 Lerp(const Vec3& a, const Vec3& b, float t) {
        vec3_t result;
        vec3_lerp(&result, &a.v, &b.v, t);
        return Vec3(result);
    }

    static Vec3 Unit(const Vec3& src) {
        vec3_t r;
        vec3_unit(&r, &src.v);
        return Vec3(r);
    }

    void to_mat4(float* out_mat4) const noexcept {
        vec3_to_mat4(&v, out_mat4);
    }

    // ---------------------------------------------------------
    // 스칼라 왼쪽 연산
    // ---------------------------------------------------------
    friend Vec3 operator*(float s, const Vec3& vec) noexcept {
        return vec * s;
    }
    friend Vec3 operator/(float s, const Vec3& vec) {
        vec3_t r;
        r.x = s / vec.v.x;
        r.y = s / vec.v.y;
        r.z = s / vec.v.z;
        return Vec3(r);
    }

    // ---------------------------------------------------------
    // 출력 스트림
    // ---------------------------------------------------------
    friend std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
        os << "(" << vec.v.x << ", " << vec.v.y << ", " << vec.v.z << ")";
        return os;
    }
};

#endif // VEC3_HPP
