#ifndef VEC3_HPP
#define VEC3_HPP

#include "internal/vec3.h"
#include <cmath>
#include <cstdint>
#include <iostream>

class Vec3 {
public:
    vec3_t v;

    // 생성자
    Vec3() : v{0.0f, 0.0f, 0.0f} {}
    Vec3(float x, float y, float z) : v{x, y, z} {}
    Vec3(const vec3_t& src) : v{src.x, src.y, src.z} {}

    // 변환자
    operator vec3_t() const { return v; }

    // 비교
    bool operator==(const Vec3& other) const {
        return vec3_equal(&v, &other.v);
    }

    bool operator!=(const Vec3& other) const {
        return !(*this == other);
    }

    // 덧셈
    Vec3 operator+(const Vec3& rhs) const {
        vec3_t result;
        vec3_add(&result, &v, &rhs.v);
        return Vec3(result);
    }

    // 뺄셈
    Vec3 operator-(const Vec3& rhs) const {
        vec3_t result;
        vec3_sub(&result, &v, &rhs.v);
        return Vec3(result);
    }

    Vec3 operator-() const { 
        vec3_t r; 
        vec3_negate(&r, &v); 
        return Vec3(r); 
    }

        // 요소별 곱
    Vec3 operator*(const Vec3& rhs) const {
        vec3_t result;
        vec3_mul(&result, &v, &rhs.v);  // 요소곱: result = v ⊙ rhs.v
        return Vec3(result);
    }

    // 요소별 나눗셈
    Vec3 operator/(const Vec3& rhs) const {
        vec3_t result;
        vec3_div(&result, &v, &rhs.v);  // 요소나눗셈: result = v ⊘ rhs.v
        return Vec3(result);
    }

    // 스칼라 곱
    Vec3 operator*(float scalar) const {
        vec3_t result;
        vec3_scale(&result, &v, scalar);
        return Vec3(result);
    }

    // 스칼라 나눗셈
    Vec3 operator/(float scalar) const {
        if (scalar == 0.0f) throw std::runtime_error("Divide by zero");
        return *this * (1.0f / scalar);
    }

    // 내적
    float dot(const Vec3& rhs) const {
        return vec3_dot(&v, &rhs.v);
    }

    // 외적
    Vec3 cross(const Vec3& rhs) const {
        vec3_t result;
        vec3_cross(&result, &v, &rhs.v);
        return Vec3(result);
    }

    // 길이
    float length() const {
        return vec3_length(&v);
    }

    // 정규화
    void normalize() {
        vec3_normalize(&v);
    }

    Vec3 unit(const Vec3& src) { 
        vec3_t r;
        vec3_unit(&r, &src); 
        return Vec3(r); 
    }

    // 거리
    float distance_to(const Vec3& other) const {
        return vec3_distance(&v, &other.v);
    }

    // 보간
    Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        vec3_t result;
        vec3_lerp(&result, &a.v, &b.v, t);
        return Vec3(result);
    }

    // 변환 행렬
    void to_mat4(float* out_mat4) const {
        vec3_to_mat4(&v, out_mat4);
    }

    // scalar left operations
    friend Vec3 operator*(float s, const Vec3& vec) {
        return vec * s;
    }

    friend Vec3 operator/(float s, const Vec3& vec) {
        vec3_t r;
        r.x = s / vec.v.x;
        r.y = s / vec.v.y;
        r.z = s / vec.v.z;
        return Vec3(r);
    }

    // 출력용
    friend std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
        os << "(" << vec.v.x << ", " << vec.v.y << ", " << vec.v.z << ")";
        return os;
    }
};

#endif // VEC3_HPP
