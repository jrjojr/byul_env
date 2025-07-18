#ifndef QUAT_HPP
#define QUAT_HPP

#include "internal/quat.h"
#include "internal/vec3.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

class Quat {
public:
    quat_t q;

    // 생성자
    Quat() { q = *quat_new(); }
    Quat(float w, float x, float y, float z) { q = *quat_new_full(w, x, y, z); }
    Quat(const quat_t& src) { q = *quat_copy(&src); }
    Quat(const Quat& other) { q = *quat_copy(&other.q); }

    // 소멸자
    ~Quat() {} // heap 메모리는 직접 관리하지 않음

    // 변환자
    operator quat_t() const { return q; }

    // 연산자 오버로딩
    Quat operator*(const Quat& rhs) const {
        quat_t out;
        quat_mul(&out, &q, &rhs.q);
        return Quat(out);
    }

    Quat operator+(const Quat& rhs) const {
        quat_t out;
        quat_add(&out, &q, &rhs.q);
        return Quat(out);
    }

    Quat operator-(const Quat& rhs) const {
        quat_t out;
        quat_sub(&out, &q, &rhs.q);
        return Quat(out);
    }

    Quat operator*(float scalar) const {
        quat_t out;
        quat_scale(&out, &q, scalar);
        return Quat(out);
    }

    // 정규화
    Quat normalized() const {
        quat_t out;
        quat_normalize(&out, &q);
        return Quat(out);
    }

    // 켤레
    Quat conjugate() const {
        quat_t out;
        quat_conjugate(&out, &q);
        return Quat(out);
    }

    // 역
    Quat inverse() const {
        quat_t out;
        quat_inverse(&out, &q);
        return Quat(out);
    }

    // 길이 및 내적
    float length() const {
        return quat_length(&q);
    }

    float dot(const Quat& rhs) const {
        return quat_dot(&q, &rhs.q);
    }

    // 회전 적용
    Vec3 rotate(const Vec3& v) const {
        vec3_t result;
        quat_rotate_vector(&result, &q, &v.v);
        return Vec3(result);
    }

    // 보간
    static Quat slerp(const Quat& a, const Quat& b, float t) {
        quat_t out;
        quat_slerp(&out, &a.q, &b.q, t);
        return Quat(out);
    }

    static Quat lerp(const Quat& a, const Quat& b, float t) {
        quat_t out;
        quat_lerp(&out, &a.q, &b.q, t);
        return Quat(out);
    }

    // 오일러 변환
    void toEuler(float& x, float& y, float& z, 
        euler_order_t order = EULER_ORDER_ZYX) const {

        quat_to_euler(&q, &x, &y, &z, order);
    }

    // 벡터 추출
    Vec3 getForward() const {
        vec3_t dir;
        quat_get_forward(&q, &dir);
        return Vec3(dir);
    }

    Vec3 getUp() const {
        vec3_t dir;
        quat_get_up(&q, &dir);
        return Vec3(dir);
    }

    Vec3 getRight() const {
        vec3_t dir;
        quat_get_right(&q, &dir);
        return Vec3(dir);
    }

    // 축+각 변환
    void toAxisAngle(Vec3& axis_out, float& radians_out) const {
        vec3_t axis;
        quat_to_axis_angle(&q, &axis, &radians_out);
        axis_out = Vec3(axis);
    }

    // 단위 회전 설정
    static Quat identity() {
        quat_t out;
        quat_identity(&out);
        return Quat(out);
    }

    // 출력 연산자
    friend std::ostream& operator<<(std::ostream& os, const Quat& quat) {
        os << "(" << quat.q.w << ", " << quat.q.x << ", " << quat.q.y << ", " << quat.q.z << ")";
        return os;
    }
};

#endif // QUAT_HPP
