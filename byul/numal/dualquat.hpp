#ifndef DUALQUAT_HPP
#define DUALQUAT_HPP

#include "internal/dualquat.h"
#include "internal/quat.hpp"
#include "internal/vec3.hpp"
#include <iostream>
#include <cmath>

class DualQuat {
public:
    dualquat_t dq;

    // 생성자
    DualQuat() {
        dualquat_identity(&dq);
    }

    DualQuat(const Quat& rot, const Vec3& trans) {
        dualquat_from_quat_vec(&dq, &rot.q, &trans.v);
    }

    DualQuat(const dualquat_t& src) {
        dualquat_copy(&dq, &src);
    }

    DualQuat(const DualQuat& other) {
        dualquat_copy(&dq, &other.dq);
    }

    // 변환자
    operator dualquat_t() const { return dq; }

    // 정적 생성
    static DualQuat from(const Quat& rot, const Vec3& trans) {
        DualQuat result;
        dualquat_from_quat_vec(&result.dq, &rot.q, &trans.v);
        return result;
    }

    static DualQuat from_mat4(const float* mat4x4) {
        DualQuat result;
        dualquat_from_mat4(&result.dq, mat4x4);
        return result;
    }

    // 이동, 회전 추출
    void to_components(Quat& out_rot, Vec3& out_trans) const {
        quat_t q;
        vec3_t v;
        dualquat_to_quat_vec(&dq, &q, &v);
        out_rot = Quat(q);
        out_trans = Vec3(v);
    }

    // 연산자 오버로딩
    DualQuat operator+(const DualQuat& rhs) const {
        dualquat_t out;
        dualquat_add(&out, &dq, &rhs.dq);
        return DualQuat(out);
    }

    DualQuat operator-(const DualQuat& rhs) const {
        dualquat_t out;
        dualquat_sub(&out, &dq, &rhs.dq);
        return DualQuat(out);
    }

    DualQuat operator*(float scalar) const {
        dualquat_t out;
        dualquat_scale(&out, &dq, scalar);
        return DualQuat(out);
    }

    DualQuat operator*(const DualQuat& rhs) const {
        dualquat_t out;
        dualquat_mul(&out, &dq, &rhs.dq);
        return DualQuat(out);
    }

    bool operator==(const DualQuat& rhs) const {
        return dualquat_equal(&dq, &rhs.dq);
    }

    bool operator!=(const DualQuat& rhs) const {
        return !(*this == rhs);
    }

    // 정규화
    DualQuat normalized() const {
        dualquat_t out;
        dualquat_normalize(&out, &dq);
        return DualQuat(out);
    }

    // 역
    DualQuat inverse() const {
        dualquat_t out;
        dualquat_inverse(&out, &dq);
        return DualQuat(out);
    }

    // 켤레
    DualQuat conjugate() const {
        dualquat_t out;
        dualquat_conjugate(&out, &dq);
        return DualQuat(out);
    }

    // 변환 적용
    Vec3 transform_point(const Vec3& p) const {
        vec3_t io = p.v;
        dualquat_apply_to_point_inplace(&dq, &io);
        return Vec3(io);
    }

    // 보간
    static DualQuat slerp(const DualQuat& a, const DualQuat& b, float t) {
        dualquat_t out;
        dualquat_slerp(&out, &a.dq, &b.dq, t);
        return DualQuat(out);
    }

    static DualQuat lerp(const DualQuat& a, const DualQuat& b, float t) {
        dualquat_t out;
        dualquat_lerp(&out, &a.dq, &b.dq, t);
        return DualQuat(out);
    }

    static DualQuat blend_weighted(const DualQuat& a, 
        float w1, const DualQuat& b, float w2) {

        dualquat_t out;
        dualquat_blend_weighted(&out, &a.dq, w1, &b.dq, w2);
        return DualQuat(out);
    }

    // 정렬
    DualQuat aligned() const {
        dualquat_t out;
        dualquat_align(&out, &dq);
        return DualQuat(out);
    }

    // 행렬 변환
    void to_mat4(float* out_mat4) const {
        dualquat_to_mat4(&dq, out_mat4);
    }

    void to_mat3(float* out_mat3) const {
        dualquat_to_mat3(&dq, out_mat3);
    }

    // 길이 및 내적
    float dot(const DualQuat& rhs) const {
        return dualquat_dot(&dq, &rhs.dq);
    }

    float length() const {
        return dualquat_length(&dq);
    }

    // 출력
    friend std::ostream& operator<<(std::ostream& os, const DualQuat& dq) {
        os << "[real: (" << dq.dq.real.w << ", " << dq.dq.real.x << ", " 
           << dq.dq.real.y << ", " << dq.dq.real.z << "), "
           << "dual: (" << dq.dq.dual.w << ", " << dq.dq.dual.x << ", " 
           << dq.dq.dual.y << ", " << dq.dq.dual.z << ")]";
        return os;
    }
};

#endif // DUALQUAT_HPP
