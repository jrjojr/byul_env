#ifndef QUAT_HPP
#define QUAT_HPP

#include "quat.h"
#include "vec3.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>

class Quat {
public:
    quat_t q;

    Quat() { quat_init(&q); }
    Quat(float w, float x, float y, float z) { quat_init_full(&q, w, x, y, z); }
    Quat(const quat_t& src) { quat_assign(&q, &src); }
    Quat(const Quat& other) { quat_assign(&q, &other.q); }

    ~Quat() {}

    operator quat_t() const { return q; }

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

    Quat normalize() const {
        quat_t out;
        quat_unit(&out, &q);
        return Quat(out);
    }

    Quat conjugate() const {
        quat_t out;
        quat_conjugate(&out, &q);
        return Quat(out);
    }

    Quat inverse() const {
        quat_t out;
        quat_inverse(&out, &q);
        return Quat(out);
    }

    float length() const {
        return quat_length(&q);
    }

    float dot(const Quat& rhs) const {
        return quat_dot(&q, &rhs.q);
    }

    Vec3 rotate(const Vec3& v) const {
        vec3_t result;
        quat_rotate_vector(&q, &v.v, &result);
        return Vec3(result);
    }

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

    void toEuler(float& x, float& y, float& z, 
        euler_order_t order = EULER_ORDER_ZYX) const {

        quat_to_euler(&q, &x, &y, &z, order);
    }

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

    void toAxisAngle(Vec3& axis_out, float& radians_out) const {
        vec3_t axis;
        quat_to_axis_angle(&q, &axis, &radians_out);
        axis_out = Vec3(axis);
    }

    static Quat identity() {
        quat_t out;
        quat_identity(&out);
        return Quat(out);
    }

    friend std::ostream& operator<<(std::ostream& os, const Quat& quat) {
        os << "(" << quat.q.w << ", " << quat.q.x << ", " << quat.q.y << ", " << quat.q.z << ")";
        return os;
    }
};

#endif // QUAT_HPP
